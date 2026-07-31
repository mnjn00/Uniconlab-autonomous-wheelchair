# Wheelchair Standard TF and Measured URDF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실제 휠체어의 구동휠 회전축 중심과 Livox MID-360 내장 IMU 실측값을 기준으로, FAST-LIO의 `camera_init -> body` 출력을 ROS 표준 `map -> odom -> base_link` 트리로 안전하게 변환하고 3D SLAM·고정 지도 localization·Navigation에서 공통으로 사용하는 실기 전용 URDF와 bringup을 추가한다.

**Architecture:** 기존 FAST-LIO와 기존 시뮬레이션/하드웨어 URDF는 그대로 보존한다. 새 `wheelchair_tf_adapter` 패키지가 `/Odometry`와 `/cloud_registered_body`를 각각 `/odom`과 `/cloud_registered_base_link`로 변환하고 `odom -> base_link`의 단독 authority가 된다. `robot_state_publisher`는 새 실측 URDF에서 `base_link -> body -> livox_frame` 및 차체 링크를 발행한다. 고정 지도 모드에서는 기존 moving ICP가 `map -> odom`만 발행하고, FAST-LIO SLAM 모드에서는 전용 launch의 identity `map -> odom`만 발행한다.

**Tech Stack:** ROS1 Noetic, catkin, C++14, roscpp, tf2/tf2_ros/tf2_sensor_msgs, Eigen3, nav_msgs, sensor_msgs, diagnostic_msgs, xacro, robot_state_publisher, rostest, GoogleTest, pytest.

## Global Constraints

- 물리 좌표 기준은 REP-103으로 `+X` 전진, `+Y` 탑승자 기준 왼쪽, `+Z` 위쪽이다.
- `base_link`는 좌우 구동휠 회전축 정중앙이다. 지면 높이는 `0.322 m`이다.
- `base_link -> body`는 `[0.500, 0.200, 0.450] m`, RPY `[0, 0, 0] rad`이다.
- `body -> livox_frame`은 `[-0.01100, -0.02329, 0.04412] m`, RPY `[0, 0, 0] rad`이다.
- 실측값은 `physical_measurements_20260731.yaml` 한 곳이 authority이며, Xacro와 adapter launch가 이 값을 읽는다. 소스 코드에 별도 복사본을 두지 않는다.
- 기존 `wheelchair.urdf.xacro`, `wheelchair_hardware.urdf.xacro`, 기존 launch/config의 기본 동작은 바꾸지 않는다. 기존 파일 수정은 새 파일 설치와 하위 호환 파라미터 추가에 필요한 최소 범위로 제한한다.
- 새 실기 URDF에는 Gazebo 태그, 가상 센서, transmission, 제어 plugin을 넣지 않는다.
- 실기 bringup은 가짜 `joint_state_publisher`를 실행하지 않는다. 별도 display launch에서만 opt-in으로 실행한다.
- `odom -> base_link`는 6DoF를 유지하며 roll/pitch를 평면화하지 않는다.
- pose/twist covariance는 6×6 Jacobian으로 변환한다. 배열 이름만 바꾸거나 원본을 그대로 복사하지 않는다.
- `PointCloud2`는 XYZ만 좌표 변환하고 intensity, time/timestamp, ring 및 알려지지 않은 추가 필드와 byte layout을 보존한다.
- 프레임 불일치, 비정상 quaternion, NaN/Inf, 시간 역행, cloud/odom 시각 차이 초과, MID-360 extrinsic 불일치, 중복 TF authority에서는 fail closed 한다. 마지막 정상 TF를 새 timestamp로 재발행하지 않는다.
- 구현 및 자동 검증 중 모터 명령을 발행하거나 실제 휠체어를 움직이지 않는다. NUC 실기 검증은 정지 상태·구동 출력 비활성 상태까지만 포함한다.
- SSH 비밀번호나 토큰을 저장소에 기록하지 않는다. NUC 접속은 기존 SSH key/agent 또는 저장소 밖의 대화형 인증만 사용한다.

---

## File Structure

### 새 파일

```text
src/wheelchair_description/
├── config/physical_measurements_20260731.yaml
├── launch/hardware_description_measured_20260731.launch
├── launch/hardware_description_measured_20260731_display.launch
├── tests/test_measured_hardware_description.py
└── urdf/wheelchair_hardware_measured_20260731.urdf.xacro

src/wheelchair_tf_adapter/
├── CMakeLists.txt
├── package.xml
├── config/livox_builtin_measured_20260731.yaml
├── include/wheelchair_tf_adapter/transform_math.hpp
├── launch/adapter_only.launch
├── launch/fastlio_builtin_standard_frames.launch
├── src/fastlio_standard_frame_adapter.cpp
├── src/transform_math.cpp
└── test/
    ├── standard_frames.test
    ├── test_adapter_contract.py
    ├── test_standard_frames_ros.py
    └── test_transform_math.cpp

src/static_livox_localization/
├── config/moving_localization_standard_frames.yaml
├── launch/moving_localization_standard_frames.launch
└── test/test_standard_frame_profile.py

src/wheelchair_navigation/
├── config/global_costmap_standard_frames.yaml
├── config/local_costmap_standard_frames.yaml
├── launch/standard_livox_navigation.launch
└── tests/test_standard_livox_navigation.py

runtime/
├── preflight_standard_tf.py
├── start_wheelchair_standard_navigation.sh
└── start_wheelchair_standard_slam.sh

tests/
├── test_standard_tf_preflight.py
└── test_standard_tf_runtime_surface.py

docs/runbooks/
└── wheelchair-standard-tf-ko.md
```

### 최소 수정 파일

```text
src/wheelchair_description/CMakeLists.txt
src/wheelchair_description/package.xml
src/static_livox_localization/scripts/auto_initial_pose.py
src/static_livox_localization/scripts/initial_pose_candidates.py
src/static_livox_localization/test/test_auto_initial_pose_unit.py
src/static_livox_localization/test/test_auto_initial_pose_surface.py
```

---

## Task 1: 실측값 authority와 실기 전용 URDF 추가

**Files:**

- Create: `src/wheelchair_description/config/physical_measurements_20260731.yaml`
- Create: `src/wheelchair_description/urdf/wheelchair_hardware_measured_20260731.urdf.xacro`
- Create: `src/wheelchair_description/launch/hardware_description_measured_20260731.launch`
- Create: `src/wheelchair_description/launch/hardware_description_measured_20260731_display.launch`
- Create: `src/wheelchair_description/tests/test_measured_hardware_description.py`
- Modify: `src/wheelchair_description/CMakeLists.txt`
- Modify: `src/wheelchair_description/package.xml`

- [ ] **Step 1: 실측값과 URDF 계약을 먼저 실패하는 테스트로 작성**

  `test_measured_hardware_description.py`에서 YAML schema와 Xacro 확장 결과를 검사한다.

  ```python
  assert measurements["units"] == {"length": "m", "angle": "rad"}
  assert measurements["base_link"]["height_from_ground_m"] == 0.322
  assert joint_xyz("base_link_to_body_joint") == (0.500, 0.200, 0.450)
  assert joint_xyz("base_link_to_base_footprint_joint") == (0.0, 0.0, -0.322)
  assert joint_xyz("body_to_livox_frame_joint") == (-0.011, -0.02329, 0.04412)
  assert root_links == {"base_link"}
  assert all(child_parent_count[link] == 1 for link in links - {"base_link"})
  assert "<gazebo" not in expanded
  assert "<transmission" not in expanded
  ```

  또한 실기 launch에 `joint_state_publisher`가 없고 display launch에만 존재하는지, 모든 drive wheel/caster joint가 `continuous`인지 검사한다.

- [ ] **Step 2: 테스트를 실행해 새 프로필 부재로 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q src/wheelchair_description/tests/test_measured_hardware_description.py
  ```

  Expected: `physical_measurements_20260731.yaml` 또는 새 Xacro가 없다는 실패.

- [ ] **Step 3: 단일 실측값 YAML 작성**

  최소 필드는 다음과 같이 고정한다.

  ```yaml
  schema_version: 1
  profile_id: wheelchair_livox_builtin_20260731
  measurement_date: "2026-07-31"
  quality: approximate_manual
  units: {length: m, angle: rad}
  axes: {x: forward, y: rider_left, z: up}
  base_link:
    definition: midpoint_of_drive_wheel_rotation_axis
    height_from_ground_m: 0.322
  base_link_to_body:
    xyz: [0.500, 0.200, 0.450]
    rpy: [0.0, 0.0, 0.0]
  body_to_livox_frame:
    xyz: [-0.01100, -0.02329, 0.04412]
    rpy: [0.0, 0.0, 0.0]
    source: deployed_fast_lio_mid360_extrinsic
  ```

  YAML에는 `ground_to_body_m: 0.772`와 `horizontal_base_to_body_m: 0.5385164807`을 파생 검산값으로 기록하고 테스트에서 원본 좌표로 다시 계산해 일치 여부를 확인한다.

- [ ] **Step 4: 실기 전용 Xacro와 launch 구현**

  Xacro는 `xacro.load_yaml`로 실측 YAML을 읽고 `base_link`를 유일 root로 만든다.

  ```text
  base_link
  ├── base_footprint                  fixed, z=-0.322
  ├── body                            fixed, xyz=0.500 0.200 0.450
  │   └── livox_frame                 fixed, xyz=-0.011 -0.02329 0.04412
  ├── chassis_link                    fixed
  ├── seat_link                       fixed
  ├── backrest_link                   fixed
  ├── nuc_link                        fixed
  ├── left_drive_wheel_link           continuous
  ├── right_drive_wheel_link          continuous
  ├── left_caster_link                continuous
  └── right_caster_link               continuous
  ```

  구동휠 중심은 `x=0`, 반지름은 `0.322 m`로 둔다. 차체·좌석·NUC visual 치수는 기존 저장소 모델에서 재사용하되 `visual_only_legacy_geometry`로 주석 처리하여 localization 실측 authority와 구분한다.

  `hardware_description_measured_20260731.launch`는 `robot_state_publisher`만 시작한다. display launch는 해당 launch를 include하고 `joint_state_publisher`와 선택적 RViz만 시작한다.

- [ ] **Step 5: 설치 규칙과 runtime dependency 반영**

  `wheelchair_description/CMakeLists.txt`의 install directory에 `config`를 추가한다. `package.xml`은 `python3-yaml` runtime/test dependency와 테스트에서 사용하는 `xacro`, `urdf`를 선언하되 기존 시뮬레이션 dependency를 제거하지 않는다.

- [ ] **Step 6: URDF 테스트와 Noetic 파서 검증**

  Run:

  ```bash
  python3 -m pytest -q src/wheelchair_description/tests/test_measured_hardware_description.py
  rosrun xacro xacro src/wheelchair_description/urdf/wheelchair_hardware_measured_20260731.urdf.xacro -o /tmp/wheelchair_measured.urdf
  check_urdf /tmp/wheelchair_measured.urdf
  ```

  Expected: pytest 통과, `robot name is: wheelchair_hardware_measured_20260731`, `Successfully Parsed XML`.

- [ ] **Step 7: 커밋**

  ```bash
  git add src/wheelchair_description
  git commit -m "feat: add measured wheelchair hardware description"
  ```

---

## Task 2: 좌표·twist·covariance 변환 수학을 독립 라이브러리로 구현

**Files:**

- Create: `src/wheelchair_tf_adapter/CMakeLists.txt`
- Create: `src/wheelchair_tf_adapter/package.xml`
- Create: `src/wheelchair_tf_adapter/include/wheelchair_tf_adapter/transform_math.hpp`
- Create: `src/wheelchair_tf_adapter/src/transform_math.cpp`
- Create: `src/wheelchair_tf_adapter/test/test_transform_math.cpp`

- [ ] **Step 1: 변환 수학 API와 실패 테스트 작성**

  공개 API는 ROS 메시지와 분리한 Eigen 기반으로 제한한다.

  ```cpp
  Eigen::Isometry3d basePoseFromBodyPose(
      const Eigen::Isometry3d& world_T_body,
      const Eigen::Isometry3d& base_T_body);

  Eigen::Matrix<double, 6, 6> twistJacobianBodyToBase(
      const Eigen::Isometry3d& base_T_body);

  Eigen::Matrix<double, 6, 1> twistBodyToBase(
      const Eigen::Matrix<double, 6, 1>& body_twist,
      const Eigen::Isometry3d& base_T_body);

  Eigen::Matrix<double, 6, 6> poseJacobianBodyToBase(
      const Eigen::Isometry3d& world_T_body,
      const Eigen::Isometry3d& base_T_body);

  Eigen::Matrix<double, 6, 6> transformCovariance(
      const Eigen::Matrix<double, 6, 6>& input,
      const Eigen::Matrix<double, 6, 6>& jacobian);
  ```

  twist 순서는 ROS covariance와 같이 `[vx, vy, vz, wx, wy, wz]`로 고정한다. `base_T_body=[R,t]`일 때 다음을 테스트한다.

  ```text
  world_T_base = world_T_body × inverse(base_T_body)
  omega_base = R × omega_body
  v_base = R × v_body + skew(t) × R × omega_body
  J_twist = [[R, skew(t)R], [0, R]]
  C_out = J × C_in × Jᵀ
  ```

  테스트 케이스는 identity, yaw 90°, roll/pitch 보존, `[0.5,0.2,0.45]` lever arm의 순수 yaw, 상관항이 있는 positive-semidefinite covariance, 비대칭 covariance 거부를 포함한다.

- [ ] **Step 2: 빌드하여 미구현 심볼 실패 확인**

  Run:

  ```bash
  catkin_make -DCATKIN_ENABLE_TESTING=ON
  catkin_make run_tests_wheelchair_tf_adapter_gtest_test_transform_math
  ```

  Expected: 헤더/구현 또는 링크 심볼 부재로 실패.

- [ ] **Step 3: 최소 transform math 구현**

  pose covariance는 ROS의 `[x,y,z, fixed-axis roll,pitch,yaw]` 순서를 명시하고, `world_T_body -> world_T_base` 함수의 중앙차분 Jacobian을 `epsilon=1e-6`으로 계산한다. 각 회전 차이는 `atan2(sin Δ, cos Δ)`로 wrap한다. 입력 pitch가 `|pitch| >= 1.45 rad`이면 Euler singularity 보호를 위해 실패 반환한다.

  covariance 구현은 다음을 강제한다.

  - 모든 원소 finite
  - 대칭 오차 `max|C-Cᵀ| <= 1e-9`
  - 변환 후 `(C+Cᵀ)/2`로 수치 대칭화
  - 고유값이 `-1e-9` 미만이면 invalid

- [ ] **Step 4: 단위 테스트 통과 확인**

  Run:

  ```bash
  catkin_make -DCATKIN_ENABLE_TESTING=ON
  catkin_make run_tests_wheelchair_tf_adapter_gtest_test_transform_math
  catkin_test_results build/test_results/wheelchair_tf_adapter
  ```

  Expected: transform math GoogleTest 전체 통과, 실패 0.

- [ ] **Step 5: 커밋**

  ```bash
  git add src/wheelchair_tf_adapter
  git commit -m "feat: add standard frame transform math"
  ```

---

## Task 3: FAST-LIO 표준 프레임 adapter node 구현

**Files:**

- Create: `src/wheelchair_tf_adapter/config/livox_builtin_measured_20260731.yaml`
- Create: `src/wheelchair_tf_adapter/src/fastlio_standard_frame_adapter.cpp`
- Create: `src/wheelchair_tf_adapter/launch/adapter_only.launch`
- Create: `src/wheelchair_tf_adapter/test/test_adapter_contract.py`
- Create: `src/wheelchair_tf_adapter/test/standard_frames.test`
- Create: `src/wheelchair_tf_adapter/test/test_standard_frames_ros.py`
- Modify: `src/wheelchair_tf_adapter/CMakeLists.txt`
- Modify: `src/wheelchair_tf_adapter/package.xml`

- [ ] **Step 1: node 정적 계약과 rostest fixture를 실패하는 테스트로 작성**

  `livox_builtin_measured_20260731.yaml`에는 아래 topic/frame 계약과 실측 YAML 경로만 기록하고 물리 좌표값을 복사하지 않는다. 정적 테스트는 이 계약과 단독 broadcaster를 검사한다.

  ```yaml
  raw_odom_topic: /Odometry
  raw_cloud_topic: /cloud_registered_body
  output_odom_topic: /odom
  output_cloud_topic: /cloud_registered_base_link
  raw_odom_frame: camera_init
  raw_body_frame: body
  odom_frame: odom
  base_frame: base_link
  max_cloud_odom_skew_s: 0.10
  ```

  rostest fixture는 synthetic `/Odometry`와 커스텀 `PointCloud2` 필드 `x,y,z,intensity,ring,time`을 발행하고 다음을 검증한다.

  - `/odom.header.frame_id == "odom"`
  - `/odom.child_frame_id == "base_link"`
  - `/odom.header.stamp == /Odometry.header.stamp`
  - `/tf`의 `odom -> base_link`와 `/odom.pose`가 같은 transform
  - cloud stamp 보존, frame만 `base_link`
  - XYZ 외 fields, offsets, datatypes, point_step, row_step, byte 값 보존
  - yaw 각속도에서 lever-arm linear velocity 발생
  - 잘못된 frame, zero quaternion, NaN, 시간 역행, 0.10 s 초과 skew에서는 출력 없음과 ERROR diagnostic

- [ ] **Step 2: 테스트를 실행해 node 부재 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q src/wheelchair_tf_adapter/test/test_adapter_contract.py
  catkin_make run_tests_wheelchair_tf_adapter_rostest_standard_frames
  ```

  Expected: 실행 파일/launch 부재 실패.

- [ ] **Step 3: adapter startup 검증 구현**

  node는 publisher와 broadcaster를 만들기 전에 다음 파라미터를 검증한다.

  - 실측 YAML의 schema, units, profile ID와 정확한 세 transform
  - ROS parameter server의 FAST-LIO `mapping/extrinsic_T`가 `[-0.011,-0.02329,0.04412]`
  - `mapping/extrinsic_R`이 identity 3×3
  - `mapping/extrinsic_est_en == false`

  허용 오차는 translation `1e-5 m`, rotation matrix 원소 `1e-6`이다. 불일치하면 exit code 2로 종료하고 어떤 원소가 다른지 진단한다.

- [ ] **Step 4: odometry와 TF 변환 구현**

  각 유효 `/Odometry`에서 다음 한 번의 계산 결과로 `/odom`과 TF를 함께 만든다.

  ```cpp
  const auto odom_T_base = basePoseFromBodyPose(
      camera_init_T_body, base_T_body);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = "odom";
  output.child_frame_id = "base_link";
  broadcaster.sendTransform(same_pose_same_stamp);
  ```

  순서가 반복되거나 감소하는 stamp는 거부한다. quaternion norm은 `[1-1e-3, 1+1e-3]` 안에서만 정규화하고, 그 밖은 오류로 거부한다.

- [ ] **Step 5: PointCloud2 무손실 변환 구현**

  `tf2_sensor_msgs::doTransform`을 사용하여 데이터 배열을 복제한 뒤 XYZ만 변환한다. 변환 전후에 `fields`, `is_bigendian`, `point_step`, `row_step`, `width`, `height`, `is_dense`가 같은지 검사하고 아니면 publish하지 않는다.

  odom/cloud callback은 최근 상대 stamp를 mutex로 관리한다. 상대 입력을 아직 받지 못했거나 차이가 `0.10 s`를 넘으면 해당 출력과 성공 상태를 발행하지 않는다.

- [ ] **Step 6: DiagnosticArray 구현**

  topic은 `/wheelchair_tf_adapter/diagnostics`, status name은 `wheelchair_tf_adapter/standard_frames`, hardware_id는 `wheelchair_livox_builtin_20260731`로 고정한다. values에는 입력/출력 frame, 마지막 stamp, skew, 측정 profile, extrinsic 검증 결과, reject count를 포함한다.

- [ ] **Step 7: 테스트 통과 확인**

  Run:

  ```bash
  catkin_make -DCATKIN_ENABLE_TESTING=ON
  source devel/setup.bash
  python3 -m pytest -q src/wheelchair_tf_adapter/test/test_adapter_contract.py
  catkin_make run_tests_wheelchair_tf_adapter
  catkin_test_results build/test_results/wheelchair_tf_adapter
  ```

  Expected: static test, GoogleTest, rostest 모두 통과.

- [ ] **Step 8: 커밋**

  ```bash
  git add src/wheelchair_tf_adapter
  git commit -m "feat: adapt FAST-LIO to standard wheelchair frames"
  ```

---

## Task 4: FAST-LIO raw TF 격리와 SLAM용 단일 TF tree 구성

**Files:**

- Create: `src/wheelchair_tf_adapter/launch/fastlio_builtin_standard_frames.launch`
- Create: `runtime/start_wheelchair_standard_slam.sh`
- Create: `tests/test_standard_tf_runtime_surface.py`
- Modify: `src/wheelchair_tf_adapter/test/test_adapter_contract.py`

- [ ] **Step 1: launch 구조 테스트 작성**

  테스트는 wrapper에 아래 요소가 모두 있는지 확인한다.

  ```xml
  <group>
    <remap from="/tf" to="/fastlio/raw_tf"/>
    <include file="$(find fast_lio)/launch/mapping_mid360.launch">
      <arg name="rviz" value="false"/>
    </include>
  </group>
  ```

  같은 launch가 measured description과 adapter를 포함하고, `slam_mode=true`일 때만 단 하나의 `map -> odom` identity static publisher를 시작하는지도 검사한다. `/tf_static`은 remap하지 않는다.

- [ ] **Step 2: 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q src/wheelchair_tf_adapter/test/test_adapter_contract.py tests/test_standard_tf_runtime_surface.py
  ```

  Expected: wrapper와 runtime script 부재 실패.

- [ ] **Step 3: wrapper launch 구현**

  인자는 `slam_mode`, `rviz`, `measurements`, `adapter_config`로 제한한다. `slam_mode=false`일 때 `map -> odom`을 만들지 않아 moving ICP가 authority가 될 수 있게 한다.

  raw `/Odometry`와 `/cloud_registered_body`는 remap하지 않는다. FAST-LIO가 보내던 `camera_init -> body`만 `/fastlio/raw_tf`에 남기고 전역 `/tf`에는 adapter의 `odom -> base_link`만 들어가게 한다.

- [ ] **Step 4: 정지 SLAM 시작 script 구현**

  `start_wheelchair_standard_slam.sh`는 다음만 실행한다.

  - `/hardware_motion_authorized=false`
  - Livox driver의 존재 확인 또는 명시적 `START_LIVOX_DRIVER=1`일 때만 시작
  - `fastlio_builtin_standard_frames.launch slam_mode:=true`
  - `/odom`, `/cloud_registered_base_link`, diagnostics 대기
  - `/cmd_vel`, `/wheel_cmd` publisher가 있으면 즉시 종료

  기존 process를 광범위하게 `pkill`하지 않고, script가 시작한 PID만 trap에서 종료한다.

- [ ] **Step 5: 테스트 통과와 launch XML 검사**

  Run:

  ```bash
  python3 -m pytest -q src/wheelchair_tf_adapter/test/test_adapter_contract.py tests/test_standard_tf_runtime_surface.py
  xmllint --noout src/wheelchair_tf_adapter/launch/fastlio_builtin_standard_frames.launch
  bash -n runtime/start_wheelchair_standard_slam.sh
  ```

  Expected: 모두 통과.

- [ ] **Step 6: 커밋**

  ```bash
  git add src/wheelchair_tf_adapter/launch runtime/start_wheelchair_standard_slam.sh tests/test_standard_tf_runtime_surface.py
  git commit -m "feat: isolate FAST-LIO raw TF for standard SLAM"
  ```

---

## Task 5: moving ICP를 `map -> odom -> base_link` 표준 프로필로 연결

**Files:**

- Create: `src/static_livox_localization/config/moving_localization_standard_frames.yaml`
- Create: `src/static_livox_localization/launch/moving_localization_standard_frames.launch`
- Create: `src/static_livox_localization/test/test_standard_frame_profile.py`
- Modify: `src/static_livox_localization/scripts/auto_initial_pose.py`
- Modify: `src/static_livox_localization/scripts/initial_pose_candidates.py`
- Modify: `src/static_livox_localization/test/test_auto_initial_pose_unit.py`
- Modify: `src/static_livox_localization/test/test_auto_initial_pose_surface.py`

- [ ] **Step 1: 표준 config/launch와 seed reference 실패 테스트 작성**

  표준 config는 기존 tuning을 그대로 복사하되 아래 여섯 값만 변경했는지 YAML diff로 검사한다.

  ```yaml
  map_frame: map
  odom_frame: odom
  base_frame: base_link
  odom_topic: /odom
  cloud_topic: /cloud_registered_base_link
  preview_input_topic: /cloud_registered_base_link
  ```

  새 seed 테스트는 chair-centred route의 첫 pose가 표준 `base_link` seed일 때 `(0.517,0.173)` sensor offset을 다시 더하지 않는지 확인한다. 기존 기본 호출은 종전 body seed를 그대로 만들어 하위 호환을 확인한다.

- [ ] **Step 2: 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q \
    src/static_livox_localization/test/test_standard_frame_profile.py \
    src/static_livox_localization/test/test_auto_initial_pose_unit.py \
    src/static_livox_localization/test/test_auto_initial_pose_surface.py
  ```

  Expected: 새 profile 부재 및 `pose_reference` 미지원 실패.

- [ ] **Step 3: 자동 초기화의 입력 topic과 seed 기준점 파라미터화**

  기존 동작을 깨지 않도록 기본값은 raw FAST-LIO로 유지한다.

  ```python
  SubmapCollector(
      window_s,
      odom_topic="/Odometry",
      cloud_topic="/cloud_registered_body",
  )

  load_known_start(
      route_path,
      expected_frame,
      expected_body_frame_profile,
      output_reference="body",  # 기존 기본값
  )
  ```

  `output_reference="chair_centre"`에서는 route가 이미 chair-centred이면 좌표 보정을 적용하지 않는다. 표준 launch는 private params로 `/odom`, `/cloud_registered_base_link`, `chair_centre`를 전달한다.

- [ ] **Step 4: 새 localization config와 thin wrapper launch 구현**

  launch는 기존 `moving_localization.launch`를 include하고 새 config를 넘긴다. FAST-LIO wrapper는 `slam_mode:=false`로 포함하여 `map -> odom` identity가 생기지 않게 한다. moving ICP만 `map -> odom`을 발행한다.

- [ ] **Step 5: 단위·surface 테스트 실행**

  Run:

  ```bash
  python3 -m pytest -q src/static_livox_localization/test
  ```

  Expected: 기존 raw profile 테스트와 새 standard profile 테스트가 모두 통과.

- [ ] **Step 6: 커밋**

  ```bash
  git add src/static_livox_localization
  git commit -m "feat: add standard-frame Livox localization profile"
  ```

---

## Task 6: Navigation 전용 base_link costmap과 shadow bringup 추가

**Files:**

- Create: `src/wheelchair_navigation/config/global_costmap_standard_frames.yaml`
- Create: `src/wheelchair_navigation/config/local_costmap_standard_frames.yaml`
- Create: `src/wheelchair_navigation/launch/standard_livox_navigation.launch`
- Create: `src/wheelchair_navigation/tests/test_standard_livox_navigation.py`
- Create: `runtime/start_wheelchair_standard_navigation.sh`
- Modify: `tests/test_standard_tf_runtime_surface.py`

- [ ] **Step 1: Navigation 프레임과 비구동 계약 테스트 작성**

  테스트에서 다음을 고정한다.

  ```yaml
  global_costmap: {global_frame: map, robot_base_frame: base_link}
  local_costmap: {global_frame: odom, robot_base_frame: base_link}
  ```

  launch는 standard moving localization을 포함하고 move_base의 odom을 `/odom`에 연결한다. hardware adapter는 반드시 `hardware_shadow`이며 `hardware_enabled.launch`와 `/wheel_cmd` 연결은 금지한다.

- [ ] **Step 2: 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q \
    src/wheelchair_navigation/tests/test_standard_livox_navigation.py \
    tests/test_standard_tf_runtime_surface.py
  ```

  Expected: standard navigation 파일 부재 실패.

- [ ] **Step 3: 표준 costmap과 navigation launch 구현**

  기존 costmap tuning 값은 그대로 유지하고 frame만 표준화한다. footprint는 `base_link` XY 평면 기준으로 기존 `costmap_common.yaml` 값을 재사용한다. launch의 command 출력은 `/cmd_vel_nav`까지만이며 실제 base driver에는 연결하지 않는다.

  `standard_livox_navigation.launch` 시작 순서는 다음과 같다.

  1. measured robot description까지 내부에서 한 번만 포함하는 FAST-LIO standard wrapper (`slam_mode=false`)
  2. moving ICP standard profile
  3. map server와 move_base
  4. hardware shadow adapter

  measured description과 adapter를 launch 바깥에서 다시 include하지 않아 `robot_state_publisher`와 `odom -> base_link` authority 중복을 막는다.

- [ ] **Step 4: 정지 navigation 시작 script 구현**

  script는 map 경로와 SHA-256을 둘 다 필수로 받고 하나라도 없으면 exit 64로 실패한다. `/hardware_motion_authorized=false`와 shadow profile을 강제하고, `/wheel_cmd` publisher가 나타나면 전체 launch를 종료한다.

  이 script는 경로 계산과 TF/costmap 관찰용이며 goal을 자동 전송하지 않는다.

- [ ] **Step 5: 테스트와 XML/shell 검증**

  Run:

  ```bash
  python3 -m pytest -q \
    src/wheelchair_navigation/tests/test_standard_livox_navigation.py \
    tests/test_standard_tf_runtime_surface.py
  xmllint --noout src/wheelchair_navigation/launch/standard_livox_navigation.launch
  bash -n runtime/start_wheelchair_standard_navigation.sh
  ```

  Expected: 모두 통과.

- [ ] **Step 6: 커밋**

  ```bash
  git add src/wheelchair_navigation runtime/start_wheelchair_standard_navigation.sh tests/test_standard_tf_runtime_surface.py
  git commit -m "feat: add shadow standard-frame navigation bringup"
  ```

---

## Task 7: fail-closed TF preflight와 rosbag 회귀 검증 추가

**Files:**

- Create: `runtime/preflight_standard_tf.py`
- Create: `tests/test_standard_tf_preflight.py`
- Modify: `runtime/start_wheelchair_standard_slam.sh`
- Modify: `runtime/start_wheelchair_standard_navigation.sh`
- Modify: `src/wheelchair_tf_adapter/test/standard_frames.test`
- Modify: `src/wheelchair_tf_adapter/test/test_standard_frames_ros.py`

- [ ] **Step 1: preflight pure policy 테스트 작성**

  ROS와 분리한 `evaluate(snapshot)` 함수에 synthetic graph를 넣어 각 오류 code를 고정한다.

  ```text
  20 measurement/profile mismatch
  21 robot_description or robot_state_publisher missing
  22 raw camera_init->body leaked onto /tf
  23 map->odom authority count != 1
  24 odom->base_link authority count != 1
  25 required transform missing or multiple-parent/loop
  26 topic frame mismatch, stale stamp, NaN or skew
  27 motion-command publisher active
  ```

  정상 snapshot은 `map -> odom -> base_link -> body -> livox_frame` 단일 체인, `/odom`, `/cloud_registered_base_link`, adapter diagnostics OK를 포함한다.

- [ ] **Step 2: 실패 확인**

  Run:

  ```bash
  python3 -m pytest -q tests/test_standard_tf_preflight.py
  ```

  Expected: preflight module 부재 실패.

- [ ] **Step 3: ROS graph 수집과 preflight CLI 구현**

  다음 데이터를 읽되 어떤 TF도 발행하지 않는다.

  - ROS master publisher 목록과 caller ID
  - `/tf`, `/tf_static`, `/fastlio/raw_tf`를 2초 관찰한 authority
  - `/robot_description`
  - `/odom`, `/cloud_registered_base_link`, adapter diagnostics의 최신 한 메시지
  - tf2 lookup `map -> base_link`, `base_link -> body`, `body -> livox_frame`
  - 측정 YAML hash와 값

  성공 시 한글 요약과 exit 0, 실패 시 해당 code와 원인을 출력한다.

- [ ] **Step 4: 시작 script에서 preflight를 gate로 연결**

  두 runtime script는 adapter가 준비된 뒤 preflight가 0을 반환해야만 `READY_FOR_STATIONARY_VALIDATION`을 출력한다. 실패해도 identity TF나 빈 `/odom`을 대신 만들지 않는다.

- [ ] **Step 5: rosbag replay rostest 확장**

  실제 bag 경로는 환경 변수 `WHEELCHAIR_TF_REPLAY_BAG`로 명시적으로 받아 자동 다운로드하지 않는다. bag의 `/tf`는 `/fastlio/raw_tf`로 remap하고 `/Odometry`, `/cloud_registered_body`만 adapter에 재생한다.

  검증 항목:

  - 유효 raw odometry 각각의 stamp가 `/odom`에 존재
  - `/odom.pose`와 `odom -> base_link` 수치 일치
  - 변환 cloud의 point 수와 모든 field metadata 일치
  - raw `map -> body` 궤적과 표준 `map -> odom -> base_link -> body`의 body pose 오차가 translation `1e-6 m`, rotation `1e-6 rad` 이내
  - 전역 `/tf`에 `camera_init -> body` 없음

- [ ] **Step 6: 테스트 통과**

  Run:

  ```bash
  python3 -m pytest -q tests/test_standard_tf_preflight.py
  catkin_make run_tests_wheelchair_tf_adapter
  catkin_test_results build/test_results/wheelchair_tf_adapter
  ```

  실제 bag이 명시된 검증 환경에서는 추가 실행:

  ```bash
  WHEELCHAIR_TF_REPLAY_BAG=/absolute/path/full_debug_20260727_214306.bag \
    rostest wheelchair_tf_adapter standard_frames.test
  ```

  Expected: synthetic tests 전체 통과. bag 지정 시 궤적/필드/authority 검증 통과.

- [ ] **Step 7: 커밋**

  ```bash
  git add runtime tests src/wheelchair_tf_adapter/test
  git commit -m "test: add fail-closed standard TF preflight"
  ```

---

## Task 8: 운영 문서, 전체 회귀, 정지 NUC 검증

**Files:**

- Create: `docs/runbooks/wheelchair-standard-tf-ko.md`
- Modify only if test requires: package manifests/install lists introduced above

- [ ] **Step 1: 한글 운영 runbook 작성**

  문서에 다음을 기록한다.

  - 실측 기준점 그림과 `[0.500,0.200,0.450]`, `0.322`, `0.772`
  - SLAM과 fixed-map localization에서 `map -> odom` authority가 달라지는 표
  - 정지 시작/종료 절차
  - `view_frames.py`, `tf_echo`, `rostopic hz`, diagnostics 확인법
  - 오류 code별 조치
  - 센서 지지대를 움직인 경우 재측정해야 하는 항목
  - 저속 실차 검증은 별도 승인이 필요하며 이 구현 범위가 아니라는 경계

- [ ] **Step 2: 전체 non-ROS 회귀 실행**

  Run:

  ```bash
  python3 -m pytest -q \
    src/wheelchair_description/tests \
    src/wheelchair_tf_adapter/test/test_adapter_contract.py \
    src/static_livox_localization/test \
    src/wheelchair_navigation/tests \
    tests/test_standard_tf_preflight.py \
    tests/test_standard_tf_runtime_surface.py
  ```

  Expected: 기존 및 신규 Python tests 전체 통과.

- [ ] **Step 3: 전체 catkin 빌드와 ROS tests**

  Run:

  ```bash
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=ON
  catkin_make run_tests
  catkin_test_results
  ```

  Expected: build 성공, failed tests 0.

- [ ] **Step 4: 저장소 위생 검사**

  Run:

  ```bash
  rg -n "0000|sshpass[[:space:]]+-p|SSHPASS=|sk-[A-Za-z0-9]" \
    src/wheelchair_tf_adapter \
    src/wheelchair_description \
    src/static_livox_localization \
    src/wheelchair_navigation \
    runtime docs/runbooks/wheelchair-standard-tf-ko.md
  git diff --check
  git status --short
  ```

  Expected: credential 검색 결과 없음, whitespace 오류 없음, 의도한 파일만 변경.

- [ ] **Step 5: NUC에 배포 전 read-only 비교**

  기존 승인된 SSH 인증으로 `10.26.116.199`에 접속하고, 휠체어를 정지·구동 비활성 상태로 둔다. 먼저 아래를 읽기만 한다.

  ```bash
  rosparam get /mapping/extrinsic_T
  rosparam get /mapping/extrinsic_R
  rosparam get /mapping/extrinsic_est_en
  rostopic echo -n1 /Odometry/header
  rostopic echo -n1 /cloud_registered_body/header
  rosnode info /laserMapping
  ```

  Expected: 내장 IMU MID-360 extrinsic과 raw frame 계약이 계획값과 일치. 하나라도 다르면 배포/시작 중단.

- [ ] **Step 6: NUC 정지 상태 검증**

  새 workspace를 빌드한 뒤 모터 driver를 연결하지 않고 표준 SLAM 또는 localization launch 하나만 시작한다.

  ```bash
  python3 runtime/preflight_standard_tf.py \
    --measurements src/wheelchair_description/config/physical_measurements_20260731.yaml
  rosrun tf2_tools view_frames.py
  rosrun tf tf_echo map base_link
  rosrun tf tf_echo base_link body
  rosrun tf tf_echo body livox_frame
  rostopic echo -n1 /odom
  rostopic hz /odom
  rostopic hz /cloud_registered_base_link
  rostopic info /cmd_vel
  rostopic info /wheel_cmd
  ```

  Expected:

  - 단일 tree `map -> odom -> base_link -> body -> livox_frame`
  - `base_link -> body = (0.500,0.200,0.450)`
  - `body -> livox_frame = (-0.011,-0.02329,0.04412)`
  - `/odom`과 cloud가 증가하며 diagnostics OK
  - `/tf`에 `camera_init -> body` 없음
  - `/cmd_vel`, `/wheel_cmd` publisher 없음

- [ ] **Step 7: 최종 운영 문서 커밋**

  ```bash
  git add docs/runbooks/wheelchair-standard-tf-ko.md
  git commit -m "docs: add standard TF field validation runbook"
  ```

- [ ] **Step 8: 최종 브랜치 push**

  ```bash
  git status --short --branch
  git log --oneline --decorate -8
  git push origin codex/wheelchair-standard-tf-design
  ```

  Expected: clean worktree, 모든 구현 커밋이 `mnjn00/Uniconlab-autonomous-wheelchair`의 `codex/wheelchair-standard-tf-design` 브랜치에 존재.

---

## Definition of Done

- [ ] 새 실측 URDF가 Xacro와 `check_urdf`를 통과하고, 기존 simulation/hardware URDF가 변경 없이 계속 통과한다.
- [ ] `T_odom_base = T_camera_init_body × inverse(T_base_body)`가 pose, twist, covariance에 일관되게 적용된다.
- [ ] `/cloud_registered_base_link`가 모든 비-XYZ PointCloud2 field를 보존한다.
- [ ] FAST-LIO raw `camera_init -> body`는 `/fastlio/raw_tf`에만 있고 전역 `/tf`에는 없다.
- [ ] SLAM과 moving ICP 모드 각각에서 `map -> odom` authority가 정확히 하나다.
- [ ] Navigation은 `map`, `odom`, `base_link`, `/odom` 계약을 사용하고 hardware shadow 상태로만 시작한다.
- [ ] malformed input, extrinsic drift, stale/skewed time, TF 중복에서 fail closed와 DiagnosticArray를 확인한다.
- [ ] 실제 bag replay에서 기존 body 궤적과 표준 tree로 복원한 body 궤적이 허용 오차 안에서 같다.
- [ ] NUC 정지 검증에서 모터 명령 publisher 없이 단일 TF tree가 확인된다.
- [ ] 전체 pytest, catkin build, GoogleTest, rostest가 모두 통과하고 저장소에 credential이 없다.
