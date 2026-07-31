# 실측 휠체어 표준 TF·URDF 설계

작성일: 2026-07-31

상태: 사용자 승인 완료

대상: ROS Noetic, Livox MID-360 내장 IMU, FAST-LIO, 3D SLAM·로컬라이제이션·Navigation

## 1. 목표

실제 휠체어에서 측정한 회전 중심과 Livox 내장 IMU 위치를 하나의 물리 기준으로 기록하고, 기존 FAST-LIO 출력인 `camera_init -> body`를 ROS 표준인 `odom -> base_link`로 변환한다.

최종 시스템은 다음 요구사항을 만족해야 한다.

- TF의 주 연결은 `map -> odom -> base_link -> sensor frames`를 따른다.
- FAST-LIO의 연속적인 6DoF 자세를 유지하여 야외 경사로의 roll·pitch를 버리지 않는다.
- `map -> odom`은 SLAM 또는 고정 지도 로컬라이저 중 정확히 하나만 발행한다.
- `/odom` 메시지의 pose, twist, covariance와 `odom -> base_link` TF는 동일한 시각과 동일한 좌표 변환을 사용한다.
- 실제 장착 치수와 센서 내부 외부파라미터는 시뮬레이션 값과 분리한다.
- 기존 FAST-LIO, 기존 로컬라이저, 기존 시뮬레이션 URDF는 보존한다.
- 새 어댑터·실차 URDF·통합 launch·시작 스크립트는 모두 새 파일로 추가한다.
- TF·토픽 계약이 깨지면 주행 준비 검사를 실패시키며 임의의 identity 변환으로 대신하지 않는다.

## 2. 근거와 기존 상태

### 2.1 실제 NUC 실행 기록

2026-07-29 rosbag에서 약 271초 동안 다음 데이터가 확인됐다.

```text
/Odometry  camera_init -> body        2,711개
/tf        camera_init -> body        2,711개
/tf        map -> camera_init         2,701개
```

따라서 기존 FAST-LIO와 moving ICP의 실제 동작 계약은 다음과 같다.

```text
map -> camera_init -> body
```

`camera_init`은 연속적인 지역 odometry 좌표계 역할을 하고, `body`는 FAST-LIO가 사용하는 Livox 내장 IMU 원점이다.

### 2.2 기존 GitHub 하드웨어 모델

기존 `src/wheelchair_description/urdf/wheelchair_hardware.urdf.xacro`에는 2026-07-27 측정값이 기록되어 있다.

```text
회전축 기준 body 수평 위치: x=0.517 m, y=0.173 m
기존 body 높이:             z=0.25588 m
기존 구동휠 반지름:         0.300 m
```

이번 설계에서 사용자가 다시 실측하고 승인한 값은 이 값들을 대체한다. 기존 파일은 이력을 보존하기 위해 수정하지 않는다.

### 2.3 표준 및 실제 로봇과의 비교

- ROS Navigation은 `map -> odom -> base_link -> sensor frames` 연결을 요구한다.
- `map -> odom`은 SLAM·전역 로컬라이저가 담당하고 `odom -> base_link`는 연속 odometry가 담당한다.
- Clearpath Husky와 같은 실차 URDF는 `base_link` 아래에 IMU, LiDAR, 바퀴, 보조 `base_footprint`를 고정 또는 가동 조인트로 연결한다.
- `robot_state_publisher`는 URDF의 fixed joint를 `/tf_static`으로 발행한다.

참고:

- https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- https://docs.nav2.org/concepts/index.html
- https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/husky.urdf.xacro
- https://github.com/mnjn00/Uniconlab-autonomous-wheelchair/blob/main/src/wheelchair_description/urdf/wheelchair_hardware.urdf.xacro

## 3. 실측 좌표 계약

모든 길이 단위는 미터, 회전 단위는 라디안이다. REP-103에 따라 `+X`는 전진, `+Y`는 탑승자 기준 왼쪽, `+Z`는 위쪽이다.

### 3.1 `base_link`

`base_link`는 좌우 큰 구동휠 회전축의 정중앙이다. 실제 휠체어의 회전 중심이며 Navigation의 `robot_base_frame`으로 사용한다.

```yaml
base_link:
  definition: midpoint_of_drive_wheel_rotation_axis
  height_from_ground_m: 0.322
```

### 3.2 Livox 내장 IMU `body`

`body`는 Livox 외함 중심이 아니라 FAST-LIO가 사용하는 MID-360 내장 IMU 원점이다.

```yaml
base_link_to_body:
  translation_m: [0.500, 0.200, 0.450]
  rotation_rpy_rad: [0.0, 0.0, 0.0]
  source: physical_wheelchair_manual_measurement
  measurement_date: 2026-07-31
  measurement_quality: approximate_manual
```

검산값:

```text
바닥 -> base_link = 0.322 m
base_link -> body  = 0.450 m
바닥 -> body      = 0.772 m
수평 이격 거리    = sqrt(0.500² + 0.200²) = 0.5385 m
```

### 3.3 Livox 광학 원점 `livox_frame`

MID-360의 내장 IMU 원점과 LiDAR 광학 원점은 같은 점으로 가정하지 않는다. 기존 실차 모델과 FAST-LIO MID-360 설정에 사용된 내부 외부파라미터를 고정 조인트로 기록한다.

```yaml
body_to_livox_frame:
  translation_m: [-0.01100, -0.02329, 0.04412]
  rotation_rpy_rad: [0.0, 0.0, 0.0]
  source: deployed_fast_lio_mid360_extrinsic
```

구현 전 배포된 NUC의 `fast_lio/config/mid360.yaml`과 위 값을 비교한다. 하나라도 다르면 자동으로 선택하지 않고 사전검사를 실패시킨다.

## 4. 최종 TF 트리

```text
map
└── odom
    └── base_link
        ├── base_footprint
        ├── body
        │   └── livox_frame
        ├── chassis_link
        ├── seat_link
        ├── backrest_link
        ├── nuc_link
        ├── left_drive_wheel_link
        ├── right_drive_wheel_link
        ├── left_caster_link
        └── right_caster_link
```

각 TF의 단일 발행자는 다음과 같다.

| TF | 종류 | 발행자 |
|---|---|---|
| `map -> odom` | 동적 | SLAM 또는 moving ICP 중 하나 |
| `odom -> base_link` | 동적 | 새 FAST-LIO 표준 프레임 어댑터 |
| `base_link -> body` | 정적 | `robot_state_publisher` |
| `body -> livox_frame` | 정적 | `robot_state_publisher` |
| `base_link -> base_footprint` | 정적 | `robot_state_publisher` |
| 차체·좌석·NUC 고정 링크 | 정적 | `robot_state_publisher` |
| 바퀴·캐스터 가동 링크 | 동적 | 실제 `/joint_states`가 있을 때만 `robot_state_publisher` |

`base_footprint`은 지면의 보조 프레임이며 주 odometry의 child로 사용하지 않는다.

```yaml
base_link_to_base_footprint:
  translation_m: [0.0, 0.0, -0.322]
  rotation_rpy_rad: [0.0, 0.0, 0.0]
```

이 구조는 `odom -> base_link`의 6DoF 자세를 그대로 유지한다.

## 5. FAST-LIO 표준 프레임 어댑터

### 5.1 입력

```text
/Odometry
  header.frame_id = camera_init
  child_frame_id  = body

/cloud_registered_body
  header.frame_id = body
```

프레임 이름이 다르면 메시지를 변환하지 않고 진단 오류를 발행한다.

### 5.2 Odometry pose 변환

`odom`은 `camera_init`과 같은 지역 좌표계로 정의하고 이름만 표준화한다.

```text
T_odom_base_link
  = T_camera_init_body × inverse(T_base_link_body)
```

확정된 고정 변환:

```text
T_base_link_body = translation(0.500, 0.200, 0.450)
```

어댑터는 동일한 계산 결과를 사용하여 다음 둘을 함께 발행한다.

```text
/odom
  header.frame_id = odom
  child_frame_id  = base_link

/tf
  odom -> base_link
```

타임스탬프는 원본 `/Odometry`의 `header.stamp`를 그대로 사용한다.

### 5.3 Twist와 covariance

원본 twist는 `body` 원점의 속도이므로 child frame 이름만 바꾸거나 수치를 그대로 복사하면 안 된다. 어댑터는 rigid-body adjoint/Jacobian을 사용해 `base_link` 원점의 선속도와 각속도로 변환한다.

Livox 장착 레버암을 `r_base_body`라고 할 때 축이 정렬된 현재 실측에서는 다음 관계를 만족해야 한다.

```text
v_base = v_body - omega × r_base_body
omega_base = omega_body
```

일반 구현은 회전이 0이 아닌 캘리브레이션도 처리한다. pose covariance와 twist covariance도 같은 좌표변환의 6×6 Jacobian으로 변환한다. 단순 복사는 허용하지 않는다.

### 5.4 PointCloud2 변환

```text
/cloud_registered_body
  -> T_base_link_body 적용
  -> /cloud_registered_base_link
```

출력은 원본과 같은 timestamp를 사용하고 `header.frame_id`를 `base_link`로 설정한다. XYZ 이외의 intensity, timestamp, ring 등 사용자 필드는 손실 없이 보존한다.

### 5.5 기존 FAST-LIO TF 격리

기존 FAST-LIO 소스는 수정하지 않는다. 새 FAST-LIO wrapper launch에서 원본 `/tf`를 `/fastlio/raw_tf`로 remap한다.

```text
camera_init -> body  : /fastlio/raw_tf에서만 관찰 가능
odom -> base_link    : 전역 /tf의 유일한 지역 odometry TF
```

원본 `/Odometry`와 `/cloud_registered_body`는 디버깅과 회귀분석을 위해 유지한다.

## 6. 로컬라이저와 SLAM 연결

### 6.1 고정 지도 Localization·Navigation 모드

moving ICP의 새 설정은 다음 입력을 사용한다.

```yaml
map_frame: map
odom_frame: odom
base_frame: base_link
odom_topic: /odom
cloud_topic: /cloud_registered_base_link
```

moving ICP는 `map -> odom`만 발행한다.

### 6.2 SLAM 모드

SLAM 모드에서도 어댑터가 `odom -> base_link`를 발행한다. 선택된 SLAM 시스템만 `map -> odom`을 발행한다.

지도 작성 중 임시로 `map == odom`을 사용해야 한다면 SLAM 전용 launch가 identity `map -> odom`을 한 번만 발행한다. moving ICP와 동시에 실행할 수 없다.

### 6.3 Navigation 모드

Navigation의 기본 계약은 다음과 같다.

```yaml
global_frame: map
odom_frame: odom
robot_base_frame: base_link
odom_topic: /odom
```

경사로에서 roll·pitch를 보존하므로 어댑터와 상태 추정기에는 `two_d_mode`에 해당하는 투영을 적용하지 않는다. 2D costmap은 `base_link`의 평면 footprint를 사용하되 3D 위치추정 자체를 평면화하지 않는다.

## 7. 실차 URDF 정책

새 실차 URDF는 기존 Gazebo용 `wheelchair.urdf.xacro`를 include하지 않는다.

- Gazebo plugin, 가상 IMU, 가상 LiDAR, transmission을 포함하지 않는다.
- 실측값은 별도 `physical_measurements_20260731.yaml`에 기록한다.
- 센서 TF와 차체 collision은 같은 기준점 `base_link`를 사용한다.
- 구동휠과 캐스터는 실제 기구학을 표현하도록 `continuous` 조인트로 유지한다.
- 실차 SLAM·Navigation bringup은 가짜 `joint_state_publisher`를 실행하지 않는다.
- 바퀴 시각화가 필요한 별도 display launch에서만 기본 joint state 발행을 허용한다.
- 실제 encoder `/joint_states`가 연결되면 URDF 변경 없이 동적 휠 TF를 사용한다.

Navigation에 필요한 센서 고정 TF는 바퀴 joint state가 없어도 `/tf_static`에 모두 존재해야 한다.

## 8. 새 파일 구조

기존 파일을 덮어쓰지 않고 다음 파일을 새로 추가한다.

```text
catkin_ws/src/wheelchair_tf_adapter/
├── CMakeLists.txt
├── package.xml
├── include/wheelchair_tf_adapter/transform_math.hpp
├── src/transform_math.cpp
├── src/fastlio_standard_frame_adapter.cpp
├── config/livox_builtin_measured_20260731.yaml
├── launch/fastlio_standard_frames.launch
└── test/
    ├── test_transform_math.cpp
    ├── test_adapter_contract.py
    └── standard_frames.test

catkin_ws/src/wheelchair_description/
├── urdf/wheelchair_hardware_measured_20260731.urdf.xacro
├── config/physical_measurements_20260731.yaml
├── launch/hardware_description_measured_20260731.launch
└── test/test_measured_hardware_description.py

overlay/static_livox_localization/
├── config/moving_localization_standard_frames.yaml
└── launch/moving_localization_standard_frames.launch

integration/wheelchair_navigation/launch/
├── standard_livox_slam.launch
└── standard_livox_navigation.launch

runtime/
├── start_wheelchair_standard_slam.sh
├── start_wheelchair_standard_navigation.sh
└── preflight_standard_tf.py
```

새 `wheelchair_description` 패키지는 구현 시 현재 로컬 checkout에 같은 이름의 패키지가 있는지 다시 검사한다. 동일 패키지가 이미 생겼다면 그 기존 파일을 수정하지 않고 충돌하지 않는 새 패키지명 `wheelchair_hardware_description`을 사용한다.

## 9. 오류 처리와 fail-closed 조건

다음 조건에서는 `/odom`, 변환 점군 또는 주행 준비 성공 상태를 발행하지 않는다.

- `/Odometry.header.frame_id != camera_init`
- `/Odometry.child_frame_id != body`
- `/cloud_registered_body.header.frame_id != body`
- Odometry와 point cloud timestamp가 허용 지연을 초과
- quaternion이 정규화 불가능하거나 NaN/Inf 포함
- 실측 변환 파일 누락 또는 단위 오류
- 배포된 MID-360 `extrinsic_T/R`과 고정된 실측 프로파일 불일치
- 전역 `/tf`에 `camera_init -> body`가 남아 있음
- `map -> odom` 발행자가 0개 또는 2개 이상
- `odom -> base_link` 발행자가 2개 이상
- `robot_description` 또는 `robot_state_publisher` 누락
- `map -> base_link`, `base_link -> body`, `body -> livox_frame` 조회 실패

어댑터는 오류 상태를 `diagnostic_msgs/DiagnosticArray`로 발행한다. 오류 중 마지막 정상 TF를 계속 새 timestamp로 재발행하지 않는다.

## 10. 테스트 및 실제 검증

### 10.1 변환 수학 단위 테스트

- identity raw pose에서 `base_link` 위치가 `body` 기준 역변환과 일치
- yaw 90도에서 레버암이 odom 축으로 올바르게 회전
- roll·pitch가 있는 경사 자세를 평면화하지 않음
- body 원점 각속도가 있을 때 base_link 선속도에 레버암 보정 적용
- pose/twist covariance가 6×6 Jacobian으로 변환
- 잘못된 frame_id, NaN, zero quaternion 거부

### 10.2 URDF 계약 테스트

- Xacro 전개와 `check_urdf` 성공
- 모든 link에 부모가 최대 하나
- `base_link -> body = (0.500, 0.200, 0.450)`
- `base_link -> base_footprint = (0, 0, -0.322)`
- `body -> livox_frame = (-0.011, -0.02329, 0.04412)`
- 실차 URDF에 Gazebo plugin, 가상 센서, transmission이 없음
- Navigation 필수 센서 링크가 모두 `base_link`에 연결됨

### 10.3 rosbag 회귀 테스트

기존 실제 rosbag `/Odometry`와 `/cloud_registered_body`를 어댑터에 재생한다.

- `/odom` 개수와 timestamp가 원본 `/Odometry`와 일치
- `odom -> base_link` TF가 `/odom.pose`와 수치적으로 일치
- 변환 점군의 필드 수와 point 수가 원본과 일치
- `map -> odom -> base_link -> body -> livox_frame`이 하나의 트리
- raw TF가 전역 `/tf`에 섞이지 않음
- 기존 `map -> body` 궤적과 새 `map -> base_link -> body` 궤적의 body pose가 허용 오차 안에서 동일

### 10.4 NUC 정지 검증

휠체어를 움직이지 않고 구동 출력을 비활성화한 상태에서 실행한다.

```text
rosrun tf2_tools view_frames.py
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link body
rosrun tf tf_echo body livox_frame
rostopic echo -n1 /odom
rostopic hz /odom
rostopic hz /cloud_registered_base_link
```

성공 조건:

- TF가 단일 트리이며 loop·multiple parent·중복 authority가 없음
- 정지 상태에서 `/odom` pose가 유한하고 timestamp가 증가
- `base_link -> body`와 `body -> livox_frame`이 실측값으로 고정
- point cloud가 RViz의 `base_link`, `odom`, `map`에서 같은 구조로 정렬
- 어떤 노드도 `/cmd_vel` 또는 모터 명령을 발행하지 않음

### 10.5 저속 실차 검증

정지 검증을 통과한 뒤 별도 승인 하에 수행한다.

- 탑승자 없이 저속 직진
- 제자리 좌우 회전
- 완만한 경사 진입·정지·후진
- 기록한 rosbag에서 map/odom 연속성, 레버암 회전 궤적, 점군 정렬 확인

저속 실차 검증은 이 구현 작업의 자동 실행 범위에 포함하지 않는다.

## 11. 구현 범위 밖

- 모터 제어기 변경
- 경로계획·Pure Pursuit 튜닝
- 지지대 CAD·재질·체결 설계
- VN-100 사용
- 휠 encoder 하드웨어 연동
- 기존 Gazebo 모델 교체
- 실제 휠체어의 무인 주행 시험

## 12. 완료 기준

- 새 실측 프로파일, 실차 URDF, 표준 프레임 어댑터, 통합 launch와 preflight가 새 파일로 존재한다.
- 변환 수학·URDF·launch 계약 테스트가 통과한다.
- 기존 실제 rosbag 회귀 테스트에서 body 궤적이 보존된다.
- NUC 정지 검증에서 다음 단일 트리가 확인된다.

```text
map -> odom -> base_link -> body -> livox_frame
```

- 기존 소스와 기존 시뮬레이션·하드웨어 URDF는 수정되지 않는다.
- 구동 명령을 발행하지 않고 정지 상태 검증까지 완료한다.
