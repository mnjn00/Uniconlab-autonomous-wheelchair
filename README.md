# UniconLab Autonomous Wheelchair

[![ROS1 Noetic](https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge&logo=ubuntu)](https://releases.ubuntu.com/20.04/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-7057ff?style=for-the-badge)](https://classic.gazebosim.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-orange.svg?style=for-the-badge)](LICENSE)

한양대학교 Unicon Lab 하드웨어 트랙 프로젝트: **전동휠체어 기반 실환경 탑승 안내 플랫폼**을 위한 ROS1 Noetic 기반 개발 저장소입니다.

이 저장소는 Intel NUC가 이미 **Ubuntu 20.04 + ROS1 Noetic** 환경으로 운용되는 점을 전제로 합니다. 실차 NUC 환경은 바꾸지 않고, 기존 `base_model` 운용 방식과 연결 가능한 시뮬레이션/내비게이션/안전 제어 scaffold를 제공합니다.

> 현재 산출물은 **소프트웨어 RC 후보**입니다. `hardware_motion_authorized: false`와 `passenger_operation_authorized: false`가 강제되며, 검증된 실제 드라이버 계약·물리 e-stop/수동 우선권·정지거리·NUC 자원 시험·별도 승인 전에는 실차 모터 토픽을 광고하거나 탑승 주행에 사용하면 안 됩니다.

---

## Project objective

최종 목표는 캠퍼스 보행 환경에서 전동휠체어가 탑승자를 태우고 지정 경로를 따라 이동하며, 보행자·차량·장애물·경사 구간을 고려해 안전하게 감속/정지/재출발하는 안내 플랫폼을 구현하는 것입니다.

현재 기준 대상 경로:

- **한양대학교 애지문 → 공업센터 입구**
- **공업센터 입구 → 애지문**
- Livox 3D LiDAR + IMU rosbag으로 폐루프 주행 데이터 확보

---

## Repository scope

| Area | Included |
| --- | --- |
| Platform target | Ubuntu 20.04, ROS1 Noetic, catkin |
| Simulator | Gazebo Classic 11 (`SIMULATION_ONLY`) |
| Runtime localization | Native Noetic candidate adapter plus independent confidence guard |
| Navigation/control | `move_base` is the sole `/cmd_vel_nav` publisher; costmap/DWA caps are simulation-only |
| Safety path | `/cmd_vel_nav` → independent safety authorities → `/cmd_vel_safe` → simulation adapter |
| Independent authorities | collision/TTC, slope, localization confidence, route safety/geofence, topology/timing |
| Mission | immutable directional routes, route progress, deterministic mission FSM/action cancellation |
| Offline-only tools | ROS 2 Livox/IMU normalization and pinned GLIM reproduction; never in the NUC runtime closure |
| Hardware boundary | exact manifest parser and shadow profile; real output is rejected while authority/evidence is incomplete |
| Tests | contract/ABI, pure/property, fail-closed safety, replay, package, Gazebo/fault, release/rollback |

ROS 2 and GLIM remain workstation/offline tools only. The production startup closure is ROS1 Noetic; no bridge, Nav2, or ROS 2 runtime process is permitted.

---

## Project status from the internship plan PDF

기준 문서: `/home/mnjn/다운로드/2026년 1학기 인턴 프로젝트 안내(로봇팀).pdf`

| PDF week | Planned target | Current status | Evidence / note |
| --- | --- | --- | --- |
| 1 | 주행 구간·성공 기준 정의 | 완료 | 애지문 ↔ 공업센터 폐루프를 우선 경로로 확정 |
| 2 | 플랫폼·센서 구성 파악 | 부분 완료 | `20230725_wheel_manual (2).pdf`로 NUC, LiDAR, IMU, 조이스틱, ROS1 운용 절차 확인. 실제 `base_model` 토픽/드라이버 소스 확인은 남음 |
| 3 | 주행 경로 조사 1 | 완료 | Livox/IMU rosbag 수집 완료 |
| 4 | 경로 조사 및 지도화 | 완료 | `data/hanyang_aegimun_loop/map.*`, waypoint YAML 커밋 |
| 5 | 시스템 아키텍처 설계 | 완료 | ROS1 Noetic catkin package 구조, navigation/safety/bringup 분리 |
| 6 | 기본 주행 인터페이스 구축 | 시뮬레이션 완료, 실차 대기 | `/cmd_vel_safe` 안전 출력까지 구성. 실차 base controller 연결 검증 필요 |
| 7 | 센서 데이터 수집 및 검증 | 부분 완료 | `/livox/lidar`, `/livox/imu` rosbag 존재. live NUC 센서 토픽 품질 리포트 필요 |
| 8 | 위치 인식 및 경로 추종 방식 선정 | 부분 완료 | GLIM 기반 지도 + 2D navigation/waypoint 추종 방향. 실차 localization 방식 확정 필요 |
| 9 | 직선 경로 저속 주행 | 시뮬레이션 완료, 실차 대기 | Gazebo scaffold 및 command safety path 구성 |
| 10 | 곡선/회전 구간 추종 | 시뮬레이션 진행 중 | 폐루프 waypoint 추종 로직은 실험 중. 실차 곡선 주행은 미검증 |
| 11 | 정적 장애물 정지 | 시뮬레이션 scaffold | safety gate, obstacle costmap 설정 존재. 실차 LiDAR 정지 임계값 검증 필요 |
| 12 | 보행자/이동 객체 대응 | 미완료 | 동적 장애물 감속/정지 실차 테스트 필요 |
| 13~14 | 사용자 안내 기능 | 미완료 | 음성/디스플레이/주요 지점 안내 시나리오 필요 |
| 15~16 | 전체 통합 | 미완료 | 센서-위치추정-경로추종-안전-안내 통합 필요 |
| 17 | 실환경 강건성 검증 | 미완료 | 보행자 밀도, 경사, 시간대별 반복 주행 필요 |
| 18 | 전체 경로 데모 준비 | 미완료 | 전 구간 데모 시나리오, 안정 조건 필요 |
| 19 | 최종 성능 정리 | 미완료 | 성공률, 정지 횟수, 평균 이동시간, 이탈거리 필요 |
| 20 | 최종 발표 및 문서화 | 미완료 | 발표자료, 보고서, 데모 영상 필요 |

요약하면 현재는 **4~5주차 산출물은 상당 부분 완료**, **8~10주차의 경로 추종은 시뮬레이션 선행 구현 단계**, **실차 완전 자율주행은 6~7주차 실차 인터페이스/센서 검증을 통과해야 다음 단계로 진행**하는 상태입니다.

---

## Post-RC gated work (not authorized by this repository)
The following physical activities are future WP7 work, not instructions to operate the current RC. They require a reviewed WP0-HW evidence bundle, actual target-NUC qualification, a verified driver/watchdog and physical override chain, measured geometry/extrinsics/braking/slope envelope, a dry segregated procedure, and named approval. Software/Gazebo success never grants hardware, campus, or passenger authority.

### 1. 실차 NUC ROS1 stack 확인

기존 매뉴얼 기준 운용 명령:

```bash
roscore
roslaunch base_model bringup.launch
roslaunch base_model navigation.launch
roslaunch base_model rviz_wheel.launch
rosrun base_model automode.py
```

확인해야 할 항목:

```bash
rostopic list
rostopic echo /odom
rostopic echo /tf
rostopic info /cmd_vel
rostopic info /base_controller/cmd_vel
rosnode list
rosparam list
```

목표는 실제 휠체어가 최종적으로 받는 제어 입력이 `/cmd_vel`, `/base_controller/cmd_vel`, custom serial/CAN node, joystick override 중 무엇인지 확인하는 것입니다.

### 2. 기존 `base_model`과 이 저장소 연결

- 기존 `base_model` driver는 유지
- navigation 출력은 반드시 `/cmd_vel_nav`로 분리
- safety gate를 통과한 `/cmd_vel_safe`만 base controller로 전달
- 직접 `/cmd_vel_nav`를 motor driver에 연결하는 bypass 금지

### 3. Livox/IMU mapping pipeline 재현성 확보

현재 대용량 rosbag 본체는 Git에 넣지 않고, metadata와 map/waypoint 산출물만 커밋합니다.

필요한 다음 작업:

- `/home/mnjn/다운로드/livox` rosbag replay 절차 정리
- Livox `CustomMsg` → GLIM input 변환 절차 문서화
- GLIM 결과에서 2D map/waypoint를 재생성하는 스크립트/명령 고정
- 경사 통계를 IMU pitch/roll 또는 trajectory z 변화와 연결

### 4. 실차 저속 주행 게이트

실차 주행은 아래 순서로만 진행합니다.

1. 바퀴 공중 또는 안전 스탠드에서 `/cmd_vel_safe` 입력 확인
2. 수동모드/자율모드 전환 확인: `automode.py`에서 `a`, `m`
3. 물리 e-stop 및 joystick override 확인
4. 보행자 없는 평지에서 0.1 m/s 직선 주행
5. 2~3개 waypoint 짧은 구간 주행
6. 경사 구간 속도 제한 적용
7. LiDAR 장애물 정지 확인
8. 애지문 ↔ 공업센터 부분 구간 반복 주행
9. 전체 폐루프 데모

### 5. 평가 지표

최종 보고와 논문형 결과물을 위해 아래 지표를 기록합니다.

| Metric | Definition |
| --- | --- |
| Route success rate | 목표 구간을 개입 없이 완료한 비율 |
| Lateral deviation | route centerline 대비 평균/최대 이탈거리 |
| Stop count | 장애물·안전게이트·수동개입에 의한 정지 횟수 |
| Average speed | 구간별 평균 속도 |
| Traversal time | 출발지-목적지 이동 시간 |
| Intervention count | 수동 조작 또는 emergency intervention 횟수 |
| Localization dropouts | localization failure / TF discontinuity 횟수 |

---

## Real campus route data

실제 주행 rosbag 위치:

```text
/home/mnjn/다운로드/livox
```

rosbag metadata 요약:

| Topic | Type | Count |
| --- | --- | ---: |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 6,882 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 137,602 |

Committed artifacts:

```text
data/hanyang_aegimun_loop/
├── livox_rosbag_metadata.yaml
├── map.yaml
├── map.pgm
├── map.metadata.json
└── hanyang_aegimun_loop.waypoints.yaml
```

The `.db3` bag files are multi-GB and are intentionally not committed to Git.

---

## Existing wheelchair platform reference

Local manual:

```text
/home/mnjn/다운로드/20230725_wheel_manual (2).pdf
```

Key information from the manual:

- ROS1 launch workflow is based on `base_model`
- SLAM: `base_model bringup.launch` + `cartographer_ros 2d_carto.launch`
- Navigation: `base_model bringup.launch` + `base_model navigation.launch`
- Visualization: `base_model rviz_wheel.launch`
- Mode switch: `rosrun base_model automode.py`
  - `a`: auto mode
  - `m`: manual mode
- NUC connections: 19V power, left/right LiDAR, IMU, joystick

---

## Package layout

```text
src/
├── wheelchair_interfaces/       # frozen ROS messages/actions and ABI contracts
├── wheelchair_perception/       # canonical sensor products; no motion permission
├── wheelchair_navigation/       # localizer adapter, route manager, move_base control
├── wheelchair_route_safety/     # independent immutable geofence intersection
├── wheelchair_decision/         # deterministic mission FSM and ExecuteRoute action
├── wheelchair_safety/           # gate plus collision/slope/localization/topology authorities
├── wheelchair_hardware/         # exact driver contract; default boundary is disabled
├── wheelchair_bringup/          # explicit sim/replay/shadow/hardware profiles
├── wheelchair_description/      # provenance-labeled URDF/xacro
└── wheelchair_gazebo/           # simulation-only scenarios and evidence collectors
```

Command authority:

```text
move_base (sole publisher)
  -> /cmd_vel_nav
  -> wheelchair_safety/safety_gate.py
  -> /cmd_vel_safe
  -> simulation_controller_adapter (sim only)
```

No real motor sink is selected by default. `hardware_enabled.launch` must fail before advertising a real command topic while the verified driver and release authorities are false.

The safety gate is the architectural boundary. Motor drivers must not subscribe directly to navigation output.

---

## Ubuntu 20.04 / ROS1 Noetic bootstrap

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-navigation \
  ros-noetic-dwa-local-planner \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-xacro \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-topic-tools \
  python3-catkin-tools python3-pytest
```

---

## Build

Clone this repository as a catkin workspace root:

```bash
git clone https://github.com/mnjn00/Uniconlab-autonomous-wheelchair.git
cd Uniconlab-autonomous-wheelchair
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Environment check:

```bash
./scripts/check_ros1_noetic_env.sh
```

---

## Launch

Display model only:

```bash
roslaunch wheelchair_description display.launch
```

Gazebo Classic software-in-loop startup (disarmed, no hardware path):

```bash
roslaunch wheelchair_bringup sim_bringup.launch auto_start:=false gui:=false
```

The bounded qualification entry points are:

```bash
python3 scripts/run_gazebo_rc_suite.py \
  --config src/wheelchair_gazebo/config/scenarios.yaml \
  --output artifacts/gazebo-rc-report.json
python3 scripts/run_fault_matrix.py \
  --config src/wheelchair_gazebo/config/scenarios.yaml \
  --output artifacts/gazebo-fault-report.json
```

Every report is tagged `SIMULATION_ONLY`; it cannot promote physical authority.

Navigation-only:

```bash
roslaunch wheelchair_navigation navigation.launch use_sim_time:=true
```

Safety gate only:

```bash
roslaunch wheelchair_safety safety.launch
```

---

## Verification

Focused host verification:

```bash
python3 scripts/validate_wp0_contracts.py --root .
python3 -m pytest -q
```

Pinned Ubuntu 20.04/Noetic verification:

```bash
docker build -f tools/noetic/Dockerfile -t wheelchair-noetic-validation .
docker run --rm --network none wheelchair-noetic-validation bash -lc \
  'source /opt/ros/noetic/setup.bash &&
   catkin_make &&
   catkin_make run_tests &&
   python3 -m pytest -q'
```

The suites cover contract/ABI hashes, package and launch topology, nonfinite/time/TTL/reset safety matrices, independent route/collision/slope/localization guards, deterministic mission behavior, replay conversion/GLIM contracts, Gazebo fault evidence, and release/rollback rejection paths. Do not replace a skipped external-data or target-NUC gate with a simulated claim; record it as blocked.

---

## Hardware notes

- Keep the NUC on Ubuntu 20.04 + ROS1 Noetic and retain the existing `base_model` stack for read-only inventory only.
- The repository default driver manifest is unverified and disabled; hardware launch must reject it without advertising a real command topic.
- `/cmd_vel_safe` may reach a real controller only after exact driver topic/unit/sign/rate/timeout/mode/manual/e-stop evidence is reviewed and hash-bound.
- Actual `/odom` and `odom -> base_footprint` ownership, physical e-stop/manual priority, and native command timeout require inert secured-stand verification.
- No grounded motion, campus operation, or passenger trial is authorized by WP0-WP6. Those gates cannot be relaxed by Gazebo, replay, or workstation results.

---

## License

This repository follows the existing repository license: GNU General Public License v2.0. See [LICENSE](LICENSE).
