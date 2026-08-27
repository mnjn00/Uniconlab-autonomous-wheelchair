# MPPI controller 실험 브랜치

브랜치: `feat/mppi-controller`

이 브랜치는 현재 실차 기본 DWA를 교체하지 않는다. 기존 DWA와 safety chain을 그대로 남겨 두고 MPPI를 별도 control profile로 추가한 bench/replay용 첫 구현이다.

## 구조

```text
WaypointFollower guards
  -> DwaFollower의 검증된 step/safety shell 재사용
  -> MppiPlanner만 교체
  -> /cmd_vel_planned
  -> semantic supervisor
  -> safety_gate
  -> terrain_guard
  -> tip_guard
  -> wheel/UART
```

`mppi_follower.py`는 `DwaFollower.step()`을 복사하지 않는다. 따라서 사람 WAIT/GO_ROUND 정책, speed policy, 0.55 s actuation lead, command ramp, manual override, stale/localization/band/geofence guard는 기존 코드 경로를 그대로 사용한다.

## MPPI 기본값

- batch size: 384
- horizon: 30 steps
- model dt: 0.10 s
- prediction horizon: 3.0 s
- temperature: 0.70
- speed noise: 0.18 m/s
- yaw noise: 0.22 rad/s
- seed: 42

RTX/CuPy가 사용 가능하면 stochastic control sampling과 rollout integration을 GPU에서 수행한다. obstacle nearest-neighbour도 기존 `gpu_dwa_backend.py`를 재사용한다. route KD-tree, safety band, route mask는 현재 검증된 CPU 구현을 그대로 유지한다.

## 안전 계약

1. route mask는 hard boundary다.
2. route mask가 없는 경우 safety band가 hard boundary다.
3. route mask가 있는 경우 safety band 이탈은 매우 큰 cost로만 허용된다.
4. obstacle clearance 0.50 m 미만인 rollout은 hard reject한다.
5. 실차에서 측정된 약 0.35 m/s 전진/회전 하한보다 낮은 moving target은 생성하지 않는다.
6. stop은 MPPI sample이 아니라 planner refusal 결과다. 기존 DWA에서 정지 trajectory가 경로 위에서 과도하게 좋은 cost를 받아 장시간 멈춘 실패를 반복하지 않기 위한 조건이다.
7. `require_gpu=true`에서 GPU가 없거나 실패하면 motion proposal을 만들지 않는다.

## 빌드

```bash
cd ~/livox_static_localization_ws/src/Uniconlab-autonomous-wheelchair
git fetch --all --prune
git checkout feat/mppi-controller

cd ~/livox_static_localization_ws
catkin_make
source devel/setup.bash
```

## CPU 단위 테스트

```bash
python3 -m pytest -q \
  src/Uniconlab-autonomous-wheelchair/src/static_livox_localization/test/test_mppi_core.py
```

현재 테스트는 다음 계약을 확인한다.

- open route에서 실행 가능한 전진 명령 반환
- drivable mask 위반 시 `OFF_BAND`
- 0.50 m obstacle floor가 모든 rollout을 막으면 `OBSTACLE`
- speed cap이 loaded-chair turn floor보다 낮으면 `SPEED_BELOW_FLOOR`

## MPPI follower 직접 실행

기존 perception/safety graph가 이미 올라와 있고 follower만 교체하는 bench/replay 상황에서 사용한다.

```bash
roslaunch static_livox_localization mppi_follower.launch \
  route:=/path/to/route.json \
  safety_band:=/path/to/safety_band.json \
  drivable_mask:=/path/to/route_mask.yaml \
  body_frame_profile:=builtin \
  prefer_gpu:=true \
  require_gpu:=false
```

확인:

```bash
rosparam get /waypoint_follower/control_law
# mppi

rosparam get /waypoint_follower/distance_backend
# cupy 또는 numpy

rosparam get /waypoint_follower/gpu_active
```

## 실차 승격 전에 필요한 것

이 브랜치는 아직 `hybrid.sh start/go`의 기본 경로에 연결하지 않았다. 기존 DWA rollback과 preflight 계약을 깨지 않고 A/B 검증하기 위해서다.

다음 순서로 검증하는 것을 전제로 한다.

1. 정적 unit test
2. 기존 rosbag replay에서 DWA와 command 비교
3. 바퀴를 띄운 상태에서 10 Hz deadline 확인
4. 장애물 없는 저속 직선/곡선
5. 정적 장애물 우회
6. DWA와 동일 경로 A/B

비교 지표는 planner latency p50/p95, lateral RMS/max deviation, minimum obstacle clearance, yaw saturation, steering sign reversal, safety_gate veto 수, 완주시간을 권장한다.

특히 현재 구현은 GPU rollout 뒤 safety band/route mask/route KD-tree 평가를 위해 host로 복사한다. 따라서 RTX 2060에서 실제로 10 Hz deadline을 만족하는지 측정하기 전에는 실차 기본 controller로 승격하면 안 된다.
