# 하이브리드 객체 감지·분류·회피 프로파일

이 문서는 `tools/hybrid.sh`가 실행하는 **실제 ROS1 그래프**를 설명한다.
스크립트 하나가 객체 감지나 회피 알고리즘을 대신하는 것이 아니다. 스크립트는
아래 노드들을 한 명령 체인으로 연결하는 진입점일 뿐이다.

기존 FAST-LIO/GICP 측위, 경로·safety band·drivable mask, 휠 베이스,
`safety_gate.py`, `tip_guard.py`, UART watchdog은 유지한다. 기존
`start_wheelchair_localization.sh`와 `go.sh`도 긴급 롤백 경로로 남는다.

## 실제 구현 범위

### 저장소 안에 구현됨

- 지도 차감 없이 height-filtered MID-360 기하학 클러스터 생성
- 기하학 객체와 선택적 학습 3D detection의 fail-closed 융합
- 모든 제어용 객체 좌표를 `chair_centre` 기준으로 통일
- 사람과 moving/unknown 객체를 DWA보다 뒤, raw safety gate보다 앞에서 정지
- 정지 사람 앞 STOP/GO 반복을 막는 release hysteresis
- DWA 제안 명령을 semantic supervisor가 승인한 뒤에만 `/cmd_vel_raw`로 전달
- route mask와 safety band를 다시 검사하는 terrain guard
- 학습 detector 및 하향 cliff detector의 stale/malformed fail-closed 계약
- PointCloud2 NumPy fast decoder 사용
- 출발 전 전체 그래프와 localization 상태 검사

### 저장소 밖에서 준비해야 함

- PointPillars/CenterPoint 학습 가중치와 TensorRT engine
- 해당 engine을 실행하여 `vision_msgs/Detection3DArray`를 발행하는 ROS1 node
- 실제 하향 2D LiDAR 또는 depth cliff detector
- NUC 배포 후 catkin build, bag replay, 무인 저속 실차 검증

따라서 이 PR은 **학습 모델을 훈련했다고 주장하지 않는다.** 모델이 없으면
geometry-only로 동작하고, `REQUIRE_LEARNED=true`일 때는 학습 detector가 정상이라는
증거가 없으면 출발을 거부한다.

## 실제 명령 체인

```text
/cloud_registered_body + /Odometry + /fast_lio_icp/pose
                  │
                  ▼
hybrid_geometric_objects.py
  - 기존 누적/자체제거/height filter/클러스터/profile/tracker 재사용
  - fixed-map subtraction은 사용하지 않음
  - mapped wall·bench도 회피 geometry에 남김
  /perception/geometric_objects_summary       lidar frame
  /perception/geometric_exclusion_candidates  body frame
                  │
       optional Detection3DArray
                  │
                  ▼
vision_detection_bridge.py
  - source frame → body TF → 측정된 body_T_lidar
  - oriented box를 보수적인 axis-aligned box로 변환
  /perception/learned_objects_summary          lidar frame
                  │
                  ▼
hybrid_object_fusion.py
  - learning은 geometry를 삭제할 수 없음
  - person label 승격 또는 고신뢰 learned-only box 추가
  /perception/objects_summary                  chair_centre frame
                  │
       ┌──────────┴──────────┐
       ▼                     ▼
localization_exclusion_   dwa_follower.py
boxes.py                  /cmd_vel_planned
  - person                 │
  - measured moving        ▼
  - learned dynamic      semantic_safety_supervisor.py
    class만 제외          - person 정지
  /perception/            - moving/unknown 정지
  dynamic_boxes           - stale/malformed 정지
                          /cmd_vel_raw
                              │
                              ▼
                        기존 safety_gate.py
                          /cmd_vel_gated
                              │
                              ▼
                        terrain_guard.py
                          /cmd_vel_terrain_safe
                              │
                              ▼
                        기존 tip_guard.py
                          /cmd_vel → wheel_cmd → UART
```

## 왜 지도 차감을 회피 geometry에서 제거했는가

기존 `obstacle_clusters.py`는 immutable map과 가까운 live return을 제거한 뒤
클러스터링했다. 그 방식은 localization용 novelty 판단에는 유용하지만 다음 교착을
만들 수 있다.

```text
raw safety gate: 벽 또는 장애물이 보이므로 정지
cluster planner : 지도와 겹쳤으므로 객체가 없음
DWA             : 계속 전진 명령
결과            : gate가 계속 0으로 만들어 영구 정지
```

하이브리드 프로파일은 map subtraction을 collision/avoidance geometry에서 끈다.
대신 모든 박스를 localizer에서 제외하지 않도록 별도 노드가 person과 실제 moving
증거만 `/perception/dynamic_boxes`로 보낸다. 정적인 mapped wall을 localization에서
지워 측위를 약화시키지 않는다.

## 좌표계

- 기존 기하학 summary: `lidar`
- 학습 bridge 출력: `lidar`
- fused control summary: `chair_centre`
- marker/localizer exclusion: `body`
- route, band, terrain guard: `map`

fusion 변환은 `body_frame.py`의 실측 extrinsic과
`CHAIR_CENTRE_IN_BODY_XYZ`를 사용한다. LiDAR 상대좌표를 의자 중심 pose에 그대로
더하던 약 0.5 m 전후 오차를 허용하지 않는다.

## 사람과 이동 객체 정책

- 사람: 정지하고 기다림
- `motion == moving`: 정지하고 기다림
- `motion == unknown`: 정지거리 안에서는 정지
- 확인된 static 비사람 객체: DWA가 우회 궤적을 찾음
- perception/command stale 또는 malformed: 정지

사람 때문에 한 번 정지하면 정지 후 작아진 braking envelope만으로 즉시 재출발하지
않는다. 가장 큰 stop radius에 0.30 m release margin을 더한 위치를 벗어날 때까지
정지 상태를 유지한다. perception이 한 프레임 끊겨도 이 latch를 초기화하지 않는다.

## 인도 경계와 낙하 방지

`terrain_guard.py`는 현재 명령을 unicycle model로 정지 horizon까지 rollout하고 다음을
검사한다.

- route mask의 모든 점과 선분이 contained인가
- safety band의 모든 점과 chord가 contained인가
- mask boundary clearance가 hard/slow threshold를 만족하는가
- pose와 입력 명령이 최신인가
- 선택적으로 `/terrain/cliff_status`가 최신이며 safe인가

안전한 인도 내부 우회가 없으면 차도나 mask 밖으로 내려가지 않고 정지한다.

MID-360만으로는 가까운 바닥이나 지도 생성 후 생긴 구멍을 확인할 수 없다.
`CLIFF_REQUIRED=false`에서는 지도·band 기반 방어만 제공한다. 하향 센서를 검증한 후
아래처럼 승격한다.

```bash
CLIFF_REQUIRED=true \
CLIFF_TOPIC=/terrain/cliff_status \
  bash ~/wheelchair_localization_src/tools/hybrid.sh start
```

cliff topic은 `std_msgs/String` JSON이다.

```json
{"stamp": 123.4, "status": "OK", "safe": true}
```

stale, malformed, `safe=false`이면 terrain guard가 0을 출력한다.

## 학습 detector 연결

외부 detector의 출력 계약:

```text
Topic: LEARNED_VISION_TOPIC
Type : vision_msgs/Detection3DArray
```

기본 class map:

| ID | class |
|---:|---|
| 0 | vehicle |
| 1 | person |
| 2 | two_wheeler |
| 3 | obstacle |

권장 기반은 NVIDIA CUDA-PointPillars, NVIDIA Lidar AI Solution 또는
MMDetection3D에서 학습한 PointPillars/CenterPoint다. 모델과 engine은 NUC CUDA,
TensorRT, GPU compute capability 및 자체 MID-360 데이터에 종속되므로 저장소에
가짜 범용 engine을 포함하지 않는다.

```bash
REQUIRE_LEARNED=true \
LEARNED_VISION_TOPIC=/pointpillars/detections \
LEARNED_MODEL_ID=pointpillars-mid360-v1 \
  bash ~/wheelchair_localization_src/tools/hybrid.sh start
```

모델이 준비되지 않은 첫 검증:

```bash
REQUIRE_LEARNED=false \
  bash ~/wheelchair_localization_src/tools/hybrid.sh start
```

이때 heuristic class와 tracked motion을 사용하지만 학습 기반 분류라고 부르지 않는다.

## 기동

전체 그래프를 올리되 PAUSED 상태로 유지:

```bash
bash ~/wheelchair_localization_src/tools/hybrid.sh start
```

출발 전 preflight 후 기존 `go.sh`에 위임:

```bash
bash ~/wheelchair_localization_src/tools/hybrid.sh go
```

정지:

```bash
bash ~/wheelchair_localization_src/tools/hybrid.sh stop
```

`go`는 최소한 다음 노드와 상태를 확인한다.

- `/waypoint_follower`가 실제 DWA control law인지
- `/hybrid_geometric_objects`
- `/hybrid_object_fusion`
- `/localization_exclusion_boxes`
- `/semantic_safety_supervisor`
- `/terrain_guard`
- `/tip_guard`
- fused summary `status == OK`, `frame == chair_centre`
- learning required 시 fusion mode가 hybrid인지
- semantic/terrain guard가 blocked가 아닌지
- localization이 `TRACKING`인지

하나라도 실패하면 auto mode와 follower start service를 호출하지 않는다.

## 배포와 검증 순서

```bash
cd ~/wheelchair_localization_src
git fetch --all --prune
git checkout main
git reset --hard github/main
```

그다음 기존 `push_to_nuc.sh`로 실제 localization workspace에 rsync하고 catkin build한다.
GitHub main만 바뀌어도 NUC 실행 파일은 자동으로 바뀌지 않는다.

승격 순서:

1. Docker/Noetic catkin build 및 unit/regression test
2. NUC build와 모든 토픽 주기 측정
3. 모터 전원을 분리하거나 바퀴를 띄운 command-chain 시험
4. geometry-only, 빈 휠체어, 0.35 m/s 제한 저속 시험
5. 박스·기둥·벽·오토바이 정적 우회
6. 사람 진입 시 정지와 release hysteresis
7. 인도 경계에서 terrain guard 차단
8. 학습 detector shadow bag 수집·평가
9. `REQUIRE_LEARNED=true`
10. 하향 센서 검증 후 `CLIFF_REQUIRED=true`

## 현재 한계

- 기본 global startup은 바뀌지 않았고 hybrid는 명시적으로 실행해야 한다.
- 실제 회피 planner는 TEB가 아니라 기존 DWA다.
- geometry-only mode의 class는 학습 기반이 아니라 기존 heuristic이다.
- 학습 engine과 하향 센서가 없으면 각각 learned semantics와 live cliff 검출은 없다.
- 소프트웨어 CI 통과는 NUC 성능, 실차 주행, 승객 안전 승인을 뜻하지 않는다.
