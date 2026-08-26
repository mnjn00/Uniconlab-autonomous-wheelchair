# 하이브리드 객체 감지·분류·회피 프로파일

이 프로파일은 ROS1 Noetic, FAST-LIO/GICP 측위, 기존 경로·안전 밴드,
`wheel_cmd_tmp.py`/UART를 바꾸지 않는다. 기존 기동은 긴급 롤백으로 그대로
남기고, `start_hybrid_avoidance.sh`가 기동 후 교체 가능한 노드만 안전하게
다시 올린다.

## 해결하는 문제

1. **학습 검출기가 놓친 장애물이 사라지는 문제**
   - 기존 기하학 클러스터가 항상 남는다.
   - 학습 결과는 클래스 보강과 고신뢰 box 추가만 할 수 있다.
   - 학습 결과가 기하학 객체를 삭제하거나 도로를 clear로 만들 수 없다.

2. **LiDAR 원점과 의자 중심 원점 혼용**
   - 기존 `/perception/geometric_objects_summary`는 `lidar` 기준이다.
   - fusion 출력은 실제 회전 중심인 `chair_centre` 기준으로 변환된다.
   - DWA가 의자 중심 pose에 상대 객체 좌표를 더할 때 약 0.5 m의 전후 오차가
     생기지 않는다.

3. **사람 앞에서 STOP/GO가 반복되는 문제**
   - DWA 명령은 바로 안전 게이트로 가지 않고 semantic supervisor를 지난다.
   - 사람 정지는 동적 정지거리가 줄어도 0.30 m release margin까지 유지된다.
  -  원직이거나 아직 정지 여부를 알 수 없는 객체는 가까우면 정지한다.
   - 확인된 정적 비사람 객체만 DWA에 우회를 맡긴다.

4. **인도 경계 밖으로 회피할 가능성**
   - `terrain_guard.py`가 현재 명령을 정지 horizon까지 rollout한다.
   - route mask와 safety band의 모든 점·선분을 통과해야 한다.
   - 경계 0.35 m 이내에서는 0.35 m/s로 제한하고, 0.12 m 미만 또는 경계
     횡단이면 즉시 정지한다.
  -  하향 cliff 센서가 추가되면 `CLIFF_REQUIRED=true`로 센서 침묵까지 정지
     조건으로 만들 수 있다.

5. **PointCloud2 Python 변환 CPU 폭주**
   - follower와 safety gate가 공유하는 `scan_accumulator.py`를
     `cloud_points.points_xyz()`의 NumPy decoder로 교체했다.

## 새 명령 체인

```text
obstacle_clusters.py
  /perception/geometric_objects_summary   lidar frame
                  │
learned Detection3DArray (optional)
  vision_detection_bridge.py
  /perception/learned_objects_summary     lidar frame
                  │
                  ▼
hybrid_object_fusion.py
  /perception/objects_summary             chair_centre frame
                  │
                  ▼
dwa_follower.py
  /cmd_vel_planned
                  │
                  ▼
semantic_safety_supervisor.py
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

각 단계가 죽으면 다음 단계의 stale watchdog이 0 명령을 낸다. 기존 joystick
manual override와 `stop.sh`도 그대로 유지된다.

## 배포

저장소를 NUC에 배포하고 `static_livox_localization` 패키지를 다시 빌드한다.
기존 `push_to_nuc.sh`가 저장소 패키지를 실제 workspace로 rsync하고 catkin build를
수행한다.

```bash
cd ~/wheelchair_localization_src
git fetch --all --prune
git checkout main
git reset --hard github/main

# 기존 지도 볼륨을 사용하는 표준 배포
./tools/push_to_nuc.sh /path/to/merged_0707_0725_v1
```

wrapper는 `$HOME` 복사본이 없어도 저장소에서 직접 실행할 수 있다.

```bash
~/wheelchair_localization_src/tools/start_hybrid_avoidance.sh
~/wheelchair_localization_src/tools/go_hybrid.sh
~/stop.sh
```

기존 검증 프로파일로 복귀할 때는 기존 스크립트를 사용한다.

```bash
~/start_wheelchair_localization.sh
~/go.sh
```

## 학습 detector 없이 시작

기본은 geometry-only다. 장애물 회피와 사람 heuristic은 기존 detector를 사용하고,
새 좌표계·semantic stop·terrain guard·fast cloud decoder는 모두 활성화된다.

```bash
REQUIRE_LEARNED=false \
  ~/wheelchair_localization_src/tools/start_hybrid_avoidance.sh
```

fusion status는 `mode: geometric_only`이고, 이것은 정상 상태다. geometric source가
`NO_CLOUD`, `NO_MAP_POSE`, stale이면 output `status`가 `OK`가 아니므로 모든
consumer가 fail closed한다.

## PointPillars 또는 CenterPoint 연결

모델·TensorRT engine은 저장소에 넣지 않는다. 엔진은 GPU compute capability,
TensorRT/CUDA 버전, 학습 클래스에 종속되기 때문이다. 외부 detector는 표준
`vision_msgs/Detection3DArray`를 발행하면 된다.

권장 구현 기반:

- NVIDIA-AI-IOT/CUDA-PointPillars
- NVIDIA-AI-IOT/Lidar_AI_Solution
- OpenMMLab MMDetection3D에서 학습 후 TensorRT export

예시:

```bash
# 외부 PointPillars node가 이 topic을 발행 중이어야 한다.
rostopic type /pointpillars/detections
# vision_msgs/Detection3DArray

REQUIRE_LEARNED=true \
LEARNED_VISION_TOPIC=/pointpillars/detections \
LEARNED_MODEL_ID=pointpillars-mid360-v1 \
  ~/wheelchair_localization_src/tools/start_hybrid_avoidance.sh
```

`vision_detection_bridge.py` 기본 class map:

| ID | class |
|---:|---|
| 0 | vehicle |
| 1 | person |
| 2 | two_wheeler |
| 3 | obstacle |

detector frame은 TF로 `lidar`에 변환된다. TF가 없으면 bridge가
`TF_UNAVAILABLE`을 내며, `REQUIRE_LEARNED=true`에서는 출발이 거부된다.

## 학습 데이터 정책

처음 클래스는 네 개면 충분하다.

```text
person
vehicle
two_wheeler
obstacle
```

세션 또는 rosbag 단위로 train/validation/test를 나눈다. 인접 프레임을 무작위로
나누면 같은 장면이 양쪽에 들어가 성능이 부풀려진다. 반드시 포함할 hard negative:

- 탑승자 다리·발·발판·팔걸이
- 경사 정상부의 도로
- 대각선 벽, 수풀, 나뭇가지
- 반사 표지판, 거울·유리
- 벽 가까이 선 사람
- 가려진 오토바이, 낮은 박스, 얇은 기둥

학습 detector는 semantic 보조다. 충돌 recall은 geometric source와 raw safety gate가
계속 담답한다.

## 인도와 cliff 센서

현재 지도 기반 방어는 즉시 사용할 수 있다.

```bash
CLIFF_REQUIRED=false \
  ~/wheelchair_localization_src/tools/start_hybrid_avoidance.sh
```

하향 2D LiDAR 또는 depth cliff node가 다음 JSON String을 발행하도록 연결한다.

```json
{"stamp": 123.4, "status": "CLEAR", "safe": true}
```

그다음:

```bash
CLIFF_REQUIRED=true \
CLIFF_TOPIC=/terrain/cliff_status \
  ~/wheelchair_localization_src/tools/start_hybrid_avoidance.sh
```

센서가 stale, malformed, unsafe이면 `terrain_guard`가 0을 출력한다. 센서가
차도 쪽 지면을 발견해도 route mask 밖을 허가하지 않는다.

## 출발 전 확인

`go_hybrid.sh`는 아래를 모두 명령 전에 확인한다.

- `/waypoint_follower/control_law == dwa`
- fused summary `status == OK`
- fused frame `chair_centre`
- learning required 시 `mode == hybrid`
- semantic supervisor가 hold 중이 아님
- terrain guard가 hold 중이 아님
- localization이 `TRACKING`

하나라도 실패하면 auto mode와 start service를 호출하지 않는다.

## 기록되는 추가 토픽

기존 black box 외에 `hybrid_*.bag`이 다음만 별도로 기록한다.

```text
/perception/geometric_objects_summary
/perception/learned_objects_summary
/perception/hybrid_status
/cmd_vel_planned
/semantic_safety/status
/cmd_vel_terrain_safe
/terrain_guard/status
```

## 실차 승격 순서

1. 바퀴를 띄우거나 모터 전원을 분리한 상태에서 topic chain 확인
2. geometry-only, `CLIFF_REQUIRED=false`, 0.35 m/s 이하 저속 시험
3. 박스·콘·벽·오토바이 정적 우회
4. 사람이 경로에 들어올 때 정지와 release 확인
5. 인도 경계에서 `MASK_CLEARANCE`와 `MASK_BOUNDARY` 확인
6. 학습 detector shadow 기록과 오검출 분석
7. `REQUIRE_LEARNED=true` 승격
8. 하향 센서 검증 후 `CLIFF_REQUIRED=true` 승격

이 프로파일은 main에 포함되어도 기존 startup의 실차 검증 상태를 덮어쓰지 않는다.
새 profile은 별도로 검증하고, 문제가 있으면 기존 startup으로 즉시 되돌린다.
