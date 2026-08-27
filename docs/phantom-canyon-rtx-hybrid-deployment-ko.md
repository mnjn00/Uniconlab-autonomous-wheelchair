# 팬텀 캐년 RTX 2060 하이브리드 회피 배포

이 문서는 `src/static_livox_localization` 실차 스택에 추가된 하이브리드
장애물 감지·회피 경로를 팬텀 캐년 NUC에 배포하는 절차다. 기존 pursuit
기동은 롤백 경로로 남는다.

## 실제 실행 경로

```text
MID-360 accumulated non-ground geometry
  -> hybrid_geometric_objects.py
  -> optional learned Detection3D fusion
  -> chair-centred tracked objects
  -> gpu_dwa_follower.py
       RTX/CuPy obstacle-clearance scoring
  -> semantic_safety_supervisor.py
  -> existing safety_gate.py
  -> terrain_guard.py
  -> existing tip_guard.py
  -> wheel_cmd / UART
```

기하학 객체는 고정 지도와 겹쳐도 삭제하지 않는다. 학습 검출기는 클래스
보강용이며 기하학 장애물을 지울 수 없다. 따라서 학습 가중치가 아직 없어도
정적 장애물 회피는 동작한다.

## 1. NUC GPU 런타임 설치

NUC 저장소에서 한 번 실행한다.

```bash
cd ~/wheelchair_localization_src
git fetch --all --prune
git checkout main
git reset --hard origin/main
bash tools/install_nuc_gpu_runtime.sh
```

스크립트는 다음을 확인한다.

- `nvidia-smi`가 NVIDIA GPU를 찾는지
- 설치된 CUDA toolkit 또는 드라이버 호환 버전
- ROS Noetic Python 버전에 맞는 고정 CuPy wheel
- 실제 GPU 메모리 할당·연산·동기화

Ubuntu 20.04의 기본 Python 3.8에서는 CuPy `12.3.0`을 사용한다. GPU probe가
실패하면 기본 하이브리드 기동은 중단된다.

## 2. 지도와 코드 배포

지도가 연결된 개발 PC에서 실행한다.

```bash
cd Uniconlab-autonomous-wheelchair
git fetch --all --prune
git checkout main
git pull --ff-only

REF=main ./tools/push_to_nuc.sh \
  /Volumes/<지도볼륨>/merged_0707_0725_v1
```

이 과정은 지도 SHA-256을 비교하고 NUC checkout을 동일한 `main` 커밋으로
맞춘 뒤 `static_livox_localization` 패키지를 다시 빌드한다.

## 3. 하이브리드 스택 시작

NUC에서 다음 명령을 사용한다.

```bash
cd ~/wheelchair_localization_src

REQUIRE_GPU=true \
REQUIRE_LEARNED=false \
bash tools/hybrid.sh start
```

`start`는 자동 주행을 시작하지 않는다. 다음이 모두 준비된 뒤 PAUSED 상태로
남는다.

- map subtraction을 하지 않는 non-ground 기하학 객체
- `chair_centre` 좌표 변환과 tracker
- RTX/CuPy DWA obstacle scorer
- 사람·이동/미확정 객체 정지 supervisor
- 기존 raw LiDAR safety gate
- route mask와 safety band terrain guard
- 기존 tip guard

## 4. 출발

첫 검증에서는 바퀴를 띄우거나 구동 전원을 차단한 상태로 명령 토픽부터
확인한다.

```bash
REQUIRE_GPU=true \
REQUIRE_LEARNED=false \
bash tools/hybrid.sh go
```

`go`는 다음 조건을 모두 만족하지 않으면 자동 모드 명령을 보내지 않는다.

- `/waypoint_follower/control_law == dwa`
- `/waypoint_follower/distance_backend == cupy`
- `/waypoint_follower/gpu_active == true`
- fused perception `status == OK`
- route와 fused object의 body-frame/의자 중심 계약 일치
- semantic supervisor와 terrain guard가 clear
- localization `TRACKING`

## 5. GPU 사용 확인

```bash
rosparam get /waypoint_follower/distance_backend
# cupy

rosparam get /waypoint_follower/gpu_active
# true

watch -n 0.5 nvidia-smi
```

장애물을 배치했을 때 `gpu_dwa_follower.py`가 rollout-obstacle 거리 행렬을
RTX에서 계산한다. 경로 최근접 인덱스는 폐루프 경로의 tie-breaking을 기존
CPU `cKDTree`와 완전히 동일하게 유지하기 위해 CPU에 남겨 두었다.

## 6. 학습 검출기 연결

학습된 PointPillars 또는 CenterPoint 노드가
`vision_msgs/Detection3DArray`를 발행할 때만 활성화한다.

```bash
REQUIRE_GPU=true \
REQUIRE_LEARNED=true \
LEARNED_VISION_TOPIC=/pointpillars/detections \
LEARNED_MODEL_ID=pointpillars-mid360-v1 \
bash tools/hybrid.sh start
```

현재 저장소에는 임의의 미검증 가중치를 넣지 않는다. 실제 MID-360 데이터로
학습·검증한 engine이 없을 때 `REQUIRE_LEARNED=true`를 사용하면 기동이
거부되는 것이 정상이다.

## 7. 정지와 롤백

```bash
bash tools/hybrid.sh stop
```

조이스틱을 움직이는 기존 수동 전환도 유지된다. 하이브리드 경로에 문제가
있으면 기존 검증 프로파일로 돌아간다.

```bash
~/start_wheelchair_localization.sh
~/go.sh
```

직접 `REQUIRE_GPU=false`로 CPU fallback 주행을 허용할 수도 있지만, 팬텀 캐년
실차 기본값은 `true`다. GPU가 죽었는데 CPU 부하가 다시 폭증하는 상태로 자동
전환하지 않고 정지하도록 만든 선택이다.
