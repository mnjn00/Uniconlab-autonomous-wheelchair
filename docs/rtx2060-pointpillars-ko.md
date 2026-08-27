# Phantom Canyon RTX 2060 PointPillars ROS1 적용

이 문서는 휠체어에 탑재된 Phantom Canyon NUC의 RTX 2060을 실제 3D 객체
분류에 사용하는 방법을 설명한다. 단순히 CUDA가 설치됐는지 확인하는 수준이
아니라 다음 계산이 GPU에서 수행된다.

1. MID-360 포인트를 CUDA voxel로 변환
2. FP16 TensorRT PointPillars backbone/head 추론
3. CUDA/TensorRT 후처리와 NMS
4. ROS1 `vision_msgs/Detection3DArray` 발행
5. 기존 기하학 객체와 학습 결과 융합
6. 사람·이동 객체 정지 정책과 DWA 회피 입력에 반영

## 고정된 상류 구현

추론 코어는 아래 NVIDIA 구현을 그대로 사용하고 커밋을 고정한다.

```text
repository: NVIDIA-AI-IOT/CUDA-PointPillars
commit:    ce7e2bd694c90207435c8751d61cdb38d48a9f4c
```

우리 저장소는 CUDA voxelizer, TensorRT runtime, decoder나 NMS를 다시
구현하지 않는다. `rtx_pointpillars_node.cpp`는 공식 `libpointpillar_core.so`의
`create_core()`와 `forward()`를 호출하는 ROS1 어댑터다.

## 안전상 역할 분리

PointPillars는 객체의 의미를 보강하지만 유일한 충돌 센서가 아니다.

```text
MID-360 기하학 군집  ── 충돌 형상 권한 ─┐
                                         ├─ hybrid fusion
RTX PointPillars     ── 학습 클래스 ─────┘
```

학습 모델은 기존 기하학 객체를 삭제하지 못한다. detector가 물체를 놓쳐도
기존 LiDAR 군집과 `safety_gate`는 남는다. detector가 죽거나 stale이면
`REQUIRE_LEARNED=true` 프로파일은 출발과 계속 주행을 거부한다.

## 1. NUC 준비 상태 확인

필요한 시스템 구성은 다음과 같다.

- Ubuntu 20.04 / ROS Noetic
- 정상 동작하는 NVIDIA driver와 `nvidia-smi`
- CUDA toolkit과 `nvcc`
- TensorRT headers, `libnvinfer.so`, `trtexec`
- `git-lfs`
- `ros-noetic-vision-msgs`

드라이버나 CUDA/TensorRT 버전을 스크립트가 임의로 교체하지 않는다. 현재
FAST-LIO/GICP 환경을 깨뜨릴 수 있기 때문이다. 일반 apt 패키지만 설치하려면
다음 옵션을 사용할 수 있다.

```bash
INSTALL_APT=true bash tools/hybrid.sh setup-gpu
```

CUDA나 TensorRT가 없다면 스크립트는 필요한 항목을 표시하고 중단한다.

## 2. RTX 2060에서 직접 빌드

NUC에서 저장소의 `main`을 체크아웃한 뒤 실행한다.

```bash
cd ~/wheelchair_localization_src
bash tools/hybrid.sh setup-gpu
```

이 명령은 다음을 자동으로 수행한다.

1. `nvidia-smi`에서 RTX 2060 확인
2. CUDA compute capability 확인
3. 공식 저장소 clone 및 고정 커밋 checkout
4. Git LFS ONNX model 다운로드
5. RTX 2060의 SM에 맞춰 `libpointpillar_core.so` 빌드
6. `trtexec --fp16`으로 해당 NUC에서 TensorRT engine 생성
7. NVIDIA sample point cloud GPU smoke test
8. `static_livox_localization`을 `ENABLE_RTX_POINTPILLARS=ON`으로 재빌드
9. `~/.config/unicon/pointpillars.env` 생성

TensorRT engine은 다른 GPU나 다른 TensorRT 버전에서 만든 파일을 복사해 쓰지
않는다. setup 스크립트가 GPU 이름, SM, ONNX SHA-256을 기록하고 조건이 달라지면
engine을 다시 만든다.

강제로 다시 만들 때는 다음을 사용한다.

```bash
REBUILD_ENGINE=true bash tools/hybrid.sh setup-gpu
```

## 3. GPU를 사용하는 전체 스택 시작

기본 hybrid 시작은 RTX PointPillars를 요구한다.

```bash
bash tools/hybrid.sh start
```

준비가 끝나도 휠체어는 정지 상태다. 실제 출발은 별도 명령이다.

```bash
bash tools/hybrid.sh go
```

정지는 기존과 같다.

```bash
bash tools/hybrid.sh stop
```

또는 조이스틱을 움직여 수동 모드로 전환한다.

## 4. 실제 GPU 사용 확인

스택이 실행 중일 때:

```bash
bash tools/hybrid.sh gpu-status
```

다음을 모두 확인한다.

- GPU 이름에 `RTX 2060`
- `/rtx_pointpillars` node 생존
- `/pointpillars/status`가 `OK`
- `gpu_active: true`
- 고정된 상류 commit
- 최근 inference timestamp
- inference latency가 설정 한도 이하
- 사용한 point 수가 0보다 큼
- NVIDIA compute process와 GPU memory 사용량

직접 토픽을 볼 수도 있다.

```bash
rostopic echo /pointpillars/status
rostopic hz /pointpillars/detections
rostopic echo /pointpillars/detections
nvidia-smi dmon -s pucm
```

## 5. ROS 토픽 흐름

```text
/cloud_registered_body
    ↓
/rtx_pointpillars
    ↓
/pointpillars/detections          vision_msgs/Detection3DArray
/pointpillars/status              std_msgs/String JSON
    ↓
/vision_detection_bridge
    ↓
/perception/learned_objects_summary
    ↓
/hybrid_object_fusion
    ↓
/perception/objects_summary       chair_centre 기준
```

제어 흐름은 다음과 같다.

```text
DWA /cmd_vel_planned
    → semantic_safety_supervisor
    → /cmd_vel_raw
    → safety_gate
    → /cmd_vel_gated
    → terrain_guard
    → /cmd_vel_terrain_safe
    → tip_guard
    → /cmd_vel
```

## 6. 클래스 계약

NVIDIA가 제공한 KITTI 모델의 기본 class ID는 다음과 같다.

```text
0 = car
1 = pedestrian
2 = cyclist
```

ROS bridge에서는 다음 의미로 정규화한다.

```text
0 → vehicle
1 → person
2 → two_wheeler
```

사람은 semantic supervisor에서 정지 대상으로 처리한다. `unknown` 또는 moving
객체도 안전 방향으로 정지한다. 확인된 static 일반 장애물만 DWA 우회 후보가 된다.

## 7. 기본 KITTI 모델의 한계

setup 스크립트로 내려받는 모델은 **GPU 경로를 검증하기 위한 KITTI bootstrap
모델**이다. Velodyne/KITTI 분포로 학습됐으므로 다음 차이가 있다.

- MID-360의 비반복 스캔 패턴
- 휠체어 센서 높이
- 보행자와 오토바이가 많은 보도 환경
- 수풀, 카트, 보도 시설물
- 가까운 거리와 부분 가림

따라서 bootstrap 모델이 NUC에서 빠르게 실행된다는 사실은 우리 경로에서 충분한
recall을 가진다는 증거가 아니다. 실제 성능 개선은 기존 rosbag을 CVAT 등으로
라벨링하고 MID-360 데이터로 PointPillars를 fine-tuning한 뒤 새 ONNX와 TensorRT
engine으로 교체해야 한다.

사용할 engine을 교체할 때:

```bash
POINTPILLARS_MODEL=/path/to/mid360-pointpillar.plan \
LEARNED_MODEL_ID=pointpillars-mid360-v1 \
bash tools/hybrid.sh start
```

단, ONNX의 voxel range, voxel size, class 수와 C++ runtime parameter가 동일해야
한다. 구조가 바뀐 모델은 단순히 plan 파일만 바꾸지 말고 runtime parameter와
class map도 함께 변경하고 replay 검증해야 한다.

## 8. GPU 부하와 실시간 조건

기본 latency gate는 90 ms다.

```yaml
max_inference_ms: 90.0
```

10 Hz cloud에서 90 ms를 넘으면 `SLOW` 상태가 되고 기본 출발 검사를 통과하지
못한다. GPU inference가 멈추거나 오래된 결과만 남는 경우에도 동일하게 출발을
거부한다.

FAST-VGICP CUDA와 PointPillars가 동시에 GPU를 사용할 수 있으므로 다음을 bag에
기록하고 확인한다.

- PointPillars inference ms 분포
- localization correction elapsed ms
- GPU memory
- GPU utilization
- perception summary age
- DWA control-loop gap

RTX 메모리 부족이나 latency 증가가 보이면 모델 입력 범위와 inference rate를
먼저 조정하고, 안전 가드나 기하학 감지를 끄지 않는다.

## 9. GPU 없이 긴급 롤백

기존 검증 경로는 그대로 남아 있다.

```bash
PROFILE=pursuit ~/start_wheelchair_localization.sh
```

hybrid 구조만 유지하고 학습 detector를 일시적으로 끄는 진단 모드는 다음과 같다.

```bash
START_POINTPILLARS=false REQUIRE_LEARNED=false \
  bash tools/hybrid.sh start
```

이 모드는 GPU 객체 분류가 없는 상태이므로 RTX 프로파일의 정식 검증 주행으로
기록하면 안 된다.

## 10. 최초 실차 검증 순서

1. 모터 전원 차단 또는 구동륜을 띄운 상태에서 setup과 start 실행
2. `gpu-status`로 실제 inference 확인
3. 사람, 자전거, 차량, 박스에 대한 RViz box와 실제 위치 비교
4. 사람 앞에서 `/cmd_vel_planned`가 있어도 `/cmd_vel_raw`가 0인지 확인
5. static 장애물에서 DWA proposal과 terrain guard 경계 확인
6. 빈 휠체어·저속·조이스틱 즉시 개입 조건으로 짧은 구간 시험
7. rosbag replay와 NUC latency 기준 통과 후 경로 길이를 늘림

소프트웨어 빌드 성공만으로 승객 운송이나 무감독 운행 권한이 생기지 않는다.
