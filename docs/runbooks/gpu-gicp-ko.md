# CUDA VGICP 배포 및 검증

`moving_icp_localizer`의 기본 정합기는 검증된 `pcl_gicp`이다. GPU 프로필은
명시적으로 CUDA 빌드를 하고 launch 인자를 바꾼 경우에만 활성화된다. GPU
backend를 요청했는데 실행 파일에 CUDA 지원이 없으면 노드는 시작 단계에서
종료한다. CPU로 조용히 되돌아가지 않는다.

## NUC 빌드

CUDA toolkit과 드라이버를 먼저 확인한다.

```bash
nvidia-smi
/usr/local/cuda-11.8/bin/nvcc --version
```

ROS Noetic 작업공간의 `src` 아래에 CUDA 지원 `fast_gicp`를 함께 둔다.

```bash
cd ~/gpu_gicp_ws/src
git clone --recursive https://github.com/SMRT-AIST/fast_gicp.git
git clone https://github.com/Eunkyo-Na/Uniconlab-autonomous-wheelchair.git wheelchair
cd ..
source /opt/ros/noetic/setup.bash
export PATH=/usr/local/cuda-11.8/bin:$PATH
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_VGICP_CUDA=ON \
  -DENABLE_FAST_GICP_CUDA=ON
```

두 CMake 스위치가 모두 필요하다. 첫 번째는 `fast_gicp`의 CUDA 라이브러리를,
두 번째는 휠체어 로컬라이저의 GPU backend 연결을 빌드한다.

## 실행 프로필

CPU 기준 프로필:

```bash
roslaunch static_livox_localization moving_localization.launch \
  registration_backend:=pcl_gicp
```

CUDA 시험 프로필:

```bash
roslaunch static_livox_localization moving_localization.launch \
  registration_backend:=fast_vgicp_cuda
```

`/fast_lio_icp/localization_diagnostics`에서 다음 값을 기록한다.

- `registration_backend`
- `registration_elapsed_ms`
- `registration_error`
- `fitness`, `inlier_ratio`, `source_points`, `target_points`

## 주행 전 게이트

같은 rosbag을 CPU와 GPU 프로필로 각각 재생해 자세, fitness, inlier ratio를
비교한다. CUDA 프로필의 p99 정합 시간이 25 ms 이하이고 기존 로컬라이제이션
판정이 유지되는 것을 확인한 뒤, 정지 실차 확인, 저속 수동 이동, 마지막으로
자율주행 순서로 진행한다. `registration_error`가 비어 있지 않거나 backend가
요청값과 다르면 모터를 활성화하지 않는다.
