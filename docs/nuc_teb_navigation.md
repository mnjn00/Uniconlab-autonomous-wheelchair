# NUC 유래 TEB 내비게이션

이 브랜치는 저장소의 TF, URDF, costmap, localization, Navfn 전역 플래너 및 안전 토폴로지를 유지합니다. `move_base`의 로컬 플래너만 TEB로 변경됩니다.

## ROS 의존성

```bash
sudo apt install ros-noetic-teb-local-planner ros-noetic-costmap-converter
```

## 로컬 명령 검사

아래 명령은 로컬 검사용으로 플래너 출력을 `/cmd_vel`에 발행합니다. 이 launch에는 모터 드라이버가 포함되지 않지만, 동일 ROS master에 이미 존재하는 `/cmd_vel` subscriber가 메시지를 소비하면 실제 모터가 구동될 수 있습니다.

실행 전 다음 조건을 모두 충족해야 합니다.

- 격리된 ROS master 또는 시뮬레이션 환경을 사용합니다.
- 하드웨어를 비활성화하거나 물리적으로 분리합니다.
- `rostopic info /cmd_vel`로 `/cmd_vel` subscriber가 없는 것을 확인합니다.

```bash
roslaunch wheelchair_navigation navigation.launch use_sim_time:=true cmd_vel_nav_topic:=/cmd_vel
rostopic info /cmd_vel
rostopic echo /cmd_vel
```

## 보호된 통합 경로

일반 bringup은 `/cmd_vel_nav -> safety_gate -> /cmd_vel_safe` 경로를 유지합니다. 이는 위의 직접 로컬 검사(`/cmd_vel`)와 구분되는 통합 경로입니다.

```bash
roslaunch wheelchair_bringup sim_bringup.launch
rostopic info /cmd_vel_nav
rostopic info /cmd_vel_safe
```

## 출처와 적용

NUC 원본 스냅샷과 SHA-256 기록은 `docs/reference/nuc_teb/`에 있습니다. 활성 TEB 튜닝은 해당 값을 보존하며, footprint는 `wheelchair_navigation/config/costmap_common.yaml`에 맞춘 polygon입니다.
