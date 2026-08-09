# NUC 유래 TEB 내비게이션

이 브랜치는 저장소의 TF, URDF, costmap, localization, Navfn 전역 플래너 및 안전 토폴로지를 유지합니다. `move_base`의 로컬 플래너만 TEB로 변경됩니다.

## ROS 의존성

```bash
sudo apt install ros-noetic-teb-local-planner ros-noetic-costmap-converter
```

## 로컬 명령 검사

아래 명령은 로컬 검사용으로 플래너 출력을 `/cmd_vel`에 발행합니다. 모터 드라이버를 추가하거나 실제 모터를 구동하지 않습니다.

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
