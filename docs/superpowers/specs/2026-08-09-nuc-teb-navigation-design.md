# NUC TEB 주행 이식 설계

- 작성일: 2026-08-09
- 대상 브랜치: `codex/nuc-teb-navigation`
- 기준 저장소: GitHub `main`의 `a54f130`
- 상태: 사용자 승인 설계를 문서화한 구현 전 명세

## 1. 목표

GitHub 저장소가 이미 정의한 ROS 1 navigation 구조, TF, URDF, costmap, 지도, localization 및 safety 구조는 유지한다. `move_base`의 로컬 경로 추종·회피 방식만 `dwa_local_planner/DWAPlannerROS`에서 NUC에 남아 있는 `teb_local_planner/TebLocalPlannerROS`로 바꾼다.

Pure Pursuit, S자 waypoint follower 또는 별도의 경로 생성기를 TEB 실행 경로에 추가하지 않는다.

## 2. 원본과 추적성

NUC `mprp3@10.242.33.199`에서 2026-08-09에 읽은 다음 파일을 기준 원본으로 사용한다.

| NUC 원본 | SHA-256 |
|---|---|
| `/home/mprp3/catkin_ws/src/base_model/config/move_base.yaml` | `dec3b50729fe9c139b6e7aaead24ce33ab39862a55538aefdc710acedbf0dc3c` |
| `/home/mprp3/catkin_ws/src/base_model/config/teb_local_planner.yaml` | `eec4ca2a275b53eb214ece71c74fc1557856c0286e10890aa419fbd383b6905a` |

두 원본은 저장소의 문서용 reference 영역에 원문 그대로 보존한다. 자동 테스트는 reference 파일의 SHA-256을 검사하여 복제 누락이나 임의 변경을 막는다.

## 3. 적용 원칙

### 그대로 가져오는 항목

- `TebLocalPlannerROS` 플러그인 선택
- 속도·가속도 제한과 전진/후진 정책
- 시간 간격과 trajectory 표본 설정
- 목표 허용 오차
- 장애물·동적 장애물 처리 설정
- 최적화 가중치
- homotopy, oscillation recovery 및 costmap converter 설정
- NUC TEB 설정에 존재하는 나머지 planner 파라미터

### GitHub 저장소를 따르는 항목

- TF 체인과 frame 이름
- URDF 및 휠체어 차체 형상
- global/local costmap과 장애물 입력 토픽
- 지도와 localization
- `move_base`의 global planner 및 현재 recovery/safety 정책
- bringup과 하드웨어 권한 구조

NUC TEB의 `line` footprint는 그대로 활성화하지 않는다. 활성 TEB 설정은 GitHub costmap/URDF가 나타내는 휠체어 외형과 일치하는 polygon footprint를 사용한다. NUC의 line footprint 값은 reference 원문에 남아 있으므로 원본은 손실되지 않는다. 이 항목이 의도적인 유일한 기하학적 adaptation이다.

NUC의 구형 `wheel_cmd.py`, UART, MQTT, AMCL, map_server, service 통신 및 모터 권한 경로는 가져오지 않는다.

## 4. 실행 구조

활성 경로는 다음과 같다.

```text
기존 GitHub 지도·localization·TF·URDF·costmap
                    |
                    v
        move_base + Navfn + TEB
                    |
                    v
          cmd_vel 출력 launch arg
```

`navigation.launch`의 출력 토픽 인자 구조는 보존한다.

- 기본 및 통합 bringup: `/cmd_vel_nav`로 출력하여 기존 safety gate가 `/cmd_vel_safe`를 생성한다.
- 로컬 단독 검증: `cmd_vel_nav_topic:=/cmd_vel` 인자를 주어 TEB 결과를 `/cmd_vel`에서 직접 확인할 수 있다.

따라서 로컬 검증 요구를 지원하면서도 브랜치를 실행했다는 이유만으로 실제 모터 명령 경로가 새로 생기지는 않는다.

## 5. 예정 변경

- NUC 원본 YAML 2개와 checksum provenance를 문서용 reference로 추가
- 활성 `teb_local_planner.yaml` 추가
- 활성 `move_base.yaml`에서 local planner 플러그인만 TEB로 변경
- `navigation.launch`가 TEB 설정을 로드하도록 변경
- `package.xml`에 `teb_local_planner`와 `costmap_converter` 실행 의존성 추가
- TEB 원본 동일성, 활성 설정 adaptation, launch graph 및 토픽 경계를 검사하는 정적 테스트 추가/수정

기존 DWA 설정 파일은 삭제하거나 재설계하지 않는다. 단, TEB 브랜치의 활성 launch graph에서는 로드하지 않는다.

## 6. 검증 기준

1. reference 파일 SHA-256이 NUC에서 읽은 값과 일치한다.
2. `base_local_planner`가 `teb_local_planner/TebLocalPlannerROS`이다.
3. 활성 launch가 `teb_local_planner.yaml`만 local planner 설정으로 로드한다.
4. 활성 TEB의 motion/optimization 값이 NUC reference와 일치한다.
5. 활성 footprint가 GitHub 휠체어 형상과 일치하고 NUC line footprint와의 차이가 테스트에 명시된다.
6. 기존 `map`, `odom`, `base_footprint`, `lidar_link` 및 `/perception/obstacle_cloud` 계약이 바뀌지 않는다.
7. 기본 통합 경로는 `/cmd_vel_nav -> safety -> /cmd_vel_safe`를 유지한다.
8. launch arg를 사용하면 로컬에서 `/cmd_vel`로 출력할 수 있다.
9. Pure Pursuit, S자 waypoint follower 및 모터 직접 제어 노드는 TEB launch graph에 없다.
10. 정적 테스트와 가능한 ROS 비의존 테스트가 통과한다.

## 7. 범위 밖 항목

- 실제 승객 탑승 시험
- 실제 모터 enable 또는 NUC 배포
- TEB 재튜닝과 성능 최적화
- DWA 삭제·개선·비교 평가
- TF/URDF/costmap/localization 재설계

## 8. 알려진 기준 환경 사항

원격 `main` 기준 navigation 정적 테스트는 Windows `core.autocrlf=true` 환경에서 지도 YAML의 CRLF 변환 때문에 8개 중 1개가 실패한다. 파일을 LF로 정규화한 SHA-256은 테스트 기대값과 일치한다. 이는 TEB 변경 전부터 존재하는 플랫폼별 기준 실패이며 이 작업에서 지도 파일이나 해시 정책을 수정하지 않는다.
