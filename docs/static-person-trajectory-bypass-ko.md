# 정지 사람·오브젝트 궤적 우회 브랜치

브랜치: `codex/person-bypass-reliability`

이 브랜치는 기존 하이브리드 스택의 두 교착을 함께 고친다.

1. `person` 클래스는 `STATIC`이어도 semantic supervisor와 follower에서 항상 정지했다.
2. DWA가 곡선 우회 궤적을 만들더라도 raw `safety_gate`는 차체 앞의 고정 직선 박스 안에 5개 점이 있으면 다시 0 명령으로 만들었다.

## 최종 정책

| 상황 | 동작 |
|---|---|
| 일반 정지 장애물 | 기존 RTX/CuPy DWA로 우회 |
| 직접 관측된 유효 트랙의 정지 오브젝트 | 짧은 trajectory permit 아래 RTX/CuPy DWA로 우회 |
| 움직이거나 motion이 unknown인 오브젝트 | 정지하고 재관측 |
| 동일 트랙의 정지 사람 1명 | 3초 연속 직접 관측 후 DWA 우회 |
| 움직이는 사람 | 정지하고 재관측 |
| motion이 unknown인 사람 | 정지 |
| learned-only 사람 | 정지 |
| 두 명 이상이 우회 영역에 있음 | 정지 |
| 사람 트랙 ID 변경·포즈 점프·프레임 누락 | 우회 허가 즉시 폐기 |
| 사람이 너무 가까움 | 정지 |
| DWA 곡선 swept footprint가 raw point와 충돌 | 정지 |
| 현재 관성 궤적이 충돌 | 완전히 멈출 때까지 정지 |
| 인도 mask/band를 벗어나는 우회 | `terrain_guard`에서 정지 |

기본값은 다음과 같다.

```text
정지 사람 추가 확인:       3.0 s
허용 관측 간격:            0.45 s
같은 트랙 위치 점프:       0.35 m 이하
같은 트랙 측면 해제 여유:   0.25 m
permit 수명:               0.45 s
최대 우회 속도:            0.35 m/s
사람 최소 중심선 clearance: 0.80 m
최소 실제 회전 명령:       0.08 rad/s
```

트래커가 `STATIC`이라고 한 번 출력했다는 이유만으로 출발하지 않는다. 위 3초는 트래커의 자체 정지 판정 이후에 같은 ID와 위치가 연속으로 유지되는 추가 확인 시간이다.

## raw safety gate 변경 범위

기존 raw gate의 아래 사유는 그대로 유지된다.

```text
INPUT_STALE
CLOUD_STALE
INPUT_INVALID
REVERSE
NO_CLOUD
ODOM_STALE
OBSTACLE_SWEEP
```

예외가 가능한 것은 오직 기존의 고정 직선 corridor가 만든 `OBSTACLE` 하나다. 그것도 다음이 모두 참일 때만 해제된다.

```text
fresh static-person permit
+ 실제 회전 명령
+ 현재 확대 footprint 안에 점이 없음
+ 현재 바퀴가 이미 가지고 있는 관성 경로가 안전함
+ 요청된 DWA 곡선 swept footprint가 모든 raw point를 피함
```

즉 raw LiDAR를 끄거나 사람 점을 지우는 방식이 아니다. 고정 직선 박스 대신 현재 DWA가 요청한 곡선 자체를 검사한다.

## 로컬 무구동 검증

다른 작업자의 checkout이나 ROS 그래프를 건드리지 않는 기본 시험은 전용
worktree에서 실행한다.

```bash
cd /Users/minjun/.codex/worktrees/unicon-wheelchair-person-bypass-reliability
bash tools/test_static_threat_bypass.sh host
```

성공하면 마지막 줄에 다음이 출력된다.

```text
STATIC_THREAT_HOST_TEST_PASS
```

이미 NUC에서 스택을 올린 뒤 노드를 교체하거나 명령을 발행하지 않고 현재
그래프의 capability와 permit heartbeat만 확인하려면 다음을 실행한다.

```bash
bash tools/test_static_threat_bypass.sh live-check
```

`host`와 `live-check` 어느 쪽도 `hybrid.sh start` 또는 `hybrid.sh go`를
호출하지 않는다.

## 원격 브랜치가 준비된 뒤 NUC에 배포

현재 로컬 전용 브랜치를 임의로 원격에 올리거나 NUC checkout을 바꾸지 않는다.
팀 작업과 분리된 `codex/person-bypass-reliability` 원격 브랜치가 명시적으로
준비된 뒤에만 아래 배포 절차를 사용한다.

휠체어를 수동 모드로 정지한 상태에서:

```bash
cd ~/wheelchair_localization_src

git fetch --all --prune
git switch codex/person-bypass-reliability
git pull --ff-only origin codex/person-bypass-reliability
```

기존 NUC remote 이름이 `github`이면 `origin` 대신 `github`를 사용한다.

실행 워크스페이스 반영:

```bash
rsync -a --delete \
  ~/wheelchair_localization_src/src/static_livox_localization/ \
  ~/livox_static_localization_ws/src/static_livox_localization/

cd ~/livox_static_localization_ws
source /opt/ros/noetic/setup.bash
catkin build static_livox_localization
```

## 시작

```bash
cd ~/wheelchair_localization_src
bash tools/hybrid.sh start
```

`start`는 기존 하이브리드 스택을 정지 상태로 올린 다음 아래 세 노드를 동일한 공식 node name으로 교체한다.

```text
person_bypass_dwa_follower.py         -> /waypoint_follower
person_bypass_semantic_supervisor.py -> /semantic_safety_supervisor
trajectory_safety_gate.py            -> /safety_gate
```

새 구현 확인:

```bash
bash tools/hybrid.sh person-bypass-status
bash tools/hybrid.sh gpu-status
```

정상 출력에는 다음이 있어야 한다.

```text
PERSON_BYPASS_PREFLIGHT_OK
```

토픽 직접 확인:

```bash
rostopic echo /person_bypass/permit
rostopic echo /semantic_safety/status
rostopic echo /safety_gate/status
rostopic echo /waypoint_follower/status
```

정지 사람을 둔 시험에서는 permit이 대략 다음 순서로 바뀐다.

```text
PERSON_NOT_CONFIRMED_STATIC
QUALIFYING_STATIC_PERSON
STATIC_PERSON_BYPASS (active=true)
```

정적 오브젝트를 둔 시험에서는 `STATIC_OBJECT_BYPASS (active=true)`를
확인한다. 같은 오브젝트가 `moving` 또는 `unknown`으로 바뀌면 permit은 즉시
비활성화되어야 한다.

## 시험 순서

1. 모터 전원 차단 또는 구동륜을 띄운다.
2. 사람이 걸어 들어올 때 `active=false`, `/cmd_vel_raw=0`인지 확인한다.
3. 사람이 정지한 뒤 같은 ID가 유지되는지 확인한다.
4. 3초 후 `active=true`와 `/cmd_vel_planned`의 곡선 명령을 확인한다.
5. `/safety_gate/status`에서 `trajectory_override_allowed=true`인지 확인한다.
6. 직선 명령, 너무 가까운 사람, 두 번째 사람을 넣으면 즉시 다시 정지하는지 확인한다.
7. 빈 휠체어, 0.35 m/s, 조이스틱 즉시 개입 상태로 실차 시험한다.
8. 인도 경계 쪽 우회가 `/terrain_guard/status`에서 차단되는지 확인한다.

## 조정

첫 실차 시험에서 확인 시간을 바꾸려면:

```bash
PERSON_BYPASS_CONFIRM_S=5.0 bash tools/hybrid.sh start
```

속도와 clearance를 줄여서 더 공격적으로 만들지 말고, 실측 후 필요하면 확인 시간을 늘리는 방향부터 조정한다.
