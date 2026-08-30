# 정지 위협 단일 정책 우회

대상 브랜치: `codex/person-bypass-feedback-smoothing`

사람과 일반 물체는 `StaticThreatBypassManager` 하나가 우회 허가를 결정한다.
기존 follower의 별도 3초/10초 타이머나 person 전용 permit은 사용하지 않는다.
같은 직접 관측 트랙이 정확히 2.0초 동안 `static`이고 기하 정보가 유효할
때만 `/static_threat_bypass/permit`이 활성화된다. planner, semantic supervisor,
raw gate는 모두 이 permit과 같은 track ID를 사용한다.

## 상태 수명주기

```text
IDLE
  -> QUALIFYING_STATIC       같은 직접 관측 트랙을 0.0~1.999초 확인
  -> BYPASS_COMMITTED        2.0초에 permit 활성화
  -> PASSING_LEFT/RIGHT      DWA가 선택한 우회 방향을 한 번 고정
  -> CLEARING                대상이 차체 뒤에 있고 선택 궤적의 tail이 clear
  -> RESUME                  tail clear 3회 연속 확인 후 원 경로로 복귀
```

commit 뒤 최종 명령이 잠시 0이 되어도 선택한 좌/우 방향은 바뀌지 않는다.
한 번의 perception 누락은 permit 수명 안에서만 유지된다. 두 번째 누락,
track 변경, 움직임 확인 또는 절대 안전 veto는 즉시 0 명령을 유지한다.

## 우회 가능 조건과 절대 정지 조건

| 입력/상태 | 2초 후 동작 | 분류 |
|---|---:|---|
| 직접 관측된 정지 사람 1명 | 안전한 곡선 제안으로 우회 | 우회 가능 |
| 직접 관측된 정지 물체 1개 | 안전한 곡선 제안으로 우회 | 우회 가능 |
| 기존 직선 corridor의 `OBSTACLE`/`OBSTACLE_SWEEP` | 동일 permit·proposal의 실제 곡선이 clear일 때만 대체 | 조건부 우회 |
| moving 또는 unknown | 정지 | 절대 veto |
| learned-only, geometry invalid, stale/malformed 관측 | 정지 | 절대 veto |
| track 변경 또는 두 번째 동적 위협 | 정지 | 절대 veto |
| localization 비정상, perception/odom stale | 정지 | 절대 veto |
| 현재 footprint 충돌 | 정지 | 절대 veto |
| 현재 관성 궤적 충돌 | 정지 | 절대 veto |
| 선택 proposal 충돌·stale·ID/명령 불일치 | 정지 | 절대 veto |
| drivable mask, terrain/cliff, tip, UART fault | 정지 | 절대 veto |

우회 허가는 센서 점을 지우지 않는다. DWA가 평가한 actuator-ramped proposal을
raw gate가 같은 collision snapshot으로 검사하며, semantic/raw/terrain/tip 중
하나라도 절대 veto를 내면 최종 명령은 0이다. tail clear 3회 뒤에는 일반
route policy가 `CLEAR`를 반환하므로 permit 없이 원 경로의 비영점 명령을 다시
낼 수 있다.

## strict trajectory proposal v2

`/static_threat_bypass/proposal`은
`wheelchair.trajectory_proposal/v2`만 허용한다. 좌표계는 proposal 생성 시점의
`current_body`이고, `distance_m`, `latency_s`, 각 sample의 `time_steps_s`를
반드시 포함한다. poses/speeds/yaw-rates/time-steps는 `RolloutSpec`과
`rollout_actuation_timed()`로 재생성한 canonical 결과와 정확히 일치해야 한다.
임의 sample, 누락 필드, horizon 불일치 또는 한 점이라도 변조된 payload는
`ProposalValidationError`로 폐기된다.

정지 상태에서는 첫 적용 yaw가 `0.0`일 수 있다. raw gate는 수신 명령을 첫
적용 `v,w`와 대조하지만, 좌/우 commit과 최소 회전 의도는 proposal의
`target_yaw_rate_rps`로 판정한다. 따라서 첫 `w=0`이라는 이유만으로 안전한
곡선 시작을 영구 차단하지 않는다. 우회 `Twist.angular.x`에는 단조 증가하는
`proposal_seq`가 실리고 semantic supervisor가 이를 그대로 `/cmd_vel_raw`까지
전달한다. bounded proposal buffer는 callback 순서와 무관하게 raw 명령과 같은
sequence의 proposal만 고른다. stale/replay, sequence·track·side·command
불일치는 계속 정지한다. safety gate는 새 출력의 `linear.x`와 `angular.z`만
채우므로 이 식별자는 `/cmd_vel_gated` 이후 구동 계층으로 전달되지 않는다.

## 진단 토픽과 필드

| 토픽 | 확인할 값 |
|---|---|
| `/static_threat_bypass/permit` | `active`, `track_id`, `threat_label`, `static_for_s`, `reason` |
| `/static_threat_bypass/proposal` | v2 schema, `current_body`, `proposal_seq`, `distance_m`, `latency_s`, `time_steps_s`, target/applied `v,w` |
| `/static_threat_bypass/status` | lifecycle, committed side, planner timing |
| `/semantic_safety/status` | permit validation, dynamic conflict |
| `/safety_gate/status` | base reason, proposal match, collision snapshot, tail clear |
| `/terrain_guard/status`, `/tip_guard/status` | downstream absolute veto |
| `/cmd_vel_planned` -> `/cmd_vel_raw` -> `/cmd_vel_gated` -> `/cmd_vel_terrain_safe` -> `/cmd_vel` | 최초 0 전환 단계 |

읽기 전용 준비 상태 명령은 다음과 같다.

```bash
bash tools/hybrid.sh static-threat-bypass-status
```

## ROS 없는 호스트 검증

전체 pytest/compile 및 generic status 명령 노출 확인:

```bash
bash tools/test_static_threat_bypass.sh host
# STATIC_THREAT_HOST_TEST_PASS
```

생산 policy/proposal/gate/route API를 이용한 결정론적 JSONL 수명주기 QA:

```bash
bash tools/test_static_threat_bypass.sh qa
# STATIC_THREAT_HOST_QA_PASS
```

QA는 strict v2 canonical rollout, 정지 첫 `w=0`에서 target turn 허용, payload
변조 거부, tail release 뒤 일반 route의 `0.35 m/s` 복귀까지 확인한다.

두 명령은 ROS master에 연결하지 않고 노드를 시작·종료하거나 코드를 배포하지
않는다. 현재 결과는 **호스트에서만 검증됨**이다. NUC 배포, 바퀴 구동 또는
실차 주행은 이번 변경에서 실행하지 않았다. 실차 전에는 수동 모드, 구동륜
리프트, 조이스틱 즉시 개입, terrain/cliff veto 확인 순서로 별도 검증해야 한다.

이미 실행 중인 그래프를 변경하지 않고 상태만 확인할 때만 다음을 사용한다.

```bash
bash tools/test_static_threat_bypass.sh live-check
```
