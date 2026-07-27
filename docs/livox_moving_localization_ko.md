# Livox 이동 로컬라이제이션 운영 런북 (수동 정합 게이트)

이 문서는 `static_livox_localization` 패키지의 보조 정합(assisted alignment) 게이트를
운영자 관점에서 설명한다. 모든 수치와 동작은 아래 파일에서 직접 확인한 것이다.

- `src/static_livox_localization/src/assisted_alignment.cpp`
- `src/static_livox_localization/src/moving_icp_localizer.cpp`
- `src/static_livox_localization/scripts/waypoint_follower.py`
- `src/static_livox_localization/config/moving_localization.yaml`

---

## 정합에 개입하면 휠체어가 선다

`waypoint_follower.py`는 정합 상태가 **`TRACKING`일 때만** 주행한다. 판정은
`localization_policy.py`의 `localization_hold_reason`이 전담한다.

| `status.message` | 판정 |
|---|---|
| `TRACKING` | 주행 |
| `DEGRADED`, 지속 시간이 `DEGRADED_STOP_S` 이내 | 주행 (유예) |
| `DEGRADED`, 유예 초과 | 정지 `LOCALIZATION_DEGRADED_TIMEOUT` |
| `LOST` | 정지 `LOCALIZATION_LOST` |
| `MANUAL_ALIGN`, `VERIFYING`, `WAITING_INITIALIZATION`, 초기 빈 값, 그 외 전부 | 정지 `LOCALIZATION_NOT_TRACKING` |

따라서 주행 중에 시드를 다시 투입하거나 자동 보정을 끄면 **휠체어가 스스로 선다.**
정합을 되찾아 `TRACKING`으로 복귀하면 자동으로 다시 출발한다. 래치되지 않으므로
운영자가 따로 풀어줄 것은 없다.

그래도 개입 전에 명시적으로 세우는 습관은 유지한다. 정지 이유가 하나로 좁혀져 로그가
읽기 쉬워지고, 의도한 정지와 시스템이 건 정지를 구분할 수 있다.

```bash
rosservice call /waypoint_follower/start "data: false"   # 주행 일시정지
# E-stop: 조이스틱을 수동 모드로. 또는:
rostopic pub -1 /mode_cmd std_msgs/Int16 77
```

### 이 규칙이 막지 못하는 것

판정은 마지막으로 수신한 진단 문자열 하나에만 의존한다. **진단의 신선도는 검사하지
않는다.** 로컬라이저가 `TRACKING`을 마지막으로 발행한 뒤 죽거나 조용해지면, follower는
그 허가를 계속 들고 주행한다. `odom_callback`이 얼어붙은 `map_T_odom_`으로 포즈를 계속
발행하므로 `NO_POSE`도 트립되지 않는다.

또한 `CLOUD_ODOMETRY_TIME_MISMATCH`, `CLOUD_REJECTED`, `INSUFFICIENT_ROLLING_SUBMAP`
같은 조기 return 경로는 **보정을 평가하지 않고도 `TRACKING`을 그대로 발행한다.**

그러므로 주행 중에는 진단이 **계속 갱신되고 있는지** 눈으로 확인한다. 값이 멈춰 있으면
`TRACKING`이라도 신뢰하지 않는다.

```bash
rostopic hz /fast_lio_icp/localization_diagnostics
```

미해결 이슈 #41이다.

---

## 현장 기본 경로는 자동이다

`tools/start_wheelchair_localization.sh:144`는 다음으로 기동한다.

```
roslaunch static_livox_localization moving_localization.launch auto_init:=true ...
```

`auto_init:=true`는 `auto_initial_pose.py`를 띄우고, 이 노드가 **운영자 조작 없이**
시드를 발행하고 자동 보정을 켠다. 즉 아래 1~3절의 수동 절차는 **평상시에 일어나지 않는다.**

자동 경로는 사람의 육안 확인을 자동화한 것이 아니라 **점수 임계값으로 대체**한 것이다.

| 인자 | 기본값 | 의미 |
|---|---|---|
| `--min-score` | 0.25 | KD-tree inlier 점수 하한. 미만이면 가설 기각 |
| `--top` | 4 | 상위 몇 개 가설까지 시도할지 |
| `--retries` | 2 | 재시도 횟수 |
| `--verify-timeout` | 20.0 | `TRACKING` 확인 대기 시간(초) |
| `--inlier-radius` | 0.45 | inlier 판정 반경(m) |

1~3절의 수동 절차는 자동 초기화가 실패해 스크립트가
`WARNING: not TRACKING yet`을 출력했을 때 쓰는 **대체 경로**다.

---

## 지도는 움직이지 않는다

지도는 고정된 PCD이고 적재 시점에 SHA-256으로 검증된다
(`moving_localization.yaml`의 `map_sha256`). 어떤 보정도 지도 점군을 변환하지 않는다.
보정은 오직 `map_T_odom_` 변환 하나만 갱신한다.

따라서 RViz에서 빨간색 고정 지도가 움직이는 것처럼 보인다면 그것은 지도가 아니라
추정 포즈가 이동한 것이다. 정합이 맞는지는 빨간색 고정 지도와 초록색 실시간 클라우드가
겹치는지로 판단한다.

참고로 `map_sha256`이 맞지 않으면 노드는 종료 코드 2로 죽고
(`moving_icp_localizer.cpp:253-254`), `moving_localization.launch:13`이 `required="true"`라
launch 전체가 내려간다. 기동 직후 아무것도 뜨지 않으면 이 경우를 먼저 의심한다.

## 두 개의 상태 기계

혼동하기 쉬우므로 구분한다.

| 구분 | 값 | 의미 |
|---|---|---|
| 정합 상태 | `WAITING_INITIALIZATION`, `MANUAL_ALIGN`, `VERIFYING`, `TRACKING` | 시드와 자동 보정 게이트의 진행 단계 |
| 추적 품질 | `TRACKING`, `DEGRADED`, `LOST` | 정합이 `TRACKING`에 도달한 뒤의 정합 품질 |

`/fast_lio_icp/localization_diagnostics`의 `status.message`는 정합 상태가 `TRACKING`이
아닌 동안에는 정합 상태를 그대로 싣는다. 정합이 `TRACKING`이 된 뒤에야 추적 품질
상태(`DEGRADED`, `LOST`)가 나타날 수 있다. `waypoint_follower`는 이 하나의 문자열만 보므로,
위 경고 절의 이야기가 여기서 나온다.

## 운영 절차 (주행 정지 상태에서만)

### 1. 시드 투입

RViz의 `2D Pose Estimate`로 `/fast_lio_icp/initialpose`에 포즈를 발행한다.
`frame_id`는 비어 있거나 `map`이어야 하고, 그 외에는 거부된다.

시드는 다음을 **강제로** 수행한다.

- 자동 보정을 끈다 (`auto_correction_enabled_ = false`)
- 정합 상태를 `MANUAL_ALIGN`으로 되돌린다
- 추적 FSM, 경로, 롤링 서브맵을 초기화한다
- ICP를 돌리지 않고 즉시 `map_T_odom_`을 시드 값으로 설정하고 포즈/TF를 발행한다

마지막 항목은 `has_latest_odom_`이 참일 때만 일어난다
(`moving_icp_localizer.cpp:304-311`). FAST-LIO 오도메트리가 아직 올라오지 않았다면 시드를
넣어도 아무것도 보이지 않는다. 첫 `/Odometry` 메시지를 기다린다.

시드 직후 화면에 보이는 정렬은 순수하게 운영자가 지정한 값이다. 아직 어떤 정합도
수행되지 않았다.

### 2. 육안 정렬 확인

`MANUAL_ALIGN` 상태에서는 클라우드가 롤링 서브맵에 누적될 뿐 **ICP가 전혀 실행되지
않는다.** 이 단계에서 확인할 것은 하나다. 빨간색 고정 지도와 초록색 실시간 클라우드가
충분히 겹치는지.

`/fast_lio_icp/wheelchair_footprint_marker`가 지름 1.0 m 원통으로 휠체어 발자국을
표시하며, 정합 상태에 따라 색이 바뀐다.

겹침이 부족하면 3절로 넘어가지 말고 시드를 다시 투입한다. 재시드는 포즈를 클릭한 위치로
불연속 점프시키고 추적 FSM을 초기화하며, 정합 상태를 `MANUAL_ALIGN`으로 되돌리므로
follower는 그 시점에 정지한다.

### 3. 자동 보정 활성화

```bash
rosservice call /fast_lio_icp/enable_auto_correction "data: true"
```

- 정합 상태가 `WAITING_INITIALIZATION`이면 서비스는 **실패**하고 응답 메시지는
  `INITIAL_POSE_REQUIRED`이다. 시드를 먼저 투입해야 한다.
- 그 외에는 합의 버퍼를 비우고 정합 상태를 `VERIFYING`으로 올린다. 롤링 서브맵도
  비워지고 초기화 창 시계가 다시 시작된다.
- **이미 `TRACKING`인 상태에서 호출해도 마찬가지다.** 경고 없이 `VERIFYING`으로
  내려가고 서브맵이 비워진다. 주행 중에는 호출하지 않는다.

노드가 스스로 자동 보정을 켜는 경로는 없다. `auto_correction_enabled_`의 기본값이
`false`이고 이 서비스 외에는 값을 바꾸는 코드가 없기 때문이다.

> `moving_localization.yaml`의 `auto_correction_on_start`는 현재 **동작하지 않는
> 파라미터**다. `moving_icp_localizer.cpp:219`에서 읽어 `:602`에 저장하지만 그 값을
> 사용하는 코드가 없다. `true`로 바꿔도 아무 일도 일어나지 않는다.

### 4. VERIFYING에서 TRACKING으로

`VERIFYING`에서 노드는 `initialization_window_s: 3.0`초를 기다린 뒤 첫 정합을 시도하고,
이후 `correction_period_s: 1.0`초마다 반복한다. 단 이 3초 대기는 **서비스로 켠 경로에만**
적용된다. `LOST` 이후 자동 재획득은 `last_correction_stamp_s_`를 초기화하지 않으므로
첫 후보가 1.0초 뒤에 나온다.

각 후보는 직전 후보와 비교된다. 비교는 **평면 기준**이다
(`assisted_alignment.cpp:53-57`은 `hypot(x, y)`와 yaw만 본다). z축만 어긋난 경우는
일관된 것으로 계산된다.

| 파라미터 | 값 |
|---|---|
| `candidate_translation_tolerance_m` | 0.30 |
| `candidate_yaw_tolerance_deg` | 3.0 |
| `required_consistent_candidates` | 3 |

진단의 `reason` 필드로 진행을 읽는다.

- `CANDIDATE_ACCUMULATING` — 일관된 후보가 쌓이는 중
- `CANDIDATE_INCONSISTENT` — 불일치. 카운터가 **1로** 초기화
- `CONSENSUS_READY` — 합의 완료. 정합 상태가 `TRACKING`으로 올라감

정합 자체가 **기각**된 경우는 다르다. `observe_rejection()`이 합의를 **0으로** 지우고,
아직 `TRACKING` 전이면 `VERIFYING`으로 되돌린다.

`VERIFYING`이 멈춘 것처럼 보일 때 실제로 자주 만나는 `reason`은 위 세 개가 아니라
다음이다. 진단을 직접 확인한다.

- `CLOUD_ODOMETRY_TIME_MISMATCH`, `INSUFFICIENT_ROLLING_SUBMAP`, `CLOUD_REJECTED`
- `WAITING_FOR_INITIALPOSE`, `ODOMETRY_FRAME_MISMATCH`
- 정합 평가 기각: `NOT_CONVERGED`, `HIGH_FITNESS`, `INSUFFICIENT_SOURCE_POINTS`,
  `INSUFFICIENT_TARGET_POINTS`, `LOW_INLIER_RATIO`, `PREDICTION_TRANSLATION_JUMP`,
  `PREDICTION_ROTATION_JUMP`

### 포즈가 한 번에 얼마나 튈 수 있는가

이 절이 안전 판단의 핵심이다. 보정 한계는 **경로에 따라 다르다.**

| 경로 | 한계 | 근거 |
|---|---|---|
| `TRACKING` 정상 보정 | 스텝당 0.20 m / 2.0° | `max_correction_translation_m`, `max_correction_yaw_deg` |
| `CONSENSUS_READY` 스냅 | **클램프 없음** | `moving_icp_localizer.cpp:447-448`이 후보를 그대로 대입 |
| `VERIFYING` 중 후보 수용 폭 | **3.0 m / 30°** | `registration_max_seed_translation_m`, `registration_max_seed_rotation_deg` |

세 번째 줄의 두 값은 **`moving_localization.yaml`에 없다.** `moving_icp_localizer.cpp:189-195`의
C++ 기본값 3.0 m / 30°가 현장에서 그대로 동작한다.

그리고 `moving_icp_localizer.cpp:465-472`에 따라 `LOST` 이후 재획득은 `VERIFYING`으로
돌아가므로, 이것은 기동 시에만 해당하는 이야기가 아니다. **주행 중 8초 상실 후 재획득에서
포즈가 한 번에 최대 3 m 이동할 수 있다.** `TRACKING` 진입 후 0.20 m 제한만 보고 판단하면
안 된다.

### 5. 자동 보정 해제

```bash
rosservice call /waypoint_follower/start "data: false"        # 반드시 먼저
rosservice call /fast_lio_icp/enable_auto_correction "data: false"
```

두 번째 명령은 항상 성공하며 정합 상태를 `MANUAL_ALIGN`으로 되돌리고 자동 보정을 끈다.

정합 상태가 `TRACKING`을 벗어나므로 **follower는 이 시점에 `LOCALIZATION_NOT_TRACKING`으로
스스로 선다.** 첫 명령으로 먼저 세우는 것은 의도한 정지와 시스템이 건 정지를 로그에서
구분하기 위해서다.

## 진단 읽기

`/fast_lio_icp/localization_diagnostics`가 싣는 주요 키.

`raw_state`, `auto_correction_enabled`, `consistent_candidate_count`, `reason`,
`fitness`, `inlier_ratio`, `prediction_translation_m`, `prediction_rotation_rad`,
`source_points`, `target_points`, `reset_count`, `map_id`, `map_sha256`,
`map_frame`, `odom_frame`, `base_frame`.

진단 레벨은 다음 순서로 결정된다.

| 조건 | 레벨 | `status.message` |
|---|---|---|
| 정합 상태가 `TRACKING`이 아님 | WARN | 정합 상태 이름 |
| 정합과 추적이 모두 `TRACKING` | OK | `TRACKING` |
| 추적이 `LOST` | ERROR | `LOST` |
| 그 외, 즉 `DEGRADED` | WARN | `DEGRADED` |

WARN만 보고는 정합 문제인지 추적 문제인지 알 수 없다. `status.message`를 봐야 한다.

## 좌표 프레임

| 파라미터 | 값 |
|---|---|
| `map_frame` | `map` |
| `odom_frame` | `camera_init` |
| `base_frame` | `body` |

## 기록

`tools/start_wheelchair_localization.sh`의 블랙박스 레코더가 `/fast_lio_icp/initialpose`와
`/fast_lio_icp/localization_diagnostics`를 함께 기록한다. 시드와 정합 상태 전이가 같은
백에 남아야 나중에 시험을 재현하고 판단 근거를 확인할 수 있다.
