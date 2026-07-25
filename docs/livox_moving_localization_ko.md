# Livox 이동 로컬라이제이션 운영 런북 (수동 정합 게이트)

이 문서는 `static_livox_localization` 패키지의 보조 정합(assisted alignment) 게이트를
운영자 관점에서 설명한다. 모든 수치와 동작은 아래 파일에서 직접 확인한 것이다.

- `src/static_livox_localization/src/assisted_alignment.cpp`
- `src/static_livox_localization/src/moving_icp_localizer.cpp`
- `src/static_livox_localization/config/moving_localization.yaml`

## 지도는 움직이지 않는다

가장 먼저 이해해야 할 원칙이다. 지도는 고정된 PCD이고 적재 시점에 SHA-256으로 고정된다
(`moving_localization.yaml`의 `map_sha256`). 어떤 보정도 지도 점군을 변환하지 않는다.
보정은 오직 `map_T_odom_` 변환 하나만 갱신한다.

따라서 RViz에서 빨간색 고정 지도가 움직이는 것처럼 보인다면 그것은 지도가 아니라
추정 포즈가 이동한 것이다. 정합이 맞는지는 빨간색 고정 지도와 초록색 실시간 클라우드가
겹치는지로 판단한다.

## 두 개의 상태 기계

혼동하기 쉬우므로 구분한다.

| 구분 | 값 | 의미 |
|---|---|---|
| 정합 상태 | `WAITING_INITIALIZATION`, `MANUAL_ALIGN`, `VERIFYING`, `TRACKING` | 시드와 자동 보정 게이트의 진행 단계 |
| 추적 품질 | `TRACKING`, `DEGRADED`, `LOST` | 정합이 `TRACKING`에 도달한 뒤의 정합 품질 |

`/fast_lio_icp/localization_diagnostics`의 `status.message`는 정합 상태가 `TRACKING`이
아닌 동안에는 정합 상태를 그대로 싣는다. 정합이 `TRACKING`이 된 뒤에야 추적 품질
상태(`DEGRADED`, `LOST`)가 나타날 수 있다.

## 운영 절차

### 1. 시드 투입

RViz의 `2D Pose Estimate`로 `/fast_lio_icp/initialpose`에 포즈를 발행한다.
`frame_id`는 비어 있거나 `map`이어야 하고, 그 외에는 거부된다.

시드는 다음을 **강제로** 수행한다.

- 자동 보정을 끈다 (`auto_correction_enabled_ = false`)
- 정합 상태를 `MANUAL_ALIGN`으로 되돌린다
- 추적 FSM, 경로, 롤링 서브맵을 초기화한다
- ICP를 돌리지 않고 즉시 `map_T_odom_`을 시드 값으로 설정하고 포즈/TF를 발행한다

즉 시드 직후 화면에 보이는 정렬은 순수하게 운영자가 지정한 값이다. 아직 어떤 정합도
수행되지 않았다.

### 2. 육안 정렬 확인

`MANUAL_ALIGN` 상태에서는 클라우드가 롤링 서브맵에 누적될 뿐 **ICP가 전혀 실행되지
않는다**. 이 단계에서 운영자가 해야 할 일은 하나다. 빨간색 고정 지도와 초록색 실시간
클라우드가 충분히 겹치는지 확인하는 것.

`/fast_lio_icp/wheelchair_footprint_marker`가 지름 1.0 m 원통으로 휠체어 발자국을
표시하며, 정합 상태에 따라 색이 바뀐다. 이 원을 지도상의 실제 위치에 맞추는 것이
수동 정합의 목표다.

겹침이 부족하면 3단계로 넘어가지 말고 시드를 다시 투입한다.

### 3. 자동 보정 활성화

```bash
rosservice call /fast_lio_icp/enable_auto_correction "data: true"
```

- 정합 상태가 `WAITING_INITIALIZATION`이면 서비스는 **실패**하고 응답 메시지는
  `INITIAL_POSE_REQUIRED`이다. 시드를 먼저 투입해야 한다.
- 그 외에는 합의 버퍼를 비우고 정합 상태를 `VERIFYING`으로 올린다. 롤링 서브맵도
  비워지고 초기화 창 시계가 다시 시작된다.
- 응답 메시지는 결과 정합 상태 이름이다.

`auto_correction_on_start: false`이므로 노드는 절대 스스로 자동 보정을 켜지 않는다.
이 서비스 호출은 항상 운영자 또는 명시적 헬퍼의 행위다.

### 4. VERIFYING에서 TRACKING으로

`VERIFYING`에서 노드는 `initialization_window_s: 3.0`초를 기다린 뒤 첫 정합을 시도하고,
이후 `correction_period_s: 1.0`초마다 반복한다.

각 후보는 직전 후보와 비교된다. 아래 두 조건을 모두 만족하면 일관된 것으로 센다.

| 파라미터 | 값 |
|---|---|
| `candidate_translation_tolerance_m` | 0.30 |
| `candidate_yaw_tolerance_deg` | 3.0 |
| `required_consistent_candidates` | 3 |

진단의 `reason` 필드로 진행을 읽을 수 있다.

- `CANDIDATE_ACCUMULATING` — 일관된 후보가 쌓이는 중
- `CANDIDATE_INCONSISTENT` — 불일치, 카운터가 1로 초기화됨
- `CONSENSUS_READY` — 합의 완료, 정합 상태가 `TRACKING`으로 올라가고 후보가 그대로 적용됨

`TRACKING` 진입 이후의 보정은 매 스텝 `max_correction_translation_m: 0.20`,
`max_correction_yaw_deg: 2.0`으로 제한된다.

정합이 계속 실패해 추적 FSM이 `lost_after_s: 8.0`초 동안 `LOST`가 되면 노드는
재획득을 시작해 정합 상태를 `VERIFYING`으로 되돌리고 합의 게이트를 다시 통과시킨다.

### 5. 자동 보정 해제

```bash
rosservice call /fast_lio_icp/enable_auto_correction "data: false"
```

항상 성공하며 정합 상태를 `MANUAL_ALIGN`으로 되돌리고 자동 보정을 끈다. 정합이
의심스러울 때 즉시 ICP를 멈추는 수단이다.

## 진단 읽기

`/fast_lio_icp/localization_diagnostics`가 싣는 주요 키.

`raw_state`, `auto_correction_enabled`, `consistent_candidate_count`, `reason`,
`fitness`, `inlier_ratio`, `prediction_translation_m`, `prediction_rotation_rad`,
`source_points`, `target_points`, `reset_count`, `map_id`, `map_sha256`,
`map_frame`, `odom_frame`, `base_frame`.

진단 레벨은 정합 상태가 `TRACKING`이 아니면 WARN, `TRACKING`이면 OK, `LOST`면 ERROR다.

## 좌표 프레임

| 파라미터 | 값 |
|---|---|
| `map_frame` | `map` |
| `odom_frame` | `camera_init` |
| `base_frame` | `body` |

## 자동 시드 경로

`auto_initial_pose.py`는 위 절차를 자동화한다. 후보 시드를 발행하고 1.0초 뒤 자동 보정을
켠 다음, 진단에서 `TRACKING`이 나올 때까지 `--verify-timeout`(기본 20초) 동안 폴링한다.
실패하면 다음 순위 가설을 발행한다. 시드 자체가 자동 보정을 다시 끄기 때문에 각 가설은
`MANUAL_ALIGN` -> `VERIFYING`으로 깨끗하게 재진입한다.

## 기록

`tools/start_wheelchair_localization.sh`의 블랙박스 레코더가 `/fast_lio_icp/initialpose`와
`/fast_lio_icp/localization_diagnostics`를 함께 기록한다. 시드와 정합 상태 전이가 같은
백에 남아야 나중에 시험을 재현하고 판단 근거를 확인할 수 있다.
