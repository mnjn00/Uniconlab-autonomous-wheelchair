# Stop Intent and DWA Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** NUC에서 검증된 측정 속도 기반 정지 램프를 로컬 소스에 반영하고, 정지 의도가 실제 바퀴에서 지켜지지 않는 상태를 분류·감시하며, DWA가 좁은 길을 거절하는 정확한 이유를 진단할 수 있게 한다.

**Architecture:** 하드웨어 출력층은 `/wheel_status`의 실측 바퀴 속도를 이용해 정지 램프를 만든다. `stop_watchdog`는 `/cmd_vel_gated`를 정지 의도, `/cmd_vel`을 최종 명령, `/wheel_status`를 실제 결과로 관찰하는 순수 상태 머신을 두고 이상 유형을 구분한다. DWA는 기존 판정과 반환 형식을 유지하면서 대역·마스크·장애물 통과 후보 수만 기록한다.

**Tech Stack:** ROS1 Python, NumPy, pytest/unittest, rosbag 회귀 픽스처, Git

**Spec:** `docs/superpowers/specs/2026-08-24-stop-intent-and-dwa-diagnostics-design.md`

## Global Constraints

- GitHub push와 NUC 배포·재시작은 하지 않는다. 검증된 커밋은 로컬 `main`에만 fast-forward한다.
- 기존 DWA 안전 거리 `0.40 m`, SafetyBand, RouteMask 허용치는 변경하지 않는다.
- 사람 WAIT, WAIT 히스테리시스, 저속 yaw 제약, 현장 주행 검증은 범위 밖이다.
- 각 생산 코드 변경은 먼저 실패하는 행동 기반 테스트를 추가한 뒤 최소 구현으로 통과시킨다.

## Task 1: 측정 속도 기반 정지 램프를 로컬 기준 소스로 반영

**Files:**

- Modify: `tests/test_wheel_cmd_guard.py`
- Modify: `tools/base_model_wheel_cmd_guard.py`

- [ ] 기존 바퀴 명령 디코딩 헬퍼를 재사용해 정지 진입 즉시 `S/S`가 아닌 실측 속도 기반 `C/C` 감속을 검증하는 테스트를 추가한다.
- [ ] 신선한 `/wheel_status`가 있으면 각 바퀴가 `0.9` 비율 감속과 `0.09 m/s` 최소 감속량을 따르는지, `0.06 m/s` 이하에서 `S`로 끄는지 테스트한다.
- [ ] 상태가 `0.30 s` 이상 낡았거나 형식이 잘못되면 즉시 `S/S`로 fail-safe하는 테스트를 추가한다.
- [ ] Run RED: `pytest -q tests/test_wheel_cmd_guard.py`
- [ ] `MeasuredStopRamp`와 상수 `RAMP_DECAY=0.9`, `RAMP_BLEED_MPS=0.09`, `RAMP_TERMINAL_MPS=0.06`, `STATUS_FRESH_S=0.30`을 최소 구현하고 기존 운전 명령 보호를 유지한다.
- [ ] Run GREEN: `pytest -q tests/test_wheel_cmd_guard.py`
- [ ] 램프 적용을 임시로 제거했을 때 신규 테스트가 실패하는지 확인한다.
- [ ] Commit: `fix: ramp stops from measured wheel speed`

## Task 2: 정지 의도·최종 명령·바퀴 결과 상태 머신을 TDD로 구현

**Files:**

- Modify: `src/static_livox_localization/test/test_stop_watchdog.py`
- Modify: `src/static_livox_localization/scripts/stop_watchdog.py`

- [ ] `observe_gated_command(linear, angular, now_s)`가 0 명령으로 정지 의도를 무장하고 운전 명령으로 해제하는 테스트를 작성한다.
- [ ] `observe_final_command(...)`가 최종 0 명령 시간을 따로 추적해 tip guard 감속 구간을 오탐하지 않는 테스트를 작성한다.
- [ ] 한 바퀴가 `<=0.15 m/s`, 다른 바퀴가 `>=0.30 m/s`, 추정 yaw가 `>=0.50 rad/s`인 상태가 `0.15 s` 지속되면 `ONE_WHEEL_PIVOT`을 한 번만 반환하는 테스트를 작성한다.
- [ ] 정지 의도 시작 속도가 `>0.15 m/s`인데 `0.50 s` 후 `0.05 m/s` 이상 줄지 않으면 `NOT_SLOWING`을 반환하는 테스트를 작성한다.
- [ ] 최종 0 명령 후 `0.40 s`가 지났는데 어느 바퀴든 `>0.15 m/s`이면 `STOP_NOT_HONOURED`를 반환하는 테스트를 작성한다.
- [ ] 수동 모드, 낡은 상태, 무장 전 상태, 이미 알람한 상태에서 오탐하지 않는 테스트를 작성한다.
- [ ] Run RED: `pytest -q src/static_livox_localization/test/test_stop_watchdog.py`
- [ ] 검사가 반환하는 fault 딕셔너리에 `code`, `reason`, `left_mps`, `right_mps`, `pivot_yaw_rps`, `intent_age_s`, `gated_age_s`, `final_age_s`를 담는 순수 상태 머신을 최소 구현한다.
- [ ] 이상 우선순위를 `ONE_WHEEL_PIVOT` → `NOT_SLOWING` → `STOP_NOT_HONOURED`로 검사하고 무장 해제 전까지 재발행하지 않는다.
- [ ] Run GREEN: `pytest -q src/static_livox_localization/test/test_stop_watchdog.py`
- [ ] 각 fault 분기를 임시로 무효화했을 때 해당 테스트가 실패하는지 확인한다.

## Task 3: ROS 감시 노드를 새 상태 머신에 연결하고 mode 77 알람을 구조화

**Files:**

- Modify: `src/static_livox_localization/test/test_stop_watchdog.py`
- Modify: `src/static_livox_localization/scripts/stop_watchdog.py`

- [ ] ROS stub을 이용해 `/cmd_vel_gated` 콜백이 정지 의도를, `/cmd_vel` 콜백이 최종 명령을 상태 머신에 전달하는 행동 테스트를 작성한다.
- [ ] fault 발생 시 `stop_watchdog/alarm`에 구조화된 JSON을 보내고 `/mode_cmd=77`을 단 한 번 발행하는 행동 테스트를 작성한다.
- [ ] Run RED: `pytest -q src/static_livox_localization/test/test_stop_watchdog.py`
- [ ] `geometry_msgs/Twist` 구독을 추가하고 기존 `/wheel_cmd` 기반 정지 검사를 제거한다. `/wheel_status`, `/uart_tx_diag`와 auto/manual 처리는 유지한다.
- [ ] 알람에 탐지 코드와 실측값·시간을 담은 뒤 mode 77을 발행하고 랙치한다.
- [ ] Run GREEN: `pytest -q src/static_livox_localization/test/test_stop_watchdog.py`
- [ ] Commit: `fix: monitor stop intent and asymmetric wheel motion`

## Task 4: DWA 거절 단계별 후보 수를 기록하고 WP1216을 회귀 픽스처로 고정

**Files:**

- Modify: `tests/test_dwa_band.py`
- Modify: `src/static_livox_localization/test/test_dwa_policy.py`
- Modify: `src/static_livox_localization/scripts/dwa_core.py`
- Modify: `src/static_livox_localization/scripts/dwa_follower.py`

- [ ] 플래너 호출 후 `last_diagnostics`에 `total`, `band_ok`, `mask_ok`, `geometry_ok`, `obstacle_ok`, `all_ok`, `max_clearance_m`가 남는 테스트를 작성한다.
- [ ] 최신 bag WP1216 픽스처(자체 상태, SafetyBand, RouteMask, 장애물 포인트)에서 `all_ok > 0`이고 현재 결과가 `OK`인 테스트를 작성한다.
- [ ] 실제로 지나갈 수 없는 벽 픽스처에서 `all_ok == 0`과 `OBSTACLE`이 유지되는 테스트를 작성한다.
- [ ] follower가 HOLD할 때만 `DWA_<status>` 뒤에 단계별 후보 수를 포함하는 행동 테스트를 작성한다.
- [ ] Run RED: `pytest -q tests/test_dwa_band.py src/static_livox_localization/test/test_dwa_policy.py`
- [ ] 기존 `plan()` 튜플과 판정을 바꾸지 않고 band, mask, geometry, obstacle 불리 mask를 계산해 `last_diagnostics`만 갱신한다.
- [ ] follower의 HOLD 상태 문자열에 진단값을 안정적인 키 순서로 추가한다.
- [ ] Run GREEN: `pytest -q tests/test_dwa_band.py src/static_livox_localization/test/test_dwa_policy.py`
- [ ] 진단 계산을 제거하면 신규 테스트가 실패하고, 기존 플래너 결과 테스트는 계속 통과하는지 확인한다.
- [ ] Commit: `feat: expose DWA rejection diagnostics`

## Task 5: 로컬 통합 검증과 main fast-forward

**Files:**

- Modify if needed: `docs/nuc_snapshot/base_model_which_copy_runs.md`
- Verify: all changed production and test files

- [ ] 로컬 기준 소스와 실제 NUC 실행 파일의 관계, 이번 턴에서 NUC를 변경하지 않았음을 문서에 명시한다.
- [ ] Run focused verification: `pytest -q tests/test_wheel_cmd_guard.py src/static_livox_localization/test/test_stop_watchdog.py tests/test_dwa_band.py src/static_livox_localization/test/test_dwa_policy.py`
- [ ] Run relevant suite: `pytest -q tests src/static_livox_localization/test`
- [ ] `git diff --check`와 `git status --short`로 의도하지 않은 변경이 없는지 확인한다.
- [ ] 검증 결과와 기존 실패가 있다면 정확한 범위를 기록한다.
- [ ] 기능 브랜치를 로컬 `main`에 `--ff-only`로 반영하고, 원격지에는 push하지 않는다.

