# MPC 폴로워 핸드오프

브랜치 `feat/mpc-vehicle-node` · 커밋 `682f40f`, `4c984fe`, `2754465` (base `origin/main` = `3b9c218`)

이 문서는 **인수인계용 진입점**이다. 사람이 읽을 맥락과, 에이전트가 바로 쓸 수 있는
경로·명령·불변식을 같이 담았다. 설계 근거는 `docs/mpc_follower_design.md`,
운용 절차는 `docs/runbooks/mpc-profile-ko.md`에 있고 여기서 중복하지 않는다.

---

## 1. 30초 요약

MPC 폴로워를 실차에 올릴 수 있는 형태로 만들었다. 기존 pure-pursuit 폴로워를
**상속**해서 "자세 → Twist" 변환만 교체했고, 가드는 전부 물려받는다.
`PROFILE=mpc`로 켜지며 **기본값은 pursuit**이다.

작업 중 MPC가 378 m 경로의 350 m에서 멈추는 문제가 있었고, 원인은 복도 폭이
아니라 **결함 3개**였다(헤딩 기준 계단, 선형화 여유 과다, 그 여유가 지평선에
평평하게 걸림). 고친 뒤 σ=2 cm 지터에서 시드 5개 전부 완주, 밴드 이탈 0.

**시뮬레이션만 통과했다. 실차 주행은 0회.** 이 둘을 절대 섞지 말 것.

---

## 2. 어디를 읽나

| 알고 싶은 것 | 파일 |
|---|---|
| 왜 이렇게 설계했나, 어떤 수치로 정했나 | `docs/mpc_follower_design.md` (특히 7절) |
| NUC에 올려서 굴리는 절차, 중단 기준 | `docs/runbooks/mpc-profile-ko.md` |
| 인수인계 (이 문서) | `docs/mpc_handoff.md` |
| 솔버 본체 (Qwen 작성) | `scripts/mpc_core.py` |
| 상태 앵커 / 속도 정책 / ROS 노드 | `scripts/mpc_anchor.py`, `mpc_speed.py`, `mpc_follower.py` |

각 모듈의 docstring에 **그 코드가 존재하는 이유와 측정값**이 적혀 있다.
코드를 고치기 전에 해당 docstring을 먼저 읽을 것. 상수 대부분은 취향이 아니라
측정 결과이고, 어떤 측정인지 거기 적혀 있다.

---

## 3. 검증된 것과 안 된 것

**섞으면 안 되는 두 칸이다.**

| 검증됨 (시뮬레이션) | 검증 안 됨 |
|---|---|
| v4 밴드 378 m 완주, σ=2 cm × 시드 5개 | 실차 주행 (0회) |
| 밴드 이탈 0.0000 m (전 조건) | NUC solve 시간 p99 / CPU |
| 횡오차 p95 0.021~0.022 m | 지연 보상 L (기본값 0) |
| σ=3 cm·5 cm에서는 335 m 정지 | 조향 감각 (hunting 여부) |
| 오프라인 테스트 49개 | 경사·`DEGRADED` 감속 (시뮬에 입력 없음) |

시뮬레이션의 충실도 한계: 유니사이클 플랜트, 작동기 지연 없음, 밴드는 녹화물.
`docs/simulator_fidelity.md` 참고.

**주입 지터가 실제보다 험하다는 근거** (이게 없으면 위 완주는 의미 없다):
블랙박스 3회 주행 직선 구간 9,782 표본에서 실제 포즈 잔차는 중앙 4.0 / p95 11.2 /
p99 21.8 mm. 완주 조건(σ=2 cm)은 같은 척도로 중앙 22.2 / p95 46.2 / p99 57.4 mm —
**완주 조건의 중앙값이 실제 의자의 p99보다 크다.**

---

## 4. 코드 구조 — 무엇이 무엇을 상속하나

```
WaypointFollower  (waypoint_follower.py, 검증된 pure-pursuit)
├─ handled_before_driving()   홀드 사다리 + 목표 도달 판정
├─ advance_progress()         진행도 + route_locked (지오펜스를 무장시킴)
└─ MpcFollower  (mpc_follower.py)
   └─ step()  ← 이것만 다르다
```

`MpcFollower`는 **가드를 하나도 재구현하지 않는다.** 홀드 사다리, 지오펜스,
밴드 포함 검사, 로컬라이제이션 건강도, 클러스터 생존 확인, 수동 모드 오버라이드,
종료 시 정지 — 전부 상속이다. `waypoint_follower.py`에서 뽑아낸 두 메서드는
동작을 바꾸지 않은 순수 추출이다(커밋 `682f40f`).

두 프로파일은 **같은 노드 이름(`waypoint_follower`), 같은 토픽, 같은 서비스**를
쓴다. 그래서 `go.sh` / `stop.sh` / 블랙박스 기록이 그대로 동작한다.

---

## 5. 깨면 안 되는 불변식

에이전트에게: 아래를 어기는 변경은 대응 테스트가 실패한다. 테스트가 실패하면
**테스트를 고치지 말고 변경을 재검토할 것.** 각 항목에 왜 그런지가 적혀 있다.

| 불변식 | 어기면 | 잡는 테스트 |
|---|---|---|
| MPC가 가드를 재구현하지 않는다 | 복사본이 조용히 원본과 어긋나고, 어긋나는 방향은 항상 "안 멈추는 폴로워" | `test_mpc_follower_does_not_reimplement_the_hold_ladder` |
| `step()`이 `handled_before_driving()`를 부른다 | 제어기가 스스로 주행 가부를 판단 | `test_mpc_step_runs_the_inherited_guards` |
| `step()`이 `advance_progress()`를 부른다 | `route_locked`가 안 켜져 `OFF_ROUTE`·`OFF_BAND`가 **영구 비활성** | `test_mpc_step_advances_progress_so_the_geofence_arms` |
| 기준속도가 0.22~0.30 죽은 구간에 들어가지 않는다 | 솔버가 OK를 반환하며 정지 — 조용한 실속 | `test_a_reference_is_never_issued_inside_the_dead_zone` |
| 근접 스텝 여유 < 원거리 스텝 여유 | 의자가 이미 서 있는 땅을 금지 구역으로 만듦 (350 m 정지의 원인) | `test_near_horizon_reserve_cannot_exclude_where_the_chair_already_is` |
| 헤딩 기준이 `w_max` 초과를 요구하지 않는다 | 없는 코너를 쫓다 횡방향 이탈 | `test_heading_reference_never_demands_more_yaw_than_the_chair_has` |
| 헤딩을 평활하지 않는다 | 372 m의 진짜 71° 코너가 뭉개짐 | `test_heading_reference_still_turns_the_real_corner` |
| `PROFILE` 기본값이 pursuit | 검증 안 된 제어기가 기본이 됨 | `test_profile_defaults_to_the_validated_control_law` |
| 노드가 `sys.path` 복구 줄을 갖는다 | devel 릴레이에서 `ModuleNotFoundError`로 죽음 (전례 있음) | `test_mpc_follower_recovers_sys_path_for_the_devel_relay` |

---

## 6. 재현 명령

```bash
python3 -m pytest tests/test_mpc_vehicle_layer.py tests/test_mpc_core.py -q
```

전체 스위트는 339 통과. `test_compare_glim_runs` / `test_glim_repro` /
`test_wp0_contracts`의 6건은 저장소에 `artifacts/software_rc/`가 없어서 나는
**기존 실패**이며 이 브랜치와 무관하다.

전 구간 폐루프 시뮬은 `osqp==0.6.7.post3` + numpy가 필요하다. NUC 배포와
osqp 확인 절차는 절차서 1절에 있다.

---

## 7. 다음 할 일 (우선순위)

1. **NUC에 올려 solve 시간과 CPU 측정.** 게이트는 p99 ≤ 25 ms. FAST-LIO와
   동시에 도는 상태에서 재야 의미가 있다 — PRIEST가 롤백된 이유가 정확히 이것.
   완료 기준: `/waypoint_follower/status`에 `REUSED`가 없을 것.
2. **지연 L 측정 후 `LATENCY_S`로 설정.** 절차서 5절에 방법.
   `LATENCY_S=0.12 PROFILE=mpc ~/start_wheelchair_localization.sh` 형태로
   브링업이 `_latency_s`에 넘긴다. 지금은 훅만 있고 기본값 0이다.
   추측값을 넣지 말 것 — 경로 전체에 한쪽으로 편향이 생긴다.
3. **짧은 구간 실차 주행**, 운용자 감시 하에. 볼 것은 조향 감각(hunting)이다.
   시뮬에서 EMA 앵커로 m당 요명령 반전 5.0회 → 1.3회까지 낮췄으나 실차 미확인.
4. **2차 SQP의 벽시계 의존 재검토.** `first_ok_ms < 예산의 40%`일 때만 도는
   정제 단계가 있어서 부하에 따라 의자가 받는 명령이 달라진다. 334 m 초크의
   원인은 아니었지만(강제 ON으로도 3개 중 2개 정지) NUC 부하에서 재확인 필요.
5. **밴드 낙차 의미론 재측정.** v4·v5 모두 `hazard_clearance`가 전 구간 무한대라
   낙차 기반 감속이 비활성이다. `test_the_hazard_ramp_is_inert_on_the_shipped_band`가
   이 사실을 고정하고 있어서, 밴드를 제대로 측정하면 **그 테스트가 실패한다.
   그 실패가 좋은 소식이다.**

---

## 8. 이미 밟은 지뢰

에이전트가 반복하기 쉬운 것들이다. 여기 적힌 결론은 전부 **측정으로 뒤집힌
가설**이므로, 같은 길로 다시 들어가지 말 것.

- **"복도가 좁아서 못 푼다"** — 아니었다. 의자는 밴드 안에 있었고 1 cm만 넓히면
  풀렸다. 좁은 복도는 증상이지 원인이 아니었다.
- **"느리게 가면 풀린다"** → 처음 측정에선 0.3이 0.6보다 **더 일찍** 멈춰서
  "속도는 레버가 아니다"라고 결론냈다. 그것도 틀렸다. 헤딩 계단이라는 결함
  위에서 잰 값이었고, 결함을 고치자 속도가 다시 레버가 됐다.
  **교훈: 결함 위에서 잰 레버는 레버가 아니라 결함을 잰 것이다.**
- **"creep 0.15로 감속"** (설계 문서 7절 원문의 요구) — 그대로 구현했으면 첫
  협착부에서 의자가 서고 오류도 안 났다. 0.22 이하는 솔버가 OK를 반환하며 정지하는
  죽은 구간이다.
- **"2차 SQP를 건너뛰어서 그렇다"** — 강제로 켜도 3개 중 2개가 정지했다.
- `pkill -f "[w]aypoint_follower.py"`를 실행 명령과 같은 `bash -c`에 넣으면
  **자기 SSH 세션을 죽인다.** kill과 launch를 분리할 것.

---

## 9. 이 브랜치가 손댄 남의 코드

| 파일 | 무엇을 | 왜 |
|---|---|---|
| `waypoint_follower.py` | `handled_before_driving()`, `advance_progress()` 추출 | 두 제어기가 가드 **복사본**이 아니라 같은 코드를 부르게 하려고. 동작 변화 없는 순수 이동 |
| `mpc_core.py` | `polyline_refs` 연속 보간, `band_inset_fraction`/`band_inset_min` 추가 | 350 m·335 m 정지의 원인 2건. 상세는 설계 문서 7절 |
| `test_python_node_packaging.py` | `import x` 형식 지원, 설치된 노드끼리의 import도 형제로 인정 | `mpc_follower`가 노드가 노드를 import 하는 첫 사례라 기존 테스트가 `ValueError`로 터졌다 |

`mpc_core.py`는 Qwen이 작성한 파일이다. 위 두 변경은 기준 생성과 수치 여유에
대한 것이고 사다리·제약 구조는 건드리지 않았다. 검토 시 참고할 것.
