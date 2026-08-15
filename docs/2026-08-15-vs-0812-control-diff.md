# 8/12 주행과 8/15 주행 사이에 무엇이 바뀌었나 — S자 주행의 원인 검토

현장 판단: **8/12 v5 기반 주행은 크게 개선되어 S자 주행이 없었고, 경로만 조금 다듬으면
된다고 보았다.** 8/15 주행에서 S자가 다시 보였고 CPU가 원인이라는 견해가 나왔습니다.

이 문서는 두 주행 사이의 **코드 차이**를 실제로 대조한 결과입니다.
기준 커밋은 8/12의 마지막인 `7f237a6`입니다.

---

## 요약

CPU보다 먼저 의심해야 할 변경이 **세 가지** 있습니다. 모두 8/12 이후에 들어왔고,
셋 다 조향 안정성에 직접 영향을 줍니다.

| 변경 | 8/12 | 8/15 | S자와의 관계 |
| :--- | :--- | :--- | :--- |
| `mpc_speed.MAX_SPEED` | **0.6** | **1.0** | 같은 yaw 상한에서 속도만 1.67배 |
| `dwa_follower.LATENCY_S` | 없음 | **0.55 s** 선행 보정 | 과보정 시 진동, 크기가 속도에 비례 |
| `dwa_core` 롤아웃 지평 | 시간 기반 `SIM_TIME_S` | 거리 기반 `SIM_DISTANCE_M=1.05` | 속도가 올라도 내다보는 거리가 안 늘어남 |

**이 셋은 서로를 증폭시킵니다.** 속도가 1.67배 오르면 선행 보정 거리도 1.67배 커지고,
반대로 내다보는 거리는 고정됩니다.

---

## 1. 실제로 구속하는 속도 상한이 0.6 → 1.0 으로 올랐습니다

8/12 커밋 `269698b`("raise max speed to 1.0 m/s")는 `waypoint_follower`와 `dwa_core`의
상한만 올렸습니다. **`mpc_speed.MAX_SPEED`는 8/12 종료 시점(`7f237a6`)에도 여전히
0.6이었습니다.** 확인:

```
$ git show 7f237a6:src/static_livox_localization/scripts/mpc_speed.py | grep '^MAX_SPEED'
84:MAX_SPEED = 0.6
```

이후 이 값이 1.0으로 올라갔고, 코드 주석이 왜 이것만 의미가 있는지 밝히고 있습니다:

> *This is the value that actually binds: shaped_reference clamps to it and the
> follower passes the result to the planner as speed_cap, so the caps in
> waypoint_follower and dwa_core do nothing on their own — raising those two
> alone left every station at 0.60.*

**따라서 8/12 주행은 실효 0.6 m/s, 8/15 주행은 1.0 m/s 천장에서 돌았습니다.**
S자가 8/12에 없다가 8/15에 나타났다면, 이 차이가 첫 번째 용의자입니다. `w_max`는
0.5 rad/s로 그대로인데 속도만 올랐으므로, 같은 곡선에서 조향 여유가 줄어듭니다.

실제로 8/15 주행 기록에서 명령 속도는 **최대 0.88 m/s**까지 올라갔습니다 — 0.6이었다면
불가능한 값입니다.

---

## 2. 0.55 초 선행 보정이 새로 들어왔습니다

`dwa_follower.py`에 `led_state()`가 추가됐습니다. 명령이 도달할 시점의 위치를 미리
계산해 거기서 계획합니다.

```python
LATENCY_S = 0.55        # 2026-08-11 측정
led[0] += state[3] * cos(state[2]) * lead      # 위치 = 속도 x 리드
led[1] += state[3] * sin(state[2]) * lead
led[2] += state[4] * lead                       # 헤딩 = yaw율 x 리드
```

코드 주석이 위험을 정확히 적어두었습니다:

> *A lead longer than the real lag over-steers exactly as badly as no lead
> under-steers, so this is a measured number and not a tuning knob.*

**핵심은 리드가 속도에 비례한다는 점입니다.** 위치 선행량은 `속도 × 0.55`이므로:

```
0.6 m/s  ->  0.33 m 앞을 보고 계획
1.0 m/s  ->  0.55 m 앞을 보고 계획
```

지연 시간 0.55 s가 **2026-08-11에 측정**됐다는 점이 중요합니다. 당시 실효 상한은
0.6이었으므로, 이 값은 **0.6 m/s 부근에서 측정된 값**일 가능성이 높습니다. 적재된
휠체어의 구동 지연이 속도에 대해 일정하다는 보장은 없습니다. 지연이 실제보다 길게
잡히면 과조향 → 반대 방향 보정 → 과조향, 즉 **S자 진동**이 됩니다.

---

## 3. 롤아웃 지평이 시간 기반에서 거리 기반으로 바뀌었습니다

```
8/12   rollout(state, v, w, sim_time_s=SIM_TIME_S, step_s=0.1)
8/15   rollout(state, v, w, distance_m=SIM_DISTANCE_M=1.05, steps=17)
```

시간 기반 지평은 속도가 오르면 내다보는 **거리**가 함께 늘어납니다. 거리 기반은
1.05 m로 고정입니다. 속도를 1.67배 올린 상태에서 내다보는 거리를 고정하면,
**같은 거리를 더 빨리 통과하므로 반응할 시간이 그만큼 줄어듭니다.**

거리 기반 자체는 합리적인 설계입니다(후보를 같은 기하로 평가). 다만 **속도 상한 인상과
같이 들어오면 유효 예측 시간이 줄어드는 방향**입니다: 1.05 m를 0.6 m/s로는 1.75초,
1.0 m/s로는 1.05초에 지납니다.

---

## 4. CPU 가설에 대해

CPU가 원인이려면 제어 루프가 주기를 놓쳐야 합니다. `dwa_follower`는 그 경우를 감지해
경고를 남깁니다:

```python
if elapsed > MAX_COMMAND_GAP_S:
    rospy.logwarn_throttle(5.0, "control loop gap %.2f s - resyncing command to measured", elapsed)
```

8/15 주행의 `~/live_follower.log`에서 확인한 경고는 전부 다른 종류였습니다 —
`pose step clamped`(손으로 밀었을 때)와 `position diverged from windowed search`.
**`control loop gap`은 관찰되지 않았습니다.**

다만 이는 결정적 증거가 아닙니다. 로그 꼬리 부분만 확인했고, NUC가 종료되어 전체
확인은 못 했습니다. **다음 주행에서 확인할 것:**

```bash
grep -c "control loop gap" ~/live_follower.log     # 0 이면 CPU 굶음 아님
top -b -n1 | head -15                              # 주행 중 부하
```

`/waypoint_follower/status`의 발행 간격도 블랙박스에서 확인 가능합니다. 10 Hz가
일정하면 루프는 정상입니다.

**현재 근거로는 CPU보다 위 세 가지 변경이 훨씬 유력합니다.** CPU 부하는 루프 주기를
불규칙하게 만들지만, 위 셋은 주기가 완벽해도 진동을 만듭니다.

---

## 5. 제안하는 확인 순서

가장 싸고 되돌리기 쉬운 것부터입니다.

**1단계 — 속도만 되돌려 재현 확인**

```bash
# mpc_speed.py 의 MAX_SPEED 를 0.6 으로 되돌리고 같은 경로 주행
```
S자가 사라지면 원인은 속도 인상이고, CPU는 무관합니다. 코드 한 줄이라 가장 먼저
해볼 값어치가 있습니다.

**2단계 — 선행 보정 끄고 확인**

```bash
rosrun static_livox_localization dwa_follower.py _latency_s:=0.0 ...
```
주석대로 `0.0`이면 항등이 되어 8/12 동작으로 돌아갑니다. 여기서 S자가 사라지면
0.55 s가 현재 속도대에 맞지 않는 것이므로 **재측정**이 필요합니다.

**3단계 — 지연 재측정**

8/11과 같은 방법(명령 `angular.z`와 `/fast_lio_icp/pose`에서 미분한 yaw율의
상호상관)으로 **1.0 m/s 부근에서** 다시 측정합니다. 0.55와 다르면 속도별 값이
필요하다는 뜻입니다.

**4단계 — CPU**

위 셋이 모두 아니면 그때 CPU를 봅니다.

---

## 6. 곡률 정지와의 관계

오늘의 `HOLD:SLOWER_THAN_FLOOR` 정지(별도 문서 `2026-08-15-dwa-first-drive.md`)는
**이 S자 문제와 다른 사안**입니다. 정지는 경로 곡률이 물리 하한을 뚫어서 생겼고,
`curvature_speed` 자체는 8/12 이후 바뀌지 않았습니다.

다만 `MAX_SPEED` 인상은 정지 지점에도 영향이 없습니다 — `curvature_speed`는
`min(MAX_SPEED, w_max/peak)`이고 그 코너의 `w_max/peak = 0.142`이므로, 상한이
0.6이든 1.0이든 결과는 같습니다. **즉 8/12 코드로 이 경로를 달렸어도 같은 자리에서
멈췄을 것입니다.** 8/12에 정지가 없었던 이유는 코드가 아니라 **경로가 v5였기
때문**입니다.

정리하면 두 문제는 독립입니다:

- **정지** ← 경로 곡률 (v5 → 알고리즘 경로 교체)
- **S자** ← 제어 파라미터 (속도 상한·선행 보정·롤아웃 지평)
