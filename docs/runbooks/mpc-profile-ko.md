# MPC 프로파일 실차 절차서

대상: NUC에서 `PROFILE=mpc`로 주행 제어기를 바꿔 실행하려는 운용자.
관련 문서: `docs/mpc_follower_design.md` (설계, 특히 7·10·12절).

---

## 0. 먼저 알아야 할 것 — 이 프로파일은 완주하지 못한다

**검증되지 않았고, 실차에서 한 번도 주행한 적이 없다.** 시뮬레이션에서 라이다
위치 지터 2 cm를 넣으면 378 m 경로의 **약 350 m 지점에서 `INFEASIBLE_STOP`으로
정지한다.** 안전한 정지(사다리가 제대로 동작한 것)이지 이탈이 아니지만, 목적지에는
도달하지 못한다.

원인은 제어기 튜닝이 아니라 **기하**다. 그 지점에서 안전밴드가 의자 중심에
남겨주는 좌우 여유가 0.13 m인데, MPC는 밴드를 **경성(hard) 반평면**으로
2.5초 구간 전체에 걸어둔다. 여유 13 cm 복도에 2 cm 지터가 들어가면 풀 수 있는
해가 없다. 그래서 **천천히 가도 해결되지 않는다** — 0.3 m/s로 접근하면 오히려 더
이른 334 m에서 멈춘다.

기존 pure-pursuit 폴로워는 2026-07-31에 같은 경로를 두 번 완주했다. 그쪽은 밴드를
경성 제약으로 걸지 않기 때문이다. **기본값은 pursuit이고, 그대로 두어야 한다.**

이 프로파일을 올리는 목적은 완주가 아니라 **NUC에서의 실측**이다:
solve 시간 p99, CPU 점유, 그리고 조향 감각.

---

## 1. 배포

```bash
cd ~/unicon-wheelchair && ./tools/push_to_nuc.sh
```

NUC에서 (실행 코드는 `~/livox_static_localization_ws`에 있다):

```bash
cd ~/livox_static_localization_ws && catkin build static_livox_localization
```

### 1.1 osqp 설치 확인 — 주행 전 필수

설계 12절의 미해결 항목이다. **첫 배포에서 반드시 확인한다.**

```bash
python3 -c "import osqp; print(osqp.__version__)"
```

`0.6.7.post3`이 나와야 한다. 없으면:

```bash
pip3 install osqp==0.6.7.post3
```

0.7 이상은 API가 다르다. `mpc_core`는 classic 0.6.x API로 작성돼 있으므로
버전이 다르면 노드가 import 단계에서 죽는다.

### 1.2 노드가 형제 모듈을 찾는지 확인

catkin은 `devel/lib`에 원본을 exec 하는 릴레이만 놓기 때문에, 이 확인은
오프라인 테스트로 대체되지 않는다. 과거에 `tip_guard`와 `waypoint_follower`가
바로 이걸로 죽었다.

설치된 릴레이를 그대로 import 해본다. `if __name__ == "__main__"` 가드가 있어
import만으로는 주행 로직이 돌지 않고, roscore도 필요 없다.

```bash
python3 -c "import sys; sys.path.insert(0, '$HOME/livox_static_localization_ws/devel/lib/static_livox_localization'); import mpc_follower; print('imports OK')"
```

`imports OK`가 나와야 한다. `ModuleNotFoundError`가 뜨면 CMakeLists의 설치
목록을 확인한다. `mpc_follower`는 `waypoint_follower`(노드)와
`mpc_core`/`mpc_speed`/`mpc_anchor`(모듈)를 모두 import 하는데, 노드가 다른
노드를 import 하는 것은 이 저장소에서 처음이다.

---

## 2. 실행

```bash
PROFILE=mpc ./tools/start_wheelchair_localization.sh
```

`PROFILE`은 `pursuit`(기본) 또는 `mpc` 두 리터럴만 받는다. 오타는 exit 65로
거부된다 — 오타가 제어기를 고르는 일은 없어야 하므로.

부팅 로그에 다음이 보여야 한다:

```
PROFILE=mpc - UNVALIDATED control law, expect a stop near 350 m
MPC profile: horizon 25 x 0.10 s, anchor gain 0.40, latency 0.000 s
latency compensation OFF (~latency_s unset)
```

주행 시작·정지는 기존과 동일하다. 계약이 같기 때문에 `go.sh`/`stop.sh`도 그대로
동작한다:

```bash
rosservice call /waypoint_follower/start "data: true"
```

---

## 3. 주행 중에 볼 것

```bash
rostopic echo /waypoint_follower/status
```

정상 주행은 이런 모양이다:

```
MPC wp=412/758 v=0.58 w=+0.03 OK
```

| 상태 | 뜻 | 대응 |
|---|---|---|
| `OK` | 정상 | — |
| `REUSED` | 예산 초과, 직전 입력 재사용 | 3연속이면 아래로 |
| `HOLD:BUDGET_STOP` | 예산 초과 3연속 → 정지 | CPU 부족. 8절 참고 |
| `HOLD:INFEASIBLE_STOP` | 밴드+동역학 충돌 → 정지 | 350 m 부근이면 **예상된 것** |
| `HOLD:BLOCKED_STOP` | 장애물로 통과 불가 → 정지 | 정상 동작 |
| `HOLD:SLOWER_THAN_FLOOR` | 정책이 0.30 m/s 미만을 요구 | 정상 동작(아래 참고) |
| `HOLD:<기타>` | 기존 가드 | pursuit과 동일 |

`HOLD:SLOWER_THAN_FLOOR`는 이 제어기에 **저속 주행 구간이 없다**는 뜻이다.
0.22 m/s 이하에서는 solver가 OK를 반환하면서 정지해버리고, 0.30 m/s 이하에서는
적재된 베이스가 아예 회전하지 않는다. 그래서 정책이 그보다 느린 속도를 요구하면
기어가는 대신 정직하게 선다.

**속도 성형은 지금 경로에서 아무 일도 하지 않는다.** v4 밴드 758 스테이션 전체에서
기준속도는 0.60으로 일정하고 `SLOWER_THAN_FLOOR` 판정은 0건이다. 낙차 항이
비활성(8절)이기 때문이다. 실제로 값이 내려가는 경우는 경사 3° 초과, 로컬라이제이션
`DEGRADED`, 그리고 장애물 감속 — 셋 다 시뮬레이션에 없던 입력이다. 즉 이 부분은
실차에서 처음 동작한다.

---

## 4. 중단 기준

다음 중 하나면 즉시 조이스틱으로 중단하고 pursuit으로 되돌린다.

- 조향이 좌우로 바쁘게 왕복한다 (설계 7절의 hunting). 시뮬에서는 EMA 앵커로
  m당 1.3회까지 낮췄지만 실차에서 확인된 적이 없다.
- `REUSED`가 산발적으로가 아니라 지속적으로 뜬다.
- 밴드 밖으로 나가려 한다 — `HOLD:OFF_BAND`가 뜨지 않는데 시각적으로 벗어난다면
  가드 자체를 의심하고 즉시 중단한다.
- 350 m가 아닌 곳에서 `INFEASIBLE_STOP`이 반복된다.

되돌리기는 `PROFILE`을 빼고 재실행하면 된다. 코드 롤백이 필요 없다:

```bash
./tools/start_wheelchair_localization.sh
```

---

## 5. 지연 보상(latency) 측정 — 아직 안 된 항목

설계 7절이 요구하지만 **NUC에서 L을 잰 적이 없어서 기본값 0으로 두었다.**
추측한 지연은 경로 전체에 한쪽으로 치우친 편향을 넣으므로 넣지 않았다.

측정 방법: 정지 상태에서 `/cmd_vel_raw`에 값이 실린 시각과 `/Odometry`의 속도가
반응하기 시작한 시각의 차이를 본다.

```bash
rosbag record -O latency.bag /cmd_vel_raw /Odometry /wheel_status
```

몇 차례 가감속 후, 두 시계열의 상호상관에서 지연을 얻는다. 값이 나오면:

```bash
PROFILE=mpc rosrun static_livox_localization mpc_follower.py _latency_s:=0.12
```

또는 부팅 스크립트에 인자를 추가한다. 넣고 나면 조향 위상이 바뀌므로 4절의
중단 기준을 다시 적용한다.

---

## 6. 이번 배포에서 실제로 재는 것

설계 10절의 NUC 게이트는 **solve 시간 p99 ≤ 25 ms**다. 개발 장비에서는
p99 6.5 ms였다.

```bash
rostopic echo /waypoint_follower/status | grep -c REUSED
```

`REUSED`가 전혀 없으면 40 ms 예산 안에 들었다는 뜻이다. CPU는 FAST-LIO와
동시에 도는 상태에서 봐야 한다 — PRIEST가 롤백된 이유가 정확히 이것이었다:

```bash
top -b -n 5 -d 2 | grep -E "mpc_follower|fastlio|moving_icp"
```

---

## 7. 이 프로파일이 상속하는 것과 바꾸는 것

**바꾸는 것은 자세를 Twist로 바꾸는 부분 하나뿐이다.** 아래는 전부
`waypoint_follower`에서 그대로 상속된다. 복사본이 아니라 같은 코드를 부른다:

- 홀드 사다리 전체(`hold_candidates` + `evaluate_holds`)
- 지오펜스(`OFF_ROUTE`), 밴드 포함 검사(`OFF_BAND`)
- 로컬라이제이션 건강도, 모션 추정 게이트
- 클러스터 생존 확인(`CLUSTERS_STALE`), 수동 모드 오버라이드
- 종료 시 정지, `POLICIES_OFF` 시의 `WOULD_HOLD` 기록

`tests/test_mpc_vehicle_layer.py`가 이 구조를 정적으로 고정한다 — 누군가
MPC 쪽에 가드를 복사해 넣으면 테스트가 실패한다. 가드 복사본은 조용히
원본과 어긋나고, 어긋나는 방향은 항상 "멈추지 않는 폴로워" 쪽이다.

장애물 입력은 설계 12절의 미해결 항목이었고, 폴로워의 기존
`corridor_threat`(클러스터 인지 포함)를 그대로 쓰는 쪽으로 정했다. 가장 가까운
위협 하나만 MPC에 연성 반평면으로 넘긴다.

---

## 8. 알려진 미해결

| 항목 | 상태 |
|---|---|
| 350 m 초크 미완주 | **미해결.** 기하 문제, 속도로 안 풀림 |
| NUC 지연 L | 미측정, 기본값 0 |
| NUC solve 시간 / CPU | 미측정 (이번 배포의 목적) |
| 실차 조향 감각 | 미확인 |
| 밴드 낙차 의미론 | v4·v5 모두 hazard_clearance가 전 구간 무한대 → 속도 정책의 낙차 항이 비활성. 밴드 재측정 필요 |
