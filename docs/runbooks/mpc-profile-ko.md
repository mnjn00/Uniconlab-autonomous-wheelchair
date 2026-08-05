# MPC 프로파일 실차 절차서

대상: NUC에서 `PROFILE=mpc`로 주행 제어기를 바꿔 실행하려는 운용자.
관련 문서: `docs/mpc_handoff.md` (인수인계 진입점 — 처음이면 여기부터),
`docs/mpc_follower_design.md` (설계, 특히 7·10·12절).

---

## 0. 먼저 알아야 할 것

**시뮬레이션에서는 전 구간을 완주한다. 실차에서는 한 번도 주행한 적이 없다.**
이 둘을 섞지 말 것.

처음 구현했을 때는 378 m 중 350 m에서 `INFEASIBLE_STOP`으로 멈췄고, 원인은
"복도가 너무 좁다"로 보였다. 아니었다. 결함이 셋 겹쳐 있었고, 하나씩 계측해서
찾았다. 자세한 내역은 `docs/mpc_follower_design.md` 7절에 있고, 요약하면:

1. **헤딩 기준의 계단 현상.** 스테이션 간격 0.5 m를 계단식으로 샘플링해서
   스테이션 경계마다 최대 26.7°를 한 스텝(0.1초)에 몰아줬다. 비용함수는 이걸
   4.7 rad/s 회전 요구로 읽는데 한계는 0.5다. 의자가 없는 코너를 쫓다 옆으로
   밀리고, 그 결과가 밴드 위반이었다. arc 기준 연속 보간으로 교체(평활이 아니다 —
   372 m의 진짜 71° 코너는 그대로 둬야 하므로).
2. **선형화 여유가 협착부의 절반을 먹음.** 여유는 안전 여유가 아니라 수치 여유인데
   복도의 25%(편측)를 잘라 0.13 m 협착부를 3.5 mm 차이로 못 풀게 만들었다. 15%로.
3. **여유가 지평선 전체에 평평하게 걸림.** 이게 결정타였다. 0~4 스텝만 완화하면
   풀리고 뒤쪽은 아무리 완화해도 안 풀렸다. 스텝 1은 0.03 m 앞이고 실측 오차가
   0.0 mm인데 거기에 52 mm를 예약하면 **의자가 이미 서 있는 땅을 금지 구역으로
   만든다.** 실측한 오차 곡선대로 스텝에 비례하는 램프로 교체.

지금은 여기에 더해 앞 15 m 중 가장 좁은 복도에 맞춰 감속하고(협착부는 0.5 m/s에서
안 풀리고 0.4에서 풀린다), 곡률이 요구하는 요레이트가 한계를 넘는 두 스테이션에서
속도를 제한한다.

### 이 검증이 얼마나 보수적인가

주입한 지터가 실제보다 큰지 블랙박스로 확인했다. 완주 3회의 직선 구간(창 내 헤딩
변화 2° 미만, 속도 0.15 m/s 초과) 9,782 표본을 같은 척도로 비교하면:

| 조건 | 중앙 | p95 | p99 |
|---|---|---|---|
| **실측** | **4.0 mm** | **11.2 mm** | **21.8 mm** |
| 주입 σ=2 cm (완주) | 22.2 mm | 46.2 mm | 57.4 mm |
| 주입 σ=3 cm (정지) | 33.2 mm | 69.2 mm | 85.9 mm |
| 주입 σ=5 cm (정지) | 55.4 mm | 115.7 mm | 143.3 mm |

**완주 조건의 중앙값이 실제 의자의 p99보다 크다.** 꼬리만이 아니라 분포 전체에서
실제보다 험한 조건에서 완주한다는 뜻이다.

다만 여유가 넉넉하다는 뜻은 아니다. σ=3 cm면 335 m에서 정지한다. 실측 p99의 1.5배를
**연속으로** 넣는 조건이므로 마진은 실재하지만, "완주한다"를 "여유롭다"로 읽지 말 것.

### 지금 싣는 밴드는 v5다

위 결함 3건과 σ 비교는 **v4 밴드**에서 잰 것이다. 2026-08-04에 브링업이 v5로
바뀌었고, v5에서 다시 돌렸다:

| | 스테이션 | 길이 | 복도폭 최소 | p5 | 중앙 |
|---|---|---|---|---|---|
| v4 | 758 | 378 m | 0.13 m | 0.50 m | 0.95 m |
| **v5 (현재)** | 802 | 379 m | **0.45 m** | 0.60 m | 1.16 m |

v4에서 334/335 m 정지를 만들던 0.13 m 협착부가 v5에는 없다. 결과도 그만큼 낫다:

| 지터 | 결과 | 밴드 이탈 | 횡오차 p95 |
|---|---|---|---|
| σ=0 | 완주 379 m | 0.0000 m | 0.008 m |
| σ=2 cm × 3시드 | **전부 완주** | 0.0000 m | 0.011~0.012 m |

v4에서 0.020~0.022 m였던 횡오차가 절반이고, 정지가 한 번도 없다. v4에서 마진의
경계였던 σ=3 cm는 v5에서 재측정하지 않았다 — 필요하면 확인할 것.

**그럼에도 기본값은 pursuit이고, 그대로 두어야 한다.** pure-pursuit은 2026-07-31에
이 경로를 실제로 두 번 완주했다. MPC는 시뮬레이션만 통과했다. 이 프로파일을 올리는
목적은 NUC에서의 실측이다: solve 시간 p99, CPU 점유, 그리고 조향 감각.

---

## 1. 배포

맵 디렉터리는 필수 인자다. 노트북에서:

```bash
cd ~/unicon-wheelchair && ./tools/push_to_nuc.sh /Volumes/무제/merged_0707_0725_v1
```

이 스크립트가 **NUC에서 catkin 빌드까지 수행한다**(`~/livox_static_localization_ws`,
빌드 스페이스를 소유한 도구를 자동 선택). 따로 빌드할 필요가 없다.
`build OK`가 출력되는지 확인할 것.

배포 전 두 가지를 거부한다는 점을 알아둘 것 — 배포 대상에 커밋 안 된 변경이
있거나, 로컬 HEAD가 `origin/main`과 정확히 같지 않으면 아무것도 바꾸지 않고
중단한다. 즉 **작업 디렉터리를 먼저 `origin/main`에 맞춰야 한다.**

같은 스크립트가 브링업 4종(`start_wheelchair_localization.sh`, `trial_0727.sh`,
`go.sh`, `stop.sh`)을 NUC의 `$HOME/`에 설치한다. 그래서 아래 명령들은
`~/tools/`가 아니라 `~/`에서 실행한다.

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
PROFILE=mpc ~/start_wheelchair_localization.sh
```

`PROFILE`은 `pursuit`(기본) 또는 `mpc` 두 리터럴만 받는다. 오타는 exit 65로
거부된다 — 오타가 제어기를 고르는 일은 없어야 하므로.

> **`trial_0727.sh`를 MPC 첫 주행에 쓰지 말 것.** 그 스크립트는
> `SAFETY_POLICIES=false`로 띄운다 — 로컬라이제이션만 측정하려고 재량 가드를
> 전부 끈 구성이고, 그때 실패해도 조이스틱밖에 안 남는다. 검증 안 된 제어기를
> 가드 없이 처음 굴리는 건 두 가지 미지수를 곱하는 짓이다. 위 명령은 가드가
> 켜진 기본 구성으로 띄운다.

부팅 로그에 다음이 보여야 한다:

```
PROFILE=mpc - simulation-only control law, never driven on the chair
watch /waypoint_follower/status; see docs/runbooks/mpc-profile-ko.md
control law: mpc
MPC profile: horizon 25 x 0.10 s, anchor gain 0.40, latency 0.000 s
latency compensation OFF (~latency_s unset)
```

주행 시작·정지는 기존과 동일하다. 계약이 같기 때문에 `go.sh`/`stop.sh`도 그대로
동작한다:

```bash
rosservice call /waypoint_follower/start "data: true"
```

### 2.1 MPC 주행은 `go_mpc.sh`로 시작한다

`go.sh`는 제어법을 가리지 않는다 — 어느 쪽이 돌든 성립해야 하는 것만 본다.
그래서 **`PROFILE=mpc`를 빼먹었는지는 아무 데서도 안 걸린다.** 두 프로파일은
노드 이름·토픽·서비스가 같고, 상태줄도 의자가 움직이기 전까지는 양쪽 다
`HOLD:PAUSED`다. pursuit 주행을 MPC 계측으로 기록해도 알 방법이 없다.

```bash
~/go_mpc.sh
```

이 스크립트는 폴로워가 **스스로 게시한** `~control_law`를 읽어 `mpc`가 아니면
거부하고, 확인되면 `go.sh`로 넘긴다. 안전 검사는 하나도 복제하지 않는다.

식별자를 실행기가 아니라 클래스가 게시하는 이유는 전례가 있어서다.
`~/preflight_priest_v5.sh`는 자기가 한 줄 위에서 export한 `PLANNER`를 자기가
다시 비교했고, `81fed5d`가 PRIEST를 되돌려 브링업이 `_planner`를 더 이상
넘기지 않게 된 뒤에도 계속 "priest"를 통과시켰다.

되돌아가려면 `PROFILE` 없이 재기동한 뒤 평소대로 `~/go.sh`를 쓴다.

---

## 3. 주행 중에 볼 것

```bash
rostopic echo /waypoint_follower/status
```

정상 주행은 이런 모양이다:

```
MPC wp=412/2004 v=0.58 w=+0.03 OK
```

| 상태 | 뜻 | 대응 |
|---|---|---|
| `OK` | 정상 | — |
| `REUSED` | 예산 초과, 직전 입력 재사용 | 3연속이면 아래로 |
| `HOLD:BUDGET_STOP` | 예산 초과 3연속 → 정지 | CPU 부족. 8절 참고 |
| `HOLD:INFEASIBLE_STOP` | 밴드+동역학 충돌 → 정지 | 아래 참고 |
| `HOLD:BLOCKED_STOP` | 장애물로 통과 불가 → 정지 | 정상 동작 |
| `HOLD:SLOWER_THAN_FLOOR` | 정책이 0.30 m/s 미만을 요구 | 정상 동작(아래 참고) |
| `HOLD:<기타>` | 기존 가드 | pursuit과 동일 |

`HOLD:SLOWER_THAN_FLOOR`는 이 제어기에 **저속 주행 구간이 없다**는 뜻이다.
0.22 m/s 이하에서는 solver가 OK를 반환하면서 정지해버리고, 0.30 m/s 이하에서는
적재된 베이스가 아예 회전하지 않는다. 그래서 정책이 그보다 느린 속도를 요구하면
기어가는 대신 정직하게 선다.

`INFEASIBLE_STOP`은 이제 "밴드 안에 머무를 방법이 정말로 없다"는 뜻이다. 시뮬에서
334 m 초크를 만들던 세 결함은 제거됐으므로, 이게 뜬다면 예상된 동작이 아니라
**보고할 사건**이다. 정지 지점의 arc와 `~/live_follower.log`를 남길 것.

**낙차 기반 감속은 지금 경로에서 여전히 비활성이다.** v5 밴드 802 스테이션 전체에서
`hazard_clearance`가 무한대라 그 항은 못 뜬다(8절). 반면 **복도 폭 감속과 곡률 제한은
동작한다** — 협착부에서 0.30까지 내려간다. 경사 3° 초과와 로컬라이제이션 `DEGRADED`
감속은 시뮬에 없던 입력이라 실차에서 처음 동작한다.

---

## 4. 중단 기준

다음 중 하나면 즉시 조이스틱으로 중단하고 pursuit으로 되돌린다.

- 조향이 좌우로 바쁘게 왕복한다 (설계 7절의 hunting). 시뮬에서는 EMA 앵커로
  m당 1.3회까지 낮췄지만 실차에서 확인된 적이 없다.
- `REUSED`가 산발적으로가 아니라 지속적으로 뜬다.
- 밴드 밖으로 나가려 한다 — `HOLD:OFF_BAND`가 뜨지 않는데 시각적으로 벗어난다면
  가드 자체를 의심하고 즉시 중단한다.
- `INFEASIBLE_STOP`이 난다. 시뮬에서는 실제보다 큰 지터를 넣고도 완주하므로,
  실차에서 이게 뜨면 시뮬이 재현하지 못한 무언가가 있다는 뜻이다.

되돌리기는 `PROFILE`을 빼고 재실행하면 된다. 코드 롤백이 필요 없다:

```bash
~/start_wheelchair_localization.sh
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

몇 차례 가감속 후, 두 시계열의 상호상관에서 지연을 얻는다. 값이 나오면
브링업에 환경변수로 넘긴다:

```bash
LATENCY_S=0.12 PROFILE=mpc ~/start_wheelchair_localization.sh
```

노드를 `rosrun`으로 직접 띄우지 말 것 — `_route`, `_safety_band`,
`_body_frame_profile`이 필수 파라미터라 그냥 죽고, 스택의 나머지도 안 뜬다.
`LATENCY_S`도 브링업 스크립트가 읽는 값이지 노드가 읽는 환경변수가 아니다.

숫자가 아니면 exit 65로 거부된다. 기본값은 0이고, 그때 부팅 로그에
`latency compensation OFF`가 뜬다. 넣고 나면 조향 위상이 바뀌므로 4절의
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
| 334/350 m 초크 | **해결.** 결함 3건 제거 후 σ=2 cm에서 시드 3개 완주, 밴드 이탈 0 |
| 실차 주행 | **미실시.** 시뮬레이션만 통과했다 |
| NUC 지연 L | 미측정, 기본값 0 |
| NUC solve 시간 / CPU | 미측정 (이번 배포의 목적) |
| 실차 조향 감각 | 미확인 |
| 2차 SQP가 벽시계에 의존 | `first_ok_ms < 예산의 40%`일 때만 도는 정제 단계가 있다. 즉 부하에 따라 의자가 받는 명령이 달라진다. 334 m 초크의 원인은 아니었지만(강제로 켜도 3개 중 2개 정지, 꺼도 3개 정지) NUC에서 부하가 실릴 때 재확인 필요 |
| 지터 마진이 얇음 | σ=2 cm 완주 / σ=3 cm 정지. 실측 p99 21.8 mm 대비 마진은 있으나 크지 않다. 로컬라이제이션이 나빠지면 여기가 먼저 무너진다 |
| 밴드 낙차 의미론 | v4·v5 모두 hazard_clearance가 전 구간 무한대 → 속도 정책의 낙차 항이 비활성. 밴드 재측정 필요 |
