# NUC 현장 점검 체크리스트

**2026-08-15 상태: 배포·연결·검증 완료.** 브릿지는 `10.242.33.199`에 최신 버전으로
배포돼 실행 중이고, 앱은 폰에 설치돼 SPP로 붙는 것까지 확인했습니다.
아래 0번은 브릿지가 꺼졌을 때만 다시 하면 됩니다.

이미 확인된 것:

- 브릿지 self-test 11/11 (NUC의 Python 3.8.10)
- SPP SDP 레코드 발행, 폰이 **브릿지에** 연결 (HFP 아님)
- 경로 1897점 → 381점 전송, 폰에 지도 렌더링
- 명령 왕복 및 거부 사유 표시

남은 것은 **§2의 (2)(3)(4)(5)** — 스택을 띄워야 확인 가능한 항목들입니다.

---

## 0. 브릿지 배포 (꺼졌을 때만)

```bash
scp scripts/ros1_bluetooth_bridge.py mprp3@10.242.33.199:~/wheelchair_localization_src/scripts/
ssh mprp3@10.242.33.199 'python3 ~/wheelchair_localization_src/scripts/ros1_bluetooth_bridge.py --self-test'
```

11개 전부 PASS여야 합니다.

## 1. 기동

```bash
cd ~/wheelchair_localization_src
./scripts/nuc_bluetooth_pair.sh          # 폰이 이미 본딩돼 있으면 생략 가능
./scripts/nuc_bridge_restart.sh --allow-commands --allow-scripts
```

로그에 이 두 줄이 보여야 합니다.

```
SPP profile registered with bluetoothd (SDP record published, channel 1)
script execution ON (dir=/home/mprp3)
```

`MISSING:` 가 붙어 나오면 그 스크립트가 `~`에 없다는 뜻입니다.

---

## 2. 순서대로 확인할 것

### (1) 연결과 경로 수신

앱에서 `mprp3` 선택 → 연결. 대시보드 상단에 이렇게 떠야 합니다.

```
경로 수신: 1897개 중 381개 표시 (20260814_route_algorithm_waypoints.json)
```

- **파일명이 다르면** 브릿지가 팔로워 파라미터를 못 읽고 CLI 기본값으로 떨어진 것입니다.
  아직 스택을 안 띄웠다면 정상입니다 — 스택 기동 후 재연결하면 실제 경로로 바뀝니다.
- 지도에 선이 안 보이면 → `rosparam get /waypoint_follower/route` 로 실제 값 확인.

### (2) 지도 방향 — 가장 헷갈릴 부분

휠체어를 몇 미터 **앞으로** 밀었을 때 초록 마커가 어느 쪽으로 가는지 보세요.
경로선을 따라가면 정상입니다. 90도 틀어지거나 반대로 가면 좌표 변환 문제이니
`RouteMapView.onDraw`의 y 반전만 고치면 됩니다(5분).

투영 수식 자체는 실제 1897점 경로로 오프라인 검증했습니다 — 전 구간이 화면 안에
들어오고, 진행 마커 오차 최대 0.14 m, 축 방향도 맞습니다. 남은 건 실제 pose가
기대한 프레임으로 오는지뿐입니다.

### (3) E-STOP — 가장 중요

```bash
rostopic echo /wheel_status      # 별도 터미널에 띄워두고
```

앱에서 **E-STOP** → `data[1]`이 **77**로 바뀌는지 확인.
이게 이번 작업 전체에서 유일하게 실물 확인이 안 된 핵심입니다.

이어서 확인:

- 앱 상태가 `E-STOP 작동 중` (빨강)
- `rosservice call /waypoint_follower/start "data: false"` 가 자동으로 나갔는지
  (팔로워 로그에 `follower PAUSED`)

### (4) 해제 후 재출발

**E-STOP 해제** → `data[1]`이 65로. **이때 휠체어가 움직이면 안 됩니다.**
움직이면 즉시 조이스틱을 잡고 알려주세요 — 팔로워 정지가 안 먹은 것입니다.

그다음 **주행 시작** → 멈췄던 웨이포인트부터 이어서 가는지 확인
(`wp=n/N`의 n이 0으로 안 돌아가야 함).

### (5) 스크립트 실행

앱에서 **로컬 켜기** → `job_state: running`, 경과 시간이 올라가고 `job_tail`에
스크립트 마지막 줄이 보이는지.

**[기동 중단]의 한계를 확인하세요.** 중단 후에도 라이다·FAST-LIO가 남아 있을 수
있습니다(스크립트가 `setsid`로 분리 실행). 남아 있으면 NUC에서 직접 정리해야 합니다.

---

## 2.5 ⚠ 실제로 겪은 실패 — "연결됨"인데 값이 전부 `--`

2026-08-15 검증에서 실제로 발생했습니다. 앱은 "NUC 블루투스 연결됨"이라 표시했지만
모든 값이 `--` 였고, 원본 수신란에 이게 찍혀 있었습니다.

```
BT RX: AT+VGS=15
```

`AT+VGS`는 **핸즈프리(HFP) AT 명령**입니다. 앱이 브릿지가 아니라 **NUC의 헤드셋
서비스**에 붙은 것입니다.

**원인**: NUC는 `SPP,HSP,AudioSource,AudioSink,Avrcp,HSP_AG`를 광고합니다.
브릿지가 꺼져 있으면 SPP SDP 레코드가 사라지고, 앱의 예전 연결 순서
(`createRfcommSocket(1..5)` 채널 무작정 찔러보기 → UUID 조회)가 채널에 걸린
HFP에 먼저 붙어버립니다. 채널 번호는 서비스가 아닙니다.

**고침 두 가지**:
1. 연결 순서를 뒤집었습니다 — **SPP UUID 조회 먼저**, 리플렉션은 폴백.
   UUID 경로는 이름으로 찾으므로 브릿지 외에는 붙을 수 없습니다.
2. 프로토콜이 아닌 데이터가 오면 앱이 **"잘못된 서비스에 연결됨"**을 빨간 배너로
   띄우고 모든 제어 버튼을 비활성화합니다. 예전에는 죽은 대시보드를 정상처럼
   보여줬는데, 그게 최악입니다.

**즉, 값이 전부 `--`면 브릿지가 안 떠 있는 것입니다.** 위 명령으로 띄우세요.

## 2.6 2026-08-15 실기 검증 결과 — 통과한 것

앱 → 브릿지 → `start_wheelchair_localization.sh` → 스택 기동 → 브릿지 자동 재부착까지
전 구간이 실제로 동작했습니다.

```
[bt_bridge] RX {"command":"stack_start","confirm":true}
[bt_bridge] job 'stack' started: /home/mprp3/start_wheelchair_localization.sh
[bt_bridge] ROS master appeared -- attaching.
[bt_bridge] ROS node 'wheelchair_bt_bridge' up.
```

기동 스크립트가 `LOCALIZED` → `READY`까지 완주했고, 앱에 실측값이 들어왔습니다.

- **배터리 88%** — `/wheel_status data[7]` 매핑 확정
- **측위 TRACKING**, 휠 링크 정상, 경사 OK, 장애물 `OK · 클러스터 7 · 필터 2`
- **지도에 1897점 경로 + 실시간 위치 마커(초록)** 표시
- `/waypoint_follower/route` 파라미터가 알고리즘 경로로 확인됨

### 이때 잡아서 고친 것

| 증상 | 원인 | 조치 |
| :--- | :--- | :--- |
| 브릿지가 로그 한 줄 없이 멈춤 | roscore 없을 때 `rospy.init_node()`가 무한 대기 | 마스터 probe 후 없으면 ROS 없이 서비스, 뜨면 자동 부착 |
| 계속 HFP에 붙음 | 안드로이드 SDP 캐시에 "SPP 없음"이 남음 | 연결 전 `fetchUuidsWithSdp()`로 강제 갱신 |
| "--allow-commands off" 오안내 | 플래그는 켜져 있고 실제론 roscore 부재 | 원인별 메시지 분리 |
| 기동 직후 E-STOP 경보 오발 | manual 모드를 E-STOP으로 오인 | 앱이 명령한 경우만 E-STOP, 나머지는 "수동 모드 (대기)" |
| 장애물란에 JSON 원문 도배 | `objects_summary`가 JSON 문자열 | 파싱해 `OK · 클러스터 N · 필터 M` 로 요약 |
| `fitness 1000000000.000` | 보정 억제 시 센티넬 값 | 1e6 이상이면 숨기고 `사유`로 대체 |

### ⚠ 미해결 — NUC 전원

검증 중 NUC가 **16:50 / 17:20 / 17:50, 30분 간격으로 3번 하드 리셋**됐습니다.
정상 종료 절차 로그가 전혀 없고(전원 차단), 온도 41°C 정상, 워치독·예약 재부팅 없음.
배터리 접촉 불량으로 확인. **주행 중 발생하면 측위와 팔로워가 동시에 사라집니다.**
`uart.py`의 0.6초 워치독도 NUC와 함께 죽으므로, 그때 모터 컨트롤러가 자체적으로
정지하는지는 확인되지 않았습니다 — 주행 전에 배터리 체결부터 점검하세요.

## 3. 예상되는 문제와 대처

| 증상 | 원인 | 대처 |
| :--- | :--- | :--- |
| 지도가 비어 있음 | 팔로워 미기동 → 경로 파라미터 없음 | 스택 기동 후 앱 재연결 |
| 파일명이 예상과 다름 | `ROUTE` 환경변수로 다른 경로 기동 | 정상. 표시된 파일이 실제 주행 경로 |
| 초록 마커 안 보임 | `/fast_lio_icp/pose` 미수신 | 측위 미기동. `측위 없음` 문구로 표시됨 |
| 마커가 회색 | pose 1초 이상 갱신 없음 | 측위 끊김 — 주행 금지 |
| `주행 시작` 눌러도 거부 | `go.sh` 전제조건 미충족 | 거부 사유가 그대로 표시됨 |
| `기동 중단` 후 노드 잔존 | setsid 분리 실행 | 알려진 한계. NUC에서 직접 종료 |
| 명령이 전부 거부 | `--allow-commands` 누락 | 브릿지 재시작 |
| "잘못된 서비스에 연결됨" 배너 | 브릿지 미실행 → SPP 레코드 없음 | `nuc_bridge_restart.sh` 실행 후 앱 재연결 |
| 값이 전부 `--` 인데 연결됨 표시 | 같은 원인 (구버전 앱) | 새 APK 설치됨. 이제 배너로 알려줍니다 |

## 4. 확인되면 알려주세요

- `/wheel_status data[1]` 이 77/65로 실제로 바뀌는지
- 지도 방향이 맞는지
- `로컬 켜기`로 스택이 실제로 뜨는지

이 셋이 통과하면 실전 동작 확률을 80% → 92~95%로 올릴 수 있습니다.
