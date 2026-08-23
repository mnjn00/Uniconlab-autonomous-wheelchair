# 자율주행 휠체어 블루투스 UI 인수인계

Intel NUC 휠체어를 블루투스 SPP(RFCOMM) 시리얼 링크로 모니터링·제어하는 안드로이드 앱과,
NUC 쪽 ROS 1 Noetic 브릿지에 대한 문서입니다.

로봇 쪽 상세 정보는 [`nuc_system_reference.md`](nuc_system_reference.md)에 따로 있습니다.
NUC 없이 작업을 이어가야 할 때 그 문서를 보세요.

---

## 1. 저작권과 출처 표기

안드로이드 앱은 **박형준(Park Hyeongjun)** 님의
[`edge-mobility-monitor`](https://github.com/Geppetto0608/edge-mobility-monitor)를
수정한 것입니다. © 2026, all rights reserved.

**박형준 님이 이 프로젝트에 대해 직접 사용·수정을 허락**하셨고, 조건은 출처 표기입니다
(2026-08-14 기록). 저장소의 `LICENSE` 파일 자체는 아무 권리도 부여하지 않으므로,
그 개인적 허락이 이 작업이 존재할 수 있는 유일한 근거입니다. UI가 노출되는 모든 곳에
아래 문구를 유지하세요.

> UI는 박형준 님의 *edge-mobility-monitor*를 기반으로 하며, 허락을 받아 사용합니다.
> <https://github.com/Geppetto0608/edge-mobility-monitor>

허락이 구두이므로 한 줄이라도 글로 받아두는 편이 좋습니다. 공개 저장소 push / 논문·포스터
게재 / 랩 외부 APK 배포까지 포함되는지도 확인이 필요합니다. 체크리스트는
[`android_wheelchair_ui/NOTICE.md`](../android_wheelchair_ui/NOTICE.md)에 있습니다.

---

## 2. ⚠ 이 저장소에는 스택이 두 개 있고, 휠체어를 움직이는 건 하나뿐입니다

이 문서에서 가장 중요한 내용입니다.

| | WP0 스캐폴드 | **실제 필드 스택** |
| :--- | :--- | :--- |
| 위치 | `src/wheelchair_safety`, `src/wheelchair_interfaces` | `~/livox_static_localization_ws/src/` |
| 실행 주체 | 없음 | `~/start_wheelchair_localization.sh` |
| 토픽 | `/safety/state`, `/cmd_vel_safe`, `/safety/estop` | `/cmd_vel_raw`, `/cmd_vel_gated`, `/cmd_vel`, `/wheel_status`, `/mode_cmd` |
| 개념 | `armed`, `reason_mask`, geofence, `sidewalk`/`road_free_space` | auto/manual 주행 모드, tip guard, 장애물 클러스터 |
| 실체 | `catkin_make`가 빌드하지만 **한 번도 실행되지 않음** | **이것이 휠체어를 움직입니다** |

`armed`, `reason_mask`, geofence 상태, `sidewalk`/`road_free_space` 모드는 전부 WP0
스캐폴드의 용어이고 **실행 중인 시스템에는 존재하지 않습니다.** 낯설게 느껴졌던 이유가
바로 이것입니다. 브릿지는 실제 필드 스택을 대상으로 합니다. `docs/interfaces.md`를 근거로
통합하지 마세요.

### 실제 명령 체인

```
follower (waypoint_follower.py | mpc_follower.py | dwa_follower.py)
      │  /cmd_vel_raw
      ▼
safety_gate.py       INPUT_STALE / CLOUD_STALE / INPUT_INVALID /
      │  /cmd_vel_gated   REVERSE / 장애물 홀드 시 정지. 사유는 발행하지 않음
      ▼
tip_guard.py         /tip_guard/status 도 발행
      │  /cmd_vel
      ▼
wheel_cmd_tmp.py     callerid가 /tip_guard 가 아니면 /cmd_vel 거부
      │  /wheel_cmd
      ▼
uart.py              모드 게이트 + 0.6초 명령 기아 워치독
      │  UART 115200
      ▼
모터 컨트롤러
```

### 주행 모드는 드라이버 레벨의 정수입니다

`uart.py`가 **`/mode_cmd` (`std_msgs/Int16`)** 를 구독합니다.

| 값 | 의미 | `uart.py` 동작 |
| :--- | :--- | :--- |
| `65` `'A'` | **Auto** | 모터 정지 프레임 송신 후 `wheel_cmd` 전달 |
| `77` `'M'` | **Manual** | 모터 정지 프레임 송신 후 **모든 `wheel_cmd` 무시** |

`/wheel_status`(`Int16MultiArray`)는 원본 UART 프레임입니다. `data[0]==72`(`'H'`)가 헤더,
**`data[1]`이 모터 컨트롤러가 되돌려주는 모드 echo**, `data[7]`이 배터리입니다. 이 echo가
명령이 실제로 먹혔는지 확인할 유일한 근거입니다.

### 브릿지가 지키는 규칙

1. **발행하는 토픽은 `/mode_cmd` 단 하나입니다.** `wheel_cmd_tmp.py`가 `/cmd_vel`의
   publisher callerid를 `/tip_guard`로, `/wheel_status`를 `/uart`로 검증합니다. 엉뚱한
   publisher가 끼어들면 `fault_latched`가 걸려 휠체어가 fault stop에 빠집니다.
   `/mode_cmd`는 callerid 검사가 없어서 안전한 레버입니다.
2. **나머지는 전부 읽기 전용입니다.**
3. `safety_gate`가 홀드 사유를 발행하지 않으므로, 필드 노드를 수정하는 대신
   **추론**합니다 — `/cmd_vel_raw`가 0이 아닌데 `/cmd_vel_gated`가 0이면 `motion_blocked`.

---

## 3. E-STOP과 해제

**E-STOP은 `/mode_cmd = 77`입니다.** 드라이버 레벨에서 모터를 세우고 자율 명령 경로를
모든 ROS 가드보다 **아래에서** 차단하므로, follower·safety_gate·tip_guard가 전부 죽어도
유지됩니다. 그리고 조종간(조이스틱) 제어로 돌아갑니다 — 탑승자가 있을 때 올바른 결과입니다.

참고로 `trial_0727.sh`의 설명대로 **조이스틱을 움직이는 것 자체가 failsafe**입니다.
base가 auto 모드에서 빠지고 follower가 한 제어 주기 안에 홀드합니다.

**해제는 `/mode_cmd = 65`입니다.** `uart.py`가 auto를 다시 켜기 전에 정지 프레임을 먼저
보내므로 급발진이 발생할 수 없습니다.

해제 경로에는 의도적으로 가드를 걸었습니다. 앱에서 할 수 있되, 실수로는 안 되게:

| 가드 | 동작 |
| :--- | :--- |
| 명시적 확인 | `{"command":"estop_release"}`는 거부. `"confirm": true` 필요 |
| 링크 신선도 | `/wheel_status`가 `--ttl`보다 오래됐으면 거부 |
| 실제 정지 여부 | 접지 속도가 0.05 m/s를 넘으면 거부. **`/odom`(엔코더) 1순위, `/Odometry` 폴백** — §6-1 참조 |
| 속도 판독 없음 | 신선한 속도 소스가 하나도 없으면 거부 (fail-closed) |
| 주행은 그대로 정지 | 해제해도 follower를 재시작하지 **않음** |

> **⚠ 정정** — 이전 판에서 "E-STOP은 `mode_cmd 77` 하나면 된다"고 썼는데 **틀렸습니다.**
> `waypoint_follower.py:826`은 MANUAL_MODE에서 *홀드*만 하고 `enabled`는 그대로 둡니다.
> 즉 `mode_cmd 77`만 걸어놓고 나중에 `65`로 해제하면 **해제하는 순간 팔로워가 그대로
> 이어서 주행합니다.** `stop.sh`가 서비스 정지와 모드 전환을 **둘 다** 하는 이유가
> 이것입니다. 지금은 E-STOP도 둘 다 합니다 — 토픽 발행이 먼저(즉시·실패 불가),
> 그다음 서비스 정지. 회귀 테스트로 고정해뒀습니다.

주행 재개는 별도의(역시 확인이 필요한) `drive_start` 명령이고, E-STOP이 걸린 동안에는
거부됩니다. 즉 탑승자가 안전을 판단해 해제하고, 움직이기 전에 한 번 더 판단합니다.

### E-STOP 후 그 자리에서 재출발 되나 — 됩니다

`mode_cmd`는 측위를 건드리지 않습니다. FAST-LIO도 `moving_icp_localizer`도 계속 돌기
때문에 진단은 `TRACKING`을 유지하고, `go.sh`의 전제조건 3개가 그대로 성립합니다.
그리고 `waypoint_follower.py`의 진행 인덱스는 일시정지해도 **초기화되지 않습니다**
(`on_start(false)`는 `enabled`만 내리고 `send_stop()`할 뿐, 인덱스를 되돌리는 코드가
없습니다). 따라서:

**E-STOP → [E-STOP 해제] → [주행 시작]** 이면 멈춘 그 지점의 웨이포인트부터 이어서
주행합니다. 스택을 다시 띄울 필요가 없습니다.

단, 오래 세워뒀다면 측위가 그 사이 열화됐을 수 있으니 재출발 전에 지도 화면의
`fitness` / `inlier` 를 확인하세요.

---

## 4. 통신 프로토콜 (JSON lines, `\n` 종단, UTF-8)

### NUC → 폰, 2 Hz

```json
{"type":"telemetry","protocol_version":3,"seq":412,"timestamp":1786704826.5,
 "bridge_uptime_s":206.0,"ros_connected":true,
 "speed_mps":0.42,"yaw_rate_radps":0.01,"commanded_mps":0.40,
 "drive_mode":"auto","drive_mode_raw":65,
 "estop_engaged":false,"estop_pending":false,"motion_blocked":false,
 "ready_to_drive":true,"follower_start_available":true,
 "wheel_link_ok":true,"wheel_status_age_s":0.05,"odom_age_s":0.03,
 "tip_guard_status":"OK","follower_status":"RUN wp=4/31 v=0.40",
 "localization_status":"TRACKING","localization_tracking":true,
 "objects_summary":"clusters=3 nearest=2.4m","robot_fault":"none",
 "speed_source":"wheel",
 "battery_percent":87,"step_level":null,
 "unavailable":["step_level"]}
```

- `drive_mode`는 앱이 요청한 값이 아니라 **모터 컨트롤러가 되돌려준 값**입니다.
  요청과 echo 사이의 구간에서는 `estop_pending`이 true입니다.
- `display_safe_to_drive`와 `ready_to_drive`는 fail-closed입니다. 모르면 false입니다.
- `unavailable`에 들어간 필드는 회색 처리하세요. `0`으로 표시하면 안 됩니다.
- `speed_source`는 속도가 어디서 왔는지입니다 — `wheel`(`/odom`, 엔코더) /
  `lio`(`/Odometry`) / `null`(신선한 소스 없음). `null`이면 속도는 `--`입니다.

### 폰 → NUC

| 명령 | 효과 |
| :--- | :--- |
| `{"command":"estop"}` | `/mode_cmd=77` 직접 발행. 확인 없음 — 비상 정지는 한 번에 걸려야 합니다 |
| `{"command":"estop_release","confirm":true}` | `/mode_cmd=65`, §3 가드 적용 |
| `{"command":"stack_start","confirm":true}` | **`start_wheelchair_localization.sh` 실행** |
| `{"command":"drive_start","confirm":true}` | **`go.sh` 실행** (없으면 mode 65 + 서비스로 폴백) |
| `{"command":"drive_stop"}` | **`stop.sh` 실행.** 기동이 도는 중에도 대기 없이 실행됩니다 |
| `{"command":"job_cancel"}` | 실행 중인 스크립트에 SIGTERM (프로세스 그룹 전체) |
| `{"command":"ping"}` | 생존 확인 |

모든 명령은 `{"type":"ack","ok":…,"detail":…}` 응답을 정확히 하나 받습니다. `detail`은
사용자에게 그대로 보여주도록 작성돼 있고, 앱도 그렇게 표시합니다.

### 스크립트 실행 — 앱에서 로컬 켜고 주행까지

`--allow-scripts`를 주면 브릿지가 **운용자가 쓰는 스크립트를 그대로 실행**합니다.
`go.sh`를 파이썬으로 재구현하지 않는 이유는, 기동 정책이 두 벌이 되어 갈라지기 때문입니다.
스크립트를 부르면 `go.sh`의 거부 문구가 그대로 폰에 뜹니다.

**허용목록 전용입니다.** 프로토콜은 작업 *이름*만 싣고, 이름을 시작 시 고정된 테이블에서
찾습니다. 폰에서 온 문자열이 셸에 닿는 경로가 없습니다. 본딩된 기기면 누구나 이 링크를
열 수 있으므로 원격 셸이 되면 안 됩니다.

| 이름 | 스크립트 |
| :--- | :--- |
| `stack` | `start_wheelchair_localization.sh` |
| `drive` | `go.sh` |
| `halt` | `stop.sh` |

`trial_0727.sh`는 **의도적으로 제외**했습니다. `SAFETY_POLICIES=false`로 기동하므로,
가드가 억제된 주행을 폰 버튼으로 시작할 수 있게 만들 이유가 없습니다.

설계상 중요한 점 두 가지:

- **`--allow-scripts`는 `--allow-commands`와 별개입니다.** 토픽 하나 발행하는 것과 로봇에서
  프로세스를 띄우는 건 위험도가 다릅니다. 둘 다 있어야 동작합니다.
- **정지는 슬롯 대기를 하지 않습니다.** 다른 작업은 한 번에 하나만 돌지만 `halt`는 예외입니다.
  기동이 도는 중에 [주행 정지]가 "먼저 끝날 때까지 기다리라"고 거부하면 그건 정지가 아닙니다.
  이건 실제로 테스트에서 잡힌 결함입니다.

E-STOP은 스크립트를 쓰지 않고 `/mode_cmd 77`을 직접 발행합니다 — 가장 빠르고, 스크립트가
없거나 슬롯이 막혀 있어도 동작합니다.

**`drive_start`/`drive_stop`은 DWA 프로파일에서도 동작합니다.**
> **⚠ 정정 (2026-08-23)** — 이전 판에서 "pursuit 프로파일에서만 동작한다"고 썼는데
> **틀렸습니다.** `dwa_follower.py`는 `WaypointFollower`를 **상속**하므로
> `/waypoint_follower/start` 서비스를 그대로 물려받습니다. DWA 주행 중
> `rosservice info /waypoint_follower/start`로 확인했습니다.
> 앱은 어차피 서비스 존재 여부(`follower_start_available`)로 판단하므로 동작은
> 옳았고, 틀린 것은 설명뿐이었습니다.

`drive_start`는 `go.sh`의 거부 조건을 그대로 따릅니다 — 휠 베이스 침묵, 객체 추적 침묵,
측위가 `TRACKING`이 아님. `go.sh` 주석의 표현이 정확합니다:
*"조용한 생산자는 빈 객체 리스트로 주행하게 두는데, 그건 텅 빈 도로와 똑같이 보인다."*

---

## 5. 운용 절차

### 최초 1회: 폰과 NUC 페어링

```bash
./scripts/nuc_bluetooth_pair.sh          # 페어링 모드 켜기
./scripts/nuc_bluetooth_pair.sh --off    # 끝나면 되돌리기
```

이 스크립트가 필요한 이유가 있습니다. BlueZ의 discoverable은 `DiscoverableTimeout`
(기본 180초) 뒤 자동으로 꺼지고, 꺼지면 `hci0`에서 **`ISCAN` 플래그가 사라져** 폰의
"연결 가능한 기기" 목록에 NUC가 아예 뜨지 않습니다. 또한 `bluetoothctl -- discoverable on`
처럼 한 번만 실행하면 프로세스가 끝나면서 **페어링 에이전트가 사라져** 페어링 요청이
거절됩니다. 스크립트는 stdin을 열어둔 채 bluetoothctl을 상주시켜 에이전트를 유지하고
타임아웃을 0으로 바꿉니다. sudo가 필요 없습니다.

NUC에서 폰으로 거는 편이 더 확실합니다. 안드로이드는 SSP numeric comparison을 요구하므로
`Confirm passkey NNNNNN (yes/no)` 프롬프트에 `yes`를 보내야 하고, 폰에서도 같은 숫자를
확인해야 합니다.

페어링이 끝나면 `--off`로 되돌리세요. 상시 discoverable은 아무나 페어링을 시도할 수
있다는 뜻이고, 이미 본딩된 폰은 discoverable이 꺼져 있어도 잘 붙습니다.

### 매 세션, NUC에서

체크아웃 위치는 `~/wheelchair_localization_src`입니다 (사용자 `mprp3`).
`~/Uniconlab-autonomous-wheelchair`는 존재하지 않습니다.

```bash
cd ~/wheelchair_localization_src
./scripts/nuc_bluetooth_check.sh              # --fix 로 rfkill/전원 자동 조치
./scripts/nuc_bridge_restart.sh --allow-commands --allow-scripts
```

`--allow-scripts`가 있으면 앱에서 [로컬 켜기]·[주행 시작]·[주행 정지]가 실제 스크립트를
실행합니다. 스크립트 위치는 `--script-dir`(기본 `~`)로 바꿉니다.

`--allow-commands` 없이 실행하면 publisher를 아예 만들지 않습니다. 모니터링만 필요한
시연에는 그쪽이 맞습니다.

`nuc_bridge_restart.sh`를 쓰는 이유: `pkill -f ros1_bluetooth_bridge.py`를 SSH 한 줄
명령으로 보내면 패턴이 **자기 명령줄과도 매칭되어 셸 자신을 죽입니다**(exit 127처럼 보임).
스크립트는 대괄호 트릭으로 자기 자신을 제외합니다.

### SDP 함정 — 해결됐고, root가 필요 없습니다

안드로이드의 `createRfcommSocketToServiceRecord(SPP_UUID)`는 SDP 조회를 합니다. 맨
`AF_BLUETOOTH` bind는 서비스 레코드를 **전혀** 발행하지 않아서 이 호출이 실패합니다.
예전 설계가 동작했던 건 앱이 먼저 숨은 `createRfcommSocket(int)` 리플렉션 경로를 시도하기
때문인데, **최신 안드로이드는 이를 차단합니다** — 실제 대상 폰이 Android 16/SDK 36이므로
해당됩니다.

그래서 브릿지는 `--transport bluez`를 기본값으로 씁니다. D-Bus로 `org.bluez.Profile1`을
등록하면 **bluetoothd가 RFCOMM listen과 SDP 레코드를 직접 소유**하고 연결된 파일
디스크립터를 브릿지에 넘겨줍니다. `sudo`도, PyBluez도, `bluetoothd --compat`도,
`sdptool`도 필요 없습니다.

`--transport socket`은 예전 raw bind 방식입니다. 참고로 물려받은 코드는
`bind(("", channel))`을 썼는데, Python은 이를 리눅스에서도 *bad bluetooth address*로
거부합니다. 기본값을 `00:00:00:00:00:00`(BDADDR_ANY)으로 바꿨습니다.

---

## 6. 검증 결과 (2026-08-14, `mprp3@10.242.33.199`, Ubuntu 20.04.6)

**프로토콜 self-test — 11/11 통과**, Windows와 NUC의 Python 3.8.10 양쪽에서
(`python3 scripts/ros1_bluetooth_bridge.py --self-test`).

**실제 필드 토픽 대상 라이브 ROS 검증 — 23/23 통과**
(roscore + 브릿지 + 토픽 픽스처. `uart.py`는 의도적으로 띄우지 않아 모터에 아무것도
전달되지 않는 상태):

```
bridge sees ROS                                   PASS
drive_mode read from /wheel_status                PASS  (auto)
battery read from /wheel_status data[7]           PASS  (87)
speed read from /Odometry                         PASS  (0.42 m/s)
follower status relayed                           PASS
safety_gate hold inferred (raw>0, gated==0)       PASS
mode_cmd=77 actually published                    PASS
telemetry confirms estop_engaged via wheel echo   PASS
release without confirm refused                   PASS
release refused while still moving                PASS  (0.30 m/s)
mode_cmd=65 actually published                    PASS
drive_start refused while E-STOP engaged          PASS
drive_start refused when localization != TRACKING PASS
```

**블루투스 실물 연결 — 성공.** 폰(Galaxy S24+ `SM-S926N`, Android 16 / SDK 36)과
NUC를 페어링하고, 앱에서 `mprp3`를 선택해 SPP로 연결한 뒤 대시보드에 프로토콜 v3
텔레메트리가 렌더링되는 것까지 확인했습니다.

- SDP 레코드가 폰 쪽에도 보입니다 — 본딩 정보의 서비스 목록에 `SPP`가 포함됨
- 실물 연결에서만 드러난 버그 3개를 잡았습니다 (§7)

### 아직 남은 것

- [x] **실제 `uart.py`를 붙인 상태에서의 E-STOP 확인 — 2026-08-23 완료.**
      실제 모터 컨트롤러를 붙이고 앱에서 왕복시켰습니다. `/mode_cmd`와
      `/wheel_status data[1]`을 동시에 로깅한 결과:

      ```
        0.14s  ECHO  data[1] None -> 77 (MANUAL)   speed=0.000 m/s
       71.73s  CMD   /mode_cmd = 65 (AUTO)          <- 앱 [자동 모드 전환]
       71.74s  ECHO  data[1] 77 -> 65 (AUTO)        speed=0.000 m/s
      114.26s  CMD   /mode_cmd = 77 (MANUAL)        <- 앱 [E-STOP]
      114.26s  ECHO  data[1] 65 -> 77 (MANUAL)      speed=0.000 m/s
      ```

      echo 지연은 양방향 모두 **10 ms 이내**. 자동 전환 동안 휠체어는 움직이지
      않았고(팔로워 `HOLD:PAUSED` 유지, 속도 0.000), 앱 표시도
      수동 대기 → 정상 → E-STOP 작동 중으로 정확히 따라왔습니다.
      로깅에 쓴 스크립트는 `tools/` 밖(임시)이었으니, 다시 할 일이 있으면
      `/mode_cmd` + `/wheel_status data[1]`을 같이 찍으면 됩니다.
- [x] 재접속 — 자동 재연결을 붙였습니다 (2→4→8→15초 백오프). 브릿지 재시작으로
      끊고 확인했습니다. 거리(범위 이탈)는 아직 안 해봤습니다.

---

## 6-1. 2026-08-23 실주행 스택 대상 검증에서 나온 것

라이브 스택(라이다·FAST-LIO·측위·`uart.py`·`tip_guard`·DWA 팔로워 전부 기동)에
붙여서 확인했습니다. 픽스처만으로는 하나도 나오지 않았을 것들입니다.

1. **속도가 항상 0이었습니다.** 브릿지가 `/Odometry`에서 속도를 읽는데
   FAST-LIO(`laserMapping`)는 **twist를 전부 0으로 채웁니다.** 주행 중
   대시보드 `0.0 km/h` / 실제 `/odom` 0.31 m/s를 직접 대조해 확인했습니다.
   표시 문제로 끝나지 않습니다 — **"움직이는 중에는 E-STOP 해제 거부" 가드가
   현장에서 한 번도 발동할 수 없었습니다.** 지난 23/23 통과는 픽스처가 twist를
   채워줬기 때문입니다.
   → `/odom`(`base_model/odom_pub.py`, 엔코더, 100 Hz)을 1순위로, `/Odometry`를
   폴백으로 씁니다. 텔레메트리에 `speed_source`(`wheel`/`lio`/`null`)를 실었고,
   **속도 판독이 아예 없으면 해제를 거부**합니다(fail-closed).
2. **기동 직후 시동을 걸 방법이 없었습니다.** 베이스가 수동으로 쉬는 정상 상태에서
   `drive_start`는 *"E-STOP이 걸려 있으니 해제하라"* 고 거부하는데, 앱은 그 상태에서
   [E-STOP 해제]를 **비활성화**합니다. 막다른 길이었습니다.
   → 거부 문구를 앱-E-STOP / 수동 대기로 분리하고, 앱 버튼을 상태에 따라
   **[자동 모드 전환 (시동)]** 으로 바꿔 수동 대기에서도 눌리게 했습니다.
3. **비활성화된 버튼이 활성화된 것처럼 보였습니다.** 모든 제어 버튼이
   `android:textColor` / `backgroundTint`를 단색으로 박아둬서 `setEnabled(false)`가
   화면에 아무 변화도 주지 않았습니다. "로봇이 받을 수 없는 명령은 버튼 비활성화"라는
   설계가 통째로 안 보이던 셈입니다.
   → `res/color/`에 상태 리스트를 만들어 연결했습니다.
4. **속도 단위가 뒤바뀌어 있었습니다.** 큰 숫자(m/s) 옆 단위가 `km/h`, 그 아래
   보조 줄도 `km/h`. → 큰 숫자 `m/s`, 보조 줄 `km/h`.
5. **스택이 꺼져 있을 때 경로 기본값이 낡아 있었습니다.** `--route` 기본값이
   `20260814_algorithm`(1897점)인데 현장 기본은 v9(1917점)입니다. 팔로워 param이
   없는 상태 — 즉 [로컬 켜기]를 누르기 직전 — 가 정확히 그 상황입니다.
   → `start_wheelchair_localization.sh`의 `ROUTE=` 기본값을 읽습니다.
   앱도 그려진 경로 점수 ≠ `wp_total`이면 진행 마커를 숨기고 경고합니다.
6. **`/robot_fault`에는 publisher가 없습니다.** `fault_check`는 기동 스크립트에
   등장하지 않습니다. 앱이 `null`을 "없음"으로 찍어서 *결함 없음* 처럼 읽혔습니다.
   → `데이터 없음`으로 구분하고, 미수신 항목이 있으면 서브시스템 줄 전체를 회색 처리.
7. `--debug-tcp` accept 루프가 일시적 `OSError` 한 번에 조용히 죽어 프로브 포트가
   그 프로세스 수명 내내 사라졌습니다. → 리스너가 실제로 닫혔을 때만 종료.

---

## 7. 실물 연결에서만 드러난 버그들

기록해 둘 가치가 있습니다. 셋 다 벤치 테스트로는 절대 나오지 않았습니다.

1. **BlueZ가 넘겨주는 fd는 논블로킹입니다.** `recv()`가 즉시 `EAGAIN`을 던지는데
   세션이 이를 치명적 오류로 처리해서, 폰이 붙는 순간 링크가 끊겼습니다.
   → fd를 감싼 뒤 `setblocking(True)`, 그리고 `BlockingIOError`를 재시도로 처리.
2. **로그인 게이트가 되돌려보냈습니다.** `MainActivity`가 `UserSession.isLoggedIn()`을
   요구하는데 NUC에는 사용자 저장소가 없어, 블루투스 연결에 성공해도 곧바로 연결
   화면으로 튕겼습니다. → `BLUETOOTH_ONLY`에서는 게이트를 건너뜁니다.
3. **기기 목록이 보이지 않았습니다.** `android.R.layout.simple_spinner_item`이 테마
   기본 텍스트 색을 쓰는 바람에 흰 카드 위에서 글자가 보이지 않았습니다.
   → 색을 명시한 전용 아이템 레이아웃 사용.

---

## 8. 안드로이드 UI 현황

실제 로봇의 어휘에 맞춰 다시 만들었습니다.

- 모드 버튼(수동/어시스트/자율) → **주행 모드 판독 표시**. 모드는 `uart.py`가 소유하므로
  앱은 확인된 값만 보여줍니다
- step ± 제거 (이 스택에 개념이 없음)
- **E-STOP**(대형 빨강, 한 번에 적용) / **E-STOP 해제**(확인 다이얼로그) /
  **주행 시작**(확인 다이얼로그) / **주행 정지**
- 주행 준비 배지: 준비 완료(초록) / 게이트 보류(노랑) / E-STOP(빨강) / 조건 미충족(회색)
- 서브시스템 한 줄: 휠 링크 · 측위 · 경사 · 장애물 · 결함
- 브릿지의 `ack.detail`을 그대로 노출 — 거부 사유가 사용자에게 그대로 보입니다
- 로봇이 받을 수 없는 명령은 버튼 비활성화 (`follower_start_available`, `ready_to_drive`)
- 프리셋 카드 / 카메라 카드 숨김, Wi-Fi HTTP 폴링·백그라운드 서비스 차단

### 남은 정리 거리

- 레이아웃에 한국어 문자열이 하드코딩된 곳이 있습니다 (원래는 `@string/` 리소스)
- `LoginActivity` / `RegisterActivity` / 프리셋 코드가 아직 트리에 남아 있습니다.
  플래그로 비활성화만 해둔 상태이므로, Wi-Fi 빌드를 유지할 필요가 없다면 삭제하세요
- 가로 화면에서는 한 칸짜리 세로 배치가 그대로 늘어나 스크롤이 많이 필요합니다.
  운용은 세로로 하지만, 손에 쥐고 회전이 걸리면 불편합니다
- `scripts/ros1_wifi_bridge.py` / `scripts/setup_nuc_wifi_ap.sh`는 WP0 어휘
  (`armed`, `step_level`, `geofence`)로 쓰인 **버려진 Wi-Fi 경로**입니다. 커밋되지
  않은 채 트리에 남아 있으니, 되살릴 계획이 없으면 지우세요

---

## 9. 파일 위치

| 무엇 | 어디 |
| :--- | :--- |
| ROS 1 ↔ RFCOMM 브릿지 | `scripts/ros1_bluetooth_bridge.py` |
| NUC 블루투스 사전 점검 | `scripts/nuc_bluetooth_check.sh` |
| 페어링 모드 스크립트 | `scripts/nuc_bluetooth_pair.sh` |
| 브릿지 재시작 | `scripts/nuc_bridge_restart.sh` |
| NUC 시스템 레퍼런스 | `docs/nuc_system_reference.md` |
| NUC 원본 덤프 | `docs/nuc_snapshot/` |
| 안드로이드 프로젝트 | `c:\Users\npgy2\.anaconda\intern\edge-mobility-monitor\` |
| 저작권/출처 | `android_wheelchair_ui/NOTICE.md` |
| 기기 선택 화면 | `WifiSetupActivity.java` |
| 대시보드 | `MainActivity.java` |
| SPP 소켓 클라이언트 | `BluetoothWheelchairClient.java` |
| 전송 방식 플래그 | `AppConfig.java` (`BLUETOOTH_ONLY`, `CAMERA_ENABLED`) |
| ADB | `c:\Users\npgy2\.anaconda\intern\platform-tools\adb.exe` |

---

## 10. 알려진 한계

- **SPP는 페어링 외에 인증이 없습니다.** 본딩된 기기라면 무엇이든 `estop`을 보낼 수
  있습니다(안전한 방향). 하지만 `drive_start`는 그렇지 않습니다. 사람이 타기 전에
  주행 명령에 PIN을 거는 것을 검토하세요.
- **동시에 한 대만 연결됩니다.** 두 번째 폰은 백로그에서 대기하며 멈춘 것처럼 보입니다.
- **`uart.py`의 0.6초 워치독** 때문에 follower가 멈추면 이미 휠체어가 정지합니다. 앱은
  아직 이 상황을 게이트 홀드와 구분해서 보여주지 않습니다.
- `android_wheelchair_ui/`는 배포된 앱의 백업이 **아닙니다.** `com.uniconlab.wheelchair.ui`
  패키지의 오래된 3파일 스켈레톤이며, 머신별 SDK 경로가 담긴 `local.properties`가 커밋돼
  있으니 삭제하고 gitignore에 넣으세요.
