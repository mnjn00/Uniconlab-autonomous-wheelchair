# NUC 시스템 레퍼런스 — NUC 없이 작업하기 위한 모든 것

2026-08-14에 `mprp3@10.242.33.199`에서 직접 수집했습니다. 이 문서의 목적은 하나입니다:
**NUC가 없어도 블루투스 UI 작업을 계속하고, 테스트하고, 완성한 뒤 한 번에 올릴 수 있게 하는 것.**

앱/브릿지 쪽은 [`handoff_bluetooth_ui.md`](handoff_bluetooth_ui.md)를 보세요.
이 문서는 로봇 쪽입니다.

---

## 1. 호스트

| | |
| :--- | :--- |
| 호스트 / 사용자 | `mprp3` / `mprp3` |
| IP | `10.242.33.199` (네트워크마다 바뀜. 지금까지 모든 랩 서브넷에서 `*.199`였음) |
| OS | Ubuntu 20.04.6 LTS, 커널 5.15.0-139-generic |
| Python | 3.8.10 |
| ROS | Noetic (`/opt/ros/noetic`) |
| 블루투스 어댑터 | `7C:B5:66:E6:A8:A6`, 이름 **`mprp3`**, `DiscoverableTimeout` 180초 |
| 시리얼 | `/dev/ttyUSB0`, `/dev/ttyUSB1`. udev 규칙 `13-uart.rules`가 CP210x(`10c4:ea60`, serial `0001`)를 `/dev/uart`로, mode 0666 |
| UART | 115200 baud |
| `sudo` | **비밀번호 필요** — 모든 스크립트는 root 없이 동작하도록 작성할 것 |
| 폰 | Galaxy S24+ `SM-S926N`, Android 16 / SDK 36, adb serial `R3CX80806GP`, BT MAC `BC:93:07:CC:F4:9D` |

Windows에서 SSH:
`C:\Windows\System32\OpenSSH\ssh.exe -i C:\Users\npgy2\.ssh\id_rsa mprp3@<ip>`
(Git Bash 자체의 `ssh`는 키를 찾지 못합니다). 키 인증이 되어 있어 비밀번호가 없습니다.

---

## 2. 워크스페이스는 하나가 아니라 셋입니다

| 경로 | git | 내용 | 역할 |
| :--- | :--- | :--- | :--- |
| `~/wheelchair_localization_src` | ✅ `mnjn00/Uniconlab-autonomous-wheelchair`, remote 이름은 **`github`** | WP0 스캐폴드 포함 전체 저장소 | 체크아웃. **실행되는 것이 아님** |
| `~/livox_static_localization_ws` | ❌ git 없음 | `static_livox_localization`, `pgy_path_planner`, `fast_gicp` | **실제 필드 스택** |
| `~/catkin_ws` | ❌ git 없음 | `base_model`, `mpr_val`, `navigation`, `vectornav`, `topology_builder`, `webrtc_ros` | **하드웨어 계층** |

저장소의 `src/wheelchair_safety`와 `src/wheelchair_interfaces`는 WP0 계약 스캐폴드입니다.
`catkin_make`가 빌드하지만 **아무것도 launch하지 않습니다.** `/safety/state`,
`/cmd_vel_safe`, `/safety/estop`, `armed`, `reason_mask`, geofence,
`sidewalk`/`road_free_space`, `topology_guard`는 전부 그 스캐폴드의 것이고 실행 중인
시스템에는 존재하지 않습니다. `docs/interfaces.md`를 근거로 통합하지 마세요.

---

## 3. 실제로 휠체어를 움직이는 명령 체인

```
 follower           waypoint_follower.py (pursuit) | mpc_follower.py | dwa_follower.py
   │                                    static_livox_localization
   │  /cmd_vel_raw          geometry_msgs/Twist
   ▼
 safety_gate.py     INPUT_STALE / CLOUD_STALE / INPUT_INVALID / REVERSE /
   │                장애물 홀드 시 정지. 사유를 계산하고 로그로 남기지만
   │  /cmd_vel_gated  발행하지는 않음
   ▼
 tip_guard.py       경사/전복 보호. /tip_guard/status (String) 도 발행
   │  /cmd_vel
   ▼
 wheel_cmd_tmp.py   base_model. /cmd_vel 은 publisher callerid 가 "/tip_guard",
   │                /wheel_status 는 "/uart" 여야 받아들임.
   │  /wheel_cmd     위반 시 fault_latched -> 정지
   ▼
 uart.py            모드 게이트 + 0.6초 auto 모드 명령 기아 워치독
   │  UART 115200
   ▼
 모터 컨트롤러
```

### 주행 모드 — `/mode_cmd` (`std_msgs/Int16`)

| 값 | 의미 | `uart.py` 동작 |
| :--- | :--- | :--- |
| `65` `'A'` | **Auto** | 모터 정지 프레임 송신 후 `wheel_cmd` 전달 |
| `77` `'M'` | **Manual** | 모터 정지 프레임 송신 후 **모든 `wheel_cmd` 무시** |

`ModeCallback`은 **callerid를 검사하지 않습니다.** 외부 도구가 안전하게 쓸 수 있는
유일한 레버가 이것인 이유입니다. `CmdCallback`은 `self.mode == 65`일 때만 전달합니다.

**조이스틱을 움직이면 그 자체로 base가 auto 모드에서 빠집니다.** 이것이 문서화된
failsafe이고, follower는 한 제어 주기 안에 `MANUAL_MODE`로 홀드합니다.

### `/wheel_status` (`Int16MultiArray`) — 원본 UART 프레임

| 인덱스 | 의미 |
| :--- | :--- |
| `data[0]` | `72` = `'H'`, 프레임 헤더 |
| `data[1]` | **모터 컨트롤러가 되돌려주는 모드** (65/77) — 유일하게 신뢰할 수 있는 확인 |
| `data[7]` | 컨트롤러 상태 바이트 1개. **배터리가 아닙니다** — 값이 88/77 두 개뿐이고 주행에 따라 토글합니다 (2026-08-23 실측). `bridge_to_server.py`가 `wheel_battery`로 재발행하지만 용도는 미확인. `docs/handoff_bluetooth_ui.md` §6-1 참조 |
| `data[-3]` | 체크섬. 프레임은 `13, 10`으로 끝남 |

---

## 4. 토픽·서비스 목록

### UI가 읽을 수 있는 것

| 토픽 | 타입 | 생산자 | 의미 |
| :--- | :--- | :--- | :--- |
| `/wheel_status` | `Int16MultiArray` | `uart.py` | 모드 echo + 배터리 + 링크 생존 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | 실제 속도 |
| `/cmd_vel_raw` | `Twist` | follower | follower가 원하는 값 |
| `/cmd_vel_gated` | `Twist` | `safety_gate.py` | 게이트가 허용한 값 |
| `/cmd_vel` | `Twist` | `tip_guard.py` | 바퀴로 가는 값 |
| `/tip_guard/status` | `String` | `tip_guard.py` | 예: `STALE` |
| `/waypoint_follower/status` | `String` | `waypoint_follower.py` | `HOLD:…`, `WOULD_HOLD:…`, `… wp=n/N v=x.xx` |
| `/fast_lio_icp/localization_diagnostics` | `DiagnosticArray` | 측위 | 정상일 때 `message`가 `TRACKING` |
| `/fast_lio_icp/pose` | pose | 측위 | 현재 자세 |
| `/perception/objects_summary` | `String` | `obstacle_clusters.py` | 클러스터 요약. **침묵 ≠ 안전** |
| `/robot_fault` | `Int16MultiArray` | `mpr_val/fault_check.py` | `[scan, odom, imu, roll, pitch]` 불리언 |
| `/cloud_registered_body` | `PointCloud2` | FAST-LIO | 게이트의 장애물 입력 |
| `wheel_battery` | `Int16` | `bridge_to_server.py` | = `wheel_status[7]` |
| `battery_consumption` | `Int16` | `bridge_to_server.py` | 누적 소모량 |

### 명령 가능한 것

| 무엇 | 방법 | 비고 |
| :--- | :--- | :--- |
| 주행 모드 / E-STOP | `/mode_cmd`에 `Int16` 발행 | 65 auto, 77 manual/정지 |
| 주행 시작·정지 | `rosservice call /waypoint_follower/start "data: true\|false"` (`std_srvs/SetBool`) | **pursuit 프로파일만.** mpc·dwa에는 없음 |
| pgy_path_planner 변형 | `/start_drive`에 `Bool` 발행 | `path_follower_node_custom.py` 전용 |

### 절대 발행하면 안 되는 것

`/cmd_vel`, `/cmd_vel_gated`, `/cmd_vel_raw`, `/wheel_cmd`, `/wheel_status` —
callerid 검증 대상이거나 소유자가 있습니다. 끼어들면 휠체어가 fault stop에 빠집니다.

---

## 5. 기동 스크립트 (모두 `~` 아래)

| 스크립트 | 내용 |
| :--- | :--- |
| `start_wheelchair_localization.sh` | 전체 필드 기동: `livox_ros_driver2 msg_MID360.launch` → `base_model vectornav.launch` → `fast_lio` → `static_livox_localization moving_localization.launch` → `obstacle_clusters.py` → `base_model wheel.launch` → `safety_gate.py` → `tip_guard.py` → follower → `route_identity_publisher.py`. `PROFILE=pursuit\|mpc\|dwa`, `SHADOW_QA=1`이면 센서+측위만. 워크스페이스는 `LOCALIZATION_WS` (기본 `~/livox_static_localization_ws`) |
| `trial_0727.sh` | 0727 측위 시험: `SAFETY_POLICIES=false` (재량 가드 억제, 대신 `WOULD_HOLD:`로 보고), 클러스터 추적은 켬, 조이스틱이 failsafe. **armed 상태로 끝나며 주행은 안 함** |
| `start_motion.sh` | 이미 TRACKING인 측위 위에 모션 체인을 올리고 arm |
| **`go.sh`** | **주행 시작.** 아래 전제조건이 모두 만족되지 않으면 거부하고, 만족하면 `rostopic pub -1 /mode_cmd std_msgs/Int16 65` 후 `rosservice call /waypoint_follower/start "data: true"` |
| **`stop.sh`** | `rosservice call /waypoint_follower/start "data: false"` 후 `rostopic pub -1 /mode_cmd std_msgs/Int16 77`. 의도적으로 아무것도 확인하지 않음 — *"전제조건이 실패했다고 거부하는 정지는 정지가 아니다"* |
| `deploy_code.sh` | 저장소 → 워크스페이스 rsync + catkin build |

### `go.sh` 전제조건 — 브릿지의 `drive_start`가 그대로 따름

1. `/waypoint_follower` 노드가 `rosnode ping`에 응답
2. `/perception/objects_summary`가 생산 중 — *"조용한 생산자는 빈 객체 리스트로 주행하게
   두는데, 그건 텅 빈 도로와 똑같이 보인다"*
3. `/fast_lio_icp/localization_diagnostics`의 `message`가 정확히 `TRACKING`

### 경로 파일

`~/wheelchair_localization_src/routes/` — `20260727_new_route_waypoints.json` +
`20260727_new_route_safety_band.json`이 `start_motion.sh`가 쓰는 한 쌍이고,
`20260727_chair_centred_*`가 0727 시험용, 그리고 `*_no_go_zones.json`이 있습니다.

---

## 6. NUC의 블루투스

- 어댑터에 `discoverable on` / `pairable on`이 필요하며 **둘 다 sudo 없이 됩니다.**
- **discoverable은 180초 뒤 자동으로 꺼지고, 꺼지면 `hci0`에서 `ISCAN` 플래그가 사라져
  폰 목록에 아예 뜨지 않습니다.** `hciconfig hci0`의 3번째 줄에 `ISCAN`이 있는지로 판별.
- **페어링 에이전트가 없으면 페어링 요청이 거절됩니다.** `bluetoothctl -- <cmd>` 형태는
  실행 직후 프로세스가 끝나 에이전트가 사라집니다. `scripts/nuc_bluetooth_pair.sh`가
  stdin을 열어둔 채 bluetoothctl을 상주시켜 해결합니다.
- 안드로이드는 SSP numeric comparison을 요구하므로 `Confirm passkey NNNNNN (yes/no)`에
  `yes`를 보내야 하고 폰에서도 같은 숫자를 확인해야 합니다.
- **SDP:** `bluetoothd`가 `--compat` 없이 돌아 `sdptool add`가 동작하지 않고,
  `python3-bluez`도 설치돼 있지 않습니다(root 필요). 대신 브릿지가 D-Bus로
  `org.bluez.Profile1`을 등록합니다 — **사용자 `mprp3` 권한으로 됩니다.** 실제 SPP
  레코드가 발행되고 bluetoothd가 RFCOMM listen을 소유합니다. 확인: 브릿지가 도는 동안에만
  `bluetoothctl show`에 `UUID: Serial Port (0x1101)`이 나타납니다.
- `python3-dbus` 1.2.16과 `gi.repository.GLib`이 설치돼 있습니다.
- raw `bind(("", 1))`은 리눅스에서도 *bad bluetooth address*로 실패합니다.
  `00:00:00:00:00:00`을 쓰세요.

---

## 7. 오프라인 작업 → 재배포

### NUC 없이 테스트 가능한 것

```bash
# 프로토콜·가드·프레이밍 — 무선도 ROS도 필요 없음
python3 scripts/ros1_bluetooth_bridge.py --self-test
```

리눅스 머신에 `roscore`만 있으면 전체 라이브 검증도 가능합니다. `/wheel_status`,
`/Odometry`, `/cmd_vel_raw`, `/cmd_vel_gated`, `/perception/objects_summary`,
`/waypoint_follower/status`에 픽스처를 발행하고 `/mode_cmd`를 구독한 뒤,
`--debug-tcp 8765`로 브릿지를 두드리면 됩니다. `uart.py`를 띄우지 않으므로 모터에는
아무것도 갈 수 없습니다. `handoff_bluetooth_ui.md` §6의 23개 검증이 정확히 이 방식입니다.

바로 쓸 수 있는 픽스처: [`nuc_snapshot/live_test_fixture.py`](nuc_snapshot/live_test_fixture.py)

중요한 픽스처 형태:

- `/wheel_status`: `data = [72, <65|77>, 0, 0, 0, 0, 0, <배터리>]`
- 측위 `message`는 정확히 `TRACKING` 문자열이어야 함
- 게이트 홀드 = `/cmd_vel_raw`가 0이 아닌데 `/cmd_vel_gated`가 0

### NUC가 돌아왔을 때 재배포

```bash
scp scripts/ros1_bluetooth_bridge.py scripts/nuc_bluetooth_check.sh \
    scripts/nuc_bluetooth_pair.sh scripts/nuc_bridge_restart.sh \
    mprp3@<ip>:~/wheelchair_localization_src/scripts/

ssh mprp3@<ip> 'cd ~/wheelchair_localization_src && bash scripts/nuc_bluetooth_check.sh --fix'
ssh mprp3@<ip> 'bash ~/wheelchair_localization_src/scripts/nuc_bridge_restart.sh --allow-commands'
```

주의할 함정 두 가지:

- 백그라운드 실행은 `setsid … </dev/null &`로 분리하세요. 단순 `nohup … &`는 SSH 채널을
  붙잡아 명령이 멈춘 것처럼 보입니다.
- NUC 스크립트를 `set -u`로 감싸지 마세요. ROS의 `setup.bash`가 미설정 `ROS_DISTRO`를
  참조해 중단됩니다.

### 실제 휠체어에서 가장 먼저 확인할 것

픽스처는 모터 컨트롤러가 `77`을 echo하는 것을 *흉내* 냈습니다. 실물로 확인하세요.

```bash
rostopic echo /wheel_status        # 앱에서 E-STOP 후 data[1] 이 77 로 바뀌는지
rostopic echo /waypoint_follower/status
```

---

## 8. 이 시스템에서 앱이 할 수 있는 것과 없는 것

**할 수 있는 것:** 실제 속도(`/Odometry`), 배터리(`wheel_status[7]`), 모터 컨트롤러가
확인해준 주행 모드, 게이트 홀드, tip guard·측위 상태, 장애물 요약, 센서 결함, follower
진행 상황 표시. E-STOP 발동, 가드를 통과한 해제, pursuit 프로파일에서의 주행 시작·정지.

**할 수 없는 것과 그 이유:**

| | |
| :--- | :--- |
| 조향·속도 지정 | `/cmd_vel`이 `/tip_guard`로 callerid 잠김. 앱은 조종기가 아니라 감시자 |
| "단계(step)" 설정 | 그런 개념이 존재하지 않음 |
| 게이트 홀드 *사유* 확인 | `safety_gate.py`가 계산은 하지만 발행하지 않음 — 추론만 가능 |
| mpc·dwa에서 주행 시작·정지 | 해당 follower들이 서비스를 제공하지 않음 |
| 경로 재계획 | 경로는 노드 시작 시 읽는 JSON 파일 |

---

## 9. 나중에 메울 만한 구멍

- `safety_gate.py`가 `blocked_reason`을 `String` 토픽으로 발행하면, 브릿지의 추론을
  진짜 값으로 대체할 수 있습니다. 두 줄짜리 변경입니다.
- `~/livox_static_localization_ws`와 `~/catkin_ws` **둘 다 git 관리가 아닙니다.**
  필드 코드가 한 머신의 한 곳에만 존재합니다. 이게 가장 큰 위험입니다.
- SPP는 페어링 외 인증이 없습니다. 본딩된 기기면 `estop`(안전한 방향)뿐 아니라
  `drive_start`도 보낼 수 있습니다. 사람이 타기 전에 PIN을 검토하세요.
- NUC에 `git-lfs`가 없습니다. `GIT_LFS_SKIP_SMUDGE=1`을 쓰세요.

---

## 10. 원본 캡처

2026-08-14에 NUC에서 그대로 받아온 덤프가 [`nuc_snapshot/`](nuc_snapshot/)에 있습니다.
이 문서의 주장을 NUC 없이도 검증할 수 있습니다.

| 파일 | 내용 |
| :--- | :--- |
| `host_env.txt` | OS·커널·Python·ROS·어댑터·시리얼 장치·udev 규칙·워크스페이스/패키지 목록 |
| `shell_scripts.txt` | `stop.sh`, `trial_0727.sh`, `preflight.sh`, `go_mpc.sh`, `drive_tonight.sh`, `resume_motion_chain.sh` 전문 |
| `ros_interfaces.txt` | 두 워크스페이스의 모든 `Publisher`/`Subscriber`/`Service`/`ServiceProxy` |
| `battery_and_status.txt` | `wheel_battery`/`battery_consumption`/`robot_fault` 출처, `/waypoint_follower/status`, `/start_drive` |
| `status_codes.txt` | `wheel_status` 프레임 해석, 결함 플래그 순서, follower·tip guard 상태 문자열 |
| `live_test_fixture.py` | 23개 라이브 검증에 쓴 픽스처와 단언. 아무 `roscore`에서나 실행 가능 |
