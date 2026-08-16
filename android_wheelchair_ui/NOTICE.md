# NOTICE — 이 안드로이드 모듈의 원저작물 고지

## 원저작물

이 모듈은 아래 저작물을 **수정한 것**입니다.

| | |
| :--- | :--- |
| 프로젝트 | `edge-mobility-monitor` |
| 저작권자 | **박형준 (Park Hyeongjun)** |
| 저작권 | © 2026 Park Hyeongjun. All rights reserved. |
| 원본 | <https://github.com/Geppetto0608/edge-mobility-monitor> |
| 라이선스 | 독점(proprietary) — "all rights reserved" |

자바 패키지명 `com.example.jetsonbtmonitor`, 액티비티 구조(`MainActivity`,
`WifiSetupActivity`, `LoginActivity`, `RegisterActivity`, `SettingsActivity`,
`CameraActivity`), 텔레메트리 파서, 프리셋 모델, 대시보드 레이아웃은 모두 박형준 님의
작업물입니다.

## 사용 허락

공개된 `LICENSE` 파일 자체는 아무 권리도 부여하지 않습니다.
**박형준 님이 이 인턴 프로젝트에 대해 직접, 대면으로 사용·수정을 허락**하셨고
조건은 출처 표기입니다. 2026-08-14 기록.

허락이 현재 **구두**입니다. 카카오톡 메시지나 메일 한 줄이면 충분하니 글로 받아
`PERMISSION.md`로 이 옆에 저장해 두세요. 그래야 이 저장소를 이어받는 사람이 그 자리에
없었어도 허락 사실을 확인할 수 있습니다. 요청 문구 예시:

> "[프로젝트명] 에서 edge-mobility-monitor 를 수정·사용하는 것을 허락합니다.
>  출처 표기 조건. 2026-08-14, 박형준"

구두 허락에는 아마 포함되지 않았을 항목들이라 따로 확인해 두면 좋습니다.

- 수정된 모듈을 **공개 저장소에 push**하는 것까지 포함되는가?
- **논문·포스터·포트폴리오**에 UI를 싣는 것은?
- 랩 외부로 **APK를 배포**하는 것은?

## 출처 표기 — 유지해야 할 것들

- [x] 이 NOTICE 파일 — 모듈의 모든 사본에
- [x] `scripts/ros1_bluetooth_bridge.py` 헤더의 출처 표기
- [x] `docs/handoff_bluetooth_ui.md`의 저작권 절
- [ ] 이 UI가 나오는 **포스터·발표자료·시연영상·보고서**의 크레딧 한 줄:

      > UI는 박형준(Park Hyeongjun) 님의 *edge-mobility-monitor*를 기반으로 하며,
      > 허락을 받아 사용합니다.
      > https://github.com/Geppetto0608/edge-mobility-monitor

- [ ] 공개 remote에 올릴 경우, 원본 `LICENSE` 전문을 함께 두고 README에
      "안드로이드 계층은 허락을 받아 사용한 파생 저작물"임을 명시

## 이 트리에서 우리 것

이 프로젝트를 위해 새로 작성했고 원저작물에서 파생되지 않은 것들입니다.

- `scripts/ros1_bluetooth_bridge.py` — ROS 1 / RFCOMM 브릿지, BlueZ SPP 프로파일 등록,
  안전 토픽 연동, self-test 하네스
- `scripts/nuc_bluetooth_check.sh` — NUC 블루투스 사전 점검
- `scripts/nuc_bluetooth_pair.sh` — 페어링 모드 스크립트
- `scripts/nuc_bridge_restart.sh` — 브릿지 재시작
- `docs/handoff_bluetooth_ui.md`, `docs/nuc_system_reference.md`, `docs/nuc_snapshot/`

이것들은 우리가 자유롭게 라이선스할 수 있지만, 안드로이드 모듈은 그렇지 않습니다.

## 참고: 이 디렉터리 자체에 대해

`android_wheelchair_ui/`는 **배포된 앱의 백업이 아닙니다.** `com.uniconlab.wheelchair.ui`
패키지의 오래된 3파일 스켈레톤이고, 아직 Wi-Fi HTTP 클라이언트를 쓰고 있어
`edge-mobility-monitor/` 쪽과 많이 벌어져 있습니다. 복원 지점으로 쓰지 마세요.

다만 만약 허락 범위 문제로 **깨끗한 재구현**이 필요해지면, 패키지명이 이미 우리 것이고
표면적이 작아서 여기서 출발하는 게 자연스럽습니다. 그 경우 통신 프로토콜(인터페이스이지
표현이 아님)은 유지하고 안드로이드 쪽만 새로 작성하면 됩니다.

머신별 SDK 경로가 담긴 `local.properties`가 커밋돼 있으니 삭제하고 gitignore에 넣으세요.
