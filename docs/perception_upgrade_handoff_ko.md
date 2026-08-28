# 인지 증설 구현 핸드오프 (2026-08-25)

이전 세션이 사용량 한도로 종료되며 남긴 인수인계 문서다. 후속 에이전트는
이 문서와 명세(`docs/perception_upgrade_spec_ko.md`)만으로 작업을 이어받을
수 있어야 한다.

## 0. 한 줄 요약

명세의 1단계(Patchwork++ 지면 분리)와 2단계(추적기 교체)를 두 서브에이전트가
각자 워크트리에서 TDD로 구현하던 중 프로세스가 죽었다. **두 워크트리 모두
"테스트 먼저" 커밋은 완료, 구현은 미커밋 상태로 남아 있으며, 그 미커밋
구현으로 패키지 테스트가 전부 그린임을 확인했다.** 남은 일은 명세 대비
완성도 검증 → 커밋 → 두 브랜치 병합 → 통합 테스트 → 보고다.

## 1. 작업 산출물 위치

기준 커밋: **origin/main = 97c14bf** (NUC 배포본과 동일 계열).
메인 작업 트리(`/Users/minjun/unicon-wheelchair`, 브랜치
`relax/tracking-thresholds`)는 main보다 44커밋 뒤이고 **다른 세션의 미커밋
수정이 들어 있다. 절대 건드리지 말 것.** 구현은 전부 아래 워크트리에서만.

### 워크트리 A — 1단계 Patchwork++ (브랜치 `feat/ground-segmentation`)

```
/private/tmp/claude-501/-Users-minjun-unicon-wheelchair/3b839acf-1070-4c4a-80f8-ece41e8b9273/scratchpad/wt-stage1
```

- 커밋됨: `d8f7c70` "test: pin ground segmentation node, fallback selector,
  and consumer reversion"
- 미커밋: 신규 `scripts/ground_segmentation.py`, `scripts/ground_seg_fallback.py`,
  수정 `scripts/obstacle_clusters.py`, `scripts/safety_gate.py`,
  `CMakeLists.txt`, `test/test_reflection_filter.py`
- 검증 실측 (2026-08-25): rospy 의존 4개 모듈 제외 시
  **607 passed, 3 skipped, 0 failed**

### 워크트리 B — 2단계 추적기 (브랜치 `feat/tracker-upgrade`)

```
/private/tmp/claude-501/-Users-minjun-unicon-wheelchair/3b839acf-1070-4c4a-80f8-ece41e8b9273/scratchpad/wt-stage2
```

- 커밋됨: `51f2da6` "test: pin stage 2 tracker behaviour before replacing
  the tracker"
- 미커밋: `scripts/cluster_tracking.py` 재작성(+387/-77 규모),
  `scripts/obstacle_clusters.py` 요약 필드 확장
- 검증 실측 (2026-08-25): 동일 제외 조건에서 **576 passed, 0 failed**

## 2. 테스트 실행법 (함정 주의)

이 Mac에는 ROS가 없다. 다음 4개 테스트 모듈은 `import rospy`를 모듈
최상단에서 하는 스크립트를 물고 있어 수집 단계에서 죽는다. 제외하고 돌린다:

```
python3 -m pytest src/static_livox_localization/test/ -q \
  --ignore=src/static_livox_localization/test/test_approach_cap.py \
  --ignore=src/static_livox_localization/test/test_gate_status.py \
  --ignore=src/static_livox_localization/test/test_obstacle_preview.py \
  --ignore=src/static_livox_localization/test/test_stop_watchdog.py
```

- 위 4개는 NUC(ROS 환경)에서만 돌릴 수 있다. 회귀 기준선은 위 실측 수치다.
- 저장소 루트 `tests/`는 main 기준 **13건 기존 실패**가 있다. 새 실패를
  더하지 않는 것이 기준이며, 13건을 고치려 들지 말 것(다른 트랙 소관).
- wt-stage1 안에 python3.9 `.venv`가 있는데 rospy 스텁이 아니다. 무시하고
  시스템 python3를 쓰면 된다.

## 3. 이어받을 작업 (순서대로)

1. **명세 대비 완성도 검증.** 두 워크트리의 미커밋 구현을
   `docs/perception_upgrade_spec_ko.md` §3(1단계)·§4(2단계)와 조목조목
   대조한다. 특히 주의:
   - 명세가 에이전트 기동 이후에 갱신되어 **킬스위치 파라미터 요구가
     추가됐다**(§2 원칙 2): 신규 노드 `~enabled`, 소비자 전환
     `~nonground_input`, (3단계용 `~learned_merge`). 구현에 없으면 추가할 것.
     "끄면 기동 전 상태와 동일"이 합격 기준.
   - 1단계: 폴백은 신선도 비교(0.5 s)의 순수 함수 공유 구현이어야 하고,
     `safety_gate`의 `status_report()`에 `GROUND_SEG_STALE`가 떠야 한다.
     pypatchworkpp 부재 시 발행 금지(패스스루 발행은 반려 사유).
   - 2단계: `Tracker` 공개 API 유지, UNKNOWN→moving 원칙 유지,
     person 진입 1프레임/이탈 다수결의 비대칭 히스테리시스,
     요약 JSON 기존 필드 불변(하위호환).
2. **각 워크트리에서 구현 커밋.** conventional commits(feat:/fix:/test:),
   이모지 금지, 로컬 테스트 그린 확인 후.
3. **병합.** origin/main에서 통합 브랜치(예: `feat/perception-stage12`)를
   만들어 두 브랜치를 순서대로 병합. 충돌은 `obstacle_clusters.py` 한
   파일에 국한될 것이다 — A는 입력(구독·폴백)쪽, B는 출력(추적기 호출·요약
   생성)쪽만 만지도록 경계를 그어놨다. 병합 후 전체 테스트 재실행.
4. **보고.** 파일별 변경 요약, 테스트 수치(기준선 대비), 명세와 달라진
   설계 판단, NUC 배포 시 필요한 것(아래 §5)을 사용자에게 보고.

## 4. 절대 규칙 (이전 세션에서 사용자와 합의된 것)

- **푸시는 사용자가 명시적으로 시킬 때만.** 커밋까지만 하고 대기.
- **주행 시작 금지.** NUC ssh 접속(`ssh nuc-tb`, 핫스팟 시 ZeroTier
  10.222.0.150)도 이 작업에는 불필요 — 배포는 사용자 지시 후 별도 단계.
- 메인 작업 트리와 다른 세션의 파일(`scripts/ros1_bluetooth_bridge.py`,
  `scripts/nuc_bridge_restart.sh`, `tools/bt_bridge_fixture.py`) 불가침.
- `waypoint_follower.py`의 `person_memory` 제거는 범위 밖(명세 §4.3 —
  추적기 실주행 검증 뒤에만).
- 코드 스타일: 기존 파일 수준의 영어 docstring/주석(측정 근거를 설명),
  이모지 금지, 파일 800줄 이하.

## 5. 병합 이후, NUC 배포 전 필요한 것 (미착수)

- NUC에 `pypatchworkpp` 빌드·설치 (Ubuntu 20.04 / py3.8; CMake 3.16이
  모자라면 pip cmake). 절차와 산출물을 저장소에 기록할 것 (명세 §3.5).
- 배포 경로: NUC에는 GitHub 자격증명이 없다 — Mac에서 push 후 NUC에서
  pull이 아니라, 기존 방식은 `git bundle` → scp였다. 세부는 메모리
  `wheelchair-nuc-setup` 참조.
- 배포 후에는 명세 §2 원칙 3에 따라 **섀도 모드 주행 먼저**(1단계는
  소비자 전환 OFF로 발행만, 2단계는 요약 필드만 추가된 상태로 관찰).
- rospy 의존 4개 테스트 모듈은 NUC에서 실행해 확인.
- 3단계(학습 검출기, 명세 §5)는 완전 미착수. 1·2단계 검증 뒤 별도 착수.
