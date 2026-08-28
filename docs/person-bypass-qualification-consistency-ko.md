# 사람 우회 확인 일관성 수정 (GitHub 검토용 / NUC 미배포)

기준 커밋: `4241607` (`950cbc4`에 raw gate 후보 사전검사를 추가한 별도 배포본).
다른 실행본이나 최신 main을 병합하지 않는다.

## 근거와 변경

2026-08-28 13:35~13:36 우리 수정본 세션에서:

- 사람 정지 검사는 좌우 0.65m, follower의 최초 사람 선택은 좌우 0.55m였다.
  track 5584의 측면 점군 프로파일(y0=0.6m)은 정지 검사에만 포함됐다.
  `PERSON`은 유지되는데 `NEAREST_THREAT_NOT_PERSON`으로 확인 시간이
  초기화되어 약 21초 동안 출발하지 못했다.
- track 5656은 3초 정지 확인 후 허가를 받았으나, 가까운 일반 객체가
  새로 선택되면서 약 0.59초 뒤 기존 사람의 우회 허가가 취소됐다.
- 그 뒤에는 여러 사람/미확정/관측 소실도 있었다. 이것까지 무조건
  우회하도록 바꾸지는 않는다. 검출 점 감소나 ID 변화의 원인을 해결하는
  추적기 수정도 이번 범위가 아니다.

수정 내용:

1. `observed_person_permit()`는 좁은 `corridor_threat()`의 승자 대신
   기존 우회 관찰 영역의 사람을 직접 확인한다. 3초·동일 ID·static·
   최신 관측·거리·위치 점프·TRACKING 조건은 그대로 적용한다.
2. 다른 객체가 더 가까워져도 사람의 확인 시간을 초기화하거나 일반 객체
   permit으로 덮어쓰지 않는다. 확인 중인 사람이 있으면 WAIT, 확인이
   끝나면 사람 geometry와 후보 사전검사를 포함한 DWA를 실행할 수 있다.
   다른 가까운 moving/unknown 객체 또는 기본 WAIT 판단은 여전히 정지한다.
   사람 permit이 있다는 것만으로 명령 발행을 허용하지 않는다.
3. semantic supervisor의 permit 검증도 동일한 측면 유지 여유
   `person_bypass_lateral_hysteresis_m`를 사용한다. 신규 획득 범위는
   qualifier가 제한하며, 여유 영역은 기존 동일 track 유지에만 적용된다.
   launcher는 follower와 supervisor에 같은 값을 전달한다.
4. semantic 상태에 `person_bypass_permit_reason`을 추가하여 정지 중
   우회 자격 확인 상태를 볼 수 있게 한다.

기존 0.65m 사람 정지 범위를 줄이지 않는다. 대신 이 범위에 포함되는 사람을
더 넓은 기존 우회 관찰 영역에서 누락하지 않는다. 관찰 영역에 미확정 또는
여러 사람이 있으면 기존 좁은 query가 clear여도 보수적으로 WAIT한다.

## 그대로 유지하는 조건

- 사람 moving/unknown, 관측 소실/지연, ID 변경, 큰 위치 점프, 여러 사람,
  너무 가까운 사람, learned-only, localization 비정상: 우회 허가 불가.
- raw safety gate의 현재 차체/이동 중 차체/요청 궤적 충돌 검사 및 DWA
  후보 사전검사: 변경 없음.
- 지도·웨이포인트·band·로컬라이제이션·바퀴 제어·속도 설정·분류기: 변경 없음.
- PAUSED에서 확인 시간은 누적 가능하지만 자동 출발하지 않음.

## 검증과 제한

`test_dwa_policy.py`에 실제 기록의 track 5584 위치/프로파일을 재구성한
회귀 사례를 포함했다. 좁은 query에서 놓치는 경우의 우회 자격 획득,
일반 장애물에 의해 확인 시간이 초기화되지 않는 경우, 다른 moving/unknown
장애물의 WAIT 유지, 관측/ID/움직임 변화 시 허가 취소, supervisor 측면
유지 범위, PAUSED에서 비주행, DWA에 geometry/사전검사 전달을 검증한다.

이는 호스트의 단위/연결 테스트이며 rosbag 전체 재생이나 실제 주행 성공의
증거는 아니다. GPU wrapper 테스트도 실제 RTX 실측이 아닌 호스트 테스트다.
NUC의 빌드·프로세스·ROS 파라미터는 변경하지 않는다.

검증: DWA/자격/semantic/회피/mask/GPU wrapper 119개, hybrid/terrain/scan/
설정 연결 53개, ROS import stub을 사용한 raw gate 후보 검사 12개.
수정 Python 파일의 Python 3.8 문법 및 launcher bash 문법도 검사한다.

## 적용 시 주의

이 브랜치는 GitHub 업로드만으로 NUC에 자동 적용되지 않는다. 포함된 GitHub
Actions에는 NUC 자동 배포 단계가 없다. 현재 실행 중인 소스 경로에 파일만
덮어쓰지 말고, 추후 명시적인 배포 요청 시 별도 배포본에서 관련 파일을
함께 반영하고 정지 상태의 검증을 진행해야 한다.
