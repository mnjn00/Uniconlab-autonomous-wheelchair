# UNICON 자율주행 휠체어 상용화 준비도 의사결정

## Meta
- **Topic**: 상용화 준비도 전수조사에 기반한 출시 여부와 다음 검증 단계 결정
- **Target Audience**: 경영·제품 책임자, 개발 리더, 안전·품질·보안·규제 담당자, 현장 운영 책임자
- **Tone/Mood**: 차분하고 근거 중심이며, 강점과 한계를 함께 밝히는 의사결정형 보고
- **Slide Count**: 14 slides
- **Aspect Ratio**: 16:9
- **style**: ppt-consulting-precision-grid

## Slide Composition

### Slide 1 - 오늘 결정할 두 가지
- **Type**: Cover
- **Title**: 승객·캠퍼스 출시는 보류하고, 제한된 검증 프로그램만 진행해야 합니다
- **Subtitle**: 승객·캠퍼스 출시는 보류하고, 근거 중심의 개선 프로그램은 진행합니다
- **Key Message**: 현재 출시는 NO-GO, 제한된 검증·개선 작업은 GO가 권고안입니다.
- **Details**:
  - 지금 판단할 것은 “출시 여부”와 “다음 검증 투자 범위”입니다.
  - 일정이나 시연 성공이 아니라 사전에 정한 근거가 다음 단계 진입을 결정합니다.
- **Visual Treatment**: 상단에 한 문장 결론, 하단에 대상 라벨이 있는 두 카드. **sole-blue target:** `출시 권한 — NO-GO 출시 보류` 카드 하나만 파랑; `검증 프로그램 — GO 검증 프로그램` 카드는 회색. 나머지는 흰색·회색조로 두고 빨강·초록·주황·두 번째 accent를 금지합니다. 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/decision-story.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`

### Slide 2 - 현재 단계는 상용 제품이 아닙니다
- **Type**: Executive Summary
- **Title**: 강한 소프트웨어 RC와 실차 연구 프로토타입이 함께 있습니다
- **Key Message**: 안전 계약은 강하지만, 실제 승객 운행을 승인할 하나의 검증된 제품 체계는 아직 없습니다.
- **Details**:
  - 소프트웨어 RC 계약·fail-closed 구조는 강점입니다.
  - 실차 통합은 연구 프로토타입, 모터·제동·E-stop은 미검증 상태입니다.
  - `hardware_motion_authorized=false`와 `passenger_operation_authorized=false`를 유지해야 합니다.
- **Visual Treatment**: `강점 / 연구 단계 / 미검증`의 3열은 모두 회색조로 구성합니다. **sole-blue target:** `상용 제품 아님·승객 운행 미승인` verdict 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/REPORT.html`

### Slide 3 - 보존할 안전 기반은 분명합니다
- **Type**: Evidence Summary
- **Title**: 이 프로젝트는 빈 데모가 아니라, 이어서 검증할 가치가 있는 기반입니다
- **Key Message**: 잘 만든 fail-closed 정책과 소프트웨어 검증 자산은 폐기하지 말고 실제 제품 경로로 연결해야 합니다.
- **Details**:
  - 증거가 없으면 권한을 닫고, 맵 해시와 시작 상태를 확인한 뒤에만 동작합니다.
  - 장애물 제거만으로 자동 재개하지 않고, 명시적 재개와 취소 확인을 요구합니다.
  - mapped drop 밴드, incident 기록 기능·구조, release 계약, 넓은 호스트 테스트가 있습니다.
- **Visual Treatment**: `권한 차단 / 명시적 재개 / 증거 기록 기능·구조`의 3개 강점 카드는 회색조로 구성합니다. **sole-blue target:** `보존 후 실제 제품 경로로 통합` 결론 한 줄만 파랑으로 두고 별도 boxed module은 만들지 않습니다. 빨강·초록·주황·두 번째 accent를 금지하며, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 4 - 강한 안전 경로가 실제 모터까지 이어지지 않습니다
- **Type**: Process Diagram
- **Title**: 소프트웨어 RC 경로와 현재 실차 경로가 분리돼 있습니다
- **Key Message**: 시뮬레이션에서 확인한 STOP이 현재 실차 모터의 물리 정지로 이어진다고 아직 증명할 수 없습니다.
- **Details**:
  - 강한 WP0/RC 경로는 `/cmd_vel_safe` 뒤 simulation 또는 read-only shadow에서 끝납니다.
  - 현재 field 경로는 별도 gate를 거쳐 저장소 밖 `base_model/wheel.launch`로 갑니다.
  - 두 경로를 하나의 서명·해시 고정 권한 체인으로 합치는 일이 첫 목표입니다.
- **Visual Treatment**: 두 수평 lane은 모두 회색조로 그리고 설명은 각 1줄로 제한합니다. **sole-blue target:** 외부 `base_model` 앞의 검증 단절점 edge/label 하나. 두 lane을 다른 색으로 구분하지 않고 빨강·초록·주황·두 번째 accent를 금지하며, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-field-authority-graph.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`

### Slide 5 - 최종 모터 명령을 하나의 체인으로 통제해야 합니다
- **Type**: Risk and Closure
- **Title**: 저장소 밖 구동부의 실제 안전 동작은 검증 근거에 포함돼 있지 않습니다
- **Key Message**: 최종 actuator까지 단일 권한과 물리 정지를 측정하기 전에는 motion authority를 열 수 없습니다.
- **Details**:
  - **P0-1**: 실제 모터 구동기(driver), 바퀴 회전 센서 기반 폐루프 제어(encoder closed-loop), 장치 자체 정지 감시기(native watchdog), 독립 E-stop, 수동 우선권, 정지거리가 검증되지 않았습니다.
  - 외부 base 내부 기능은 “없음”이 아니라 “검사한 근거에서 알 수 없음”으로 표시해야 합니다.
  - `/cmd_vel_safe`부터 최종 모터 구동부(actuator)까지 명령 발행자(publisher) 1개, 변환기(transformer) 1개, 구동기(driver) 1개와 고장별 zero latency를 증명해야 합니다.
- **Visual Treatment**: 최대 2 modules만 사용합니다: 단일 권한 chain 1개와 `확인됨 / 알 수 없음 / 통과 조건`을 묶은 3-row ledger 1개. **sole-blue target:** ledger의 `통과 조건` row 하나만 파랑; chain과 나머지 row는 회색조. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-field-authority-graph.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-security-counter.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 6 - 현재 0727 경로는 자체 안전 규칙을 통과하지 못합니다
- **Type**: Statistics
- **Title**: 재현된 경로 결함은 승객 운행 승격을 막습니다
- **Key Message**: 현재 bundle은 0.10 m 허용 여유에서도 기록한 중심과 연결 이동선이 허용 구역 안이라는 자체 규칙을 만족하지 못합니다.
- **Details**:
  - **P0-2**: 0.10 m 허용 여유(grace)에서 371개 band 기준점(station) 중 center 35개 거부, usable interval 2개 역전, 두 점 사이 이동선(chord) 42개 실패가 재현됐습니다.
  - 75개 waypoint 중 9개가 band 밖이고, route의 두 점 사이 이동선 17개가 실패했습니다.
  - 이 결과는 승격을 막지만 실제 kerb crossing이 일어난다는 직접 증거는 아닙니다.
  - 모든 center·chord의 통과와 양방향 geometric replay 뒤에만 빈 휠체어 closed course를 검토합니다.
- **Visual Treatment**: 정확히 3 modules로 구성합니다: `band 기준점 숫자`, `route 숫자`, `의미·경계·다음 조건`을 합친 모듈. **sole-blue target:** `현재 bundle 승격 불가` 결론 하나만 파랑; 모든 숫자와 나머지 설명은 회색조. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-route-bundle.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 7 - 검사한 궤적과 실제 명령 궤적이 다를 수 있습니다
- **Type**: Technical Exhibit
- **Title**: 안전 검사는 바퀴에 최종 전달되는 명령과 같은 궤적을 확인해야 합니다
- **Key Message**: 다른 곡선을 검사하고 다른 곡선을 명령하면 “검사 통과”를 물리 안전 근거로 쓸 수 없습니다.
- **Details**:
  - **P0-3**: emitted radius 0.25 m와 evaluated radius 1.375 m가 달라질 수 있습니다.
  - 요청 yaw 0.5 rad/s가 integer wheel count 뒤 0.2058 rad/s로 복원된 값은 tracked wheel guard 재현 결과이며, 이 guard의 실제 field 배포는 미확정입니다.
  - gate → governor → wheel quantization → firmware 전체가 같은 geometry를 쓰고, 실제 executable을 HIL에서 대조해야 합니다.
- **Visual Treatment**: 동일 시작점에서 갈라지는 두 arc와 yaw 요청·복원 caveat를 하나의 dominant exhibit로 구성합니다. **sole-blue target:** 바퀴에 최종 전달되는 명령 궤적 하나만 파랑; 비교 궤적과 caveat는 회색조. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-command-geometry.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-git-regression.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 8 - 위치와 위험 인식의 공백도 남아 있습니다
- **Type**: Two-Column Risk Exhibit
- **Title**: 잘못된 위치 확정, 점이 적게 잡히는 작은 장애물, 새 낙차를 제품 수준으로 검증하지 못했습니다
- **Key Message**: mapped drop 방어와 위치 gate는 있지만, 현재 field 경로의 인식·정합 공백은 승객 운행을 막습니다.
- **Details**:
  - **P0-4**: rider/self mask에 z bound가 없고, retained point 5개 미만이면 clear가 될 수 있으며 live negative-hazard channel이 없습니다.
  - **P0-5**: strict 승인 실패 뒤 약한 후보가 시도될 수 있고, stale cloud·전체 deadline·ground-truth holdout 근거가 부족합니다.
  - 3D mask 보정, coverage fail-closed, live drop·동적 TTC, strict candidate와 실제 NUC 성능 기준이 필요합니다.
- **Visual Treatment**: `사람·장애물·낙차`와 `위치·잘못된 위치 확정` 두 column은 회색조로 구성합니다. **sole-blue target:** `두 공백 모두 승객 운행 차단` 공통 결론 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-negative-hazards.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-localization-attempts.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 9 - 큰 지도 자체는 핵심 blocker가 아닙니다
- **Type**: Before-and-After Statistics
- **Title**: 가벼운 주행용 지도 선택은 타당하지만, 실제 차량 컴퓨터 검증은 남았습니다
- **Key Message**: 지도 형식을 전면 교체하기보다 재현 가능한 bundle과 target NUC 성능부터 증명해야 합니다.
- **Details**:
  - canonical PLY는 **594,886,982 B / 37,180,425 points**의 보관 정본입니다.
  - runtime은 exact-hash **0.20 m PCD, 43,141,936 B / 2,696,359 points**를 사용하며 point 수가 92.75% 줄었습니다.
  - 이 축소 선택은 타당하지만 실제 NUC의 RAM·열·fallback·60분/8시간 적합성은 미측정입니다.
  - signed bundle, deterministic reproduction, 중복 적재 제거 뒤 필요할 때만 out-of-core를 검토합니다.
- **Visual Treatment**: `canonical → runtime` 흐름과 두 exact-number card는 모두 회색조로 구성합니다. **sole-blue target:** `실차 NUC 적합성 미증명` label 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-large-map-ops.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/evidence-translator.md`

### Slide 10 - 테스트는 소프트웨어 기반을 증명했을 뿐입니다
- **Type**: Evidence Ledger
- **Title**: 많은 소프트웨어 테스트 통과는 실차 안전 승인을 뜻하지 않습니다
- **Key Message**: 현재 수치는 저장소·호스트 범위의 강점을 보여 주지만, 실제 휠체어의 안전·인허가·승객 운행을 증명하지 않습니다.
- **Details**:
  - **P0-6**: canonical field bringup이 불완전하고 clean-checkout 기준선도 완전한 green이 아닙니다.
  - 365/365 tracked paths 분류, targeted safety/contracts 853 tests + 104 subtests PASS, security counter-suite 297 PASS는 소프트웨어 범위입니다.
  - full suite는 **7 failed, 1422 passed, 1 skipped, 190 subtests**이며, 실패 5건은 portability, 2건은 clean-checkout regression입니다.
  - live 23-scenario ROS/Gazebo는 `PLATFORM_UNAVAILABLE`; 실제 NUC·모터·encoder·E-stop·HIL·closed course·승객·인적요인은 미실행입니다.
- **Visual Treatment**: `확인된 소프트웨어 근거 / 남은 회귀 / 도구가 없어 실행하지 못한 범위와 물리 미실행` 3 modules는 회색조로 두고 PASS 숫자와 caveat를 같은 크기로 표시합니다. **sole-blue target:** `물리 제품 증명 아님` 결론 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/verify-full-pytest.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-bringup-resume.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`

### Slide 11 - 보안 경계와 제품 시험 근거도 상용 기준에 못 미칩니다
- **Type**: Risk and Evidence Gap
- **Title**: 현장 보안 경계와 실제 제품 시험 결과는 상용 기준에 못 미칩니다
- **Key Message**: 안전 알고리즘을 고쳐도 보안 경계와 물리 qualification 근거가 없으면 승객 운행을 승인할 수 없습니다.
- **Details**:
  - **P0-7**: 실제 field driver가 signed release에 없고, ROS name/topic은 인증된 주체가 아니며 SBOM·host hardening·secure update 근거가 부족합니다.
  - **P0-8**: tracked `evidence/**` 제품 결과는 0개이며 HIL, 제동거리, E-stop, EMC, 신뢰성, 인적요인 결과가 없습니다.
  - “근거에서 확인되지 않음”은 실물에 기능이 없다는 단정이 아니지만, 출시 승인 근거로 쓸 수는 없습니다.
- **Visual Treatment**: `보안 경계`와 `실제 제품 시험 결과` 두 column, 그리고 “미확인 ≠ 부재” 주석은 모두 회색조로 구성합니다. **sole-blue target:** `출시 승인 근거 미충족` 결론 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-repo-security.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-security-counter.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-coverage-evidence.md`

### Slide 12 - 제품화는 기술 수정 다음의 별도 과제입니다
- **Type**: Productization Framework
- **Title**: 사용자 화면·도움 요청·자산·플랫폼·규제를 하나의 제품 운영 체계로 묶어야 합니다
- **Key Message**: P1 제품화는 접근 가능한 사용자 경험, 유지 가능한 자산·플랫폼, 명확한 ODD와 규제 전략을 함께 완성하는 일입니다.
- **Details**:
  - 사용자 화면(HMI)은 상태·역할·이유·acknowledgement를 가진 idempotent lifecycle 요청이어야 하며, local stop과 별개로 rider alert, caregiver/help delivery, acknowledgement, retry·escalation을 제공해야 합니다.
  - map·route·band·software를 signed bundle로 운영하고, ROS Noetic EOL에는 patch owner·격리 또는 단계적 ROS 2 migration으로 대응해야 합니다.
  - intended use와 ODD를 먼저 동결하고, 사용자 검증·서비스·incident·CAPA/PMS와 MFDS 서면 분류 전략을 연결해야 합니다.
- **Visual Treatment**: `사용자·도움 / 자산·플랫폼 / 품질·규제` 3개 lane은 회색조로 구성합니다. **sole-blue target:** `하나의 제품 운영 체계` 결론 하나. 빨강·초록·주황·두 번째 accent를 금지하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-replanning-hmi.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-2-large-map-ops.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/wave-3-external-gate.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`

### Slide 13 - 다음 단계는 각 Gate의 증거로 관리합니다
- **Type**: Timeline
- **Title**: 각 단계의 증거가 충족될 때만 다음 Gate로 넘어갑니다
- **Key Message**: Gate 0–3 program 범위 승인은 단계별 작업 계획 승인일 뿐, 뒤 Gate의 실행 권한을 미리 여는 결정이 아닙니다.
- **Details**:
  - Gate 0: authority false 유지, 외부 base·NUC 실행 truth 동결.
  - Gate 1–2: 단일 software chain과 signed bundle을 만들고 target NUC isolated HIL에서 stop·fault·resource를 측정.
  - Gate 0–3 program 범위 승인에는 Gate 3 실행 authority가 포함되지 않으며, Gate 3 실행은 Gate 1–2 evidence 통과 뒤 별도로 엽니다.
  - Gate 3–5: 무승객 closed course, 제품·인적요인·규제 검증을 순서대로 마친 뒤에만 제한 pilot을 검토합니다.
- **Visual Treatment**: 6개 Gate node는 회색조로 두고, program 범위는 회색 bracket/label로 표시합니다. **sole-blue target:** 현재 다음 필수 단계인 `Gate 1` edge/label 하나. 빨강·초록·주황·두 번째 accent를 금지하고, Gate 3 실행 조건을 같은 화면에 명시하며 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/decision-story.md`

### Slide 14 - NO-GO 유지와 제한된 GO 승인을 요청합니다
- **Type**: Closing
- **Title**: 승객·캠퍼스 출시는 보류하고, Gate 0–3 검증 프로그램만 승인해 주십시오
- **Message**: 승객·캠퍼스 출시는 NO-GO로 유지하고, Gate 0–3 program 범위의 검증·개선 계획은 GO로 승인해 주십시오.
- **Key Message**: 기능 추가보다 단일 권한 체인, 현재 결함 폐쇄, 무승객 물리 근거 확보를 먼저 승인해야 합니다.
- **Details**:
  - **NO-GO**: 현재 hardware motion, passenger operation, campus operation, limited pilot 권한을 열지 않습니다.
  - **GO**: field truth 동결, 단일 authority, 0727 bundle·geometry·localization·perception 수정, HIL·무승객 closed course 근거 수집 program을 승인하되 각 Gate 실행은 앞선 evidence 통과 뒤 별도 authority로 엽니다.
  - 다음 release review는 사전 기준을 충족한 서명 receipt와 독립 residual-risk 승인으로만 엽니다.
  - 두 결정을 분리해 명시적으로 승인하고 회의 기록에 남겨 주십시오.
- **Visual Treatment**: 대상 라벨이 있는 두 카드와 하단 review trigger를 사용합니다. **sole-blue target:** `출시 권한 — NO-GO 출시 보류` 카드 하나만 파랑; `검증 프로그램 — GO program 범위 승인` 카드는 회색. 나머지는 흰색·회색조로 두고 빨강·초록·주황·두 번째 accent를 금지합니다. 색상 없이도 두 대상을 읽을 수 있게 하고, 출처는 우하단 10pt caption으로 표시합니다.
- **Sources**:
  - `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/decision-story.md`
  - `/Users/minjun/unicon-wheelchair/.omo/ulw-research/20260730-115236/SYNTHESIS.md`
