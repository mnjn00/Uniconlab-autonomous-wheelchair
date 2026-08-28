# 인지 스택 3단계 증설 개발 명세

작성일 2026-08-25. 대상: Patchwork++ 지면 분리, 추적기 교체, 학습 3D 검출기.
검증: 인용 상수 전부를 origin/main(97c14bf) 소스와 대조 완료.
전제가 되는 실측: NUC RTX 2060 6 GB(여유 약 5.4 GB, 사용률 14~45%),
드라이버 570.133.07, torch 2.4.1+cpu(CUDA 빌드 아님), spconv/TensorRT/nvcc 없음,
Ubuntu 20.04.6 / Python 3.8.10.

## 1. 현재 파이프라인과 결함-부품 대응

```
/livox/lidar
   └─ FAST-LIO ─ /cloud_registered_body (바디 프레임, 왜곡 보정, 10 Hz)
        ├─ safety_gate.py          원시 점 + 높이띠 필터(filter_obstacle_points)
        ├─ obstacle_clusters.py    0.6 s 누적 → 0.2 m 격자 군집 → 휴리스틱 분류
        │      └─ cluster_tracking.py  최근접 연합 추적(odom 프레임)
        │      └─ /perception/objects_summary (JSON, 5 Hz)
        └─ (블랙박스 기록)
/perception/objects_summary
        ├─ waypoint_follower.py / cluster_guard.py   WAIT / GO_ROUND 판단
        └─ dwa_follower.py → dwa_core.py             장애물 박스 회피 채점
```

세 층(게이트·판단·플래너)이 서로 다른 세계를 본다는 구조는 유지한다.
게이트가 원시 점을 보는 것은 의도된 최후 방어선이다. 증설은 그 앞단의
**입력 품질**을 올리는 것이지 방어선을 학습 모듈로 대체하는 것이 아니다.

확인된 결함과 담당 부품:

| 결함 (실측 근거) | 원인 위치 | 담당 |
|---|---|---|
| 고개에서 지면을 장애물로 오검출, 1.3 s 급정지 (t+2253, 피치 5.7°에서 3.30 m 전방 0.33 m 유령) | `motion_safety.py`가 의자 자세 평면 기준으로 높이를 잼. `ground_reference()`는 실클라우드 40장 검증에서 전제 자체가 기각되어 OFF | 1. Patchwork++ |
| 라벨 플리커(person↔obstacle), 프레임 통째 드롭아웃 → follower에 `PERSON_MEMORY_S=1.0` 임시 기억, 부분 관측 중심점 표류를 이동으로 오독 | `cluster_tracking.py` 최근접 연합, 차폐 시 트랙 소멸(DROP_AFTER_S=0.8), 라벨은 매 프레임 덮어씀 | 2. 추적기 |
| 사람 박스 과소(평균 0.44×0.45 m, 5백분위 0.18) → `cluster_guard.py`의 `PERSON_MIN_HALF_EXTENT_M=0.35` 하한 임시조치, 발/옆구리 스침 | 격자 군집은 보이는 면만 상자로 만듦. 휴리스틱 분류는 발자국·키만 봄 | 3. 학습 검출기 |

## 2. 목표 아키텍처

```
/cloud_registered_body (10 Hz)
   └─ [신규] ground_segmentation.py (Patchwork++)
        ├─ /cloud_nonground   PointCloud2, 입력과 동일 프레임·스탬프
        └─ /cloud_ground      (진단·기록용, 다운샘플 가능)
/cloud_nonground
   ├─ safety_gate.py          (폴백: 토픽 정체 시 원본+높이띠로 복귀)
   ├─ obstacle_clusters.py    (동일 폴백)
   │     └─ [교체] cluster_tracking.py  칼만 + 헝가리안 + 차폐 관성
   └─ [신규] learned_detector.py  (격리 venv, GPU)
         └─ /perception/detections (JSON)
              └─ obstacle_clusters.py가 병합: 기하는 군집이,
                 클래스·크기는 IoU 매칭된 학습 박스가 우선
```

공통 원칙 세 가지. 위반하는 설계는 반려한다.

1. **기하 불삭제.** 학습 모듈이든 지면 분리든, 어떤 단계도 군집이 만든
   충돌 박스를 지울 수 없다. 의미(클래스·크기·운동)만 고칠 수 있다.
   `obstacle_clusters.py`의 기존 원칙("semantic relabel only, never a
   reason to delete a collision box")을 전 단계로 확장한 것이다.
2. **정체 시 원래 동작 복귀.** 신규 노드가 죽거나 출력이 0.5 s 이상
   정체하면 소비자는 현행 입력·필터로 자동 복귀하고 사유를 상태 토픽에
   남긴다. 새 노드의 부재가 정지 사유가 되어서는 안 된다
   (게이트-플래너 시야 불일치 교착의 교훈: 두 노드가 다른 것을 보게 만들지 말 것).
   킬스위치는 ROS 파라미터 하나로 통일한다 — 신규 노드 자체는 `~enabled`,
   소비자의 비지면 입력 전환은 `~nonground_input`, 학습 상자 병합은
   `obstacle_clusters`의 `~learned_merge`. 끄면 코드 경로가 아니라
   기동 전 상태와 동일해야 한다.
3. **섀도 → 개입 2단 배포.** 모든 단계는 먼저 기록만 하는 섀도 모드로
   실주행 로스백을 쌓고, 오프라인 비교로 합격 기준을 넘긴 뒤에만
   판단 경로에 연결한다. 연결은 파라미터 한 개로 끄고 켤 수 있어야 한다.

## 3. 단계 1: Patchwork++ 지면 분리

### 3.1 선정 근거
- CPU 전용(C++/Eigen, pybind11 바인딩 `pypatchworkpp`) — GPU·파이썬 환경을
  건드리지 않아 현행 스택과 충돌 지점이 없다.
- 지면을 의자 자세가 아니라 **점군 자체**에서 추정한다. 피치 과도 시
  자세 평면이 도로를 파고드는 현재 결함의 정공법이다.
- 다중 패치·경사 적응이라 6° 지형 + 5.7° 자세 과도의 겹침(겉보기 약 12°)을
  단일 평면 RANSAC과 달리 처리한다.

### 3.2 신규 노드 `ground_segmentation.py`
- 위치: `src/static_livox_localization/scripts/`. 기존 노드 관례
  (모듈 상단 상수, docstring에 설계 사유) 준수.
- 구독: `/cloud_registered_body`. 발행: `/cloud_nonground`(전체 비지면 점,
  입력 스탬프·프레임 유지), `/cloud_ground`(1/4 다운샘플, 진단용),
  `/ground_segmentation/status`(JSON: 처리 ms, 지면점 비율, 파라미터 해시).
- 처리율: 입력 10 Hz 전량. 프레임당 예산 30 ms(3.4절에서 실측 후 확정).
  초과 시 소비자 정합을 위해 프레임 스킵이 아니라 입력 다운샘플로 대응.
- 파라미터(초기값, 필드 튜닝 대상):
  - `sensor_height`: 0.725 (`SENSOR_HEIGHT_M` 실측치)
  - `max_range`: 12.0 (`ROI_X` 상한과 일치)
  - uprightness/slope 관련 임계는 겉보기 경사 12°를 덮도록 설정
- 주의: Patchwork++는 센서 원점 기준 프레임을 전제한다.
  `/cloud_registered_body`는 바디 프레임이므로 `body_frame.py`의
  `body_to_lidar` 외부 파라미터로 z 오프셋을 맞춰 입력한다.

### 3.3 소비자 변경
- `safety_gate.py`: 구독을 `/cloud_nonground`로 전환. **높이띠 필터는
  유지**하되 하한 역할이 바뀐다 — 지면 제거는 Patchwork++가 하고,
  `max_height_m` 상한(가지·간판 통과 허용)은 그대로 절대 기준으로 남긴다.
  `/cloud_nonground` 스탬프가 0.5 s 이상 정체하면
  `/cloud_registered_body` + 현행 필터로 복귀, 게이트 상태 토픽에
  `GROUND_SEG_STALE` 기록.
- `obstacle_clusters.py`: 동일 전환·동일 폴백. `RIDER_EXCLUDE_*`,
  레트로리플렉터 컷 등 기존 전처리는 순서만 뒤로 밀고 전부 유지.
- `motion_safety.py::ground_referenced`: 삭제하지 않고 OFF 유지.
  실측으로 기각된 이력(피크 0.63 vs 0.65)을 테스트에 남겨둔 상태를 보존.

### 3.4 검증과 합격 기준
- 오프라인: `aejimum_to_gongsen.bag` 실클라우드 40장(기존 검증 세트) +
  고개 통과 구간 로스백 재생.
  - 고개 오검출: 현행 재현 케이스(t+2253 성격의 피치 반전 유령 객체) 0건
  - 0.20 m 연석: 비지면으로 유지(현행 `ground_reference`의 알려진 한계
    지점이므로 명시적 케이스로 작성)
  - 처리 시간: NUC에서 프레임당 중앙값과 p95 실측, p95 < 30 ms
- 단위 테스트: `test/test_ground_segmentation.py` — 합성 경사면(6°),
  피치 5.7° 기울인 평지, 연석 단차, 폴백 타이머.
- 섀도 주행: 최소 1회 전체 루트, `/cloud_ground` 비율과 게이트
  차단 사유 분포를 현행 주행과 비교.

### 3.5 리스크
- `pypatchworkpp` 빌드가 Ubuntu 20.04 기본 CMake(3.16)보다 높은 버전을
  요구할 수 있다. pip CMake로 해결 가능하나 **빌드 산출물과 절차를
  저장소에 기록**할 것(NUC 재클론 사고 재발 대비).
- 비지면 점 수가 줄어 게이트 감도가 변한다. 섀도 구간에서
  `HALF_WIDTH_M` 회랑 내 점 개수 분포를 전후 비교해 임계 재조정 여부 판단.

## 4. 단계 2: 추적기 교체

### 4.1 방침
외부 레포(AB3DMOT류) 이식이 아니라 `cluster_tracking.py`의 **동일 파일
교체 확장**으로 간다. 이유: 현재 모듈은 odom 프레임 변환·생산자 결합 구조가
이미 우리 파이프라인에 맞게 설계돼 있고, 부족한 것은 연합·예측·기억
세 가지뿐이다. 외부 코드가 주는 것도 정확히 그 세 가지라 이식 비용이
구현 비용보다 크다. 알고리즘은 AB3DMOT와 동일 계열(등속 칼만 + 헝가리안)로 한다.

### 4.2 변경 명세 (`cluster_tracking.py`)
- **상태 모델**: 트랙당 등속 칼만 필터, 상태 (x, y, vx, vy).
  구현은 numpy 직접(4x4 행렬), 외부 의존 추가 없음.
- **연합**: 최근접 탐욕 → `scipy.optimize.linear_sum_assignment`
  (scipy는 dwa_core의 cKDTree로 이미 의존성에 있음). 게이트는 예측
  위치 기준 마할라노비스 또는 유클리드 1.2 m에서 시작.
- **차폐 관성(coasting)**: 미매칭 트랙을 즉시 버리지 않고
  `COAST_MAX_S = 2.0`까지 예측만으로 유지, 관성 중 게이트는 공분산에
  비례해 확장. 관성 중 트랙은 요약에 `coasting: true`로 표시.
  DROP_AFTER_S(0.8)는 관성 진입 임계로 의미가 바뀐다.
- **라벨 이력**: 매 프레임 덮어쓰기 → 최근 `HISTORY_S` 창 다수결 +
  이력. person 판정은 진입은 1프레임, 이탈은 다수결로 — 비대칭
  히스테리시스. 정지 오인보다 사람 놓침이 더 나쁘다는 기존 원칙과 정렬.
- **크기 기억**: 트랙별 half-extent를 창 내 최댓값(감쇠 포함)으로 발행.
  부분 관측으로 상자가 갑자기 줄어드는 것을 상류에서 완화한다.
  `PERSON_MIN_HALF_EXTENT_M=0.35` 하한은 3단계 전까지 유지.
- **운동 판정**: 변위 기반 `speed_mps()`를 칼만 속도 추정으로 교체하되,
  STATIC 판정 임계(0.20)·CONFIRM_S(1.5)·"UNKNOWN은 moving 취급" 원칙은 유지.

### 4.3 인터페이스 변화
- `/perception/objects_summary` 객체에 추가: `track_id`(차폐를 넘어 안정),
  `vx`, `vy`, `coasting`, `label_votes`. 기존 필드는 불변 —
  소비자 하위호환 유지.
- 소비자 후속 정리(추적기 검증 후 별도 커밋):
  - `waypoint_follower.py`의 `person_memory`(1.0 s 드롭아웃 기억)는
    상류 관성과 중복이므로 제거. 단 관성이 실주행에서 검증된 뒤에만.
  - `cluster_guard.py`는 `vx, vy`로 이동 차량의 계획-지평선 시점 위치를
    예측해 GO_ROUND 진로 선택에 반영할 수 있게 된다(별도 명세로 분리,
    이 명세 범위 밖).

### 4.4 검증과 합격 기준
- 단위: `test/test_cluster_tracking.py` 확장 — 횡단 보행자 연속 추적,
  1.5 s 차폐 후 동일 track_id 복원, 주차 차량의 시야 확장 중심점 표류를
  STATIC으로 판정, 두 객체 근접 교차 시 라벨 스왑 없음.
- 재생: 사람 개입 실험 로스백들에서 트랙당 라벨 반전 횟수,
  드롭아웃 유발 WAIT 진입 횟수를 전후 비교. 합격: 라벨 반전 50% 이상
  감소, 프레임 드롭아웃발 WAIT 0건.
- 성능: 5 Hz 주기 내 처리(현행 대비 증분 < 5 ms, 객체 40개 기준).

## 5. 단계 3: 학습 3D 검출기

### 5.1 실측으로 확정된 조건
- 하드웨어는 충분: RTX 2060 6 GB 중 약 5.4 GB 유휴, 사용률 14~45%.
  CenterPoint-Pillar급 10 Hz 추론에 2~3 GB — 정합 스택과 동시 구동 가능.
- 소프트웨어는 전무: torch가 CPU 빌드, spconv·TensorRT·CUDA 툴킷 없음.
  따라서 이 단계의 실제 작업은 모델이 아니라 **환경 격리 구축**이 절반이다.

### 5.2 환경 격리 (필수)
- 시스템 파이썬을 절대 건드리지 않는다. 정합·주행 스택 전체가 그 위에 있다.
- `~/venvs/det3d` venv를 `--system-site-packages`로 생성(rospy 접근용),
  그 안에서만: torch 2.4.1+cu121(py3.8을 지원하는 마지막 계열,
  드라이버 570은 CUDA 12.x 호환), spconv-cu120 cp38 휠.
  **설치 전 각 휠의 cp38 존재를 확인하고 결과를 이 문서에 추기할 것.**
  cp38 휠이 없으면 venv 대신 nvidia-container-toolkit 도커로 전환한다.
- 검출 노드는 이 venv의 인터프리터로 launch하며, 시스템 파이썬과의
  접점은 ROS 토픽뿐이다.

### 5.3 모델 선정
- 1순위 **livox_detection v2**: Livox 자체 데이터로 학습된 공개 가중치 —
  MID360 비반복 스캔 패턴과의 도메인 간극이 가장 작다. 이 간극이
  이 단계 최대 리스크다(공개 벤치마크 가중치는 64채널 회전식 기준).
- 2순위 OpenPCDet CenterPoint-Pillar(nuScenes 가중치): livox_detection이
  구동 불가하거나 성능 미달일 때. pillar 계열은 spconv 의존이 얕아
  환경 리스크도 작다.
- 입력은 원시 스캔이 아니라 **0.6 s 누적 창**(`Accumulator` 재사용)을
  기본으로 실험한다. 단일 0.1 s MID360 스윕은 학습 데이터보다 희박하다.

### 5.4 신규 노드 `learned_detector.py`
- 구독: `/cloud_nonground`(1단계 산출; 지면 제거는 검출 recall도 올린다).
  발행: `/perception/detections` — JSON String, `objects_summary`와 동일
  좌표 규약(의자 정렬 프레임), 필드 `{x, y, half_x, half_y, height,
  yaw, class, score, latency_ms}`.
- 주기 5 Hz(클러스터링과 동일), 추론 p95 < 100 ms, GPU 상주 < 2.5 GB.
- 워치독: 추론이 0.5 s 지연되면 해당 주기 발행 생략(오래된 박스 발행 금지).

### 5.5 병합 규칙 (`obstacle_clusters.py`)
- 군집 상자와 학습 상자를 IoU(BEV) 0.3 이상으로 매칭.
- 매칭된 군집: 클래스·크기·yaw를 학습 상자로 **덮어쓰기**(score 임계
  이상일 때). 사람 박스 과소 문제의 정공법이며, 이때
  `cluster_guard.py`의 `PERSON_MIN_HALF_EXTENT_M` 하한을 은퇴시킨다
  (`test_cluster_guard.py`의 하한 고정 단정 2건 포함).
- 학습에만 있는 상자: 신뢰 임계 이상이면 추적기에 신규 관측으로 투입.
- **군집에만 있는 상자: 무조건 유지.** 원칙 1(기하 불삭제).
- 검출 토픽 정체 시: 병합 생략, 현행 휴리스틱 단독 — 원칙 2.

### 5.6 검증과 합격 기준
- 섀도 모드(발행만, 병합 OFF)로 실주행 로스백 축적 후 오프라인 평가:
  - 사람 박스 폭: 실측 어깨폭 대비 과소율, 현행 5백분위 0.18 m 소멸
  - 사람 recall: 개입 실험 구간에서 프레임 단위 미검출률 < 5%
  - 오검출: 회랑 내 유령 상자 빈도가 현행 휴리스틱 이하
- 동시 구동 검증: `moving_icp_localizer`와 함께 전체 루트 주행,
  정합 지연·GPU 메모리 경합 모니터링. 정합 주기 열화 0이 조건.
- 도메인 간극이 합격선을 못 넘으면: 결과를 기록하고 병합 없이 섀도
  유지. 자체 데이터 파인튜닝은 별도 명세로 분리한다.

## 6. 브링업·배포·기록 연계

### 6.1 기동 순서 (`start_wheelchair_localization.sh`)
- 현행 체인: livox 드라이버 → vectornav → fast_lio → moving_localization.launch →
  `obstacle_clusters.py` → wheel.launch → `safety_gate.py` → tip_guard → follower →
  route_identity_publisher.
- `ground_segmentation.py`는 `obstacle_clusters.py` 바로 앞에 넣는다.
  구독 기반이라 기동 순서가 밀려도 폴백이 흡수하지만, 첫 프레임부터
  비지면 입력을 보게 하기 위해 순서상 앞에 둔다.
- `learned_detector.py`는 별도 줄로 마지막에 넣고 인터프리터를
  `~/venvs/det3d/bin/python3`로 고정한다. venv가 파손돼 시스템 파이썬으로
  떨어져도 import에서 즉시 죽고 소비자 폴백이 흡수한다 — 시스템 환경
  오염이 없다는 게 이 배치의 요점이다.
- 신규 노드는 워치독(정지 사유) 대상에 넣지 않는다(원칙 2).
  `/ground_segmentation/status`만 SHADOW_QA 진단 출력에 추가한다.

### 6.2 배포 경로와 예외
- 코드: `deploy_code.sh`(저장소→워크스페이스 rsync + catkin build)가
  신규 스크립트를 모두 커버한다. Python 노드라 빌드 산출물은 없다.
- **예외: `~/venvs/det3d`는 rsync 밖이다.** NUC 재설치·재클론 시 재구축이
  필요하므로 설치 로그와 휠 버전을 `docs/nuc_snapshot/`에 남긴다
  (5.2의 cp38 확인 결과와 같은 커밋에).

### 6.3 블랙박스
- 추가 기록: `/cloud_ground`(1/4 다운샘플 그대로),
  `/ground_segmentation/status`, `/perception/detections`.
  `/cloud_nonground`는 원본(`/cloud_registered_body`, 이미 기록 중)과
  지면만 있으면 재생성되므로 굳이 기록하지 않는다.
- 섀도 평가의 1차 데이터가 블랙박스다. record 목록 변경은 각 단계의
  착수 커밋에 함께 넣고, 누락된 구간은 그 단계 합격 판정에서 제외한다.

### 6.4 go.sh 전제조건은 불변
- 전제 3개(waypoint_follower 응답, `objects_summary` 생산, TRACKING)는
  그대로다. "조용한 생산자는 빈 리스트로 주행하게 된다"는 현행 원칙과
  동일하게, 새 계층의 침묵이 주행을 막지도 시작시키지도 않게 한다.

## 7. 일정과 의존 관계

```
1. Patchwork++  ──┐ (독립, 즉시 착수 가능, CPU만)
2. 추적기        ──┤ (독립, 즉시 착수 가능, 1과 병행 가능)
3. 학습 검출기      └─ 입력으로 1을 쓰고, 출력을 2가 소비 → 1·2 뒤
```

- 1과 2는 서로 접점이 없어 병행 가능하다. 단 **배포는 한 번에 하나씩**
  — 주행 거동 변화의 원인을 가릴 수 있어야 한다.
- 각 단계는 독립 커밋 계열 + 파라미터 킬스위치 + 섀도 기간을 갖는다.
- 다른 세션이 같은 저장소에서 작업 중이므로, 신규 파일 위주(1, 3)는
  충돌 위험이 낮고 `cluster_tracking.py` 교체(2)는 착수 전 조율 필요.

## 8. 이 명세가 다루지 않는 것

- 이동 차량 회피의 계획 로직 개선(추적기 속도 벡터의 소비 방법) —
  4.3에서 분리한 대로 별도 명세.
- 좌측 바퀴 정지 미적용(베이스 펌웨어), 고개 급정지의 잔여 케이스 검증 —
  기존 트랙에서 계속.
- 자체 데이터셋 구축·파인튜닝 — 5.6의 섀도 평가 결과가 나온 뒤 판단.
