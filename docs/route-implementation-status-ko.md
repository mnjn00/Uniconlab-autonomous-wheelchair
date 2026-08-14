# 경로 구현 및 커밋 통합 기록

- 기준 시점: 2026-08-14
- 기준 브랜치: `main`
- 경로 구현 병합 기준 커밋: `5de3028033eddf128c2c4f6857744032da792688`

이 문서는 서로 다른 OMO 세션에서 만든 두 경로 계열을 구분하고, 각각의
입력·알고리즘·검증 수준·현재 운용 상태를 한곳에 기록한다. 두 경로는 같은
시작점과 목적지를 다루지만 목적과 안전 근거가 다르므로 하나의 경로처럼
혼용하면 안 된다.

## 커밋과 세션 출처

| 구분 | OMO 세션 | 핵심 커밋 | 역할 |
| --- | --- | --- | --- |
| v6/v8 운용 경로 | `019fea71-e382-7726-83c1-4fe0e7143f8f` | `6436d0f` | v6 선호선과 v8 주행가능 마스크를 결합해 운용 경로·밴드 생성 |
| v6/v8 운용 연결 | 같은 세션 | `ef1c670` | 배포 스크립트와 MPC를 v6/v8 경로에 고정 |
| v6/v8 경계 경화 | 같은 세션 | `4367c15`, `8c34b0b` | 모든 교차 셀과 raster corner를 보수적으로 검사 |
| dense-map 알고리즘 경로 | `019ff614-762b-7915-9056-8ef2ddc0c12d` | `2951327` | `map_by_algorithm_*` 경로·마스크·밴드·감사·시각화 추가 |
| 알고리즘 의존성 | 같은 세션 | `0bb592e` | `terrain_graph.py`와 독립 회귀 테스트 추가 |

`main`에는 `2951327`과 `0bb592e`를 깨끗한 `origin/main` 위에 적용한
동등 커밋 `9c4829e`, `4e71f0d`도 존재한다. 이후 브랜치 병합 커밋
`5de3028`이 원본 커밋 계보까지 합쳤다. Git 이력에는 양쪽 계보가 보이지만
최종 tree의 경로 파일은 각각 한 벌이다.

## 1. 현재 운용 경로: `20260812_route_v6_v8_*`

### 목적

운영자가 정답에 가깝다고 지정한 v6 선호선을 가능한 한 따르면서, v8
주행가능 영역 밖으로는 절대 나가지 않는 운용 경로다.

### 입력

- 선호 비용 지도: [`routes/route_2d_map_v6.yaml`](../routes/route_2d_map_v6.yaml)
- hard 주행가능 지도: [`routes/route_2d_map_v8.yaml`](../routes/route_2d_map_v8.yaml)
- seed 경로: `routes/20260812_route_v6_waypoints.json`
- seed 밴드: `routes/20260812_route_v6_safety_band.json`
- 재현 정보와 SHA-256:
  [`routes/20260812_route_v6_v8_provenance.json`](../routes/20260812_route_v6_v8_provenance.json)

### 알고리즘

[`tools/build_preferred_mask_route.py`](../tools/build_preferred_mask_route.py)는
0.1 m 격자에서 8방향 A*를 수행한다.

```text
cell_cost =
  1
  + 4.0 * distance_to_v6_preferred
  + 2.0 * exp(-v8_boundary_clearance / 0.5)
```

- v8 free cell만 탐색한다.
- 대각선 이동 시 양쪽 직교 셀도 free여야 하므로 corner cutting을 막는다.
- Savitzky-Golay smoothing을 두 번 적용해 raster 계단식 조향을 줄인다.
- smoothing 이후 모든 선분이 통과하는 raster interval과 정확한 corner를
  다시 검사한다.
- 최종 경로는 0.2 m, 밴드는 0.5 m 간격으로 재표본화한다.

### 결과와 현재 상태

| 항목 | 값 |
| --- | --- |
| 경로 | [`routes/20260812_route_v6_v8_waypoints.json`](../routes/20260812_route_v6_v8_waypoints.json) |
| 안전 밴드 | [`routes/20260812_route_v6_v8_safety_band.json`](../routes/20260812_route_v6_v8_safety_band.json) |
| waypoint | 1,900개 |
| 길이 | 379.167 m |
| band station | 761개, 0.5 m 간격 |
| 런타임 기본 경로 | **예** |
| 실제 위치추정 검증 | 아직 미완료 |

`moving_localization.launch`, `deploy_merged_map.sh`, `push_to_nuc.sh`가 이
경로를 기본 자산으로 사용한다. 따라서 현재 배포·주행 파이프라인의 기준은
이 경로다.

### 남은 위험

v8 경계는 운용자가 그린 hard mask다. seed 밴드에서 가까운 station의
`drop`, `step_up`, `lip` 의미를 전달하지만, 모든 최종 밴드 경계가 dense
point cloud에서 직접 재측정된 것은 아니다. 승객 운용 전에는 전체 경계
재측정과 실제 위치추정·추종 검증이 필요하다.

## 2. dense-map 후보 경로: `map_by_algorithm_*`

### 목적

기록 경로를 탐색 비용으로 사용하지 않고, 시작점과 목적지 및 dense-map에서
측정한 연석 corridor만으로 연결 경로가 존재하는지 독립적으로 확인한다.

### 입력

- 시작점: `(-7.9, -2.9)`
- 목적지: `(226.652, 24.252)`
- 원본 기하: `mergedmap.ply`에서 만든 dense-map 측정 밴드
- 경로 탐색용 마스크:
  [`output/map_by_algorithm_mask.yaml`](../output/map_by_algorithm_mask.yaml)

`planner_recorded_route_used`는 `false`다. 다만 측정 밴드의 station coverage는
실제 주행을 복원한 corridor를 따라 dense PLY에서 추출했다. 즉 bag 궤적이
최단경로 비용으로 들어가지는 않지만, 어디에서 연석을 정밀 측정할지를
제한하는 기하학적 coverage 근거로는 사용됐다.

### 알고리즘

1. 가까운 측정 station을 KD-tree로 찾는다.
2. station 법선에 대한 셀의 횡방향 위치를 계산한다.
3. 좌·우 측정 경계까지 각각 0.45 m 이상 남는 0.1 m 셀만 허용한다.
4. 허용 셀의 8방향 그래프에서 clearance 가중 Dijkstra를 수행한다.
5. 2.5 cm 간격으로 모든 점이 마스크 안에 있는 선분만 string-pull한다.
6. 최종 경로를 5 cm 간격으로 재표본화해 중심·휠·전체 footprint를 감사한다.

감사 footprint는 길이 0.97 m, 폭 0.76 m이며 다음 조건 중 하나라도
실패하면 `BLOCKED`다.

- 중심의 좌·우 측정 여유 각각 0.45 m 이상
- 좌·우 휠 라인의 경계 여유 0.10 m 이상
- 회전된 전체 footprint 네 모서리의 경계 여유 0.07 m 이상

### 결과와 현재 상태

| 항목 | 값 |
| --- | --- |
| 경로 | [`output/map_by_algorithm_route.json`](../output/map_by_algorithm_route.json) |
| 안전 밴드 | [`output/map_by_algorithm_band.json`](../output/map_by_algorithm_band.json) |
| 감사 | [`output/map_by_algorithm_audit.json`](../output/map_by_algorithm_audit.json) |
| waypoint | 1,897개 |
| 길이 | 376.210 m |
| band station | 1,897개, 0.2 m 간격 |
| 최소 좌/우 중심 여유 | 0.505316 / 0.454797 m |
| 최소 휠 경계 여유 | 0.104883 m |
| 최소 전체 footprint 여유 | 0.075214 m |
| 5 cm 감사 sample | 7,540개 |
| 감사 상태 | **APPROVED** |
| 런타임 기본 경로 | **아니오** |

원본 지도 중첩 이미지는
[`output/map_by_algorithm_on_original_map.png`](../output/map_by_algorithm_on_original_map.png)이며,
이전 독립 경로가 같은 검사에서 거부된 근거는
[`output/map_by_algorithm_prior_route_rejection.json`](../output/map_by_algorithm_prior_route_rejection.json)에
있다.

## 두 경로의 관계

| 기준 | v6/v8 운용 경로 | `map_by_algorithm` 후보 |
| --- | --- | --- |
| 선호 근거 | 운영자 v6 선호선 | 없음 |
| hard 경계 | 운영자 v8 raster | dense-map 측정 연석 밴드 |
| 탐색 | 선호·경계 비용 A* | clearance 가중 Dijkstra |
| smoothing | Savitzky-Golay 후 raster 재검사 | 마스크 내부 선분만 string-pull |
| 안전 감사 | v8 mask·runtime band | 중심·휠·회전 footprint 수치 감사 |
| 배포 상태 | 현재 기본 | 분석·승격 후보 |

두 결과를 평균하거나 waypoint를 섞으면 각 경로가 보유한 안전 증명이
사라진다. `map_by_algorithm`을 운용 경로로 승격하려면 별도 변경으로 다음을
모두 수행해야 한다.

1. runtime route/band/mask 자산 해시를 새 경로에 맞춰 결속한다.
2. `moving_localization.launch`와 배포 스크립트의 기본 자산을 함께 변경한다.
3. localization replay와 follower shadow test를 통과한다.
4. 내리막 구간의 실제 위치추정·추종 오차를 비주행 또는 저속 현장 절차로
   확인한다.
5. 승인 전까지 기존 v6/v8 운용 경로를 자동으로 대체하지 않는다.

## 검증 기록

- v6/v8 OMO 세션은 경로 생성 후 1,900 waypoint, 379.167 m를 기록했다.
- `map_by_algorithm` 커밋 전 focused 70개, 영향 범위 125개 테스트가 통과했다.
- `main` 통합 worktree에서 경로·terrain 관련 테스트 100개와 Python compile이
  통과했다.
- 전체 저장소 pytest는 기존 `main`에서도 발생하는 Python 3.9
  `hashlib.file_digest` import 오류 때문에 수집 단계에서 중단된다. 이 오류는
  두 경로 구현 변경과 무관하다.
