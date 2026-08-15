# 은교 마스크 세트 (2026-08-12 작성, 08-15 수령)

경로 재생성용 원본 마스크입니다. **아직 route/band 가 없으므로 런타임 자산이
아닙니다** — `build_preferred_mask_route.py` 로 경로를 만들고
`promote_algorithm_route.py` 로 해시 바인딩을 만들어야 씁니다.

| 파일 | 내용 |
| :--- | :--- |
| `route_2d_map_v6.*` | **preferred** — 은교가 그린 초록 경로 |
| `route_2d_map_v7.*` | **drivable** 영역만 (`route_red_mask_v7.png` 동봉) |
| `route_2d_map_v8.*` | v6 + v7 합본 (`green_route` 64,386 px, `red_overlay`) |

세 개 모두 같은 격자라 `build_preferred_mask_route.py` 의 shape/resolution/origin
일치 검사를 통과합니다.

```
resolution  0.1 m/px
origin      [-106.5, -168.9, 0.0]
size        4151 x 2628
frame       map
```

## 왜 이걸로 다시 그리는가

현재 필드 기본값인 `20260814_route_algorithm` 은 곡률이 물리 한계를 넘는 코너가
13곳(144/1897 스테이션, 7.6%) 있어 자율 주행이 그 지점에서 멈춥니다.
`docs/2026-08-15-dwa-first-drive.md` 참조. v5 계열에서는 이런 정지가 없었다는
현장 보고가 있습니다.

## 재생성 절차

```bash
python3 tools/build_preferred_mask_route.py \
  --preferred-yaml routes/eungyo_20260812/route_2d_map_v6.yaml \
  --drivable-yaml  routes/eungyo_20260812/route_2d_map_v7.yaml \
  --seed-route <기존 route> --seed-band <기존 band> \
  --out-route output/eungyo_route.json --out-band output/eungyo_band.json

# 주행 전 필수: 곡률 게이트. 막힌 스테이션 0 이어야 함
python3 docs/nuc_snapshot/curvature_profile.py

python3 tools/promote_algorithm_route.py \
  --source-route output/eungyo_route.json \
  --source-band  output/eungyo_band.json \
  --source-mask-yaml routes/eungyo_20260812/route_2d_map_v7.yaml \
  --route-name 20260816_route_eungyo_waypoints.json \
  --band-name  20260816_route_eungyo_safety_band.json \
  --mask-name  route_2d_map_v7
```

`build_preferred_mask_route.py` 는 Savitzky-Golay 평활화만 하고 결과 곡률을
검사하지 않으므로, 가운데 게이트를 건너뛰면 같은 문제가 반복됩니다.
