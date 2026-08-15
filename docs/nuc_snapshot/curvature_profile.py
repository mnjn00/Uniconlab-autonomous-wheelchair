#!/usr/bin/env python3
"""경로 전체의 곡률 한계 속도를 뽑아, 어디가 0.30 m/s 하한을 뚫는지 찾는다.

curvature_speed(band, point) 는 이 휠체어가 그 지점을 지날 때 yaw 상한을 지키며
낼 수 있는 최대 속도다. 그것이 TURN_FLOOR_SPEED 아래면 자율 주행이 불가능하다 --
천천히 돌아야 하는데 베이스가 천천히 못 돌기 때문.
"""
import json
import sys

sys.path.insert(0, "/home/mprp3/livox_static_localization_ws/src/static_livox_localization/scripts")
import numpy as np
import mpc_speed
from safety_band import SafetyBand

BAND = "/home/mprp3/wheelchair_localization_src/routes/20260814_route_algorithm_safety_band.json"
band = SafetyBand(BAND)
FLOOR = mpc_speed.TURN_FLOOR_SPEED

pts = band.xy
speeds = []
for i in range(len(pts)):
    try:
        speeds.append(float(mpc_speed.curvature_speed(band, pts[i])))
    except Exception:
        speeds.append(float("nan"))
speeds = np.array(speeds)

blocked = speeds < FLOOR
print("스테이션 %d개, 하한 %.2f m/s" % (len(pts), FLOOR))
print("주행 불가 스테이션: %d개 (%.1f%%)"
      % (blocked.sum(), 100.0 * blocked.sum() / len(pts)))
print()

# 연속 구간으로 묶기
runs, start = [], None
for i, bad in enumerate(blocked):
    if bad and start is None:
        start = i
    elif not bad and start is not None:
        runs.append((start, i - 1)); start = None
if start is not None:
    runs.append((start, len(blocked) - 1))

print("=== 막히는 구간 (최소 속도 순) ===")
runs_sorted = sorted(runs, key=lambda r: float(np.nanmin(speeds[r[0]:r[1] + 1])))
for a, b in runs_sorted[:12]:
    seg = speeds[a:b + 1]
    arc = float(np.sum(np.linalg.norm(np.diff(pts[a:b + 2], axis=0), axis=1))) if b + 2 <= len(pts) else 0.0
    print("  스테이션 %4d-%-4d (%5.1f m)  최소 %.3f m/s  위치 (%.1f, %.1f)"
          % (a, b, arc, float(np.nanmin(seg)), pts[a][0], pts[a][1]))
print()
print("총 %d개 구간" % len(runs))

out = {
    "floor_mps": FLOOR,
    "stations": len(pts),
    "blocked_stations": int(blocked.sum()),
    "xy": [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in pts],
    "curvature_speed": [None if np.isnan(v) else round(float(v), 3) for v in speeds],
    "runs": [[int(a), int(b), round(float(np.nanmin(speeds[a:b + 1])), 3)] for a, b in runs],
}
with open("/tmp/curvature_profile.json", "w") as fh:
    json.dump(out, fh)
print("저장: /tmp/curvature_profile.json")
