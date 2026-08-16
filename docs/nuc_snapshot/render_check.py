"""RouteMapView 의 투영 수식을 그대로 파이썬으로 재현해 실제 경로를 그려본다.

목적은 그림이 예쁜지가 아니라, 축 방향과 fit 계산이 맞는지 확인하는 것.
안드로이드에서 축이 뒤집혀 보이면 현장에서 원인을 찾기 어렵다.
"""
import importlib.util
import json
import sys

sys.stdout.reconfigure(encoding="utf-8")
BRIDGE = r"c:\Users\npgy2\.anaconda\intern\Uniconlab-autonomous-wheelchair\scripts\ros1_bluetooth_bridge.py"
spec = importlib.util.spec_from_file_location("bb", BRIDGE)
bb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bb)

ROUTE = (r"c:\Users\npgy2\.anaconda\intern\Uniconlab-autonomous-wheelchair"
         r"\routes\20260814_route_algorithm_waypoints.json")

frame = bb.load_route(ROUTE)
pts = frame["points"]
full = json.load(open(ROUTE, encoding="utf-8"))["waypoints"]

W, H, PAD = 900, 600, 24.0


def project(points, pose=None):
    """RouteMapView.onDraw 와 동일한 계산."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    minX, maxX, minY, maxY = min(xs), max(xs), min(ys), max(ys)
    if pose:
        minX, maxX = min(minX, pose[0]), max(maxX, pose[0])
        minY, maxY = min(minY, pose[1]), max(maxY, pose[1])
    spanX = max(maxX - minX, 0.5)
    spanY = max(maxY - minY, 0.5)
    usableW, usableH = W - 2 * PAD, H - 2 * PAD
    scale = min(usableW / spanX, usableH / spanY)
    offX = PAD + (usableW - spanX * scale) / 2.0
    offY = PAD + (usableH - spanY * scale) / 2.0
    out = [(offX + (x - minX) * scale, offY + (maxY - y) * scale) for x, y in points]
    ppose = None
    if pose:
        ppose = (offX + (pose[0] - minX) * scale, offY + (maxY - pose[1]) * scale)
    return out, ppose, scale


print("=== 경로 기하 ===")
xs = [p[0] for p in pts]
ys = [p[1] for p in pts]
print("  전체 %d점 -> 전송 %d점 (stride %d)"
      % (frame["count_full"], frame["count"], frame["stride"]))
print("  x 범위 %.1f ~ %.1f m  (%.1f m)" % (min(xs), max(xs), max(xs) - min(xs)))
print("  y 범위 %.1f ~ %.1f m  (%.1f m)" % (min(ys), max(ys), max(ys) - min(ys)))

# 시작점 근처를 현재 위치로 가정
pose = (pts[0][0], pts[0][1])
proj, ppose, scale = project(pts, pose)
print("  화면 축척 %.2f px/m" % scale)
print("  모든 점이 화면 안? %s"
      % all(0 <= x <= W and 0 <= y <= H for x, y in proj))

# 진행도 매핑 검증: wp_index 는 전체 인덱스, 화면은 thinned 인덱스
print("\n=== 진행도 매핑 (stride %d) ===" % frame["stride"])
bad = 0
for full_i in (0, 500, 1000, 1896):
    drawn = full_i // frame["stride"]
    drawn = min(drawn, len(pts) - 1)
    fx, fy = full[full_i]["x"], full[full_i]["y"]
    dx, dy = pts[drawn]
    err = ((fx - dx) ** 2 + (fy - dy) ** 2) ** 0.5
    flag = "OK" if err < 1.5 else "오차 큼"
    if err >= 1.5:
        bad += 1
    print("  wp %4d -> 표시 %3d   실제(%.1f,%.1f) 표시(%.1f,%.1f)  오차 %.2f m  %s"
          % (full_i, drawn, fx, fy, dx, dy, err, flag))
print("  판정: %s" % ("진행 마커가 실제 위치와 1.5m 이내" if bad == 0
                      else "%d개 지점에서 어긋남" % bad))

# 축 방향 확인: ROS +y(좌) 는 화면 위쪽이어야 한다
p_lo, _, _ = project([[0.0, 0.0], [0.0, 10.0]])
print("\n=== 축 방향 ===")
print("  ROS y=0 -> 화면 y %.1f,  ROS y=+10 -> 화면 y %.1f" % (p_lo[0][1], p_lo[1][1]))
print("  판정: %s" % ("+y(좌) 가 화면 위 = 올바름" if p_lo[1][1] < p_lo[0][1]
                      else "뒤집힘 — 수정 필요"))

# SVG 로 저장해 눈으로 확인
poly = " ".join("%.1f,%.1f" % p for p in proj)
done = int(len(proj) * 0.35)
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}">
<rect width="{W}" height="{H}" fill="#F8FBFF"/>
<polyline points="{" ".join("%.1f,%.1f" % p for p in proj[done:])}"
  fill="none" stroke="#C7D6E5" stroke-width="6" stroke-linejoin="round"/>
<polyline points="{" ".join("%.1f,%.1f" % p for p in proj[:done+1])}"
  fill="none" stroke="#0175C2" stroke-width="6" stroke-linejoin="round"/>
<circle cx="{proj[0][0]:.1f}" cy="{proj[0][1]:.1f}" r="7" fill="#98A2B3"/>
<circle cx="{proj[-1][0]:.1f}" cy="{proj[-1][1]:.1f}" r="7" fill="#98A2B3"/>
<circle cx="{proj[done][0]:.1f}" cy="{proj[done][1]:.1f}" r="13" fill="#12805C"/>
<text x="12" y="24" font-family="sans-serif" font-size="16" fill="#667085">
{frame["count_full"]}점 중 {frame["count"]}점 표시 · {max(xs)-min(xs):.0f}m x {max(ys)-min(ys):.0f}m</text>
</svg>'''
out = (r"C:\Users\npgy2\AppData\Local\Temp\claude"
       r"\C--Users-npgy2--claude-intern\1efe909a-efbd-49ae-8f03-dbcbc65b5718"
       r"\scratchpad\route_preview.svg")
open(out, "w", encoding="utf-8").write(svg)
print("\n미리보기 저장: %s" % out)
