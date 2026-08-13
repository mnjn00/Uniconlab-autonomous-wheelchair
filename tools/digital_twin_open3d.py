#!/usr/bin/env python3
"""Digital twin v2: virtual wheelchair + 6-field analysis overlay.

Keys (press in the Open3D window):
  1  scan-density heatmap along the route
  2  map-coverage gaps (sparse red zones)
  3  toggle FOV / rider-exclusion filter on the live scan
  4  speed profile (follower's actual speed cap per station)
  5  ICP localization quality (fitness at each waypoint)
  6  virtual obstacles + detection corridor
  Space  pause / resume
  R  reset camera to follow mode
  Q  quit
"""

import argparse, json, math, sys, time
import numpy as np
import open3d as o3d

# ── MID360 ────────────────────────────────────────────────────
SENSOR_H = 0.30
VFOV_MIN = math.radians(-7.0)
VFOV_MAX = math.radians(52.0)
SCAN_R   = 12.0
PTS_SCAN = 24000
FOV_HALF = math.radians(50.0)          # forward cone half-angle
RIDER_X  = (-1.0, 0.55)               # rider exclusion box
RIDER_YH = 0.40
RIDER_Z  = (-0.5, 1.8)
CORRIDOR_MIN_R = 0.50

# ── follower constants (mirrors waypoint_follower.py) ─────────
MAX_SPEED = 1.0; SLOPE_SPEED = 0.3; CREEP = 0.15
MAX_YAW_RATE = 0.5; MAX_ACCEL = 0.18; MAX_DECEL = 0.6
SLOPE_PITCH = math.radians(3.0)

# ── colours ───────────────────────────────────────────────────
C_BG      = [0.07, 0.07, 0.09]
C_MAP     = [0.50, 0.50, 0.54]
C_SCAN    = [0.20, 0.90, 0.30]
C_SCAN_RAW= [0.60, 0.60, 0.20]
C_CHAIR   = [0.90, 0.15, 0.10]
C_ROUTE   = [0.25, 0.45, 1.00]
C_TRAIL   = [1.00, 0.55, 0.00]
C_WP      = [1.00, 0.85, 0.00]
C_OBS     = [0.95, 0.20, 0.55]
C_CORR    = [0.10, 0.70, 0.95]

# heatmap: blue(good) -> red(bad)
def heat(t):
    t = max(0.0, min(1.0, t))
    return [t, 0.15 + 0.5*(1-abs(2*t-1)), 1.0-t]


# ═══════════════════════════════════════════════════════════════
#  Loading
# ═══════════════════════════════════════════════════════════════
def load_map(path):
    print(f"  map: {path}")
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points, dtype=np.float64)
    print(f"  {len(pts):,} pts")
    return pcd, pts

def load_route(path):
    with open(path) as f:
        d = json.load(f)
    wps = np.array([[w["x"], w["y"], w.get("z",0.0)] for w in d["waypoints"]])
    yaws = np.array([math.radians(w.get("yaw_deg",0.0)) for w in d["waypoints"]])
    print(f"  route: {len(wps)} wp  ({wps[0,0]:.1f},{wps[0,1]:.1f})->"
          f"({wps[-1,0]:.1f},{wps[-1,1]:.1f})")
    return wps, yaws

def densify(wps, yaws, step=0.25):
    xy, yw = [], []
    for i in range(len(wps)-1):
        p0, p1 = wps[i], wps[i+1]
        n = max(int(np.linalg.norm(p1[:2]-p0[:2])/step), 1)
        for j in range(n):
            t = j/n
            xy.append((p0 + t*(p1-p0)).tolist())
            yw.append(yaws[i] + t*(yaws[i+1]-yaws[i]))
    xy.append(wps[-1].tolist()); yw.append(yaws[-1])
    return np.array(xy), np.array(yw)


# ═══════════════════════════════════════════════════════════════
#  Analysis  (all pre-computed at waypoints)
# ═══════════════════════════════════════════════════════════════
def build_kdtree(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return o3d.geometry.KDTreeFlann(pcd)

def scan_density(tree, wps, radius=SCAN_R):
    """Point count within radius at each waypoint."""
    n = len(wps)
    counts = np.zeros(n, dtype=np.int32)
    for i in range(n):
        _, idx, _ = tree.search_radius_vector_3d(wps[i], radius)
        counts[i] = len(idx)
    return counts

def coverage_gaps(tree, wps, yaws, radius=3.0, n_probes=8):
    """For each wp, probe n_probes directions at `radius` and count
    how many have < 50 map points within 1 m -> gap score 0..1."""
    n = len(wps)
    scores = np.zeros(n)
    for i in range(n):
        gaps = 0
        for k in range(n_probes):
            ang = 2*math.pi*k/n_probes
            probe = wps[i] + np.array([radius*math.cos(ang),
                                       radius*math.sin(ang), 0])
            _, idx, _ = tree.search_radius_vector_3d(probe, 1.0)
            if len(idx) < 50:
                gaps += 1
        scores[i] = gaps / n_probes
    return scores

def speed_profile(wps, yaws):
    """Replicate the follower's speed cap logic (no obstacles)."""
    n = len(wps)
    speeds = np.full(n, MAX_SPEED)
    for i in range(n):
        # curvature from 3-point circle
        if 0 < i < n-1:
            v1 = wps[i][:2] - wps[i-1][:2]
            v2 = wps[i+1][:2] - wps[i][:2]
            cross = abs(v1[0]*v2[1] - v1[1]*v2[0])
            l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if l1 > 1e-3 and l2 > 1e-3:
                kappa = 2*cross / (l1*l2*(l1+l2+1e-9))
                # v = sqrt(a_lat / kappa), a_lat ~ 0.8 m/s^2
                if kappa > 1e-4:
                    speeds[i] = min(speeds[i], math.sqrt(0.8/kappa))
        # slope
        if i < n-1:
            dz = wps[i+1,2] - wps[i,2]
            dl = max(np.linalg.norm(wps[i+1,:2]-wps[i,:2]), 0.01)
            pitch = abs(math.atan2(dz, dl))
            if pitch > SLOPE_PITCH:
                speeds[i] = min(speeds[i], SLOPE_SPEED)
        speeds[i] = max(speeds[i], CREEP)
    # accel/decel smoothing
    dt = 0.25 / MAX_SPEED  # approx time between waypoints
    for i in range(1, n):
        speeds[i] = min(speeds[i], speeds[i-1] + MAX_ACCEL*dt)
    for i in range(n-2, -1, -1):
        speeds[i] = min(speeds[i], speeds[i+1] + MAX_DECEL*dt)
    return speeds

def icp_quality(map_pts, tree, wps, yaws):
    """Run point-to-plane ICP at each waypoint with a perturbed
    initial guess. Returns fitness and inlier_ratio arrays."""
    n = len(wps)
    fitness = np.zeros(n)
    inlier  = np.zeros(n)
    for i in range(n):
        # synthetic scan
        scan = _synthetic(map_pts, wps[i], yaws[i], filtered=True)
        if len(scan) < 100:
            continue
        src = o3d.geometry.PointCloud()
        src.points = o3d.utility.Vector3dVector(scan)
        src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.5, 30))
        # target: map crop
        _, idx, _ = tree.search_radius_vector_3d(wps[i], SCAN_R)
        tgt = o3d.geometry.PointCloud()
        tgt.points = o3d.utility.Vector3dVector(map_pts[idx])
        tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.5, 30))
        # perturb initial guess
        dx, dy, dyaw = np.random.uniform(-0.3,0.3), \
                        np.random.uniform(-0.3,0.3), \
                        np.random.uniform(-0.05,0.05)
        c, s = math.cos(yaws[i]+dyaw), math.sin(yaws[i]+dyaw)
        T_init = np.array([[c,-s,0,wps[i,0]+dx],
                           [s, c,0,wps[i,1]+dy],
                           [0, 0,1,wps[i,2]],
                           [0, 0,0,1]])
        # scan is in body frame; transform to world for ICP
        src_w = src.transform(T_init)
        reg = o3d.pipelines.registration.registration_icp(
            src_w, tgt, 0.5, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
        fitness[i] = reg.fitness
        inlier[i]  = reg.inlier_rmse
        if i % 20 == 0:
            print(f"    ICP {i}/{n}  fitness={reg.fitness:.3f} "
                  f"rmse={reg.inlier_rmse:.3f}")
    return fitness, inlier

def place_obstacles(wps, yaws, n_obs=6):
    """Place virtual obstacles near the route at even intervals."""
    obs = []
    step = max(len(wps) // (n_obs+1), 1)
    for k in range(1, n_obs+1):
        i = min(k * step, len(wps)-1)
        # offset 1.5 m to the left of the route
        nx, ny = -math.sin(yaws[i]), math.cos(yaws[i])
        pos = wps[i] + np.array([nx*1.5, ny*1.5, 0])
        obs.append({"pos": pos, "radius": 0.35, "height": 1.7,
                    "wp_idx": i})
    return obs


# ═══════════════════════════════════════════════════════════════
#  Synthetic scan
# ═══════════════════════════════════════════════════════════════
def _synthetic(map_pts, pos, yaw, filtered=True):
    rel = map_pts - pos
    c, s = math.cos(-yaw), math.sin(-yaw)
    x = rel[:,0]*c - rel[:,1]*s
    y = rel[:,0]*s + rel[:,1]*c
    z = rel[:,2]
    r = np.sqrt(x*x + y*y)
    m = r < SCAN_R
    zs = z + SENSOR_H
    elev = np.arctan2(zs, np.maximum(r, 0.01))
    m &= (elev > VFOV_MIN) & (elev < VFOV_MAX) & (zs > -0.05)
    if filtered:
        az = np.abs(np.arctan2(y, x))
        m &= (x > CORRIDOR_MIN_R) & (az < FOV_HALF)
        rider = ((x > RIDER_X[0]) & (x < RIDER_X[1]) &
                 (np.abs(y) < RIDER_YH) &
                 (z > RIDER_Z[0]) & (z < RIDER_Z[1]))
        m &= ~rider
    scan = np.column_stack([x[m], y[m], z[m]])
    if len(scan) > PTS_SCAN:
        idx = np.random.choice(len(scan), PTS_SCAN, replace=False)
        scan = scan[idx]
    return scan


# ═══════════════════════════════════════════════════════════════
#  Visualisation
# ═══════════════════════════════════════════════════════════════
class Twin:
    def __init__(self, args):
        print("Loading assets …")
        self.map_pcd, self.map_pts = load_map(args.map)
        self.wps, self.yaws = load_route(args.route)
        self.dense, self.dense_yaw = densify(self.wps, self.yaws)
        segs = np.linalg.norm(np.diff(self.dense[:,:2], axis=0), axis=1)
        self.cumdist = np.concatenate([[0], np.cumsum(segs)])
        self.total_dist = self.cumdist[-1]
        self.total_frames = len(self.dense)

        print("Pre-computing analysis …")
        tree = build_kdtree(self.map_pts)

        print("  [1/5] scan density")
        self.density = scan_density(tree, self.wps)
        d_lo, d_hi = np.percentile(self.density, [5, 95])
        self.density_norm = np.clip(
            (self.density - d_lo) / max(d_hi - d_lo, 1), 0, 1)

        print("  [2/5] coverage gaps")
        self.gaps = coverage_gaps(tree, self.wps, self.yaws)

        print("  [3/5] speed profile")
        self.speeds = speed_profile(self.wps, self.yaws)

        print("  [4/5] ICP quality")
        self.icp_fit, self.icp_rmse = icp_quality(
            self.map_pts, tree, self.wps, self.yaws)

        print("  [5/5] obstacles")
        self.obstacles = place_obstacles(self.wps, self.yaws)

        # ── state ──
        self.frame = 0
        self.paused = False
        self.follow = True
        self.fov_on = True
        self.show = {"density": False, "gaps": False,
                     "speed": False, "icp": False, "obs": False}
        self.trail_pts = []
        self.args = args

        self._build_scene()

    # ── scene construction ────────────────────────────────────
    def _build_scene(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            "Digital Twin v2 — Autonomous Wheelchair",
            width=1500, height=950)

        opt = self.vis.get_render_option()
        opt.background_color = np.array(C_BG)
        opt.point_size = 1.5

        # map
        self.map_pcd.paint_uniform_color(C_MAP)
        self.vis.add_geometry(self.map_pcd)

        # route line
        self.route_line = o3d.geometry.LineSet()
        self.route_line.points = o3d.utility.Vector3dVector(
            self.wps + [0,0,0.05])
        self.route_line.lines = o3d.utility.Vector2iVector(
            np.column_stack([np.arange(len(self.wps)-1),
                             np.arange(1, len(self.wps))]))
        self.route_line.paint_uniform_color(C_ROUTE)
        self.vis.add_geometry(self.route_line)

        # waypoint dots
        self.wp_pcd = o3d.geometry.PointCloud()
        self.wp_pcd.points = o3d.utility.Vector3dVector(
            self.wps + [0,0,0.1])
        self.wp_pcd.paint_uniform_color(C_WP)
        self.vis.add_geometry(self.wp_pcd)

        # dynamic: chair, scan, trail
        self.chair = o3d.geometry.TriangleMesh.create_box(1.1, 0.65, 0.6)
        self.chair.compute_vertex_normals()
        self.chair.paint_uniform_color(C_CHAIR)
        self.vis.add_geometry(self.chair)

        self.scan_pcd = o3d.geometry.PointCloud()
        self.scan_pcd.points = o3d.utility.Vector3dVector(
            np.zeros((1,3)))
        self.scan_pcd.paint_uniform_color(C_SCAN)
        self.vis.add_geometry(self.scan_pcd)

        self.trail = o3d.geometry.PointCloud()
        self.trail.points = o3d.utility.Vector3dVector(np.empty((0,3)))
        self.trail.paint_uniform_color(C_TRAIL)
        self.vis.add_geometry(self.trail)

        # analysis overlays (hidden by default)
        self._make_density_markers()
        self._make_gap_markers()
        self._make_speed_markers()
        self._make_icp_markers()
        self._make_obstacle_markers()

        # key bindings
        self.vis.register_key_callback(ord("1"), lambda v: self._toggle("density"))
        self.vis.register_key_callback(ord("2"), lambda v: self._toggle("gaps"))
        self.vis.register_key_callback(ord("3"), lambda v: self._toggle_fov())
        self.vis.register_key_callback(ord("4"), lambda v: self._toggle("speed"))
        self.vis.register_key_callback(ord("5"), lambda v: self._toggle("icp"))
        self.vis.register_key_callback(ord("6"), lambda v: self._toggle("obs"))
        self.vis.register_key_callback(32, lambda v: self._pause())  # space
        self.vis.register_key_callback(ord("R"), lambda v: self._reset_cam())

        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.04)
        self._print_help()

    def _make_density_markers(self):
        self.density_pcd = o3d.geometry.PointCloud()
        pts = self.wps + [0,0,0.15]
        self.density_pcd.points = o3d.utility.Vector3dVector(pts)
        cols = [heat(1.0 - d) for d in self.density_norm]
        self.density_pcd.colors = o3d.utility.Vector3dVector(cols)
        self.vis.add_geometry(self.density_pcd)
        # big spheres for visibility
        self.density_spheres = []
        for i in range(0, len(self.wps), 3):
            s = o3d.geometry.TriangleMesh.create_sphere(0.25)
            s.translate(pts[i])
            s.paint_uniform_color(heat(1.0 - self.density_norm[i]))
            s.compute_vertex_normals()
            self.density_spheres.append(s)
            self.vis.add_geometry(s)
        self._set_visible(self.density_spheres, False)

    def _make_gap_markers(self):
        self.gap_spheres = []
        for i in range(len(self.wps)):
            if self.gaps[i] > 0.25:
                s = o3d.geometry.TriangleMesh.create_sphere(
                    0.3 + self.gaps[i])
                s.translate(self.wps[i] + [0,0,0.2])
                r = min(1.0, self.gaps[i] * 1.5)
                s.paint_uniform_color([r, 0.1, 0.1])
                s.compute_vertex_normals()
                self.gap_spheres.append(s)
                self.vis.add_geometry(s)
        self._set_visible(self.gap_spheres, False)

    def _make_speed_markers(self):
        self.speed_pcd = o3d.geometry.PointCloud()
        pts = self.wps + [0,0,0.25]
        self.speed_pcd.points = o3d.utility.Vector3dVector(pts)
        cols = [heat(1.0 - s/MAX_SPEED) for s in self.speeds]
        self.speed_pcd.colors = o3d.utility.Vector3dVector(cols)
        self.vis.add_geometry(self.speed_pcd)
        self.speed_spheres = []
        for i in range(0, len(self.wps), 2):
            s = o3d.geometry.TriangleMesh.create_sphere(0.2)
            s.translate(pts[i])
            s.paint_uniform_color(heat(1.0 - self.speeds[i]/MAX_SPEED))
            s.compute_vertex_normals()
            self.speed_spheres.append(s)
            self.vis.add_geometry(s)
        self._set_visible(self.speed_spheres, False)

    def _make_icp_markers(self):
        self.icp_spheres = []
        for i in range(len(self.wps)):
            s = o3d.geometry.TriangleMesh.create_sphere(0.3)
            s.translate(self.wps[i] + [0,0,0.35])
            s.paint_uniform_color(heat(self.icp_fit[i]))
            s.compute_vertex_normals()
            self.icp_spheres.append(s)
            self.vis.add_geometry(s)
        self._set_visible(self.icp_spheres, False)

    def _make_obstacle_markers(self):
        self.obs_meshes = []
        self.corr_lines = []
        for ob in self.obstacles:
            cyl = o3d.geometry.TriangleMesh.create_cylinder(
                ob["radius"], ob["height"])
            cyl.translate(ob["pos"] + [0, 0, ob["height"]/2])
            cyl.paint_uniform_color(C_OBS)
            cyl.compute_vertex_normals()
            self.obs_meshes.append(cyl)
            self.vis.add_geometry(cyl)
        # detection corridor along the full route
        left, right = [], []
        for i in range(len(self.wps)):
            nx = -math.sin(self.yaws[i])
            ny =  math.cos(self.yaws[i])
            left.append(self.wps[i,:2] + np.array([nx, ny])*0.45)
            right.append(self.wps[i,:2] - np.array([nx, ny])*0.45)
        for side, col in [(left, C_CORR), (right, C_CORR)]:
            ls = o3d.geometry.LineSet()
            pts3 = [np.array([p[0], p[1], self.wps[i,2]+0.1])
                    for i, p in enumerate(side)]
            ls.points = o3d.utility.Vector3dVector(pts3)
            ls.lines = o3d.utility.Vector2iVector(
                np.column_stack([np.arange(len(pts3)-1),
                                 np.arange(1, len(pts3))]))
            ls.paint_uniform_color(col)
            self.corr_lines.append(ls)
            self.vis.add_geometry(ls)
        self._set_visible(self.obs_meshes, False)
        self._set_visible(self.corr_lines, False)

    # ── helpers ───────────────────────────────────────────────
    def _save_colors(self, geoms):
        """Store original colours so toggle can restore them."""
        saved = []
        for g in geoms:
            if hasattr(g, 'vertex_colors') and len(g.vertex_colors) > 0:
                saved.append(np.asarray(g.vertex_colors).copy())
            elif hasattr(g, 'vertex_colors'):
                saved.append(None)
            else:
                saved.append(None)
        return saved

    def _set_visible(self, geoms, vis, saved_colors=None):
        HIDE_POS = np.array([0, 0, -9999.0])
        if not hasattr(self, '_hidden_ids'):
            self._hidden_ids = set()
        for i, g in enumerate(geoms):
            gid = id(g)
            if vis:
                if gid in self._hidden_ids:
                    g.translate(-HIDE_POS)
                    self._hidden_ids.discard(gid)
                if saved_colors and saved_colors[i] is not None:
                    g.vertex_colors = o3d.utility.Vector3dVector(
                        saved_colors[i])
            else:
                if gid not in self._hidden_ids:
                    g.translate(HIDE_POS)
                    self._hidden_ids.add(gid)
            self.vis.update_geometry(g)

    def _toggle(self, key):
        self.show[key] = not self.show[key]
        on = self.show[key]
        label = {"density":"Scan Density","gaps":"Coverage Gaps",
                 "speed":"Speed Profile","icp":"ICP Quality",
                 "obs":"Obstacles"}[key]
        print(f"  [{label}] {'ON' if on else 'OFF'}")
        if key == "density":
            self._set_visible(self.density_spheres, on)
        elif key == "gaps":
            self._set_visible(self.gap_spheres, on)
        elif key == "speed":
            self._set_visible(self.speed_spheres, on)
        elif key == "icp":
            self._set_visible(self.icp_spheres, on)
        elif key == "obs":
            self._set_visible(self.obs_meshes, on)
            self._set_visible(self.corr_lines, on)
        return False

    def _toggle_fov(self):
        self.fov_on = not self.fov_on
        print(f"  [FOV+Rider Filter] {'ON' if self.fov_on else 'OFF (raw 360°)'}")
        return False

    def _pause(self):
        self.paused = not self.paused
        print(f"  {'PAUSED' if self.paused else 'RESUMED'}")
        return False

    def _reset_cam(self):
        self.follow = True
        return False

    def _print_help(self):
        print("""
  ┌─────────────────────────────────────────────┐
  │  1  Scan density heatmap                    │
  │  2  Map coverage gaps (red = sparse)        │
  │  3  Toggle FOV / rider-exclusion filter     │
  │  4  Speed profile (red = slow)              │
  │  5  ICP localization quality                │
  │  6  Virtual obstacles + corridor            │
  │  Space  Pause / Resume                      │
  │  R  Re-enable camera follow                 │
  │  Q  Quit                                    │
  └─────────────────────────────────────────────┘""")

    # ── animation ─────────────────────────────────────────────
    def _update(self, vis):
        if self.paused:
            return True

        dist = self.frame * (self.args.speed / self.args.fps)
        if dist > self.total_dist:
            sys.stdout.write(
                f"\r  100.0%  GOAL  ({self.dense[-1,0]:.1f},"
                f"{self.dense[-1,1]:.1f})  "
                f"route complete ({self.total_dist:.0f} m). "
                f"Close window to exit.\n")
            return True  # keep rendering, just stop advancing

        idx = np.searchsorted(self.cumdist, dist, side="right") - 1
        idx = min(idx, self.total_frames - 2)
        segs = np.diff(self.cumdist)
        frac = (dist - self.cumdist[idx]) / max(segs[idx], 1e-6)
        pos = self.dense[idx] + frac*(self.dense[idx+1]-self.dense[idx])
        yaw = self.dense_yaw[idx] + frac*(self.dense_yaw[idx+1]-self.dense_yaw[idx])

        # chair
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c,-s,0],[s,c,0],[0,0,1.0]])
        new_chair = o3d.geometry.TriangleMesh.create_box(1.1, 0.65, 0.6)
        new_chair.rotate(R, center=(0,0,0))
        new_chair.translate(pos + [0,0,0.05])
        new_chair.compute_vertex_normals()
        new_chair.paint_uniform_color(C_CHAIR)
        self.chair.vertices = new_chair.vertices
        self.chair.triangles = new_chair.triangles
        self.chair.vertex_normals = new_chair.vertex_normals
        self.chair.paint_uniform_color(C_CHAIR)

        # scan
        scan = _synthetic(self.map_pts, pos, yaw, filtered=self.fov_on)
        if len(scan) > 0:
            scan_w = scan @ R.T + pos
            self.scan_pcd.points = o3d.utility.Vector3dVector(scan_w)
            col = C_SCAN if self.fov_on else C_SCAN_RAW
            self.scan_pcd.paint_uniform_color(col)

        # trail
        self.trail_pts.append(pos.tolist())
        if len(self.trail_pts) > 3000:
            self.trail_pts = self.trail_pts[-3000:]
        self.trail.points = o3d.utility.Vector3dVector(
            np.array(self.trail_pts))

        # camera follow
        if self.follow:
            ctr = vis.get_view_control()
            ctr.set_lookat(pos.tolist())

        vis.update_geometry(self.chair)
        vis.update_geometry(self.scan_pcd)
        vis.update_geometry(self.trail)

        # HUD
        wp_idx = np.argmin(np.linalg.norm(self.wps[:,:2]-pos[:2], axis=1))
        pct = 100*dist/self.total_dist
        spd = self.speeds[wp_idx]
        fit = self.icp_fit[wp_idx]
        den = self.density[wp_idx]
        sys.stdout.write(
            f"\r  {pct:5.1f}%  wp {wp_idx:3d}/{len(self.wps)}  "
            f"({pos[0]:6.1f},{pos[1]:5.1f})  "
            f"v={spd:.2f}  scan={len(scan):5d}  "
            f"dens={den:6d}  icp={fit:.3f}  "
            f"fov={'ON ' if self.fov_on else 'OFF'}  ")
        sys.stdout.flush()

        self.frame += 1
        return True

    def run(self):
        self.vis.register_animation_callback(self._update)
        self.vis.run()
        self.vis.destroy_window()


# ═══════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="/Volumes/무제/merged_0707_0725_v1/"
                    "merged_0707_0725_0p20m_xyzi.pcd")
    ap.add_argument("--route", default="/Users/minjun/unicon-wheelchair/"
                    "routes/20260727_new_route_waypoints.json")
    ap.add_argument("--speed", type=float, default=0.6)
    ap.add_argument("--fps", type=float, default=10.0)
    args = ap.parse_args()
    Twin(args).run()
    print("\nDone.")

if __name__ == "__main__":
    main()
