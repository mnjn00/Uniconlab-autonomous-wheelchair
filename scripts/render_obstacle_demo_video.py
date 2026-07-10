#!/usr/bin/env python3
"""Render a top-down MP4 from the Gazebo-recorded obstacle avoidance trajectory."""

import argparse
import csv
import math
import os
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OBSTACLES = [
    ('person_obstacle_1', 2.0, 0.45, 0.22, (40, 80, 230)),
    ('person_obstacle_2', 4.6, -0.35, 0.20, (230, 80, 40)),
    ('bollard', 6.0, 0.82, 0.06, (30, 30, 30)),
]
WAYPOINTS = [(0.0, 0.0), (0.9, -0.42), (2.8, -0.64), (3.8, 0.55), (5.25, 0.55), (6.35, 0.12), (7.35, 0.0)]


def load_rows(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) if k != 'waypoint_index' else int(v) for k, v in row.items()})
    if not rows:
        raise ValueError('trajectory CSV is empty')
    return rows


def load_font(size):
    for p in [
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf',
    ]:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def rotated_rect(cx, cy, w, h, yaw):
    pts = []
    for lx, ly in [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]:
        x = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
        y = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
        pts.append((x, y))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    args = ap.parse_args()

    rows = load_rows(args.csv)
    W, H = args.width, args.height
    margin = 70
    xmin, xmax = -0.5, 8.0
    ymin, ymax = -1.45, 1.45

    def sx(x):
        return margin + (x - xmin) / (xmax - xmin) * (W - 2 * margin)

    def sy(y):
        return H - margin - (y - ymin) / (ymax - ymin) * (H - 2 * margin)

    def world_poly(points):
        return [(sx(x), sy(y)) for x, y in points]

    font = load_font(24)
    small = load_font(18)
    title_font = load_font(30)
    frames_dir = Path(tempfile.mkdtemp(prefix='wheelchair_demo_frames_'))

    duration = rows[-1]['t'] - rows[0]['t']
    nframes = max(1, int(math.ceil(duration * args.fps)))
    step = max(1, int(len(rows) / nframes))
    frame_rows = rows[::step]
    if frame_rows[-1] is not rows[-1]:
        frame_rows.append(rows[-1])

    path_so_far = []
    for i, r in enumerate(frame_rows):
        # Include all raw samples up to this frame for a smooth trail.
        cutoff_t = r['t']
        path_so_far = [(row['x'], row['y']) for row in rows if row['t'] <= cutoff_t]
        img = Image.new('RGB', (W, H), (244, 247, 251))
        d = ImageDraw.Draw(img)

        # Sidewalk slab and curbs.
        d.rectangle([sx(xmin), sy(1.2), sx(xmax), sy(-1.2)], fill=(214, 214, 214), outline=(150, 150, 150), width=2)
        d.rectangle([sx(xmin), sy(1.30), sx(xmax), sy(1.20)], fill=(105, 105, 105))
        d.rectangle([sx(xmin), sy(-1.20), sx(xmax), sy(-1.30)], fill=(105, 105, 105))
        d.text((sx(0.1), sy(1.08)), 'sidewalk corridor / Gazebo Classic world', font=small, fill=(70, 70, 70))

        # Waypoints and ideal path.
        wp_screen = world_poly(WAYPOINTS)
        if len(wp_screen) > 1:
            d.line(wp_screen, fill=(130, 130, 130), width=2)
        for idx, p in enumerate(wp_screen):
            x, y = p
            d.ellipse([x-5, y-5, x+5, y+5], fill=(255, 255, 255), outline=(90, 90, 90), width=2)
            d.text((x+7, y-7), str(idx), font=small, fill=(70, 70, 70))

        # Obstacles.
        for name, ox, oy, radius, color in OBSTACLES:
            x, y = sx(ox), sy(oy)
            rr = abs(sx(ox + radius) - sx(ox))
            d.ellipse([x-rr, y-rr, x+rr, y+rr], fill=color, outline=(255, 255, 255), width=3)
            label = name.replace('_obstacle_', ' ')
            d.text((x + rr + 7, y - 12), label, font=small, fill=(20, 20, 20))
            # approximate keep-clear bubble
            clear = radius + 0.34
            cr = abs(sx(ox + clear) - sx(ox))
            d.ellipse([x-cr, y-cr, x+cr, y+cr], outline=(color[0], color[1], color[2]), width=1)

        # Actual recorded path.
        if len(path_so_far) > 1:
            d.line(world_poly(path_so_far), fill=(20, 160, 80), width=5)

        # Robot body/wheels from actual Gazebo pose.
        x, y, yaw = r['x'], r['y'], r['yaw']
        body = world_poly(rotated_rect(x, y, 0.97, 0.60, yaw))
        d.polygon(body, fill=(255, 215, 60), outline=(120, 100, 0))
        front = (x + 0.48 * math.cos(yaw), y + 0.48 * math.sin(yaw))
        d.line([sx(x), sy(y), sx(front[0]), sy(front[1])], fill=(180, 60, 0), width=5)
        d.ellipse([sx(x)-5, sy(y)-5, sx(x)+5, sy(y)+5], fill=(80, 80, 80))

        # HUD.
        d.rounded_rectangle([20, 16, W-20, 104], radius=12, fill=(255, 255, 255), outline=(210, 210, 210))
        d.text((36, 26), 'ROS1 Noetic + Gazebo Classic: wheelchair obstacle-avoidance demo', font=title_font, fill=(20, 20, 20))
        d.text((36, 66), f't={r["t"]:5.1f}s  cmd_vel_nav -> safety_gate -> cmd_vel_safe -> diff_drive_controller', font=small, fill=(20, 90, 160))
        d.text((36, H-42), 'Green trail = actual /gazebo/model_states trajectory. Dashed bubbles = approximate obstacle keep-clear zones.', font=small, fill=(50, 50, 50))

        img.save(frames_dir / f'frame_{i:05d}.png')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        'ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning',
        '-framerate', str(args.fps), '-i', str(frames_dir / 'frame_%05d.png'),
        '-vf', 'format=yuv420p', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23',
        str(out),
    ]
    subprocess.run(cmd, check=True)
    print(f'RESULT video={out} frames={len(frame_rows)} duration_s={duration:.2f}')


if __name__ == '__main__':
    main()
