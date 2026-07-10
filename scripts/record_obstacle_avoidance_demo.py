#!/usr/bin/env python3
"""Drive a Gazebo wheelchair through obstacle-avoidance waypoints and record model state.

This is a simulator demo utility, not an autonomy certification test. It publishes an
obstacle-avoidance trajectory to /cmd_vel_nav so the already-verified safety chain
(/cmd_vel_nav -> safety_gate -> /cmd_vel_safe -> base controller) can be visualized.
"""

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float
    safe_linear: float = 0.0
    safe_angular: float = 0.0
    controller_linear: float = 0.0
    controller_angular: float = 0.0
    waypoint_index: int = 0


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Recorder:
    def __init__(self, model_name):
        self.model_name = model_name
        self.latest_pose = None
        self.safe_cmd = Twist()
        self.controller_cmd = Twist()
        self.samples = []
        self.start_time = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel_safe', Twist, self.safe_cb, queue_size=10)
        rospy.Subscriber('/wheelchair_base_controller/cmd_vel', Twist, self.controller_cb, queue_size=10)

    def model_cb(self, msg):
        if self.model_name not in msg.name:
            return
        idx = msg.name.index(self.model_name)
        p = msg.pose[idx].position
        o = msg.pose[idx].orientation
        if self.start_time is None:
            self.start_time = rospy.Time.now().to_sec()
        self.latest_pose = (p.x, p.y, yaw_from_quat(o), rospy.Time.now().to_sec() - self.start_time)

    def safe_cb(self, msg):
        self.safe_cmd = msg

    def controller_cb(self, msg):
        self.controller_cmd = msg

    def record(self, waypoint_index):
        if self.latest_pose is None:
            return
        x, y, yaw, t = self.latest_pose
        self.samples.append(Pose2D(
            t=t, x=x, y=y, yaw=yaw,
            safe_linear=self.safe_cmd.linear.x,
            safe_angular=self.safe_cmd.angular.z,
            controller_linear=self.controller_cmd.linear.x,
            controller_angular=self.controller_cmd.angular.z,
            waypoint_index=waypoint_index,
        ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='/workspace/artifacts/obstacle_demo/trajectory.csv')
    parser.add_argument('--model-name', default='wheelchair')
    parser.add_argument('--max-seconds', type=float, default=75.0)
    args = parser.parse_args()

    rospy.init_node('record_obstacle_avoidance_demo', anonymous=True)
    recorder = Recorder(args.model_name)
    pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=10)

    rospy.loginfo('waiting for Gazebo model state for %s', args.model_name)
    deadline = time.time() + 30.0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and recorder.latest_pose is None and time.time() < deadline:
        rate.sleep()
    if recorder.latest_pose is None:
        raise RuntimeError('Timed out waiting for /gazebo/model_states wheelchair pose')

    # Slalom path: pass below the first pedestrian, above the second obstacle, then return to center.
    waypoints = [
        (0.0, 0.0),
        (0.9, -0.42),
        (2.8, -0.64),
        (3.8, 0.55),
        (5.25, 0.55),
        (6.35, 0.12),
        (7.35, 0.00),
    ]
    wp_i = 1
    start_wall = time.time()
    reached_final = False

    while not rospy.is_shutdown() and time.time() - start_wall < args.max_seconds:
        if recorder.latest_pose is None:
            rate.sleep()
            continue
        x, y, yaw, _t = recorder.latest_pose
        tx, ty = waypoints[wp_i]
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = norm_angle(desired - yaw)

        if dist < 0.20:
            rospy.loginfo('reached waypoint %d/%d at x=%.2f y=%.2f', wp_i, len(waypoints)-1, x, y)
            if wp_i >= len(waypoints) - 1:
                reached_final = True
                break
            wp_i += 1
            continue

        cmd = Twist()
        # Slow down when heading error is large; keep turning toward waypoint.
        heading_factor = max(0.10, math.cos(err))
        cmd.linear.x = clamp(0.16 + 0.22 * min(dist, 1.0), 0.10, 0.38) * heading_factor
        if abs(err) > 1.25:
            cmd.linear.x = 0.05
        cmd.angular.z = clamp(1.55 * err, -0.95, 0.95)
        pub.publish(cmd)
        recorder.record(wp_i)
        rate.sleep()

    stop = Twist()
    for _ in range(20):
        pub.publish(stop)
        recorder.record(wp_i)
        rate.sleep()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'yaw', 'safe_linear', 'safe_angular', 'controller_linear', 'controller_angular', 'waypoint_index'])
        for s in recorder.samples:
            writer.writerow([f'{s.t:.3f}', f'{s.x:.4f}', f'{s.y:.4f}', f'{s.yaw:.4f}', f'{s.safe_linear:.4f}', f'{s.safe_angular:.4f}', f'{s.controller_linear:.4f}', f'{s.controller_angular:.4f}', s.waypoint_index])

    # Compute rough obstacle clearances from recorded center path.
    obstacles = {
        'person_1': (2.0, 0.45, 0.22),
        'person_2': (4.6, -0.35, 0.20),
        'bollard': (6.0, 0.82, 0.06),
    }
    wheelchair_radius = 0.34
    clearances = {}
    for name, (ox, oy, radius) in obstacles.items():
        if recorder.samples:
            min_center_dist = min(math.hypot(s.x - ox, s.y - oy) for s in recorder.samples)
        else:
            min_center_dist = float('nan')
        clearances[name] = min_center_dist - radius - wheelchair_radius

    summary_path = os.path.join(os.path.dirname(args.out), 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write('reached_final=%s\n' % reached_final)
        f.write('samples=%d\n' % len(recorder.samples))
        if recorder.samples:
            f.write('duration_s=%.3f\n' % recorder.samples[-1].t)
            f.write('start_xy=%.3f,%.3f\n' % (recorder.samples[0].x, recorder.samples[0].y))
            f.write('end_xy=%.3f,%.3f\n' % (recorder.samples[-1].x, recorder.samples[-1].y))
        for k, v in clearances.items():
            f.write('clearance_%s_m=%.3f\n' % (k, v))

    print('RESULT reached_final=%s samples=%d out=%s summary=%s' % (reached_final, len(recorder.samples), args.out, summary_path))
    for k, v in clearances.items():
        print('CLEARANCE %s %.3f m' % (k, v))
    if not reached_final:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
