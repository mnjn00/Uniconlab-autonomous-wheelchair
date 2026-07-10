#!/usr/bin/env python3
"""Run and record a static+dynamic obstacle avoidance Gazebo demo.

The demo publishes commands through the verified safety chain:
  /cmd_vel_nav -> safety_gate -> /cmd_vel_safe -> /wheelchair_base_controller/cmd_vel

It also moves the red cylinder model with /gazebo/set_model_state so the recorded
trajectory contains both a moving obstacle and a static obstacle.
"""

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist


@dataclass
class Sample:
    t: float
    robot_x: float
    robot_y: float
    robot_yaw: float
    moving_x: float
    moving_y: float
    safe_linear: float
    safe_angular: float
    controller_linear: float
    controller_angular: float
    waypoint_index: int


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


def moving_obstacle_xy(t):
    """A red cylinder crosses the corridor while the wheelchair approaches.

    The crossing is intentionally slow enough that the wheelchair can pass behind/below it,
    while the object remains visibly moving throughout the Gazebo recording.
    """
    x = 2.35
    # Start at the top of the corridor and move downward across the wheelchair's nominal path.
    y = 0.95 - min(max(t, 0.0), 40.0) * (1.90 / 40.0)
    return x, y


class Recorder:
    def __init__(self, robot_name, moving_name):
        self.robot_name = robot_name
        self.moving_name = moving_name
        self.robot_pose = None
        self.moving_pose = None
        self.safe_cmd = Twist()
        self.controller_cmd = Twist()
        self.samples = []
        self.start_time = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel_safe', Twist, self.safe_cb, queue_size=10)
        rospy.Subscriber('/wheelchair_base_controller/cmd_vel', Twist, self.controller_cb, queue_size=10)

    def model_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            self.robot_pose = (p.x, p.y, yaw_from_quat(q), t)
        if self.moving_name in msg.name:
            idx = msg.name.index(self.moving_name)
            p = msg.pose[idx].position
            self.moving_pose = (p.x, p.y)

    def safe_cb(self, msg):
        self.safe_cmd = msg

    def controller_cb(self, msg):
        self.controller_cmd = msg

    def record(self, waypoint_index, moving_xy):
        if self.robot_pose is None:
            return
        x, y, yaw, t = self.robot_pose
        mx, my = moving_xy
        self.samples.append(Sample(
            t=t, robot_x=x, robot_y=y, robot_yaw=yaw,
            moving_x=mx, moving_y=my,
            safe_linear=self.safe_cmd.linear.x,
            safe_angular=self.safe_cmd.angular.z,
            controller_linear=self.controller_cmd.linear.x,
            controller_angular=self.controller_cmd.angular.z,
            waypoint_index=waypoint_index,
        ))


def set_moving_obstacle(proxy, model_name, x, y):
    state = ModelState()
    state.model_name = model_name
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = 0.85
    state.pose.orientation.w = 1.0
    state.twist.linear.x = 0.0
    state.twist.linear.y = 0.0
    state.twist.angular.z = 0.0
    state.reference_frame = 'world'
    proxy(state)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/workspace/artifacts/static_dynamic_demo/trajectory.csv')
    ap.add_argument('--robot-name', default='wheelchair')
    ap.add_argument('--moving-name', default='moving_obstacle_red_person')
    ap.add_argument('--max-seconds', type=float, default=80.0)
    args = ap.parse_args()

    rospy.init_node('static_dynamic_obstacle_demo', anonymous=True)
    pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=10)
    rec = Recorder(args.robot_name, args.moving_name)

    rospy.loginfo('waiting for /gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rospy.loginfo('waiting for robot pose')
    deadline = time.time() + 30.0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rec.robot_pose is None and time.time() < deadline:
        rate.sleep()
    if rec.robot_pose is None:
        raise RuntimeError('Timed out waiting for wheelchair pose from /gazebo/model_states')

    # The route deliberately goes below the moving red person, then above the static blue box.
    # The steering remains online waypoint tracking so the actual Gazebo pose/clearance is measured.
    waypoints = [
        (0.0, 0.0),
        (0.75, -0.22),
        (1.45, -0.78),
        (2.85, -0.92),
        (3.35, -0.82),
        (3.95, 0.62),
        (5.45, 0.65),
        (6.35, 0.30),
        (7.25, 0.02),
    ]
    wp_i = 1
    reached_final = False
    start_wall = time.time()

    while not rospy.is_shutdown() and time.time() - start_wall < args.max_seconds:
        if rec.robot_pose is None:
            rate.sleep()
            continue
        sim_t = rec.robot_pose[3]
        mx, my = moving_obstacle_xy(sim_t)
        try:
            set_moving_obstacle(set_model_state, args.moving_name, mx, my)
        except Exception as exc:
            rospy.logwarn('set_model_state failed: %s', exc)

        x, y, yaw, _ = rec.robot_pose
        tx, ty = waypoints[wp_i]
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = norm_angle(desired - yaw)

        if dist < 0.25:
            rospy.loginfo('reached waypoint %d/%d at x=%.2f y=%.2f', wp_i, len(waypoints)-1, x, y)
            if wp_i >= len(waypoints) - 1:
                reached_final = True
                break
            wp_i += 1
            continue

        # Extra caution: if the moving obstacle is still close to the lower bypass corridor,
        # bias the robot farther downward and reduce speed. This is a visible dynamic avoidance behavior.
        moving_clearance_est = math.hypot(x - mx, y - my) - 0.22 - 0.34
        cmd = Twist()
        heading_factor = max(0.12, math.cos(err))
        base_speed = clamp(0.17 + 0.23 * min(dist, 1.0), 0.10, 0.40)
        if abs(err) > 1.25:
            base_speed = 0.06
        if moving_clearance_est < 0.38 and x < 3.0:
            base_speed *= 0.55
            err = norm_angle(err - 0.25)  # steer a bit lower/behind the moving obstacle
        cmd.linear.x = base_speed * heading_factor
        cmd.angular.z = clamp(1.55 * err, -0.95, 0.95)
        pub.publish(cmd)
        rec.record(wp_i, (mx, my))
        rate.sleep()

    stop = Twist()
    for _ in range(20):
        pub.publish(stop)
        if rec.robot_pose is not None:
            mx, my = moving_obstacle_xy(rec.robot_pose[3])
            try:
                set_moving_obstacle(set_model_state, args.moving_name, mx, my)
            except Exception:
                pass
            rec.record(wp_i, (mx, my))
        rate.sleep()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'robot_x', 'robot_y', 'robot_yaw', 'moving_x', 'moving_y',
                    'safe_linear', 'safe_angular', 'controller_linear', 'controller_angular', 'waypoint_index'])
        for s in rec.samples:
            w.writerow([f'{s.t:.3f}', f'{s.robot_x:.4f}', f'{s.robot_y:.4f}', f'{s.robot_yaw:.4f}',
                        f'{s.moving_x:.4f}', f'{s.moving_y:.4f}', f'{s.safe_linear:.4f}', f'{s.safe_angular:.4f}',
                        f'{s.controller_linear:.4f}', f'{s.controller_angular:.4f}', s.waypoint_index])

    robot_radius = 0.34
    moving_radius = 0.22
    static_x, static_y = 4.65, -0.45
    static_radius = math.hypot(0.60 / 2.0, 0.55 / 2.0)
    if rec.samples:
        min_moving = min(math.hypot(s.robot_x - s.moving_x, s.robot_y - s.moving_y) for s in rec.samples) - robot_radius - moving_radius
        min_static = min(math.hypot(s.robot_x - static_x, s.robot_y - static_y) for s in rec.samples) - robot_radius - static_radius
    else:
        min_moving = float('nan')
        min_static = float('nan')

    summary_path = os.path.join(os.path.dirname(args.out), 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write('reached_final=%s\n' % reached_final)
        f.write('samples=%d\n' % len(rec.samples))
        if rec.samples:
            f.write('duration_s=%.3f\n' % rec.samples[-1].t)
            f.write('start_xy=%.3f,%.3f\n' % (rec.samples[0].robot_x, rec.samples[0].robot_y))
            f.write('end_xy=%.3f,%.3f\n' % (rec.samples[-1].robot_x, rec.samples[-1].robot_y))
        f.write('min_clearance_moving_obstacle_m=%.3f\n' % min_moving)
        f.write('min_clearance_static_obstacle_m=%.3f\n' % min_static)
        f.write('command_chain=/cmd_vel_nav -> /cmd_vel_safe -> /wheelchair_base_controller/cmd_vel\n')

    print('RESULT reached_final=%s samples=%d out=%s summary=%s' % (reached_final, len(rec.samples), args.out, summary_path))
    print('CLEARANCE moving_obstacle %.3f m' % min_moving)
    print('CLEARANCE static_obstacle %.3f m' % min_static)
    if not reached_final or min_moving < 0.05 or min_static < 0.05:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
