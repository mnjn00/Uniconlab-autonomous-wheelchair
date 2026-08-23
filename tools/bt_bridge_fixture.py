#!/usr/bin/env python3
"""Publish the live field stack's topics so the Bluetooth operator app can be
exercised end to end without powering the wheel base.

This stands in for the nodes ros1_bluetooth_bridge.py subscribes to -- and only
for those. It never touches /wheel_cmd or the UART, so no frame can reach the
motors.

The mode echo is the part that matters. uart.py is not running, so nothing would
otherwise answer a /mode_cmd; this fixture echoes it back on /wheel_status
data[1] the way the motor controller does, which is what makes E-STOP, E-STOP
release and the estop_pending window observable from the phone.

Scenarios are switched live through a ROS param, so one run covers the whole
dashboard:

    rosparam set /bt_fixture/scenario hold

    nominal      auto, TRACKING, rolling along the route
    hold         safety_gate holding: cmd_vel_raw > 0 while cmd_vel_gated == 0
    manual       base in manual (joystick failsafe / post-bring-up rest state)
    loc_lost     localization diagnostics stop saying TRACKING
    no_objects   /perception/objects_summary goes silent -- go.sh refuses
    wheel_silent /wheel_status goes silent -- the wheel link reads stale
    fault        fault_check bits set
    tip          tip guard warns
    parked       stopped, auto, everything healthy (the ready-to-drive state)
"""

import argparse
import json
import math
import os
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

AUTO_MODE = 65
MANUAL_MODE = 77
RATE_HZ = 10.0

DEFAULT_ROUTE = os.path.expanduser(
    "~/wheelchair_localization_src/routes/20260816_route_v9_clearance_waypoints.json")


def load_route(path):
    """Waypoints in the map frame, so the pose the fixture reports is a place the
    chair could actually be rather than a circle drawn next to the route."""
    try:
        with open(path) as handle:
            blob = json.load(handle)
    except (IOError, ValueError) as exc:
        rospy.logwarn("fixture: no route (%s) -- driving a synthetic circle", exc)
        return []
    points = blob.get("waypoints") or blob.get("points") or []
    out = []
    for item in points:
        if isinstance(item, dict):
            if item.get("x") is None:
                continue
            out.append((float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append((float(item[0]), float(item[1])))
    return out


class _Silent(object):
    """Stands in for a publisher this run is not allowed to own."""

    def publish(self, *_args, **_kwargs):
        pass


class Fixture(object):
    def __init__(self, route, skip=()):
        self.route = route
        # Half the stack is often already up -- localization runs long before the
        # base is powered. Publishing over a real node would make the dashboard
        # alternate between the fixture's value and the robot's at 10 Hz, which
        # looks like flapping hardware. Yield those topics instead.
        self.skip = set(skip)
        self.mode = MANUAL_MODE      # bring-up leaves the base in manual
        self.battery = 87
        self.wp = 0
        self.follower_running = False
        self.started_at = time.time()

        self.pub_wheel = self._publisher("/wheel_status", Int16MultiArray, 5)
        self.pub_odom = self._publisher("/Odometry", Odometry, 1)
        self.pub_raw = self._publisher("/cmd_vel_raw", Twist, 1)
        self.pub_gated = self._publisher("/cmd_vel_gated", Twist, 1)
        self.pub_out = self._publisher("/cmd_vel", Twist, 1)
        self.pub_tip = self._publisher("/tip_guard/status", String, 2)
        self.pub_follower = self._publisher("/waypoint_follower/status", String, 2)
        self.pub_objects = self._publisher("/perception/objects_summary", String, 2)
        self.pub_fault = self._publisher("/robot_fault", Int16MultiArray, 2)
        self.pub_pose = self._publisher("/fast_lio_icp/pose",
                                        PoseWithCovarianceStamped, 1)
        self.pub_diag = self._publisher("/fast_lio_icp/localization_diagnostics",
                                        DiagnosticArray, 5)

        rospy.Subscriber("/mode_cmd", Int16, self.on_mode, queue_size=5)
        # Only waypoint_follower.py offers this, and the bridge greys out the
        # drive buttons when it is missing -- so offering it here is what makes
        # the enabled state testable.
        rospy.Service("/waypoint_follower/start", SetBool, self.on_start)

    def _publisher(self, topic, msg_type, queue_size):
        if topic in self.skip:
            rospy.loginfo("fixture: NOT publishing %s (skipped)", topic)
            return _Silent()
        return rospy.Publisher(topic, msg_type, queue_size=queue_size)

    # ------------------------------------------------------------- inputs
    def on_mode(self, msg):
        value = int(msg.data)
        if value in (AUTO_MODE, MANUAL_MODE):
            rospy.loginfo("fixture: mode_cmd=%d -- echoing on /wheel_status", value)
            self.mode = value
            if value == MANUAL_MODE:
                self.follower_running = False

    def on_start(self, req):
        self.follower_running = bool(req.data)
        return SetBoolResponse(success=True,
                               message="fixture follower %s"
                                       % ("running" if req.data else "stopped"))

    # ------------------------------------------------------------ helpers
    def scenario(self):
        return rospy.get_param("/bt_fixture/scenario", "nominal")

    def pose_at(self, index):
        if not self.route:
            t = (time.time() - self.started_at) * 0.2
            return 3.0 * math.cos(t), 3.0 * math.sin(t), t + math.pi / 2
        i = index % len(self.route)
        x, y = self.route[i]
        nx, ny = self.route[(i + 1) % len(self.route)]
        return x, y, math.atan2(ny - y, nx - x)

    def publish_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(msg)

    def publish_diag(self, message, fitness, inlier, reason):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if message == "TRACKING" else DiagnosticStatus.WARN
        status.name = "fast_lio_icp: localization"
        status.message = message
        status.values = [KeyValue(key="fitness", value="%.4f" % fitness),
                         KeyValue(key="inlier_ratio", value="%.3f" % inlier),
                         KeyValue(key="reason", value=reason)]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.pub_diag.publish(array)

    def twist(self, linear, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    # --------------------------------------------------------------- loop
    def spin(self):
        rate = rospy.Rate(RATE_HZ)
        total = max(len(self.route), 1)
        while not rospy.is_shutdown():
            what = self.scenario()
            moving = (what in ("nominal", "hold") and self.mode == AUTO_MODE)

            if moving and what == "nominal":
                self.wp = (self.wp + 1) % total

            # Wheel link. Silent means the bridge should report the link stale
            # rather than showing the last mode it happened to see.
            if what != "wheel_silent":
                frame = Int16MultiArray()
                frame.data = [72, self.mode, 0, 0, 0, 0, 0, self.battery]
                self.pub_wheel.publish(frame)

            speed = 0.40 if (moving and what == "nominal") else 0.0
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.twist.twist.linear.x = speed
            odom.twist.twist.angular.z = 0.01 if speed else 0.0
            self.pub_odom.publish(odom)

            # The gate publishes a zero Twist while holding; it does not go
            # silent. Inferring the hold depends on both being present.
            if what == "hold":
                self.pub_raw.publish(self.twist(0.40))
                self.pub_gated.publish(self.twist(0.0))
                self.pub_out.publish(self.twist(0.0))
            else:
                self.pub_raw.publish(self.twist(speed))
                self.pub_gated.publish(self.twist(speed))
                self.pub_out.publish(self.twist(speed))

            self.pub_tip.publish(String(
                data="WARN roll=6.1deg" if what == "tip" else "OK"))

            if self.mode == MANUAL_MODE:
                follower = "MANUAL_MODE wp=%d/%d v=0.00" % (self.wp, total)
            elif what == "hold":
                follower = "HOLD:OBSTACLE wp=%d/%d v=0.00" % (self.wp, total)
            elif self.follower_running:
                follower = "RUN wp=%d/%d v=%.2f" % (self.wp, total, speed)
            else:
                follower = "IDLE wp=%d/%d v=0.00" % (self.wp, total)
            self.pub_follower.publish(String(data=follower))

            if what != "no_objects":
                self.pub_objects.publish(String(data=json.dumps({
                    "status": "OK",
                    "band_status": "CLEAR" if what != "hold" else "BLOCKED",
                    "counts": {"static": 2, "dynamic": 1 if what == "hold" else 0},
                    "bloom_filtered": 0,
                })))

            fault = Int16MultiArray()
            fault.data = [0, 0, 1, 0, 0] if what == "fault" else [0, 0, 0, 0, 0]
            self.pub_fault.publish(fault)

            x, y, yaw = self.pose_at(self.wp)
            self.publish_pose(x, y, yaw)

            if what == "loc_lost":
                self.publish_diag("LOST", 0.94, 0.11, "INSUFFICIENT_INLIERS")
            else:
                self.publish_diag("TRACKING", 0.0121, 0.87, "OK")

            rate.sleep()


def already_published(topic):
    """True when a node other than this fixture already owns the topic."""
    try:
        import rosgraph                                           # noqa: PLC0415
        master = rosgraph.Master("/bt_bridge_fixture")
        for name, publishers in master.getSystemState()[0]:
            if name == topic:
                return [p for p in publishers if p != "/bt_bridge_fixture"] != []
    except Exception:                                             # noqa: BLE001
        return False
    return False


REAL_STACK_TOPICS = ("/fast_lio_icp/pose", "/fast_lio_icp/localization_diagnostics",
                     "/Odometry", "/cmd_vel_raw", "/cmd_vel_gated", "/cmd_vel",
                     "/tip_guard/status", "/waypoint_follower/status",
                     "/perception/objects_summary", "/robot_fault", "/wheel_status")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route", nargs="?", default=DEFAULT_ROUTE)
    parser.add_argument("--skip", action="append", default=[], metavar="TOPIC",
                        help="do not publish this topic (repeatable)")
    parser.add_argument("--defer-to-live", action="store_true",
                        help="skip every topic a real node is already publishing")
    args = parser.parse_args()

    rospy.init_node("bt_bridge_fixture", anonymous=False)
    skip = list(args.skip)
    if args.defer_to_live:
        skip += [t for t in REAL_STACK_TOPICS if already_published(t)]
    route = load_route(args.route)
    rospy.loginfo("fixture: %d waypoints from %s", len(route), args.route)
    rospy.set_param("/bt_fixture/scenario",
                    rospy.get_param("/bt_fixture/scenario", "nominal"))
    Fixture(route, skip).spin()


if __name__ == "__main__":
    main()
