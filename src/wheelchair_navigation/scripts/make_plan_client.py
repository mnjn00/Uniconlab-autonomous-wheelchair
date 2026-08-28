#!/usr/bin/env python3
"""Plan a global route with navfn and hand it to the follower.

This is the runtime half of the auto-route workflow. The drop-safe costmap
(baked offline by bake_dropsafe_costmap.py) is served by map_server, and
move_base runs navfn over it. Given a start and goal in the map frame, this
node calls /move_base/make_plan (nav_msgs/GetPlan), writes the returned
nav_msgs/Path to JSON, and - if asked - runs path_to_route_assets.py to emit
the route waypoints and safety band the follower loads.

navfn with allow_unknown=false (set in move_base.yaml) will not plan through a
cell the costmap marked lethal, and the costmap marked lethal every cell the
3D terrain analysis refused - kerb, grade, obstruction, unmapped, or outside
the analysed corridor. So the path this returns cannot cross a drop the map
knows about. That is the first layer; the band is the second.

The node refuses a plan whose start or goal is not in free space rather than
letting navfn snap to the nearest free cell, because on a kerb edge the
nearest free cell is the wrong side of it.

Run on the NUC, with auto_planner.launch bringing map_server + move_base up:

    rosrun wheelchair_navigation make_plan_client.py \
        _start_x:=1.0 _start_y:=1.5 _goal_x:=156.0 _goal_y:=-84.3 \
        _out_path:=/tmp/plan.json

then convert to follower assets:

    tools/path_to_route_assets.py /tmp/plan.json --costmap dropsafe.npz \
        --out-route routes/auto_route.json --out-band-prefix routes/auto_band \
        --map-pcd <map.pcd>
"""

import json
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan


def make_pose(x, y, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rospy.init_node("make_plan_client", anonymous=True)
    start_x = rospy.get_param("~start_x")
    start_y = rospy.get_param("~start_y")
    goal_x = rospy.get_param("~goal_x")
    goal_y = rospy.get_param("~goal_y")
    frame = rospy.get_param("~map_frame", "map")
    tolerance = float(rospy.get_param("~tolerance", 0.0))
    out_path = rospy.get_param("~out_path", "/tmp/global_plan.json")

    rospy.loginfo("waiting for /move_base/make_plan ...")
    rospy.wait_for_service("/move_base/make_plan", timeout=60.0)
    make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

    start = make_pose(start_x, start_y, frame)
    goal = make_pose(goal_x, goal_y, frame)
    resp = make_plan(start, goal, tolerance)
    if not resp.plan.poses:
        rospy.logerr("navfn returned no path: start or goal is not in free "
                     "space, or no drop-safe route exists between them. Check "
                     "the costmap and the start/goal cells.")
        return 1

    poses = [{"pose": {"position": {"x": p.pose.position.x,
                                    "y": p.pose.position.y,
                                    "z": p.pose.position.z}}}
             for p in resp.plan.poses]
    with open(out_path, "w") as f:
        json.dump({"frame": frame, "poses": poses}, f)
    rospy.loginfo("navfn plan: %d poses -> %s", len(poses), out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
