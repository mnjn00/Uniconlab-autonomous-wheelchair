#!/usr/bin/env python3
"""Hybrid control-geometry producer without fixed-map subtraction.

The legacy ``obstacle_clusters.py`` deliberately removes returns represented by
the immutable localization map. That is useful for localization exclusions,
but it is the wrong collision contract: a wall, a bench recorded in the map,
or a person standing close to a mapped surface can disappear from the planner
while the independent raw safety gate still sees it and vetoes every command.

This wrapper reuses the field-tested accumulation, rider filtering, clustering,
profiles, tracking, and publishing code, but replaces only the map-membership
filter with an identity filter. The hybrid launcher remaps its dynamic boxes
to a candidate topic; ``localization_exclusion_boxes.py`` republishes only
people and moving/uncertain objects to the localizer, so mapped walls do not get
removed from registration.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import obstacle_clusters as legacy


class KeepAllGeometry(object):
    """Drop-in replacement for FixedMapFilter used only in this process."""

    def __init__(self, *_args, **_kwargs):
        pass

    def retain_novel(self, points_lidar, _map_T_lidar):
        return points_lidar


def _positive_int(name, default):
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool):
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    if value <= 0:
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    return value


def main():
    # ObstacleClusters constructs the filter in __init__. Replace the class
    # before construction so the 500+ MB map is not loaded into a second
    # KD-tree merely to be ignored afterwards.
    legacy.FixedMapFilter = KeepAllGeometry
    node = legacy.ObstacleClusters()

    # Defaults preserve the field-tested clustering thresholds. They are ROS
    # parameters so bag replay may evaluate a more sensitive thin-object
    # profile without editing the implementation.
    legacy.MIN_CELL_POINTS = _positive_int(
        "min_cell_points", legacy.MIN_CELL_POINTS)
    legacy.MIN_CLUSTER_POINTS = _positive_int(
        "min_cluster_points", legacy.MIN_CLUSTER_POINTS)
    legacy.MAX_CLUSTERS = _positive_int(
        "max_clusters", legacy.MAX_CLUSTERS)
    if legacy.MIN_CLUSTER_POINTS < legacy.MIN_CELL_POINTS:
        raise rospy.ROSInitException(
            "~min_cluster_points must be >= ~min_cell_points")

    rospy.logwarn(
        "hybrid geometry: fixed-map subtraction is OFF for collision and "
        "avoidance; mapped surfaces remain visible (cell=%d cluster=%d max=%d)",
        legacy.MIN_CELL_POINTS, legacy.MIN_CLUSTER_POINTS,
        legacy.MAX_CLUSTERS)
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
