"""Fail-closed ROS graph policy for static Livox localization."""

FORBIDDEN_COMMAND_PREFIX = "/cmd_vel"

def is_safe_graph(topic_publishers, external_map_to_odom_authorities):
    if external_map_to_odom_authorities:
        return False
    return not any(topic.startswith(FORBIDDEN_COMMAND_PREFIX) and publishers for topic, publishers in topic_publishers.items())
