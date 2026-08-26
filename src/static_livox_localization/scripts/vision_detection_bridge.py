#!/usr/bin/env python3
"""Bridge a learned ``vision_msgs/Detection3DArray`` into fusion JSON.

The learned inference engine is deliberately outside this repository: model
weights and TensorRT engines are hardware- and dataset-specific.  This node is
the stable ROS1 contract for PointPillars, CenterPoint, or another 3D detector.
It transforms each detection centre into the configured lidar frame and emits
``/perception/learned_objects_summary``.

When ``vision_msgs`` is not installed the node refuses at startup with an
explicit message; the hybrid runtime keeps geometric-only operation unless
learning was declared required.
"""

import json
import math

import rospy
import tf.transformations as tft
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    from vision_msgs.msg import Detection3DArray
except ImportError as error:  # pragma: no cover - depends on target ROS install
    raise ImportError(
        "vision_detection_bridge requires ros-noetic-vision-msgs; "
        "install it or run the hybrid stack in geometric-only mode") from error


class VisionDetectionBridge:
    def __init__(self):
        rospy.init_node("vision_detection_bridge")
        self.input_topic = str(rospy.get_param(
            "~input_topic", "/pointpillars/detections"))
        self.output_topic = str(rospy.get_param(
            "~output_topic", "/perception/learned_objects_summary"))
        self.output_frame = str(rospy.get_param("~output_frame", "lidar"))
        self.model_id = str(rospy.get_param(
            "~model_id", "unidentified_3d_detector"))
        self.minimum_score = float(rospy.get_param("~minimum_score", 0.05))
        if not math.isfinite(self.minimum_score) or \
                not 0.0 <= self.minimum_score <= 1.0:
            raise rospy.ROSInitException("~minimum_score must be within [0, 1]")

        raw_map = rospy.get_param("~class_map", {
            "0": "vehicle",
            "1": "person",
            "2": "two_wheeler",
            "3": "obstacle",
        })
        if not isinstance(raw_map, dict):
            raise rospy.ROSInitException("~class_map must be a dictionary")
        self.class_map = {str(key): str(value) for key, value in raw_map.items()}

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=1)
        rospy.Subscriber(self.input_topic, Detection3DArray,
                         self.on_detections, queue_size=2)
        rospy.loginfo(
            "learned detection bridge: %s -> %s frame=%s model=%s",
            self.input_topic, self.output_topic, self.output_frame,
            self.model_id)

    @staticmethod
    def _hypothesis(result):
        return result.hypothesis if hasattr(result, "hypothesis") else result

    def _class_and_score(self, detection):
        best = None
        for result in detection.results:
            hypothesis = self._hypothesis(result)
            score = getattr(hypothesis, "score", None)
            try:
                score = float(score)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(score):
                continue
            identifier = str(getattr(hypothesis, "id", ""))
            if best is None or score > best[1]:
                best = (identifier, score)
        if best is None:
            return None, 0.0
        return self.class_map.get(best[0], best[0] or "obstacle"), best[1]

    def _pose_in_output(self, header, pose):
        stamped = PoseStamped()
        stamped.header = header
        stamped.pose = pose
        if not stamped.header.frame_id:
            raise ValueError("detection array has no frame")
        if stamped.header.frame_id == self.output_frame:
            return stamped
        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            stamped.header.frame_id,
            stamped.header.stamp,
            rospy.Duration(0.10),
        )
        return tf2_geometry_msgs.do_transform_pose(stamped, transform)

    def publish_status(self, stamp_s, status, objects, source_frame=""):
        payload = {
            "stamp": float(stamp_s),
            "status": str(status),
            "frame": self.output_frame,
            "source_frame": source_frame,
            "model_id": self.model_id,
            "objects": objects,
        }
        self.pub.publish(String(data=json.dumps(
            payload, separators=(",", ":"), sort_keys=True)))

    def on_detections(self, message):
        stamp = message.header.stamp
        if stamp.isZero():
            stamp = rospy.Time.now()
        objects = []
        for index, detection in enumerate(message.detections):
            label, score = self._class_and_score(detection)
            if label is None or score < self.minimum_score:
                continue
            size = detection.bbox.size
            dimensions = (float(size.x), float(size.y), float(size.z))
            if not all(math.isfinite(value) and value > 0.0
                       for value in dimensions):
                continue
            try:
                centre = self._pose_in_output(
                    message.header, detection.bbox.center)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError) as error:
                rospy.logwarn_throttle(
                    2.0, "learned detection TF unavailable: %s", error)
                self.publish_status(
                    stamp.to_sec(), "TF_UNAVAILABLE", [],
                    source_frame=message.header.frame_id)
                return
            p = centre.pose.position
            q = centre.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(float(value)) for value in values):
                continue
            yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            objects.append({
                "id": int(index),
                "class": str(label),
                "score": round(float(score), 5),
                "x": round(float(p.x), 4),
                "y": round(float(p.y), 4),
                "z": round(float(p.z), 4),
                "yaw": round(float(yaw), 5),
                "size": [round(value, 4) for value in dimensions],
                "motion": "unknown",
            })
        self.publish_status(
            stamp.to_sec(), "OK", objects,
            source_frame=message.header.frame_id)


if __name__ == "__main__":
    try:
        VisionDetectionBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
