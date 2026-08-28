#!/usr/bin/env python3
"""Bridge a learned ``vision_msgs/Detection3DArray`` into fusion JSON.

The learned inference engine is deliberately outside this repository: model
weights and TensorRT engines are hardware- and dataset-specific. This node is
the stable ROS1 contract for PointPillars, CenterPoint, or another 3D detector.
It transforms each detection centre into the conceptual chair-aligned ``lidar``
frame used by the existing object summary.

``lidar`` is not required to exist as a TF child. FAST-LIO commonly publishes
learned detections in ``body``; that conversion uses the same measured
body_T_lidar extrinsic as the field stack. Other source frames are transformed
to ``body`` with TF first and then through the measured extrinsic.
"""

import json
import math
import os
import sys

import numpy as np
import rospy
import tf.transformations as tft
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from body_frame import lidar_extrinsics

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
        self.body_frame = str(rospy.get_param("~body_frame", "body"))
        profile = str(rospy.get_param("~body_frame_profile", "builtin"))
        offset, rotation = lidar_extrinsics(profile)
        self.lidar_in_body = np.asarray(offset, dtype=float)
        self.lidar_to_body_rotation = np.asarray(rotation, dtype=float)
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
            "learned detection bridge: %s -> %s frame=%s model=%s "
            "body=%s profile=%s",
            self.input_topic, self.output_topic, self.output_frame,
            self.model_id, self.body_frame, profile)

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

    def _body_pose_to_lidar(self, stamped):
        p = stamped.pose.position
        q = stamped.pose.orientation
        body_point = np.asarray((p.x, p.y, p.z), dtype=float)
        lidar_point = self.lidar_to_body_rotation.T @ (
            body_point - self.lidar_in_body)

        body_R_object = tft.quaternion_matrix(
            [q.x, q.y, q.z, q.w])[:3, :3]
        lidar_R_object = self.lidar_to_body_rotation.T @ body_R_object
        matrix = np.eye(4)
        matrix[:3, :3] = lidar_R_object
        quaternion = tft.quaternion_from_matrix(matrix)

        result = PoseStamped()
        result.header = stamped.header
        result.header.frame_id = self.output_frame
        result.pose.position.x = float(lidar_point[0])
        result.pose.position.y = float(lidar_point[1])
        result.pose.position.z = float(lidar_point[2])
        result.pose.orientation.x = float(quaternion[0])
        result.pose.orientation.y = float(quaternion[1])
        result.pose.orientation.z = float(quaternion[2])
        result.pose.orientation.w = float(quaternion[3])
        return result

    def _pose_in_output(self, header, pose):
        stamped = PoseStamped()
        stamped.header = header
        stamped.pose = pose
        source_frame = stamped.header.frame_id
        if not source_frame:
            raise ValueError("detection array has no frame")
        if source_frame == self.output_frame:
            return stamped

        # The object-summary frame is conceptual and may not be broadcast in
        # TF. Reach the physical body frame first, then apply the pinned
        # body_T_lidar measurement numerically.
        if self.output_frame == "lidar":
            if source_frame != self.body_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.body_frame, source_frame, stamped.header.stamp,
                    rospy.Duration(0.10))
                stamped = tf2_geometry_msgs.do_transform_pose(
                    stamped, transform)
            return self._body_pose_to_lidar(stamped)

        transform = self.tf_buffer.lookup_transform(
            self.output_frame, source_frame, stamped.header.stamp,
            rospy.Duration(0.10))
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
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        objects = []
        for index, detection in enumerate(message.detections):
            label, score = self._class_and_score(detection)
            if label is None or score < self.minimum_score:
                continue
            size = detection.bbox.size
            dimensions = np.asarray(
                (float(size.x), float(size.y), float(size.z)), dtype=float)
            if not np.isfinite(dimensions).all() or (dimensions <= 0.0).any():
                continue
            try:
                centre = self._pose_in_output(
                    message.header, detection.bbox.center)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError) as error:
                rospy.logwarn_throttle(
                    2.0, "learned detection transform unavailable: %s", error)
                self.publish_status(
                    stamp.to_sec(), "TF_UNAVAILABLE", [],
                    source_frame=message.header.frame_id)
                return
            p = centre.pose.position
            q = centre.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(float(value)) for value in values):
                continue
            output_R_object = tft.quaternion_matrix(
                [q.x, q.y, q.z, q.w])[:3, :3]
            # The downstream JSON contract carries axis-aligned boxes. Convert
            # the detector's oriented dimensions conservatively instead of
            # silently treating object-axis lengths as frame-axis lengths.
            axis_aligned = np.abs(output_R_object) @ dimensions
            yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            objects.append({
                "id": int(index),
                "class": str(label),
                "score": round(float(score), 5),
                "x": round(float(p.x), 4),
                "y": round(float(p.y), 4),
                "z": round(float(p.z), 4),
                "yaw": round(float(yaw), 5),
                "size": [round(float(value), 4) for value in axis_aligned],
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
