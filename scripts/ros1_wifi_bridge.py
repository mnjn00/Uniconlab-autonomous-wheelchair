#!/usr/bin/env python3
"""
ROS 1 Noetic Wi-Fi HTTP Bridge for UniconLab Autonomous Wheelchair.
Exposes ROS 1 topics as JSON REST API on port 8081 for the Android Wheelchair UI app.
"""

import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String, Int32
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# Shared state guarded by lock
state_lock = threading.Lock()
telemetry_state = {
    "speed_mps": 0.0,
    "angular_speed_radps": 0.0,
    "battery_percent": 95,
    "battery_status": "OK",
    "mode": "manual",
    "step_level": 3,
    "safety_state": "ARMED",
    "system_state": "READY",
    "geofence_ok": True,
    "last_updated": time.time()
}

mode_pub = None
estop_pub = None
step_pub = None


class RosBridgeHttpHandler(BaseHTTPRequestHandler):
    def _send_json(self, status_code, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/api/health", "/api/health/"):
            self._send_json(200, {
                "status": "ok",
                "system": "nuc_ros1_wheelchair",
                "ros1_connected": ROS_AVAILABLE and (not rospy.is_shutdown() if ROS_AVAILABLE else False),
                "timestamp": time.time()
            })
        elif path in ("/api/telemetry", "/api/telemetry/"):
            with state_lock:
                data = dict(telemetry_state)
            self._send_json(200, data)
        elif path.startswith("/api/users/"):
            self._send_json(200, {"status": "ok", "presets": [], "logs": []})
        else:
            self._send_json(404, {"error": "endpoint not found"})

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        content_len = int(self.headers.get("Content-Length", 0))
        body_bytes = self.rfile.read(content_len) if content_len > 0 else b"{}"

        try:
            payload = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
        except json.JSONDecodeError:
            payload = {}

        if path == "/api/mode":
            new_mode = payload.get("mode", "manual")
            with state_lock:
                telemetry_state["mode"] = new_mode
            if ROS_AVAILABLE and mode_pub:
                mode_pub.publish(String(data=new_mode))
            self._send_json(200, {"status": "success", "mode": new_mode})

        elif path == "/api/step":
            new_step = payload.get("step", 3)
            with state_lock:
                telemetry_state["step_level"] = new_step
            if ROS_AVAILABLE and step_pub:
                step_pub.publish(Int32(data=new_step))
            self._send_json(200, {"status": "success", "step_level": new_step})

        elif path == "/api/stop":
            with state_lock:
                telemetry_state["speed_mps"] = 0.0
                telemetry_state["safety_state"] = "DISARMED / E-STOP"
            if ROS_AVAILABLE and estop_pub:
                estop_pub.publish(Bool(data=True))
            self._send_json(200, {"status": "success", "stopped": True})

        elif path == "/api/preset":
            preset_id = payload.get("preset_id", "base")
            with state_lock:
                telemetry_state["active_preset"] = preset_id
            self._send_json(200, {"status": "success", "preset_id": preset_id})

        elif path == "/api/users/log":
            self._send_json(200, {"status": "logged"})

        else:
            self._send_json(404, {"error": "unknown POST endpoint"})

    def log_message(self, format, *args):
        pass


def ros_cmd_vel_cb(msg):
    speed = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
    angular = msg.angular.z
    with state_lock:
        telemetry_state["speed_mps"] = round(speed, 2)
        telemetry_state["angular_speed_radps"] = round(angular, 2)
        telemetry_state["last_updated"] = time.time()


def ros_mode_cb(msg):
    with state_lock:
        telemetry_state["mode"] = msg.data


def ros_geofence_cb(msg):
    with state_lock:
        telemetry_state["geofence_ok"] = bool(msg.data)
        telemetry_state["safety_state"] = "ARMED" if msg.data else "GEOFENCE_VIOLATION"


def start_ros_listeners():
    global mode_pub, estop_pub, step_pub
    if not ROS_AVAILABLE:
        print("[WARNING] ROS 1 (rospy) not found. Running in standalone mock HTTP server mode.")
        return

    try:
        rospy.init_node("ros1_wifi_bridge", anonymous=True, disable_signals=True)
        rospy.Subscriber("/cmd_vel_safe", Twist, ros_cmd_vel_cb, queue_size=1)
        rospy.Subscriber("/runtime/mode", String, ros_mode_cb, queue_size=1)
        rospy.Subscriber("/safety/geofence_ok", Bool, ros_geofence_cb, queue_size=1)

        mode_pub = rospy.Publisher("/runtime/mode_request", String, queue_size=1)
        estop_pub = rospy.Publisher("/wheelchair/e_stop", Bool, queue_size=1)
        step_pub = rospy.Publisher("/wheelchair/step_level", Int32, queue_size=1)

        print("[ROS1 Bridge] ROS 1 Node 'ros1_wifi_bridge' initialized successfully.")
    except Exception as e:
        print(f"[ROS1 Bridge] ROS initialization error: {e}")


def main():
    start_ros_listeners()

    port = 8081
    server = HTTPServer(("0.0.0.0", port), RosBridgeHttpHandler)
    print(f"[ROS1 Bridge] HTTP JSON API Server running on http://0.0.0.0:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[ROS1 Bridge] Shutting down HTTP server...")
        server.server_close()


if __name__ == "__main__":
    main()
