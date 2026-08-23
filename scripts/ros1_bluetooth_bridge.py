#!/usr/bin/env python3
"""Bluetooth SPP (RFCOMM) telemetry/command bridge for the UniconLab wheelchair NUC.

ATTRIBUTION
-----------
The JSON-lines telemetry/command shape this bridge speaks was derived from
``edge-mobility-monitor`` by Park Hyeongjun (박형준),
https://github.com/Geppetto0608/edge-mobility-monitor -- used with the author's
permission, on condition of attribution.  Keep this notice in every copy.
See ``android_wheelchair_ui/NOTICE.md``.

WHICH STACK THIS TALKS TO
-------------------------
The repo contains two different worlds and only one of them runs in the field:

* ``src/wheelchair_safety`` + ``src/wheelchair_interfaces`` -- the WP0 contract
  scaffold (``/safety/state``, ``/cmd_vel_safe``, ``sidewalk``/``road_free_space``,
  ``armed``, ``reason_mask``).  It is *built*, but ``start_wheelchair_localization.sh``
  never launches it.  Do not integrate against it.
* ``livox_static_localization_ws`` -- what actually drives the chair.  This bridge
  targets that, verified by reading the live workspace on the NUC 2026-08-14.

The real command chain is::

    follower (waypoint/mpc/dwa)  ->  /cmd_vel_raw
      safety_gate.py             ->  /cmd_vel_gated
      tip_guard.py               ->  /cmd_vel
      wheel_cmd_tmp.py           ->  /wheel_cmd   (Int16MultiArray)
      uart.py                    ->  UART         -> motor controller

DESIGN RULES
------------
1. The bridge PUBLISHES EXACTLY ONE TOPIC: ``mode_cmd`` (Int16).  Everything else
   is read-only.  ``wheel_cmd_tmp.py`` rejects ``/cmd_vel`` unless the publisher's
   callerid is ``/tip_guard`` and ``/wheel_status`` unless it is ``/uart``; a
   stray publisher there sets ``fault_latched`` and jams the chair into a fault
   stop.  ``mode_cmd`` has no callerid check, which is why it is the safe lever.
2. E-STOP is ``mode_cmd = 77`` (Manual).  ``uart.py`` immediately transmits the
   motor stop frame and then ignores every autonomous ``wheel_cmd``, because
   ``CmdCallback`` only forwards while mode == 65.  It therefore holds even if
   every ROS node above it dies, and it returns the chair to joystick control.
3. E-STOP also PAUSES the follower, because waypoint_follower.py only *holds* on
   MANUAL_MODE and stays enabled -- without the pause, releasing the e-stop would
   drive off immediately.  stop.sh does both for the same reason.
4. Release is ``mode_cmd = 65`` (Auto), which transmits a stop frame first, so
   re-arming cannot lurch.  It needs an explicit confirm flag and does not
   restart the follower -- driving resumes only on a separate command.
5. Truth comes from ``/wheel_status``: ``data[1]`` is the mode the motor
   controller echoes back.  A command is only reported as effective once that
   echo agrees.
6. Telemetry never invents a value.  Fields with no source are ``null`` and named
   in ``unavailable``.

Usage
-----
    python3 scripts/ros1_bluetooth_bridge.py --self-test
    python3 scripts/ros1_bluetooth_bridge.py                    # observer only
    python3 scripts/ros1_bluetooth_bridge.py --allow-commands   # e-stop + drive
"""

import argparse
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int16, String
    from std_msgs.msg import Int16MultiArray
    from std_srvs.srv import SetBool
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from diagnostic_msgs.msg import DiagnosticArray
    DIAGNOSTICS_AVAILABLE = True
except ImportError:
    DIAGNOSTICS_AVAILABLE = False

PROTOCOL_VERSION = 3
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

AUTO_MODE = 65          # 'A' -- autonomous wheel commands accepted
MANUAL_MODE = 77        # 'M' -- joystick; autonomous commands ignored
MODE_LABELS = {AUTO_MODE: "auto", MANUAL_MODE: "manual"}

FOLLOWER_START_SERVICE = "/waypoint_follower/start"
MOTION_EPS = 0.02       # m/s and rad/s below which a Twist counts as zero

# The interpreter the operator scripts are run with. A module constant rather
# than a literal so the job tests can point it at a bash that exists on the
# machine running them; on the NUC this is always /bin/bash.
BASH = "/bin/bash"


def log(message):
    sys.stdout.write("[bt_bridge] %s\n" % message)
    sys.stdout.flush()


def _as_float(text):
    try:
        return round(float(text), 4)
    except (TypeError, ValueError):
        return None


def twist_magnitude(twist):
    return max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))


# Above this many points the route is thinned before sending. A field route is
# captured at 0.2 m spacing; at ~1 m it is visually identical on a phone-sized
# top-down view and costs a fifth of the link budget.
ROUTE_MAX_POINTS = 400


_BRINGUP_ROUTE_RE = re.compile(r'^\s*ROUTE=\"\$\{ROUTE:-(?P<path>[^}]+)\}\"')


def route_from_bringup_script(script_dir):
    """The route start_wheelchair_localization.sh would launch with.

    Second-best after the live param and far better than a filename pinned here:
    the bring-up script is where the field default is actually chosen, so reading
    it means the app follows a route promotion without this file being touched.
    """
    seen = []
    for base in (script_dir, "~"):
        if not base:
            continue
        path = os.path.join(os.path.expanduser(base),
                            "start_wheelchair_localization.sh")
        if path in seen:
            continue
        seen.append(path)
        try:
            with open(path, "r") as handle:
                for line in handle:
                    match = _BRINGUP_ROUTE_RE.match(line)
                    if match:
                        return os.path.expandvars(match.group("path"))
        except OSError:
            continue
    return None


def resolve_route_path(cli_default, script_dir=None):
    """Ask the follower which route it is actually driving.

    start_wheelchair_localization.sh launches the follower with `_route:="$ROUTE"`,
    and ROUTE is overridable by environment, so any hard-coded filename here is a
    guess. Guessing wrong is worse than showing nothing: the app would draw one
    route while the chair drove another, and the progress marker would be placed
    on a line that is not the line being followed. The launched value lands in the
    private param /waypoint_follower/route, so read that first.

    With the stack down there is no param -- and that is exactly when the app is
    open, because the operator is about to press [로컬 켜기]. So read the bring-up
    script's own ROUTE default next, and only then the CLI default. Skipping that
    step is how the app came to draw the 1897-point 20260814 algorithm route while
    the chair was pinned to the 1917-point v9 clearance route.
    """
    if ROS_AVAILABLE:
        for key in ("/waypoint_follower/route", "/waypoint_follower/_route"):
            try:
                value = rospy.get_param(key)
            except Exception:                                     # noqa: BLE001
                continue
            if value:
                log("route from follower param %s" % key)
                return str(value)
        log("follower route param not found -- reading the bring-up script")
    from_script = route_from_bringup_script(script_dir)
    if from_script:
        log("route from start_wheelchair_localization.sh: %s"
            % os.path.basename(from_script))
        return from_script
    return cli_default


def load_route(path):
    """Read the waypoint JSON the follower is driving, for the app's map view.

    Sent once per connection rather than in every telemetry frame: the field
    default is 1897 waypoints (~32 kB of JSON), which is fine once but would be
    several times the SPP budget at 2 Hz. The route and /fast_lio_icp/pose share
    the ``map`` frame -- the route was captured from that very topic -- so no
    transform is needed to draw them on the same axes.
    """
    if not path:
        return None
    path = os.path.expanduser(path)
    try:
        with open(path, "r") as handle:
            data = json.load(handle)
    except (OSError, ValueError) as exc:
        log("route not loaded (%s): %s" % (path, exc))
        return None
    points = data.get("waypoints") or []
    if not points:
        log("route %s has no waypoints" % path)
        return None

    full = [[round(float(p.get("x", 0.0)), 2), round(float(p.get("y", 0.0)), 2)]
            for p in points]
    stride = max(1, (len(full) + ROUTE_MAX_POINTS - 1) // ROUTE_MAX_POINTS)
    if stride > 1:
        slim = full[::stride]
        # The end of the route is where the chair is going; never let thinning
        # drop it, or the drawn line stops short of the actual destination.
        if slim[-1] != full[-1]:
            slim.append(full[-1])
    else:
        slim = full

    log("route loaded: %d waypoints (%d sent, stride %d) from %s"
        % (len(full), len(slim), stride, os.path.basename(path)))
    return {
        "type": "route",
        "frame": data.get("frame"),
        "body_frame_profile": data.get("body_frame_profile"),
        "count": len(slim),
        "count_full": len(full),
        "stride": stride,          # app maps wp_index -> drawn index with this
        "points": slim,
        "source": os.path.basename(path),
        "path": path,
    }


class JobRunner:
    """Runs the operator's own shell scripts, from a fixed allowlist.

    The app should press the same buttons the operator presses at the keyboard --
    ``go.sh`` already publishes ``/mode_cmd 65``, calls the follower service, and
    refuses with a written reason when a precondition fails. Reimplementing that
    in Python would mean two copies of the launch policy that drift apart, so the
    bridge shells out to the real scripts and relays their output.

    ALLOWLIST ONLY. Any bonded phone can open this link, so it must never be able
    to run an arbitrary command -- the wire protocol carries a job *name*, and the
    name is looked up in a table fixed at start-up. Nothing from the phone ever
    reaches a shell.
    """

    #  name  ->  (script filename, human label)
    #
    # trial_0727.sh is deliberately NOT here: it brings the stack up with
    # SAFETY_POLICIES=false, and a phone button that starts a guard-suppressed
    # run is not something this link should offer.
    JOBS = {
        "stack": ("start_wheelchair_localization.sh", "로컬라이제이션 스택 기동"),
        "stack_stop": ("stop_stack.sh", "스택 내리기"),
        "drive": ("go.sh", "주행 시작"),
        "halt": ("stop.sh", "주행 정지"),
    }

    def __init__(self, script_dir, enabled, env=None):
        self.script_dir = os.path.expanduser(script_dir)
        self.enabled = enabled
        # start_wheelchair_localization.sh picks its controller from $PROFILE and
        # defaults to pursuit. The field DWA runs are launched as
        # `PROFILE=dwa SAFETY_POLICIES=true start_wheelchair_localization.sh`, so a
        # bridge started from a plain shell would bring up a *different controller*
        # than the one that was last driven -- same button, same script, different
        # robot behaviour. Carry the operator's environment explicitly instead.
        self.env_overlay = dict(env or {})
        self.lock = threading.Lock()
        self.proc = None
        self.name = None
        self.label = None
        self.started_at = None
        self.finished_at = None
        self.exit_code = None
        self.tail = None
        self.log_path = None

    def resolve(self, name):
        entry = self.JOBS.get(name)
        if entry is None:
            return None, "unknown job %r" % (name,)
        path = os.path.join(self.script_dir, entry[0])
        if not os.path.isfile(path):
            return None, "%s not found at %s" % (entry[0], path)
        return path, entry[1]

    def job_env(self):
        env = os.environ.copy()
        env.update(self.env_overlay)
        return env

    def available(self):
        """Which jobs actually exist on this machine, for the UI to grey buttons."""
        out = {}
        for name in self.JOBS:
            path, _ = self.resolve(name)
            out[name] = path is not None
        return out

    def busy(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    # Stopping must never queue behind anything. stop.sh deliberately checks
    # nothing, and a stop that waits for a bring-up to finish is not a stop.
    #
    # stack_stop is here for a second reason: it is the only real abort for a
    # bring-up in progress. job_cancel signals the tracked process group, but the
    # bring-up detaches its nodes with setsid, so the sensors it already started
    # never see that signal. Making the teardown wait for the bring-up it is
    # meant to undo would be the same bug as a stop that queues.
    ALWAYS_ALLOWED = ("halt", "stack_stop")

    def _spawn(self, path, name):
        """Launch detached without touching the tracked slot."""
        try:
            handle = open("/tmp/bt_job_%s.log" % name, "wb")
        except OSError:
            handle = subprocess.DEVNULL
        return subprocess.Popen(
            [BASH, path], cwd=os.path.expanduser("~"), env=self.job_env(),
            stdin=subprocess.DEVNULL, stdout=handle,
            stderr=subprocess.STDOUT, start_new_session=True)

    def start(self, name):
        if not self.enabled:
            return False, ("script execution is disabled on this bridge "
                           "(--allow-scripts off)")
        path, label = self.resolve(name)
        if path is None:
            return False, label                      # label carries the reason

        if name in self.ALWAYS_ALLOWED:
            # Runs even while a bring-up holds the slot, and does not overwrite
            # that job's status -- the operator still needs to see how it ended.
            try:
                self._spawn(path, name)
            except OSError as exc:
                return False, "failed to launch %s: %s" % (path, exc)
            log("job '%s' started (unqueued): %s" % (name, path))
            return True, "%s 실행함 (로그: /tmp/bt_job_%s.log)" % (label, name)

        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, "'%s' is still running; wait for it or stop it first" % self.name
            self.log_path = "/tmp/bt_job_%s.log" % name
            try:
                handle = open(self.log_path, "wb")
            except OSError as exc:
                return False, "cannot open %s: %s" % (self.log_path, exc)
            try:
                # start_new_session so a bridge restart does not kill a bring-up
                # that is already half way through starting the sensors.
                self.proc = subprocess.Popen(
                    [BASH, path],
                    cwd=os.path.expanduser("~"), env=self.job_env(),
                    stdin=subprocess.DEVNULL, stdout=handle,
                    stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                handle.close()
                return False, "failed to launch %s: %s" % (path, exc)
            self.name, self.label = name, label
            self.started_at = time.time()
            self.finished_at = None
            self.exit_code = None
            self.tail = None
        log("job '%s' started: %s" % (name, path))
        return True, "%s 시작함 (로그: %s)" % (label, self.log_path)

    def cancel(self):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                return False, "no job is running"
            name = self.name
            try:
                # The job runs in its own session, so signal the whole group --
                # start_wheelchair_localization.sh spawns roslaunch children and
                # killing only the parent would leave the sensors up.
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except AttributeError:
                # No process groups on this platform (Windows bench runs).
                self.proc.terminate()
            except Exception as exc:                              # noqa: BLE001
                return False, "could not signal '%s': %s" % (name, exc)
        # Honest about the limit: start_wheelchair_localization.sh launches its
        # nodes with setsid/$DETACH, so those grandchildren sit in their own
        # sessions and this signal never reaches them. Cancelling a bring-up
        # stops the script, not necessarily the sensors it already started.
        return True, ("'%s' 에 종료 신호를 보냈습니다. 단, 이미 분리 실행된 "
                      "노드(라이다·FAST-LIO 등)는 살아있을 수 있으니 NUC에서 확인하세요."
                      % name)

    def snapshot(self):
        with self.lock:
            if self.proc is None:
                return {"job_name": None, "job_state": "idle", "job_elapsed_s": None,
                        "job_exit_code": None, "job_tail": None}
            code = self.proc.poll()
            if code is not None and self.finished_at is None:
                self.finished_at = time.time()
                self.exit_code = code
            state = ("running" if code is None
                     else "succeeded" if code == 0 else "failed")
            end = self.finished_at or time.time()
            # Last non-empty line is what the operator would be reading.
            tail = self.tail
            if self.log_path:
                try:
                    with open(self.log_path, "rb") as handle:
                        lines = [l for l in handle.read().decode(
                            "utf-8", errors="replace").splitlines() if l.strip()]
                    if lines:
                        tail = lines[-1][:160]
                except OSError:
                    pass
            self.tail = tail
            return {
                "job_name": self.name,
                "job_label": self.label,
                "job_state": state,
                "job_elapsed_s": round(end - self.started_at, 1),
                # How long ago it FINISHED, so the app can present an old result
                # as history instead of as the current state of the robot. A
                # failed bring-up from ten minutes ago must not keep shouting
                # after the operator has fixed the thing by hand.
                "job_age_s": (None if self.finished_at is None
                              else round(time.time() - self.finished_at, 1)),
                "job_exit_code": self.exit_code,
                "job_tail": tail,
            }


class BridgeState:
    """Everything the phone can see, with provenance for each field."""

    # A speed reading older than this is not evidence about what the chair is
    # doing now. Wheel odometry runs at 100 Hz, so half a second is generous.
    SPEED_TTL_S = 0.5

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.started_at = time.time()

        self.drive_mode = None          # 65 / 77, echoed by the motor controller
        # /wheel_status data[7]. A coarse battery level, almost certainly:
        # every value observed is a multiple of 11 -- 88, 77, 66 -- which reads
        # as an 8-step gauge at ~12.5% per step rather than a percentage.
        #
        # It was briefly reported here as not-a-battery on the strength of a
        # 55 s sample that happened to hold only 88 and 77. A longer look
        # settled it: at rest it read 88 early in the session and 77 two and a
        # half hours of driving later, and it dips one step (to 66) under load
        # and comes back. That is a discharge trend with load sag, not a flag.
        #
        # Which is why only the at-rest reading is reported as the level. Under
        # motor current the gauge sits a step low, and a number that drops 12%
        # every time the chair sets off is worse than no number.
        #
        # The scale itself is still unconfirmed -- 88 is merely the largest
        # value seen, not a documented full charge. Treat it as a level out of
        # 88, not as a percentage, until the controller manual says otherwise.
        self.battery_raw = None
        self.battery_at_rest = None
        self.wheel_status_stamp = None
        self.follower_status = None
        self.robot_fault = None
        # navigation view
        self.pose_x = None
        self.pose_y = None
        self.pose_yaw_deg = None
        self.pose_stamp = None
        self.loc_fitness = None
        self.loc_inlier_ratio = None
        self.loc_reason = None
        self.wp_index = None
        self.wp_total = None
        self.follower_state = None
        # FAST-LIO's /Odometry. Pose only: laserMapping publishes an all-zero
        # twist, verified on the moving chair 2026-08-23, so this is NOT a speed
        # source. Kept for freshness, and as a fallback if wheel odometry dies.
        self.odom_speed_mps = None
        self.odom_yaw_rate = None
        self.odom_stamp = None
        # /odom from base_model's odom_pub.py: encoder speed integrated in the
        # world frame, 100 Hz. This is the one that knows the chair is rolling.
        self.wheel_odom_speed_mps = None
        self.wheel_odom_yaw_rate = None
        self.wheel_odom_stamp = None
        self.cmd_raw = 0.0              # what the follower wants
        self.cmd_gated = 0.0            # what safety_gate allows
        self.cmd_out = 0.0              # what tip_guard sends to the wheels
        self.cmd_raw_stamp = None
        self.tip_guard_status = None
        self.localization_status = None
        self.objects_summary = None
        self.estop_requested_at = None
        self.last_command_detail = None

    def ground_speed(self, now=None):
        """Best available answer to "is the chair moving", and where it came from.

        Wheel odometry first. /Odometry looks like the natural source and is what
        this bridge used to read, but FAST-LIO leaves its twist at zero -- so the
        speed tile read 0.0 while the chair drove at 0.30 m/s, and the guard that
        refuses an E-STOP release on a rolling chair could never fire. Caller must
        hold the lock.

        :returns: ``(speed_mps, yaw_rate_radps, source)`` where source is
                  ``"wheel"``, ``"lio"`` or ``None`` when nothing is fresh.
        """
        now = time.time() if now is None else now
        if (self.wheel_odom_stamp is not None
                and now - self.wheel_odom_stamp <= self.SPEED_TTL_S):
            return self.wheel_odom_speed_mps, self.wheel_odom_yaw_rate, "wheel"
        if (self.odom_stamp is not None
                and now - self.odom_stamp <= self.SPEED_TTL_S):
            return self.odom_speed_mps, self.odom_yaw_rate, "lio"
        return None, None, None

    def snapshot(self, ros_connected, ttl_s, follower_available):
        with self.lock:
            self.seq += 1
            now = time.time()

            def age(stamp):
                return None if stamp is None else round(now - stamp, 2)

            wheel_age = age(self.wheel_status_stamp)
            wheel_link_ok = wheel_age is not None and wheel_age <= ttl_s
            speed, yaw_rate, speed_source = self.ground_speed(now)
            stopped = speed is not None and speed <= 0.05
            if stopped and self.battery_raw is not None:
                self.battery_at_rest = self.battery_raw
            battery_under_load = (not stopped
                                  and self.battery_raw is not None
                                  and self.battery_at_rest is not None
                                  and self.battery_raw < self.battery_at_rest)

            # safety_gate holds motion by zeroing its output while the follower is
            # still asking for movement. The gate does not publish its reason, so
            # infer the hold rather than patching a field node to expose it.
            raw_fresh = self.cmd_raw_stamp is not None and now - self.cmd_raw_stamp <= ttl_s
            motion_blocked = bool(raw_fresh and self.cmd_raw > MOTION_EPS
                                  and self.cmd_gated <= MOTION_EPS)

            # Manual mode is NOT the same as "the app e-stopped the chair".
            # start_wheelchair_localization.sh finishes with the base in manual and
            # the follower paused -- that is the normal resting state, and calling
            # it E-STOP made the app shout after every clean bring-up. It is also
            # what the joystick failsafe produces. Only claim E-STOP when this
            # bridge actually commanded one and the controller echoed it back.
            in_manual = self.drive_mode == MANUAL_MODE
            estopped = in_manual and self.estop_requested_at is not None
            pending = (self.estop_requested_at is not None
                       and not in_manual
                       and now - self.estop_requested_at < 3.0)

            frame = {
                "protocol_version": PROTOCOL_VERSION,
                "seq": self.seq,
                "timestamp": now,
                "bridge_uptime_s": round(now - self.started_at, 1),
                "ros_connected": ros_connected,

                "speed_mps": speed,
                "yaw_rate_radps": yaw_rate,
                "speed_source": speed_source,
                "commanded_mps": self.cmd_out,

                "drive_mode": MODE_LABELS.get(self.drive_mode),
                "drive_mode_raw": self.drive_mode,
                "estop_engaged": estopped,
                "estop_pending": pending,
                # Manual without an app e-stop: joystick failsafe, or simply not
                # armed yet after bring-up. Different message, same "won't drive".
                "manual_idle": in_manual and self.estop_requested_at is None,
                "motion_blocked": motion_blocked,
                "follower_start_available": follower_available,

                "wheel_link_ok": wheel_link_ok,
                "wheel_status_age_s": wheel_age,
                "odom_age_s": age(self.odom_stamp),
                "tip_guard_status": self.tip_guard_status,
                "follower_status": self.follower_status,
                "localization_status": self.localization_status,
                "localization_tracking": self.localization_status == "TRACKING",
                "objects_summary": self.objects_summary,
                "robot_fault": self.robot_fault,

                # Navigation view. Pose is the same /fast_lio_icp/pose the route
                # was captured from, so route and pose share one frame and can be
                # drawn on the same axes without any transform.
                "pose_x": self.pose_x,
                "pose_y": self.pose_y,
                "pose_yaw_deg": self.pose_yaw_deg,
                "pose_age_s": age(self.pose_stamp),
                "loc_fitness": self.loc_fitness,
                "loc_inlier_ratio": self.loc_inlier_ratio,
                "loc_reason": self.loc_reason,
                "wp_index": self.wp_index,
                "wp_total": self.wp_total,
                "follower_state": self.follower_state,
                "last_command_detail": self.last_command_detail,

                # Level, not percent -- see the note on battery_raw. The
                # at-rest reading is the one worth showing; battery_raw is what
                # the frame says right now, which sags a step under load.
                "battery_percent": None,
                "battery_level": self.battery_at_rest,
                "battery_level_max_seen": 88,
                "battery_raw": self.battery_raw,
                "battery_under_load": battery_under_load,
                # No step-level concept exists in this stack.
                "step_level": None,
            }
            # go.sh refuses to start unless all of these hold. Mirror it so the
            # app can grey out the drive button for the same reasons.
            frame["ready_to_drive"] = bool(
                wheel_link_ok
                and self.drive_mode == AUTO_MODE
                and self.localization_status == "TRACKING"
                and self.objects_summary is not None
                and not motion_blocked)
            frame["unavailable"] = sorted(k for k, v in frame.items() if v is None)
            # Fail closed: unknown reads as not-driving-safely.
            frame["display_safe_to_drive"] = bool(
                wheel_link_ok and self.drive_mode == AUTO_MODE and not motion_blocked)
            return frame


class RosLink:
    def __init__(self, state, allow_commands, node_name):
        self.state = state
        self.allow_commands = allow_commands
        self.node_name = node_name
        self.connected = False
        self.mode_pub = None

    # ------------------------------------------------------------------ setup
    @staticmethod
    def master_online(timeout=1.5):
        """Is a ROS master actually listening?

        rospy.init_node() blocks indefinitely when the master is absent -- it sits
        in select() retrying, so the bridge never reaches its first log line and
        never registers the SPP profile. That is fatal here, because the intended
        field workflow is: start the bridge, connect the phone, THEN press
        [로컬 켜기] to launch the stack (which is what starts roscore). The bridge
        must therefore come up happily with no master and attach later.
        """
        uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
        try:
            hostport = uri.split("//", 1)[1]
            host, _, port = hostport.partition(":")
            port = int(port.rstrip("/") or 11311)
        except (IndexError, ValueError):
            return False
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.settimeout(timeout)
        try:
            probe.connect((host, port))
            return True
        except OSError:
            return False
        finally:
            probe.close()

    def start(self):
        if not ROS_AVAILABLE:
            log("rospy not importable -- protocol-only mock server.")
            return
        if not self.master_online():
            log("no ROS master yet -- serving without ROS and retrying in "
                "the background (use the app's [로컬 켜기] to start the stack).")
            threading.Thread(target=self._await_master, daemon=True).start()
            return
        self._connect_ros()

    def _await_master(self):
        while not self.connected:
            time.sleep(3.0)
            if self.master_online():
                log("ROS master appeared -- attaching.")
                self._connect_ros()

    def _connect_ros(self):
        try:
            rospy.init_node(self.node_name, anonymous=False, disable_signals=True)
            rospy.Subscriber("/wheel_status", Int16MultiArray, self._wheel_cb, queue_size=5)
            rospy.Subscriber("/Odometry", Odometry, self._odom_cb, queue_size=1)
            rospy.Subscriber("/odom", Odometry, self._wheel_odom_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel_raw", Twist, self._raw_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel_gated", Twist, self._gated_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel", Twist, self._out_cb, queue_size=1)
            rospy.Subscriber("/tip_guard/status", String, self._tip_cb, queue_size=2)
            rospy.Subscriber("/waypoint_follower/status", String,
                             self._follower_status_cb, queue_size=2)
            rospy.Subscriber("/perception/objects_summary", String, self._objects_cb, queue_size=2)
            rospy.Subscriber("/robot_fault", Int16MultiArray, self._fault_cb, queue_size=2)
            rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                             self._pose_cb, queue_size=1)
            if DIAGNOSTICS_AVAILABLE:
                rospy.Subscriber("/fast_lio_icp/localization_diagnostics",
                                 DiagnosticArray, self._diag_cb, queue_size=5)

            if self.allow_commands:
                # The one and only publisher. uart.py does not check callerid here.
                # Absolute name, matching what go.sh publishes.
                self.mode_pub = rospy.Publisher("/mode_cmd", Int16, queue_size=1)
                log("command mode ON -> publishes mode_cmd only")
            else:
                log("observer mode -- no publishers created.")
            self.connected = True
            log("ROS node '%s' up." % self.node_name)
        except Exception as exc:                                  # noqa: BLE001
            log("ROS init failed (%s: %s) -- continuing without ROS."
                % (type(exc).__name__, exc))

    # -------------------------------------------------------------- callbacks
    def _wheel_cb(self, msg):
        with self.state.lock:
            self.state.wheel_status_stamp = time.time()
            if len(msg.data) > 1:
                self.state.drive_mode = int(msg.data[1])
            if len(msg.data) > 7:
                self.state.battery_raw = int(msg.data[7])

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw only; the view is top-down so roll/pitch are not wanted
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self.state.lock:
            self.state.pose_x = round(p.x, 3)
            self.state.pose_y = round(p.y, 3)
            self.state.pose_yaw_deg = round(math.degrees(math.atan2(siny, cosy)), 1)
            self.state.pose_stamp = time.time()

    # waypoint_follower.py publishes "%s wp=%d/%d v=%.2f%s"; parse it rather than
    # making the app do string surgery. A blocking reason arrives in place of the
    # state word (MANUAL_MODE, BASE_STALE, CLUSTERS_STALE, HOLD:...).
    _STATUS_RE = re.compile(r"^(?P<state>\S+)\s+wp=(?P<i>\d+)/(?P<n>\d+)")

    def _follower_status_cb(self, msg):
        text = msg.data[:120]
        match = self._STATUS_RE.match(text)
        with self.state.lock:
            self.state.follower_status = text
            if match:
                self.state.follower_state = match.group("state")
                self.state.wp_index = int(match.group("i"))
                self.state.wp_total = int(match.group("n"))
            else:
                self.state.follower_state = text.split()[0] if text.split() else None

    def _fault_cb(self, msg):
        # fault_check.py order: scan, odom, imu, roll, pitch
        names = ("scan", "odom", "imu", "roll", "pitch")
        active = [n for n, v in zip(names, msg.data) if v]
        with self.state.lock:
            self.state.robot_fault = ",".join(active) if active else "none"

    def _odom_cb(self, msg):
        v = msg.twist.twist
        speed = (v.linear.x ** 2 + v.linear.y ** 2) ** 0.5
        with self.state.lock:
            self.state.odom_speed_mps = round(speed, 3)
            self.state.odom_yaw_rate = round(v.angular.z, 3)
            self.state.odom_stamp = time.time()

    def _wheel_odom_cb(self, msg):
        # odom_pub.py integrates the encoder speed in the world frame, so linear
        # x and y are both populated and the magnitude is the ground speed.
        v = msg.twist.twist
        speed = (v.linear.x ** 2 + v.linear.y ** 2) ** 0.5
        with self.state.lock:
            self.state.wheel_odom_speed_mps = round(speed, 3)
            self.state.wheel_odom_yaw_rate = round(v.angular.z, 3)
            self.state.wheel_odom_stamp = time.time()

    def _raw_cb(self, msg):
        with self.state.lock:
            self.state.cmd_raw = round(twist_magnitude(msg), 3)
            self.state.cmd_raw_stamp = time.time()

    def _gated_cb(self, msg):
        with self.state.lock:
            self.state.cmd_gated = round(twist_magnitude(msg), 3)

    def _out_cb(self, msg):
        with self.state.lock:
            self.state.cmd_out = round(msg.linear.x, 3)

    def _tip_cb(self, msg):
        with self.state.lock:
            self.state.tip_guard_status = msg.data

    def _objects_cb(self, msg):
        # obstacle_clusters.py publishes a JSON blob, not a sentence. Dumping the
        # raw string onto a phone line pushes everything useful off screen, so
        # reduce it to the fields an operator actually reads.
        text = msg.data
        try:
            blob = json.loads(text)
            counts = blob.get("counts") or {}
            total = sum(v for v in counts.values() if isinstance(v, int)) or None
            parts = [str(blob.get("status", "?"))]
            if blob.get("band_status") and blob["band_status"] != blob.get("status"):
                parts.append("band %s" % blob["band_status"])
            if total is not None:
                parts.append("클러스터 %d" % total)
            if blob.get("bloom_filtered"):
                parts.append("필터 %s" % blob["bloom_filtered"])
            text = " · ".join(parts)
        except (ValueError, TypeError, AttributeError):
            pass
        with self.state.lock:
            self.state.objects_summary = text[:80]

    def _diag_cb(self, msg):
        try:
            worst = None
            for status in msg.status:
                if worst is None or status.level > worst.level:
                    worst = status
            if worst is not None:
                # go.sh gates the drive on this message being exactly "TRACKING",
                # so store it verbatim rather than decorating it with a level.
                fields = {}
                for item in getattr(worst, "values", []):
                    fields[item.key] = item.value
                with self.state.lock:
                    self.state.localization_status = worst.message.strip()[:80]
                    # The localizer emits a sentinel (1e9) for fitness while ICP
                    # correction is suppressed -- e.g. STATIONARY_CORRECTION_
                    # SUPPRESSED, which is normal for a parked chair. Printing
                    # "1000000000.000" on the dashboard is worse than printing
                    # nothing, so drop it and let `reason` carry the meaning.
                    fitness = _as_float(fields.get("fitness"))
                    inlier = _as_float(fields.get("inlier_ratio"))
                    suppressed = fitness is not None and fitness >= 1e6
                    self.state.loc_fitness = None if suppressed else fitness
                    self.state.loc_inlier_ratio = (
                        None if suppressed or not inlier else inlier)
                    reason = (fields.get("reason") or "").strip()
                    self.state.loc_reason = reason[:60] or None
        except Exception:                                         # noqa: BLE001
            pass

    # --------------------------------------------------------------- commands
    def _publish_mode(self, value):
        if self.mode_pub is None:
            # Two very different causes, and telling them apart matters: one is a
            # flag, the other is "the robot software is not running yet". The old
            # message blamed the flag for both and sent people hunting the wrong
            # thing in the field.
            if not self.allow_commands:
                return False, "이 브릿지는 명령이 꺼져 있습니다 (--allow-commands off)"
            if not self.connected:
                return False, ("ROS 마스터가 아직 없습니다 — [로컬 켜기]로 스택을 "
                               "먼저 기동하세요. 기동되면 자동으로 붙습니다.")
            return False, "mode_cmd 퍼블리셔가 없습니다 (브릿지 내부 오류)"
        self.mode_pub.publish(Int16(data=value))
        return True, "mode_cmd=%d published" % value

    def engage_estop(self):
        """mode_cmd=77 first, then pause the follower -- exactly what stop.sh does.

        Pausing matters more than it looks. waypoint_follower.py holds on
        MANUAL_MODE (line ~826) but stays ``enabled``; it never disables itself.
        So mode 77 alone stops the chair, and then mode 65 would let the follower
        resume *the instant the e-stop is released* -- the release itself would
        drive off. stop.sh calls the service for exactly this reason.

        Order is deliberate: publishing the topic is instant and cannot fail, so
        it happens before the service call, and a missing service never blocks
        the stop.
        """
        ok, detail = self._publish_mode(MANUAL_MODE)
        if not ok:
            return ok, detail
        with self.state.lock:
            self.state.estop_requested_at = time.time()
        paused, pause_detail = self.set_follower(False)
        return True, ("E-STOP 발동 (mode_cmd=77). uart.py가 모터 정지 프레임을 보내고 "
                      "자율 명령을 무시합니다. 팔로워 %s"
                      % ("정지됨" if paused else "정지 실패(%s) — 해제 전에 확인 필요"
                         % pause_detail[:60]))

    def release_estop(self):
        ok, detail = self._publish_mode(AUTO_MODE)
        if ok:
            with self.state.lock:
                self.state.estop_requested_at = None
            detail = ("released (mode_cmd=65). A stop frame is sent first, so the "
                      "chair does not lurch. Driving stays stopped until you start it.")
        return ok, detail

    def set_follower(self, running, ensure_auto=False):
        if not self.allow_commands:
            return False, "이 브릿지는 명령이 꺼져 있습니다 (--allow-commands off)"
        if not ROS_AVAILABLE:
            return False, "rospy를 쓸 수 없습니다"
        if not self.connected:
            return False, ("ROS 마스터가 아직 없습니다 — [로컬 켜기]로 스택을 먼저 "
                           "기동하세요")
        if ensure_auto:
            # go.sh publishes /mode_cmd 65 before calling the service. Without
            # this the follower would start while uart.py is still in manual and
            # discarding every wheel_cmd -- the chair looks armed and does not move.
            self._publish_mode(AUTO_MODE)
            time.sleep(0.3)          # let uart.py transmit its stop frame first
        try:
            rospy.wait_for_service(FOLLOWER_START_SERVICE, timeout=2.0)
            proxy = rospy.ServiceProxy(FOLLOWER_START_SERVICE, SetBool)
            response = proxy(running)
            return bool(response.success), str(response.message)[:160]
        except Exception as exc:                                  # noqa: BLE001
            return False, ("%s unavailable (%s). Only the pursuit profile "
                           "(waypoint_follower.py) offers it; mpc/dwa do not."
                           % (FOLLOWER_START_SERVICE, type(exc).__name__))

    def follower_available(self):
        if not (ROS_AVAILABLE and self.connected):
            return False
        try:
            import rosservice                                     # noqa: PLC0415
            return FOLLOWER_START_SERVICE in rosservice.get_service_list()
        except Exception:                                         # noqa: BLE001
            return False


class Session:
    """One connected phone. A bad command costs one reply, never the link."""

    def __init__(self, sock, state, ros, ttl_s, rate_hz, jobs=None, route=None,
                 route_finder=None):
        self.sock = sock
        self.state = state
        self.ros = ros
        self.jobs = jobs
        self.route = route
        self.route_finder = route_finder
        self.ttl_s = ttl_s
        self.period = 1.0 / rate_hz
        self.stop = threading.Event()
        self.write_lock = threading.Lock()
        self._follower_cache = (0.0, False)

    def send(self, obj):
        line = (json.dumps(obj) + "\n").encode("utf-8")
        try:
            with self.write_lock:
                self.sock.sendall(line)
            return True
        except OSError:
            self.stop.set()
            return False

    def _follower_available(self):
        now = time.time()
        stamp, value = self._follower_cache
        if now - stamp > 5.0:                 # service lookups are not free
            value = self.ros.follower_available()
            self._follower_cache = (now, value)
        return value

    def broadcast(self):
        # Resolve here rather than at start-up. The intended workflow is: connect
        # the app FIRST, then press [로컬 켜기] to launch the stack -- so at process
        # start the follower does not exist yet and its route param is unset.
        # Resolving once at boot would pin whatever the CLI default happens to be
        # and then keep showing it after the real route became known.
        if self.route is None and self.route_finder is not None:
            self.route = self.route_finder()
        # Route first, so the app can draw the map before any pose arrives.
        if self.route is not None:
            self.send(self.route)
        while not self.stop.is_set():
            frame = self.state.snapshot(self.ros.connected, self.ttl_s,
                                        self._follower_available())
            frame["type"] = "telemetry"
            if self.jobs is not None:
                frame.update(self.jobs.snapshot())
                frame["jobs_available"] = self.jobs.available()
                frame["scripts_enabled"] = self.jobs.enabled
                # Which controller [로컬 켜기] would actually bring up. The app
                # shows it on the button, because "start the stack" means a
                # different robot depending on this one word.
                frame["stack_profile"] = self.jobs.env_overlay.get(
                    "PROFILE", os.environ.get("PROFILE", "pursuit"))
            if not self.send(frame):
                return
            self.stop.wait(self.period)

    def serve(self):
        broadcaster = threading.Thread(target=self.broadcast, daemon=True)
        broadcaster.start()
        buffer = b""
        try:
            while not self.stop.is_set():
                try:
                    chunk = self.sock.recv(4096)
                except BlockingIOError:
                    # Belt and braces if the descriptor is non-blocking anyway:
                    # EAGAIN means "nothing yet", not "the link is gone".
                    time.sleep(0.05)
                    continue
                except OSError as exc:
                    log("link read error: %s" % exc)
                    break
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 64 * 1024:
                    log("dropping oversized command buffer")
                    buffer = b""
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    text = raw.decode("utf-8", errors="replace").strip()
                    if text:
                        self.send(self.handle(text))
        finally:
            self.stop.set()
            broadcaster.join(timeout=1.0)

    def handle(self, text):
        log("RX %s" % text)
        reply = {"type": "ack", "request": text[:200], "ok": False, "detail": ""}
        try:
            payload = json.loads(text)
        except ValueError:
            reply["detail"] = "not valid JSON"
            return reply
        if not isinstance(payload, dict):
            reply["detail"] = "expected a JSON object"
            return reply

        command = payload.get("command") or payload.get("action")
        reply["command"] = command
        try:
            if command in ("stop", "estop"):
                ok, detail = self.ros.engage_estop()
            elif command in ("estop_release", "release", "rearm", "arm"):
                ok, detail = self._release(payload)
            elif command == "drive_start":
                ok, detail = self._drive(payload, True)
            elif command == "drive_stop":
                ok, detail = self._halt()
            elif command == "stack_start":
                ok, detail = self._stack_start(payload)
            elif command == "stack_stop":
                ok, detail = self._stack_stop(payload)
            elif command == "job_cancel":
                ok, detail = (self.jobs.cancel() if self.jobs is not None
                              else (False, "스크립트 실행이 설정되지 않았습니다"))
            elif command == "route":
                ok, detail = self._resend_route()
            elif command == "ping":
                ok, detail = True, "pong"
            elif command in ("mode", "step"):
                ok, detail = False, (
                    "not applicable to this stack. Drive mode is auto/manual via "
                    "mode_cmd and is controlled by estop/estop_release; there is "
                    "no step level.")
            else:
                ok, detail = False, "unknown command %r" % (command,)
            reply["ok"], reply["detail"] = ok, detail
            with self.state.lock:
                self.state.last_command_detail = ("%s: %s" % (command, detail))[:160]
            return reply
        except Exception as exc:                                  # noqa: BLE001
            reply["detail"] = "%s: %s" % (type(exc).__name__, exc)
            return reply

    def _resend_route(self):
        """Re-resolve and re-send the route frame.

        The route goes out once per connection, ahead of any telemetry, so a
        client that was not listening yet never sees it -- which is exactly what
        happened: the device picker owns the shared Bluetooth callback until the
        dashboard swaps itself in a moment later, and its onLineReceived is
        empty. The map then stayed blank for the whole session while telemetry
        streamed happily, because telemetry repeats and the route did not.

        Re-resolving rather than replaying the cached copy also picks up the real
        route once the stack is up: connect first, press [로컬 켜기], and the
        follower's own param finally exists.
        """
        if self.route_finder is None:
            return False, "경로가 설정되지 않았습니다 (--route \"\")"
        route = self.route_finder()
        if route is None:
            return False, "경로 파일을 읽지 못했습니다"
        self.route = route
        if not self.send(route):
            return False, "경로 전송 실패"
        return True, "경로 %d개 지점 전송 (%s)" % (route.get("count_full", 0),
                                              route.get("source", "?"))

    def _release(self, payload):
        """Two-step: the app must send confirm=true, and the chair must be stopped."""
        if not payload.get("confirm"):
            return False, ("confirmation required: resend with \"confirm\": true "
                           "after the rider has checked it is safe to proceed")
        with self.state.lock:
            moving, _yaw, source = self.state.ground_speed()
            link_age = (None if self.state.wheel_status_stamp is None
                        else time.time() - self.state.wheel_status_stamp)
        if link_age is None or link_age > self.ttl_s:
            return False, "refusing: no fresh /wheel_status, cannot confirm the chair is stopped"
        # Fail closed. "No speed reading" used to sail straight past this check,
        # which is the same as asserting the chair is stopped on no evidence.
        if moving is None:
            return False, ("refusing: no fresh speed reading (/odom, /Odometry) — "
                           "cannot confirm the chair is stopped")
        if moving > 0.05:
            return False, ("refusing: chair is still moving (%.2f m/s, %s odometry)"
                           % (moving, source))
        return self.ros.release_estop()

    def _halt(self):
        """Stop must never refuse, so fall back to the direct path when the script
        is unavailable -- mirroring stop.sh, which checks nothing on purpose."""
        if self.jobs is not None and self.jobs.enabled:
            ok, detail = self.jobs.start("halt")
            if ok:
                return ok, detail
            fallback_ok, fallback_detail = self.ros.set_follower(False)
            return fallback_ok, "%s / 직접 정지: %s" % (detail, fallback_detail)
        return self.ros.set_follower(False)

    def _stack_start(self, payload):
        if self.jobs is None or not self.jobs.enabled:
            return False, "스크립트 실행이 꺼져 있습니다 (--allow-scripts off)"
        if not payload.get("confirm"):
            return False, ("확인 필요: \"confirm\": true 를 함께 보내세요 — "
                           "라이다·IMU·측위 노드가 기동됩니다 (수 분 소요)")
        return self.jobs.start("stack")

    def _stack_stop(self, payload):
        """Undo [로컬 켜기]. Confirmed, because it ends the drive and the sensors."""
        if self.jobs is None or not self.jobs.enabled:
            return False, "스크립트 실행이 꺼져 있습니다 (--allow-scripts off)"
        if not payload.get("confirm"):
            return False, ("확인 필요: \"confirm\": true 를 함께 보내세요 — "
                           "주행을 멈추고 라이다·측위 노드를 모두 내립니다")
        return self.jobs.start("stack_stop")

    def _drive(self, payload, running):
        """Mirror go.sh's refusals rather than starting into a broken precondition."""
        with self.state.lock:
            in_manual = self.state.drive_mode == MANUAL_MODE
            app_estop = self.state.estop_requested_at is not None
            tracking = self.state.localization_status == "TRACKING"
            objects = self.state.objects_summary
            link_age = (None if self.state.wheel_status_stamp is None
                        else time.time() - self.state.wheel_status_stamp)
        # Both refusals are "the base is in manual", but they are not the same
        # situation and they do not have the same next step. Calling the resting
        # state an E-STOP sent the operator looking for an emergency that never
        # happened -- and the app greys out [E-STOP 해제] unless one is engaged,
        # so the advice pointed at a disabled button. Say which one it is.
        if in_manual and app_estop:
            return False, ("거부: E-STOP이 걸려 있습니다. [E-STOP 해제]로 먼저 "
                           "푼 다음 주행하세요.")
        if in_manual:
            return False, ("거부: 베이스가 수동 모드입니다 (기동 직후의 정상 상태이거나 "
                           "조종간 페일세이프). [자동 모드 전환]으로 시동을 건 뒤 "
                           "주행하세요.")
        if not payload.get("confirm"):
            return False, ("confirmation required: resend with \"confirm\": true -- "
                           "the chair will begin moving")
        if link_age is None or link_age > self.ttl_s:
            return False, "refusing: wheel base is silent (/wheel_status stale)"
        if objects is None:
            return False, ("refusing: object tracking is silent "
                           "(/perception/objects_summary) -- an empty object list "
                           "reads exactly like clear road")
        if not tracking:
            return False, ("refusing: localization is '%s', not TRACKING"
                           % (self.state.localization_status or "silent"))
        # Prefer the operator's own go.sh: it re-checks authoritatively and its
        # refusal text is the wording the team already knows. Reimplementing the
        # launch policy in Python would mean two copies that drift apart.
        if self.jobs is not None and self.jobs.enabled:
            return self.jobs.start("drive")
        return self.ros.set_follower(running, ensure_auto=True)


# ---------------------------------------------------------------- transports

def start_debug_tcp(args, state, ros, jobs=None, route=None, route_finder=None):
    """Serve the same Session on 127.0.0.1 for bench checks. Loopback only."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.debug_tcp))
    listener.listen(2)
    # rospy installs a process-wide default socket timeout, which a socket created
    # after init_node inherits -- so accept() would keep raising socket.timeout.
    listener.settimeout(None)
    log("DEBUG probe on tcp://127.0.0.1:%d (loopback only)" % args.debug_tcp)

    def accept_loop():
        while True:
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                # A client that vanishes between SYN and accept() raises
                # ECONNABORTED, which is not the listener dying -- but treating
                # every OSError as fatal returned from this thread, dropped the
                # last reference to `listener`, and took the probe port down for
                # the rest of the bridge's life with nothing in the log. Only
                # give up once the socket really is closed.
                if listener.fileno() < 0:
                    return
                log("debug probe accept failed (%s: %s) -- still listening"
                    % (type(exc).__name__, exc))
                time.sleep(0.1)
                continue
            threading.Thread(
                target=lambda c=conn: (Session(c, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve(),
                                       c.close()),
                daemon=True).start()

    threading.Thread(target=accept_loop, daemon=True).start()


def serve_bluez_profile(args, state, ros, jobs=None, route=None, route_finder=None):
    """Let bluetoothd own the RFCOMM listen and publish the SPP SDP record.

    Android's createRfcommSocketToServiceRecord(SPP_UUID) needs an SDP record; a
    bare AF_BLUETOOTH bind publishes none.  Registering an org.bluez.Profile1
    needs no root and no PyBluez.  Verified on the NUC as user mprp3.
    """
    import dbus                                                   # noqa: PLC0415
    import dbus.mainloop.glib                                      # noqa: PLC0415
    import dbus.service                                            # noqa: PLC0415
    from gi.repository import GLib                                 # noqa: PLC0415

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    path = "/uniconlab/wheelchair/spp"

    class Profile(dbus.service.Object):
        @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
        def Release(self):
            log("BlueZ released the profile.")

        @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
        def NewConnection(self, device, fd, properties):
            raw = fd.take()
            log("phone connected: %s (fd %d)" % (device, raw))
            client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                   socket.BTPROTO_RFCOMM, fileno=raw)
            # BlueZ hands the descriptor over in non-blocking mode, so recv()
            # returns EAGAIN straight away and the session would tear down the
            # instant the phone connected. Session.serve() expects to block.
            client.setblocking(True)

            def run():
                try:
                    Session(client, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    log("phone disconnected: %s" % device)

            threading.Thread(target=run, daemon=True).start()

        @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
        def RequestDisconnection(self, device):
            log("disconnect requested by %s" % device)

    Profile(bus, path)
    manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"),
                             "org.bluez.ProfileManager1")
    options = {
        "Name": "UniconLab Wheelchair Bridge",
        "Role": "server",
        "Channel": dbus.UInt16(args.channel),
        "RequireAuthentication": dbus.Boolean(True),
        "RequireAuthorization": dbus.Boolean(False),
        "AutoConnect": dbus.Boolean(False),
    }
    try:
        manager.RegisterProfile(path, SPP_UUID.lower(), options)
    except Exception as exc:                                      # noqa: BLE001
        log("RegisterProfile failed (%s) -- falling back to a raw RFCOMM bind." % exc)
        return serve_raw_socket(args, state, ros, jobs, route, route_finder)

    log("SPP profile registered with bluetoothd (SDP record published, channel %d)"
        % args.channel)
    log("serving at %.1f Hz, commands=%s -- Ctrl-C to stop"
        % (args.rate, "on" if args.allow_commands else "OFF"))
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        log("shutting down.")
    finally:
        try:
            manager.UnregisterProfile(path)
        except Exception:                                         # noqa: BLE001
            pass
    return 0


def register_sdp_record(channel):
    """Legacy SDP registration for the raw-socket transport."""
    try:
        import bluetooth                                          # noqa: PLC0415
        bluetooth.advertise_service(
            None, "UniconLab Wheelchair Bridge",
            service_id=SPP_UUID, service_classes=[SPP_UUID, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE])
        log("SDP record advertised via PyBluez.")
        return True
    except Exception:                                             # noqa: BLE001
        pass
    try:
        subprocess.run(["sdptool", "add", "--channel=%d" % channel, "SP"],
                       check=True, capture_output=True, timeout=10)
        log("SDP record added via sdptool.")
        return True
    except Exception as exc:                                      # noqa: BLE001
        log("WARNING: no SDP record (%s). Prefer --transport bluez, which needs "
            "no root and publishes one properly." % exc)
        return False


def serve_raw_socket(args, state, ros, jobs=None, route=None, route_finder=None):
    if not hasattr(socket, "AF_BLUETOOTH"):
        log("this Python has no AF_BLUETOOTH; run the bridge on the Linux NUC.")
        return 1
    try:
        server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server.bind((args.bdaddr, args.channel))
        server.listen(1)
    except OSError as exc:
        log("RFCOMM bind failed on channel %d: %s: %s" % (args.channel, type(exc).__name__, exc))
        log("on the NUC check: rfkill list bluetooth / systemctl status bluetooth / "
            "another process already on this channel")
        return 1
    register_sdp_record(args.channel)
    log("listening on RFCOMM channel %d (rate %.1f Hz, commands=%s)"
        % (args.channel, args.rate, "on" if args.allow_commands else "OFF"))
    try:
        while True:
            client, info = server.accept()
            log("phone connected: %s" % (info,))
            try:
                Session(client, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
            finally:
                try:
                    client.close()
                except OSError:
                    pass
                log("phone disconnected.")
    except KeyboardInterrupt:
        log("shutting down.")
        return 0
    finally:
        server.close()


# ---------------------------------------------------------------- self-test

def self_test(args, state, ros, jobs=None, route=None, route_finder=None):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]

    def accept_once():
        conn, _ = listener.accept()
        Session(conn, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
        conn.close()

    threading.Thread(target=accept_once, daemon=True).start()
    client = socket.create_connection(("127.0.0.1", port))
    stream = client.makefile("rb")

    def next_of(kind):
        deadline = time.time() + 6.0
        while time.time() < deadline:
            line = stream.readline()
            if not line:
                return None
            obj = json.loads(line.decode())
            if obj.get("type") == kind:
                return obj
        return None

    def send(obj):
        client.sendall((json.dumps(obj) + "\n").encode())

    failures = []

    def check(name, condition, note=""):
        print("  %-56s %s%s" % (name, "PASS" if condition else "FAIL",
                                "" if not note else "  (%s)" % note))
        if not condition:
            failures.append(name)

    print("\n--- protocol self-test ---")
    first = next_of("telemetry")
    check("telemetry frame arrives", first is not None)
    if first is None:
        return 1
    print("  first frame: %s" % json.dumps(first)[:150])
    check("battery reported as null, not a fake number", first["battery_percent"] is None)
    check("no scaffold fields leak in (armed/reason_mask)",
          "armed" not in first and "reason_mask" not in first)
    check("drive_mode exposed", "drive_mode" in first and "estop_engaged" in first)
    check("fail-closed display flag is False without ROS",
          first["display_safe_to_drive"] is False)

    client.sendall(b"not json\n")
    check("garbage line does NOT drop the link", next_of("telemetry") is not None)

    send({"command": "step", "step": "abc"})
    ack = next_of("ack")
    check("malformed legacy command answers, link survives",
          ack is not None and ack["ok"] is False)

    send({"command": "estop_release"})
    ack = next_of("ack")
    check("release without confirm is refused",
          ack is not None and ack["ok"] is False and "confirmation" in ack["detail"])

    send({"command": "estop_release", "confirm": True})
    ack = next_of("ack")
    check("release with confirm still refused when /wheel_status is stale",
          ack is not None and ack["ok"] is False and "wheel_status" in ack["detail"])

    send({"command": "drive_start", "confirm": True})
    ack = next_of("ack")
    check("drive_start refused with commands disabled",
          ack is not None and ack["ok"] is False)

    send({"command": "ping"})
    ack = next_of("ack")
    check("ping answers", ack is not None and ack["ok"] is True)

    client.close()
    listener.close()
    print("--- %s ---\n" % ("all checks passed" if not failures
                            else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
    return 1 if failures else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--channel", type=int, default=1)
    # BDADDR_ANY must be spelled out: Python's AF_BLUETOOTH bind rejects "" with
    # "bad bluetooth address" on Linux as well as Windows. The inherited bridge
    # used "" and could never have bound on the NUC either.
    parser.add_argument("--bdaddr", default="00:00:00:00:00:00")
    parser.add_argument("--rate", type=float, default=2.0, help="telemetry Hz (default 2)")
    parser.add_argument("--ttl", type=float, default=1.0,
                        help="seconds before /wheel_status counts as stale (default 1.0)")
    parser.add_argument("--allow-commands", action="store_true",
                        help="permit e-stop, release and drive start/stop")
    parser.add_argument("--allow-scripts", action="store_true",
                        help="run the allowlisted operator scripts "
                             "(start_wheelchair_localization.sh, go.sh, stop.sh). "
                             "Separate from --allow-commands on purpose: publishing a "
                             "topic and spawning processes on the robot are different "
                             "risks, and this one is opt-in.")
    parser.add_argument("--route", default="~/wheelchair_localization_src/routes/"
                                          "20260814_route_algorithm_waypoints.json",
                        help="waypoint JSON sent to the app once per connection "
                             "for the map view; empty string disables it")
    parser.add_argument("--job-env", action="append", default=[], metavar="KEY=VALUE",
                        help="environment for the launched scripts, repeatable. "
                             "The field DWA runs need "
                             "--job-env PROFILE=dwa --job-env SAFETY_POLICIES=true; "
                             "without it the bring-up script defaults to pursuit.")
    parser.add_argument("--script-dir", default="~",
                        help="directory holding the operator scripts (default ~)")
    parser.add_argument("--node-name", default="wheelchair_bt_bridge")
    parser.add_argument("--transport", choices=("auto", "bluez", "socket"), default="auto",
                        help="'bluez' registers an SPP profile so an SDP record exists")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--debug-tcp", type=int, default=0, metavar="PORT",
                        help="also serve the protocol on 127.0.0.1:PORT for bench checks")
    args = parser.parse_args(argv)

    if args.rate <= 0:
        parser.error("--rate must be positive")

    state = BridgeState()
    ros = RosLink(state, args.allow_commands, args.node_name)
    # Scripts spawn processes on the robot, so they need BOTH gates.
    job_env = {}
    for item in args.job_env:
        key, _, value = item.partition("=")
        if not key or not _:
            log("ignoring malformed --job-env %r (expected KEY=VALUE)" % item)
            continue
        job_env[key] = value
    jobs = JobRunner(args.script_dir, bool(args.allow_scripts and args.allow_commands),
                     job_env)
    # Deliberately NOT resolved here -- see Session.broadcast.
    route = None
    route_finder = lambda: load_route(
        resolve_route_path(args.route, args.script_dir))
    if args.allow_scripts and not args.allow_commands:
        log("--allow-scripts ignored: it also requires --allow-commands.")
    if not args.self_test:
        ros.start()
        if jobs.enabled:
            missing = [n for n, ok in jobs.available().items() if not ok]
            log("script execution ON (dir=%s)%s"
                % (jobs.script_dir,
                   "" if not missing else "; MISSING: %s" % ", ".join(sorted(missing))))

    if args.self_test:
        return self_test(args, state, ros, jobs, route, route_finder)

    if args.debug_tcp:
        start_debug_tcp(args, state, ros, jobs, route, route_finder)

    transport = args.transport
    if transport == "auto":
        try:
            import dbus                                           # noqa: F401,PLC0415
            from gi.repository import GLib                        # noqa: F401,PLC0415
            transport = "bluez"
        except ImportError:
            log("python3-dbus/gi unavailable -- raw RFCOMM bind (no SDP record).")
            transport = "socket"
    if transport == "bluez":
        return serve_bluez_profile(args, state, ros, jobs, route, route_finder)
    return serve_raw_socket(args, state, ros, jobs, route, route_finder)


if __name__ == "__main__":
    sys.exit(main())
