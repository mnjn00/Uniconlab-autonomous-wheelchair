#!/usr/bin/env python3
"""Final-stage anti-tip guard: predictive pitch-rate trip + adaptive accel governor.

Sits as the LAST node before the wheel base (safety_gate -> tip_guard ->
wheel_cmd_tmp.py), independent of routes, obstacles, and the map - it only
cares about not tipping over, regardless of whether a rider is aboard.

Why angle thresholds are not enough: the follower's static pitch abort
(+-8 deg) only fires once the chair is ALREADY tipping. With momentum
already built up - especially on a light, unladen frame where the same
wheel torque produces much more angular acceleration - a stop command at
that point is often too late. This node instead predicts pitch a short
time ahead from the CURRENT angular rate and trips before the static
threshold is ever reached.

Sensor fusion, honestly stated:
  - fast path: raw gyro rate from the VectorNav VN-100 (/vectornav/IMU,
    200 Hz) - lowest latency signal for "is it rotating right now, and
    how fast". Only angular_velocity is read, so the VN's m/s^2
    acceleration units (the Livox driver reports g) never enter here.
    The VN frame was measured to sit within 0.4 deg of the lidar in roll
    and pitch, so the pitch axis index and sign carry over unchanged.
  - reference path: fused pitch from /Odometry (FAST-LIO, ~10 Hz) - this
    IS the "LiDAR information" contribution: FAST-LIO's orientation is
    LiDAR-inertial fused, not IMU alone. Direct point-cloud ground-plane
    tilt sensing was tried and is NOT usable here (the MID360's blind
    ring means it cannot see near-field ground at all, as established
    while debugging the drop guard), so LiDAR enters through the fused
    localization estimate rather than raw geometry.
  - self-check: because the exact IMU mounting axis/sign was never
    verified on hardware, this node cross-validates the raw gyro rate
    against the ACTUAL pitch change measured between fused Odometry
    samples. If they don't correlate, predictive tripping is disabled
    and the node falls back to a fixed conservative accel cap instead of
    trusting a possibly-backwards fast path. Sign-symmetric trip logic
    (see should_trip) means a wrong axis SIGN degrades to "trips on any
    large rate" rather than "fails to trip", but the correlation check
    still gates trust in the predictive (early) part of the response.

Feedback loop for "loaded or empty, either way": accel_budget adapts down
whenever recent pitch-rate magnitude rises above a caution level and
recovers slowly when calm. An empty, tippy chair naturally produces
bigger pitch-rate for the same command and gets throttled harder; a
loaded, stable chair is not needlessly restricted.

Trip response default is a hard brake to zero (linear.x -> 0), not an
active reverse. Braking while pitching back already applies a
nose-down reaction that helps arrest a backward tip. An active bounded
counter-motion (briefly commanding a small reverse speed while the tilt
is still growing) is available via ~enable_counter_motion but defaults
to OFF and only ever engages once the IMU/Odometry correlation
self-check has passed - never on unverified axis mapping, since a wrong
sign there would turn a corrective reverse into an actively harmful one.

This node reduces tip risk substantially but is NOT a substitute for a
physical anti-tip caster/wheel - sensor latency, wheel slip, and uneven
ground mean no software layer can give an absolute guarantee.
"""

import math
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import tf.transformations as tft

GUARD_HZ = 50.0
INPUT_STALE_S = 0.6
IMU_STALE_S = 0.3
ODOM_STALE_S = 0.5

LOOKAHEAD_S = 0.35
# Tip-over is a FAST pitch change; hills are SLOW ones. Trip on deviation
# from a slow terrain baseline (EMA of fused pitch), never on absolute
# pitch alone - the aejimun route legitimately climbs ~5 deg slopes.
BASELINE_TAU_S = 3.0
TRIP_DEV_RAD = math.radians(4.5)
TRIP_RATE_RAD_S = math.radians(20.0)
# angular-acceleration early trip: a tip event ramps the pitch rate hard;
# rate AND accel in the same direction beyond these both -> trip earliest
TRIP_ACCEL_RAD_S2 = math.radians(80.0)
TRIP_ACCEL_MIN_RATE_RAD_S = math.radians(8.0)
RELEASE_DEV_RAD = math.radians(1.5)
RELEASE_RATE_RAD_S = math.radians(3.0)

CAUTION_RATE_RAD_S = math.radians(6.0)
GOVERNOR_MIN_ACCEL = 0.05
GOVERNOR_MAX_ACCEL = 0.30
GOVERNOR_RECOVER_PER_S = 0.05
GOVERNOR_CUT_FACTOR = 0.5
HARD_DECEL = 1.0

CORRELATION_WINDOW = 20
CORRELATION_MIN_AGREEMENT = 0.6
CORRELATION_MIN_DELTA_RAD = math.radians(0.5)
FALLBACK_ACCEL = 0.12

COUNTER_SPEED_MAX = 0.15
COUNTER_ENGAGE_PITCH_RAD = TRIP_DEV_RAD

# Closed-loop climb assist: compare requested speed with MEASURED ground
# speed (FAST-LIO odometry). Uphill sag -> boost the outgoing command so
# the wheels get more torque; downhill overspeed -> negative boost brakes.
# The boost can only GROW at the tip governor's adaptive accel budget, so
# rising pitch-rate throttles or freezes the assist before it can tip the
# chair - torque assistance and tip suppression form one coupled loop.
CLIMB_DEADBAND_MPS = 0.08
# assist only engages on an actual slope: at flat-ground launch the
# measured speed always lags the ramping command, and boosting that gap
# just produces a jerky start (observed: commanded 1.3 m/s at pull-away)
CLIMB_MIN_PITCH_RAD = math.radians(3.5)
LAUNCH_SPEED_MPS = 0.2
LAUNCH_ACCEL = 0.12
CLIMB_GAIN = 0.8
CLIMB_DECAY_PER_S = 0.6
CLIMB_BOOST_MAX = 1.0
CLIMB_BRAKE_MAX = 0.5

SPEED_MEASURE_WINDOW_S = 0.4

# Hard ceiling on what this stage may ever publish, independent of what
# arrives on /cmd_vel_gated. Set at the follower's MAX_SPEED rather than the
# gate's HARD_V_LIMIT so the assist cannot push the chair past the fastest
# speed the route planner is allowed to ask for.
ABSOLUTE_V_LIMIT = 1.5


class TipGuard:
    def __init__(self):
        rospy.init_node("tip_guard")
        imu_topic = rospy.get_param("~imu_topic", "/vectornav/IMU")
        self.enable_counter_motion = rospy.get_param(
            "~enable_counter_motion", False)
        gyro_pitch_axis = rospy.get_param("~gyro_pitch_axis", "y")
        self.gyro_sign = float(rospy.get_param("~gyro_pitch_sign", 1.0))
        self._gyro_index = {"x": 0, "y": 1, "z": 2}[gyro_pitch_axis]

        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.current_speed = 0.0

        self.imu_stamp = rospy.Time(0)
        self.pitch_rate = 0.0
        self.pitch_accel = 0.0
        self._last_rate = None
        self._last_rate_stamp = None

        self.odom_stamp = rospy.Time(0)
        self.fused_pitch = 0.0
        self.measured_speed = 0.0
        self._last_odom_xy = None
        self._last_odom_t = None
        self.climb_boost = 0.0
        self.baseline_pitch = None
        self._odom_track = deque()
        self._last_fused_pitch = None
        self._last_fused_stamp = None
        self._gyro_since_last_fused = deque()

        self._agreement = deque(maxlen=CORRELATION_WINDOW)
        self.axis_config_ok = None  # unknown until enough samples seen

        self.tripped = False
        self.accel_budget = GOVERNOR_MAX_ACCEL
        self.status = ""

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/tip_guard/status", String, queue_size=2)
        rospy.Subscriber("/cmd_vel_gated", Twist, self.on_raw, queue_size=1)
        rospy.Subscriber(imu_topic, Imu, self.on_imu, queue_size=50)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=20)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    # ------------------------------------------------------------ callbacks
    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    def on_imu(self, message):
        now = rospy.Time.now()
        self.imu_stamp = now
        rate = message.angular_velocity.x if self._gyro_index == 0 else (
            message.angular_velocity.y if self._gyro_index == 1
            else message.angular_velocity.z)
        self.pitch_rate = self.gyro_sign * rate
        stamp_s = now.to_sec()
        if self._last_rate is not None and self._last_rate_stamp is not None:
            dt = stamp_s - self._last_rate_stamp
            if 1e-4 < dt < 0.1:
                raw_accel = (self.pitch_rate - self._last_rate) / dt
                self.pitch_accel += 0.3 * (raw_accel - self.pitch_accel)
        self._last_rate = self.pitch_rate
        self._last_rate_stamp = stamp_s
        self._gyro_since_last_fused.append((now.to_sec(), self.pitch_rate))
        cutoff = now.to_sec() - 2.0
        while self._gyro_since_last_fused and \
                self._gyro_since_last_fused[0][0] < cutoff:
            self._gyro_since_last_fused.popleft()

    def on_odom(self, message):
        now = rospy.Time.now()
        self.odom_stamp = now
        q = message.pose.pose.orientation
        _, pitch, _ = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.fused_pitch = pitch

        # measured forward speed from odometry displacement projected on
        # the body forward axis (signed)
        position = message.pose.pose.position
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        forward = R[:3, 0]
        self._odom_track.append(
            (now.to_sec(), position.x, position.y, position.z))
        cutoff = now.to_sec() - SPEED_MEASURE_WINDOW_S - 0.2
        while self._odom_track and self._odom_track[0][0] < cutoff:
            self._odom_track.popleft()
        if len(self._odom_track) >= 2:
            t0, x0, y0, z0 = self._odom_track[0]
            t1, x1, y1, z1 = self._odom_track[-1]
            dt = t1 - t0
            if dt > 0.05:
                displacement = ((x1 - x0) * forward[0] +
                                (y1 - y0) * forward[1] +
                                (z1 - z0) * forward[2])
                self.measured_speed = displacement / dt

        if self.baseline_pitch is None:
            self.baseline_pitch = pitch
        else:
            alpha = min(1.0, 0.1 / BASELINE_TAU_S)
            self.baseline_pitch += alpha * (pitch - self.baseline_pitch)

        stamp = now.to_sec()
        if self._last_fused_pitch is not None and self._last_fused_stamp is not None:
            dt = stamp - self._last_fused_stamp
            actual_delta = pitch - self._last_fused_pitch
            if dt > 0.02 and abs(actual_delta) > CORRELATION_MIN_DELTA_RAD:
                window = [r for t, r in self._gyro_since_last_fused
                         if self._last_fused_stamp <= t <= stamp]
                if window:
                    integrated = sum(window) / len(window) * dt
                    self._agreement.append(
                        1.0 if integrated * actual_delta > 0 else 0.0)
        self._last_fused_pitch = pitch
        self._last_fused_stamp = stamp

        if len(self._agreement) >= CORRELATION_WINDOW:
            ratio = sum(self._agreement) / len(self._agreement)
            ok = ratio >= CORRELATION_MIN_AGREEMENT
            if ok != self.axis_config_ok:
                if not ok:
                    rospy.logwarn(
                        "tip_guard: IMU/odometry pitch correlation only "
                        "%.0f%% - predictive trip DISABLED, capping accel "
                        "conservatively. Verify ~imu_topic / "
                        "~gyro_pitch_axis / ~gyro_pitch_sign.",
                        100.0 * ratio)
                else:
                    rospy.loginfo(
                        "tip_guard: IMU/odometry pitch correlation "
                        "confirmed (%.0f%%) - predictive trip active",
                        100.0 * ratio)
            self.axis_config_ok = ok

    # ------------------------------------------------------------ logic
    def deviation(self):
        """Pitch relative to the slow terrain baseline - large only when
        the chair rotates FAST relative to the ground it stands on."""
        if self.baseline_pitch is None:
            return 0.0
        return self.fused_pitch - self.baseline_pitch

    def should_trip(self):
        """Sign-symmetric on the deviation: trips only when the deviation
        is GROWING in either direction (or the raw rotation rate alone is
        already extreme). Steady slopes converge into the baseline and
        never trip; a fast tip event outruns the baseline and does."""
        if abs(self.pitch_rate) > TRIP_RATE_RAD_S:
            return True
        if abs(self.pitch_rate) > TRIP_ACCEL_MIN_RATE_RAD_S and \
                abs(self.pitch_accel) > TRIP_ACCEL_RAD_S2 and \
                self.pitch_accel * self.pitch_rate > 0.0:
            return True
        if not self.axis_config_ok:
            return False
        dev = self.deviation()
        predicted = dev + self.pitch_rate * LOOKAHEAD_S
        growing = (predicted * dev) >= 0.0
        return growing and abs(predicted) > TRIP_DEV_RAD

    def should_release(self):
        """Self-recovering on slopes: the baseline converges to the
        terrain pitch within a few seconds, so deviation decays to zero
        even while parked mid-hill."""
        return abs(self.deviation()) < RELEASE_DEV_RAD and \
            abs(self.pitch_rate) < RELEASE_RATE_RAD_S

    def counter_motion_target(self):
        """Small bounded reverse command while still actively tipping.
        Only ever engages with a verified IMU axis mapping; direction is
        derived from the SAME sign-symmetric growth test as the trip
        itself, so a bad axis config cannot silently reverse it."""
        if not (self.enable_counter_motion and self.axis_config_ok):
            return 0.0
        dev = self.deviation()
        predicted = dev + self.pitch_rate * LOOKAHEAD_S
        still_growing = (predicted * dev) >= 0.0 and \
            abs(dev) > COUNTER_ENGAGE_PITCH_RAD
        if not still_growing:
            return 0.0
        # counter in the direction opposite the wheelchair's own forward
        # command history: back off from whatever direction it was
        # driving when the tip began.
        direction = -1.0 if self.current_speed >= 0.0 else 1.0
        return direction * COUNTER_SPEED_MAX

    def update_governor(self, dt):
        if abs(self.pitch_rate) > CAUTION_RATE_RAD_S:
            self.accel_budget = max(
                GOVERNOR_MIN_ACCEL, self.accel_budget * GOVERNOR_CUT_FACTOR)
        else:
            ceiling = GOVERNOR_MAX_ACCEL if self.axis_config_ok else FALLBACK_ACCEL
            self.accel_budget = min(
                ceiling, self.accel_budget + GOVERNOR_RECOVER_PER_S * dt)

    def spin(self):
        rate = rospy.Rate(GUARD_HZ)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1e-3, (now - last).to_sec())
            last = now

            self.update_governor(dt)

            if self.tripped:
                if self.should_release():
                    self.tripped = False
            elif self.should_trip():
                self.tripped = True
                rospy.logwarn(
                    "tip_guard TRIP: pitch=%.1fdeg rate=%.1fdeg/s",
                    math.degrees(self.fused_pitch),
                    math.degrees(self.pitch_rate))

            stale = ((now - self.raw_stamp).to_sec() > INPUT_STALE_S or
                     (now - self.imu_stamp).to_sec() > IMU_STALE_S or
                     (now - self.odom_stamp).to_sec() > ODOM_STALE_S)

            if self.tripped:
                desired = self.counter_motion_target()
                self.climb_boost = 0.0
            elif stale:
                desired = 0.0
                self.climb_boost = 0.0
            else:
                desired = self.raw.linear.x
                # The assist may only SCALE a request to move, never create
                # one. Adding a positive boost to a zero command let this
                # node keep driving through every upstream stop: OBSTACLE,
                # TILT_LIMIT, OFF_BAND, LOCALIZATION_* and MANUAL_MODE all
                # arrive here as linear.x == 0, and a boost integrated on a
                # slope then carried the chair on at up to CLIMB_BOOST_MAX,
                # bleeding off only at CLIMB_DECAY_PER_S - roughly 0.5 m of
                # travel after a commanded full stop, with no layer able to
                # override it.
                if abs(desired) <= 0.05:
                    self.climb_boost = 0.0
                # climb-assist feedback: track measured ground speed
                on_slope = abs(self.fused_pitch) > CLIMB_MIN_PITCH_RAD
                if desired > 0.05 and on_slope:
                    error = desired - self.measured_speed
                    if error > CLIMB_DEADBAND_MPS:
                        # growth rate bounded by the tip governor's budget
                        self.climb_boost += min(
                            CLIMB_GAIN * error, self.accel_budget) * dt
                    elif error < -CLIMB_DEADBAND_MPS:
                        self.climb_boost -= CLIMB_GAIN * (-error) * dt
                    else:
                        self.climb_boost -= CLIMB_DECAY_PER_S * dt * \
                            (1 if self.climb_boost > 0 else -1) * 0.3
                    self.climb_boost = max(-CLIMB_BRAKE_MAX,
                                           min(CLIMB_BOOST_MAX,
                                               self.climb_boost))
                else:
                    # off-slope: bleed any leftover boost quickly
                    self.climb_boost -= CLIMB_DECAY_PER_S * dt * \
                        (1 if self.climb_boost > 0 else -1)
                    if abs(self.climb_boost) < 0.02:
                        self.climb_boost = 0.0
                desired = max(0.0, desired + self.climb_boost)
            if desired > self.current_speed:
                budget = self.accel_budget
                if self.measured_speed < LAUNCH_SPEED_MPS:
                    budget = min(budget, LAUNCH_ACCEL)  # soft launch
                step = min(desired - self.current_speed, budget * dt)
            else:
                decel = HARD_DECEL if self.tripped else (2.0 * HARD_DECEL)
                step = max(desired - self.current_speed, -decel * dt)
            self.current_speed += step

            out = Twist()
            # climb assist is already folded into `desired` above, so it
            # passes through the accel budget with everything else -
            # nothing may be added to the output past the rate limiter.
            out.linear.x = max(0.0, min(ABSOLUTE_V_LIMIT,
                                        self.current_speed))
            self.current_speed = out.linear.x
            out.angular.z = 0.0 if (self.tripped or stale) else self.raw.angular.z
            self.pub.publish(out)

            state = "TRIPPED" if self.tripped else (
                "STALE" if stale else (
                    "CONFIG_UNVERIFIED" if not self.axis_config_ok else "OK"))
            self.status_pub.publish(String(
                data="%s pitch=%.1f dev=%.1f rate=%.1f budget=%.2f "
                     "v=%.2f boost=%.2f" % (
                    state, math.degrees(self.fused_pitch),
                    math.degrees(self.deviation()),
                    math.degrees(self.pitch_rate), self.accel_budget,
                    self.measured_speed, self.climb_boost)))
            if state != self.status:
                rospy.loginfo("tip_guard: %s", state)
                self.status = state
            rate.sleep()


if __name__ == "__main__":
    TipGuard().spin()
