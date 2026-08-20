#!/usr/bin/env python3
"""base_model uart.py with an auto-mode command watchdog.

Drop-in replacement for catkin_ws/src/base_model/src/uart.py. Changes vs
the original:
  - watchdog: in auto mode (65), if no wheel_cmd arrives for 0.6 s the node
    sends the stop frame itself (repeats at 4 Hz while starved) so a dead
    planner or gate can never leave the last speed latched in the motors
  - self.mode initialized (the original referenced it before assignment
    when a command arrived ahead of the first status frame)
  - serial writes serialized with a lock (TX now happens from two threads)
  - per-frame console debug prints removed (10 Hz spam)
Manual mode (77) behaviour is unchanged.
"""

import json
import threading
import time

import serial
import rospy
from std_msgs.msg import Int16MultiArray, Int16, String

ser = serial.Serial(
  port     = '/dev/uart',
  baudrate = 115200,
  bytesize = serial.EIGHTBITS,
  parity   = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
  # Both timeouts are load-bearing. Without write_timeout a blocked port makes
  # ser.write() wait forever inside the subscriber thread while the base holds
  # its last speed - the shape of the stop that went unhonoured for 13 s on
  # 2026-08-19. Without a read timeout the RX loop cannot notice shutdown.
  # A frame is 10 bytes at 115200 baud, so 50 ms is ~60x an unblocked write.
  timeout       = 0.1,
  write_timeout = 0.05,
)

# Not yet a failure, but the leading edge of one, and the black box should
# carry it.
SLOW_WRITE_S = 0.02

WATCHDOG_TIMEOUT_S = 0.6
EXPECTED_WHEEL_COMMAND_CALLER = "/wheel_cmd"
STOP_COMMAND = [83, 33, 83, 33, 79]


def message_caller_id(message):
  header = getattr(message, "_connection_header", None)
  if not isinstance(header, dict):
    return ""
  return str(header.get("callerid", "")).strip()


def valid_wheel_command(values):
  if len(values) != 5:
    return False
  return values[0] in (67, 83, 87) and \
    values[2] in (67, 83, 87) and \
    33 <= values[1] <= 127 and \
    33 <= values[3] <= 127 and \
    values[4] == 79


class UARTCommunication():
  def __init__(self):
    rospy.init_node('uart')
    # Pub
    self.uart_pub = rospy.Publisher('wheel_status', Int16MultiArray, queue_size=1)
    # Sub
    rospy.Subscriber('wheel_cmd', Int16MultiArray, self.CmdCallback)
    rospy.Subscriber('mode_cmd', Int16, self.ModeCallback)
    # Param
    self.wheel_data = []
    self.stop_cmd = list(STOP_COMMAND)
    self.mode = None
    self.command_fault_latched = False
    self.tx_lock = threading.Lock()
    self.last_cmd_monotonic = time.monotonic()
    # TX accounting. Without it a stop the wheels ignored cannot be told from
    # a stop that never reached the wire, which is why 2026-08-19 stayed open.
    self.tx_diag_pub = rospy.Publisher("uart_tx_diag", String, queue_size=1)
    self.tx_ok = 0
    self.tx_fail = 0
    self.tx_slow = 0
    self.last_write_s = 0.0
    self.last_tx_monotonic = 0.0
    self.last_frame = []
    rospy.Timer(rospy.Duration(0.25), self.WatchdogTick)
    rospy.Timer(rospy.Duration(0.2), self.PublishTxDiag)

  def Checksum(self, ckdata):
    return (~(sum(ckdata))+1) & 0xFF

  def RX(self):
    chunk = ser.read(1)
    if not chunk:
      return
    self.wheel_data.append(chunk[0] & 0xFF)
    if self.wheel_data[0] == 72:
      if self.wheel_data[-2:] == [13,10]:
        if self.wheel_data[-3] == self.Checksum(self.wheel_data[1:-3]):
          uart_msg = Int16MultiArray()
          uart_msg.data = self.wheel_data
          self.uart_pub.publish(uart_msg)
          self.UpdateMode(self.wheel_data[1])
          self.wheel_data = []
        else:
          self.wheel_data = []
      else:
        pass
    else:
      self.wheel_data = []

  def TX(self, wheel_cmd):
    # One write, not ten. Ten leaves nine points at which the frame can tear;
    # the base checksums it away, but the cycle it costs is the one carrying
    # a stop.
    cmd_data = [72] + wheel_cmd + [self.Checksum(wheel_cmd),13,10]
    frame = bytes(cmd_data)
    started = time.monotonic()
    with self.tx_lock:
      try:
        ser.write(frame)
      except serial.SerialTimeoutException:
        self.tx_fail += 1
        try:
          ser.reset_output_buffer()
        except Exception:
          pass
        rospy.logwarn_throttle(
          1.0,
          "uart TX timeout after %.0f ms - port blocked, dropped %s"
          % (1000.0 * (time.monotonic() - started), cmd_data))
        return False
      except Exception as exc:
        self.tx_fail += 1
        rospy.logwarn_throttle(1.0, "uart TX error: %s" % (exc,))
        return False
    elapsed = time.monotonic() - started
    self.last_write_s = elapsed
    self.last_tx_monotonic = time.monotonic()
    self.last_frame = cmd_data
    self.tx_ok += 1
    if elapsed > SLOW_WRITE_S:
      self.tx_slow += 1
      rospy.logwarn_throttle(
        1.0, "uart TX slow: %.0f ms for one frame" % (1000.0 * elapsed,))
    return True

  def PublishTxDiag(self, _event):
    since = -1.0
    if self.last_tx_monotonic:
      since = round(time.monotonic() - self.last_tx_monotonic, 3)
    self.tx_diag_pub.publish(String(data=json.dumps({
      "tx_ok": self.tx_ok,
      "tx_fail": self.tx_fail,
      "tx_slow": self.tx_slow,
      "last_write_ms": round(1000.0 * self.last_write_s, 2),
      "since_last_tx_s": since,
      "last_frame": self.last_frame,
      "mode": self.mode,
    })))

  def CmdCallback(self, msg):
    values = list(msg.data)
    if message_caller_id(msg) != EXPECTED_WHEEL_COMMAND_CALLER or \
        not valid_wheel_command(values):
      self.command_fault_latched = True
    if self.command_fault_latched:
      if self.mode == 65:
        self.TX([65] + self.stop_cmd)
      return
    if self.mode == 65:
      self.last_cmd_monotonic = time.monotonic()
      wheel_cmd = [self.mode] + values
      self.TX(wheel_cmd)
    else:
      pass

  def WatchdogTick(self, _event):
    if self.mode != 65:
      return
    if time.monotonic() - self.last_cmd_monotonic > WATCHDOG_TIMEOUT_S:
      rospy.logwarn_throttle(2.0, 'uart watchdog: wheel_cmd starved, sending stop')
      self.TX([65] + self.stop_cmd)

  def UpdateMode(self, next_mode):
    previous_mode = self.mode
    self.mode = next_mode
    if next_mode != 65 or previous_mode != 65:
      self.command_fault_latched = False

  def ModeCallback(self, msg):
    self.UpdateMode(msg.data)
    if self.mode == 65:
      print('\n\n[[[ Auto Mode ]]]')
      self.last_cmd_monotonic = time.monotonic()
      self.TX([self.mode] + self.stop_cmd)
    elif self.mode == 77:
      print('\n\n[[[ Manual Mode ]]]')
      self.TX([self.mode] + self.stop_cmd)
    else:
      pass


if __name__=="__main__":
  uart = UARTCommunication()
  try:
    while not rospy.is_shutdown():
      uart.RX()

  except KeyboardInterrupt:
    print('keyboard interrupt')

  finally:
    ser.close()
    pass
