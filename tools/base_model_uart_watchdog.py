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

import threading

import serial
import rospy
from std_msgs.msg import Int16MultiArray, Int16

ser = serial.Serial(
  port     = '/dev/uart',
  baudrate = 115200,
  bytesize = serial.EIGHTBITS,
  parity   = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
  # Without a timeout, ser.read() blocks forever and rospy.is_shutdown()
  # is never re-checked, so the node cannot exit cleanly - and ord() on an
  # empty read would raise. RX() treats a timeout as "no byte yet".
  timeout  = 0.1,
)

WATCHDOG_TIMEOUT_S = 0.6
AUTO_MODE = 65
MANUAL_MODE = 77
# [72, mode, payload..., checksum, 13, 10] - the shortest frame that can be
# parsed without reading a position that does not exist
MIN_FRAME_LEN = 6
MAX_FRAME_LEN = 64


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
    self.stop_cmd = [83,33,83,33,79]
    self.mode = None
    self.tx_lock = threading.Lock()
    self.last_cmd_time = rospy.Time.now()
    rospy.Timer(rospy.Duration(0.25), self.WatchdogTick)

  def Checksum(self, ckdata):
    return (~(sum(ckdata))+1) & 0xFF

  def RX(self):
    byte = ser.read()
    if not byte:
      return                      # read timeout, not a frame boundary
    self.wheel_data.append(ord(byte) & 0xFF)
    if len(self.wheel_data) > MAX_FRAME_LEN:
      # a frame that never terminates would otherwise grow for the life of
      # the process; resync instead of buffering line noise forever
      rospy.logwarn_throttle(5.0, 'uart: oversized frame, resyncing')
      self.wheel_data = []
      return
    if self.wheel_data[0] != 72:
      self.wheel_data = []
      return
    if self.wheel_data[-2:] != [13,10]:
      return
    # A frame must be long enough for [72, mode, ..., checksum, 13, 10]
    # before any of those positions may be read. Without this a 4-byte
    # corruption makes wheel_data[1:-3] empty, so the checksum of nothing
    # can match by chance and wheel_data[1] - actually the checksum byte -
    # is then latched as the drive mode. A value of 65 there authorises
    # motion with no auto-mode confirmation behind it.
    if len(self.wheel_data) < MIN_FRAME_LEN:
      rospy.logwarn_throttle(5.0, 'uart: short frame (%d bytes), discarding',
                             len(self.wheel_data))
      self.wheel_data = []
      return
    if self.wheel_data[-3] != self.Checksum(self.wheel_data[1:-3]):
      self.wheel_data = []
      return
    mode = self.wheel_data[1]
    uart_msg = Int16MultiArray()
    uart_msg.data = self.wheel_data
    self.uart_pub.publish(uart_msg)
    # Only the two defined modes may be latched. Anything else leaves the
    # previous mode alone rather than becoming a third, unhandled state.
    if mode in (AUTO_MODE, MANUAL_MODE):
      self.mode = mode
    else:
      rospy.logwarn_throttle(5.0, 'uart: unknown mode byte %d ignored', mode)
    self.wheel_data = []

  def TX(self, wheel_cmd):
    cmd_data = [72] + wheel_cmd + [self.Checksum(wheel_cmd),13,10]
    with self.tx_lock:
      for i in range(len(cmd_data)):
        ser.write(chr(cmd_data[i]).encode())

  def CmdCallback(self, msg):
    if self.mode == 65:
      self.last_cmd_time = rospy.Time.now()
      wheel_cmd = [self.mode] + list(msg.data)
      self.TX(wheel_cmd)
    else:
      pass

  def WatchdogTick(self, _event):
    if self.mode != 65:
      return
    if (rospy.Time.now() - self.last_cmd_time).to_sec() > WATCHDOG_TIMEOUT_S:
      rospy.logwarn_throttle(2.0, 'uart watchdog: wheel_cmd starved, sending stop')
      self.TX([65] + self.stop_cmd)

  def ModeCallback(self, msg):
    if msg.data not in (AUTO_MODE, MANUAL_MODE):
      rospy.logwarn('uart: ignoring unknown mode command %d', msg.data)
      return
    self.mode = msg.data
    if self.mode == 65:
      print('\n\n[[[ Auto Mode ]]]')
      self.last_cmd_time = rospy.Time.now()
      self.TX([self.mode] + self.stop_cmd)
    elif self.mode == 77:
      print('\n\n[[[ Manual Mode ]]]')
      self.TX([self.mode] + self.stop_cmd)
    else:
      pass


def stop_motors(uart):
  """Best-effort stop frame. Called on EVERY exit path.

  The whole point of this file is that the motors stop when the command
  stream dies - but only KeyboardInterrupt used to be caught, so a
  SerialException from a USB re-enumeration or a jostled cable fell
  straight through to `finally`, closed the port and exited WITHOUT ever
  sending a stop. The last commanded speed stays latched in the motor
  controller and the watchdog timer dies with the process: exactly the
  failure this node exists to prevent, one level down.
  """
  for mode in (AUTO_MODE, MANUAL_MODE):
    try:
      uart.TX([mode] + uart.stop_cmd)
    except Exception as exc:                       # noqa: BLE001
      rospy.logerr('uart: could not send stop frame for mode %d: %s',
                   mode, exc)


if __name__=="__main__":
  uart = UARTCommunication()
  try:
    while not rospy.is_shutdown():
      uart.RX()

  except KeyboardInterrupt:
    print('keyboard interrupt')

  except Exception as exc:                         # noqa: BLE001
    # log loudly: this path means the link died mid-drive
    rospy.logfatal('uart: link failed (%s) - stopping motors and exiting',
                   exc)

  finally:
    stop_motors(uart)
    try:
      ser.close()
    except Exception:                              # noqa: BLE001
      pass
