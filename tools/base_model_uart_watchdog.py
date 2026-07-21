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
)

WATCHDOG_TIMEOUT_S = 0.6


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
    self.wheel_data.append(ord(ser.read()) & 0xFF)
    if self.wheel_data[0] == 72:
      if self.wheel_data[-2:] == [13,10]:
        if self.wheel_data[-3] == self.Checksum(self.wheel_data[1:-3]):
          uart_msg = Int16MultiArray()
          uart_msg.data = self.wheel_data
          self.uart_pub.publish(uart_msg)
          self.mode = self.wheel_data[1]
          self.wheel_data = []
        else:
          self.wheel_data = []
      else:
        pass
    else:
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
