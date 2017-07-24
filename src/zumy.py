#!/usr/bin/python

from packet_parser import *

class Zumy():
  def __init__(self):
    self.parser = PacketParser()
    self.parser.start()

  def reset(self):
    print 'Resetting mbed...'
    self.parser.put(get_reset_packet())
    reset_packet = self.parser.get(True,5.0)
    if reset_packet is not None:
      print 'Got reset packet.'
    else:
      print 'Didn\'t get reset packet.'
    
  def cmd(self, left=0.0, right=0.0):
    packet = get_command_packet(left,right)
    self.parser.put(packet)
  
  def read_sensors(self):
    self.parser.flush()
    self.parser.put(get_read_packet())
    return self.parser.get(True,0.01)

  def read_voltage(self):
    return self.read_sensors()['voltage'] * 3.3 * 3.0

  def read_current(self):
    return self.read_sensors()['current'] * 3.3 * 0.6

  def read_enc(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['el','er']]

  def read_imu(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['ax','ay','az','gx','gy','gz']]

  def read_vel(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['vl','vr']]

  def set_markers(self, colors = None):
    packet = get_marker_packet(colors)
    self.parser.put(packet)

  def set_laser_galvo(self, laser_cmd = 0.0, galvo_cmd_0 = 0, galvo_cmd_1 = 0):
    packet = get_laser_packet(laser_cmd, galvo_cmd_0, galvo_cmd_1)
    self.parser.put(packet)

if __name__ == '__main__':
  z = Zumy()
  z.reset()

