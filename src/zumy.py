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

  def read_enc(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['el','er']]

  def read_imu(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['ax','ay','az','gx','gy','gz']]

  def read_vel(self):
    sensors = self.read_sensors()
    return [sensors[k] for k in ['vl','vr']]

if __name__ == '__main__':
  z = Zumy()
  z.reset()

