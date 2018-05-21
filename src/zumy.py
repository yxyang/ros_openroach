# -*- coding: utf-8 -*-
"""
Created on Wed Oct 29 06:39:30 2014
@author: ajc
"""

from mbedrpc import *
import threading
import time
from serial import SerialException
import pigpio

imu_names = ['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z']
l_enc_pin = 23 # Use Raspberry PI pin 23 for left encoder interrupts
r_enc_pin = 24 # Use Raspberry PI pin 24 for right encoder interrupts

class Zumy:
    def __init__(self, dev='/dev/ttyACM0'):
        self.mbed=SerialRPC(dev, 115200)
        self.m_left = RPCVariable(self.mbed, 'duty_cycle_left')
        self.m_right = RPCVariable(self.mbed, 'duty_cycle_right')
        self.an = AnalogIn(self.mbed, 'p15')
        self.imu_vars = [RPCVariable(self.mbed,name) for name in imu_names]
        self.camera = RPCFunction(self.mbed, 'readCamera')
        self.rlock=threading.Lock()

        self.pi = pigpio.pi()
        self.l_enc = self.pi.callback(11)
        self.r_enc = self.pi.callback(8)

    def cmd(self, left, right):
        self.rlock.acquire()
        # As of Rev. F, positive command is sent to both left and right
        try:
            self.m_left.write(left)
            self.m_right.write(right)
            pass
        except SerialException:
          pass
        self.rlock.release()

    def read_imu(self):
      self.rlock.acquire()
      try:
	rval = [float(var.read()) for var in self.imu_vars]
      except SerialException:
        pass
      self.rlock.release()
      return rval
    
    def read_enc(self):
      self.rlock.acquire()
      rval = [self.l_enc.tally(), self.r_enc.tally()]
      self.rlock.release()
      return rval

    def read_camera(self):
        ans = [0 for _ in range(128)]
        for i in range(128):
            res = self.camera.run(str(i))
            ans[i]= int(res.split(' ')[0])
        return ans

if __name__ == '__main__':
    z=Zumy()
 
