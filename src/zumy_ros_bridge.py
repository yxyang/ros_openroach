#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist, Vector3
from threading import Condition
import numpy as np
from zumy import Zumy
from std_msgs.msg import String,Header,Int32,Float32 
from sensor_msgs.msg import Imu, Image

import socket,time

class ZumyROS:  
  def __init__(self):
    self.zumy = Zumy()
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('keyboard', Twist, self.keyboard_callback, queue_size=1)
    rospy.Subscriber('cmd_pwm', Vector3, self.pwm_callback, queue_size=1)
    self.lock = Condition()
    self.rate = rospy.Rate(300.0)
    self.name = socket.gethostname()
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 5)
    self.r_enc_pub = rospy.Publisher('r_enc', Int32, queue_size = 1)
    self.l_enc_pub = rospy.Publisher('l_enc', Int32, queue_size = 1)
    self.camera_pub = rospy.Publisher('camera', Image, queue_size = 5)
    self.imu_count = 0
    self.l_pwm, self.r_pwm = 0, 0

  def pwm_callback(self, msg):
    self.l_pwm, self.r_pwm = msg.x, msg.y
    self.l_pwm = max(self.l_pwm, -1)
    self.l_pwm = min(self.l_pwm, 1)
    self.r_pwm = max(self.r_pwm, -1)
    self.r_pwm = min(self.r_pwm, 1)
    self.lock.acquire()
    self.cmd = (self.l_pwm,self.r_pwm)
    self.lock.release()

  def keyboard_callback(self, msg):
    v = msg.linear.x
    a = msg.angular.z
    if (a == 0):
      if (v > 0):
        self.l_pwm += 0.1
        self.r_pwm += 0.1
      else:
        self.l_pwm -= 0.1
        self.r_pwm -= 0.1
    elif (a > 0):
      self.l_pwm -= 0.1
      self.r_pwm += 0.1
    else:
      self.l_pwm += 0.1
      self.r_pwm -= 0.1

    self.l_pwm = max(self.l_pwm, 0)
    self.l_pwm = min(self.l_pwm, 1)
    self.r_pwm = max(self.r_pwm, 0)
    self.r_pwm = min(self.r_pwm, 1)
    print((self.l_pwm, self.r_pwm))
    self.lock.acquire()
    self.cmd = (self.l_pwm,self.r_pwm)
    self.lock.release()

  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)
      imu_data = self.zumy.read_imu()
      enc_data = self.zumy.read_enc()
      self.lock.release()
      
      imu_msg = Imu()
      imu_msg.header = Header(self.imu_count,rospy.Time.now(),self.name)
      imu_msg.linear_acceleration.x = 9.81 * imu_data[0]
      imu_msg.linear_acceleration.y = 9.81 * imu_data[1]
      imu_msg.linear_acceleration.z = 9.81 * imu_data[2]
      imu_msg.angular_velocity.x = 3.14 / 180.0 * imu_data[3]
      imu_msg.angular_velocity.y = 3.14 / 180.0 * imu_data[4]
      imu_msg.angular_velocity.z = 3.14 / 180.0 * imu_data[5]
      self.imu_pub.publish(imu_msg)

      enc_msg = Int32()
      enc_msg.data = enc_data[0]
      self.r_enc_pub.publish(enc_msg)
      enc_msg = Int32()
      enc_msg.data = enc_data[1]
      self.l_enc_pub.publish(enc_msg)

      '''image = Image()
      image.header.stamp = rospy.Time.now()
      image.height = 1
      image.width = 128
      image.step = 128
      image.encoding = "mono8"
      raw_data = self.zumy.read_camera()
      min_raw, max_raw = np.min(raw_data), np.max(raw_data)      
      image.data = [x // 256 for x in raw_data]
      self.camera_pub.publish(image)'''

      self.rate.sleep()

    # If shutdown, turn off motors
    self.zumy.cmd(0,0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
