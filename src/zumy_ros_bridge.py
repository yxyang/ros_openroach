#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
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
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback, queue_size=1)
    self.lock = Condition()
    self.rate = rospy.Rate(30.0)
    self.name = socket.gethostname()
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 5)
    self.camera_pub = rospy.Publisher('camera', Image, queue_size = 5)
    self.imu_count = 0

  def cmd_callback(self, msg):
    lv = 10
    la = 12
    v = msg.linear.x
    a = msg.angular.z
    r = lv*v + la*a
    l = lv*v - la*a
    l, r = max(l, 0), max(r, 0)
    print((l, r))
    self.lock.acquire()
    self.cmd = (l,r)
    self.lock.release()

  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)
      imu_data = self.zumy.read_imu()
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

      image = Image()
      image.header.stamp = rospy.Time.now()
      image.height = 1
      image.width = 128
      #image.is_bigendian = True
      image.step = 128
      image.encoding = "mono8"
      raw_data = self.zumy.read_camera()
      min_raw, max_raw = np.min(raw_data), np.max(raw_data)      
      image.data = [x // 256 for x in raw_data]
      self.camera_pub.publish(image)

      self.rate.sleep()

    # If shutdown, turn off motors
    self.zumy.cmd(0,0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
