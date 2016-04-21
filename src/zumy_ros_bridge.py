#!/usr/bin/python

from zumy import *

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Header,Int32,Float32
from sensor_msgs.msg import Imu

import socket

ROBOT_RADIUS = 0.02
ROBOT_BASE = 0.1

class ZumyROS:
  def __init__(self):
    self.zumy = Zumy()
    
    rospy.init_node('zumy_ros')
    self.rate = rospy.Rate(100.0)

    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback, queue_size=1)
    
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 5)
    
    self.r_enc_pub = rospy.Publisher('r_enc', Int32, queue_size = 5)
    self.r_vel_pub = rospy.Publisher('r_vel', Float32, queue_size = 5)
    self.l_enc_pub = rospy.Publisher('l_enc', Int32, queue_size = 5)
    self.l_vel_pub = rospy.Publisher('l_vel', Float32, queue_size = 5)
    
    self.volt_pub = rospy.Publisher('voltage', Float32, queue_size = 1)
    
    self.imu_count = 0

    self.name = socket.gethostname()

    self.last_enc_time = None
    self.last_r_enc = None
    self.last_l_enc = None
    self.first_sensor_time = None

  def cmd_callback(self, msg):
    v = msg.linear.x
    a = msg.angular.z
    l = (v - a*ROBOT_BASE/2) / ROBOT_RADIUS
    r = (v + a*ROBOT_BASE/2) / ROBOT_RADIUS
    self.zumy.cmd(l,r)

  def run(self):
    while not rospy.is_shutdown():
      sensor_packet = self.zumy.read_sensors()
        

      if sensor_packet is not None:
        if sensor_packet['type'] is 'S':
          
          if self.first_sensor_time is None:
            self.first_sensor_time = (sensor_packet['time'],rospy.Time.now())

          time_delta =  rospy.Duration((sensor_packet['time']-self.first_sensor_time[0]) / 1000000.0)
          sensor_time = self.first_sensor_time[1] + time_delta

          imu_msg = Imu()
          imu_msg.header = Header(self.imu_count, sensor_time, self.name)
          self.imu_count += 1
          imu_msg.linear_acceleration.x = 9.81 * sensor_packet['ax']
          imu_msg.linear_acceleration.y = 9.81 * sensor_packet['ay']
          imu_msg.linear_acceleration.z = 9.81 * sensor_packet['az']
          imu_msg.angular_velocity.x = 3.14 / 180.0 * sensor_packet['gx']
          imu_msg.angular_velocity.y = 3.14 / 180.0 * sensor_packet['gy']
          imu_msg.angular_velocity.z = 3.14 / 180.0 * sensor_packet['gz']
          self.imu_pub.publish(imu_msg)

          r_enc, l_enc = sensor_packet['er'], sensor_packet['el']

          enc_msg = Int32()
          enc_msg.data = r_enc
          self.r_enc_pub.publish(enc_msg)
          
          enc_msg = Int32()
          enc_msg.data = l_enc
          self.l_enc_pub.publish(enc_msg)

          vel_msg = Float32()
          vel_msg.data = sensor_packet['vl']
          self.r_vel_pub.publish(vel_msg)
          
          vel_msg = Float32()
          vel_msg.data = sensor_packet['vr']
          self.l_vel_pub.publish(vel_msg)

          volt_msg = Float32()
          volt_msg.data = sensor_packet['voltage'] * 3.3 * 3.0
          self.volt_pub.publish(volt_msg)

      self.rate.sleep()

    # Command Zumy to stop, dump any packets left in the queue
    self.zumy.cmd(0.0,0.0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
