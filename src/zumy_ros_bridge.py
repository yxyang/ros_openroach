#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from threading import Condition
from zumy import Zumy
from std_msgs import String

import socket

class ZumyROS:	
  def __init__(self):
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
    self.lock = Condition()
    self.rate = rospy.Rate(30.0)
    self.zumy = Zumy()
    self.name = socket.gethostname()
    self.heartBeat = rospy.Publisher('/' + self.name + '/heartBeat', String, queue_size=5)

  def cmd_callback(self, msg):
    lv = 0.6
    la = 0.4
    v = msg.linear.x
    a = msg.angular.z
    r = lv*v + la*a
    l = lv*v - la*a
    self.lock.acquire()
    self.cmd = (l,r)
    self.lock.release()

  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)
      self.hearBeat.publish("I am alive")
      self.lock.release()
      self.rate.sleep()
    self.zumy.cmd(0,0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
