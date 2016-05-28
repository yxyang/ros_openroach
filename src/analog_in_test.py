#!/usr/bin/python

import rospy
from mbedrpc import *
from std_msgs.msg import Float32
import sys

if __name__ == '__main__':
  # Initialize mbed RPC interface and AnalogIn peripheral
  mbed = SerialRPC('/dev/ttyACM0',115200)

  if len(sys.argv) > 1:
    pin_name = sys.argv[1]
  else:
    pin_name = 'p19'

  analog_in = AnalogIn(mbed, pin_name)

  # Create a message object to be sent later
  volt_msg = Float32()

  # Initialize this as a ROS node
  rospy.init_node('analog_in_test')

  # Create a loop rate timer for 10 Hz
  rate_timer = rospy.Rate(10)

  # Create a message publisher with the topic 'voltage' of type Float32
  publisher = rospy.Publisher('voltage', Float32, queue_size = 5)

  while not rospy.is_shutdown():
    # Set data field of message to analog in value, scale by VDD
    volt_msg.data = 3.3 * analog_in.read()

    # Publish the message
    publisher.publish(volt_msg)

    # Sleep until ready to send the next message
    rate_timer.sleep()
