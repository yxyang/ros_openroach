#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose, Vector3
import numpy as np
import time
import tf

x_data, y_data = [], []
vl, vr = 0, 0
i_l, i_r = 0, 0
kp_r, kp_theta = 12, 0.5

def calibration_callback(msg):
	x_data.append(msg.position.x)
	y_data.append(msg.position.z)

def threshold(val, lb, ub):
        if val < lb:
                return lb
        if val > ub:
                return ub
        return val

def control_callback(msg):
	global vl, vr, i_l, i_r
	diff_r = np.sqrt(msg.orientation.x**2 + msg.orientation.y ** 2)
	
	quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	actual_yaw = euler[2] 
	desired_yaw = np.arctan2(-msg.position.y, -msg.position.x)
	
	diff_theta = desired_yaw - actual_yaw
	#print('actual_yaw:', actual_yaw / np.pi * 180)
	#print('desired_yaw:', desired_yaw / np.pi * 180)
	
	print('diff_theta:', diff_theta / np.pi*180)
	print('diff_r:', diff_r)
	vl = kp_r * diff_r - kp_theta * diff_theta
	vr = kp_r * diff_r + kp_theta * diff_theta
	vl = threshold(vl, 0, 1)
	vr = threshold(vr, 0, 1)
	print(vl, vr)
'''
Calibrates a point by repeatedly reading from OptiTrack and taking average
'''
def calibrate_point():
	global x_data, y_data
	x_data, y_data = [], []
	subscribe_calibration = rospy.Subscriber("/Robot_1/pose", Pose, calibration_callback)
	while len(y_data) <= 10:
		pass
	subscribe_calibration.unregister()
	ans_x, ans_y = np.mean(x_data), np.mean(y_data)
	return ans_x, ans_y

rospy.init_node("treadmill_optitrack", anonymous=True)
'''
# Find center of OpenRoACH
raw_input("Put OpenRoACH on treadmill center, then press any key to start calibration")

center_x, center_y = calibrate_point();
print("Center found at {:.3f}, {:.3f}".format(center_x, center_y))

# Find Track Orientation
print("Now let's find the orientation of treadmill by finding 2 points on the treadmill")
print("First, put OpenRoACH at an 'earlier' position on the treadmill")
raw_input("Press Any Key to Continue...")
start_x, start_y = calibrate_point();
print("Starting point found at {:.3f}, {:.3f}".format(start_x, start_y))

print("Now, place OpenRoACH at a 'later' position in the treadmill. That is, assume OpenRoACH is static w.r.t. treadmill while treadmill is moving, where would it be now?")
raw_input("Press Any Key to Continue...")
end_x, end_y = calibrate_point();
print("Ending point found at {:.3f}, {:.3f}".format(end_x, end_y))

'''
print("Now let's see how the control goes!")
rospy.Subscriber("/Robot_1/pose", Pose, control_callback)

control_pub = rospy.Publisher('/cmd_pwm', Vector3, queue_size = 1)

x = 'a'
while not rospy.is_shutdown():
	control_pub.publish(Vector3(vl, vr, 0))
	time.sleep(0.1);



