#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Int32
import numpy as np
import time
import tf

# Speed/phase defined in terms of revs per second
desired_phi_l, desired_phi_r, desired_w_l, desired_w_r = 0, 0, 0, 0
ei_l, ei_r, old_ep_l, old_ep_r = 0, 0, 0, 0
old_l_count, old_r_count, l_count, r_count = 0, 0, 0, 0
ticks_per_rev = 3*298
kp, ki, kd = 0.6, 0.00, 0.2


def threshold(val, lb, ub):
        if val < lb:
                return lb
        if val > ub:
                return ub
        return val

def l_enc_callback(msg):
        global l_count
        l_count = msg.data

def r_enc_callback(msg):
        global r_count
        r_count = msg.data

def update_vel(time):
        #print(l_count)
        global desired_phi_l, desired_phi_r, old_l_count, old_r_count
        # Compute Speed
        actual_w_l = abs(l_count-old_l_count) * 1.0 / ticks_per_rev
        actual_w_r = abs(r_count-old_r_count) * 1.0 / ticks_per_rev
	old_l_count, old_r_count = l_count, r_count

        # Compute left/right errors
        global ei_l, ei_r, old_ep_l, old_ep_r
        ep_l, ep_r = desired_w_l - actual_w_l, desired_w_r - actual_w_r
        ei_l = ei_l + ep_l
        ei_r = ei_r + ep_r
        ed_l, ed_r = ep_l - old_ep_l, ep_r - old_ep_r
        old_ep_l, old_ep_r = ep_l, ep_r
        
        # PID control of speed
        pwm_l = kp * ep_l + ki * ei_l + kd * ed_l
        pwm_r = kp * ep_r + ki * ei_r + kd * ed_r
        pwm_l = threshold(pwm_l, 0, 1)
        pwm_r = threshold(pwm_r, 0, 1)
        control_pub.publish(Vector3(pwm_r, pwm_l, 0))

kp_r, kp_theta = 8,1 
def control_callback(msg):
	global desired_w_l, desired_w_r
	diff_r = np.sqrt(msg.position.x**2 + msg.position.y ** 2)

        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        actual_yaw = euler[2]
        desired_yaw = np.arctan2(-msg.position.y, -msg.position.x)

        diff_theta = desired_yaw - actual_yaw
	
	desired_w_l = kp_r * diff_r + kp_theta * diff_theta
	desired_w_r = kp_r * diff_r - kp_theta * diff_theta

	if msg.position.y > 0:
		desired_w_l, desired_w_r = 0, 0
	
	print('diff_theta:', diff_theta / np.pi*180)
        print('diff_r:', diff_r)
	print(desired_w_l, desired_w_r)

rospy.init_node("phase_control", anonymous=True)
rospy.Subscriber("/l_enc", Int32, l_enc_callback)
rospy.Subscriber("/r_enc", Int32, r_enc_callback)
rospy.Subscriber("/Robot_1/pose", Pose, control_callback)
control_pub = rospy.Publisher('/cmd_pwm', Vector3, queue_size = 1)

dt = 0.1
#desired_w_l = 1;
#desired_w_r = 1;
rospy.Timer(rospy.Duration(dt), update_vel)
rospy.spin()
