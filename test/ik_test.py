#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState


def print_msg(msg):
    print(msg.position)

nh = rospy.init_node("ik_test")
pub = rospy.Publisher("in", Pose, queue_size=10)
#sub = rospy.Subscriber("/franka_ros_interface/custom_franka_state_controller/joint_states_desired", JointState, print_msg)


rospy.sleep(1)

t = 0
while True:
    t += 80 * 1e-4
    x = 0.6 #np.cos(t) / 6 + 0.6
    y = np.sin(t) / 3
    z = 0.25

    msg = Pose()
    msg.position = Point(x = x, y = y, z = z)
    msg.orientation = Quaternion(w = 1, x = 0, y = 0, z = 0)

    pub.publish(msg)
    rospy.sleep(0.001)

