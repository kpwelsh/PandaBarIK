#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np


def print_msg(msg):
    print(msg)

nh = rospy.init_node("ik_test")
pub = rospy.Publisher("in", Pose, queue_size=10)
sub = rospy.Subscriber("out", Float64MultiArray, print_msg)


rospy.sleep(1)

t = 0
while True:
    t += 5 * 1e-4
    x = np.cos(t) / 6 + 0.6
    y = np.sin(t) / 6
    z = 0.15

    msg = Pose()
    msg.position = Point(x = x, y = y, z = z)
    msg.orientation = Quaternion(w = 1, x = 0, y = 0, z = 0)

    pub.publish(msg)
    rospy.sleep(0.001)

