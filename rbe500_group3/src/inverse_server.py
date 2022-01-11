#!/usr/bin/env python
import sys

import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *


def handle_inverse_kine(InverseKinematics):
    # Get position and orientation from pose message
    Pose = InverseKinematics.pose
    position = Pose.position
    xp = position.x
    yp = position.y
    zp = position.z

    quat = Pose.orientation
    xq = quat.x
    yq = quat.y
    zq = quat.z
    wq = quat.w

    # Hard coded joint lengths
    L1 = .5
    L2 = .5
    L3 = .6

    # Calculate d3
    d3 = -(L3 - zp)

    # Calculate physical parameters based on geometry
    D = (xp**2 + yp**2 - L1**2 - L2**2)/(2*L1*L2)

    # Calculate q2
    q2 = np.arctan2(np.sqrt(1-D**2),D)

    # Calculate q1
    q1_x = L1 + L2*np.cos(q2)
    q1_y = L2*np.sin(q2)
    q1 = np.arctan2(yp,xp) + np.arctan2(q1_y,q1_x)

    # Prepare message for printing in console
    message = 'Solved for joint values: ' + \
              '\nq1: ' + "{:.5f}".format(q1) + \
              '\nq2: ' + "{:.5f}".format(q2) + \
              '\nd3: ' + "{:.5f}".format(d3)

    # Send message
    rospy.loginfo(message)

    joint_names = ['q1','q2','d3']
    joint_pos = [q1, q2, d3]

    joint_msg = JointState()
    joint_msg.name = joint_names
    joint_msg.position = joint_pos
    return InverseKinematicsResponse(joint_msg)

def inverse_kine_server():
    # Initialize each listener node uniquely
    rospy.init_node('inverse_kine_server', anonymous=True)
    s = rospy.Service('inverse_kine',InverseKinematics,handle_inverse_kine)
    rospy.loginfo("Ready to perform inverse kinematics")
    rospy.spin()

if __name__ == '__main__':
    inverse_kine_server()
