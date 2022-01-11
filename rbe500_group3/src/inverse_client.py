#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *

def inverse_kine_client(pos):
    rospy.wait_for_service('inverse_kine')
    try:
        inverse_kine = rospy.ServiceProxy('inverse_kine',InverseKinematics)
        resp1 = inverse_kine(pos)
        return resp1.joints
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":
    xp = float(sys.argv[1])
    yp = float(sys.argv[2])
    zp = float(sys.argv[3])
    xo = float(sys.argv[4])
    yo = float(sys.argv[5])
    zo = float(sys.argv[6])
    wo = float(sys.argv[7])
    pose = Pose()
    pos = Point(xp,yp,zp)
    ori = Quaternion(xo,yo,zo,wo)
    pose.position = pos
    pose.orientation = ori
    print(pose)
    result = inverse_kine_client(pose)
    print(result)
    rospy.loginfo(result)
