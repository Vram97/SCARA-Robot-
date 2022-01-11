#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *
import time
import csv

def callback(data):
    q1 = data.position[0]
    q2 = data.position[1]
    d3 = data.position[2]
    q1d = data.velocity[0]
    q2d = data.velocity[1]
    d3d = data.velocity[2]
    seq = data.header.seq
    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs
    with open('rbe500-ros/src/rbe500_group/data/Joint_State_Data.csv','a') as csvfile:
        joints = csv.writer(csvfile)
        joints.writerow([seq,secs,nsecs,q1,q2,d3,q1d,q2d,d3d])
        rospy.loginfo([q1,q2,d3,q1d,q2d,d3d])
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()