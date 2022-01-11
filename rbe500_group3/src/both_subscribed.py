#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *
import csv

def callback(JointState,LinkStates):
    data1 = JointState
    q1 = data1.position[0]
    q2 = data1.position[1]
    d3 = data1.position[2]
    q1d = data1.velocity[0]
    q2d = data1.velocity[1]
    d3d = data1.velocity[2]
    seq = data1.header.seq
    secs = data1.header.stamp.secs
    nsecs = data1.header.stamp.nsecs

    time = [seq,secs,nsecs]
    jpos = [q1,q2,d3]
    jvel = [q1d,q2d,d3d]
    
    vals =  np.concatenate((time,jpos,jvel),axis=None)
    
    data2 = LinkStates
    ground = data2.pose[0]
    l1a = data2.pose[1]
    l3a = data2.pose[2]
    l4a = data2.pose[3]

    l4 = l4a.position

    x = l4.x
    y = l4.y
    z = l4.z

    endpos = [x,y,z]
    
    states = np.concatenate((vals,endpos),axis=None)
    rospy.loginfo(states)
    with open('rbe500-ros/src/rbe500_group/data/Joint_State_Data.csv','a') as csvfile:
        joints = csv.writer(csvfile)
        joints.writerow(states)
        rospy.loginfo(states)
    
def listener():
    rospy.init_node('write_states',anonymous=True)
    j_mes = message_filters.Subscriber('/robot/joint_states', JointState)
    end_mes = message_filters.Subscriber('/gazebo/link_states', LinkStates)

    ts = message_filters.TimeSynchronizer([j_mes, end_mes], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    listener()