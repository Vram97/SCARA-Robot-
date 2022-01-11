#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *
from gazebo_msgs.msg import *
import csv

def callback(LinkStates):
    data = LinkStates
    ground = data.pose[0]
    l1a = data.pose[1]
    l3a = data.pose[2]
    l4a = data.pose[3]

    l4 = l4a.position

    x = l4.x
    y = l4.y
    z = l4.z

    endpos = [x,y,z]

    with open('rbe500-ros/src/rbe500_group/data/Link_State_Data.csv','a') as csvfile:
        links = csv.writer(csvfile)
        links.writerow([x,y,z])
        rospy.loginfo(endpos)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


    
