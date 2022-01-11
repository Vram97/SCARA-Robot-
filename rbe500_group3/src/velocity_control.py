#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *

def velocity():
    pub1 = rospy.Publisher('/robot/joint0_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/robot/joint3_velocity_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo('publishing...')
    
    while not rospy.is_shutdown():
        q1 = -0.04
        q2 = -0.1
        q3 = 0.0
        q1 = 0.0
        q2 = 0.0
        q3 = 0.0
        pub1.publish(q1)
        pub2.publish(q2)
        pub3.publish(q3)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity()
    except rospy.ROSInterruptException:
        pass
