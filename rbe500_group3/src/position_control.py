#!/usr/bin/env python
import sys

import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.srv import *
from controller_manager_msgs.srv import SwitchController




def position_control():
    # Initialize each listener node uniquely
    pub_q0_pos = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=10)
    pub_q2_pos = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
    pub_q3_pos = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
    rospy.init_node('position_control', anonymous=True)
    q1 = 0.84
    q2 = 1.21
    q3 = 0.2
    rate = rospy.Rate(10)
  
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo("publishing...")
        pub_q0_pos.publish(q1)
        pub_q2_pos.publish(q2)
        pub_q3_pos.publish(q3)
        rate.sleep()
    
    
    # rospy.loginfo("Ready to perform inverse kinematics")
    rospy.spin()
    rospy.wait_for_service('/robot/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/robot/controller_manager/switch_controller', SwitchController)
        start_controllers = ['joint0_velocity_controller','joint2_velocity_controller','joint3_velocity_controller']
        stop_controllers = ['joint0_position_controller','joint2_position_controller','joint3_position_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)

    except rospy.ServiceException as e:
        print("Service Call Failed")	

if __name__ == '__main__':	
	try:
		position_control()
		rospy.sleep(30)
	except rospy.ROSInterruptException:
		pass
