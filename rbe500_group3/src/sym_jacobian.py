#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
from sympy import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500_group.msg import *
from rbe500_group.srv import *

t1,t2,t3,d1,d2,d3,d4 = symbols('t1 t2 t3 d1 d2 d3 d4')
prev_q1 = 0
prev_q2 = 0
prev_q3 = 0
t = 0

def update_qvals(q1,q2,d3):
    global prev_q1
    global prev_q2
    global prev_q3
    prev_q1 = q1
    prev_q2 = q2
    prev_q3 = d3

# Can be used to test manual values
def callback_once(Joints):
    # Get joint values from Joints message
    q1 = Joints.q1
    q2 = Joints.q2
    d3 = Joints.q3
    update_qvals(q1,q2,d3)

    calculate_jacobian(q1,q2,d3)

def callback_gazebo(JointState):
    # Get joint values from Joints message
    q1 = JointState.velocity[0]
    q2 = JointState.velocity[1]
    d3 = JointState.velocity[2]
    update_qvals(q1,q2,d3)

    calculate_jacobian(q1,q2,d3)
    straight_line_pub()

def calculate_jacobian(q1,q2,q3):
    # Hard coded joint lengths
    L1 = 0.5
    L2 = 0.5
    L3 = 0.5
    L4 = 0.1

    # Symbolic matrices
    Ts = transformation_matrix()
    Js = jacobian_sym()
    Js = Js[0:3,0:3]
    
    # Numerical evaluation

    # Homogeneous transformation matrix
    Tsn = Ts.subs([(t1,q1),(t2,q2),(t3,q3),(d1,L1),(d2,L2),(d3,L3),(d4,L4)])
    Tn = Tsn.evalf()
    T = np.array(Tn).astype(np.float64)

    # Jacobian matrix
    Jsn = Js.subs([(t1,q1),(t2,q2),(t3,q3),(d1,L1),(d2,L2),(d3,L3),(d4,L4)])
    Jn = Jsn.evalf()
    J = np.array(Jn).astype(np.float64)

    # Inverse Jacobian
    Jinv = np.linalg.pinv(J)

    # Solve for end-effector velocities
    v = np.array([0,1,0])
    x = Jinv @ v

    check = J @ x

    matrix_message = '\nThe homogeneous transformation matrix is: \n' + \
                np.array2string(T, None, 4) + '\n' + \
                    '\nThe Jacobian matrix is: \n' + \
                np.array2string(J, None, 4) + '\n' + \
                    '\nThe inverse Jacobian matrix is: \n' + \
                np.array2string(Jinv, None, 4) + '\n' + \
                    '\nThe joint velocities to go in a straight line are: \n' + \
                np.array2string(x, None, 4) + '\n' + \
                     '\nChecking gives: \n' + \
                np.array2string(check, None, 4) + '\n'

    rospy.loginfo(matrix_message)
    
    return J

def calculate_inv_jacobian(v1,v2,v3):
    # Get Jacobian
    Jn = calculate_jacobian(prev_q1,prev_q2,prev_q3)
    J = np.array(Jn).astype(np.float64)

    # Inverse Jacobian
    Jinv = np.linalg.pinv(J)

    # Solve for end-effector velocities
    v = np.array([v1,v2,v3])
    x = Jinv @ v

    update_qvals(x[0],x[1],x[2])

    return x

def straight_line_pub():
    # global t
    # valp = calculate_inv_jacobian(0,-1*np.cos(t),0)
    # t = t + 0.1

    valp = calculate_inv_jacobian(0,1,0)

    pub1 = rospy.Publisher('/robot/joint0_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/robot/joint3_velocity_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    q1 = valp[0]
    q2 = valp[1]
    d3 = valp[2]
    pub1.publish(q1)
    pub2.publish(q2)
    pub3.publish(d3)
    rate.sleep()

def transformation_matrix():
    # # DH Parameters
    # a = [L1,L3,0]
    # theta = [q1,q2,0]
    # d = [L2,0,L4+q3]
    # alpha = [0,0,0]

    A1s = sym_matrix(d1,t1,d2,0)
    A2s = sym_matrix(d3,t2,0,0)
    A3s = sym_matrix(0,0,d4+t3,0)
    T = A1s*A2s*A3s

    return T

def jacobian_sym():
    A1s = sym_matrix(d1,t1,d2,0)
    A2s = sym_matrix(d3,t2,0,0)
    A3s = sym_matrix(0,0,d4+t3,0)

    T10 = A1s
    T20 = A1s @ A2s
    T30 = A1s @ A2s @ A3s

    # Rotation matrices
    R00 = sp.Matrix(eye(3,3))
    R10 = T10[0:3,0:3]
    R20 = T20[0:3,0:3]
    R30 = T30[0:3,0:3]

    # Origins taken from homogeneous transformation matrices
    o00 = sp.Matrix([0,0,0])
    o10 = sp.Matrix(T10[0:3,3])
    o20 = sp.Matrix(T20[0:3,3])
    o30 = sp.Matrix(T30[0:3,3])

    z00 = sp.Matrix([0,0,1])
    z10 = R10 @ z00
    z20 = R20 @ z00

    # Jacobian components
    J11 = sp.Matrix(z00.cross(o30-o00))
    J12 = sp.Matrix(z00)

    J21 = sp.Matrix(z10.cross(o30-o10))
    J22 = sp.Matrix(z10)

    J31 = sp.Matrix(z20)
    J32 = sp.Matrix(o00)

    J1 = J11.col_join(J12)
    J2 = J21.col_join(J22)
    J3 = J31.col_join(J32)

    Jm1 = J1.row_join(J2)
    J = Jm1.row_join(J3)

    # init_printing(wrap_line=false)
    # pprint(J)

    return J

def inv_jacobian_sym():
    
    J = jacobian_sym()
    # Remove unnecessary rows in Jacobian
    Js = J[0:3,0:3]

    Jinv = Js.inv()

    # init_printing(wrap_line=false)
    # pprint(Jinv)

    return Jinv

def sym_matrix(a, theta, d, alpha):
    Ai = sp.Matrix([[sp.cos(theta), -1*sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                  [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -1*sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                  [0, sp.sin(alpha), sp.cos(alpha), d],
                  [0, 0, 0, 1]])

    return Ai

def listener():
    # Initialize each listener node uniquely
    rospy.init_node('jacobian_solver', anonymous=True)

    # Listen to jacobian_chatter
    rospy.Subscriber('jacobian_chatter', Joints, callback_once)

    # Listen to robot JointStates
    rospy.Subscriber('/robot/joint_states', JointState, callback_gazebo)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
