#include "ros/ros.h"
#include "rbe500_group3/InverseKinematics.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose"
#include <math.h>   

using namespace std;

bool handle_inverse_kine(rbe500_group3::InverseKinematics::Request &req, rbe500_group3::InverseKinematics::Response &res){

    //Get position and orientation from pose message

    float xp=req.pose.position.x;
    float yp=req.pose.position.y;
    float zp=req.pose.position.z;
    float xp=req.pose.position.x;
    
    float xq=req.pose.orientation.x;
    float yq=req.pose.orientation.y;
    float zq=req.pose.orientation.z;
    float wq=req.pose.orientation.w;

    //Hard coded joint lengths

    float L1 = .5;
    float L2 = .5;
    float L3 = .6;

    //Calculate d3
    float d3 = -(L3 - zp);

    //Calculate physical parameters based on geometry
    float D = (xp**2 + yp**2 - L1**2 - L2**2)/(2*L1*L2);

    //Calculate q2
    float q2 = atan2(sqrt(1-D**2),D);

    // Calculate q1
    float q1_x = L1 + L2*np.cos;(q2)
    float q1_y = L2*sin(q2);
    float q1 = atan2(yp,xp) + atan2(q1_y,q1_x);

    // Prepare message for printing in console
    //message = format('Solved for joint values: ' + \
              '\nq1: ' + "{}" + \
              '\nq2: ' + "{}"+ \
              '\nd3: ' + "{}",q1,q2,d3))

    // Send message
    ROS_INFO(format('Solved for joint values: ' + \
              '\nq1: ' + "{}" + \
              '\nq2: ' + "{}"+ \
              '\nd3: ' + "{}",q1,q2,d3)));

    float joint_names[] = {'q1','q2','d3'}
    float joint_pos[] = {q1, q2, d3}

    sensor_msgs::JointState joint_msg 
    joint_msg.name = joint_names
    joint_msg.position = joint_pos
    return InverseKinematicsResponse(joint_msg)







    return true
}



int main(int argc,char** argv){
    ros::init(argc,argv,'inverse_kine_server');
    ros::NodeHandle n;
    ros::ServiceServer service= n.advertiseService('inverse_kine_server',handle_inverse_kine);
    ROS_INFO("Ready to perform inverse kinematics");
    ros::spin();
    return 0;
}
