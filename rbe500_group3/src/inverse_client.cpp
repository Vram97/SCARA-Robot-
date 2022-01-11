#include "ros/ros.h"
#include "rbe500_group3/InverseKinematics.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose"
#include "rbe500_group/InverseKinematics.h"
#include <math.h>   

using namespace std;

int main(int argc,char** argv){
    ros::init(argc, argv, "inversenode_client");
    ros::NodeHandle n;
    ros::ServiceClient client=n.serviceClient<rbe500_group::InverseKinematics>("inverse_kine_server");
    rbe500_group::InverseKinematics srv;

    geometry_msgs::Pose pose;

    pose.position.x = stof(argv[1]);
    pose.position.y= stof(argv[2]);
    pose.position.z= stof(argv[3]);
    pose.orientation.x = stof(argv[4]);
    pose.orientation.y= stof(argv[5]);
    pose.orientation.z= stof(argv[6]);
    pose.orientation.w= stof(argv[7]);

    srv.request.pose=pose;
    if(client.call(srv)){
        sensor_msgs::JointState resp=srv.response.joints;
        ROS_INFO(resp);
    }
    else{
        ROS_INFO("Service call failed");
    }
    return 0;
}