#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include "geometry_msgs/Pose.h"

float len[]={0.5,0.5,0.5,0.1};   
                               //Assumed parameters for length of links               
float a[]={len[1],len[2],0};

float alpha[]={0,0,0};
float theta[]={0,0,0};

using namespace tf2;

void fk_callback(const sensor_msgs::JointState msg){
    float q1,q2,d3=0;
    theta[0]=msg.position[0];
    theta[1]=msg.position[1];
    d3=msg.position[2];
    float d[]={len[0],0,len[3]+d3};
    Matrix3x3 I(1,0,0,                           //Identity Matrix
                0,1,0,
                0,0,1);
    Vector3 s(0,0,0);
    Transform identity(I,s);
    for(int i=0;i<3;i++){                     /*Creating individual homogenous matrices for each rotation and multiplying them together*/
       Matrix3x3 x(cos(theta[i]), -sin(theta[i])*cos(alpha[i]),sin(theta[i])*sin(alpha[i]),    
       sin(theta[i]),cos(theta[i])*cos(alpha[i]),-cos(theta[i])*sin(alpha[i]),
       0,sin(alpha[i]),cos(alpha[i]));
       Vector3 v(a[i]*cos(theta[i]),a[i]*sin(theta[i]),d[i]);
       Transform A(x,v);
       identity=identity*A;
    }
    
    geometry_msgs::Pose pose;
    toMsg(identity,pose);
    std::cout<<pose;
    ROS_INFO_STREAM(pose);                    //Outputting the end effector pose as a Pose message

}


int main(int argc,char** argv){
    ros::init(argc,argv,"fk_node");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("/robot/joint_states",1000,fk_callback);
    ros::spin();
    return 0;
}