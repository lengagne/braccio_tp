#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include "Robot.h"


Eigen::Matrix<double,3,1> current_position, desired_position;

VECTOR q;

Robot Braccio;

std_msgs::Float64 error;

bool gripper_open;
int main(int argc, char** argv){
   
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    
    // init messages
    q.data.resize(6);
    q.data[0] = 0;
    q.data[1] = 1.57;
    q.data[2] = 0;
    q.data[3] = 0;
    q.data[4] = 0;
    q.data[5] = 0.72;
    
    error.data = 1e3;
        
    gripper_open = true;
    
    ros::Publisher pub_q = nh.advertise<std_msgs::Float64MultiArray>("joint_angles", 1000);
    ros::Publisher pub_error = nh.advertise<std_msgs::Float64>("control_error", 1000);
    
    ROS_INFO("start");
    ros::Rate loop_rate(2);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;    
    while (ros::ok())
    {
        try{
            listener.lookupTransform("/CubeJaune", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }        
        desired_position(0) = transform.getOrigin().x();
        desired_position(1) = transform.getOrigin().y();
        desired_position(2) = transform.getOrigin().z();


        try{
            listener.lookupTransform("/EndEffector", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }        
        current_position(0) = transform.getOrigin().x();
        current_position(1) = transform.getOrigin().y();
        current_position(2) = transform.getOrigin().z();
        
         if (gripper_open)
             q.data[5] = 0.0;
         else
             q.data[5] = 1.0;
        
        pub_q.publish(q);

        error.data = (desired_position-current_position).norm();
        pub_error.publish(error);
        
        ros::spinOnce();  
        loop_rate.sleep();        
    }
        
    return 0;
};


