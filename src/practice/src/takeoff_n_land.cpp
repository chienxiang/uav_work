/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <iomanip>
using namespace std;
#define PI 3.1415927
mavros_msgs::State current_state;
sensor_msgs::NavSatFix NavSatStatus;
geometry_msgs::PoseStamped posesub;
float_t latitude; //經度
float_t longitude; //緯度
float px,py,pz;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void golbal_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    NavSatStatus = *msg;
    latitude = NavSatStatus.latitude;
    longitude = NavSatStatus.longitude;
}
void get_golbal_pos(){
    cout<<"latitude:"<<fixed<<setprecision(7)<<latitude<<" longitude:"<<fixed<<setprecision(7)<<longitude<<endl; //小數點後7位差距為cm
}
// void get_position()
// {
//   cout<<"P_x:"<<px<<" P_y:"<<py<<" P_z:"<<pz<<endl; 
// }
// void subpose(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {   
//     posesub = *msg;
//     px=posesub.pose.position.x;  py=posesub.pose.position.y;  pz=posesub.pose.position.z;
// }
int main(int argc, char **argv)
{

    ros::init(argc, argv, "uav1");
    ros::NodeHandle nh("~");
    //ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber golbal_pos_sub = nh.subscribe <sensor_msgs::NavSatFix>
            ("mavros/global_position/global",10,golbal_pos_cb);
    // ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("/mavros/local_position/pose", 10,subpose);



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

     // wait for FCU connection
      while(ros::ok() && !current_state.connected){
          ros::spinOnce();        
          rate.sleep();
      }
 
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int ang = 0,height=0;
    printf("input height=");
    scanf("%d",&height);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                    
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        pose.pose.position.z = height;
        local_pos_pub.publish(pose);
        if(1)
        {
          //get_golbal_pos();
          // get_position();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



