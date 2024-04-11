#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <complex>

using namespace std;

class uav_state
{
    public:
        ros::NodeHandle nh;
        ros::Subscriber local_pos_sub;
        ros::Subscriber local_vec_sub;
        ros::Subscriber state_sub;
        ros::Subscriber obstacle_sub;
        ros::Publisher local_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Subscriber local_yaw_sub;

        mavros_msgs::PositionTarget pose;
        mavros_msgs::State current_state;
    

        int well,mode;
        float px,py,pz,vx,vy,vz,ax,ay,az,yaw_t,avoid[3],distent;

        uav_state(){
            local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10,&uav_state::subpose,this);

            local_vec_sub =  nh.subscribe< geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 10, &uav_state::subvec,this);

            local_yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/target_local", 10, &uav_state::subyaw, this);

            state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10,&uav_state::state_cb,this );

            // obstacle_sub = nh.subscribe<uav_path::avoid_vector>("avoid_vector", 10,&uav_state::obstacle,this);

            local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
            
            //local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");

            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");  // /uav1/        
                     
            }

            mavros_msgs::SetMode offb_set_mode;
            //ros::Rate rate();
            mavros_msgs::CommandBool arm_cmd;
  


        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state = *msg;}

        void subpose(const geometry_msgs::PoseStamped::ConstPtr &msg){   
            px=msg->pose.position.x;  py=msg->pose.position.y;  pz=msg->pose.position.z; }

        void subvec(const  geometry_msgs::TwistStamped::ConstPtr &msg){
            vx=msg->twist.linear.x; vy=msg->twist.linear.y; vz=msg->twist.linear.z;}

        void subyaw(const  mavros_msgs::PositionTarget::ConstPtr &msg){
            yaw_t=msg->yaw;}

        // void obstacle(const uav_path::avoid_vector::ConstPtr &msg){
        //     avoid[0]=msg->velocity.x;    avoid[1]=msg->velocity.y; avoid[2]=msg->velocity.z;well=msg->well;distent=msg->distent; mode=msg->mode;}

        void pub_position(float x,float y,float z){
            pose.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            local_pos_pub.publish(pose); }

        void pub_position(float x,float y,float z,float rad){
            pose.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
            pose.coordinate_frame = 1;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void pub_velocity(float x,float y,float z){
            pose.type_mask =1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.velocity.x=x;
            pose.velocity.y=y;
            pose.velocity.z=z;
            local_pos_pub.publish(pose); }
        
        void pub_velocity(float x,float y,float z,float rad){
            pose.type_mask =1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + /*1024 +*/ 2048;
            pose.coordinate_frame = 1;
            pose.velocity.x=x;
            pose.velocity.y=y;
            pose.velocity.z=z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void pub_acceleration(float x,float y,float z){
            pose.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 */+ 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.acceleration_or_force.x=x;
            pose.acceleration_or_force.y=y;
            pose.acceleration_or_force.z=z;
            local_pos_pub.publish(pose); }

        void pub_acceleration(float x,float y,float z,float rad){
            pose.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 + 1024*/ + 2048;
            pose.coordinate_frame = 1;
            pose.acceleration_or_force.x=x;
            pose.acceleration_or_force.y=y;
            pose.acceleration_or_force.z=z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void get_position(){
            cout<<"P_x:"<<px<<" P_y:"<<py<<" P_z:"<<pz<<endl;     }

        void get_velocity(){
            cout<<"V_x:"<<vx<<" V_y:"<<vy<<" V_z:"<<vz<<endl;     }

        void get_acceleration(){
            cout<<"A_x:"<<ax<<" A_y:"<<ay<<" A_z:"<<az<<endl;     }
};
