#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <iomanip>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <complex>
#include "practice/information.h"
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
class Uav
{
    public:
        Uav(int id);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void subpose_origin(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void subpose_compensate(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void subvel(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void wait_for_FCU();
        void send_setpoints();
        void set_mode_offb();
        void set_mode_manual();
        void set_arm_cmd();
        void init();
        void enable_offboard();
        void enable_armed();
        void landing();
        void pub_position(float x, float y, float z);
        void pub_position(float x,float y,float z,float rad);
        void pub_velocity(float x,float y,float z);
        void pub_velocity(Matrix3f _Frep);
        void pub_velocity(Matrix3f _F,float rad);
        void pub_velocity(float x,float y,float z,float rad);
        void pub_acceleration(float x,float y,float z);
        void pub_acceleration(float x,float y,float z,float rad);
        void Incremental_PID(float targetx,float targety,float targetz);
        void get_position();
        void get_velocity();
        void get_acceleration();
        void get_global_pos();
        int uav_drifting();
        void data_update();




        ros::Subscriber state_sub;
        ros::Publisher local_pos_pub;
        ros::Publisher PositionTarget_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Subscriber local_pos_sub_origin;
        ros::Subscriber local_pos_sub_compensate;
        ros::Subscriber local_vel_sub;
        ros::ServiceClient land_client;
        ros::Subscriber global_pos_sub;

        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pose;
        mavros_msgs::PositionTarget PositionTarget;
        mavros_msgs::SetMode set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::CommandTOL land_cmd;
        sensor_msgs::NavSatFix NavSatStatus;
        practice::information inf;
        Vector3f p_data; //uav information of position
        Vector3f v_data; //uav information of velocity
          

        ros::NodeHandle nh;
        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate;

        std::ifstream ifs;
        std::vector<float> drifting;
        int uav_id;
        float px,py,pz,vx,vy,vz,ax,ay,az,Opx,Opy,Opz,tmp;
        float driftX=0,driftY=0,home_driftX=0,home_driftY=0,home_driftZ=0;
        float latitude; //經度
        float longitude; //緯度
        float Current_Error[3];//当前误差
        float Last_Error[3];//上一次误差
        float Previous_Error[3];//上上次误差
        float v[3]={0,0,0}; //pid 速度

        bool mission_done = false;
        bool island = false;

};
