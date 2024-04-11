#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <string> 

using namespace std;

mavros_msgs::State current_state;

// 无人机状态回调函数
void state_callback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe("/mavros/state", 10, state_callback);

    ros::Rate rate(1);

    ROS_INFO("Initializing...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected.");
    
    ros::Publisher local_vel_pub = n.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher rc_override = n.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    ros::Publisher local_att_pub = n.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude/cmd_vel", 10);
    ros::ServiceClient land_client = n.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::Publisher thrust_pub = n.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
    ros::Publisher att_pose_pub = n.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/target_attitude", 10);
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    
    mavros_msgs::CommandTOL srv_takeoff;
    mavros_msgs::CommandBool srv;    
    mavros_msgs::SetMode srv_setMode;
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    
    geometry_msgs::TwistStamped att;
    att.twist.linear.x = 1;
    att.twist.linear.y = 0;
    att.twist.linear.z = 0;

    mavros_msgs::Thrust th;
    th.thrust=0.5;
    
    
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    
    mavros_msgs::AttitudeTarget attp;
    
            
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    
    mavros_msgs::OverrideRCIn rc;
    
    ros::Time t1=ros::Time::now();
    float step=0;
    int choose=0;
    cout<<"choose pose(1) or attitude_vel(2) or attiutde_pose(3) or rc(4) :";
    cin>>choose;
    
    ROS_INFO("start!!");
    
        switch(choose){
        case 1:
        
        // 设置无人机模式为GUIDED
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    // 解锁无人机
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
    
        while (ros::ok()) {
        pose.pose.position.x = 5*sin(step);
        pose.pose.position.y = 5*cos(step);
        pose.pose.position.z = -2;
        step=step+0.05;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        }
        break;
        

        case 2:
       
        // 设置无人机模式为GUIDED_NOGPS    
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "STABILIZE";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }
    
    // 解锁无人机
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
        ROS_INFO("down");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(10)){
        att.twist.linear.x = 0;
        att.twist.linear.y = 0;
        att.twist.linear.z = 0;
        th.thrust= 0.3;
        local_att_pub.publish(att);
        thrust_pub.publish(th);
        ros::spinOnce();        
        sleep(2);  }
                
        ROS_INFO("forward");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(5)){
        att.twist.linear.x = 0;
        att.twist.linear.y = 0.5;
        att.twist.linear.z = 0;
        th.thrust= 0.5;
        local_att_pub.publish(att);
        thrust_pub.publish(th);
        ros::spinOnce();        
        rate.sleep();  }
        
        ROS_INFO("up");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(10)){
        att.twist.linear.x = 0;
        att.twist.linear.y = 0;
        att.twist.linear.z = 0;
        th.thrust= 0.7;
        local_att_pub.publish(att);
        thrust_pub.publish(th);
        ros::spinOnce();        
        rate.sleep();  }

        ros::spinOnce();
        rate.sleep();       
        break;
        
        case 3:
        
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "stabilize";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }
    
    // 解锁无人机
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
    
        ROS_INFO("down");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(10)){
        attp.type_mask = /*1 + 2 + 4 + 64 +*/ 128;
        attp.orientation.w = 1;
        attp.orientation.x = 0;
        attp.orientation.y = 0;
        attp.orientation.z = 0;
        attp.thrust = 0.3;
        att_pose_pub.publish(attp);
        ros::spinOnce();        
        rate.sleep();  }

        
        ros::spinOnce();
        rate.sleep();
        
        break;
        
        case 4:
        srv_setMode.request.custom_mode = "alt_hold";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }
        
        srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
        ROS_INFO("down");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[2]=1350;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        t1=ros::Time::now();
        ROS_INFO("hold");
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[2]=1500;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        t1=ros::Time::now();
        ROS_INFO("forward");
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[4]=1550;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        break;
        
        }       
        
        ros::spinOnce();
        rate.sleep();
    
    ROS_INFO("END!!");
    return 0;
}

