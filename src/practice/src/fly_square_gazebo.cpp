/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    //uav1
    ros::NodeHandle nh1;
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    //uav2
    ros::NodeHandle nh2;
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // x_num, y_num  refer to the desired uav input position
    //all the uavs have the same height : z

    float x0 = 0.0, y0 = 0.0;
    float x1 = 0.0, y1 = 0.0;
    float x2 = 0.0, y2 = 0.0;
    float z = 2.0;
    float w = 0.0;
    const float pi = 3.1415926;
    geometry_msgs::PoseStamped pose0;
    geometry_msgs::PoseStamped pose1;
    geometry_msgs::PoseStamped pose2;
    pose0.pose.position.x = x0;
    pose0.pose.position.y = y0;
    pose0.pose.position.z = z;
    pose1.pose.position.x = x1;
    pose1.pose.position.y = y1;
    pose1.pose.position.z = z;
    pose2.pose.position.x = x2;
    pose2.pose.position.y = y2;
    pose2.pose.position.z = z;

    //uavs have the initial position offset because of the .launch config file
    float x0_offset = -3;
    float x1_offset = -1;
    float x2_offset = 1;

    //define uavs' coordinates in the formation coordinate system
    float x0_f = -0.5, y0_f = 0.5;
    float x1_f = 0.5, y1_f = 0.5;
    float x2_f = -0.5, y2_f = -0.5;
    float xx, yy;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 3");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 3");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);

        w = w + 2*pi/(30/(1/20.0));
        if(w > 2*pi){
            w = w - 2*pi;
        }
        xx = 4.75*cos(w);
        yy = 4.75*sin(w);
        
        x0 = x0_f*cos(w) - y0_f*sin(w) + xx - x0_offset;
        y0 = y0_f*cos(w) + x0_f*sin(w) + yy;
        pose0.pose.position.x = x0;
        pose0.pose.position.y = y0;

        x1 = x1_f*cos(w) - y1_f*sin(w) + xx - x1_offset;
        y1 = y1_f*cos(w) + x1_f*sin(w) + yy;
        pose1.pose.position.x = x1;
        pose1.pose.position.y = y1;

        x2 = x2_f*cos(w) - y2_f*sin(w) + xx - x2_offset;
        y2 = y2_f*cos(w) + x2_f*sin(w) + yy;
        pose2.pose.position.x = x2;
        pose2.pose.position.y = y2;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
