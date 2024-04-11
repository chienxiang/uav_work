#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <PID.h>
#include <uav_state.h>
#define PI 3.1415927
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    uav_state uav;
    pid pid_control;

    //ros::NodeHandle nh;
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);


    // wait for FCU connection
    while(ros::ok() && !uav.current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // uav.local_pos_pub.publish(pose);
        uav.pub_position(0,0,0);
        ros::spinOnce();
        rate.sleep();
    }
    // geometry_msgs::TwistStamped vel_linear;
    // vel_linear.twist.linear.x=1;
    // vel_linear.twist.linear.y=1;
    // vel_linear.twist.linear.z=1;
    // geometry_msgs::Vector3Stamped acc;
    // acc.vector.x=1;
    // acc.vector.y=1;
    // acc.vector.z=1;
    // int count = 0;
    // //
    // mavros_msgs::PositionTarget PositionTarget;
    // mavros_msgs::AttitudeTarget AttitudeTarget;
    // local_pos_pub.publish(pose);

    uav.offb_set_mode.request.custom_mode = "OFFBOARD";
    uav.arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( uav.current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if( uav.set_mode_client.call(uav.offb_set_mode) &&
                    uav.offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !uav.current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if( uav.arming_client.call(uav.arm_cmd) &&
                        uav.arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
        }

        if(ros::Time::now() - last_request < ros::Duration(10.0)) //懸停10秒
        uav.pub_position(0,0,1);
        // if(ros::Time::now() - last_request > ros::Duration(10.0))
        // {
        //         // PositionTarget.position.x=0;
        //         // PositionTarget.position.y=0;
        //         // PositionTarget.position.z=2;
        //         // PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
        //         // PositionTarget.type_mask=PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
        //         // PositionTarget_pub.publish(PositionTarget);
        //         ROS_INFO("position offboard");
        // }
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
