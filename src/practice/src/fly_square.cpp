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

    ros::NodeHandle nh;
    uav.offb_set_mode.request.custom_mode = "OFFBOARD";
    uav.arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
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
    int cnt=1,length=4,stop=0;
    double disx=0,disy=0,i=0.02,targetPoint=0,tmp=0;
    printf("Square side length = 4");
    // scanf("%d",&length);
    // if (length%2 == 0 && length < 7)
    //     length = length;
    // else
    //     length = 2;
    while(ros::ok())
    {
        if( uav.current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
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
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( uav.arming_client.call(uav.arm_cmd) &&
                        uav.arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
        }

        if(ros::Time::now() - last_request < ros::Duration(8.0)) //懸停8秒
        uav.pub_position(0,0,5);
        if(ros::Time::now() - last_request > ros::Duration(8.0))
        {
            if(cnt == 1)
            {   
                targetPoint = length/2;
                if(disx < targetPoint)
                {
                    disx=disx+i;
                    uav.pub_position(disx,0,5);
                }
                else
                {
                    ROS_INFO("Path1 finish");
                    cnt=2;
                }
            }
            else if(cnt == 2)
            {
                targetPoint = length/2;
                if(disy < targetPoint)
                {   
                    disy=disy+i;
                    uav.pub_position(disx,disy,5);
                }
                else
                {
                    ROS_INFO("Path2 finish");
                    cnt=3;
                }
                    
            }
            else if (cnt == 3)
            {
                targetPoint = -length/2;
                if(disx > targetPoint)
                {   
                    disx=disx-i;
                    uav.pub_position(disx,disy,5);
                }
                else
                {
                    ROS_INFO("Path3 finish");
                    cnt=4;
                }
            }
            else if (cnt == 4)
            {
                targetPoint = -length/2;
                if(disy > targetPoint)
                {   
                    disy=disy-i;
                    uav.pub_position(disx,disy,5);
                }
                else
                {               
                    ROS_INFO("Path4 finish");
                    cnt=5;
                }
            }
            else if (cnt == 5)
            {
                targetPoint = length/2;
                if(disx < targetPoint)
                {   
                    disx=disx+i;
                    uav.pub_position(disx,disy,5);
                }
                else
                {              
                    ROS_INFO("Path5 finish");
                    cnt=6;
                }
            }
            else if (cnt == 6)
            {
                targetPoint = length/2;
                if(disy < targetPoint)
                {   
                    disy=disy+i;
                    uav.pub_position(disx,disy,5);
                }
                else
                {
                    ROS_INFO("Path6 finish");
                    cnt=2;
                    if (stop == 1)
                    {
                        cnt = 7;
                    }
                    stop++;
                }
            }
            else if(cnt == 7)
            {
                uav.pub_position(disx,disy,2);
                disx = disx-0.1;
                disy = disy-0.1;
                tmp++;
                if(tmp == 10)
                cnt = 8;
            }
            else if (cnt == 8)
            {
                if(uav.px <= 0.2&&uav.py <= 0.2)
                {
                    pid_control.landing();
                    break;
                }
                else
                {   uav.get_position();
                    uav.pub_position(0,0,2);
                }
                

            }            
        }
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
