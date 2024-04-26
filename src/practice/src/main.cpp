#include "practice/Uav.h"
#include "practice/controller.h"
#include <iostream>
#include <fstream>
using namespace std;
#define PI 3.1415927
bool flag = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Controller controller;
    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    uav1.init();
    uav0.init();
    uav2.init();
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {   
        if(uav0.current_state.mode != "OFFBOARD" && uav1.current_state.mode != "OFFBOARD" && uav2.current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            uav0.enable_offboard();
            uav1.enable_offboard();
            uav2.enable_offboard();
            last_request = ros::Time::now();
        }
        else
        {
            if( !uav0.current_state.armed && !uav1.current_state.armed && !uav2.current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                uav0.enable_armed();
                uav1.enable_armed();
                uav2.enable_armed();
                last_request = ros::Time::now();
            }
        }
        if(ros::Time::now() - last_request < ros::Duration(15.0))
        {   
            uav0.pub_position(0,0,5);
            uav1.pub_position(3,0,5);
            uav2.pub_position(0,3,5);  
        }
        else if(ros::Time::now() - last_request > ros::Duration(15.0))
        {   
            uav0.pub_position(0,0,5);
            uav1.pub_position(3,3,5);
            uav2.pub_position(0,3,5);

            uav0.data_update();
            uav1.data_update();
            uav2.data_update();

            controller.data_update(uav0.p_data,uav1.p_data,uav2.p_data,uav0.v_data,uav1.v_data,uav2.v_data);
            controller.safe_range();
        }
        ros::spinOnce();
        uav0.rate.sleep();
    }    

    

    return 0;
}
