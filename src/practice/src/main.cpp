#include "practice/Uav.h"
#include "practice/controller.h"
#include <iostream>
#include <fstream>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Controller controller;
    float p = 0,tmp;
    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    uav1.init();
    uav0.init();
    uav2.init();
    controller.Circumcentre_init();
    ros::Time last_request = ros::Time::now();
    //讀取路徑
    std::ifstream file;
    std::vector<float> path;
    file.open("./src/practice/src/path.txt");
    if(!file.is_open())
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }
    while(file >> tmp)
    {
        path.push_back(tmp);
    }
    file.close();

    int choose=0;
    cout<<"choose IAPF(1) or formation(2) or NSB(3):";
    cin>>choose;
    switch(choose)
    {
        case 1:
                while(ros::ok())
                {   
                    if(uav0.current_state.mode != "OFFBOARD" && uav1.current_state.mode != "OFFBOARD" && uav2.current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
                    {
                        uav0.enable_offboard();
                        uav1.enable_offboard();
                        uav2.enable_offboard();
                        last_request = ros::Time::now();
                    }
                    else
                    {
                        if( !uav0.current_state.armed && !uav1.current_state.armed && !uav2.current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
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
                        uav0.data_update();
                        uav1.data_update();
                        uav2.data_update();
                        controller.data_update(uav0.p_data,uav1.p_data,uav2.p_data,uav0.v_data,uav1.v_data,uav2.v_data);
                        uav0.Incremental_PID(0,6,5);
                        uav1.pub_position(3,0,5);
                        uav2.pub_position(0,3,5);
                        // uav2.Incremental_PID(0,3,5);
                        controller.virtual_vel();
                        uav0.pub_velocity(controller.Frep,0);
                        // uav2.pub_velocity(controller.Frep);
                    }
                    ros::spinOnce();
                    uav0.rate.sleep();
                }
                break;
        case 2:
                while(ros::ok())
                {   
                    if(uav0.current_state.mode != "OFFBOARD" && uav1.current_state.mode != "OFFBOARD" && uav2.current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
                    {
                        uav0.enable_offboard();
                        uav1.enable_offboard();
                        uav2.enable_offboard();
                        last_request = ros::Time::now();
                    }
                    else
                    {
                        if( !uav0.current_state.armed && !uav1.current_state.armed && !uav2.current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                            uav0.enable_armed();
                            uav1.enable_armed();
                            uav2.enable_armed();
                            last_request = ros::Time::now();
                        }
                    }
                    if(ros::Time::now() - last_request < ros::Duration(10.0))
                    {   
                        uav0.pub_position(-3/sqrt(3),0,5);
                        uav1.pub_position(1.5/sqrt(3),-1.5,5);
                        uav2.pub_position(1.5/sqrt(3),1.5,5);  
                    }
                    else if(ros::Time::now() - last_request > ros::Duration(10.0))
                    {
                        uav0.data_update();
                        uav1.data_update();
                        uav2.data_update();
                        controller.data_update(uav0.p_data,uav1.p_data,uav2.p_data,uav0.v_data,uav1.v_data,uav2.v_data);
                        controller.Uav_Circumcentre_move(path[p],path[p+1],path[p+2]);
                        // controller.Uav_Circumcentre_move(1,1,5);
                        uav0.pub_velocity(controller.Fformation,0);
                        uav1.pub_velocity(controller.Fformation,0);
                        uav2.pub_velocity(controller.Fformation,0);
                        if(p!=path.size()-3)
                            p+=3;

                    }
                    ros::spinOnce();
                    uav0.rate.sleep();
                }
                break;
                        
    }
    

    return 0;
}
