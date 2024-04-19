#include "practice/Uav.h"
#include "practice/controller.h"
#include <coordination_transfer.h>
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
    Controller ctr;
    uav1.init();
    uav0.init();
    uav2.init();
    ros::Time last_request = ros::Time::now();
    float tgt=-10,dis=0,i=0.02,tmp;
    float init_p[3][3]={{-3/sqrt(3),0,5},{1.5/sqrt(3),-1.5,5},{1.5/sqrt(3),1.5,5}};
    int j=0;
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
    cout<<"choose formation(1) or IAPF(2) or NSB(3):";
    cin>>choose;
    switch(choose)
    {
        case 1:
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
                    uav0.pub_position(init_p[0][0],init_p[0][1],init_p[0][2]);
                    uav1.pub_position(init_p[1][0],init_p[1][1],init_p[1][2]);
                    uav2.pub_position(init_p[2][0],init_p[2][1],init_p[2][2]);  
                }
                if(ros::Time::now() - last_request > ros::Duration(15.0))
                {   
                    uav0.data_update();
                    uav1.data_update();
                    uav2.data_update();

                    // controller.data_update(uav0.inf,uav1.inf,uav2.inf);
                    // controller.update_center(path[j],path[j+1],path[j+2]);
                    // controller.follower();
                    uav0.pub_velocity(controller.Fuav[0].vel.x,controller.Fuav[0].vel.y,controller.Fuav[0].vel.z);
                    uav1.pub_velocity(controller.Fuav[1].vel.x,controller.Fuav[1].vel.y,controller.Fuav[1].vel.z);
                    uav2.pub_velocity(controller.Fuav[2].vel.x,controller.Fuav[2].vel.y,controller.Fuav[2].vel.z);
                    if(j!=path.size()-3)
                        j+=3;
                }
                ros::spinOnce();
                uav0.rate.sleep();
            }
            break;

        case 2:
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
                    uav0.pub_position(init_p[0][0],init_p[0][1],init_p[0][2]);
                    uav1.pub_position(init_p[1][0],init_p[1][1],init_p[1][2]);
                    uav2.pub_position(init_p[2][0],init_p[2][1],init_p[2][2]);  
                }
                if(ros::Time::now() - last_request > ros::Duration(15.0))
                {   
                    uav0.data_update();
                    uav1.data_update();
                    uav2.data_update();

                    // controller.data_update(uav0.inf,uav1.inf,uav2.inf);
                    // controller.process();
                    if(dis>tgt)
                    {   
                        dis=dis-i;
                    }
                    uav0.pub_position(dis,0,5);
                    //uav0.pub_velocity(controller.Cuav[0].vel.x,controller.Cuav[0].vel.y,controller.Cuav[0].vel.z);
                    uav1.pub_velocity(controller.Cuav[1].vel.x,controller.Cuav[1].vel.y,controller.Cuav[1].vel.z);
                    uav2.pub_velocity(controller.Cuav[2].vel.x,controller.Cuav[2].vel.y,controller.Cuav[2].vel.z);
                }
                ros::spinOnce();
                uav0.rate.sleep();
            }
            break;

        case 3:
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
                    uav0.pub_position(init_p[0][0],init_p[0][1],init_p[0][2]);
                    uav1.pub_position(init_p[1][0],init_p[1][1],init_p[1][2]);
                    uav2.pub_position(init_p[2][0],init_p[2][1],init_p[2][2]);  
                }
                if(ros::Time::now() - last_request > ros::Duration(15.0))
                {   
                    uav0.data_update();
                    uav1.data_update();
                    uav2.data_update();

                    // controller.data_update(uav0.inf,uav1.inf,uav2.inf);
                    // controller.update_center(path[j],path[j+1],path[j+2]);
                    // controller.null_space_behavior();
                    uav0.pub_velocity(controller.Nuav[0].vel.x,controller.Nuav[0].vel.y,controller.Nuav[0].vel.z);
                    uav1.pub_velocity(controller.Nuav[1].vel.x,controller.Nuav[1].vel.y,controller.Nuav[1].vel.z);
                    uav2.pub_velocity(controller.Nuav[2].vel.x,controller.Nuav[2].vel.y,controller.Nuav[2].vel.z);
                    if(j!=path.size()-3)
                        j+=3;
                }
                ros::spinOnce();
                uav0.rate.sleep();
            }
            break;

    }    
    return 0;
}
