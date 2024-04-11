#include "practice/Uav.h"
#include <coordination_transfer.h>
#include <iostream>
#include <fstream>
using namespace std;
#define PI 3.1415927
bool flag = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    uav1.init();
    uav0.init();
    uav2.init();
    ros::Time last_request = ros::Time::now();
    float i=0.02,tmp;
    float init_p[3][3]={{0,0,1},{0,0,1},{0,0,1}};
    int j=0;
    //讀取路徑
    std::ifstream file;
    std::vector<float> pose;
    file.open("./src/practice/src/pose.txt");
    if(!file.is_open())
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }
    while(file >> tmp)
    {
        pose.push_back(tmp);
    }
    file.close();

    int choose=0;
    cout<<"choose homepos_sub(1) , offboard2uav(2) , check home pos:";
    cin>>choose;
    switch(choose)
    {
        case 1:
            while(ros::ok())
            {
                cout<<"uav0_pos"<<endl;
                uav0.get_position();   
                cout<<"uav1_pos"<<endl;
                uav1.get_position();
                cout<<"uav2_pos"<<endl;
                uav2.get_position();
                ros::spinOnce();
                uav1.rate.sleep();
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
                else if(ros::Time::now() - last_request > ros::Duration(15.0))
                {   
                    uav0.pub_position(init_p[0][0],init_p[0][1],init_p[0][2]);
                    uav1.pub_position(init_p[1][0],init_p[1][1],init_p[1][2]);
                    uav2.pub_position(init_p[2][0],init_p[2][1],init_p[2][2]); 
                }
                ros::spinOnce();
                uav0.rate.sleep();
            }
            break;
        case 3:

            break;
    }    
    return 0;
}
