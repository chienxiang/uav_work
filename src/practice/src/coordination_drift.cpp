#include "practice/Uav.h"
#include <coordination_transfer.h>
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;
#define PI 3.1415927
int main(int argc, char **argv)
{
    ros::init(argc, argv, "golbalPos_node");

    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    coordination_transfer gps_trans;
    uav1.init();
    uav0.init();
    uav2.init();

    std::ofstream ofs;
    ofs.open("./src/practice/src/coordination_shifting.txt");
    if (!ofs.is_open()) 
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }

    std::ofstream ofs2;
    ofs2.open("./src/practice/src/uav_golbalPos.txt");
    if (!ofs2.is_open()) 
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }

    float shiftX1,shiftY1,shiftX2,shiftY2;
    gps_trans.GPS2XY(uav0.latitude,uav0.longitude,uav1.latitude,uav1.longitude);
    shiftX1 = gps_trans.local_y;shiftY1 = gps_trans.local_x;
    gps_trans.GPS2XY(uav0.latitude,uav0.longitude,uav2.latitude,uav2.longitude);
    shiftX2 = gps_trans.local_y;shiftY2 = gps_trans.local_x;

    ROS_INFO("uav1 coordination drift = (%f,%f)",shiftX1,shiftY1);
    ROS_INFO("uav2 coordination drift = (%f,%f)",shiftX2,shiftY2);
    ofs << shiftX1 <<" "<< shiftY1 << "\n";
    ofs << shiftX2 <<" "<< shiftY2 << "\n";

    ROS_INFO("uav0 home drift = (%f,%f)",uav0.Opx,uav0.Opy);
    ROS_INFO("uav1 home drift = (%f,%f)",uav1.Opx,uav1.Opy);
    ROS_INFO("uav2 home drift = (%f,%f)",uav2.Opx,uav2.Opy);

    ofs << uav0.Opx <<" "<< uav0.Opy << "\n";
    ofs << uav1.Opx <<" "<< uav1.Opy << "\n";
    ofs << uav2.Opx <<" "<< uav2.Opy << "\n";

    ofs.close();


    ofs2 <<fixed<<setprecision(8)<< "uav0 (lat,lon)" <<"("<<uav0.latitude<<","<<uav0.longitude<<")" << "\n"; 
    ofs2 <<fixed<<setprecision(8)<< "uav1 (lat,lon)" <<"("<<uav1.latitude<<","<<uav1.longitude<<")" << "\n";
    ofs2 <<fixed<<setprecision(8)<< "uav2 (lat,lon)" <<"("<<uav2.latitude<<","<<uav2.longitude<<")" << "\n";  
    ofs2.close();
    return 0;
}
