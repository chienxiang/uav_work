#include "practice/Uav.h"
#include "practice/controller.h"
#include "coordination_transfer.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;
#define PI 3.1415927
bool flag = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Controller controller;
    vector<float> uav;
    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    uav1.init();
    uav0.init();
    uav2.init();
    ros::Time last_request = ros::Time::now();
    return 0;
}
