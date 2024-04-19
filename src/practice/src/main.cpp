#include "practice/Uav.h"
#include "practice/controller.h"
#include "coordination_transfer.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
#define PI 3.1415927
bool flag = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Controller controller;
    MatrixXd Uav_pos = MatrixXd::Zero(3,3);
    MatrixXd Uav_vel = MatrixXd::Zero(3,3);
    Uav uav1(1);
    Uav uav0(0);
    Uav uav2(2);
    uav1.init();
    uav0.init();
    uav2.init();
    ros::Time last_request = ros::Time::now();
    uav0.data_update();
    uav1.data_update();
    uav2.data_update();
    
    controller.data_update(uav0.p_data,uav1.p_data,uav2.p_data,uav0.v_data,uav1.v_data,uav2.v_data);
    return 0;
}
