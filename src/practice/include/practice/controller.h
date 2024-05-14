#include <ros/ros.h>
#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <mavros_msgs/CommandTOL.h>
#include <iostream>
#include <vector>
#include <tuple>
#include "practice/information.h"
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
class Controller
{   
    private:
        float rmin = 2,rsafe[6] = {0};;
        float beta=60,delta[6];
        float krep_p=6,krep_v=3;
        Vector3f center_move = Vector3f::Zero();
        Vector3f circumcentre;
        Matrix3f desireUavPos = Matrix3f::Zero();
        Matrix3f ed = Matrix3f::Zero();
        Matrix3f ed1 = Matrix3f::Zero();
        Matrix3f ed2 = Matrix3f::Zero();
        float Kp=0.35,Ki=0.65,Kd=0.005,limitV=3;
    public:
        practice::information Cuav[3],Fuav[3],Nuav[3];
        float aij[3]={1.5,2,1},Ftot[3]={0},F[3]={0},Fcenter[3];

        MatrixXf Uav = MatrixXf::Zero(3,6); //uav position and velocity data
        MatrixXf Uav_pos = MatrixXf::Zero(3,6); //uav position state vector
        MatrixXf Uav_vel = MatrixXf::Zero(3,6); //uav velocity state vector
        Matrix3f Frep = Matrix3f::Zero();    //uav repulsion force
        Matrix3f Fformation = Matrix3f::Zero();
        float V_norm[6],theta[6],P_norm[6],THlev[6];

        void data_update(Vector3f u0p ,Vector3f u1p,Vector3f u2p ,Vector3f u0v,Vector3f u1v ,Vector3f u2v);
        void cacl_delta();
        void cal_velocity_norm();
        void cal_position_norm();
        void cacl_collision_free_region();
        void cacl_uav_state_vector();
        void safe_range();
        void cacl_threat_level();                                           
        void repulsive_potential();
        void attractive_potential();
        void virtual_vel();
        void follower();
        void Uav_Circumcentre_move(float x,float y,float z);
        void Circumcentre_init();
        void process();
        void null_space_behavior();

};
