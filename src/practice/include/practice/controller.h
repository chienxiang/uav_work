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
// class point {
//     public:
//         float x, y, z;
//         point() {} // default constructor
//         point (float _x, float _y,float _z) {
//             x = _x;
//             y = _y;
//             z = _z;
//         }
// };
class Controller
{   
    private:
        float rmin = 1,kv=10,kw=15,wmax=5,rsafe[6] = {0};;
        float beta=60,delta[6];
        float krep_p,krep_v;
        float Kp=0.35,Ki=0.65,Kd=0.005,ed[3][3]={0},ed1[3][3]={0},ed2[3][3]={0},limitV=3;
    public:
        practice::information Cuav[3],Fuav[3],Nuav[3];
        float aij[3]={1.5,2,1},Ftot[3]={0},F[3]={0},Fcenter[3]={0,0,5};
        float duav0[3]={-3/sqrt(3),0,5},duav1[3]={1.5/sqrt(3),-1.5,5},duav2[3]={1.5/sqrt(3),1.5,5};
        float deltax,deltay,deltaz;

        MatrixXf Uav = MatrixXf::Zero(3,6); //uav position and velocity data
        MatrixXf Uav_pos = MatrixXf::Zero(3,6); //uav position state vector
        MatrixXf Uav_vel = MatrixXf::Zero(3,6); //uav velocity state vector
        MatrixXf Frep = MatrixXf::Zero(3,6);    //uav repulsion force
        float V_norm[6],theta[6],P_norm[6],THlev[6];

        float kij=0.3;
        void data_update(Vector3f u0p ,Vector3f u1p,Vector3f u2p ,Vector3f u0v,Vector3f u1v ,Vector3f u2v);
        tuple<float, float> cacl_delta(int i,int j,int k);
        tuple<float, float> cal_velocity_norm(int i,int j,int k);
        tuple<float, float> cal_position_norm(int i,int j,int k);
        tuple<float, float> cacl_collision_free_region(int i,int j,int k);
        tuple<float, float> cacl_theta1(int i,int j,int k);
        tuple<float, float,float, float> cacl_pointM(int i,int j,int k);
        void cacl_uav_state_vector();
        void safe_range();
        void cacl_threat_level();                                           
        void repulsive_potential();
        tuple<float, float> attractive_potential(int i,int j,int k);
        void virtual_vel();
        void follower();
        void cal_desire_point();
        void update_center(float x,float y,float z);
        void process();
        void null_space_behavior();

};
