#include <ros/ros.h>
#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <mavros_msgs/CommandTOL.h>
#include <iostream>
#include "practice/information.h"
using namespace std;
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
        float uav_dis_min = 1,pre_dis = 3,r1 = 1.5,r2 = 4,uav_dis_max=8,max_v=1;
        float Kp=0.35,Ki=0.65,Kd=0.005,ed[3][3]={0},ed1[3][3]={0},ed2[3][3]={0},limitV=3;
    public:
        practice::information Cuav[3],Fuav[3],Nuav[3];
        vector<float> Cuav,Fuav,Nuav;
        float uav_dis[3],aij[3]={1.5,2,1},Ftot[3]={0},F[3]={0},Fcenter[3]={0,0,5};
        float duav0[3]={-3/sqrt(3),0,5},duav1[3]={1.5/sqrt(3),-1.5,5},duav2[3]={1.5/sqrt(3),1.5,5};
        float deltax,deltay,deltaz;
        float kij=0.3;
        int k=0;
        float discal(practice::information d1 , \
                     practice::information d2);
        void data_update(practice::information s1, \
                         practice::information s2, \
                         practice::information s3);
        void cacl_k();
        float repulsive_potential(float dis);
        float attractive_potential(float dis);
        float exact_range(float dis);
        void virtual_vel();
        void scale_down();
        float max_vel(float a1,float a2,float a3);
        void follower();
        void cal_desire_point();
        void update_center(float x,float y,float z);
        void process();
        void null_space_behavior();

};
