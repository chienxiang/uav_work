#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;
int main()
{
  float x=-0.37,y=2.3,z=-0.03;
  Vector3f p;   
  Vector3f m(0,0,5);
  p <<x,y,z;
  Matrix3f Frep_v_rotation;
  Frep_v_rotation<< 0,-1,0,1,0,0,0,0,1;
  Matrix3f rotation;
  Matrix3f desireUavPos;
  desireUavPos <<-3/sqrt(3),1.5/sqrt(3),1.5/sqrt(3),0,-1.5,1.5,5,5,5;
  rotation<< cos(240*M_PI/180),-sin(240*M_PI/180),0,sin(240*M_PI/180),cos(240*M_PI/180),0,0,0,1;
  Frep_v_rotation.col(0) << -1.5*sin(90*M_PI/180),1.5*cos(90*M_PI/180),5;
  Frep_v_rotation.col(1) << -1.5*sin(210*M_PI/180),1.5*cos(210*M_PI/180),5;
  Frep_v_rotation.col(2) << -1.5*sin(330*M_PI/180),1.5*cos(330*M_PI/180),5;
  // Matrix3f i,j; 
  // MatrixXf v = MatrixXf::Zero(3,2);
  // for(int i=0;i<3;i++)
  // {
  //   for(int j=0;j<2;j++)
  //   {
  //     v(i,j) =3;
  //   }
  // }
  // v.col(1) << -v.col(0);
  //p<<(v.col(1)-v.col(0))*3;

  // Vector3f j;
  // j<<0,0,0;
  // // float x = (p+m).dot(p-m)/p.norm()*m.norm();
  // // v(0,0) = x;  // cout << acos(p.dot(m)/(p.norm()*m.norm()))*180/M_PI <<endl;
  // cout << rotation<<endl;
  // cout << -3*cos(90*M_PI/180)<<" "<<3*sin(90*M_PI/180)<<endl;
  cout << Frep_v_rotation;
    
} 