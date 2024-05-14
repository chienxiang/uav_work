#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;
int main()
{
  float x=-0.37,y=2.3,z=-0.03;
  Vector3f p;   
  Vector3f m(0.46,-0.15,-0.05);
  p <<x,y,z;
  Matrix3f Frep_v_rotation;
  Frep_v_rotation<< 0,-1,0,1,0,0,0,0,1;
  // Matrix3f i,j; 
  MatrixXf v = MatrixXf::Zero(3,2);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<2;j++)
    {
      v(i,j) =3;
    }
  }
  // v.col(1) << -v.col(0);
  //p<<(v.col(1)-v.col(0))*3;
  // Vector3f j;
  // j<<0,0,0;
  // // float x = (p+m).dot(p-m)/p.norm()*m.norm();
  // // v(0,0) = x;
  // cout << acos(p.dot(m)/(p.norm()*m.norm()))*180/M_PI <<endl;
  // cout << Frep_v_rotation*m<<endl;
  cout << v;
    
} 