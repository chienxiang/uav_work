#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;
int main()
{
  float x=1,y=2,z=3;
  Vector3f p;   
  Vector3f m(3,3,0);
  p <<x,y,z; 
  MatrixXf v = MatrixXf::Zero(3,2);

  v.col(0) << p-m;
  // Vector3f j;
  // j<<0,0,0;
  // // float x = (p+m).dot(p-m)/p.norm()*m.norm();
  // // v(0,0) = x;
  cout << v <<endl;
  // cout << acos(1.0)*180/M_PI;
    
} 