#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;
int main()
{
  float x=-0.37,y=2.3,z=-0.03,t=0;
  Vector3f p,m,f;
  Matrix3f j,k;
  p <<-1.73,-0.02,-0.0001;   
  m <<-1.73,-0.876,-0.853;
  f << -1.418,-2.53,-0.03;
  j <<1,2,3,4,5,6,7,8,9;
  k <<0,0,0,0,0,0,0,0,0;

  Matrix3f I = Matrix3f::Identity();


  // cout << (I - p*m.transpose())*f;
  // cout <<j.col(0).transpose();;
  cout << j.col(0)*j.col(0).transpose();

    
} 