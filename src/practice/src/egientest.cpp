#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;
int main()
{
  RowVector3f p(1,2,3);   
  RowVector3f m(1,1,1);
  RowVectorXf v(6);
  v.fill(0);
  float x = (p+m).dot(p-m)/p.norm()*m.norm();
  v(0,0) = x;
  cout << v <<endl;
    
}