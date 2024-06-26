/*直接執行不需要用rosrun*/
#include<iostream>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>
#define PI 3.1415927
using namespace std;
using namespace Eigen;

int main(){

    std::ofstream ofs;
    ofs.open("path.txt");
    if (!ofs.is_open()) 
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }
    float angle=0,R=5,cnt=0;
    float x=0,y=0,z=5,i=0.001,t=0;
    while(1)
    {
        if(angle!=360)
        {
          ofs<<x<<" "<<y<<" "<<z<< "\n";  
          angle+=0.5;
          x=-R*sin(angle*PI/180);
          y=R*cos(angle*PI/180);
          z+=i;
          t++;
        }
        else
        {
          angle=0;
          cnt++;  
        }
            

        if(cnt==2)
            break;
    }
    
    ofs.close();
return 0;
}