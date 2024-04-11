/*直接執行不需要用rosrun*/
#include<iostream>
#include <fstream>
#include <math.h>
#define PI 3.1415927
using namespace std;

int main(){

    std::ofstream ofs;
    ofs.open("path.txt");
    if (!ofs.is_open()) 
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }
    int angle=0,R=5,cnt=0;
    float x=0,y=0,z=5,i=0.001;
    while(1)
    {
        if(angle!=360)
        {
          ofs<<x<<" "<<y<<" "<<z<< "\n";  
          angle++;
          x=-R*sin(angle*PI/180);
          y=R*cos(angle*PI/180);
          z+=i;
        }
        else
        {
          angle=0;
          cnt++;  
        }
            

        if(cnt==3)
            break;
    }
    
    ofs.close();
return 0;
}
