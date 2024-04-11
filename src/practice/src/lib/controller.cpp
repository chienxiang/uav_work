#include "practice/controller.h"

float Controller::discal(practice::information d1 ,practice::information d2)
{
    float dis = sqrt(pow(d1.point.x - d2.point.x ,2) + \
                     pow(d1.point.y - d2.point.y ,2) + \
                     pow(d1.point.z - d2.point.z ,2));
    return dis;
}
void Controller::data_update(practice::information s1 , \
                             practice::information s2 , \
                             practice::information s3)
{
    Nuav[0].point.x = Fuav[0].point.x = Cuav[0].point.x = s1.point.x;
    Nuav[0].point.y = Fuav[0].point.y = Cuav[0].point.y = s1.point.y;
    Nuav[0].point.z = Fuav[0].point.z = Cuav[0].point.z = s1.point.z;
    Nuav[0].vel.x = Fuav[0].vel.x = Cuav[0].vel.x = s1.vel.x;
    Nuav[0].vel.y = Fuav[0].vel.y = Cuav[0].vel.y = s1.vel.y;
    Nuav[0].vel.z = Fuav[0].vel.z = Cuav[0].vel.z = s1.vel.z;

    Nuav[1].point.x = Fuav[1].point.x = Cuav[1].point.x = s2.point.x;
    Nuav[1].point.y = Fuav[1].point.y = Cuav[1].point.y = s2.point.y;
    Nuav[1].point.z = Fuav[1].point.z = Cuav[1].point.z = s2.point.z;
    Nuav[1].vel.x = Fuav[1].vel.x = Cuav[1].vel.x = s2.vel.x;
    Nuav[1].vel.y = Fuav[1].vel.y = Cuav[1].vel.y = s2.vel.y;
    Nuav[1].vel.z = Fuav[1].vel.z = Cuav[1].vel.z = s2.vel.z;

    Nuav[2].point.x = Fuav[2].point.x = Cuav[2].point.x = s3.point.x;
    Nuav[2].point.y = Fuav[2].point.y = Cuav[2].point.y = s3.point.y;
    Nuav[2].point.z = Fuav[2].point.z = Cuav[2].point.z = s3.point.z;
    Nuav[2].vel.x = Fuav[2].vel.x = Cuav[2].vel.x = s3.vel.x;
    Nuav[2].vel.y = Fuav[2].vel.y = Cuav[2].vel.y = s3.vel.y;
    Nuav[2].vel.z = Fuav[2].vel.z = Cuav[2].vel.z = s3.vel.z;

}
void Controller::cacl_k()
{
    kij = (1/pre_dis)*(1/pow(exp(pre_dis)-exp(uav_dis_min),2)*exp(pre_dis));
}
float Controller::exact_range(float dis)
{
    float ftot;
    if(dis>uav_dis_min && dis<r1) //req       
    {
        ftot = repulsive_potential(dis);
    }
    else if(dis>=r1 && dis<=r2)//balance
    {
        ftot = 0;
    }
    else if(dis>r2 && dis<uav_dis_max)//att
    {            
        ftot = attractive_potential(dis);
    }
    else 
    {
        ftot = 0;
    }
    return ftot;
}
float Controller::repulsive_potential(float dis)
{
    float freq = (1/pow(exp(dis)-exp(uav_dis_min),2)*exp(dis));
    return freq;
}
float Controller::attractive_potential(float dis)
{
    float fatt = -kij*dis;
    return fatt;
}
void Controller::virtual_vel()
{
    // //u1=u12+u13
    // Cuav[0].vel.x = aij[1]*F[0]*(Cuav[0].point.x - Cuav[1].point.x)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.x - Cuav[2].point.x)/uav_dis[1] + Cuav[0].vel.x; 
    // Cuav[0].vel.y = aij[1]*F[0]*(Cuav[0].point.y - Cuav[1].point.y)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.y - Cuav[2].point.y)/uav_dis[1] + Cuav[0].vel.y;
    // Cuav[0].vel.z = aij[1]*F[0]*(Cuav[0].point.z - Cuav[1].point.z)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.z - Cuav[2].point.z)/uav_dis[1] + Cuav[0].vel.z;
    // //u2=u21+u23
    // Cuav[1].vel.x = aij[0]*F[0]*(Cuav[1].point.x - Cuav[0].point.x)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.x - Cuav[2].point.x)/uav_dis[2] + Cuav[1].vel.x; 
    // Cuav[1].vel.y = aij[0]*F[0]*(Cuav[1].point.y - Cuav[0].point.y)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.y - Cuav[2].point.y)/uav_dis[2] + Cuav[1].vel.y;
    // Cuav[1].vel.z = aij[0]*F[0]*(Cuav[1].point.z - Cuav[0].point.z)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.z - Cuav[2].point.z)/uav_dis[2] + Cuav[1].vel.z;

    // //u3=u31+u32
    // Cuav[2].vel.x = aij[0]*F[1]*(Cuav[2].point.x - Cuav[0].point.x)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.x - Cuav[1].point.x)/uav_dis[2] + Cuav[2].vel.x; 
    // Cuav[2].vel.y = aij[0]*F[1]*(Cuav[2].point.y - Cuav[0].point.y)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.y - Cuav[1].point.y)/uav_dis[2] + Cuav[2].vel.y;
    // Cuav[2].vel.z = aij[0]*F[1]*(Cuav[2].point.z - Cuav[0].point.z)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.z - Cuav[1].point.z)/uav_dis[2] + Cuav[2].vel.z;

    //u1=u12+u13
    Cuav[0].vel.x = aij[1]*F[0]*(Cuav[0].point.x - Cuav[1].point.x)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.x - Cuav[2].point.x)/uav_dis[1]; 
    Cuav[0].vel.y = aij[1]*F[0]*(Cuav[0].point.y - Cuav[1].point.y)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.y - Cuav[2].point.y)/uav_dis[1];
    Cuav[0].vel.z = aij[1]*F[0]*(Cuav[0].point.z - Cuav[1].point.z)/uav_dis[0] + aij[1]*F[1]*(Cuav[0].point.z - Cuav[2].point.z)/uav_dis[1];
    //u2=u21+u23
    Cuav[1].vel.x = aij[0]*F[0]*(Cuav[1].point.x - Cuav[0].point.x)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.x - Cuav[2].point.x)/uav_dis[2]; 
    Cuav[1].vel.y = aij[0]*F[0]*(Cuav[1].point.y - Cuav[0].point.y)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.y - Cuav[2].point.y)/uav_dis[2];
    Cuav[1].vel.z = aij[0]*F[0]*(Cuav[1].point.z - Cuav[0].point.z)/uav_dis[0] + aij[2]*F[2]*(Cuav[1].point.z - Cuav[2].point.z)/uav_dis[2];

    //u3=u31+u32
    Cuav[2].vel.x = aij[0]*F[1]*(Cuav[2].point.x - Cuav[0].point.x)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.x - Cuav[1].point.x)/uav_dis[2]; 
    Cuav[2].vel.y = aij[0]*F[1]*(Cuav[2].point.y - Cuav[0].point.y)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.y - Cuav[1].point.y)/uav_dis[2];
    Cuav[2].vel.z = aij[0]*F[1]*(Cuav[2].point.z - Cuav[0].point.z)/uav_dis[1] + aij[2]*F[2]*(Cuav[2].point.z - Cuav[1].point.z)/uav_dis[2];
}
float Controller::max_vel(float a1,float a2,float a3)
{
    float max=1,m[3]={abs(a1),abs(a2),abs(a3)};
    for(int i=0;i<3;i++)
    {
       if(m[i]>max)
       {
         max=m[i];
       } 
    }
    return max; 
}
void Controller::scale_down()
{
    float scale=1;
    for(int i=0;i<3;i++)
    {
        if(Cuav[i].vel.x > max_v || Cuav[i].vel.y > max_v || Cuav[i].vel.z > max_v)
        {
            scale = max_vel(Cuav[i].vel.x,Cuav[i].vel.y,Cuav[i].vel.z);               
        }
        Cuav[i].vel.x = Cuav[i].vel.x/scale;
        Cuav[i].vel.y = Cuav[i].vel.y/scale;
        Cuav[i].vel.z = Cuav[i].vel.z/scale;
    } 
}
void Controller::update_center(float x,float y,float z)
{
    deltax=x-Fcenter[0];
    deltay=y-Fcenter[1];
    deltaz=z-Fcenter[2];

    Fcenter[0]=x;
    Fcenter[1]=y;
    Fcenter[2]=z;
}
void Controller::follower()
{   
    cal_desire_point();
    float tmp[3];
    /*uav0 pid*/
    ed[0][0] = duav0[0]-Fuav[0].point.x; //計算當前誤差
    ed[0][1] = duav0[1]-Fuav[0].point.y;
    ed[0][2] = duav0[2]-Fuav[0].point.z;

    tmp[0] = Kp*(ed[0][0]-ed1[0][0]) + Ki*ed[0][0] + Kd*(ed[0][0]-2*ed1[0][0]+ed2[0][0]); //pid increase
    tmp[1] = Kp*(ed[0][1]-ed1[0][1]) + Ki*ed[0][1] + Kd*(ed[0][1]-2*ed1[0][1]+ed2[0][1]);
    tmp[2] = Kp*(ed[0][2]-ed1[0][2]) + Ki*ed[0][2] + Kd*(ed[0][2]-2*ed1[0][2]+ed2[0][2]);

    ed2[0][0]=ed1[0][0]; //保存上上次偏差
    ed2[0][1]=ed1[0][1];
    ed2[0][2]=ed1[0][2];

    ed1[0][0]=ed[0][0]; //保存上次偏差
    ed1[0][1]=ed[0][1];
    ed1[0][2]=ed[0][2];
    // cout<<"edz="<<ed[0][2]<<endl; 
    for(int i=0;i<3;i++)
    {
        if(abs(tmp[i])>limitV && tmp[i]<0)
        {
            tmp[i]=-limitV;
        }
        else if(abs(tmp[i])>limitV && tmp[i]>0)
        {
           tmp[i]=limitV; 
        }    
    }

    Fuav[0].vel.x = tmp[0];
    Fuav[0].vel.y = tmp[1];
    Fuav[0].vel.z = tmp[2];
    
    /*uav1 pid*/
    ed[1][0] = duav1[0]-Fuav[1].point.x;
    ed[1][1] = duav1[1]-Fuav[1].point.y;
    ed[1][2] = duav1[2]-Fuav[1].point.z;

    tmp[0] = Kp*(ed[1][0]-ed1[1][0]) + Ki*ed[1][0] + Kd*(ed[1][0]-2*ed1[1][0]+ed2[1][0]);
    tmp[1] = Kp*(ed[1][1]-ed1[1][1]) + Ki*ed[1][1] + Kd*(ed[1][1]-2*ed1[1][1]+ed2[1][1]);
    tmp[2] = Kp*(ed[1][2]-ed1[1][2]) + Ki*ed[1][2] + Kd*(ed[1][2]-2*ed1[1][2]+ed2[1][2]);

    ed2[1][0]=ed1[1][0];
    ed2[1][1]=ed1[1][1];
    ed2[1][2]=ed1[1][2];

    ed1[1][0]=ed[1][0];
    ed1[1][1]=ed[1][1];
    ed1[1][2]=ed[1][2];
    for(int i=0;i<3;i++)
    {
        if(abs(tmp[i])>limitV && tmp[i]<0)
        {
            tmp[i]=-limitV;
        }
        else if(abs(tmp[i])>limitV && tmp[i]>0)
        {
           tmp[i]=limitV; 
        }
    }
    Fuav[1].vel.x = tmp[0];
    Fuav[1].vel.y = tmp[1];
    Fuav[1].vel.z = tmp[2];
    /*uav2 pid*/
    ed[2][0] = duav2[0]-Fuav[2].point.x;
    ed[2][1] = duav2[1]-Fuav[2].point.y;
    ed[2][2] = duav2[2]-Fuav[2].point.z;

    tmp[0] = Kp*(ed[2][0]-ed1[2][0]) + Ki*ed[2][0] + Kd*(ed[2][0]-2*ed1[2][0]+ed2[2][0]);
    tmp[1] = Kp*(ed[2][1]-ed1[2][1]) + Ki*ed[2][1] + Kd*(ed[2][1]-2*ed1[2][1]+ed2[2][1]);
    tmp[2] = Kp*(ed[2][2]-ed1[2][2]) + Ki*ed[2][2] + Kd*(ed[2][2]-2*ed1[2][2]+ed2[2][2]);

    ed2[2][0]=ed1[2][0];
    ed2[2][1]=ed1[2][1];
    ed2[2][2]=ed1[2][2];

    ed1[2][0]=ed[2][0];
    ed1[2][1]=ed[2][1];
    ed1[2][2]=ed[2][2];
    for(int i=0;i<3;i++)
    {
        if(abs(tmp[i])>limitV && tmp[i]<0)
        {
            tmp[i]=-limitV;
        }
        else if(abs(tmp[i])>limitV && tmp[i]>0)
        {
           tmp[i]=limitV; 
        }
    }
    Fuav[2].vel.x = tmp[0];
    Fuav[2].vel.y = tmp[1];
    Fuav[2].vel.z = tmp[2];
}
void Controller::cal_desire_point()
{
    duav0[0] += deltax;
    duav0[1] += deltay;
    duav0[2] += deltaz;

    duav1[0] += deltax;
    duav1[1] += deltay;
    duav1[2] += deltaz;

    duav2[0] += deltax;
    duav2[1] += deltay;
    duav2[2] += deltaz;   
}  
void Controller::process()
{
    uav_dis[0] = discal(Cuav[0],Cuav[1]);//12
    uav_dis[1] = discal(Cuav[0],Cuav[2]);//13
    uav_dis[2] = discal(Cuav[1],Cuav[2]);//23
    for(int i=0;i<3;i++)
    {
        F[i] = exact_range(uav_dis[i]);
    }
    virtual_vel();
}
void Controller::null_space_behavior()
{
    process();
    follower();
    float P[3][3],N1[3][3],N2[3][3],N3[3][3];
    P[0][0] = (Nuav[0].point.x-Nuav[1].point.x)/uav_dis[0] + (Nuav[0].point.x-Nuav[2].point.x)/uav_dis[1];
    P[0][1] = (Nuav[0].point.y-Nuav[1].point.y)/uav_dis[0] + (Nuav[0].point.y-Nuav[2].point.y)/uav_dis[1];
    P[0][2] = (Nuav[0].point.z-Nuav[1].point.z)/uav_dis[0] + (Nuav[0].point.z-Nuav[2].point.z)/uav_dis[1];

    P[1][0] = (Nuav[1].point.x-Nuav[0].point.x)/uav_dis[0] + (Nuav[1].point.x-Nuav[2].point.x)/uav_dis[2];
    P[1][1] = (Nuav[1].point.y-Nuav[0].point.y)/uav_dis[0] + (Nuav[1].point.y-Nuav[2].point.y)/uav_dis[2];
    P[1][2] = (Nuav[1].point.z-Nuav[0].point.z)/uav_dis[0] + (Nuav[1].point.z-Nuav[2].point.z)/uav_dis[2];

    P[2][0] = (Nuav[2].point.x-Nuav[0].point.x)/uav_dis[1] + (Nuav[2].point.x-Nuav[1].point.x)/uav_dis[2];
    P[2][1] = (Nuav[2].point.y-Nuav[0].point.y)/uav_dis[1] + (Nuav[2].point.y-Nuav[1].point.y)/uav_dis[2];
    P[2][2] = (Nuav[2].point.z-Nuav[0].point.z)/uav_dis[1] + (Nuav[2].point.z-Nuav[1].point.z)/uav_dis[2];

    N1[0][0]=1-P[0][0]*P[0][0];
    N1[0][1]=P[0][0]*P[0][1];
    N1[0][2]=P[0][0]*P[0][2];
    N1[1][0]=P[0][0]*P[0][1];
    N1[1][1]=1-P[0][1]*P[0][1];
    N1[1][2]=P[0][1]*P[0][2];
    N1[2][0]=P[0][0]*P[0][2];
    N1[2][1]=P[0][1]*P[0][2];
    N1[2][2]=1-P[0][2]*P[0][2];

    N2[0][0]=1-P[1][0]*P[1][0];
    N2[0][1]=P[1][0]*P[1][1];
    N2[0][2]=P[1][0]*P[1][2];
    N2[1][0]=P[1][0]*P[1][1];
    N2[1][1]=1-P[1][1]*P[1][1];
    N2[1][2]=P[1][1]*P[1][2];
    N2[2][0]=P[1][0]*P[1][2];
    N2[2][1]=P[1][1]*P[1][2];
    N2[2][2]=1-P[1][2]*P[1][2];

    N3[0][0]=1-P[2][0]*P[2][0];
    N3[0][1]=P[2][0]*P[2][1];
    N3[0][2]=P[2][0]*P[2][2];
    N3[1][0]=P[2][0]*P[2][1];
    N3[1][1]=1-P[2][1]*P[2][1];
    N3[1][2]=P[2][1]*P[2][2];
    N3[2][0]=P[2][0]*P[2][2];
    N3[2][1]=P[2][1]*P[2][2];
    N3[2][2]=1-P[2][2]*P[2][2];

    Nuav[0].vel.x = Cuav[0].vel.x+Fuav[0].vel.x*(N1[0][0]+N1[0][1]+N1[0][2]);
    Nuav[0].vel.y = Cuav[0].vel.y+Fuav[0].vel.y*(N1[1][0]+N1[1][1]+N1[1][2]);
    Nuav[0].vel.z = Cuav[0].vel.z+Fuav[0].vel.z*(N1[2][0]+N1[2][1]+N1[2][2]);

    Nuav[1].vel.x = Cuav[1].vel.x+Fuav[1].vel.x*(N2[0][0]+N2[0][1]+N2[0][2]);
    Nuav[1].vel.y = Cuav[1].vel.y+Fuav[1].vel.y*(N2[1][0]+N2[1][1]+N2[1][2]);
    Nuav[1].vel.z = Cuav[1].vel.z+Fuav[1].vel.z*(N2[2][0]+N2[2][1]+N2[2][2]);

    Nuav[2].vel.x = Cuav[2].vel.x+Fuav[2].vel.x*(N3[0][0]+N3[0][1]+N3[0][2]);
    Nuav[2].vel.y = Cuav[2].vel.y+Fuav[2].vel.y*(N3[1][0]+N3[1][1]+N3[1][2]);
    Nuav[2].vel.z = Cuav[2].vel.z+Fuav[2].vel.z*(N3[2][0]+N3[2][1]+N3[2][2]);
}