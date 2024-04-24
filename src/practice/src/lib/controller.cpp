#include "practice/controller.h"

std::tuple<float, float> Controller::cal_position_norm(int i,int j,int k) //uav position norm
{
    float np1 = (Uav.col(j) - Uav.col(i)).norm();
    float np2 = (Uav.col(k) - Uav.col(i)).norm();
    return std::make_tuple(np1,np2);
}
std::tuple<float, float> Controller::cal_velocity_norm(int i,int j,int k) //uav velocity norm
{
    float vp1 = (Uav.col(i+3) - Uav.col(j+3)).norm();
    float vp2 = (Uav.col(i+3) - Uav.col(k+3)).norm();
    return std::make_tuple(vp1,vp2);
}
void Controller::data_update(Vector3f u0p ,Vector3f u1p,Vector3f u2p ,Vector3f u0v,Vector3f u1v ,Vector3f u2v)
{
    Uav << u0p,u1p,u2p,u0v,u1v,u2v;
}
std::tuple<float, float> Controller::cacl_delta(int i,int j,int k) //uav delta
{
    float np1,np2,vp1,vp2,dta1,dta2;
    tie(np1,np2) = cal_position_norm(i,j,k);
    tie(vp1,vp2) = cal_velocity_norm(i,j,k);

    dta1 = acos((Uav.col(j) - Uav.col(i)).dot(Uav.col(i+3) - Uav.col(j+3))/np1*vp1);
    dta2 = acos((Uav.col(k) - Uav.col(i)).dot(Uav.col(i+3) - Uav.col(k+3))/np1*vp1);
    
    return std::make_tuple(dta1,dta2);
}
std::tuple<float, float> Controller::cacl_collision_free_region(int i,int j,int k) //collisoin-free region M
{
    float vp1,vp2,om1,om2;
    tie(vp1,vp2) = cal_velocity_norm(i,j,k);

    om1 = vp1*cos(beta);
    om2 = vp2*cos(beta);
    return std::make_tuple(om1,om2);
}
std::tuple<float, float> Controller::cacl_theta1(int i,int j,int k)
{
    float m1x,m1y,m2x,m2y,pmx,pmy,pr,th1,th2;
    tie(m1x,m1y,m2x,m2y) = cacl_pointM(i,j,k);
    pmx = pow(m1x,2);
    pmy = pow(m1y,2);
    pr = pow(rmin,2);
    th1 = atan((m1x*m1y-sqrt(pmx*pmy-((pmx-pr)*(pmy-pr))))/(pmx-pr));
    pmx = pow(m2x,2);
    pmy = pow(m2y,2);
    th2 = atan((m2x*m2y-sqrt(pmx*pmy-((pmx-pr)*(pmy-pr))))/(pmx-pr));

    return std::make_tuple(th1,th2);
    
}
std::tuple<float, float,float, float> Controller::cacl_pointM(int i,int j,int k) // M點(px+rcos,py+rsin)
{
    float m1x,m1y,m2x,m2y,om1,om2;
    tie(om1,om2) = cacl_collision_free_region(i,j,k);
    m1x = Uav(0,i)+om1*cos(beta);
    m1y = Uav(1,i)+om1*sin(beta);
    m2x = Uav(0,i)+om1*cos(beta);
    m2y = Uav(1,i)+om1*sin(beta);
   
    return std::make_tuple(m1x,m1y,m2x,m2y);
}
void Controller::safe_range() //uav safety distance
{

    
    // int j=0;
    // for(int i = 0; i <6;i++)
    // {
    //     if(i == 0 || i == 1)
    //         j=0;
    //     else if(i == 2 || i == 3)
    //         j=1;
    //     else
    //         j=2;

    //     if((delta(i) > 0 && delta(i) < beta) || (delta(i) > -beta && delta(i) < 0)) //rvar
    //     {
    //         rsafe(i) = kv/(kw+wmax)*V_norm(j)*cos(delta(i));
    //     }
    //     else if((delta(i) >= beta && delta(i) <= 90+theta) || (delta(i) >= -90-theta && delta(i) < -beta)) //rconnect
    //     {

    //     }
    //     else
    //     {
    //         rsafe(i)=rmin;
    //     }
    // }

}
float Controller::cacl_threat_level(practice::information t1 ,practice::information t2)
{
    
}
float Controller::repulsive_potential(float dis)
{

}
float Controller::attractive_potential(float dis)
{

}
void Controller::virtual_vel()
{
    
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
// void Controller::scale_down()
// {
//     float scale=1;
//     for(int i=0;i<3;i++)
//     {
//         if(Cuav[i].vel.x > max_v || Cuav[i].vel.y > max_v || Cuav[i].vel.z > max_v)
//         {
//             scale = max_vel(Cuav[i].vel.x,Cuav[i].vel.y,Cuav[i].vel.z);               
//         }
//         Cuav[i].vel.x = Cuav[i].vel.x/scale;
//         Cuav[i].vel.y = Cuav[i].vel.y/scale;
//         Cuav[i].vel.z = Cuav[i].vel.z/scale;
//     } 
// }
void Controller::process()
{
    // position_norm[0] = discal(Cuav[0],Cuav[1]);//12
    // position_norm[1] = discal(Cuav[0],Cuav[2]);//13
    // position_norm[2] = discal(Cuav[1],Cuav[2]);//23
    // vel_norm[0] = 
    // vel_norm[1]
    // vel_norm[2]
    // uav_delta[0] = cacl_delta(Cuav[0],Cuav[1]); //12
    // uav_delta[1] = cacl_delta(Cuav[1],Cuav[0]); //21
    // uav_delta[2] = cacl_delta(Cuav[1],Cuav[2]); //13
    // uav_delta[3] = cacl_delta(Cuav[2],Cuav[1]); //31
    // uav_delta[4] = cacl_delta(Cuav[1],Cuav[2]); //23
    // uav_delta[5] = cacl_delta(Cuav[2],Cuav[1]); //32
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
void Controller::null_space_behavior()
{
    // process();
    // follower();
    // float P[3][3],N1[3][3],N2[3][3],N3[3][3];
    // P[0][0] = (Nuav[0].point.x-Nuav[1].point.x)/uav_dis[0] + (Nuav[0].point.x-Nuav[2].point.x)/uav_dis[1];
    // P[0][1] = (Nuav[0].point.y-Nuav[1].point.y)/uav_dis[0] + (Nuav[0].point.y-Nuav[2].point.y)/uav_dis[1];
    // P[0][2] = (Nuav[0].point.z-Nuav[1].point.z)/uav_dis[0] + (Nuav[0].point.z-Nuav[2].point.z)/uav_dis[1];

    // P[1][0] = (Nuav[1].point.x-Nuav[0].point.x)/uav_dis[0] + (Nuav[1].point.x-Nuav[2].point.x)/uav_dis[2];
    // P[1][1] = (Nuav[1].point.y-Nuav[0].point.y)/uav_dis[0] + (Nuav[1].point.y-Nuav[2].point.y)/uav_dis[2];
    // P[1][2] = (Nuav[1].point.z-Nuav[0].point.z)/uav_dis[0] + (Nuav[1].point.z-Nuav[2].point.z)/uav_dis[2];

    // P[2][0] = (Nuav[2].point.x-Nuav[0].point.x)/uav_dis[1] + (Nuav[2].point.x-Nuav[1].point.x)/uav_dis[2];
    // P[2][1] = (Nuav[2].point.y-Nuav[0].point.y)/uav_dis[1] + (Nuav[2].point.y-Nuav[1].point.y)/uav_dis[2];
    // P[2][2] = (Nuav[2].point.z-Nuav[0].point.z)/uav_dis[1] + (Nuav[2].point.z-Nuav[1].point.z)/uav_dis[2];

    // N1[0][0]=1-P[0][0]*P[0][0];
    // N1[0][1]=P[0][0]*P[0][1];
    // N1[0][2]=P[0][0]*P[0][2];
    // N1[1][0]=P[0][0]*P[0][1];
    // N1[1][1]=1-P[0][1]*P[0][1];
    // N1[1][2]=P[0][1]*P[0][2];
    // N1[2][0]=P[0][0]*P[0][2];
    // N1[2][1]=P[0][1]*P[0][2];
    // N1[2][2]=1-P[0][2]*P[0][2];

    // N2[0][0]=1-P[1][0]*P[1][0];
    // N2[0][1]=P[1][0]*P[1][1];
    // N2[0][2]=P[1][0]*P[1][2];
    // N2[1][0]=P[1][0]*P[1][1];
    // N2[1][1]=1-P[1][1]*P[1][1];
    // N2[1][2]=P[1][1]*P[1][2];
    // N2[2][0]=P[1][0]*P[1][2];
    // N2[2][1]=P[1][1]*P[1][2];
    // N2[2][2]=1-P[1][2]*P[1][2];

    // N3[0][0]=1-P[2][0]*P[2][0];
    // N3[0][1]=P[2][0]*P[2][1];
    // N3[0][2]=P[2][0]*P[2][2];
    // N3[1][0]=P[2][0]*P[2][1];
    // N3[1][1]=1-P[2][1]*P[2][1];
    // N3[1][2]=P[2][1]*P[2][2];
    // N3[2][0]=P[2][0]*P[2][2];
    // N3[2][1]=P[2][1]*P[2][2];
    // N3[2][2]=1-P[2][2]*P[2][2];

    // Nuav[0].vel.x = Cuav[0].vel.x+Fuav[0].vel.x*(N1[0][0]+N1[0][1]+N1[0][2]);
    // Nuav[0].vel.y = Cuav[0].vel.y+Fuav[0].vel.y*(N1[1][0]+N1[1][1]+N1[1][2]);
    // Nuav[0].vel.z = Cuav[0].vel.z+Fuav[0].vel.z*(N1[2][0]+N1[2][1]+N1[2][2]);

    // Nuav[1].vel.x = Cuav[1].vel.x+Fuav[1].vel.x*(N2[0][0]+N2[0][1]+N2[0][2]);
    // Nuav[1].vel.y = Cuav[1].vel.y+Fuav[1].vel.y*(N2[1][0]+N2[1][1]+N2[1][2]);
    // Nuav[1].vel.z = Cuav[1].vel.z+Fuav[1].vel.z*(N2[2][0]+N2[2][1]+N2[2][2]);

    // Nuav[2].vel.x = Cuav[2].vel.x+Fuav[2].vel.x*(N3[0][0]+N3[0][1]+N3[0][2]);
    // Nuav[2].vel.y = Cuav[2].vel.y+Fuav[2].vel.y*(N3[1][0]+N3[1][1]+N3[1][2]);
    // Nuav[2].vel.z = Cuav[2].vel.z+Fuav[2].vel.z*(N3[2][0]+N3[2][1]+N3[2][2]);
}