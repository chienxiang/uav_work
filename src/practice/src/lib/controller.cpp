#include "practice/controller.h"

void Controller::cal_position_norm()
{
    P_norm(0)= (U1p - U0p).norm(); //10
    P_norm(1)= (U2p - U0p).norm();; //20
    P_norm(2)= (U1p - U2p).norm();; //12

}
void Controller::cal_velocity_norm()
{
    V_norm(0)= (U0v - U1v).norm(); //01
    V_norm(1)= (U0v - U2v).norm(); //02
    V_norm(2)= (U1v - U2v).norm(); //12
}
void Controller::data_update(RowVector3f u0p ,RowVector3f u1p,RowVector3f u2p ,RowVector3f u0v,RowVector3f u1v ,RowVector3f u2v)
{
    U0p = u0p;
    U1p = u1p;
    U2p = u2p;
    U0v = u0v;
    U1v = u1v;
    U2v = u2v;
}
void Controller::cacl_delta()
{
    delta(0) = acos((U1p - U0p).dot(U0v - U1v)/P_norm(0)*V_norm(0)); //01   
    delta(1) = acos((U2p - U0p).dot(U0v - U2v)/P_norm(1)*V_norm(1)); //02
    delta(2) = acos((U0p - U1p).dot(U1v - U0v)/P_norm(0)*V_norm(0)); //10
    delta(3) = acos((U2p - U1p).dot(U1v - U2v)/P_norm(2)*V_norm(2)); //12
    delta(4) = acos((U0p - U2p).dot(U2v - U0v)/P_norm(1)*V_norm(1)); //20
    delta(5) = acos((U1p - U2p).dot(U2v - U1v)/P_norm(2)*V_norm(2)); //21
}
void Controller::cacl_collision_free_region()
{
    OM(0) = V_norm(0)*cos(beta); //01
    OM(1) = V_norm(1)*cos(beta); //02   
    OM(2) = V_norm(2)*cos(beta); //12

}
void Controller::safe_range()
{
    int j=0;
    for(int i = 0; i <6;i++)
    {
        if(i == 0 || i == 1)
            j=0;
        else if(i == 2 || i == 3)
            j=1;
        else
            j=2;

        if((delta(i) > 0 && delta(i) < beta) || (delta(i) > -beta && delta(i) < 0)) //rvar
        {
            rsafe(i) = kv/(kw+wmax)*V_norm(j)*cos(delta(i));
        }
        else if((delta(i) >= beta && delta(i) <= 90+theta) || (delta(i) >= -90-theta && delta(i) < -beta)) //rconnect
        {

        }
        else
        {
            rsafe(i)=rmin;
        }
    }

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