#include "practice/controller.h"

std::tuple<float, float> Controller::cal_position_norm(int i,int j,int k) //uav position norm
{
    float np1 = (Uav.col(j) - Uav.col(i)).norm();
    float np2 = (Uav.col(k) - Uav.col(i)).norm();
    return std::make_tuple(np1,np2);
}
std::tuple<float, float> Controller::cal_velocity_norm(int i,int j,int k) //uav velocity norm
{
    float nv1 = (Uav.col(i+3) - Uav.col(j+3)).norm();
    float nv2 = (Uav.col(i+3) - Uav.col(k+3)).norm();
    return std::make_tuple(nv1,nv2);
}
void Controller::data_update(Vector3f u0p ,Vector3f u1p,Vector3f u2p ,Vector3f u0v,Vector3f u1v ,Vector3f u2v)
{
    Uav << u0p,u1p,u2p,u0v,u1v,u2v;
}
std::tuple<float, float> Controller::cacl_delta(int i,int j,int k) //uav delta
{
    float np1,np2,nv1,nv2,dta1,dta2;
    tie(np1,np2) = cal_position_norm(i,j,k);
    tie(nv1,nv2) = cal_velocity_norm(i,j,k);

    dta1 = acos((Uav.col(j) - Uav.col(i)).dot(Uav.col(i+3) - Uav.col(j+3))/np1*nv1)*180/M_PI;
    dta2 = acos((Uav.col(k) - Uav.col(i)).dot(Uav.col(i+3) - Uav.col(k+3))/np1*nv1)*180/M_PI;
    
    return std::make_tuple(dta1,dta2);
}
std::tuple<float, float> Controller::cacl_collision_free_region(int i,int j,int k) //collisoin-free region M
{
    float nv1,nv2,om1,om2;
    tie(nv1,nv2) = cal_velocity_norm(i,j,k);

    om1 = nv1*cos(beta*M_PI/180);
    om2 = nv2*cos(beta*M_PI/180);
    return std::make_tuple(om1,om2);
}
std::tuple<float, float> Controller::cacl_theta1(int i,int j,int k)
{
    float m1x,m1y,m2x,m2y,pmx,pmy,pr,th1,th2;
    tie(m1x,m1y,m2x,m2y) = cacl_pointM(i,j,k);
    pmx = pow(m1x,2);
    pmy = pow(m1y,2);
    pr = pow(rmin,2);
    th1 = atan((m1x*m1y-sqrt(pmx*pmy-((pmx-pr)*(pmy-pr))))/(pmx-pr))*180/M_PI;
    pmx = pow(m2x,2);
    pmy = pow(m2y,2);
    th2 = atan((m2x*m2y-sqrt(pmx*pmy-((pmx-pr)*(pmy-pr))))/(pmx-pr))*180/M_PI;

    return std::make_tuple(th1,th2);
    
}
std::tuple<float, float,float, float> Controller::cacl_pointM(int i,int j,int k) // M點(px+rcos,py+rsin)
{
    float m1x,m1y,m2x,m2y,om1,om2;
    tie(om1,om2) = cacl_collision_free_region(i,j,k);
    m1x = Uav(0,i)+om1*cos(beta*M_PI/180);
    m1y = Uav(1,i)+om1*sin(beta*M_PI/180);
    m2x = Uav(0,i)+om1*cos(beta*M_PI/180);
    m2y = Uav(1,i)+om1*sin(beta*M_PI/180);
   
    return std::make_tuple(m1x,m1y,m2x,m2y);
}
void Controller::safe_range() //uav safety distance
{
    tie(delta[0],delta[1]) = cacl_delta(0,1,2);
    tie(delta[2],delta[3]) = cacl_delta(1,0,2);
    tie(delta[4],delta[5]) = cacl_delta(2,0,1);

    tie(V_norm[0],V_norm[1]) = cal_velocity_norm(0,1,2);
    tie(V_norm[2],V_norm[3]) = cal_velocity_norm(1,0,2);
    tie(V_norm[4],V_norm[5]) = cal_velocity_norm(2,0,1);

    tie(theta[0],theta[1]) = cacl_theta1(0,1,2);
    tie(theta[2],theta[3]) = cacl_theta1(1,0,2);
    tie(theta[4],theta[5]) = cacl_theta1(2,0,1);

    for(int i = 0; i <6;i++)
    {
        if((delta[i] > 0 && delta[i] < beta) || (delta[i] > -beta && delta[i] < 0)) //rvar
        {
            rsafe[i] = kv/(kw+wmax)*V_norm[i]*cos(delta[i]*M_PI/180);
        }
        else if((delta[i] >= beta && delta[i] <= 90+theta[i]) || (delta[i] >= -90-theta[i] && delta[i] < -beta)) //rconnect
        {
            rsafe[i] = V_norm[i]*cos(beta*M_PI/180)*sin((beta-theta[i])*M_PI/180)/sin((M_PI-delta[i]+theta[i])*M_PI/180);
        }
        else
        {
            rsafe[i]=rmin;
        }
    }
}
void Controller::cacl_threat_level()
{
    tie(P_norm[0],P_norm[1]) = cal_position_norm(0,1,2);
    tie(P_norm[2],P_norm[3]) = cal_position_norm(1,0,2);
    tie(P_norm[4],P_norm[5]) = cal_position_norm(2,0,1);

    for(int i=0; i<6;i++)
    {
        if(cos(delta[i]*M_PI/180)>0)
        {
            THlev[i] = (1/P_norm[i]-1/rsafe[i])*V_norm[i]*cos(delta[i]*M_PI/180);
        }
        else
        {
            THlev[i] = 0;
        }
    }
}
void Controller::cacl_uav_state_vector()
{
    Uav_pos.col(0) = Uav.col(1) - Uav.col(0);
    Uav_pos.col(1) = Uav.col(2) - Uav.col(0);
    Uav_pos.col(2) = Uav.col(0) - Uav.col(1);
    Uav_pos.col(3) = Uav.col(2) - Uav.col(1);
    Uav_pos.col(4) = Uav.col(0) - Uav.col(2);
    Uav_pos.col(5) = Uav.col(1) - Uav.col(2);

    Uav_vel.col(0) = Uav.col(3) - Uav.col(4);
    Uav_vel.col(1) = Uav.col(3) - Uav.col(5);
    Uav_vel.col(2) = Uav.col(4) - Uav.col(3);
    Uav_vel.col(3) = Uav.col(4) - Uav.col(5);
    Uav_vel.col(4) = Uav.col(5) - Uav.col(3);
    Uav_vel.col(5) = Uav.col(5) - Uav.col(4);
}
void Controller::repulsive_potential()
{
    MatrixXf Frep_p = MatrixXf::Zero(3,6);
    MatrixXf Frep_v = MatrixXf::Zero(3,6);
     
   for(int i=0; i<6; i++)
    {
        if(P_norm[i]<rsafe[i])
        {
            if((delta[i]>=0 && delta[i] < 90) || (delta[i]>=-90 && delta[i] < 0))
            {
                Frep_p.col(i) << krep_p*THlev[i]*(-Uav_pos.col(i))/P_norm[i];
            }

            else if(delta[i]>=90 && delta[i] <=270 )
            {
                Frep_v.col(i) << krep_v*THlev[i]*((-Uav_pos.col(i))/P_norm[i]*1/cos(delta[i]*M_PI/180)+(Uav_vel.col(i)/V_norm[i]));
            }
            else
            {
                Frep_p.col(i) <<0,0,0;
                Frep_v.col(i) <<0,0,0;
            }
            
        }
        else
        {
            Frep_p.col(i) <<0,0,0;
            Frep_v.col(i) <<0,0,0;
        }

    }

    Frep = Frep_p + Frep_v;

}
std::tuple<float, float> Controller::attractive_potential(int i,int j,int k)
{
    //目標點追蹤
}
void Controller::virtual_vel()
{
    cacl_threat_level();
    safe_range();
    repulsive_potential();
}
void Controller::process()
{

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