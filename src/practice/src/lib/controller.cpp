#include "practice/controller.h"

void Controller::cal_position_norm() //uav position norm
{
    for(int i=0;i<6;i++)
    {
        P_norm[i] = (Uav_pos.col(i)).norm();
    }
}
void Controller::cal_velocity_norm() //uav velocity norm
{
    for(int i=0;i<6;i++)
    {
        V_norm[i] = (Uav_vel.col(i)).norm();
    }
}
void Controller::data_update(Vector3f u0p ,Vector3f u1p,Vector3f u2p ,Vector3f u0v,Vector3f u1v ,Vector3f u2v)
{
    Uav << u0p,u1p,u2p,u0v,u1v,u2v;
}
void Controller::cacl_delta() //uav delta
{
    for(int i=0;i<6;i++)
    {
        delta[i] = acos((Uav_pos.col(i)).dot(Uav_vel.col(i))/(P_norm[i]*V_norm[i]))*180/M_PI;
        // cout <<"(V"<<i<<")" <<Uav_vel.col(i) << endl;
        // cout <<"(P"<<i<<")" <<Uav_pos.col(i) << endl;
        // cout <<"(delta"<<i<<")" <<delta[i] << endl;
    }
}
void Controller::cacl_collision_free_region() //collisoin-free region M
{
    float om[6]={0},Mx[6],My[6],powmx[6],powmy[6],pr;
    int j=0;
    pr = pow(rmin,2);
    for(int i=0;i<6;i++)
    {
        if(i == 0 || i == 1){j=0;}
        else if(i == 2 || i == 3){j=1;}
        else {j=2;}
        om[i] = V_norm[i]*cos(beta*M_PI/180);
        Mx[i] = Uav(0,j)+om[i]*cos(beta*M_PI/180);
        My[i] = Uav(1,j)+om[i]*sin(beta*M_PI/180);
        powmx[i] = pow(Mx[i],2);
        powmy[i] = pow(My[i],2);
        theta[i] = atan((powmx[i]*powmy[i]-sqrt(powmx[i]*powmy[i]-((powmx[i]-pr)*(powmy[i]-pr))))/(powmx[i]-pr))*180/M_PI;
    }
}
void Controller::safe_range() //uav safety distance
{
    cal_position_norm();
    cal_velocity_norm();
    cacl_delta();
    cacl_collision_free_region();
    for(int i = 0; i <6;i++)
    {
        if(cos(delta[i]*M_PI/180)>0)
        {
            if((delta[i] > 0 && delta[i] < beta) || (delta[i] > -beta && delta[i] < 0)) //rvar
            {
                rsafe[i] = V_norm[i]*cos(delta[i]*M_PI/180)+rmin;
            }
            else if((delta[i] >= beta && delta[i] <= 90+theta[i]) || (delta[i] >= -90-theta[i] && delta[i] < -beta)) //rconnect
            {
                rsafe[i] = (rmin+V_norm[i]*cos(beta*M_PI/180))*sin((beta-theta[i])*M_PI/180)/sin((M_PI-delta[i]+theta[i])*M_PI/180);
            }
        }
        else
        {
            rsafe[i]=rmin;
        }
    }
}
void Controller::cacl_threat_level()
{
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

    Uav_vel.col(0) = Uav.col(3) - Uav.col(4);   //0-1
    Uav_vel.col(1) = Uav.col(3) - Uav.col(5);   //0-2
    Uav_vel.col(2) = Uav.col(4) - Uav.col(3);   //1-0
    Uav_vel.col(3) = Uav.col(4) - Uav.col(5);   //1-2
    Uav_vel.col(4) = Uav.col(5) - Uav.col(3);   //2-0
    Uav_vel.col(5) = Uav.col(5) - Uav.col(4);   //2-1

}
void Controller::repulsive_potential()
{
    MatrixXf Frep_p = MatrixXf::Zero(3,6);
    MatrixXf Frep_v = MatrixXf::Zero(3,6);
    Matrix3f Frep_v_rotation;
    Frep_v_rotation<< 0,-1,0,1,0,0,0,0,1;
     
   for(int i=0; i<6; i++)
    {
        if(P_norm[i]<rsafe[i])
        {
            if((delta[i]>=0 && delta[i] < 90) || (delta[i]>=-90 && delta[i] < 0))
            {
                Frep_p.col(i) << krep_p*THlev[i]*(-Uav_pos.col(i))/P_norm[i];
                Frep_v.col(i) << krep_v*THlev[i]*((-Uav_pos.col(i)/P_norm[i])+(-Frep_v_rotation*Uav_vel.col(i)/V_norm[i]));
            }
            else if(delta[i]>=90 && delta[i] <=270 )
            {
                Frep_p.col(i) <<0,0,0;
                Frep_v.col(i) <<0,0,0;
            }
            else
            {
                Frep_p.col(i) <<0,0,0;
                Frep_v.col(i) <<0,0,0;
            }
            // cout <<"("<<i<<")" <<P_norm[i]<<" "<<rsafe[i] << endl;
            // cout <<"(TH"<<i<<")" <<THlev[i] << endl;
            
        }
        else
        {
            Frep_p.col(i) <<0,0,0;
            Frep_v.col(i) <<0,0,0;
        }
        // cout << rsafe[i] << endl;
        // cout <<"("<<i<<")" <<P_norm[i]<<" "<<rsafe[i] << endl;   
        // cout <<"(theta"<<i<<")" <<theta[i] << endl;
        // cout <<"(delta"<<i<<")" <<delta[i] << endl;
    }

    Frep.col(0) << Frep_p.col(0) + Frep_v.col(0)+Frep_p.col(1) + Frep_v.col(1);     
    Frep.col(1) << Frep_p.col(2) + Frep_v.col(2)+Frep_p.col(3) + Frep_v.col(3);     
    Frep.col(2) << Frep_p.col(4) + Frep_v.col(4)+Frep_p.col(5) + Frep_v.col(5);
    // cout << Frep <<endl;     

}
void Controller::virtual_vel()
{
    cacl_uav_state_vector();
    safe_range();
    cacl_threat_level();
    repulsive_potential();
}
void Controller::Circumcentre_init()
{
    center << 0,0,5;
    desireUavPos <<-length*sin(90*M_PI/180),-length*sin(210*M_PI/180),-length*sin(330*M_PI/180),length*cos(90*M_PI/180),length*cos(210*M_PI/180),length*cos(330*M_PI/180),5,5,5;
}
void Controller::Uav_Circumcentre_move(float x,float y,float z,bool sw)
{
    center << x,y,z;
    float angle = 90;
    if(sw)
    {
        /*後兩台交換位置*/
        desireUavPos.col(0) << center(0)+length*-sin(90*M_PI/180),center(1)+length*cos(90*M_PI/180),center(2);
        desireUavPos.col(1) << center(0)+length*-sin(330*M_PI/180),center(1)+length*cos(330*M_PI/180),center(2);
        desireUavPos.col(2) << center(0)+length*-sin(210*M_PI/180),center(1)+length*cos(210*M_PI/180),center(2);
        /*倒三角*/
        // desireUavPos.col(0) << center(0)+length*-sin(270*M_PI/180),center(1)+length*cos(270*M_PI/180),center(2);
        // desireUavPos.col(1) << center(0)+length*-sin(150*M_PI/180),center(1)+length*cos(150*M_PI/180),center(2);
        // desireUavPos.col(2) << center(0)+length*-sin(30*M_PI/180),center(1)+length*cos(30*M_PI/180),center(2);
    }
    else
    {
        for(int i = 0; i <3;i++)
        {
            desireUavPos.col(i) << center(0)+length*-sin(angle*M_PI/180),center(1)+length*cos(angle*M_PI/180),center(2);
            angle += 120; 
        }
    }
    follower();
}
void Controller::follower()
{   
    for(int i = 0; i <3;i++)
    {
        ed.col(i) = desireUavPos.col(i) - Uav.col(i); //計算當前誤差
        // if(center(1)>=0 && center(0)>=0)
        // {
        //     yaw = atan(center(0)/center(1))*180/M_PI;
        // }
        // else if(center(1)>=0 && center(0)<0)
        // {
        //     yaw = atan(center(0)/center(1))*180/M_PI;
        // }
        // else if(center(1)<0 && center(0)>0)
        // {
        //     yaw = 180+atan(center(0)/center(1))*180/M_PI;
        // }
        // else if(center(1)<0 && center(0)<0)
        // {
        //     yaw = -(180-atan(center(0)/center(1))*180/M_PI);
        // }
        
    }
    Fformation = Kp*(ed-ed1) + Ki*ed + Kd*(ed-2*ed1+ed2);
    ed2 = ed1;
    ed1 = ed;
    // cout <<"0="<<yaw[0]<<endl;
    // cout <<"1="<<yaw[0]<<endl;
}
void Controller::process(float x,float y,float z,bool sw)
{
    virtual_vel();
    Uav_Circumcentre_move(x,y,z,sw);
    null_space_behavior();
}
void Controller::null_space_behavior()
{
    MatrixXf tmp = MatrixXf::Zero(3,6);
    Matrix3f JacoP = Matrix3f::Zero();
    Matrix3f projN = Matrix3f::Zero();
    Matrix3f I = Matrix3f::Identity();

    for(int i=0;i<6;i++)
    {
        if(P_norm[i]<rsafe[i])
        {
            if((delta[i]>=0 && delta[i] < 90) || (delta[i]>=-90 && delta[i] < 0))
            {
                tmp.col(i) << -Uav_pos.col(i)/P_norm[i];
            }
            else
            {
                tmp.col(i) <<0,0,0;
            }
        }
        else
        {
            tmp.col(i) <<0,0,0;
        }
    }
    JacoP.col(0) << tmp.col(0)+tmp.col(1);
    JacoP.col(1) << tmp.col(2)+tmp.col(3);
    JacoP.col(2) << tmp.col(4)+tmp.col(5);
    for(int i=0;i<3;i++)
    {
        projN << I - JacoP.col(i)*JacoP.col(i).transpose();
        Fnsb.col(i) << Frep.col(i)+projN*Fformation.col(i);
    }

}
// void Controller::fuzzy_NSB()
// {
//     Fuzzy fuzzy;
//     fuzzy.fx =0;
// }