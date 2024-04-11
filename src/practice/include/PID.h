#include <ros/ros.h>
#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <mavros_msgs/CommandTOL.h>


using namespace std;

class pid
{
    public:
        float kp=0.35,ki=0.65,kd=0.005,a=kp+ki+kd,b=2*kd-kp, c=kd;
        float velocity_x,velocity_y,velocity_z,pre_px=0,ppre_px=0,pre_py=0,ppre_py=0,pre_pz=0,ppre_pz=0, velocity_size,acceleration_size;
        float acceleration_x,acceleration_y,acceleration_z,pre_vx=0,ppre_vx=0,pre_vy=0,ppre_vy=0,pre_vz=0,ppre_vz=0,vector_size,rad;
        int land=0;
        ros::NodeHandle nh_pid;
        ros::ServiceClient land_client;
        ros::ServiceClient land_client0;
        ros::ServiceClient land_client1;
        ros::ServiceClient land_client2;
        
        pid(){
            land_client = nh_pid.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
            land_client0 = nh_pid.serviceClient<mavros_msgs::CommandTOL>("/uav0/mavros/cmd/land");
            land_client1 = nh_pid.serviceClient<mavros_msgs::CommandTOL>("/uav1/mavros/cmd/land");
            land_client2 = nh_pid.serviceClient<mavros_msgs::CommandTOL>("/uav2/mavros/cmd/land");       
            }

        void velocity(float e_x,float e_y,float e_z,float limit_v){
            velocity_x= (e_x*a+pre_px*b+ppre_px*c);
            ppre_px=pre_px;
            pre_px=e_x;

            velocity_y= (e_y*a+pre_py*b+ppre_py*c);
            ppre_py=pre_py;
            pre_py=e_y;

            velocity_z= (e_z*a+pre_pz*b+ppre_pz*c);
            ppre_pz=pre_pz;
            pre_pz=e_z;

            vector_size=sqrt(pow(velocity_x,2)+pow(velocity_y,2)+pow(velocity_z,2));

            if(vector_size>limit_v) {
                velocity_x=velocity_x*(limit_v/vector_size);
                velocity_y=velocity_y*(limit_v/vector_size);
                velocity_z=velocity_z*(limit_v/vector_size);      }      }

        void velocity(float e_x,float e_y,float e_z){
            velocity_x= (e_x*a+pre_px*b+ppre_px*c);
            ppre_px=pre_px;
            pre_px=e_x;

            velocity_y= (e_y*a+pre_py*b+ppre_py*c);
            ppre_py=pre_py;
            pre_py=e_y;

            velocity_z= (e_z*a+pre_pz*b+ppre_pz*c);
            ppre_pz=pre_pz;
            pre_pz=e_z;

            vector_size=sqrt(pow(velocity_x,2)+pow(velocity_y,2)+pow(velocity_z,2));    }

        void acceleration(float e_px,float e_py,float e_pz,float e_vx,float e_vy,float e_vz,float limit_a){
            acceleration_x= (e_px*a+pre_px*b+ppre_px*c)+(e_vx*a+pre_vx*b+ppre_vx*c);
            ppre_px=pre_px; ppre_vx=pre_vx;
            pre_px=e_px; pre_vx=e_vx;

            acceleration_y= (e_py*a+pre_py*b+ppre_py*c);
            ppre_py=pre_py; ppre_vy=pre_vy;
            pre_py=e_py;  pre_vy=e_vy; 

            acceleration_z= (e_pz*a+pre_pz*b+ppre_pz*c);
            ppre_pz=pre_pz; ppre_vz=pre_vz;
            pre_pz=e_pz; pre_vz=e_vz;

            vector_size=sqrt(pow(acceleration_x,2)+pow(acceleration_y,2)+pow(acceleration_z,2));

             if(vector_size>limit_a) {
                acceleration_x=acceleration_x*(limit_a/vector_size);
                acceleration_y=acceleration_y*(limit_a/vector_size);
                acceleration_z=acceleration_z*(limit_a/vector_size);   }}

        void acceleration(float e_px,float e_py,float e_pz,float e_vx,float e_vy,float e_vz){
            acceleration_x= (e_px*a+pre_px*b+ppre_px*c)+(e_vx*a+pre_vx*b+ppre_vx*c);
            ppre_px=pre_px; ppre_vx=pre_vx;
            pre_px=e_px; pre_vx=e_vx;

            acceleration_y= (e_py*a+pre_py*b+ppre_py*c);
            ppre_py=pre_py; ppre_vy=pre_vy;
            pre_py=e_py;  pre_vy=e_vy; 

            acceleration_z= (e_pz*a+pre_pz*b+ppre_pz*c);
            ppre_pz=pre_pz; ppre_vz=pre_vz;
            pre_pz=e_pz; pre_vz=e_vz;

            vector_size=sqrt(pow(acceleration_x,2)+pow(acceleration_y,2)+pow(acceleration_z,2));     }
        
        void *combined_force(float main[],float avoid[],float combined[],int mode,int well,float yaw,float scale){
            vector_size=sqrt(pow(main[0],2)+pow(main[1],2)+pow(main[2],2));
            switch(mode){
                case 1:
                    combined[0]= main[0];
                    combined[1]= main[1];
                    combined[2]= main[2];
	            break;
                case 2:
                    switch(well) {
                        case 0:
                            float rotate[3];
                            rotate[0]=avoid[0]*cos(yaw)-avoid[1]*sin(yaw);
                            rotate[1]=avoid[0]*sin(yaw)+avoid[1]*cos(yaw);
                            rotate[2]=avoid[2];
                            combined[0]= main[0]*(1-scale)+vector_size*rotate[0]*scale;
                            combined[1]= main[1]*(1-scale)+vector_size*rotate[1]*scale;
                            combined[2]= main[2]*(1-scale)+vector_size*rotate[2]*scale;
                            search_time = ros::Time::now();
                            break;
                        case 3:
                            tau = tau + (back_time2 - back_time1);
                            combined[0]=-search_path_y[step]*sin(yaw);
                            combined[1]=search_path_y[step]*cos(yaw);
                            combined[2]=search_path_z[step] ;
                            back_time1 = ros::Time::now();
                            back_time2 = ros::Time::now();
                            if((ros::Time::now() - search_time) - tau > ros::Duration(5.0)){
                                step = step + 1;
                                search_time = ros::Time::now();
                                tau = ros::Duration(0); }
                            if(step>4) landing();
                            break;
                        case 4:
                            combined[0]=-0.5*cos(yaw);
                            combined[1]=-0.5*sin(yaw);
                            combined[2]=0;
                            back_time2 = ros::Time::now();                             
                            break; }}}
        
        void yaw_controll(float yaw_x,float yaw_y,int well){            //面向目標
            if(well == 3 || well == 4){}
            else{
            switch(yaw_y>0){
                case true:
                    switch(yaw_x>0){
                        case true:
                            rad=atan(yaw_y/yaw_x);
                            break;
                        case false:
                            rad=3.14159265359+atan(yaw_y/yaw_x);
                            break;   }
                        break;
                case false:
                    switch(yaw_x>0){
                        case true:
                            rad=atan(yaw_y/yaw_x);
                            break;
                        case false:
                            rad=-3.14159265359+atan(yaw_y/yaw_x);
                            break;   }
                        break;   }}
           
        if(yaw_x==0){
            if(yaw_y>0) {
                rad=1.57079632679; }
            else {
                rad=-1.57079632679;}}
        if(yaw_y==0){
            if(yaw_x>0){
               rad=0;}
            else{
               rad=3.14159265359;}}}

        void landing0(){                    
            mavros_msgs::CommandTOL land_cmd0;
           land_cmd0.request.yaw = 0;
            land_cmd0.request.latitude = 0;
            land_cmd0.request.longitude = 0;
            land_cmd0.request.altitude = 0; 
            while (!(land_client0.call(land_cmd0) &&  land_cmd0.response.success)){
                ROS_INFO("tring to land0");
                land=1;
                ros::spinOnce();   } }

        void landing(){                    
            mavros_msgs::CommandTOL land_cmd;
            land_cmd.request.yaw = 0;
            land_cmd.request.latitude = 0;
            land_cmd.request.longitude = 0;
            land_cmd.request.altitude = 0; 
            while (!(land_client.call(land_cmd) &&  land_cmd.response.success)){
                ROS_INFO("tring to land");
                land=1;
                ros::spinOnce();   } }

        void landing1(){                    
            mavros_msgs::CommandTOL land_cmd1;
           land_cmd1.request.yaw = 0;
            land_cmd1.request.latitude = 0;
            land_cmd1.request.longitude = 0;
            land_cmd1.request.altitude = 0; 
            while (!(land_client1.call(land_cmd1) &&  land_cmd1.response.success)){
                ROS_INFO("tring to land1");
                land=1;
                ros::spinOnce();   } }

        void landing2(){                    
            mavros_msgs::CommandTOL land_cmd2;
           land_cmd2.request.yaw = 0;
            land_cmd2.request.latitude = 0;
            land_cmd2.request.longitude = 0;
            land_cmd2.request.altitude = 0; 
            while (!(land_client2.call(land_cmd2) &&  land_cmd2.response.success)){
                ROS_INFO("tring to land2");
                land=1;
                ros::spinOnce();   } }

        private:
            float search_path_y[5]{-0.5,0.5,0.5,-0.5,0}, search_path_z[5]{0,0,0,0,0.5}; 
            int step=0;
            ros::Time search_time = ros::Time::now();
            ros::Time back_time1 = ros::Time::now();
            ros::Time back_time2 = ros::Time::now();
            ros::Duration tau = ros::Duration(0);
};


class fuzzy{
public:

    float Table[5][5],membership_scale[5],membership_v[5],membership_d[5],c,pt,scale;

    void fuzzy_scale(float v,float d){
        f_speed(v);
        f_distent(d);
        f_scale(membership_v,membership_d,membership_scale);
        float c[]={0.2,0.4,0.5,0.6,0.8};
        pt=membership_scale[0] + membership_scale[1] + membership_scale[2] + membership_scale[3] + membership_scale[4];
        scale=membership_scale[0]*c[0] + membership_scale[1]*c[1] + membership_scale[2]*c[2] + membership_scale[3]*c[3] + membership_scale[4]*c[4]/pt;
    }

private:
    void f_speed(float v){
        float vs=0,s=0,n=0,h=0,vh=0;
        float pb=1.5,ps=1.2,z=1,ns=0.8,nb=0.5;
        if(v >=pb){
            vh=1;  }
            else if(v>=ps){
                h=exp((-(v-ps)*(v-ps))/(0.02)); vh=exp((-(v-pb)*(v-pb))/(0.02));                }
                else if(v>=z){
                    n=exp((-(v-z)*(v-z))/(0.02)); h=exp((-(v-ps)*(v-ps))/(0.02));                 }
                    else if(v>=ns){
                        s=exp((-(v-ns)*(v-ns))/(0.02)); n=exp((-(v-z)*(v-z))/(0.02));              }
                        else if(v>=nb){
                            vs=exp((-(v-nb)*(v-nb))/(0.02)); s=exp((-(v-ns)*(v-ns))/(0.02));     }
                            else{
                                vs=1;                                               }

        membership_v[0]=vs;
        membership_v[1]=s;
        membership_v[2]=n;
        membership_v[3]=h;
        membership_v[4]=vh;
    }

    void f_distent(float d){
        float vc=0,c=0,n=0,f=0,vf=0;
        float pb=1.5,ps=1.2,z=1,ns=0.8,nb=0.5;
        if(d >=pb){
            vf=1;  }
            else if(d>=ps){
                f=exp((-(d-ps)*(d-ps))/(0.02)); vf=exp((-(d-pb)*(d-pb))/(0.02));                }
                else if(d>=z){
                    n=exp((-(d-z)*(d-z))/(0.02)); f=exp((-(d-ps)*(d-ps))/(0.02));                 }
                    else if(d>=ns){
                        c=exp((-(d-ns)*(d-ns))/(0.02)); n=exp((-(d-z)*(d-z))/(0.02));              }
                        else if(d>=nb){
                            vc=exp((-(d-nb)*(c-nb))/(0.02)); c=exp((-(d-ns)*(d-ns))/(0.02));     }
                            else{
                                vc=1;                                               }

        membership_d[0]=vc;
        membership_d[1]=c;
        membership_d[2]=n;
        membership_d[3]=f;
        membership_d[4]=vf;
    }

    void *f_scale(float membership_v[],float membership_d[],float membership_scale[]){
        for(int i=0;i<5;i++){
            for(int j=0;j<5;j++){
                Table[i][j]=min(membership_v[i],membership_d[j]);            }        }

        initializer_list<float> vs ={Table[0][3],Table[0][4],Table[1][4]};

        initializer_list<float> s={Table[0][1],Table[0][2],Table[1][2],Table[1][3],Table[2][3],Table[2][4],Table[3][4]};

        initializer_list<float> e={Table[0][0],Table[1][1],Table[2][2],Table[3][3],Table[4][4]};

        initializer_list<float> l={Table[1][0],Table[2][0],Table[2][1],Table[3][1],Table[3][2],Table[4][2],Table[4][3]};

        initializer_list<float> vl={Table[3][0],Table[4][0],Table[4][1]};

        membership_scale[0]=max(vs);
        membership_scale[1]=max(s);
        membership_scale[2]=max(e);
        membership_scale[3]=max(l);
        membership_scale[4]=max(vl);
    }
};

class atf_fuzzy{
public:

    float Table[5][5],membership_scale[5],membership_v[5],membership_d[5],c,pt,scale;

    void fuzzy_scale(float v,float d){
        f_speed(v);
        f_distent(d);
        f_scale(membership_v,membership_d,membership_scale);
        float c[]={1.1,0.9,0.7,0.55,0.4};
        pt=membership_scale[0] + membership_scale[1] + membership_scale[2] + membership_scale[3] + membership_scale[4];
        scale=membership_scale[0]*c[0] + membership_scale[1]*c[1] + membership_scale[2]*c[2] + membership_scale[3]*c[3] + membership_scale[4]*c[4]/pt;
    }

private:
    void f_speed(float v){
        float vs=0,s=0,n=0,h=0,vh=0;
        float pb=0.2,ps=0.4,z=0.6,ns=0.8,nb=1;
        if(v >=pb){
            vh=1;  }
            else if(v>=ps){
                h=exp((-(v-ps)*(v-ps))/(0.02)); vh=exp((-(v-pb)*(v-pb))/(0.02));                }
                else if(v>=z){
                    n=exp((-(v-z)*(v-z))/(0.02)); h=exp((-(v-ps)*(v-ps))/(0.02));                 }
                    else if(v>=ns){
                        s=exp((-(v-ns)*(v-ns))/(0.02)); n=exp((-(v-z)*(v-z))/(0.02));              }
                        else if(v>=nb){
                            vs=exp((-(v-nb)*(v-nb))/(0.02)); s=exp((-(v-ns)*(v-ns))/(0.02));     }
                            else{
                                vs=1;                                               }

        membership_v[0]=vs;
        membership_v[1]=s;
        membership_v[2]=n;
        membership_v[3]=h;
        membership_v[4]=vh;
    }

    void f_distent(float d){
        float vc=0,c=0,n=0,f=0,vf=0;
        float pb=3.8,ps=3,z=2,ns=1,nb=0.5;
        if(d >=pb){
            vf=1;  }
            else if(d>=ps){
                f=exp((-(d-ps)*(d-ps))/(0.05)); vf=exp((-(d-pb)*(d-pb))/(0.05));                }
                else if(d>=z){
                    n=exp((-(d-z)*(d-z))/(0.05)); f=exp((-(d-ps)*(d-ps))/(0.05));                 }
                    else if(d>=ns){
                        c=exp((-(d-ns)*(d-ns))/(0.05)); n=exp((-(d-z)*(d-z))/(0.05));              }
                        else if(d>=nb){
                            vc=exp((-(d-nb)*(c-nb))/(0.05)); c=exp((-(d-ns)*(d-ns))/(0.05));     }
                            else{
                                vc=1;                                               }

        membership_d[0]=vc;
        membership_d[1]=c;
        membership_d[2]=n;
        membership_d[3]=f;
        membership_d[4]=vf;
    }

    void *f_scale(float membership_v[],float membership_d[],float membership_scale[]){
        for(int i=0;i<5;i++){
            for(int j=0;j<5;j++){
                Table[i][j]=min(membership_v[i],membership_d[j]);            }        }

        initializer_list<float> pb ={Table[2][4],Table[3][4],Table[4][4],Table[4][3]};

        initializer_list<float> ps ={Table[1][4],Table[2][3],Table[3][2],Table[3][3],Table[4][1],Table[4][2]};

        initializer_list<float> z ={Table[0][0],Table[1][1],Table[2][2],Table[3][3],Table[4][4]};

        initializer_list<float> ns ={Table[0][2],Table[0][3],Table[1][1],Table[1][2],Table[2][1],Table[3][0]};

        initializer_list<float> nb ={Table[0][0],Table[0][1],Table[1][0],Table[2][0]};

        membership_scale[0]=max(pb);
        membership_scale[1]=max(ps);
        membership_scale[2]=max(z);
        membership_scale[3]=max(ns);
        membership_scale[4]=max(nb);
    }
};