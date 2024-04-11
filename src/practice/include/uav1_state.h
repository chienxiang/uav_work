#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <uav_path/avoid_vector.h>
#include <uav_path/flight_mode.h>
#include <uav_path/formation.h>
#include <uav_path/land.h>
#include <uav_path/scale.h>
#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <complex>

using namespace std;

class uav_state
{
    public:
        ros::NodeHandle nh;
        ros::Subscriber local_pos_sub;
        ros::Subscriber local_vec_sub;
        ros::Subscriber local_yaw_sub;
        ros::Subscriber state_sub;
        ros::Subscriber obstacle_sub;
        ros::Subscriber target;
        ros::Subscriber land;
        ros::Publisher local_pos_pub;
        ros::Publisher local_vec_pub;
        ros::Publisher scale;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Subscriber local_pose_uav2_sub;
        ros::Subscriber local_vec_uav2_sub;
        ros::Subscriber local_pos_uav0_sub;
        ros::Subscriber local_vec_uav0_sub;

        uav_path::scale scale1;
        geometry_msgs::TwistStamped vec;
        mavros_msgs::PositionTarget pose;
        mavros_msgs::State current_state;
    

        int well,mode;
        float px,py,pz,vx,vy,vz,ax,ay,az,yaw_t,g_x=9,g_y=5,g_z=2,g_rad,avoid[3],distent,uav2_vx,uav2_vy,uav2_vz,uav2_px,uav2_py,uav2_pz,uav0_vx,uav0_vy,uav0_vz,uav0_px,uav0_py,uav0_pz,start=1,landing=0;

        uav_state(){
            local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10,&uav_state::subpose,this);

            local_vec_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/pidvec", 10,&uav_state::subvec,this);

            local_yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/target_local", 10, &uav_state::subyaw, this);

            local_vec_pub =  nh.advertise<geometry_msgs::TwistStamped>("/uav1/pidvec", 10);

            state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10,&uav_state::state_cb,this );
	    
            obstacle_sub = nh.subscribe<uav_path::avoid_vector>("/uav1/avoid_vector", 10,&uav_state::obstacle,this);

            target = nh.subscribe<uav_path::formation>("/uav1/target", 10, &uav_state::subtarget,this);

            land = nh.subscribe<uav_path::land>("/uav1/land", 10, &uav_state::subland,this);

            local_pose_uav2_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, &uav_state::subpose_uav2,this);

            local_vec_uav2_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav2/pidvec", 10, &uav_state::subvec_uav2,this);

            local_pos_uav0_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &uav_state::subpose_uav0,this);

            local_vec_uav0_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav0/pidvec", 10, &uav_state::subvec_uav0,this);  

            local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);

            scale = nh.advertise<uav_path::scale>("/uav1/scale", 10);

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");

            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");          
                      
            }

            mavros_msgs::SetMode offb_set_mode;
            //ros::Rate rate();
            mavros_msgs::CommandBool arm_cmd;
  


        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state = *msg;}

        void subpose(const geometry_msgs::PoseStamped::ConstPtr &msg){   
            px=msg->pose.position.x+1;  py=msg->pose.position.y;  pz=msg->pose.position.z; }

        void subvec(const  geometry_msgs::TwistStamped::ConstPtr &msg){
            vx=msg->twist.linear.x; vy=msg->twist.linear.y; vz=msg->twist.linear.z;}

        void subyaw(const mavros_msgs::PositionTarget::ConstPtr &msg){
            yaw_t=msg->yaw;                                                      }

        void obstacle(const uav_path::avoid_vector::ConstPtr &msg){
            avoid[0]=msg->velocity.x;    avoid[1]=msg->velocity.y; avoid[2]=msg->velocity.z;well=msg->well;distent=msg->distent; mode=msg->mode;}

       void pub_pidvec(float x,float y,float z){
            vec.twist.linear.x=x;
            vec.twist.linear.y=y;
            vec.twist.linear.z=z;
            local_vec_pub.publish(vec); }

        void pub_position(float x,float y,float z){
            pose.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            local_pos_pub.publish(pose); }

        void pub_position(float x,float y,float z,float rad){
            pose.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
            pose.coordinate_frame = 1;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void pub_velocity(float x,float y,float z){
            pose.type_mask =1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.velocity.x=x;
            pose.velocity.y=y;
            pose.velocity.z=z;
            local_pos_pub.publish(pose); }
        
        void pub_velocity(float x,float y,float z,float rad){
            pose.type_mask =1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + /*1024 +*/ 2048;
            pose.coordinate_frame = 1;
            pose.velocity.x=x;
            pose.velocity.y=y;
            pose.velocity.z=z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void pub_acceleration(float x,float y,float z){
            pose.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 */+ 1024 + 2048;
            pose.coordinate_frame = 1;
            pose.acceleration_or_force.x=x;
            pose.acceleration_or_force.y=y;
            pose.acceleration_or_force.z=z;
            local_pos_pub.publish(pose); }

        void pub_acceleration(float x,float y,float z,float rad){
            pose.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 + 1024*/ + 2048;
            pose.coordinate_frame = 1;
            pose.acceleration_or_force.x=x;
            pose.acceleration_or_force.y=y;
            pose.acceleration_or_force.z=z;
            pose.yaw=rad;
            local_pos_pub.publish(pose); }

        void get_position(){
            cout<<"P_x:"<<px<<" P_y:"<<py<<" P_z:"<<pz<<endl;     }

        void get_velocity(){
            cout<<"V_x:"<<vx<<" V_y:"<<vy<<" V_z:"<<vz<<endl;     }

        void get_acceleration(){
            cout<<"A_x:"<<ax<<" A_y:"<<ay<<" A_z:"<<az<<endl;     }

	void subtarget(const uav_path::formation::ConstPtr &msg){
            g_x=msg->position.x; g_y=msg->position.y; g_z=msg->position.z; start=msg->start;        }

	void subland(const uav_path::land::ConstPtr &msg){
            landing=msg->landing;        }

	void subvec_uav2(const geometry_msgs::TwistStamped::ConstPtr &msg){
   	uav2_vx=msg->twist.linear.x; uav2_vy=msg->twist.linear.y; uav2_vz=msg->twist.linear.z;      }

	void subpose_uav2(const geometry_msgs::PoseStamped::ConstPtr &msg){
   	uav2_px=msg->pose.position.x-1; uav2_py=msg->pose.position.y; uav2_pz=msg->pose.position.z;   }

	void subvec_uav0(const geometry_msgs::TwistStamped::ConstPtr &msg){
   	uav0_vx=msg->twist.linear.x; uav0_vy=msg->twist.linear.y; uav0_vz=msg->twist.linear.z;      }

	void subpose_uav0(const geometry_msgs::PoseStamped::ConstPtr &msg){
   	uav0_px=msg->pose.position.x; uav0_py=msg->pose.position.y; uav0_pz=msg->pose.position.z;   }

void *qr1(float atf[],float cx,float cy,float cz){
    float size1=sqrt(pow(cx,2)+pow(cy,2)+pow(cz,2));
    float size2=sqrt(pow(uav2_vx,2)+pow(uav2_vy,2)+pow(uav2_vz,2));
    atf[0]=(cz/size1)*(uav2_vy/size2)-(cy/size1)*(uav2_vz/size2); atf[1]=(cx/size1)*(uav2_vz/size2)-(cz/size1*uav2_vx/size2); atf[2]=(cy/size1)*(uav2_vx/size2)-(cx/size1)*(uav2_vy/size2);
    float size=sqrt(pow(atf[0],2)+pow(atf[1],2)+pow(atf[2],2));
    atf[0]=atf[0]/size; atf[1]=atf[1]/size; atf[2]=atf[2]/size;
}

void *qr2(float atf[],float cx,float cy,float cz){
    float size1=sqrt(pow(cx,2)+pow(cy,2)+pow(cz,2));
    float size2=sqrt(pow(uav0_vx,2)+pow(uav0_vy,2)+pow(uav0_vz,2));
    atf[0]=(cz/size1)*(uav0_vy/size2)-(cy/size1)*(uav0_vz/size2); atf[1]=(cx/size1)*(uav0_vz/size2)-(cz/size1*uav0_vx/size2); atf[2]=(cy/size1)*(uav0_vx/size2)-(cx/size1)*(uav0_vy/size2);
    float size=sqrt(pow(atf[0],2)+pow(atf[1],2)+pow(atf[2],2));
    atf[0]=atf[0]/size; atf[1]=atf[1]/size; atf[2]=atf[2]/size;
}
};
