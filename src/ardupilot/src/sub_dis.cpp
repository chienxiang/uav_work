#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string> 
#include <stdio.h>

using namespace std;

mavros_msgs::State current_state;
sensor_msgs::Imu imu_data;
geometry_msgs::PoseStamped position;

// 无人机状态回调函数
void state_callback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_data=*msg;
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    position=*msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe("/mavros/state", 10, state_callback);
    ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 10, imu_callback);
    ros::Subscriber position_sub = n.subscribe("/mavros/local_position/pose", 10, position_callback);

    ros::Rate rate(1);

    ROS_INFO("Initializing...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected.");
    
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher rc_override = n.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    ros::ServiceClient land_client = n.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    
    mavros_msgs::CommandTOL srv_takeoff;
    mavros_msgs::CommandBool srv;    
    mavros_msgs::SetMode srv_setMode;
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2; 

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    
    mavros_msgs::OverrideRCIn rc;
    
    ros::Time t1=ros::Time::now();
    ros::Time t2=ros::Time::now();
    float step=0,t3;
    int choose=0;
    char boolstart;
    
    ROS_INFO("initial position...");
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
    while( abs(position.pose.position.x - 0) > 0.2 || abs(position.pose.position.y - 0) > 0.2){
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = -3;
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    cout<<".";}
    cout<<endl;

    cout<<"choose pose(1) or rc(4) or set position(5):";
    cin>>choose;

    ROS_INFO("START!!!");
    switch(choose){
        case 1:
        
        // 设置无人机模式为GUIDED
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    // 解锁无人机
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
    
        while (ros::ok()) {
        pose.pose.position.x = 5*sin(step);
        pose.pose.position.y = 5*cos(step);
        pose.pose.position.z = -2;
        step=step+0.05;
        local_pos_pub.publish(pose);
        float w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
        float yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z); cout<<yaw<<endl;
        ros::spinOnce();
        rate.sleep();
        }
        break;
               
        case 4:
        srv_setMode.request.custom_mode = "alt_hold";
        if (cl.call(srv_setMode)) {
            ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
        } else {
            ROS_ERROR("Failed SetMode");
            return -1;
        }
        
        srv.request.value = true;
        if (arming_cl.call(srv)) {
            ROS_INFO("ARM send ok %d", srv.response.success);
        } else {
            ROS_ERROR("Failed arming or disarming");
        }

        ROS_INFO("down");
        t1=ros::Time::now();
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[2]=1350;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        t1=ros::Time::now();
        ROS_INFO("hold");
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[2]=1500;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        t1=ros::Time::now();
        ROS_INFO("forward");
        while(ros::Time::now() - t1 < ros::Duration(10)){
        rc.channels[5]=1550;
        rc_override.publish(rc);
        ros::spinOnce();
        rate.sleep();   }

        break;

        //測試計算與實際距離之差
        case 5:
        srv_setMode.request.custom_mode = "alt_hold";
        if (cl.call(srv_setMode)) {
            ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
        } else {
            ROS_ERROR("Failed SetMode");
            return -1;
        }
        
        srv.request.value = true;
        if (arming_cl.call(srv)) {
            ROS_INFO("ARM send ok %d", srv.response.success);
        } else {
            ROS_ERROR("Failed arming or disarming");
        }
        
        float vpx=0,vpy=0;
        
        cout<<"start?(Y or N):";
        cin>>boolstart;
        if (boolstart== 'n') {break;}
        float w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
        float yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z); cout<<cos(yaw)<<endl;
        
        //ROS_INFO("initialing linear_acceleration...");
        /*
        float init_ax=0,init_ay=0;
        for(int i=0; i<600; i++){
            init_ax=init_ax+imu_data.linear_acceleration.x;
            init_ay=init_ay+imu_data.linear_acceleration.y;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            if((i%60) == 0) ROS_INFO("%3.f %%", float(i)/600*100);
        }
        init_ax=init_ax/600;
        init_ay=init_ay/600; cout<<init_ax<<" "<<init_ay<<endl;
        */
        float init_ax=0,init_ay=0;
        //float init_ax=-0.0136476,init_ay=0.00748574;
        ros::spinOnce();
        float now_x=position.pose.position.x,now_y=position.pose.position.y,now_z=position.pose.position.z;
        float avx=0,avy=0,avx_tf=0,avy_tf=0;
        float xtarget[]={0,-100,-100,0},ytarget[]={100,100,0,0}; 
        int step=0;
        

        w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
        yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
        while(abs(yaw-3.14159)>0.02){
            w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
            yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
            rc.channels[3]=1450;
            rc_override.publish(rc);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } 

        t1=ros::Time::now(); t2=ros::Time::now();
        /*
        pose.pose.position.x = xtarget[step];
        pose.pose.position.y = ytarget[step];
        pose.pose.position.z = -3;
        local_pos_pub.publish(pose);
        */

        while (ros::ok()) {
/*
            if (abs(xtarget[step]-now_x)<1 && abs(ytarget[step]-now_y)<1){
                
                step=step+1; if (step == 4) break;
                pose.pose.position.x = xtarget[step];
                pose.pose.position.y = ytarget[step];
                pose.pose.position.z = -3;
                local_pos_pub.publish(pose);
                cout<<"change";
                ros::spinOnce();
                rate.sleep();
                
            }
            else{
                t1=ros::Time::now();       
                t3=(t1-t2).toSec();
                
                //初始
                avx=imu_data.linear_acceleration.x-init_ax;
                avy=imu_data.linear_acceleration.y-init_ay;
                
                //旋轉
                w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
                yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
                avx_tf=avx*cos(yaw)-avy*sin(yaw);
                avy_tf=avx*sin(yaw)+avy*cos(yaw);

                //計算位置
                now_x=now_x+vpx*t3+0.5*(avx)*t3*t3;
                now_y=now_y+vpy*t3+0.5*(avy)*t3*t3;
                vpx=vpx+(avx)*t3; vpy=vpy+(avy)*t3;
                //cout<<imu_data.linear_acceleration.x*t3<<"  "<<imu_data.linear_acceleration.y*t3<<"  "<<t3<<endl;
                cout<<"count positoin:  "<<now_x<<" "<<now_y<<" "<<atan2(2*w*z+2*x*y,1-2*y*y-2*z*z)<<endl;
                cout<<"true positoin:   "<<position.pose.position.x<<" "<<position.pose.position.y<<endl;
                cout<<"target positoin: "<<xtarget[step]<<" "<<ytarget[step]<<endl;
                ros::spinOnce();
                ros::Duration(0.1).sleep();  

                t2=t1;
            }
*/
            t1=ros::Time::now();       
            t3=(t1-t2).toSec();

            //前進 
            rc.channels[3]=1500;           
            rc.channels[4]=1550;
            rc_override.publish(rc);
            ros::spinOnce();

            //初始
            avx=imu_data.linear_acceleration.x-init_ax;
            avy=imu_data.linear_acceleration.y-init_ay;

            //計算位置
            now_x=now_x+vpx*t3+0.5*(avx)*t3*t3;
            now_y=now_y+vpy*t3+0.5*(avy)*t3*t3;
            vpx=vpx+(avx)*t3; vpy=vpy+(avy)*t3;
            //cout<<imu_data.linear_acceleration.x*t3<<"  "<<imu_data.linear_acceleration.y*t3<<"  "<<t3<<endl;
            cout<<"count positoin:  "<<now_x<<" "<<now_y<<" "<<atan2(2*w*z+2*x*y,1-2*y*y-2*z*z)<<endl;
            cout<<"true positoin:   "<<position.pose.position.x<<" "<<position.pose.position.y<<endl;
            //cout<<"target positoin: "<<xtarget[step]<<" "<<ytarget[step]<<endl;
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            t2=t1;

            if(abs(now_x)>100) {
                w=imu_data.orientation.w,x=imu_data.orientation.x,y=imu_data.orientation.y,z=imu_data.orientation.z;
                yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
                cout<<"count position(without y) -> x:"<<now_x*cos(yaw)<<" y:"<<now_x*sin(yaw)<<endl;
                cout<<"count position(with y) -> x:"<<now_x*cos(yaw)-now_y*sin(yaw)<<" y:"<<now_x*sin(yaw)+now_y*cos(yaw)<<endl;
                //avx_tf=avx*cos(yaw)-avy*sin(yaw);
                //avy_tf=avx*sin(yaw)+avy*cos(yaw);
                break;
            }
        }       
        
        break;
        }       
        
        ros::spinOnce();
        rate.sleep();
    
    ROS_INFO("END!!");
    return 0;
}

