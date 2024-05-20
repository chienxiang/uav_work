#include "practice/Uav.h"
#include <string>
Uav::Uav(int id):nh(), rate(100.0)
{
    std::string name = "/uav" + std::to_string(id);

    state_sub = nh.subscribe<mavros_msgs::State>
            (name + "/mavros/state", 10, &Uav::state_cb, this);
    PositionTarget_pub = nh.advertise<mavros_msgs::PositionTarget>
            (name + "/mavros/setpoint_raw/local", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (name + "/mavros/setpoint_position/local", 10);         
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (name + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (name + "/mavros/set_mode");
    local_pos_sub_origin = nh.subscribe<geometry_msgs::PoseStamped>
            (name + "/mavros/local_position/pose", 10,&Uav::subpose_origin,this);
    local_pos_sub_compensate = nh.subscribe<geometry_msgs::PoseStamped>
            (name + "/mavros/local_position/pose", 10,&Uav::subpose_compensate,this);        
    local_vel_sub = nh.subscribe< geometry_msgs::TwistStamped>
            (name +"/mavros/local_position/velocity_local", 10, &Uav::subvel,this);
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            (name + "/mavros/cmd/land"); 
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            (name + "/mavros/global_position/global",10,&Uav::global_pos_cb,this);  
    uav_id = id;
}
// callback function
void Uav::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg; 
}
void Uav::subpose_origin(const geometry_msgs::PoseStamped::ConstPtr &msg)
{   
    Opx=msg->pose.position.x;  Opy=msg->pose.position.y;  Opz=msg->pose.position.z;
}
void Uav::subpose_compensate(const geometry_msgs::PoseStamped::ConstPtr &msg)
{   
    px=msg->pose.position.x + driftX - home_driftX;  py=msg->pose.position.y + driftY - home_driftY;  pz=msg->pose.position.z - home_driftZ;
}
void Uav::subvel(const  geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vx=msg->twist.linear.x; vy=msg->twist.linear.y; vz=msg->twist.linear.z;
}
void Uav::global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latitude = msg->latitude;
    longitude = msg->longitude;
}
// wait for FCU connection
void Uav::wait_for_FCU()
{
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
} 
//send a few setpoints before starting
void Uav::send_setpoints()
{
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}
void Uav::set_mode_offb()
{
    set_mode.request.custom_mode = "OFFBOARD";
}
void Uav::set_arm_cmd()
{
    arm_cmd.request.value = true;
}

void Uav::init()
{
    wait_for_FCU();
    send_setpoints();
    set_mode_offb();
    set_arm_cmd();
    uav_drifting();

}
void Uav::enable_offboard()
{
    if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
    }
}
void Uav::enable_armed()
{
    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
    }
}
void Uav::landing()
{                    
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0; 
    while (!(land_client.call(land_cmd) &&  land_cmd.response.success))
    {
        ROS_INFO("tring to land");
        ros::spinOnce();    
    } 
}
int Uav::uav_drifting() 
{
    ifs.open("./src/practice/src/coordination_shifting.txt");
    if (!ifs.is_open()) 
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }
    while (ifs >> tmp) {
        // cout<<tmp<<endl;
        drifting.push_back(tmp);
    }
    ifs.close();
    if(uav_id == 1)
    {
        driftX=3;
        driftY=0;     
        // driftX=drifting[0];
        // driftY=drifting[1];
        home_driftX = drifting[6];
        home_driftY = drifting[7];
    }
    else if(uav_id == 2)
    {
        driftX=0;
        driftY=3;
        // driftX=drifting[2];
        // driftY=drifting[3];
        home_driftX = drifting[8];
        home_driftY = drifting[9];        
    }
    else
    {   
        driftX=0;
        driftY=0;
        home_driftX = drifting[4];
        home_driftY = drifting[5];
        
    }
    home_driftZ = Opz;

}
void Uav::data_update()
{
    p_data << px,py,pz;    
    v_data << vx,vy,vz;
}
void Uav::pub_position(float x,float y,float z)
{
    PositionTarget.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.position.x = x - driftX + home_driftX;
    PositionTarget.position.y = y - driftY + home_driftY;
    PositionTarget.position.z = z + home_driftZ;
    PositionTarget_pub.publish(PositionTarget);
}
void Uav::pub_position(float x,float y,float z,float rad)
{
    PositionTarget.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.position.x = x - driftX + home_driftX;
    PositionTarget.position.y = y - driftY + home_driftY;
    PositionTarget.position.z = z + home_driftZ;
    PositionTarget.yaw = rad;
    PositionTarget_pub.publish(PositionTarget); 
}

void Uav::pub_velocity(float x,float y,float z)
{
    PositionTarget.type_mask =1 + 2 + 4 + /*8 + 16 + 32 */+ 64 + 128 + 256 + 512 + 1024 + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.velocity.x=x;
    PositionTarget.velocity.y=y;
    PositionTarget.velocity.z=z;
    PositionTarget_pub.publish(PositionTarget); 
}
void Uav::pub_velocity(Matrix3f _F)
{
    PositionTarget.type_mask =1 + 2 + 4 + /*8 + 16 + 32 */+ 64 + 128 + 256 + 512 + 1024 + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.velocity.x=_F(0,uav_id)+v[0];
    PositionTarget.velocity.y=_F(1,uav_id)+v[1];
    PositionTarget.velocity.z=_F(2,uav_id)+v[2];
    PositionTarget_pub.publish(PositionTarget); 
}
void Uav::pub_velocity(Matrix3f _F,float rad)
{
    PositionTarget.type_mask =1 + 2 + 4 + /*8 + 16 + 32 */+ 64 + 128 + 256 + 512 + /*1024*/ + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.velocity.x=_F(0,uav_id)+v[0];
    PositionTarget.velocity.y=_F(1,uav_id)+v[1];
    PositionTarget.velocity.z=_F(2,uav_id)+v[2];
    PositionTarget.yaw=rad;
    PositionTarget_pub.publish(PositionTarget); 
}        
void Uav::pub_velocity(float x,float y,float z,float rad)
{
    PositionTarget.type_mask =1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + /*1024 +*/ 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.velocity.x=x;
    PositionTarget.velocity.y=y;
    PositionTarget.velocity.z=z;
    PositionTarget.yaw=rad;
    PositionTarget_pub.publish(PositionTarget); 
}

void Uav::pub_acceleration(float x,float y,float z)
{
    PositionTarget.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 */+ 1024 + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.acceleration_or_force.x=x;
    PositionTarget.acceleration_or_force.y=y;
    PositionTarget.acceleration_or_force.z=z;
    PositionTarget_pub.publish(PositionTarget); 
}

void Uav::pub_acceleration(float x,float y,float z,float rad)
{
    PositionTarget.type_mask =1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128 + 256 + 512 + 1024*/ + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.acceleration_or_force.x=x;
    PositionTarget.acceleration_or_force.y=y;
    PositionTarget.acceleration_or_force.z=z;
    PositionTarget.yaw=rad;
    PositionTarget_pub.publish(PositionTarget); 
}
void Uav::Incremental_PID(float targetx,float targety,float targetz)
{
    float Current_Error[3] = {(targetx-px),(targety-py),(targetz-pz)};
    float Kp=0.35,Ki=0.65,Kd=0.005,limitV=3;
    v[0]= Kp*(Current_Error[0]-Last_Error[0]) + Ki*Current_Error[0] + Kd*(Current_Error[0]-2*Last_Error[0]+Previous_Error[0]);
    v[1]= Kp*(Current_Error[1]-Last_Error[1]) + Ki*Current_Error[1] + Kd*(Current_Error[1]-2*Last_Error[1]+Previous_Error[1]);
    v[2]= Kp*(Current_Error[2]-Last_Error[2]) + Ki*Current_Error[2] + Kd*(Current_Error[2]-2*Last_Error[2]+Previous_Error[2]);

    for(int i=0;i<3;i++)
    {
        Previous_Error[i] = Last_Error[i];
        Last_Error[i] = Current_Error[i];
        if(abs(v[i])>limitV && v[i]<0)
        {
            v[i]=-limitV;
        }
        else if(abs(v[i])>limitV && v[i]>0)
        {
           v[i]=limitV; 
        }
    }
    // pub_velocity(v[0],v[1],v[2]);
}
void Uav::get_position()
{
    cout<<"P_x:"<<px<<" P_y:"<<py<<" P_z:"<<pz<<endl;     
}
void Uav::get_velocity()
{
    cout<<"V_x:"<<vx<<" V_y:"<<vy<<" V_z:"<<vz<<endl;     
}

void Uav::get_acceleration()
{
    cout<<"A_x:"<<ax<<" A_y:"<<ay<<" A_z:"<<az<<endl;     
}
void Uav::get_global_pos()
{
    cout<<"latitude:"<<latitude<<" longitude:"<<longitude<<endl; //小數點後7位差距為cm
}
