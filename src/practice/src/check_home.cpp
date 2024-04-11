#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#define PI 3.1415927
mavros_msgs::State current_state;
float px=0,py=0,pz=0,latitude=0,longitude=0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void subpose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{ 
  px=msg->pose.position.x;  py=msg->pose.position.y;  pz=msg->pose.position.z;  
}
void subglobal(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    latitude = msg->latitude;
    longitude = msg->longitude;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "uav1");
    ros::NodeHandle nh("~");
    //ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher PositionTarget_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);       
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,subpose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global",10,subglobal);        
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
     // wait for FCU connection
      while(ros::ok() && !current_state.connected){
          ros::spinOnce();    
          rate.sleep();
      }

    mavros_msgs::PositionTarget PositionTarget;
    PositionTarget.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    PositionTarget.coordinate_frame = 1;
    PositionTarget.position.x = 2;
    PositionTarget.position.y = 0;
    PositionTarget.position.z = 2;
    PositionTarget_pub.publish(PositionTarget);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    int ang = 0,mode=0;
    float height=0,homex=0,homey=0;
    printf("input mode=");
    scanf("%d",&mode);
    printf("Current position = (%f,%f)\n",px,py);
    switch (mode)
    {
        case 1:
            while(ros::ok())
            {
                // printf("Current position = (%f,%f)\n",px,py);
                printf("Current GPS = (%f,%f)\n",latitude,longitude);
                ros::Duration(0.5).sleep();
                ros::spinOnce();
                rate.sleep();
            }
            break;
        case 2:
            printf("input height=");
            scanf("%f",&height);
            while(ros::ok())
            {
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } 
                else 
                {
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                    {   
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
                PositionTarget.position.z = height;
                PositionTarget_pub.publish(PositionTarget);
                ros::spinOnce();
                rate.sleep();
            }
            break;

        case 3:
            homex = px;
            homey = py;
            printf("input height=");
            scanf("%f",&height);
            while(ros::ok())
            {
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } 
                else 
                {
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                    {   
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
                PositionTarget.position.x = px;
                PositionTarget.position.y = py;
                PositionTarget.position.z = height;
                PositionTarget_pub.publish(PositionTarget);
                
                ros::spinOnce();
                rate.sleep();
            }
            break;
    }
    return 0;
}



