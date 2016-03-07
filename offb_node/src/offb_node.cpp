/**
 *
 * Modifyed by Stig Turner
 * Controls attitude and throttle!
 * (position control is commented out)
 *
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //Attitude
    ros::Publisher att_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude",10);
    ros::Publisher att_throttle_pub = nh.advertise<std_msgs::Float64>
            ("/mavros/setpoint_attitude/att_throttle",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    std_msgs::Float64 throttle;
    uint testCount = 0;
    /*pose.pose.position.z = 3;
    pose.pose.position.x = 3;
    pose.pose.position.y = 0;*/
    //pose.pose.orientation.z = 0.0; //YAW

    throttle.data = 0.1; //throttle
    pose.pose.orientation.x = 0; //pitch
    pose.pose.orientation.y = 0; //roll
    pose.pose.orientation.z = 0; //yaw

    //send a few setpoints before starting (Otherwise it is not possible to change to OFFBOARD control)
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_pos_pub.publish(pose);
        att_throttle_pub.publish(throttle);
        att_att_pub.publish(pose);
ROS_INFO("BUMB");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //pose.pose.position.z = 4;
        //pose.pose.position.x = 3;
        //pose.pose.position.y = 2;
        //local_pos_pub.publish(pose);
	
	testCount++;
    if(testCount < 250)
	{
        ROS_INFO("Throttle: 0.1 (%d)",(testCount*100)/250);
        att_throttle_pub.publish(throttle);
        att_att_pub.publish(pose);
	}
    else if( testCount < 500)
    {
        ROS_INFO("Throttle: 0.5 (%d)",((testCount-250)*100)/250);
        throttle.data = 0.2;
        att_throttle_pub.publish(throttle);
    }
	else
	{
        ROS_INFO("Throttle: 0.0");
        throttle.data = 0.0;
        att_throttle_pub.publish(throttle);
	};

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
