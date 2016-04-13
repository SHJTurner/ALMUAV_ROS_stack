/**
 *
 * Author: Stig Turner
 * Control node for ALMUAV
 *
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define InitPosHeight 2.0

ros::Rate rate(20.0);
ros::ServiceClient set_mode_client;
ros::Publisher local_pos_pub;
geometry_msgs::PoseStamped InitPos;
mavros_msgs::State current_state; //UAV state

//State machine enum
enum States{
    IDLE,
    READY,
    MOVE_TO_INIT_POS,
    START_SETPOINT_STREAM,
    ENABLE_OFFBOARD,
    DESCEND,
    EMERGENCY_STOP
};

//Current_state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Estimated_pose callback
geometry_msgs::PoseWithCovarianceStamped estimated_pose;
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped& msg){
        estimated_pose = *msg;
}

//Before entering offboard mode, the setpoint stream must be started otherwise the mode switch will be rejected.
void start_setpoint_stream()
{
    ROS_INFO("Started streaming setpoints");
    for(int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(InitPos);
        ros::spinOnce();
        rate.sleep();
    }
}

//Enable offboard mode
bool enable_offboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ROS_INFO("Swithing to OFFBOARD mode");
    if(current_state.mode != "OFFBOARD")
    {
        if( !(set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) )
        {
            ROS_WARN("Failed to enable OFFBOARD mode!");
            local_pos_pub.publish(pose); //keep sending setpoints if it fails
            return false;
        }
    }
    ROS_INFO("OFFBOARD mode enabled");
    return true;
}

//calc Euclidean distance
double eu_dist_to_setpoint(double x, double y, double z)
{
    return sqrt( ( pow((x+estimated_pose.pose.pose.position.x),2) + pow((y+estimated_pose.pose.pose.position.y),2) + pow((z+estimated_pose.pose.pose.position.z),2) )  );
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    //Create subscribers and publichers
    //Subscribe to pose estimater
    ros::Subscriber estimated_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov",1,pose_cb);
    //Subscribe to UAV state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //Create published for publiching setpoints
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local");
    //Create service client for setting UAV state
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //Ready InitPos meassage
    InitPos.pose.position.z = InitPosHeight;
    InitPos.pose.position.x = 0;
    InitPos.pose.position.y = 0;
    InitPos.pose.orientation.z = 0.0; //YAW

    ROS_INFO("Waiting for FCU connection");
    //wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to FCU");

    ROS_INFO("Wating for UAV to be armed");
    //wait for UAV to be armed
    while(ros::ok() && !current_state.armed )
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV is armed");

    int state = IDLE;
    int next_state = IDLE;
    bool emergency_stop = false;

    //state machine!
    while(ros::ok())
    {
        state = next_state;

        if(emergency_stop)
        {
            state = EMERGENCY_STOP;
            next_state = EMERGENCY_STOP;
        }
        switch(state)
        {
            case IDLE:
                //DONT DO ANYTHING
                break;

            case READY:
                //WAIT FOR UAV TO BE DETECTED

                break;
            case START_SETPOINT_STREAM:
                start_setpoint_stream();
                next_state = ENABLE_OFFBOARD;
                break;
            case ENABLE_OFFBOARD:
                if(enable_offboard())
                    next_state = MOVE_TO_INIT_POS;
                break;
            case MOVE_TO_INIT_POS:
                local_pos_pub.publish(InitPos); //publich INIT POS
                //Go to descend if within 0.05 meters of setpoint
                if(eu_dist_to_setpoint(InitPos.pose.position.x,InitPos.pose.position.y,InitPos.pose.position.z)> 0.05)
                    next_state = DESCEND;
                break;
            case DESCEND:
                //IF within tollerenct and not landend
                    //DESCENT
                //ELSE Switch state to MOVE TO INIT POS
                break;
            case EMERGENCY_STOP:
                //DISARM AT ONCE!
                ROS_ERROR_ONCE("EMERGENCY_STOP! DISARMING UAV!");
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Shutting down contril node");
    return 0;
}
