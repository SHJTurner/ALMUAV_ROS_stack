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

#define InitPosHeight 1.0 //Meter
#define descentSpeed 0.1 // Meter/secunds
#define updateRate 20.0 //Hz

ros::Rate rate(updateRate);
ros::ServiceClient set_mode_client;
ros::Publisher local_pos_pub;
mavros_msgs::State current_state; //UAV state
geometry_msgs::PoseStamped TargetPos;
ros::Time last_descent_request;
double init_altitude;
bool streaming_setpoints;

//State machine enum
enum States{
    IDLE,
    SEARCHING, //Searching for the UAV
    START_SETPOINT_STREAM, //Start sending target positions to the UAV
    ENABLE_OFFBOARD, //Enable offboard control
    WAIT_FOR_POS_REACHED, //Wait for the UAV to center it self over the platform
    DESCEND, //Descend until landed
    LANDED, //Landed, Disarm UAV
    FAILSAFE //Failsafe, land the UAV regardless of position/state
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
bool start_setpoint_stream()
{ros::Time::now();
    setpoint_center(); //update setpoint, to location over the targer, in same altitude
    streaming_setpoints = true;
    static int i = 10;
    i--;
    ROS_INFO("Started streaming setpoints");
    if(ros::ok() && i > 0)
    {
        i = 10;
        return true;
    }
    else
        return false;
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
            return false;
        }
    }
    ROS_INFO("OFFBOARD mode enabled");
    return true;
}

//Moves UAV to center over platform at a minimum altitude of 1 meter (otherwise the altitude is holded)
void setpoint_center()
{
    if(estimated_pose.pose.pose.position.z < InitPosHeight)
    {
        geometry_msgs::PoseStamped Pos;
        InitPos.pose.position.z = InitPosHeight;
        init_altitude = InitPosHeight;
        InitPos.pose.position.x = 0;
        InitPos.pose.position.y = 0;
        InitPos.pose.orientation.z = 0.0;
        TargetPos = Pos;
    }
    else
    {
        geometry_msgs::PoseStamped Pos;
        InitPos.pose.position.z = estimated_pose.pose.pose.position.z;
        init_altitude = estimated_pose.pose.pose.position.z;
        InitPos.pose.position.x = 0;
        InitPos.pose.position.y = 0;
        InitPos.pose.orientation.z = 0.0;
        TargetPos = Pos;
    }
}

void descend()
{
        last_descent_request = ros::Time::now();
        geometry_msgs::PoseStamped Pos;
        InitPos.pose.position.z = init_altitude - (descentSpeed/updateRate);
        InitPos.pose.position.x = 0;
        InitPos.pose.position.y = 0;
        InitPos.pose.orientation.z = 0.0;
        TargetPos = Pos;
}

//calc Euclidean distance to center
double eu_dist_to_center()
{
    return sqrt( ( pow((TargetPos.pose.position.x+estimated_pose.pose.pose.position.x),2) + pow((TargetPos.pose.position.y+estimated_pose.pose.pose.position.y),2) )  );
}

//calc Euclidean distance to target
double eu_dist_to_target_point()
{
    return sqrt( ( pow((TargetPos.pose.position.x+estimated_pose.pose.pose.position.x),2) + pow((TargetPos.pose.position.y+estimated_pose.pose.pose.position.y),2) + pow((TargetPos.pose.position.z+estimated_pose.pose.pose.position.z),2) )  );
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

    streaming_setpoints = false; //init boolean for streaming commands enabled/disabled
    last_descent_request = ros::Time::now(); //assign value to las_descent request (as long as the time is past time it fine)

    //wait for FCU connection
    ROS_INFO("Waiting for FCU connection");
    while(current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        if(!ros::ok())
        {
            ROS_INFO("Shutting down control node");
            return 0;
        }
    }
    ROS_INFO("Connected to FCU");

    //wait for UAV to be armed
    ROS_INFO("Wating for UAV to be armed");
    while(!current_state.armed )
    {
        ros::spinOnce();
        rate.sleep();
        if(!ros::ok())
        {
            ROS_INFO("Shutting down control node");
            return 0;
        }
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
            state = FAILSAFE;
            next_state = FAILSAFE;
        }
        switch(state)
        {
            case IDLE:
                //DONT DO ANYTHING
                break;

            case SEARCHING:
                //WAIT FOR UAV TO BE DETECTED
                break;

            case START_SETPOINT_STREAM:
                if(start_setpoint_stream())
                    next_state = ENABLE_OFFBOARD;
                break;

            case ENABLE_OFFBOARD:
                if(enable_offboard())
                    next_state = WAIT_FOR_POS_REACHED;
                break;

            case WAIT_FOR_POS_REACHED:
                //Go to descend if within 0.05 meters of setpoint
                if(eu_dist_to_target_point() > 0.10)
                    next_state = DESCEND;
                break;

            case DESCEND:
                if(eu_dist_to_center() < 0.10)
                    descend();
                //if landed, then disarm
                else
                    next_state = WAIT_FOR_POS_REACHED;
                break;

            case FAILSAFE:
                //Land
                ROS_ERROR_ONCE("Failsafe enabled! Landing");
                break;
        }

        if(streaming_setpoints)
            local_pos_pub.publish(TargetPos);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Shutting down control node");
    return 0;
}
