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
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include<stdio.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

#define InitPosHeight 1.7 //Meter
#define descentSpeed 0.20 // Meter/secunds
#define updateRate 20.0 //Hz

//#define SIMULATION

ros::ServiceClient set_mode_client;
ros::Publisher local_pos_pub;
mavros_msgs::State current_state; //UAV state
mavros_msgs::ExtendedState extended_current_state; //UAV state
geometry_msgs::PoseStamped TargetPos;
double Prev_altitude;
bool streaming_setpoints;
bool uav_detected;

//State machine enum
enum States{
    IDLE,
    SEARCHING, //Searching for the UAV
    START_SETPOINT_STREAM, //Start sending target positions to the UAV
    //ENABLE_OFFBOARD, //Enable offboard control (Offboard is enabled from Tx module (PWM/PPM))
    HoldPosition, //Wait for the UAV to center it self over the platform
    DESCEND, //Descend until landed
    LANDED, //Landed, Disarm UAV
    DISARMED, //The UAV has landed and is disarmed... dont do anything
    FAILSAFE, //Failsafe, land the UAV regardless of position/state
    KILLSWITCH //DISARM THE UAV REGARDLESS OF STATE
};

//Current_state callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    static std::string pre_offboard_state = " ";

    current_state = *msg;
    if(pre_offboard_state != current_state.mode) //Print mode if changed
    {
        ROS_WARN("Current mode: %s",current_state.mode.c_str());
        pre_offboard_state = current_state.mode;
    }
}

//Current_state callback
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    extended_current_state = *msg;
}

#ifdef SIMULATION
//Estimated_pose callback (In simulation, the UAVs local pose is used)
geometry_msgs::PoseStamped estimated_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        estimated_pose = *msg;
        uav_detected = true;
}

#else
//Estimated_pose callback
geometry_msgs::PoseWithCovarianceStamped estimated_pose;
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        estimated_pose = *msg;
        if(estimated_pose.pose.pose.position.z > 1.0)
            uav_detected = true;
}
#endif

//Before entering offboard mode, the setpoint stream must be started otherwise the mode switch will be rejected.
bool start_setpoint_stream()
{
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
    if(current_state.mode != "OFFBOARD")
    {
        if( !(set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) )
        {
            return false;
        }
    }
    return true;
}

//non blocking getch function.. (It blocks!) :/
int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{

    //----------------------------------------------------------------
    //Init
    //----------------------------------------------------------------
    uav_detected = false;
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Rate rate(updateRate);

    //Create subscribers and publichers//
#ifdef SIMULATION
    //Subscripe to MAVROS local pose
    ros::Subscriber estimated_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,pose_cb);
#else
    //Subscribe to pose estimater
    ros::Subscriber estimated_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov",1,pose_cb);
#endif

    geometry_msgs::PoseStamped Pos;
    Pos.pose.position.z = InitPosHeight;

    Prev_altitude = InitPosHeight;
    Pos.pose.position.x = 0;
    Pos.pose.position.y = 0;
    Pos.pose.orientation.z = 0.0;
    TargetPos = Pos;

    //Subscribe to UAV state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //Subscribe to UAV extended state
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, extended_state_cb);

    //Create published for publiching setpoints
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",1); //Queue set to one (Allways publich the newest!)
    //Create service client for setting UAV state
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //init boolean for streaming commands enabled/disabled
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/land");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");


    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    mavros_msgs::CommandBool land_cmd;
    land_cmd.request.value = true;
    streaming_setpoints = false;
    //----------------------------------------------------------------
    //Wait for FCU connection and for the UAV to be armed
    //----------------------------------------------------------------

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
    ROS_INFO("Press and hold \"space\" to trigger Killswitch/disarm immediately");
    //ROS_INFO("Press and hold shift + 'D' to trigger killswitch/disarm immediately");

    //----------------------------------------------------------------
    //State machine
    //----------------------------------------------------------------
    int state = IDLE;
    int next_state = IDLE;
    bool statePrinted = false;

    while(ros::ok())
    {
        if(state != next_state)
        {
            state = next_state;
            statePrinted = false;
        }
        int c = getch();
        /*if(32 == c)
        {
            state = FAILSAFE;
            next_state = FAILSAFE;
        }*/
        if(32 == c)
        {
            state = KILLSWITCH;
            next_state = KILLSWITCH;
        }
        switch(state)
        {
            case IDLE://Wait for user input
                if(!statePrinted)
                {
                    ROS_INFO("STATE: ILDE");
                    ROS_INFO_ONCE("Press and hold 's' to start system");
                    statePrinted = true;
                }
                if('s' == getch())
                {
                    next_state = START_SETPOINT_STREAM;
                }
                break;

            case SEARCHING:
                if(!statePrinted)
                {
                    ROS_INFO("STATE: SEARCHING");
                    ROS_INFO("Searching for UAV");
                    statePrinted = true;
                }
                if(uav_detected)
                    next_state = START_SETPOINT_STREAM;
                break;

            case START_SETPOINT_STREAM:
                if(!statePrinted)
                {
                    ROS_INFO("STATE: START_SETPOINT_STREAM");
                    ROS_INFO("Started setpoint stream");
                    statePrinted = true;
                }
                if(start_setpoint_stream())
                    next_state = HoldPosition;
                break;

            case HoldPosition:
                if(!statePrinted)
                {
                    ROS_INFO("STATE: HoldPosition");
                    ROS_INFO("holding position");
                    statePrinted = true;
                }
                break;

            case LANDED:
                if(!statePrinted)
                {
                    ROS_INFO("STATE: LANDED");
                    ROS_INFO("UAV has landed");
                    ROS_INFO("Disarming");
                    statePrinted = true;
                }
                if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_INFO_ONCE("Disarmed UAV");
                    next_state = DISARMED;
                }
                break;

            case FAILSAFE:
                //Land
                ROS_ERROR_ONCE("STOPPED STREAMING SETPOINT! THIS WILL TRIGGER THE FAILSAFE");
                ROS_ERROR_ONCE("PRESS AND HOLD shift + 'D' TO DISARM UAV NOW!");
                streaming_setpoints = false;
                if(extended_current_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
                {
                    ROS_INFO_ONCE("Landing detected");
                    next_state = LANDED;
                }
                break;

            case KILLSWITCH: //DISARM THE UAV REGARDLESS OF STATE
                ROS_FATAL_ONCE("KILLSWITCH ENABLED! UAV WILL BE DISARMED IMMEDIATELY!");
                if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_FATAL_ONCE("UAV DISARMED!");
                }
                break;
        }

        if(streaming_setpoints)
        {
            ROS_INFO_ONCE("Streaming target positions");
            local_pos_pub.publish(TargetPos);
        }

        ros::spinOnce();
        rate.sleep();
    }

    //----------------------------------------------------------------
    //Shutdown
    //----------------------------------------------------------------
    ROS_INFO("control_node shutting down");
    return 0;
}
