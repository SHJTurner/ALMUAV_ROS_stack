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
#include<stdio.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

#define InitPosHeight 1.5 //Meter
#define descentSpeed 0.20 // Meter/secunds
#define updateRate 20.0 //Hz

#define SIMULATION

ros::ServiceClient set_mode_client;
ros::Publisher local_pos_pub;
mavros_msgs::State current_state; //UAV state
geometry_msgs::PoseStamped TargetPos;
double Prev_altitude;
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

#ifdef SIMULATION
//Estimated_pose callback (In simulation, the UAVs local pose is used)
geometry_msgs::PoseStamped estimated_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        estimated_pose = *msg;
}

#else
//Estimated_pose callback
geometry_msgs::PoseWithCovarianceStamped estimated_pose;
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        estimated_pose = *msg;
}
#endif

//Moves UAV to center over platform at a minimum altitude of InitPosHeight (otherwise the altitude is holded)
void setpoint_center()
{
#ifdef SIMULATION
    if(estimated_pose.pose.position.z < InitPosHeight)
#else
    if(estimated_pose.pose.pose.position.z < InitPosHeight)
#endif
    {
        geometry_msgs::PoseStamped Pos;
        Pos.pose.position.z = InitPosHeight;

        Prev_altitude = InitPosHeight;
        Pos.pose.position.x = 0;
        Pos.pose.position.y = 0;
        Pos.pose.orientation.z = 0.0;
        TargetPos = Pos;
    }
    else
    {
        geometry_msgs::PoseStamped Pos;
#ifdef SIMULATION
        Pos.pose.position.z = estimated_pose.pose.position.z;
        Prev_altitude = estimated_pose.pose.position.z;
#else
        Pos.pose.position.z = estimated_pose.pose.pose.position.z;
        Prev_altitude = estimated_pose.pose.pose.position.z;
#endif
        Pos.pose.position.x = 0;
        Pos.pose.position.y = 0;
        Pos.pose.orientation.z = 0.0;
        TargetPos = Pos;
    }
}

//Before entering offboard mode, the setpoint stream must be started otherwise the mode switch will be rejected.
bool start_setpoint_stream()
{
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

//Slowly descend the UAV (Fuction must be calle with "updateRate" Rate)
void descend()
{
    geometry_msgs::PoseStamped Pos;
    Pos.pose.position.z = Prev_altitude - (descentSpeed/updateRate);
    Prev_altitude = Prev_altitude - (descentSpeed/updateRate);
    Pos.pose.position.x = 0;
    Pos.pose.position.y = 0;
    Pos.pose.orientation.z = 0.0;
    TargetPos = Pos;
}

//Calculate euclidean distance to center (X-/Y-plane)
double eu_dist_to_center()
{
#ifdef SIMULATION
    return sqrt( ( pow((TargetPos.pose.position.x-estimated_pose.pose.position.x),2) + pow((TargetPos.pose.position.y-estimated_pose.pose.position.y),2) )  );
#else
    return sqrt( ( pow((TargetPos.pose.position.x-estimated_pose.pose.pose.position.x),2) + pow((TargetPos.pose.position.y-estimated_pose.pose.pose.position.y),2) )  );
#endif
}

//Calculate euclidean distance to target (X-/Y-/Z-plane)
double eu_dist_to_target_point()
{
#ifdef SIMULATION
    //ROS_INFO("dist_to_target: %f",sqrt( ( pow((TargetPos.pose.position.x-estimated_pose.pose.position.x),2) + pow((TargetPos.pose.position.y-estimated_pose.pose.position.y),2) + pow((TargetPos.pose.position.z-estimated_pose.pose.position.z),2) )  ));
    return sqrt( ( pow((TargetPos.pose.position.x-estimated_pose.pose.position.x),2) + pow((TargetPos.pose.position.y-estimated_pose.pose.position.y),2) + pow((TargetPos.pose.position.z-estimated_pose.pose.position.z),2) )  );
#else
    return sqrt( ( pow((TargetPos.pose.position.x-estimated_pose.pose.pose.position.x),2) + pow((TargetPos.pose.position.y-estimated_pose.pose.pose.position.y),2) + pow((TargetPos.pose.position.z-estimated_pose.pose.pose.position.z),2) )  );
#endif
}

//non blocking getch function
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char **argv)
{

    //----------------------------------------------------------------
    //Init
    //----------------------------------------------------------------

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

    //Subscribe to UAV state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //Create published for publiching setpoints
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",1); //Queue set to one (Allways publich the newest!)
    //Create service client for setting UAV state
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //init boolean for streaming commands enabled/disabled
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

    //----------------------------------------------------------------
    //State machine
    //----------------------------------------------------------------
    int state = IDLE;
    int next_state = IDLE;
    bool emergency_stop = false;
    bool statePrinted = false;

    while(ros::ok())
    {
        if(state != next_state)
        {
            state = next_state;
            statePrinted = false;
        }

        if(emergency_stop)
        {
            state = FAILSAFE;
            next_state = FAILSAFE;
        }
        switch(state)
        {
            case IDLE:
                //DONT DO ANYTHING
                ROS_INFO_ONCE("Idle (press 's' to start system)");
                if('s' == getch())
                {
                    ROS_INFO("Searching for UAV");
                    next_state = START_SETPOINT_STREAM;
                }
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
                if(eu_dist_to_target_point() < 0.30)
                    next_state = DESCEND;
                break;

            case DESCEND:
                if(eu_dist_to_center() < 0.30)
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
    ROS_INFO("Shutting down control node");
    return 0;
}
