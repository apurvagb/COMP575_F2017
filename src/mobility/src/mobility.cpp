#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>
#define NO_OF_ROVERS 6
using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;



string rover_name;
char host[128];
bool is_published_name = false;


int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;



//Structure for Rover Pose
struct rover_location
{
    string rovername;
    float x_loc;
    float y_loc;
    float theta;
};

std_msgs::String global_avg_msg;
std_msgs::String local_avg_msg;
struct rover_location location_struct[NO_OF_ROVERS];
float global_avg =0.0;
float local_avg =0.0;

float average_x_position_algnmnt = 0.0;
float average_y_position_algnmnt =0.0;
float average_x_position_sep =0.0;
float average_y_position_sep =0.0;
float sin_theta_sum_loc =0.0;
float cos_theta_sum_loc=0.0;
float direction_theta;
float final_theta;
float desired_x;
float desired_y;
// Proportional constant kp
float Kp;


// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher posePublish;
ros::Publisher global_average_headingPublish;
ros::Publisher local_average_headingPublish;
//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;

ros::Subscriber messageSubscriber;
ros::Subscriber poseSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); //
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
void poseHandler(const std_msgs::String::ConstPtr &message);



int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);
    cout<<"yES"<<endl;
    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selectaverage_x_positioned. Default is: " << rover_name << endl;
    }
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    poseSubscriber = mNH.subscribe(("poses"), 10, poseHandler);
    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);
    posePublish = mNH.advertise<std_msgs::String>(("poses"), 10 , true);
    global_average_headingPublish = mNH.advertise<std_msgs::String>(("global_average_heading"), 10 , true);
    local_average_headingPublish = mNH.advertise<std_msgs::String>(( "local_average_heading"), 10 , true);
    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;
    std_msgs::String pose_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case STATE_MACHINE_TRANSLATE:
        {
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();

            float angular_velocity;
            float linear_velocity = 0.05;

            Kp= 0.5;

            // Add all positions: cohesion, alignment and dseparation
            desired_x = cos_theta_sum_loc + average_x_position_algnmnt + average_x_position_sep;
            desired_y = sin_theta_sum_loc + average_y_position_algnmnt+ average_y_position_sep;
            final_theta = atan2(desired_y,desired_x);
            //angular_velocity = Kp * (global_avg - current_location.theta);
            //angular_velocity = Kp * (local_avg - current_location.theta);
            //angular_velocity = Kp * (direction_theta - current_location.theta);

            angular_velocity = Kp * (final_theta - current_location.theta);
            setVelocity(linear_velocity, angular_velocity);
            break;
        }
        default:
        {
            state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT autor

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
    stateMachinePublish.publish(state_machine_msg);

    //convert to string
    std::stringstream converter;
    converter << rover_name << "," << current_location.x << "," << current_location.y << "," << current_location.theta;
    pose_msg.data = converter.str();
    //Publish the location
    posePublish.publish(pose_msg);

}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();
    float linear_velocity = 0.1;
    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr& message)
{
}
//Function to get rover index
int getRoverNameIndex(std::string str)
{
    int index;
    // ajax, aeneas,achilles,diomedes,hector,paris
     if(str.compare("ajax") ==0)
     {
        index=0;
     }
     else if(str.compare("aeneas") ==0)
     {
        index=1;
     }
     else if(str.compare("achilles") ==0)
     {
        index=2;
     }
     else if(str.compare("diomedes") ==0)
     {
        index=3;
     }
     else if(str.compare("hector") ==0)
     {
        index=4;
     }
     else if(str.compare("paris") ==0)
     {
        index=5;
     }
     else
     {
       /* Do nothing */
     }
     return index;
}
//Function to calculate global average
float Calculate_Global_Avg()
{
    float sin_theta_sum=0.0;
    float cos_theta_sum=0.0;
    float theta_avg=0.0;
    int j;
    for(j=0;j<NO_OF_ROVERS;j++)
    {
        sin_theta_sum += sin(location_struct[j].theta);
        cos_theta_sum += cos(location_struct[j].theta);
        
    }
    
     theta_avg= atan2(sin_theta_sum,cos_theta_sum);
     return theta_avg;
}

//Function to calculate neighbors and local average
float calculate_Neighbors_LocalAvg(int input_index)
{
    int n;
    float dist;
    //float sin_theta_sum_loc =0.0;
    //float cos_theta_sum_loc =0.0;
    bool neighbor_found = 0;
    float theta_avg_loc =0.0;


    for(n=0;n<NO_OF_ROVERS;n++)
    {
        if(input_index!=n)
        {
            dist = sqrt(pow((location_struct[input_index].x_loc-location_struct[n].x_loc),2)+pow((location_struct[input_index].y_loc-location_struct[n].y_loc),2));
            if(dist <= 2)
            {
                /* Cohesion */
                sin_theta_sum_loc += sin(location_struct[n].theta);
                cos_theta_sum_loc += cos(location_struct[n].theta);
                /* Alignment */
                average_x_position_algnmnt +=(location_struct[input_index].x_loc + ((location_struct[n].x_loc-location_struct[input_index].x_loc)/NO_OF_ROVERS));
                average_y_position_algnmnt +=(location_struct[input_index].y_loc+ ((location_struct[n].y_loc-location_struct[input_index].y_loc)/NO_OF_ROVERS));
                neighbor_found = 1;


            }
        }

    }
    if(neighbor_found==0)
    {
       sin_theta_sum_loc = sin(location_struct[input_index].theta);
       cos_theta_sum_loc = cos(location_struct[input_index].theta);
       average_x_position_algnmnt = location_struct[input_index].x_loc;
       average_x_position_algnmnt = location_struct[input_index].y_loc;
    }

    theta_avg_loc = atan2(sin_theta_sum_loc,cos_theta_sum_loc);
    //direction_theta = atan2(average_y_position,average_x_position);
    neighbor_found =0;
    return theta_avg_loc;
}
/* Separation logic */
float Separation_logic(int input_index)
{
    int n;
    float distance;

    for(n=0;n<NO_OF_ROVERS;n++)
    {
        if(input_index!=n)
        {
            distance = sqrt(pow((location_struct[input_index].x_loc-location_struct[n].x_loc),2)+pow((location_struct[input_index].y_loc-location_struct[n].y_loc),2));
            if(distance > 0 && distance <= 3)
            {
                average_x_position_sep +=(location_struct[input_index].x_loc + ((location_struct[n].x_loc-location_struct[input_index].x_loc)))/distance;
                average_y_position_sep +=(location_struct[input_index].y_loc+ ((location_struct[n].y_loc-location_struct[input_index].y_loc)))/distance;

            }
            else
            {
                average_x_position_sep = location_struct[input_index].x_loc;
                average_y_position_sep = location_struct[input_index].y_loc;
            }
        }

    }
}

void poseHandler(const std_msgs::String::ConstPtr& message)
{
   // int counter_substr =0;
    int rover_index;
    std::string message_copy;
    std::string token_name;
    std::string token_x;
    std::string token_y;
    std::string token_theta;

    // Take copy of every message
    message_copy = message->data;
    
    //Split the message string using comma delimiter
    std::istringstream ss(message_copy);
    std::getline(ss, token_name, ',');
    std::getline(ss, token_x, ',');
    std::getline(ss, token_y, ',');
    std::getline(ss, token_theta, ',');

    /* Check the name of rover and store the message in rover structre */
   rover_index = getRoverNameIndex(token_name);

   location_struct[rover_index].rovername = token_name;
   location_struct[rover_index].x_loc = ::atof(token_x.c_str());
   location_struct[rover_index].y_loc = ::atof(token_y.c_str());
   location_struct[rover_index].theta = ::atof(token_theta.c_str());


    global_avg = Calculate_Global_Avg();
    local_avg = calculate_Neighbors_LocalAvg(rover_index);
    //Added separation logic
    Separation_logic(rover_index);
    //Form String to publish for global average
    std::stringstream converter_1;
    converter_1 <<"global_average"<<":" << global_avg;

    //converter_1 <<"global_average"<<":" << rover_index<<","<<token_name;

    global_avg_msg.data = converter_1.str();

    //Form String to publish for local average
    std::stringstream converter_2;
    converter_2 <<"local_average"<<":" << location_struct[rover_index].rovername<<"," << local_avg;
    local_avg_msg.data = converter_2.str();

    global_average_headingPublish.publish(global_avg_msg);
    local_average_headingPublish.publish(local_avg_msg);


}



