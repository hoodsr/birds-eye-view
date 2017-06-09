/****************************************
 * Author: Daniel R. Madison
 *
 * Public Liscense: 
 *
 ***************************************/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"
#include <vector>
#include "std_msgs/UInt8.h"

#ifndef AFRL_DRIVER_H
#define AFRL_DRIVER_H

#define DEBUG_MSG 1


// TAG PARAMETERS
enum TAG_STATES {
    TAG_NULL,
    TAG_FOUND,
    TAG_LOST
};

static const bool TAG_FINDING = false;

// PID Controller and tuning paramaters
const static double Kp = 0.8; // Proportional gain
const static double Kd = 0.5; // Derivative gain
const static double Ki = 0.05; // Integral gain
const static double CONTROL_THRESHOLD = 0.05;

// Max distance of the laser scan topic. In Meters
//const static double PROXIMITY_RANGE_MAX = 5.9;
const static double PROXIMITY_RANGE_MAX = 20;
// Closet distance you want the laserscan topic to record before issueing a backup command. In meters.
const static double PROXIMITY_RANGE_MIN = 0.1;

// Robot movement 
const static double FORWARD_SPEED_MPS = 0.15; // Forward speed.
const static double REVERSE_SPEED_MPS = -0.1; // Reverse speed. Must be negative.
const static double REVERSE_ANGLE_DEGS = 45;
const static double ROTATE_CONTROL_STEP = 0.01; // step at which to increment the control of the PID

const static int MSG_RANGES_ANGLE = 180; // Max Angle of the sensor. 
//const static int MSG_RANGES_ANGLE = 270; // Max Angle of the sensor. 
const static int MSG_RANGES_OFFSET = 45; // Offset angle at which to take data from.
//const static int MSG_RANGES_OFFSET = 65; // Offset angle at which to take data from.

using namespace std;

class AFRL_Driver {
    public:
        // Constructor and Destructor
        AFRL_Driver(ros::NodeHandle& nh);

        // Callback Functions
        void bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        void commandCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void commsCallBack(std_msgs::UInt8 comm);
        // Movement Functions
        bool turnOdom(double radians, bool clockwise, bool backtrack);
        void move(double linearVelMPS, double angularVelRadPS);

        // PID Functions
        void PID_control(const ros::TimerEvent& e);
        void PID_clear();

        // TAG Functions
        void setTagState(int s) { this->tagState = s; }
        
        // ROS Functions
        void spin();
        void plotStart();
        void plotIntersection();

    private:
        int bumperState;
        double control;
        double prev_sig;
        double error;
        double prev_error;
        double error_sum;

        int tagState;
 
    protected:
        ros::Publisher commandPub; // Publisher to the robot's velocity command topic
        ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
        ros::Subscriber bumperSub; // Subscriber to the robot's bump sensors topic.
        tf::TransformListener listener_; // Listener for the transform topic
        
        ros::Subscriber commsSub; // Communication subscriber
        //ros::Publisher commsPub; // Communication publisher

        ros::Time prev;
        ros::Duration dt;
        ros::Time rotateStartTime; // Start time of the rotation
        ros::Duration rotateDuration; // Duration of the rotation
        ros::Timer PIDTimer;
};

#endif

