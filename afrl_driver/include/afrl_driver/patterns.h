/****************************************
 * Author: Daniel R. Madison
 *
 * Public Liscense: 
 *
 ***************************************/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "kobuki_msgs/BumperEvent.h"

#ifndef AFRL_PATTERNS_H
#define AFRL_PATTERNS_H

#define DEBUG_MSG 1

// Robot movement 
const static double FORWARD_SPEED_MPS = 0.2; // Forward speed.
const static double REVERSE_SPEED_MPS = -0.1; // Reverse speed. Must be negative.
const static double REVERSE_ANGLE_DEGS = 45;
const static double ROTATE_CONTROL_STEP = 0.05; // step at which to increment the control of the PID

using namespace std;

class Pattern_Driver {
    public:
        // Constructor and Destructor
        Pattern_Driver(ros::NodeHandle& nh);

        // Callback Functions
        void bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg);
      
        // Movement Functions
        bool turn(double angle);
        bool forward(double dist);
        void stop();

        // Patterns
        void Star();
        void Square();
        void Rectangle();
        void Triangle();

        // ROS Functions
        void spin();
       
    private:
        int bumperState;
 
    protected:
        ros::Publisher commandPub; // Publisher to the robot's velocity command topic
        ros::Subscriber bumperSub; // Subscriber to the robot's bump sensors topic.
        tf::TransformListener listener_; // Listener for the transform topic
};

#endif

