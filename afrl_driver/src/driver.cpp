/**************************************************************************************
 * Author: Daniel R Madison
 * Last Updated: 04/2/2016
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"
#include <vector>

#include "std_msgs/UInt8.h"
#include "afrl_driver/driver.h"

using namespace std;
// Consstruct a new RandomWalk object and hook up this ROS node
// to the simulated robot's velocity control and laser topics
AFRL_Driver::AFRL_Driver(ros::NodeHandle& nh) {
    PIDTimer = nh.createTimer(ros::Duration(0.1), &AFRL_Driver::PID_control, this);
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 
    laserSub = nh.subscribe("/base_scan", 1, &AFRL_Driver::commandCallBack, this);
    bumperSub = nh.subscribe("/mobile_base/events/bumper", 1, &AFRL_Driver::bumperCallBack, this);

    commsSub = nh.subscribe("/drone_comms", 1, &AFRL_Driver::commsCallBack, this);
    //  commsPub = nh.advertise<std_msgs::UInt8>("/ground_comms", 1);

    tagState = TAG_NULL;
    this->error_sum = 0;
    this->control = 0;
    ros::spinOnce();
}

void AFRL_Driver::commsCallBack(std_msgs::UInt8 comm) {

    int s = comm.data;
    if(this->tagState != s)
        ROS_INFO_STREAM("TAG STATE CHANGED: " << this->tagState);
    switch(s) {
        case TAG_NULL:
            this->setTagState(TAG_NULL);
            break;
        case TAG_FOUND:
            this->setTagState(TAG_FOUND);
            break;
        case TAG_LOST:
            this->setTagState(TAG_LOST);
            break;
    }
}


// Check the bumpers for objects and backup to turn around if encountered.
void AFRL_Driver::bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    if(msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        switch(msg->bumper) {
            case kobuki_msgs::BumperEvent::LEFT:
                turnOdom(REVERSE_ANGLE_DEGS, true, true);
                break;
            case kobuki_msgs::BumperEvent::RIGHT:
                turnOdom(REVERSE_ANGLE_DEGS, false, true);
                break;
            case kobuki_msgs::BumperEvent::CENTER:
                turnOdom(REVERSE_ANGLE_DEGS, true, true);
                break;
        }
    }
}
// Process the incoming laser scan message
void AFRL_Driver::commandCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {

    int rangesMin = 0;
    int rangesMax = msg->ranges.size() - 1;

    double midDist = msg->ranges[rangesMax / 2]; // Half of max distance

    // Since the hokuyo sensor has a 180 degree range, we device the max range
    // by a number to get a degree offset. To get the number, just divide your 
    // sensors max angle by the degree you want to pick up from, then replace
    // the result into MSG_ANGLE_OFFSET
    double rightDist = msg->ranges[rangesMin + (rangesMax * MSG_RANGES_OFFSET / MSG_RANGES_ANGLE)];
    double leftDist = msg->ranges[rangesMax - (rangesMax * MSG_RANGES_OFFSET / MSG_RANGES_ANGLE)];

    double cDist = msg->ranges[0];
    int cIndex = 0;

    for(int i = 0; i < rangesMax; i++) {
        if(msg->ranges[i] < cDist) {
            cDist = msg->ranges[i];
            cIndex = i;
           
        }
    } 
    //cout << cDist << "closest dist/n";
/*    if(cDist < PROXIMITY_RANGE_MIN) {
        if(cIndex <= rangesMax / 2 ) {
            turnOdom(REVERSE_ANGLE_DEGS, false, true);
        } else {
            turnOdom(REVERSE_ANGLE_DEGS, true, true);
        }
    }*/


    // If our middle most distances is less then 0.5, then there is something right infront
    // of the bot and we should stop and turn.
    //    if(midDist < PROXIMITY_RANGE_MIN)
    //        turnOdom(REVERSE_ANGLE_DEGS, true, true);

    // if the left or the rihght distance from the laser scan goes to infinity, 
    // then we set this to the greatest scan range possible, otherwise it could
    // make the PID controller go funky. 
    if(isinf(rightDist)) rightDist = PROXIMITY_RANGE_MAX;
    if(isinf(leftDist)) leftDist = PROXIMITY_RANGE_MAX;

    if(DEBUG_MSG == 1) {
        std::cout << "\n##############################################\n";
        ROS_INFO_STREAM("Left Dist: " << leftDist);
        ROS_INFO_STREAM("Right Dist: " << rightDist);
        ROS_INFO_STREAM("Control: " << this->control);
        ROS_INFO_STREAM("Closest Dist: " << cDist);
        ROS_INFO_STREAM("cIndex: " << cIndex);
        std::cout << "##############################################\n";
    }

    // Throw out NANs. You may not want to use this if you use a sensor with a minimum range
    // such as the kinect. TODO add this as a toggleable condition. 
    if(!isnan(leftDist) && !isnan(rightDist)) {
        this->error = leftDist - rightDist;
    }
}


// Send a velocity command
void AFRL_Driver::move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
}


/* Method for turning the robot a certain distance, with the enableing of going
 * backwards and turning either clockwise or counter clockwise. 
 */
bool AFRL_Driver::turnOdom(double angle, bool clockwise, bool backtrack) {
    move(0,0);
    sleep(1);
    PID_clear();

    bool done = false;
    double radians = angle * M_PI/180;

    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    listener_.waitForTransform("base_footprint", "odom", 
            ros::Time(0), ros::Duration(1.0));

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    listener_.lookupTransform("base_footprint", "odom", 
            ros::Time(0), start_transform);

    geometry_msgs::Twist base_cmd;


    backtrack ? base_cmd.linear.x = REVERSE_SPEED_MPS : base_cmd.linear.x = 0.0;
    clockwise ? base_cmd.angular.z = -1 : base_cmd.angular.z = 1; 
    tf::Vector3 desired_turn_axis(0,0,1);
    if (clockwise == false) desired_turn_axis = -desired_turn_axis;

    while (!done && ros::ok()) {
        commandPub.publish(base_cmd);

        listener_.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();

        double angle_turned = relative_transform.getRotation().getAngle();

        if ( fabs(angle_turned) < 1.0e-2) continue;

        if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
            angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned > radians) return true;
    }

    return false;
}


/* Method for calculating all the gains for the PID controller and based on the resulting error,
 * determine the control variable of where the robot needs to adjust.
 */
void  AFRL_Driver::PID_control(const ros::TimerEvent& e) {

    ros::Time now = ros::Time::now();
    ros::Duration dt = now - this->prev;

    this->error_sum += this->error * dt.toSec();

    double dError = (this->error - this->prev_error) / dt.toSec();

    // Calculate error of the PID controller.
    double sig = (this->error * Kp) + (error_sum * Ki) + (dError * Kd);

    // Based on the calculated error in relation to the threshold, decide
    // whether to increase, decrease, or reset our control. Resetting the control
    // happens when the error's sign is suddenly flipped. The control is also
    // limited to be between a max rotation speed, else it begins spinning to quickly
    // and can't recover. 
    if (sig > CONTROL_THRESHOLD)
        prev_sig < 0 ? control = 0 : control += ROTATE_CONTROL_STEP; 
    else if (sig < CONTROL_THRESHOLD) 
        prev_sig > 0 ? control = 0 : control -= ROTATE_CONTROL_STEP;
    else
        control = 0;

    // Store the previous total error. Used for when the error suddenly flips 
    // signs and the control needs to be reset to the base of 0.
    this->prev_sig = sig;
    this->prev = now;
    this->prev_error = this->error;
}

void AFRL_Driver::PID_clear() {
    this->prev_sig = 0;
    this->prev_error = 0;
    this->error_sum = 0;
    this->control = 0;
}

/*  Main spin method. Drives the robot forward by default and activates the PID controller. 
 *  Various triggers in the call backs will pause the loop and have the robot rotate or 
 *  reverse its direction.
 */
void  AFRL_Driver::spin() {
    ros::Rate rate(15); // Specify the looprate in Hz
this->PID_clear();
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

        if(TAG_FINDING) {
            if(this->tagState == TAG_NULL || this->tagState == TAG_LOST) {
                this->PID_clear();
                this->move(0, 0);
            } else if(this->tagState == TAG_FOUND)  
                move(FORWARD_SPEED_MPS, control);
        } else {
            move(FORWARD_SPEED_MPS, control);
        }

        ros::spinOnce(); // Need to call this function often to allow ROS to process incomingmessages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "AFRL_Driver");
    ros::NodeHandle n;
    AFRL_Driver walker(n); 
    walker.spin(); 
    return 0;
}
