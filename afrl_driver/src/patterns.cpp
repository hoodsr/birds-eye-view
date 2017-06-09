/**************************************************************************************
 * Author: Daniel R Madison
 * Last Updated: 04/2/2016
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "kobuki_msgs/BumperEvent.h"

#include "afrl_driver/patterns.h"

using namespace std;
// Consstruct a new RandomWalk object and hook up this ROS node
// to the simulated robot's velocity control and laser topics
Pattern_Driver::Pattern_Driver(ros::NodeHandle& nh) {
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 
    bumperSub = nh.subscribe("/mobile_base/events/bumper", 1, &Pattern_Driver::bumperCallBack, this);

    ros::spinOnce();
}

// Check the bumpers for objects and backup to turn around if encountered.
void Pattern_Driver::bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    if(msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        switch(msg->bumper) {
            case kobuki_msgs::BumperEvent::LEFT:
            case kobuki_msgs::BumperEvent::RIGHT:
            case kobuki_msgs::BumperEvent::CENTER:
                turn(REVERSE_ANGLE_DEGS);
                break;
        }
    }
}



void Pattern_Driver::stop() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = 0;
    msg.angular.z = 0;
    commandPub.publish(msg);
}

bool Pattern_Driver::forward(double dist) {
    stop();
    sleep(1);
    bool done = false;
    listener_.waitForTransform("base_footprint", "odom", 
            ros::Time(0), ros::Duration(1.0));

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    listener_.lookupTransform("base_footprint", "odom", 
            ros::Time(0), start_transform);

    geometry_msgs::Twist base_cmd;
    
    base_cmd.linear.x = FORWARD_SPEED_MPS;
    base_cmd.angular.z = base_cmd.linear.y = 0;
    while(!done && ros::ok()) {
        commandPub.publish(base_cmd);
        listener_.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);

        tf::Transform relative_transform = start_transform.inverse() * current_transform;

        double delta = relative_transform.getOrigin().length();
        if(delta > dist)
            done = true;
    }

    return done;
}
/* Method for turning the robot a certain distance, with the enableing of going
 * backwards and turning either clockwise or counter clockwise. 
 */
bool Pattern_Driver::turn(double angle) {

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


    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 1; 
    tf::Vector3 desired_turn_axis(0,0,1);

    while (!done && ros::ok()) {
        commandPub.publish(base_cmd);

        try {
            listener_.lookupTransform("base_footprint", "odom", 
                    ros::Time(0), current_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            break;
        }

        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();

        double angle_turned = relative_transform.getRotation().getAngle();

        if ( fabs(angle_turned) < 1.0e-2) continue;

        if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
            angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned > radians) done = true;
    }

    return done;
}

void Pattern_Driver::Square() {
    std::cout << "SQUARE\n";

    std::cout << "Moving Forward 1 Meter\n";
    forward(1);
    stop();
    sleep(1);   

    std::cout << "Turning 90 degrees\n";
    turn(90);
        stop();
    sleep(1);
    std::cout << "Moving Forward 1 Meter\n";
    forward(1);
        stop();
    sleep(1);
    std::cout << "Turning 90 degrees\n";
    turn(90);
        stop();
    sleep(1);
    std::cout << "Moving Forward 1 Meter\n";
    forward(1);
        stop();
    sleep(1);
    std::cout << "Turning 90 degrees\n";
    turn(90);
        stop();
    sleep(1);
    std::cout << "Moving Forward 1 Meter\n";
    forward(1);
        stop();
    sleep(1);
    std::cout << "Turning 90 degrees\n";
    turn(90);
        stop();
    sleep(1);
}

/*  Main spin method. Drives the robot forward by default and activates the PID controller. 
 *  Various triggers in the call backs will pause the loop and have the robot rotate or 
 *  reverse its direction.
 */
void  Pattern_Driver::spin() {
    ros::Rate rate(15); // Specify the looprate in Hz

    Square();

    ros::spinOnce(); // Need to call this function often to allow ROS to process incomingmessages
    rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Pattern_Driver");
    ros::NodeHandle n;
    Pattern_Driver walker(n); 
    walker.spin(); 
    return 0;
}
