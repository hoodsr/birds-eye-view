#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/CliffEvent.h"
#include "kobuki_msgs/BumperEvent.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class RandomWalk {
    public:
        // Construst a new RandomWalk object and hook up this ROS node
    // to the simulated robot's velocity control and laser topics
    RandomWalk(ros::NodeHandle& nh) :
    fsm(FSM_MOVE_FORWARD),
    rotateStartTime(ros::Time::now()),
    rotateDuration(0.f) {
        // Initialize random time generator
        srand(time(NULL));
        isCliff = isObstacle = false;
        printTime = ros::Time::now();
        commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 
        laserSub = nh.subscribe("/base_scan", 1, &RandomWalk::commandCallback, this);
        bumperSub = nh.subscribe("/mobile_base/events/bumper", 1, &RandomWalk::bumperCallBack, this);
        cliffSub = nh.subscribe("/mobile_base/events/cliff", 1, &RandomWalk::cliffCallback, this);
    };

    void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
      if(msg->state == 1) {
        isCliff = true;
      } else {
        isCliff = false;
      }
    };

    void bumperCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
        if(msg->state == kobuki_msgs::BumperEvent::PRESSED) {
            isObstacle = true;
        } else {
            isObstacle = false;
        }
    };

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS) {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    };

    // Process the incoming laser scan message
    void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (fsm == FSM_MOVE_FORWARD) {
        // Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
        unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
        unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
        float closestRange = msg->ranges[minIndex];
        float currAngle;
        for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
            currAngle = msg->angle_min + msg->angle_increment*currIndex;
            if (msg->ranges[currIndex] < closestRange) {
                if(currAngle <= msg->angle_max) { // check to ensure indices are not out of bounds
                    closestRange = msg->ranges[currIndex];
               }
            }  
        }

        if(printTime.toSec() > ros::Time::now().toSec()+3) {
            ROS_INFO_STREAM("Range: " << closestRange);
            printTime = ros::Time::now();
        }
        // TODO: if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime,
        // and also choose a reasonable rotateDuration (keeping in mind of the value
        // of ROTATE_SPEED_RADPS)
        // HINT: you can set a ros::Duration by calling:
        //
        // - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)
        if(closestRange < PROXIMITY_RANGE_M || isCliff || isObstacle) {
            fsm = FSM_ROTATE;
            rotateStartTime = ros::Time::now();
            // Set rotate duration based on ROTATE_SPEED_RADPS = PI/2
            rotateDuration = ros::Duration(rand() % 8); 
           // float random = (rand()%21-10)*M_PI/180;
            //ros::Duration epsilon = ros::Duration(random*1/(ROTATE_SPEED_RADPS));
            //rotateDuration = ros::Duration(10/3);
            rotateCliff = ros::Duration(rand() % 4 + 1);
        } 
        }
    };

    // Main FSM loop for ensuring that ROS messages are
    // processed in a timely manner, and also for sending
    // velocity controls to the simulated robot based on the FSM state
    void spin() {
    ros::Rate rate(10); // Specify the FSM loop rate in Hz
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
        if(fsm == FSM_MOVE_FORWARD && !isCliff && !isObstacle) {    
       // if(fsm == FSM_MOVE_FORWARD) {
            move(FORWARD_SPEED_MPS, 0);
        } else {
            if(isCliff || isObstacle) {
                move(BACKWARD_SPEED_MPS, 0);
                move(BACKWARD_SPEED_MPS, 0);
                while(ros::Time::now() <= rotateStartTime+rotateCliff) {
                    move(0, ROTATE_SPEED_RADPS);
                }
            } else if(ros::Time::now() > rotateStartTime+rotateDuration) {
                fsm = FSM_MOVE_FORWARD;
            } else { //if(ros::Time::now() < rotateStartTime+rotateDuration)               
                usleep(1000*1000);
                move(0, ROTATE_SPEED_RADPS);
            }
        }
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }
    }; enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};

    // Tunable parameters
    // TODO: tune parameters as you see fit
    const static double MIN_SCAN_ANGLE_RAD = -20.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +20.0/180*M_PI;
    const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static double FORWARD_SPEED_MPS = 0.17;
    const static double BACKWARD_SPEED_MPS = -0.2;
    const static double ROTATE_SPEED_RADPS = M_PI/6;

    protected:
    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic

    ros::Subscriber cliffSub;
    ros::Subscriber bumperSub;
    bool isCliff;
    bool isObstacle;
    ros::Time printTime;

    enum FSM fsm; // Finite state machine for the random walk algorithm
    ros::Time rotateStartTime; // Start time of the rotation
    ros::Duration rotateDuration; // Duration of the rotation
    ros::Duration rotateCliff;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
    ros::NodeHandle n;
    RandomWalk walker(n); // Create new random walk object
    walker.spin(); // Execute FSM loop
    return 0;
};
