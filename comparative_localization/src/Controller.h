/************************************************************************
* Controller header file.
* The Controller class controls the movement of the AR.Drone through tum_ardrone
* Publishes to the rostopic /tum_ardrone/com
* 
* Authors: Shannon Hood, Patrick Hamod, Kelly Benson
* Last Modified: 09/09/16
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
// #include <ar_track_alvar_msgs/AlvarMarker.h>
#include "nav_msgs/Odometry.h"
#include <vector>
#include <ctime>
#include <sstream>
#include <math.h>
#include <tf/transform_listener.h>
#define NO_TAG_FOUND 0
#define FOUND_TAG 1
#define LOST_TAG 2

class Controller {
  public:     
    //constructor to interact with the rosnode
    Controller(ros::NodeHandle& nh);
   
    //callback functions
    void positionCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void cameraCallback(const std_msgs::Bool::ConstPtr& isBottomCam);
    void turtlebotCallback(const nav_msgs::Odometry::ConstPtr& msg); 

    //misc. functions
    void shutDown();
          
          
  private:
    //subscribers
    ros::Subscriber positionsub;
    ros::Subscriber bottomimage;
    ros::Subscriber cameraInUse;
    ros::Subscriber turtlebotPose;
    //ros::Subscriber frontimage;

    //publishers
    ros::Publisher commands;
    ros::Publisher cameraPub;
    ros::Publisher drone_comms;

    ros::ServiceClient flattrim;
    ros::ServiceClient autoPilot;
    ros::ServiceClient kbControl;
    ros::ServiceClient clearCommands;
    std_srvs::Empty srv;
   
    //bool val only look for tag if using bottom cam
    bool currentCam;
    bool tagDetected;
    static const int BOTTOMCAM = 1;
    static const int FRONTCAM = 0;
    double lastSeen;
   
    //pose of the tag from ar_track_alvar
    geometry_msgs::Point tagPoint;
    double tagYaw;

    //pose of turtlebot from GVG
    geometry_msgs::Point tbPoint;
    double tbYaw;

    void initialize();
};
