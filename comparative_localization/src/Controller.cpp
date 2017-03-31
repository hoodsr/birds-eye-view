/********************************************************************************************
* Controller class
* This class is designed to track a tag using ar_track_alvar and move ar drone parrot 
* and communicates with turtlebot about tag detection status
* 
**/

#include "Controller.h"
using namespace std;

Controller::Controller(ros::NodeHandle& nh) {
  //initializes the boolean to keep track of tag spotting
  tagDetected = false;

  //publisher initialization
  commands = nh.advertise<std_msgs::String>("/tum_ardrone/com", 1);
  cameraPub = nh.advertise<std_msgs::String>("/ardrone/cameraswap",1);
  drone_comms = nh.advertise<std_msgs::UInt8>("/drone_comms",1); //not used now
 
  //ros services
  flattrim = nh.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
  kbControl = nh.serviceClient<std_srvs::Empty>("/ar_drone/kb_control");
  autoPilot = nh.serviceClient<std_srvs::Empty>("/drone_autopilot/start");
  clearCommands = nh.serviceClient<std_srvs::Empty>("/drone_autopilot/clearCommands");

  //ros subscribers
  positionsub = nh.subscribe("/ar_pose_marker",      100000, &Controller::positionCallback, this);
  cameraInUse = nh.subscribe("/ardrone/cameraInUse", 100,    &Controller::cameraCallback,   this);
  //TODO: odom topic might change, definitely need tf
  turtlebotPose = nh.subscribe("/odom", 100,    &Controller::turtlebotCallback,   this);
 
  std_srvs::Empty srv;
  usleep(1000*1000);//wait 1 second to ensure commands are published 
  flattrim.call(srv);
  autoPilot.call(srv);
  usleep(100);
  initialize();
}

/******************************************************************************
* Function: shutDown
* Input: none
*
* Called on exit. Clear all drone commands and land the drone
*/
void Controller::shutDown() {
  clearCommands.call(srv);
  string s = "";
  std_msgs::String command;
  
  s ="c land";
  command.data = s;
  commands.publish(command);
    
}

/******************************************************************************
* Function: initialize
* Input: none
*
* Makes the drone take off and sets variables used by the tum_ardrone
* package. To do this we publish to /tum_ardrone/com using a std_msgs::String 
* message type.
*/
void Controller::initialize() {
  string s ="";
  std_msgs::String command;
  
  //these four lines let the drone take off with presets found in tum_ardrone
  s = "c autoInit 500 800 4000 1.0"; //changed from 500 800 4000 1.0
  command.data = s;
  commands.publish(command);
  //need to wait five seconds for ptams
  usleep(5000*1000);

  //sets the position of the robot so movement is relative to original position
  s = "c setReference $POSE$";
  command.data = s;
  commands.publish(command);
  usleep(1000);

  //this command ranges 0-1, 1 has more control and slows the speed of blades
  s = "c setMaxControl 1.0";
  command.data = s;
  commands.publish(command);
  usleep(1000);

  //sets the distance (in meters) before drone reaches its destination
  s = "c setInitialReachDist 0.15"; //changed from .20
  command.data = s;
  commands.publish(command);
  usleep(1000);
 
  //sets the distance (in meters) the drone needs to stay in once dest reached 
  s = "c setStayWithinDist 0.35"; //TEST changed from 0.50
  command.data = s;
  commands.publish(command);
  usleep(1000);
  
  //sets how long the drone should stay at the dest when reached
  s = "c setStayTime 0.6"; //TEST changed from 2.0
  command.data = s;
  commands.publish(command);
  usleep(100);

  //sets fixpoint when scaling map requires ptams running
  s = "c lockScaleFP";
  command.data = s;
  commands.publish(command);
  usleep(100);

  usleep(1000*1000); //wait 1 second to ensure commands are published 

  /*std::cout << "{INFO}: Flying up 1.5 meters" << std::endl;
  s = "c moveBy 0.0 0.0 1.5 0.0"; //test raising by 2 meters. drone follows to close to TB in x dim.
  command.data = s;
  commands.publish(command);
  usleep(100);*/

  //start switching the cameras after ptam has started
  s = "start";
  command.data = s;
  cameraPub.publish(command);
  usleep(100);

  //let turtlebot know to start
  std_msgs::UInt8 comm;
  comm.data = NO_TAG_FOUND;
  drone_comms.publish(comm);
  usleep(100);

  std::cout << "{INFO}: AR.Drone initialization finished" << std::endl;
}

/**************************************************************************************
* Function: positionCallback
* Input: ar_track_alvar msg
* TODO: find the tag we want to follow and only use that tag
* 
* If the drone is using its bottom camera, then it will move towards the tag and 
* maintain its height.
*/
void Controller::positionCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if(currentCam == BOTTOMCAM) {
    std_msgs::UInt8 comm;
    if(!msg->markers.empty()) { 
      if(!tagDetected) {
        autoPilot.call(srv);
        usleep(100);
        
        /*s << "c clearCommands";
        command.data = s.str();
        commands.publish(command);
        usleep(100);
        s.flush();*/

        tagDetected = true;
      }

      std::cout << "{INFO}: Tag found by ar_track_alvar" << std::endl;

      tagPoint = msg->markers.at(0).pose.pose.position;
      geometry_msgs::Quaternion qtn = msg->markers.at(0).pose.pose.orientation; 
      
      tagYaw = tf::getYaw(qtn)*180.0/M_PI+90;
      ostringstream s;
      std_msgs::String command;

      //lets turtlebot know a tag is found
      comm.data = FOUND_TAG;
      drone_comms.publish(comm);
      usleep(100);
      s.flush();      
      
      lastSeen = ros::Time::now().toSec();
      
      //TEST changed from moveBy to moveByRel. look into modifying z to fixed height
      s << "c moveByRel " << tagPoint.x << " " << tagPoint.y << " " << tagPoint.z + 1 << " " << tagYaw;
      command.data = s.str();
      commands.publish(command);
      usleep(100);
      s.flush();
 
      /*s << "c setReference $POSE$";
      command.data = s.str();
      commands.publish(command);
      usleep(100);
      s.flush();*/
    }
    
    //if there are no tags detected, switch to keyboard control
    else {
      if(ros::Time::now().toSec()-lastSeen>4) {
        std::cout << "{WARN}: Tag lost by ar_track_alvar" << std::endl;
        comm.data= LOST_TAG;
        drone_comms.publish(comm);
        usleep(100);
      }

      if(tagDetected) {
        tagDetected = false;
       // kbControl.call(srv);
      }

      //move to the turtlebot's last known pose 
      /*if(tbPoint && tbYaw) {
        ostringstream s;
        std_msgs::String command;
        //needs testing for x coord. use fixed height or take TB's and add certain amount??
        s << "c goto " << tbPoint.x << " " << tbPoint.y << " " << tbPoint.z+0.80 << " " << tagYaw;
        command.data = s.str();
        commands.publish(command);
        usleep(100);
        s.flush();
      }*/

    }
  }
}

/******************************************************************************
* Function: turtlebotCallback
* Input: Predicted pose of the TurtleBot from the GVG algorithm
* TODO: Use the TurtleBot's pose when the drone loses sight of the tag
* 
* Inform the drone of the TurtleBot's location
*/
void Controller::turtlebotCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::Point turtlePose = msg->pose.pose.position;
  geometry_msgs::Quaternion turtleQtn = msg->pose.pose.orientation;
  //cout << "TurtleBot Point: " << turtlePose.x << " " << turtlePose.y << " " << turtlePose.z << endl;
  //cout << "TurtleBot Yaw: " << tf::getYaw(turtleQtn)*180.0/M_PI << endl;

  if(!tagDetected) {
    //move to the turtlebot's last known pose 
    /*ostringstream s;
    std_msgs::String command;

    //needs testing for x coord. use fixed height or take TB's and add certain amount??
    s << "c goto " << tbPoint.x << " " << tbPoint.y << " " << 2 << " " << tagYaw;
    command.data = s.str();
    commands.publish(command);
    usleep(100);
    s.flush();*/
  }
}

/******************************************************************************
* Function: cameraCallback 
* Input: std_msgs::BoolConstPtr& isBottomCam
* 
* Checks message data if camera is in use and sets boolean accordingly.
*/
void Controller::cameraCallback(const std_msgs::BoolConstPtr& isBottomCam) {
  if(isBottomCam->data == true) {
    currentCam = true;
  } else {
    currentCam = false;
  }
}

/******************************************************************************
* Function: main
* Input: none required
*
* Runs this node, allowing the AR.Drone to start following tags
*/
main(int argc, char **argv) {
  ros::init(argc, argv, "pilot");
  ros::NodeHandle n;

  Controller pilot(n);
  ros::Rate rate(10);
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  pilot.shutDown();
}
