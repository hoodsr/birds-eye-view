/********************************************************************************************
* cl class
* 
* This class is designed to work with the ar drone parrot 2.0 and turtlebot 2
* to provide cooperitive localization of the drone from the turtlebots frame
* of reference using the turtlebots pose and ar_track_alvar tag info
*
* Authors: Shannon Hood
* Last Edited: 23 January 2017
* 
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <vector>
#include <ctime>
#include <sstream>
#include <math.h>

#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
using namespace std;
class cl {
  public:
  cl(ros::NodeHandle& nh) { 
    orbSLAMSub = nh.subscribe("/ORB_SLAM/odom", 10000, &cl::orbSLAMCallback, this);
    tagDataSub = nh.subscribe("/ar_pose_marker", 100000, &cl::tagCallback, this);
    turtlePathPub = nh.advertise<nav_msgs::Path>("/birdseye/cl_turtle_path", 1000);
    dronePathPub = nh.advertise<nav_msgs::Path>("/birdseye/cl_drone_path", 1000);
    orbSLAMPathPub = nh.advertise<nav_msgs::Path>("/birdseye/orb_slam_path", 1000);
  
    //tfServer = new tf::TransformBroadcaster();  
    //tfListener = new tf::TransformListener();
    dronePath.header.frame_id = "map";
    turtlePath.header.frame_id = "map";
  }
  ~cl(void) {
   // if(tfServer)
     // delete tfServer; 
  }



  /********************************************************************************
  * function should transfrom data to turtlebots frame of reference
  * then using the odom data from the turtlebot, calculate the pose of the drone
  */
  void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {  
    listener.waitForTransform("map", "base_link", 
            ros::Time(0), ros::Duration(7.0));
    try { 
      listener.lookupTransform("map", "base_link", ros::Time(0), tfTransform);
    } catch(tf::TransformException &exception) { 
      ROS_ERROR("%s", exception.what());
    }
    turtlePose.header.stamp = ros::Time::now();

    //==== Position ====//
    turtlePose.pose.position.x = tfTransform.getOrigin().x();
    turtlePose.pose.position.y = tfTransform.getOrigin().y();
    turtlePose.pose.position.z = tfTransform.getOrigin().z();
    if(turtlePose.pose.position.z > .5) return;
    turtlePose.pose.orientation.w = tfTransform.getRotation().w();
    turtlePose.pose.orientation.x = tfTransform.getRotation().x();
    turtlePose.pose.orientation.y = tfTransform.getRotation().y();
    turtlePose.pose.orientation.z = tfTransform.getRotation().z();
    //turtlePlan.push_back(turtlePose);
    turtlePath.poses.push_back(turtlePose);

    if(!turtlePath.poses.empty()){
      turtlePath.header.stamp = turtlePath.poses[0].header.stamp;
    }
    turtlePathPub.publish(turtlePath);
    if(!msg->markers.empty()) {
      tagPose = msg->markers[0].pose;
      dronePose = getDronePose(turtlePose);
      dronePath.poses.push_back(dronePose);

      if(!dronePath.poses.empty()){
        dronePath.header.stamp = dronePath.poses[0].header.stamp;
      }
      dronePathPub.publish(dronePath);
    }
  }

  geometry_msgs::PoseStamped getDronePose(geometry_msgs::PoseStamped turtlePose) {
    geometry_msgs::PoseStamped dronePose;
    dronePose = turtlePose;

    dronePose.pose.position.x += tagPose.pose.position.x;
    dronePose.pose.position.y += tagPose.pose.position.y;
    dronePose.pose.position.z += tagPose.pose.position.z+1;

    //might need -90*M_PI/180
    dronePose.pose.orientation.w += tagPose.pose.orientation.w;
    dronePose.pose.orientation.x += tagPose.pose.orientation.x;
    dronePose.pose.orientation.y += tagPose.pose.orientation.y;
    dronePose.pose.orientation.z += tagPose.pose.orientation.z;

    return dronePose;
  }

  void orbSLAMCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    /*listener.waitForTransform("map", "base_link", 
            ros::Time(0), ros::Duration(7.0));
    try { 
      listener.lookupTransform("map", "base_link", ros::Time(0), tfTransform);
    } catch(tf::TransformException &exception) { 
      ROS_ERROR("%s", exception.what());
    }
    turtlePose.header.stamp = ros::Time::now();*/

    //==== Position ====//
    orbSLAMPose.pose.position.x = odom->pose.pose.position.x;
    orbSLAMPose.pose.position.y = odom->pose.pose.position.y;
    orbSLAMPose.pose.position.z = odom->pose.pose.position.z;
    orbSLAMPose.pose.orientation.w = odom->pose.pose.orientation.w;
    orbSLAMPose.pose.orientation.x = odom->pose.pose.orientation.x;
    orbSLAMPose.pose.orientation.y = odom->pose.pose.orientation.y;
    orbSLAMPose.pose.orientation.z = odom->pose.pose.orientation.z;
    //turtlePlan.push_back(turtlePose);
    orbSLAMPath.poses.push_back(orbSLAMPose);

    if(!orbSLAMPath.poses.empty()){
      orbSLAMPath.header.stamp = orbSLAMPath.poses[0].header.stamp;
    }
    orbSLAMPathPub.publish(orbSLAMPath);
  }
  /*********************************************************************************
  * private cl variables
  */
  private:
    ros::Subscriber tagDataSub;
    ros::Subscriber orbSLAMSub;

    ros::Publisher turtlePathPub;
    ros::Publisher dronePathPub;
    ros::Publisher orbSLAMPathPub;

    geometry_msgs::PoseStamped turtlePose;
    geometry_msgs::PoseStamped orbSLAMPose;
    geometry_msgs::PoseStamped dronePose;
    geometry_msgs::PoseStamped tagPose;
    nav_msgs::Path dronePath;  
    nav_msgs::Path turtlePath;
    nav_msgs::Path orbSLAMPath;

    tf::TransformListener listener;
    tf::StampedTransform tfTransform;
};

int main(int argc, char **argv) {
  //create the node ros functions can be used
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
 // ros::Rate rate(30);
  
  std::cout<<"CL initializing"<< std::endl;
  int seq = 0;

  cl pilot(n);
  //while(ros::ok()) {
    //rate.sleep();
    ros::spin();
  //}
    
  return 0;
}
