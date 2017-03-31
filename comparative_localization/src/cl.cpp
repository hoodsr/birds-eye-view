/********************************************************************************************
* cl class
* 
* This class is designed to work with the ar drone parrot 2.0 and turtlebot 2
* to provide comparitive localization of the drone from the turtlebots frame
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
#include "nav_msgs/Odometry.h"

#include <vector>
#include <ctime>
#include <sstream>
#include <math.h>

#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"

class cl {
  public:
  cl(ros::NodeHandle& nh) { 
    turtlebotPose = nh.subscribe("/odom", 100,    &cl::turtlebotCallback,   this);
    tagData = nh.subscribe("/ar_pose_marker", 100000, &cl::tagCallback, this);
    cl_posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("cl_drone_pose", 1, true);
   // cl_particlecloudPub = nh.advertise<geometry_msgs::PoseArray>("cl_particlecloud", 1, true);
    tfServer = new tf::TransformBroadcaster();  
    tfListener = new tf::TransformListener();
    std_srvs::Empty srv;
  }
  ~cl(void) {
    if(tfServer)
      delete tfServer; 
  }

  /********************************************************************************
  * function should transfrom data to turtlebots frame of reference
  * then using the odom data from the turtlebot, calculate the pose of the drone
  */
  void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    tf::Stamped<tf::Pose> alvar_;
    tagPoint = msg->markers.at(0).pose.pose.position;
    geometry_msgs::Quaternion qtn = msg->markers.at(0).pose.pose.orientation; 
    /*  
    tagYaw = tf::getYaw(qtn)*180.0/M_PI;
    tf::Stamped<tf::Pose> alvar_to_map;
    try {
      tfListener->transformPose("base_footprint", msg->markers.at(0).pose, alvar_to_map);
    } catch(tf::TransformException &e) {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", "base_footprint", "ar_track_alvar_msgs", e.what());
        return;
    }
    m_tfServer->sendTransform(tf::StampedTransform(odom_to_map.inverse(),
                                                   message->header.stamp + ros::Duration(transform_tolerance_),
                                                   "global_frame_id_", message->header.frame_id));
    // Publish localized pose
    m_currentPos.header = message->header;
    m_currentPos.header.frame_id = global_frame_id_;
    m_currentPos.pose.pose = current_msg;
    posePub.publish(m_currentPos);*/
  }

  /*********************************************************************************
  * function to save turtlebot pose from odom topic
  */
  void turtlebotCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    /*turtlePose = msg->pose.pose.position;
    turtleQtn = msg->pose.pose.orientation;*/

    /*listener_.waitForTransform("base_footprint", "odom", 
            ros::Time(0), ros::Duration(1.0));

    tf::StampedTransform start_transform;

    listener_.lookupTransform("base_footprint", "odom", 
            ros::Time(0), start_transform);*/
  }

  /*********************************************************************************
  * private cl variables
  */
  private:
    ros::Subscriber tagData;
    ros::Subscriber turtlebotPose;
    ros::Publisher cl_posePub;
    ros::Publisher cl_particlecloudPub;
    tf::TransformBroadcaster* tfServer;
    tf::TransformListener *tfListener;

    ros::ServiceClient client;
    std_srvs::Empty srv;
    geometry_msgs::Point turtlePose;
    geometry_msgs::Quaternion turtleQtn;
    geometry_msgs::Point tagPoint;
    double tagYaw;
};

int main(int argc, char **argv) {
  //create the node ros functions can be used
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  //spin once starts the timers
  ros::spinOnce();
  ros::Rate rate(10);
  
  std::cout<<"CL initializing"<< std::endl;
  int switchCam = 0;
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
    
  return 0;
}
