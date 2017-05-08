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
#include <iostream>
#include <fstream>

#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
using namespace std;
class cl {
  public:
  cl(ros::NodeHandle& nh) { 
    scaleX = 1;
    seen = false;
    scaleY = 7.632;
    scaleZ = 2.8;
    first = true;
    test_flag = true;
    orbSLAMSub = nh.subscribe("/ORB_SLAM/odom", 10000, &cl::orbSLAMCallback, this);
    tagDataSub = nh.subscribe("/ar_pose_marker", 100000, &cl::tagCallback, this);
    bebopOdomSub = nh.subscribe("/bebop/odom", 1, &cl::bebopCallback, this);
    turtlePathPub = nh.advertise<nav_msgs::Path>("/birdseye/cl_ugv_path", 1000);
    dronePathPub = nh.advertise<nav_msgs::Path>("/birdseye/cl_uav_path", 1000);
    orbSLAMPathPub = nh.advertise<nav_msgs::Path>("/birdseye/orbslam_path", 1000);
    //tfServer = new tf::TransformBroadcaster();  
    //tfListener = new tf::TransformListener();
    dronePath.header.frame_id = "map";
    turtlePath.header.frame_id = "map";
    orbSLAMPath.header.frame_id = "map";
    output.open ("3poses.txt");
    output << "Timestamp  TBx  TBy  TBz  CLx  CLy  CLz  ORBx  ORBy  ORBz" << endl;
  }
  ~cl(void) {
      output.close();

   // if(tfServer)
     // delete tfServer; 
  }

  void bebopCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    /*lastBebopOdom.header = odom->header;
    lastBebopOdom.pose = odom->pose;
    lastBebopOdom.twist = odom->twist;*/
  }


  /********************************************************************************
  * function should transfrom data to turtlebots frame of reference
  * then using the odom data from the turtlebot, calculate the pose of the drone
  */
  void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) { 
    listener.waitForTransform("map", "base_link", 
            ros::Time(0), ros::Duration(5.0));
    try { 
      listener.lookupTransform("map", "base_link", ros::Time(0), tfTransform);
    } catch(tf::TransformException &exception) { 
      ROS_ERROR("%s", exception.what());
    }

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
      turtlePose.header.stamp = msg->markers[0].header.stamp;

      if(!dronePath.poses.empty()){
        dronePath.header.stamp = dronePath.poses[0].header.stamp;
      }
      dronePathPub.publish(dronePath);
      seen = true; 

    } else {
      seen = false;
    }
  }

  geometry_msgs::PoseStamped getDronePose(geometry_msgs::PoseStamped turtlePose) {
    geometry_msgs::PoseStamped dronePose;
    dronePose = turtlePose;

    dronePose.pose.position.x += tagPose.pose.position.x;
    dronePose.pose.position.y += tagPose.pose.position.y;
    dronePose.pose.position.z += tagPose.pose.position.z+.45; //.45 height of tb

    //might need -90*M_PI/180
    dronePose.pose.orientation.w += tagPose.pose.orientation.w;
    dronePose.pose.orientation.x += tagPose.pose.orientation.x;
    dronePose.pose.orientation.y += tagPose.pose.orientation.y;
    dronePose.pose.orientation.z += tagPose.pose.orientation.z;

    nav_msgs::Odometry odom_data;

    odom_data.pose.pose.position.x = dronePose.pose.position.x;
    odom_data.pose.pose.position.y = dronePose.pose.position.y;
    odom_data.pose.pose.position.z = dronePose.pose.position.z;
    odom_data.pose.pose.orientation.x = dronePose.pose.orientation.x;
    odom_data.pose.pose.orientation.y = dronePose.pose.orientation.y;
    odom_data.pose.pose.orientation.z = dronePose.pose.orientation.z;
    odom_data.pose.pose.orientation.w = dronePose.pose.orientation.w;
    lastBebopOdom = odom_data;
    return dronePose;
  }

  void orbSLAMCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    orbSLAMPose.header.stamp = odom->header.stamp;
    if(first && seen) {
      origTurtlePose = turtlePose;
      startHeight = lastBebopOdom.pose.pose.position.z;
      meterAheadTB = odom->pose.pose.position.z-lastBebopOdom.pose.pose.position.y;
      k1ORBSLamOdom.pose = odom->pose;
      k1ORBSLamOdom.twist = odom->twist;
      k1ORBSLamOdom.header = odom->header;
      k1BebopOdom = lastBebopOdom; // make sure they are from the same timestamp
      first = false;
      usleep(1000);
    } 
    if(!first && ros::Time::now().toSec()-lastScaled>1 && seen) {
      nav_msgs::Odometry kbebop = lastBebopOdom;
      double bebopX = /*sqrt(abs(lastBebopOdom.pose.pose.position.x**/kbebop.pose.pose.position.x-
                      k1BebopOdom.pose.pose.position.x/**k1BebopOdom.pose.pose.position.x))*/;
      double orbSLAMX = /*sqrt(abs(odom->pose.pose.position.x**/odom->pose.pose.position.x-
                    k1ORBSLamOdom.pose.pose.position.x/**k1ORBSLamOdom.pose.pose.position.x))*/;
      double bebopY = /*sqrt(abs(lastBebopOdom.pose.pose.position.y**/kbebop.pose.pose.position.y-
                    k1BebopOdom.pose.pose.position.y/**k1BebopOdom.pose.pose.position.y))*/;
      double orbSLAMY = /*sqrt(abs(odom->pose.pose.position.z**/odom->pose.pose.position.z-
                    k1ORBSLamOdom.pose.pose.position.z/**k1ORBSLamOdom.pose.pose.position.z))*/;
      scaleX = bebopX/orbSLAMX;
      scaleY = bebopY/orbSLAMY;
      scaleZ = (kbebop.pose.pose.position.z-k1BebopOdom.pose.pose.position.z)
              /(odom->pose.pose.position.y-k1ORBSLamOdom.pose.pose.position.y);
      
      lastScaled = ros::Time::now().toSec();
      
      if(test_flag) {
      cout << "////////////////ORB-SLAM Scale Factors////////////////" << endl;
      cout << "ScaleX = " << scaleX << endl; //left/right
      cout << "ScaleY = " << scaleY << endl; //forward
      cout << "ScaleZ = " << scaleZ << "\n" << endl; //height
      cout << "DifforbSLAMY = " << odom->pose.pose.position.z-k1ORBSLamOdom.pose.pose.position.z << endl;
      cout << "DiffBebopY = " <<lastBebopOdom.pose.pose.position.y-k1BebopOdom.pose.pose.position.y << "\n"<< endl;
      cout << "orbSLAMPoseX = " << orbSLAMPose.pose.position.x << endl;
      cout << "orbSLAMPoseY = " << orbSLAMPose.pose.position.y << endl;
      cout << "orbSLAMPoseZ = " << orbSLAMPose.pose.position.z << endl;
      cout << "//////////////////////////////////////////////////////\n" << endl;
      }
    }

    if(seen){
      orbSLAMPose.pose.position.x = odom->pose.pose.position.x*abs(scaleX);
      orbSLAMPose.pose.position.y = odom->pose.pose.position.z*abs(scaleY); //orbslam y and z axis swapped
      orbSLAMPose.pose.position.z = odom->pose.pose.position.y*abs(-scaleZ);

      orbSLAMPose.pose.position.x = origTurtlePose.pose.position.x+orbSLAMPose.pose.position.x;
      orbSLAMPose.pose.position.y = origTurtlePose.pose.position.y+orbSLAMPose.pose.position.y;
      orbSLAMPose.pose.position.z = origTurtlePose.pose.position.z+orbSLAMPose.pose.position.z;
      orbSLAMPose.pose.orientation.w = origTurtlePose.pose.orientation.w;
      orbSLAMPose.pose.orientation.x = odom->pose.pose.orientation.x;
      orbSLAMPose.pose.orientation.y = odom->pose.pose.orientation.y;
      orbSLAMPose.pose.orientation.z = odom->pose.pose.orientation.z;

      orbSLAMPose.pose.position.y += meterAheadTB;
      orbSLAMPose.pose.position.z += startHeight;
      orbSLAMPath.poses.push_back(orbSLAMPose);

      if(!orbSLAMPath.poses.empty()){
        orbSLAMPath.header.stamp = orbSLAMPath.poses[orbSLAMPath.poses.size()-1].header.stamp;
      }

      orbSLAMPathPub.publish(orbSLAMPath);
      output << turtlePose.header.stamp << "  "
             << turtlePose.pose.position.x << "  " << turtlePose.pose.position.y << "  " << turtlePose.pose.position.z << "  "
             << dronePose.pose.position.x << "  " << dronePose.pose.position.y << "  " << dronePose.pose.position.z << "  "
             << orbSLAMPose.pose.position.x << "  " << orbSLAMPose.pose.position.y << "  " << orbSLAMPose.pose.position.z << endl;
      output.flush();
    }
    
  }
  /*********************************************************************************
  * private cl variables
  */
  private:
    ros::Subscriber tagDataSub;
    ros::Subscriber orbSLAMSub;
    ros::Subscriber bebopOdomSub;

    ros::Publisher turtlePathPub;
    ros::Publisher dronePathPub;
    ros::Publisher orbSLAMPathPub;

    geometry_msgs::PoseStamped turtlePose;
    geometry_msgs::PoseStamped origTurtlePose;
    geometry_msgs::PoseStamped orbSLAMPose;
    geometry_msgs::PoseStamped dronePose;
    geometry_msgs::PoseStamped tagPose;
    nav_msgs::Path dronePath;  
    nav_msgs::Path turtlePath;
    nav_msgs::Path orbSLAMPath;

    nav_msgs::Odometry lastBebopOdom;
    nav_msgs::Odometry k1BebopOdom;
    nav_msgs::Odometry k1ORBSLamOdom;
    tf::TransformListener listener;
    tf::StampedTransform tfTransform;

    ofstream output;

    bool first;
    bool test_flag;
    bool seen;
    double scaleX;
    double scaleY;
    double scaleZ;;
    double lastScaled;  
    double meterAheadTB;
    double startHeight;
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
