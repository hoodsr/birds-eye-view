# Bird's Eye View
v. 2.2

[Bird's Eye View][git-repo-url] is a package for [ros][ROS] that allows an autonomous aerial vehicle to track and maintain a relative position to an autonomous ground robot. This paper was presented at ICUAS 2017.

## Authors
---
Shannon Hood

Kelly Benson

Daniel Madison

Patrick Hamod

Jason M. O'Kane

Ioannis Rekleitis

## Installation
---
As of now, this has only been built on ROS Indigo using Ubuntu 14.04
For help installing ros indigo, go [here][ROS-INSTALL]. See wiki for detailed installation instructions.
##### Required Packages
---
* ros-indigo-desktop-full
* ros-indigo-turtlebot
* urg-node  (or equivalent laser range finder package)
* usb_cam
* bebop_autonomy
* ORB-SLAM 2

##### Downloading and setting up the build enviroment
---
```sh
cd /your/catkin/workspace/src
git clone https://github.com/madisodr/visiontracking.git
cd visiontracking/
```

## License
---
Released under the MIT Liscense 


[afrl]: http://afrl.cse.sc.edu/afrl/home/
[git-repo-url]: <https://github.com/madisodr/visiontrackingr>
[ROS]: <http://www.ros.org/>
[ROS-INSTALL]: <http://wiki.ros.org/indigo/Installation/Ubuntu>
