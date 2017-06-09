# AFRL DRIVER

AFRL_Driver is a ROS node designed around the Turtlebot 2 for Voronoi path finding and following. Created in the [Autonomous Field Robotics Lab][afrl],  AFRL_Driver serves as a easy to implement driver for use with a ground robot that uses bump sensors and a laserscaner. 

[Github][git-repo-url]

### Version
1.0.2


### Installation

ROS is required and as of Version 1.0.1, it has only been tested and built using ROS Indigo on Ubuntu 14.04.3 LTS.

```sh
$ cd /path/to/catkin/workspace/src
$ git clone https://github.com/madisodr/afrl_driver.git
$ cd /path/to/catkin/workspace
$ catkin_make
```
### Launching
Make sure to have roscore running and to have sourced the setup.bash file.

```sh
$ roslaunch afrl_driver turtlebot-laser.launch
```
In a seperate terminal run 
```sh
$ roslaunch afrl_driver driver.launch
```
### Development

If you want to contribute or report any bugs or errors, let us know! 
We could also use help testing on various platforms and builds of ROS. 

### Todos
 - Add some TODOS
  - Elaborate on the Installation portion of the readme.md

  License
  ----
  Released under the MIT Liscense 


  [afrl]: http://afrl.cse.sc.edu/afrl/home/
  [git-repo-url]: <https://github.com/madisodr/afrl_driver>
  [ROS]: <http://www.ros.org/>
