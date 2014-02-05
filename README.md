ros-oculus-monocular_viewer
===========================

An Oculus Rift monocular viewer integrated with the Robot Operating System (ROS)

It's based off of remote-eyes project which in turn is based off of OculusWorld_demo from Oculus Rift SDK

This viewers is built to allow the usage of the configurable oculus rift viewer for SINGLE images
published and subscibed to by the ROS message passing service

RosBuild Install
----------------
to install clone the repo to your stack
rospack profile && rosstack profile
roscd ros-oculus-monocular_viewer
rosmake ros-oculus-monocular_viewer


Catkin Install
--------------
Currently working on this

To run the application to make sure it works 
rosrun ros-oculus-monocular_viewer ros-oculus-monocular_viewer

rostopic pub /oculus/fpv/message std_msgs/String "Hello World" 
The message should show up on the viewer

ROS Subscribers
---------------
/*  publish to this channel to send the images to the headset */

/oculus/fpv/image

/*  publish a string to print to the oculus rift screen */

/oculus/fpv/message

Future Updates
--------------
  -Move to Catkin
  -Support ROS Hydro
  -Will be adding support later for printing to a specific height in the message subscriber

