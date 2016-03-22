# ALMUAV
ROS Nodes

##offb_node
offboard MAVLink node for testing. Should not be used with UAV with props!
Sets trust to 0.1 500 times and then sets the trust to 0.0.
Uses the MAVROS package.
To launch MAVROS: roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0"

##robot_camera_launch
Camera node. Pusblishes images captured from the systems /dev/video0 device.

##monocular_pose_estimator
Monocular pose estimator from https://github.com/uzh-rpg
Please note that this is released under a GPL licence. Please contact the authors for a commercial license.

M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.

##monocular_pose_estimator_lib
Monocular pose estimator libs from https://github.com/uzh-rpg
Please note that this is released under a GPL licence. Please contact the authors for a commercial license.

M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.

# matlab
Folder with matlab scripts for analysing data
## EstimatedPosePlotter.m
Subscribes to the monocular_pose_estimator's estimated pose and plots the data
