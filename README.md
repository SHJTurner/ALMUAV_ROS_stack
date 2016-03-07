# ALMUAV
ROS Nodes

##offb_node
offboard MAVLink node for testing. Should not be used with UAV with props!
Sets trust to 0.1 500 times and then sets the trust to 0.0.
Uses the MAVROS package.
To launch MAVROS: roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0"

##robot_camera_launch
Camera node. Pusblishes images captured from the systems /dev/video0 device.

#monocular_pose_estimator
Monocular pose estimator from [link]https://github.com/uzh-rpg
Ref: 
M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.

#monocular_pose_estimator_lib
Monocular pose estimator libs from [link]https://github.com/uzh-rpg
Ref: 
M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.
