# ALMUAV
ROS Nodes

##offb_node
offboard MAVLink node for testing. Should not be used with UAV with props!
Sets trust to 0.1 500 times and then sets the trust to 0.0.
Uses the MAVROS package.
To launch MAVROS: roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0"

##robot_camera_launch
Camera node. Pusblishes images captured from the systems /dev/video0 device.
