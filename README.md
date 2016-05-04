# ALMUAV
ROS Nodes
The code depends on opencv 3.. As ROS (jade) precombiled binarys link upagainst opencv 2, they have to be recompiled. To do so: "sudo apt-get remove ros-jade-cv-bridge" (removes cv_bridge and all packages that depends on it), clone https://github.com/ros-perception/image_pipeline and https://github.com/ros-perception/vision_opencv/ into your catkin workspace. go thouth all the cmakelists.txt in the downloaded code and change find_package(opencv) to find_package(opencv 3). call "catkin_make clean". install opencv3 "sudo apt-get install ros-jade-opencv3". Now call catkin_make. All the packages needed is now recompiled.

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
Folder with matlab scripts for analysing data directly from ROS topics.
Uses matlabs robotic system toolbox (tested on R2016a)
## EstimatedPosePlotter.m
Creates multiple plots of the estimated pose's from ros bag files.
Can be changed to, Subscribe to the monocular_pose_estimator's estimated pose and plots the data, by commenting and uncommenting.
