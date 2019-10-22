# planner_ros_matlab
ROS+Matlab for path planning
Fake_Laser_Package is a ROS package which contains a laserScan subscriber
It contains:
LaserSub: subscriber for LaserScan
start_laser: converts camera feed to laserScan
video.launch publishes webcam feed on /webcam topic
//
Remaining files are matlab files which use Navigation Toolbox
https://www.mathworks.com/help/robotics/examples/path-following-for-differential-drive-robot.html
https://in.mathworks.com/help/ros/ug/obstacle-avoidance-with-turtlebot-and-vfh.html;jsessionid=4e60634e331ddffcdf6e7ba1c025
https://in.mathworks.com/help/nav/ref/binaryoccupancymap.html
BO.m generates random objects on binary occupancy grid and calculates a path to a goal using prm.
