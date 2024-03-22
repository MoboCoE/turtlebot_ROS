# turtlebot_ROS
Use the basic ROS system to create maps with gmapping and use YOLO to detect objects.

- Raspberry pi4
- STM32
- RPLidar A1
- Pi Camera

Raspberry Pi4
- Set ROS_MATER_URI bash according to the host's IP according to this link: https://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/
- run roscore
- rosserial
git clone from https://github.com/ros-drivers/rosserial
rosrun rosserial_server serial_node _port:=/dev/ttyUSB0
- rplidar
git clone from https://github.com/Slamtec/rplidar_ros
roslaunch rplidar_ros rplidar_a1.launch
- open camera
rosrun my_cam image_publisher.py

https://github.com/MoboCoE/turtlebot_ROS/assets/119753018/80c303c6-1b68-4c6d-877b-d0014f7eedf4

