# Adaptive-Point-Line-Fusison-SLAM-with-MOT
This is a point-line feature fusion SLAM system with moving object tracking
# Compiling
The program was only tested under a 64-bit Linux distribution.(Ubuntu 22.04)
The code depends on Eigen3, OpenCV, Solov2, ROS2
# Usage
For running the program:  
1- Enter the root directory and open a terminal  
2- Colcon build  
3- source your directory/install/setup.bash  
4- ros2 run semantic solo_service
5- ros2 run PointLineSLAM rgbd_line ...(the same as ORB-SLAM3)  
