# Adaptive-Point-Line-Fusison-SLAM-with-MOT
This is a point-line feature fusion SLAM system with moving object tracking. It's a system running in ROS2 environment. 'PointLimeSLAM' include the source code of SLAM system, 'semantic' is a document that includes the semantic topic, 'slam_interfaces' contains communication files between the SLAM system and semantic nodes.
# Compiling
The program was only tested under a 64-bit Linux distribution.(Ubuntu 22.04)
The code depends on Eigen3, OpenCV, Solov2, ROS2
# Usage
For running the program:  
1- Enter ./PLMOT-SLAM and execute [chmod +x build.sh and ./build.sh]
2- Enter the root directory and execute [Colcon build]  
3- source your directory/install/setup.bash  
4- ros2 run semantic solo_service  
5- ros2 run PointLineSLAM rgbd_line ...(the same as ORB-SLAM3)  
# Usage
The code is mainly based on ORB-SLAM3(https://github.com/UZ-SLAMLab/ORB_SLAM3) and ORB-LINE-SLAM(https://github.com/Giannis-Alamanos/ORB-LINE-SLAM).  
The test dataset can be download at TUM : https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download; Bonn: https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/ EuRoc: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
