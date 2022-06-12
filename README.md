# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />
<img src="media/dataset_1.png" width="700" height="400" />
<img src="media/dataset_2.gif" width="700" height="400" />

Uncompleted project and data from: https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
Udacity sensor fusion project for Lidar.

To install on windows:

0. Install cvpkg
1. Istall PCL: e.g. https://pointclouds.org/downloads/
2. Clone this github repo
3. Use the same CMake list here
4. Install Visual Studio
5. Follo these in shell:
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
6. Now project file is open and ready to open in Visual Studio

I had to fix several issues along the way, you can contact me if there was a problem.

### Code for binary search using 3D Kd-Tree structure for nearest neighbours search has been implemented

### Code for RANSAC has been implemented

The procedure is as follows:

At each Frame:
- Read the point cloud
- Downsample the cloud by Voxel filtering
- Segmentation: Separate road point cloud from the rest of cloud points. Left with road and obstacles clouds. (RANSAC used)
- Cluster the obstacle cloud. Each object to be assigned to a cluster and a bonding box that shows its range of points. (Binary search and KdTree is used)


