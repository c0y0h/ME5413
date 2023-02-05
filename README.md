# ME5413 HW1
# perception_lidar_clustering
This is the Homework_1: Perception of ME5413: Autonomous Mobile Robot. 

Group member: Chen Yihui, Wang Renjie

## Task 1: Lidar Clustering
Perform lidar clustering of a set of 10 lidar scene samples using DBSCAN and other methods.

Reference: https://scikit-learn.org/stable/modules/clustering.html

Task 1 folder includes the followings:

i.  Code:       task1_lidar_cls.ipynb

Before run the code, remember to put the lidar_data file in this folder (Task 1). 

If you use VScode, remember to uncomment the followings in the code and change the path:
# import sys
# sys.path.append('/home/cyh/anaconda3/lib/python3.8/site-packages')

After running the code, the clustering results will be written to lidar_clustering.json.


ii. Results:    lidar_clustering.json

The results are saved in a json file, which is in the form of direction.
For each frame in lidar_data file, number of ground points and surrounding points, number of clusters and number of noise points are recorded. Also a list of bounding box objects is created, for each cluster, bounding box information is recorded (position–x, position–y , position–z , size-x, size-y, size-z, class it belongs).
 


## Task 2: Image Segmentation

To be written.



## Bonus Task: Implementation in ROS
1. Perform Lidar Clustering in ROS by subscribing to the topic - /me5413/lidar_top 
2. Save the clustering results as a rosbag.

# Installation Process
cd ~/catkin_ws/
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

# Execution for python file (subscriber: receive point cloud data from rosbag)
cd ~/catkin_ws/src/perception_lidar_clustering/scripts/
chmod +x lidar_clustering.py 

# Do clustering and record the results as rosbag 
To be written

# Visualize the initial lidar data
** Terminal 1 **
roscore

** Terminal 2 **
roslaunch perception_lidar_clustering RosbagPlay_InitialData.launch

# Visualize the clustering results
** Terminal 1 **
roscore

** Terminal 2 **
roslaunch perception_lidar_clustering RosbagRecord_ClusteredData.launch
