# ME5413 HW1 perception_lidar_clustering
This is the Homework_1: Perception of ME5413: Autonomous Mobile Robot. 

Group member: Chen Yihui, Wang Renjie


## Task 1: Lidar Clustering
Perform lidar clustering of a set of 10 lidar scene samples using DBSCAN and other methods.

Reference: https://scikit-learn.org/stable/modules/clustering.html

Task 1 folder includes the followings:

    i.  Code:       task1_lidar_cls.ipynb

        Directly run the .ipynb code, the clustering results will be visualized for each frame. 

        If you use VScode, remember to uncomment the followings codes and change the path:
        import sys
        sys.path.append('/home/cyh/anaconda3/lib/python3.8/site-packages')

        After running the code, the clustering results will be written to Results/lidar_clustering.json.


    ii. Results:    lidar_clustering.json

        The results are saved in a json file, which is in the form of dictionary.

        For each frame in lidar_data file, number of ground points and surrounding points, number of clusters and number of noise points are recorded. Also a list of bounding box objects is created, for each cluster, bounding box information is recorded (position–x, position–y , position–z , size-x, size-y, size-z, class).
 

## Task 2: Image Segmentation

    i.  Code:       MATLAB files 
    ii. Results:    Image files 

### Execution process
1. Download retrained model and put it in the Code file

[DeepLabv3plusResnet18CamVid](https://ssd.mathworks.com/supportfiles/vision/data/deeplabv3plusResnet18CamVid.zip)

2. Open "Task2" file with MATLAB
3. Enter "Code" file and run "Execution_2A.m" (do not foget to change the image path to your absolute path) and will find result images in "Results/result_2A/" file
3. Enter "Code" file and run "Execution_2B.m" (do not foget to change the image path to your absolute path) and will find result images in "Results/result_2B/" file
3. Enter "Code" file and run "Execution_2C.m" (do not foget to change the image path to your absolute path) and will find result images in "Results/result_2C/" file


## Bonus Task: Implementation in ROS
1. Perform Lidar Clustering in ROS by subscribing to the topic - /me5413/lidar_top 
2. Save the clustering results as a rosbag.

### Installation Process
```
cd ~/catkin_ws/
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

### Execution for python file (subscriber: receive point cloud data from rosbag)
```
cd ~/catkin_ws/src/perception_lidar_clustering/scripts/
chmod +x lidar_clustering.py
```

### Visualize the initial lidar data
```
roslaunch perception_lidar_clustering RosbagPlay_InitialData.launch
```

### Do clustering and record the results as rosbag 
** Terminal 1 **
```
roscore
```


### Visualize the clustering results
```
roslaunch perception_lidar_clustering RosbagRecord_ClusteredData.launch
```
