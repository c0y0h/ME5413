# perception_lidar_clustering
This is the Homework_1: Perception of ME5413: Autonomous Mobile Robot. 

Group member: Chen Yihui, Wang Renjie

## Installation Process
```
cd ~/catkin_ws/src
git clone https://github.com/Wang-Theo/perception_lidar_clustering.git
cd ~/catkin_ws/
catkin_makePerception
source devel/setup.bash
```

## Execution for python file (subscriber: receive point cloud data from rosbag)
```
cd ~/catkin_ws/src/perception_lidar_clustering/scripts/
chmod +x lidar_clustering.py 
```

** Terminal 1 **
```
roscore
```

** Terminal 2 **
```
rosrun perception_lidar_clustering lidar_clustering.py
```

** Terminal 3 **
```
rosbag play me5413_lidar.bag
```
