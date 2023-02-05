#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# import
import sys
sys.path.append('/home/cyh/anaconda3/lib/python3.8/site-packages')

import numpy as np
import open3d as o3d
import struct
from sklearn import cluster, datasets, mixture
import matplotlib.pyplot as plt
from pandas import DataFrame
from pyntcloud import PyntCloud
import math
import random
from collections import defaultdict
from sklearn.cluster import DBSCAN


def receive_points(data):
    matrix = np.array(data)
    points = matrix[:, :3]
    return points

# filter the ground pointcloud
# Input:
#     data: one frame of pointcloud
# Output:
#     segmengted_cloud: pointcloud after filtering
def ground_segmentation(data):
    # initialize data
    idx_segmented = []
    segmented_cloud = []
    iters = 100     # maximum iteration  
    sigma = 0.4     # maximum accepted error between data and model 数据和模型之间可接受的最大差值  
    ## plane model:  aX + bY + cZ +D= 0
    best_a = 0
    best_b = 0
    best_c = 0
    best_d = 0
    pretotal = 0    # number of inliers in last iteration
    P = 0.99        # the probability of getting the correct model 希望的到正确模型的概率
    n = len(data)   # number of points 
    outline_ratio = 0.6   #e: outline_ratio  
    for i in range(iters):
        ground_cloud = []
        idx_ground = []
        # step1 choose the smallest datset to estimate the model, 3 points for a plane
        # 选择可以估计出模型的最小数据集，对于平面拟合来说，就是三个点
        sample_index = random.sample(range(n),3)    # randomly select 3 points from dataset
        point1 = data[sample_index[0]]
        point2 = data[sample_index[1]]
        point3 = data[sample_index[2]]
        # step2 solve the model
        ## first solve the normal vector 先求解法向量
        point1_2 = (point1-point2)      # vector poin1 -> point2
        point1_3 = (point1-point3)      # vector poin1 -> point3
        N = np.cross(point1_3,point1_2)            # get the normal vector of plane 向量叉乘求解 平面法向量
        ## slove model parameter a,b,c,d
        a = N[0]
        b = N[1]
        c = N[2]
        d = -N.dot(point1)
        # step3 use all the data to count the number of inliers 
        # 将所有数据带入模型，计算出“内点”的数目；(累加在一定误差范围内的适合当前迭代推出模型的数据)
        total_inlier = 0
        pointn_1 = (data - point1)    # sample（三点）外的点 与 sample内的三点其中一点 所构成的向量
        distance = abs(pointn_1.dot(N))/ np.linalg.norm(N)     # distance
        # judge the inliers by distance
        idx_ground = (distance <= sigma)
        total_inlier = np.sum(idx_ground == True)    # number of inliers
        # compare the performance of model 
        if total_inlier > pretotal:                                           #     log(1 - p)
            iters = math.log(1 - P) / math.log(1 - pow(total_inlier / n, 3))  #N = ------------
            pretotal = total_inlier                                               #log(1-[(1-e)**s])
            # the best model param
            best_a = a
            best_b = b
            best_c = c
            best_d = d

        # enough inliers?
        if total_inlier > n*(1-outline_ratio):
            break
    print("iters = %f" %iters)
    # points after segmentation
    idx_segmented = np.logical_not(idx_ground)
    ground_cloud = data[idx_ground]
    segmented_cloud = data[idx_segmented]
    return ground_cloud,segmented_cloud


# do clustering
# Input:
#     data: cloud points without ground cloud
# Output:
#     clusters_index： class index of every points
def dbscan_clustering(data):
    # sklearn dbscan
    # eps: scan radius;  min_samples: min samples in the circle; n_jobs ：CPU form
    cluster_index = DBSCAN(eps=0.3,min_samples=5,n_jobs=-1).fit_predict(data)
    print(cluster_index)
    return cluster_index


def do_clustering(data_oneFrame):

    origin_points = receive_points(data_oneFrame) # read data
    origin_points_df = DataFrame(origin_points,columns=['x', 'y', 'z'])  # [0,3)
    point_cloud_pynt = PyntCloud(origin_points_df)  # store in structs
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False) # to instance

    # ground segmentation
    ground_points, segmented_points = ground_segmentation(data=origin_points)
    
    # show clustering results
    cluster_index = dbscan_clustering(segmented_points)
    
    return cluster_index
    
    # # print(max(cluster_index) + 1)

    # # Number of clusters in labels, ignoring noise if present.
    # n_clusters_ = len(set(cluster_index)) - (1 if -1 in cluster_index else 0)
    # n_noise_ = list(cluster_index).count(-1)

 
def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data)
    gen_list = list(gen)
    # for p in gen:
    #     print (p)
    # print(gen_list)
    # print(type(gen_list))
    
    cluster_index = do_clustering(gen_list)
    global cluster_indexList
    cluster_indexList.append(cluster_index)

    print(len(cluster_indexList))

 
def listener():
    rospy.init_node('pylistener', anonymous=True)
    #Subscriber(name of topic，type of data, callback function)
    rospy.Subscriber('/me5413/lidar_top', PointCloud2, callback_pointcloud)
    rospy.spin()

 
def talker():
    pub = rospy.Publisher('pypubulisher', String, queue_size=10)
    rospy.init_node('cloudpoint', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(hello_str)
        rate.sleep()
 
if __name__ == '__main__':
    cluster_indexList = []
    listener()
    # talker()

