#!/usr/bin/env python

# import sys, since we use open3d, you may need to change the path below to your anaconda site-packages
import sys
sys.path.append('/home/cyh/anaconda3/lib/python3.8/site-packages')

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField

import numpy as np
import open3d as o3d
import math
import random
from sklearn.cluster import DBSCAN
import time


def receive_points(data):
    matrix = np.array(data)
    points = matrix[:, :3]
    return points


# self-written RANSAC ground pointcloud segmentation
def ground_segmentation(data):
    # initialize data
    idx_segmented = []
    segmented_cloud = []
    iters = 100     # maximum iteration  
    sigma = 0.4     # maximum accepted error between data and model
    ## plane model:  aX + bY + cZ +D= 0
    best_a = 0
    best_b = 0
    best_c = 0
    best_d = 0
    pretotal = 0    # number of inliers in last iteration
    P = 0.99        # the probability of getting the correct model 
    n = len(data)   # number of points 
    outline_ratio = 0.6   #e: outline_ratio  
    for i in range(iters):
        ground_cloud = []
        idx_ground = []
        # step1 choose the smallest datset to estimate the model, 3 points for a plane
        sample_index = random.sample(range(n),3)    # randomly select 3 points from dataset
        point1 = data[sample_index[0]]
        point2 = data[sample_index[1]]
        point3 = data[sample_index[2]]
        # step2 solve the model
        ## first solve the normal vector
        point1_2 = (point1-point2)      # vector poin1 -> point2
        point1_3 = (point1-point3)      # vector poin1 -> point3
        N = np.cross(point1_3,point1_2)            # get the normal vector of plane 
        ## slove model parameter a,b,c,d
        a = N[0]
        b = N[1]
        c = N[2]
        d = -N.dot(point1)
        # step3 use all the data to count the number of inliers 
        total_inlier = 0
        pointn_1 = (data - point1)   
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
    # print("iters = %f" %iters)
    # points after segmentation
    idx_segmented = np.logical_not(idx_ground)
    ground_cloud = data[idx_ground]
    segmented_cloud = data[idx_segmented]
    return ground_cloud,segmented_cloud


# dbscan clustering method in sklearn
def dbscan_clustering(data):
    # sklearn dbscan
    # eps: scan radius;  min_samples: min samples in the circle; n_jobs ：CPU form
    cluster_index = DBSCAN(eps=0.6,min_samples=8,algorithm='ball_tree', leaf_size=30).fit_predict(data)
    
    return cluster_index


# do clustering
# input:  initial cloudpoint data
# output: points (N, 3), colors (N, 3)
def do_clustering(initial_data):

    origin_points = receive_points(initial_data) # read data

    # # ground segmentation using self-written RANSAC
    # ground_points, segmented_points = ground_segmentation(data=origin_points)

    # ground segmentation using o3d ransac
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(origin_points)
    plane_model, ground_cloud = pcd.segment_plane(distance_threshold=0.3,
                                         ransac_n=3,
                                         num_iterations=1000)
    segmented_cloud = pcd.select_by_index(ground_cloud, invert=True)

    # downsample
    down_pcd = segmented_cloud.voxel_down_sample(voxel_size=0.2)
    print("Before downsample: %d, After downsample: %d" %(np.asarray(segmented_cloud.points).shape[0], np.asarray(down_pcd.points).shape[0]))

    # # remove points that are further away from their neighbors in average
    # cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
    # downpcd_inlier_cloud = down_pcd.select_by_index(ind)

    segmented_points = np.asarray(down_pcd.points)

    # use dbscan
    cluster_index = dbscan_clustering(segmented_points)

    # give a random color for every class
    points = segmented_points
    colors = np.zeros(shape=points.shape)
    for i in range(max(cluster_index) + 1):
        r = round(255. * np.random.rand())
        g = round(255. * np.random.rand())
        b = round(255. * np.random.rand())
        colors[np.argwhere(cluster_index == i)[:,0]] = [r, g, b]

    print("This frame have %d surrounding points and form %d clusters!" %(len(points), max(cluster_index) + 1))
    
    return points, colors


# transform the clustered results to RGB point cloud for publishing and visualizing in rviz
# optimize the bindwidth of msg
def numpy2cloud_msg2(points, colors, frame_id, stamp=None, seq=None):
    """
    @param points: numpy.ndarray float32
    @param colors: numpy.ndarray uint8 [0, 255]
    """
    r, g, b = colors[:, 0].astype(np.uint32), colors[:, 1].astype(np.uint32), colors[:, 2].astype(np.uint32)
    rgb = np.array((r << 16) | (g << 8) | (b << 8), dtype=np.uint32)
    rgb = np.reshape(rgb, newshape=(rgb.shape[0], 1))
    rgb.dtype = np.float32
    array = np.hstack((points, rgb))

    msg = PointCloud2()
    if stamp is not None:
        msg.header.stamp = stamp
    if frame_id is not None:
        msg.header.frame_id = frame_id
    if seq is not None:
        msg.header.seq = seq
    
    msg.height = 1
    msg.width = len(array)
    msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.FLOAT32, 1)
                  ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * array.shape[0]
    msg.is_dense = np.isfinite(array).all()

    msg.data = array.tobytes()

    return msg


# callback function, process the initial data 
def callback_pointcloud(msg, pub):

    assert isinstance(msg, PointCloud2)
    global count
    if count % 9 == 0:
        t_start = time.time()

        initial_points = point_cloud2.read_points_list(msg)

        t_start1 = time.time()
        points, colors = do_clustering(initial_points)
        t_end1 = time.time()
        print(t_end1-t_start1)
        
        points = points.astype(np.float32)
        colors = colors.astype(np.uint8)

        pc2_msg = numpy2cloud_msg2(points, colors, frame_id='lidar_top', stamp=rospy.Time().now())
        pub.publish(pc2_msg)
        
        t_end = time.time()
        count = count + 1
        print("Published the %d th frame, cost %f s" %(count + 1, t_end - t_start))
    else:
        count = count + 1
        print("Skip the %d th frame" %count)

 
 
if __name__ == '__main__':
    count = 0
    rospy.init_node('mynode', anonymous=True)
    pub = rospy.Publisher('pypublisher', PointCloud2, queue_size=1)
    rospy.Subscriber('/me5413/lidar_top', PointCloud2, callback_pointcloud, pub, queue_size=1)
    rospy.spin()

