#include <stdlib.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int frame_num = 0;
 
void pc_callback(sensor_msgs::PointCloud2::ConstPtr msg_pc_ptr,
                 const ros::Publisher& pc_pub,
                 const float& cluster_tolerance,
                 const float& min_cluster_size) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg_pc_ptr,*cloud);
 
    pcl::console::TicToc tt;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    {
        tt.tic(); //before voxel grid
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud (cloud);
        float leaf=0.1f;
        vg.setLeafSize(leaf,leaf,leaf);
        vg.setDownsampleAllData(true); //work on all points?
        vg.filter(*cloud_filter);
        std::cerr << "voxel grid done,cost " << tt.toc() << " ms," \
            << cloud->points.size() << " -> " << cloud_filter->points.size() << " points" << std::endl;
    }
 
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filter);
 
    tt.tic(); //before voxel grid
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance); //设置近邻搜索的搜索半径,e.g. 0.5
    ec.setMinClusterSize(min_cluster_size);    //设置一个聚类需要的最少点数
    ec.setMaxClusterSize(cloud_filter->size()/5.);  //设置一个聚类需要的最大点数目
    ec.setSearchMethod(tree);     //设置点云的搜索机制
    ec.setInputCloud(cloud_filter); //设置原始点云
    ec.extract(cluster_indices);  //从点云中提取聚类
    frame_num++;
    std::cerr << "cluster done,cost " << tt.toc() << " ms," << cluster_indices.size() << " clusters" 
            << "Frame Number: " << frame_num << std::endl;
 
    for (const auto& c : cluster_indices) {
        int r = 255. * std::rand() / RAND_MAX;
        int g = 255. * std::rand() / RAND_MAX;
        int b = 255. * std::rand() / RAND_MAX;
        for (const auto& idx : c.indices) {
            cloud_filter->at(idx).r = r;
            cloud_filter->at(idx).g = g;
            cloud_filter->at(idx).b = b;
        }
    }
    sensor_msgs::PointCloud2 msg_cloud_filter_cluster;
    pcl::toROSMsg(*cloud_filter, msg_cloud_filter_cluster);
    msg_cloud_filter_cluster.header.frame_id = "lidar_top";
    pc_pub.publish(msg_cloud_filter_cluster);
};
 
int main (int argc, char** argv) {
    ros::init(argc, argv, "pcl_euclidean_cluster");
    float cluster_tolerance = std::atof(argv[1]);
    float min_cluster_size = std::atof(argv[2]);
    ros::NodeHandle nd;
    ros::Publisher pc_pub = nd.advertise<sensor_msgs::PointCloud2>("euclidean_cluster",1);
    ros::Subscriber pc_sub = nd.subscribe<sensor_msgs::PointCloud2>("/me5413/lidar_top",10,
                                        boost::bind(&pc_callback,_1,pc_pub,cluster_tolerance,min_cluster_size));
    ros::spin();
    return 0;
}