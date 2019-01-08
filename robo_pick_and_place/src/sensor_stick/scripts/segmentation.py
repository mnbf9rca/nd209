#!/usr/bin/env python

import pcl
import rospy

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_pointcloud_data):

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(ros_pointcloud_data)

    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    pcl_downsampled = vox.filter()

    # TODO: PassThrough Filter
    passthrough_filter = pcl_downsampled.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_filter.set_filter_field_name(filter_axis)
    axis_min = 0.76
    axis_max = 1.1
    passthrough_filter.set_filter_limits(axis_min, axis_max)
    pcl_filtered = passthrough_filter.filter()

    # TODO: RANSAC Plane Segmentation
    seg = pcl_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)


    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    
    pcl_objects = pcl_filtered.extract(inliers, negative=True)
    pcl_table = pcl_filtered.extract(inliers, negative=False)

    white_cloud = XYZRGB_to_XYZ(pcl_objects)
    outlier_filter = white_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    extracted_outliers = outlier_filter.filter()

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects =  pcl_to_ros(pcl_objects)
    ros_cloud_table =  pcl_to_ros(pcl_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
