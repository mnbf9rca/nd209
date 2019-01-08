#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn import svm
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    # from https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/fdb3a445-43e0-4a02-81e2-0448432c156f?contentVersion=1.0.0&contentLocale=en-us
    '''
    commented out for now as doesnt seem to be working
    see https://github.com/strawlab/python-pcl/issues/224
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    # Set threshold scale factor
    x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    '''
    cloud_filtered = pcl_data
    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    pcl_downsampled = vox.filter()
    # TODO: PassThrough Filter
    passthrough_filter = pcl_downsampled.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_filter.set_filter_field_name(filter_axis)
    axis_min = 0.60
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

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()    
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    rospy.loginfo('Detected {} object clusters'.format(len(cluster_indices)))
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
# Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = pcl_objects.extract(pts_list)
        ros_cluster =  pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        # originally: pr2_mover(detected_objects_list)
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    test_scene_num = std_msgs.msg.Int32()

    ###########
    test_scene_num.data = 2
    ###########
    object_name = std_msgs.msg.String()
    arm_name = std_msgs.msg.String()
    pick_pose =  geometry_msgs.msg.Pose()
    place_pose =  geometry_msgs.msg.Pose()

    # store detected objects in a dictionary
    # makes the assumption that each object type occurs only once
    detected_objects_dict = {}
    for detected_object in object_list:
        points_arr = ros_to_pcl(detected_object.cloud).to_array()
        mean_of_points = np.mean(points_arr, axis=0)[:3].tolist()
        detected_objects_dict[detected_object.label] = mean_of_points
    
    rospy.loginfo('MApped {} objects to a dictionary of length {} (removing duplicates)'.format(len(object_list), len(detected_objects_dict)))
    
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    rospy.loginfo('Loaded /object_list with {} entries'.format(len(object_list_param)))
    dropbox_param = rospy.get_param('/dropbox')
    dropboxes = dict(zip([box['group'] for box in dropbox_param], dropbox_param))
    rospy.loginfo('Identified {} drop boxe(s)'.format(len(dropboxes)))
    yaml_dict_list = []

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for target_object in object_list_param:
        rospy.loginfo('Attempting to process object name: {} in group {}'.format(target_object['name'], target_object['group']))
        object_name.data = target_object['name']

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        located_object_centroid = detected_objects_dict.get(target_object['name'])
        if located_object_centroid:
            pick_pose.position.x = float(located_object_centroid[0])
            pick_pose.position.y = float(located_object_centroid[1])
            pick_pose.position.z = float(located_object_centroid[2])        

            # TODO: Create 'place_pose' for the object
            place_pose.position.x = dropboxes[target_object['group']]['position'][0]
            place_pose.position.y = dropboxes[target_object['group']]['position'][1]
            place_pose.position.z = dropboxes[target_object['group']]['position'][2]

            # TODO: Assign the arm to be used for pick_place
            # "green box is located on the right side of the robot, select the 
            # right arm for objects with green group and left arm for objects with red group"
            # but i can pull thsi from config...
            arm_name.data = dropboxes[target_object['group']]['name']

            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                # Helper function to create a yaml friendly dictionary from ROS messages
                #4 def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            yaml_dict_list.append(yaml_dict)
            
            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                '''
                commented out as not necessary for submission
                # TODO: Insert your message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print ("Response: ",resp.success)
                '''

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            

    # TODO: Output your request parameters into output yaml file
    rospy.loginfo('Found a list of {} yaml dictionaries for {} objects requested'.format(len(yaml_dict_list), len(object_list_param)))
    if yaml_dict_list:
        filename = 'objects_for_world_{}.yaml'.format(test_scene_num.data)
        send_to_yaml(filename, yaml_dict_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

 
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()