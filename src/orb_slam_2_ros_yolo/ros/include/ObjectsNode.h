/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_RGBDODE_H_
#define ORBSLAM2_ROS_RGBDODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "System.h"
#include <pangolin/pangolin.h>
#include "std_msgs/Int32MultiArray.h"

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// mask_rcnn_ros
//#include <mask_rcnn_ros/Result.h>

#include <yolact_ros_msgs/Detections.h>


#include <object_map_msgs/KeyframeTrans.h>

#include "ObjectsBoundingBoxes.h"
#include <tf/transform_listener.h>

// For visualizing things in rviz
#include "rviz_visual_tools.h"

//#include "ObjectDatabase.h"

// to calculate the 3DBB
//#include "Objects3DBB.h"

//#include "object_builders/base_object_builder.hpp"
//#include "object_builders/object_builder_manager.hpp"

using namespace std;

class ObjectsNode
{
public:
    typedef pcl::PointXYZRGBA Point7D;
    typedef pcl::PointCloud<Point7D> PointCloud7D;

    ObjectsNode (ros::NodeHandle &node_handle);
    ~ObjectsNode ();
    void KeyframeCallback(const sensor_msgs::ImageConstPtr& msgRGB,
                          const sensor_msgs::ImageConstPtr& msgD,
//                        //const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
                          //const mask_rcnn_ros::ResultConstPtr& msgMasks,
                          const yolact_ros_msgs::DetectionsConstPtr& msgDetections,

                          const object_map_msgs::KeyframeTransConstPtr& msgKeyframeTrans);

    // void ObjectMFECallback(const std_msgs::String::ConstPtr& msg);
    //void ObjectMFECallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    bool flagReceiveObject; // receive objects

    ObjectsBoundingBoxes* mpObjectsBoundingBoxes;    // calculate bounding boxes of objects
    //Objects3DBB* mpObjects3DBB; //  calculate 3DBB

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    ros::Time keyframe_time_;    

    void PublishObjectsBoundingBoxes(std::vector<Cluster> clusters);
    void PublishLabel(const Eigen::Affine3d& pose, const std::string& label);
    void PublishObjectsPointCloud(std::vector<Cluster> clusters);
    void DatasetCompletedCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

    ros::Publisher objects_point_cloud_publisher_; // all objects point cloud
    ros::Publisher single_object_point_cloud_publisher_; // each object point cloud to publish for MFE in Matlab
    //ros::Subscriber object_manhattan_frame_subscriber_;

    std::vector<Cluster> mClustersAll_;
    std::vector<Cluster> mClustersCurrentKeyframe_;

    bool mIsObjectDatabaseUpdate; // true: database is updated, false: database is not updated

    //void ColorSingleObject(PointCloud7D::Ptr& single_object_points, int color_r, int color_g, int color_b,Eigen::Vector3d pc);
    void ColorSingleObject(PointCloud7D::Ptr& single_object_points,Eigen::Vector3d pc);

    // add all single object point cloud to one
    void AllObjectsPointCloud(std::vector<Cluster> clusters, PointCloud7D::Ptr& all_objects_points);
    void SaveAllObjectsPointCloud(std::vector<Cluster> clusters, string point_cloud_name);

    std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters);
    void Calculate3DBB(PointCloud7D::Ptr cloud, Cluster& cluster);

    int mReceivedMFENum; // all received MFE number
    int mKeyframeNum;

    // running time
    vector<double> mvTimeKeyframeUpdate;
    vector<double> mvTimeMFE;

    std::mutex mMutexSaveDatabase;

    bool mDatasetCompleted; // dataset is completed
    int mMaskRcnnDetectedObjectsNum; // the total number of objects detected by mask rcnn

    std::vector<string> mvCocoNames_;

private:
    typedef message_filters::sync_policies::ApproximateTime<
//    sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes, object_map_msgs::KeyframeTrans> sync_pol;
//    sensor_msgs::Image, sensor_msgs::Image, mask_rcnn_ros::Result, object_map_msgs::KeyframeTrans> sync_pol;
    sensor_msgs::Image, sensor_msgs::Image, yolact_ros_msgs::Detections, object_map_msgs::KeyframeTrans> sync_pol;


    message_filters::Subscriber<sensor_msgs::Image>* keyframe_rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image>* keyframe_depth_subscriber_;
    message_filters::Synchronizer<sync_pol>* sync_;

//    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>* bounding_boxes_subscriber_;
//     message_filters::Subscriber<mask_rcnn_ros::Result>* mask_rcnn_subscriber_;
     message_filters::Subscriber<yolact_ros_msgs::Detections>* yolact_subscriber_;

//    message_filters::Subscriber<sensor_msgs::Image>* mask_rcnn_visualization_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image>* yolact_visualization_subscriber_;

    message_filters::Subscriber<object_map_msgs::KeyframeTrans>* keyframe_transform_subscriber_;

    ros::Subscriber dataset_completed_subscriber_;

    void Convert2Transform(tf::StampedTransform transform, Eigen::Isometry3d& T);

    // Get objects from darknet_ros_msgs
//    void GetObjects(const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
//                                 std::vector<Object>& vecObjects);
//    void GetObjects(const mask_rcnn_ros::ResultConstPtr& msgMasks,
//                                 std::vector<Object>& vecObjects);

    void GetObjects(const yolact_ros_msgs::DetectionsConstPtr& msgDetectios,
                                 std::vector<Object>& vecObjects);

    // Get transform of keyframe
    void GetKeyframeTrans(const object_map_msgs::KeyframeTransConstPtr& msgKeyframeTrans, Eigen::Isometry3d& T);

    void Update(cv::Mat img_rgb, cv::Mat img_depth, Eigen::Isometry3d trans2Map, std::vector<Object> vecObjects);

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    // Publish all object point clouds for display in rviz
    sensor_msgs::PointCloud2 PointsToPointCloud (PointCloud7D::Ptr pointCloud);
    // Publish an object point cloud with the sequence number
    sensor_msgs::PointCloud2 ObjectsPointsToPointCloud (PointCloud7D::Ptr pointCloud);

    // convert an array to mat
    cv::Mat Array2Mat(vector<uchar> array);
    cv::Mat YolactMask2Mat(const yolact_ros_msgs::Mask &mask,int x_min, int y_min);


    // calculate the mean time
    double MeanTime(vector<double> time);
    void WriteDatabase2File(string file_path, std::vector<Cluster> clusters);

};

#endif //ORBSLAM2_ROS_RGBDODE_H_
