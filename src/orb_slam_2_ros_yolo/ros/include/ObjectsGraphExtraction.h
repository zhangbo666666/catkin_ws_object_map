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

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

#include <tf/transform_listener.h>

// For visualizing things in rviz

#include "rviz_visual_tools.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>              // 欧式距离分割
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>    // 欧式距离 平面分割系数
#include <pcl/segmentation/impl/extract_clusters.hpp>  // 分割器 提取点云

#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

// 1.include files
//#include "object_builders/base_object_builder.hpp"
//#include "object_builders/object_builder_manager.hpp"

using namespace std;

typedef pcl::PointXYZRGBA Point7D;
typedef pcl::PointCloud<Point7D> PointCloud7D;

// 目标语义信息
typedef struct Cluster
{
    Eigen::Vector3d size;    // 3d框尺寸
    Eigen::Vector3d centroid;// 点云中心点
    std::string object_name; // 物体类别名

    Eigen::Vector3d minPt; // 所有点中最小的x值，y值，z值,original point in world coordinate system, not in MF
    Eigen::Vector3d maxPt; // 所有点中最大的x值，y值，z值
    Eigen::Vector3d boxCenter; // 包围盒中心点

    Eigen::Affine3d coordinate_system_t; // translation + rotation
    Eigen::Affine3d coordinate_system_t_label; // label position
} Cluster;


typedef struct Object3d
{
    cv::Rect_<float> rect;// 边框
    Eigen::Vector3d minPt;                       // 所有点中最小的x值，y值，z值, original point in world coordinate system, not in MF
    Eigen::Vector3d maxPt;                       // 所有点中最大的x值，y值，z值,
    Eigen::Vector3d centroid;                    // 点云中心点 齐次表示
    Eigen::Vector3d sizePt;                      // 长宽高===
    Eigen::Vector3d boxCenter;                   // 包围盒中心点, in the world coordinate system

    // save bounding box eight corners
    // Eight corners sequence: 1-2-3-4, 5-6-7-8
    //    5-------6
    //   /|      /|
    //  8-|-----7 |
    //  | 1-----| 2
    //  |/      |/
    //  4-------3
    vector<pcl::PointXYZ> eightCorners;

    // object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
    double bb3d[10];

    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Affine3d coordinate_system_t; // translation + rotation

    Eigen::Affine3d coordinate_system_t_label; // label position

    PointCloud7D::Ptr object_point_cloud;

} Object3d;



class ObjectsPublish
{
public:
    ObjectsPublish (ros::NodeHandle &node_handle);
    ~ObjectsPublish ();

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    ros::Time keyframe_time_;    

    void PublishObjectsBoundingBoxes(std::vector<Cluster> clusters);
    void PublishLabel(const Eigen::Affine3d& pose, const std::string& label);
    void PublishObjectsPointCloud(std::vector<Cluster> clusters);
    void PublishMapPointCloud(PointCloud7D::Ptr mapCloud);
    void PublishObjectsPointCloud(PointCloud7D::Ptr objectsCloud);

    ros::Publisher map_point_cloud_publisher_;
    ros::Publisher objects_point_cloud_publisher_; // all objects point cloud
    ros::Publisher single_object_point_cloud_publisher_; // each object point cloud to publish for MFE in Matlab
    ros::Subscriber object_manhattan_frame_subscriber_;

    void ColorSingleObject(PointCloud7D::Ptr& single_object_points);

    void GetWallObject(PointCloud7D::Ptr mapCloud, std::vector<Cluster>& clusters);
    void pointCloudEuclidean(PointCloud7D::Ptr cloud, PointCloud7D::Ptr& objectCloud);
    void Calculate3DBB(PointCloud7D::Ptr cloud, Cluster& cluster);
    std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters);
    void boundingBoxEstimation(autosense::ObjectPtr singleObject, Object3d& object3d);
    void WriteDatabase2File(string file_path, std::vector<Cluster> clusters);
    //std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters);


private:

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    // Publish all object point clouds for display in rviz
    sensor_msgs::PointCloud2 PointsToPointCloud (PointCloud7D::Ptr pointCloud);
    // Publish an object point cloud with the sequence number
    sensor_msgs::PointCloud2 ObjectsPointsToPointCloud (PointCloud7D::Ptr pointCloud);

};

#endif //ORBSLAM2_ROS_RGBDODE_H_
