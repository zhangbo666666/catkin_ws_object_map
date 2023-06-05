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

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include "System.h"
//#include </home/zb/catkin_ws_object_map/devel/include/object_map_msgs/KeyframeTrans.h>

#include <object_map_msgs/KeyframeTrans.h>

class Node
{
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    Node (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();

  protected:
    //void Update ();
//    void Update(bool mbIsKeyframe,
//                const cv::Mat &imRGB,
//                const cv::Mat &imDepth);

    void Update(const sensor_msgs::ImageConstPtr& msgRGB,
                const sensor_msgs::ImageConstPtr& msgD,
                bool bIsKeyframe, // is keyframe?
                cv::Mat Tcw); // transform of keyframe

    void Update(const cv::Mat & msgRGB,
                const cv::Mat & msgD,
                bool bIsKeyframe, // is keyframe?
                cv::Mat Tcw); // transform matrix

    ORB_SLAM2::System* orb_slam_;
    ORB_SLAM2::Tracking* orb_slam_tracking_;

    ros::Time current_frame_time_;

  private:
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    void UpdateParameters ();

    // publish the RGB and depth images of keyframes
    void PublishKeyFrameRGBImage(const sensor_msgs::ImageConstPtr& msgRGB);
    void PublishKeyFrameDepthImage(const sensor_msgs::ImageConstPtr& msgD);
    void PublishKeyFrameRGBImage (const cv::Mat &imRGB);
    void PublishKeyFrameDepthImage (const cv::Mat &imDepth);

//    void PublishKeyFrameRGBImage (const cv::Mat &imRGB);
//    void PublishKeyFrameDepthImage (const cv::Mat &imDepth);

    // Publish the transform of keyframe to map with the time stamp
    void PublishKeyframeTransform2Map(tf::Transform transform);

    void PublishKeyframeTransform (cv::Mat pose); // not to map

    tf::Transform TransformFromMat (cv::Mat position_mat);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void InitParameters ();

    image_transport::Publisher rendered_image_publisher_;

    image_transport::Publisher keyframe_rgb_image_publisher_;
    image_transport::Publisher keyframe_depth_image_publisher_;

    ros::Publisher map_points_publisher_;
    ros::Publisher keyframe_transform_publisher_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;

    bool localize_only_param_;
    bool reset_map_param_;
    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    int minimum_num_of_kf_in_map_param_;
    bool publish_pointcloud_param_;

    // Publish the point cloud
    void PublishPointCloud (PointCloud::Ptr pointCloud);
    sensor_msgs::PointCloud2 PointsToPointCloud (PointCloud::Ptr pointCloud);
    ros::Publisher point_cloud_publisher_;

};

#endif //ORBSLAM2_ROS_NODE_H_
