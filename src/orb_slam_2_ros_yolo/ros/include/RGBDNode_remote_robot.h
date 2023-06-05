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

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "Node.h"
#include <pangolin/pangolin.h>

class RGBDNode : public Node
{
  public:
    RGBDNode (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~RGBDNode ();
    void ImageCallback (const sensor_msgs::CompressedImageConstPtr& msgRGB,const sensor_msgs::CompressedImageConstPtr& msgD);

    bool grabRGBD(std::string& depthSeqencePath, std::string& colorSeqencePath, cv::Mat& depthImage, cv::Mat& bgrImage);
    bool isFileExist(char const* filename);

    bool mbIsKeyframe;
    cv::Mat mTcw;
    cv::Mat mTcwLast; // The pose of last keyframe

    // depth image sequence filename [.png]
    std::string depthSeqencePath;
    // color image sequence filename [.png]
    std::string colorSeqencePath;

  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *depth_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;

    // The min distance between two near keyframes
    float mMinDistanceBetweenKeyframes;
};

#endif //ORBSLAM2_ROS_RGBDODE_H_
