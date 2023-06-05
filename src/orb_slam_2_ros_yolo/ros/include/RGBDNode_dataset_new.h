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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "Node_dataset_new.h"
#include <pangolin/pangolin.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"

#include <sys/stat.h>

class RGBDNode : public NodeDatasetNew
{
  public:
    //RGBDNode (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    RGBDNode(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport,
             string dataset_num, int all_images_num, int image_interval_num, int keyframeDelay);
    ~RGBDNode();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    bool grabRGBD(std::string& depthSeqencePath, std::string& colorSeqencePath, cv::Mat& depthImage, cv::Mat& bgrImage);

    bool grabRGBD(std::string& depthSeqencePath,
                            std::string& colorSeqencePath,
                            std::string& thermalSeqencePath,
                            std::string& tempSeqencePath,
                            cv::Mat& depthImage,
                            cv::Mat& bgrImage,
                            cv::Mat& thermalImage,
                            cv::Mat& tempImage);

    bool isFileExist(char const* filename);

    bool mbIsKeyframe;
    cv::Mat mTcw;
    cv::Mat mTcwLast; // The pose of last keyframe

    // depth image sequence filename [.png]
    string depthSeqencePath;
    // color image sequence filename [.png]
    string colorSeqencePath;

    string thermalSeqencePath;
    string tempSeqencePath;
    string trajectoryPath;
    int mDatasetCompleted; // dataset is completed

    ros::Publisher dataset_completed_publisher_;
    // publish the status: completed, true; not completed: false
    void PublishDatasetCompleted(int flag);

    string mDatasetNum;
    int mImageIntervalNum; // every interval image number to set a keyframe
    int mAllImagesNum; // the number of all images in the dataset
    int mKeyframeDelay; // the delay duration between two keyframes

  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
};

#endif //ORBSLAM2_ROS_RGBDODE_H_
