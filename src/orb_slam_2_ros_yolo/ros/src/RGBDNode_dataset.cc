#include "RGBDNode.h"
#include <string>
#include <iostream>

string dataset_num;
string image_num; // start from this image


int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_rgbd");
    ros::start();

    if(argc != 5)
    {
        ROS_ERROR ("Path to vocabulary and path to settings need to be set.");
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD);

    dataset_num = argv[3];
    image_num = argv[4];

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (&SLAM, node_handle, image_transport);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << "Trajectory saved!" << endl;

    ros::shutdown();

    return 0;
}


RGBDNode::RGBDNode (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) :
    Node (pSLAM, node_handle, image_transport) {
    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    //mMinDistanceBetweenKeyframes = 0.05;
    mMinDistanceBetweenKeyframes = 0.0;

    mbIsKeyframe = false;
    mTcw = cv::Mat::ones(4, 4, CV_32F);
    mTcwLast = cv::Mat::ones(4, 4, CV_32F);

//    // depth image sequence filename [.png]
//    depthSeqencePath = "/home/ubuntu/Dataset/SceneNN/016/depth/depth";
//    // color image sequence filename [.png]
//    colorSeqencePath = "/home/ubuntu/Dataset/SceneNN/016/image/image";

    depthSeqencePath = "/media/gxc/Backup Plus/Dataset/SceneNN/" + dataset_num + "/depth/depth";
    colorSeqencePath = "/media/gxc/Backup Plus/Dataset/SceneNN/" + dataset_num + "/image/image";

    while(ros::ok())
    {
        static int img_num = std::stoi(image_num);
        cout << "img_num = " << img_num << endl;
        img_num++;

        current_frame_time_ = ros::Time::now();

        cv::Mat depthImage;
        cv::Mat bgrImage;

        if(!grabRGBD(depthSeqencePath, colorSeqencePath, depthImage, bgrImage))
            break;

        orb_slam_->TrackRGBD(bgrImage, depthImage, current_frame_time_.toSec(), mbIsKeyframe, mTcw);
        Update(bgrImage, depthImage, mbIsKeyframe, mTcw);

        // Save camera trajectory
        orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

//    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/color/image_rect_color", 1);
//    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth/image_rect_raw", 1);

//    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);

//    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
}


RGBDNode::~RGBDNode () {
    delete rgb_subscriber_;
    delete depth_subscriber_;
    delete sync_;
}

/*
void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {

    static int img_num = 1;
    cout << "img_num = " << img_num << endl;
    img_num++;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //current_frame_time_ = msgRGB->header.stamp;

    // read RGB and Depth images from the dataset
    current_frame_time_ = ros::Time::now();

    cv::Mat depthImage;
    cv::Mat bgrImage;

    grabRGBD(depthSeqencePath, colorSeqencePath, depthImage, bgrImage);
    orb_slam_->TrackRGBD(bgrImage, depthImage, current_frame_time_.toSec(), mbIsKeyframe, mTcw);

    // Calculate the distance between two keyframes
//    float distance = 0.0;
//    distance = sqrt((mTcw.at<float>(0, 3) - mTcwLast.at<float>(0, 3)) * (mTcw.at<float>(0, 3) - mTcwLast.at<float>(0, 3)) +
//                    (mTcw.at<float>(1, 3) - mTcwLast.at<float>(1, 3)) * (mTcw.at<float>(1, 3) - mTcwLast.at<float>(1, 3)) +
//                    (mTcw.at<float>(2, 3) - mTcwLast.at<float>(2, 3)) * (mTcw.at<float>(2, 3) - mTcwLast.at<float>(2, 3)));

    // Only when the first keyframe and distance > mMinDistanceBetweenKeyframes to update
    //static int cnt = 0;
//    if((cnt == 0) || (distance >= mMinDistanceBetweenKeyframes)){
//        cout << "Distance between two near keyframes = " << distance << endl;
        //Update(msgRGB, msgD, mbIsKeyframe, mTcw);
        Update(bgrImage, depthImage, mbIsKeyframe, mTcw);
//        mTcwLast = mTcw;
//        cnt++;
//    }
}
*/

/// grab RGB and Depth images from the dataset
bool RGBDNode::grabRGBD(std::string& depthSeqencePath, std::string& colorSeqencePath, cv::Mat& depthImage, cv::Mat& bgrImage)
{
    static int frameIndex = std::stoi(image_num); // start from the image_num
    std::stringstream depthFilename;
    depthFilename << depthSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    std::stringstream colorFilename;
    colorFilename << colorSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";

    bool isExist = isFileExist(depthFilename.str().c_str()) || isFileExist(colorFilename.str().c_str());

    if (isExist == false){
        frameIndex = 0;
        return false;
    }

    bgrImage = cv::imread(colorFilename.str(), -1);
    depthImage = cv::imread(depthFilename.str(), -1);

    frameIndex++;
    return true;
}

bool RGBDNode::isFileExist(char const* filename)
{
    std::ifstream file_tmp(filename);
    if (!file_tmp.is_open())
    {
        return false;
    }
    file_tmp.close();
    return true;
}
