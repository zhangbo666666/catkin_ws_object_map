#include "RGBDNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_rgbd");
    ros::start();

    if(argc != 4)
    {
        ROS_ERROR ("Path to vocabulary and path to settings need to be set.");
        ros::shutdown();
        return 1;
    }
    string removePerson = argv[3];
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD);

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (&SLAM, node_handle, image_transport);
    node.mbIsRemovePersonFeatures = atoi(removePerson.c_str());

    // ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
        if(node.kbhit()) // 检查当前是否有键盘输入，若有则返回一个非0值，否则返回0
        {
            node.mCompleted = 1;
            node.PublishCompleted(node.mCompleted);
            cout << "To save object database ......" << endl;
            ros::Duration(5).sleep(); // to save data
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/gxc/dataset/SceneNN/000/KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("/home/gxc/dataset/SceneNN/000/CameraTrajectory.txt");

    // Save image and depth images of keyframes and trajectory
    //SLAM.SaveKeyFrameImagesAndTrajectory("/home/gxc/dataset/SceneNN/000/trajectory.log");
    SLAM.SaveKeyFrameImagesAndTrajectory("/home/gxc/dataset/SceneNN/000/trajectory.log");


    ros::shutdown();

    cout << "The program is stopped ......" << endl;
    return 0;
}


RGBDNode::RGBDNode (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) :
    Node (pSLAM, node_handle, image_transport) {
    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    // mMinDistanceBetweenKeyframes = 0.05;
    mMinDistanceBetweenKeyframes = 0.0;

    mbIsKeyframe = false;
    mTcw = cv::Mat::ones(4, 4, CV_32F);
    mTcwLast = cv::Mat::ones(4, 4, CV_32F);

    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/color/image_rect_color", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth/image_rect_raw", 1);

    // for Realsense
    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/aligned_depth_to_color/image_raw", 1);

    // for Xtion Pro
    //rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/kinect2/qhd/image_color_rect", 1);

    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/kinect2/qhd/image_depth_rect", 1);

    thermal_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_raw", 1);
    temp_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_temp", 1);

//    bounding_boxes_subscriber_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(node_handle,
//                                                                                                  "/darknet_ros/bounding_boxes",
//                                                                                                  1);
    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
    cout << "0.ImageCallback" << endl;

//    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(2000), *rgb_subscriber_, *depth_subscriber_,
//                                                         *thermal_img_subscriber_, *temp_img_subscriber_, *bounding_boxes_subscriber_);
//    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(2000), *rgb_subscriber_, *depth_subscriber_,
//                                                         *thermal_img_subscriber_, *temp_img_subscriber_);
//    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
//    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3, _4,_5));

     sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3, _4));


    cout << "2.ImageCallback" << endl;

    mCompleted = 0;
    completed_publisher_ = node_handle.advertise<std_msgs::Int32MultiArray> ("/orb_slam2_rgbd/dataset_completed", 100);
}

RGBDNode::~RGBDNode () {
    delete rgb_subscriber_;
    delete depth_subscriber_;
    delete thermal_img_subscriber_;
    delete temp_img_subscriber_;
    delete sync_;
}

//void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,
//        const sensor_msgs::ImageConstPtr& msgD,
//        const sensor_msgs::ImageConstPtr& msgThermalImg,
//        const sensor_msgs::ImageConstPtr& msgTempImg,
//        const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes) {

void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,
        const sensor_msgs::ImageConstPtr& msgD)
 //       const sensor_msgs::ImageConstPtr& msgThermalImg,
  //      const sensor_msgs::ImageConstPtr& msgTempImg
 {
    // Copy the ros image message to cv::Mat.

    cout << "1.ImageCallback" << endl;
    cout << " msgRGB->header.stamp.toSec() " << msgRGB->header.stamp << endl;
    cout << " msgD->header.stamp.toSec() " << msgD->header.stamp << endl;
  //  cout << " msgThermal->header.stamp.toSec() " << msgThermalImg->header.stamp << endl;
  //  cout << " msgTemp->header.stamp.toSec() " << msgTempImg->header.stamp<< endl;


    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrThermalImg;
    try {
        cv_ptrThermalImg = cv_bridge::toCvShare(msgThermalImg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrTempImg;
    try {
        cv_ptrTempImg = cv_bridge::toCvShare(msgTempImg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get objects from darknet_ros_msgs
//    std::vector<Object2D> vecObjects;// 2D检测结果
//    GetObjects(msgBoxes, vecObjects);
    current_frame_time_ = msgRGB->header.stamp;

//    cv::imshow("rgb", cv_ptrRGB->image);
//    cv::imshow("depth", cv_ptrD->image);
//    cv::imshow("thermal", cv_ptrThermalImg->image);
//    cv::imshow("temp", cv_ptrTempImg->image);
//    cv::waitKey(10);
//    cout << "cv_ptrRGB->image" << cv_ptrRGB->image.size() << endl;

//    cv::waitKey(10);

    cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_RGB2BGR);
//    orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrThermalImg->image, cv_ptrTempImg->image,vecObjects,
//                         cv_ptrRGB->header.stamp.toSec(), mbIsKeyframe, mbIsRemovePersonFeatures, mTcw);

    orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrThermalImg->image, cv_ptrTempImg->image,
                         cv_ptrRGB->header.stamp.toSec(), mbIsKeyframe, mbIsRemovePersonFeatures, mTcw);
//    orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec(), mbIsKeyframe, mTcw);

    if(mbIsKeyframe)
    {
        // Calculate the distance between two keyframes
        if(!mTcw.empty())
        {
//            float distance = 0.0;
//            distance = sqrt((mTcw.at<float>(0, 3) - mTcwLast.at<float>(0, 3)) * (mTcw.at<float>(0, 3) - mTcwLast.at<float>(0, 3)) +
//                            (mTcw.at<float>(1, 3) - mTcwLast.at<float>(1, 3)) * (mTcw.at<float>(1, 3) - mTcwLast.at<float>(1, 3)) +
//                            (mTcw.at<float>(2, 3) - mTcwLast.at<float>(2, 3)) * (mTcw.at<float>(2, 3) - mTcwLast.at<float>(2, 3)));

//            // Only when the first keyframe and distance > mMinDistanceBetweenKeyframes to update
//            static int cnt = 0;
//            if((cnt == 0) || (distance >= mMinDistanceBetweenKeyframes))
//            {
//                Update(msgRGB, msgD, mbIsKeyframe, mTcw);
//                mTcwLast = mTcw;
//                cnt++;
//            }

            Update(msgRGB, msgD, mbIsKeyframe, mTcw);
            //Update(cv_ptrRGB->image, cv_ptrD->image, mbIsKeyframe, mTcw);
        }
        else
        {
            cout << "Tracking lost, mTcw is empty! " << endl;
        }
    }
//    else
//        cout << "The frame is not a keyframe!" << endl;
}

void RGBDNode::PublishCompleted(int flag)
{
    std_msgs::Int32MultiArray msg;
    int saveFolderName = 0; // save the data in the folder "0"
    msg.data.push_back(saveFolderName);
    msg.data.push_back(flag);
    completed_publisher_.publish(msg);
}

// 检测按键
int RGBDNode::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}


// Get objects from darknet_ros_msgs
void RGBDNode::GetObjects(const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
                          std::vector<Object2D>& vecObjects)
{
    int objects_num = msgBoxes->bounding_boxes.size();

    vecObjects.clear();
    for (int i=0; i<objects_num; i++)
    {
        darknet_ros_msgs::BoundingBox boundingBox;
        boundingBox = msgBoxes->bounding_boxes[i];
        Object2D object;
        object.prob = boundingBox.probability;
        object.object_name = std::string(boundingBox.Class);
        object.x_min = boundingBox.xmin;
        object.y_min = boundingBox.ymin;
        object.x_max = boundingBox.xmax;
        object.y_max = boundingBox.ymax;

        vecObjects.push_back(object);

//        cout << "object_name = " << object.object_name << endl;
//        cout << "prob = " << object.prob << "\n" << endl;

        cout << "object: " << object.object_name << "   " << object.prob << endl;
    }
    cout << "\n" << endl;
}
