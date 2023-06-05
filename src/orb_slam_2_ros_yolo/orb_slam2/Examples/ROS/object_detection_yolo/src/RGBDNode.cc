#include "RGBDNode.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/cpc_segmentation.h>

#include <pcl/PCLPointCloud2.h>


// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
pcl::visualization::CloudViewer viewer ("Viewer");//直接创造一个显示窗口
PointCloud::Ptr personPointCloud( new PointCloud() );

bool show_normals = false, normals_changed = false;
bool show_adjacency = false, line_changed = false;
bool show_supervoxels = false;
bool show_segmentation = true;
bool show_help = true;
bool bg_white = false;
float normals_scale;
float line_width = 2.0f;
float textcolor;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_rgbd");
    ros::start();

    if(argc != 3)
    {
        ROS_ERROR ("Path to vocabulary and path to settings need to be set.");
        ros::shutdown();
        return 1;
    }

    string strVocFile = argv[1];
    string strSettingsFile = argv[2];
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    RGBDNode node (strVocFile, strSettingsFile,node_handle, image_transport);
//    node.mbIsRemovePersonFeatures = atoi(removePerson.c_str());

    // ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
    }
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
    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    thermal_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_raw", 1);
    temp_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_temp", 1);

    bounding_boxes_subscriber_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(node_handle,
                                                                                                  "/darknet_ros/bounding_boxes",
                                                                                                  1);
//    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
//    cout << "0.ImageCallback" << endl;

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(2000), *rgb_subscriber_, *depth_subscriber_,
                                                         *thermal_img_subscriber_, *temp_img_subscriber_, *bounding_boxes_subscriber_);
//    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2,));
    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3, _4,_5));

//    cout << "2.ImageCallback" << endl;

    mCompleted = 0;
//    completed_publisher_ = node_handle.advertise<std_msgs::Int32MultiArray> ("/orb_slam2_rgbd/dataset_completed", 100);
}

RGBDNode::RGBDNode (const string &strVocFile, const string &strSettingsFile, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) :
        Node (node_handle, image_transport) {
    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    // mMinDistanceBetweenKeyframes = 0.05;
    mMinDistanceBetweenKeyframes = 0.0;
    mbIsKeyframe = false;
    mTcw = cv::Mat::ones(4, 4, CV_32F);
    mTcwLast = cv::Mat::ones(4, 4, CV_32F);

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    // for point cloud resolution
    float resolution = fsSettings["PointCloudMapping.Resolution"];

    //Load ORB Vocabulary
//    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
//
//    clock_t tStart = clock();
//    ORBVocabulary* mpVocabulary = new ORBVocabulary();
//    bool bVocLoad = false; // chose loading method based on file extension
////    bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
//
//    if(!bVocLoad)
//    {
//        cerr << "Wrong path to vocabulary. " << endl;
//        cerr << "Failed to open at: " << strVocFile << endl;
//        exit(-1);
//    }
//    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    personProbTh = fsSettings["Frame.PersonProbThreshold"];
    personThermalMaskMin = fsSettings["Frame.PersonThermalMaskMin"];
    personThermalMaskMax = fsSettings["Frame.PersonThermalMaskMax"];

    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fy"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    k1 = fsSettings["Camera.k1"];
    k2 = fsSettings["Camera.k2"];
    p1 = fsSettings["Camera.p1"];
    p2 = fsSettings["Camera.p2"];
    k3 = fsSettings["Camera.k3"];

    depthMapFactor = fsSettings["DepthMapFactor"];

    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/color/image_rect_color", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth/image_rect_raw", 1);

    // for Realsense
    //    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    //    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/aligned_depth_to_color/image_raw", 1);

    // for Xtion Pro
    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

    thermal_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_raw", 1);
    temp_img_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/thermal/image_temp", 1);

    bounding_boxes_subscriber_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(node_handle,
                                                                                                  "/darknet_ros/bounding_boxes",
                                                                                                  1);
//    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
//    cout << "0.ImageCallback" << endl;

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(2000), *rgb_subscriber_, *depth_subscriber_,
                                                         *thermal_img_subscriber_, *temp_img_subscriber_, *bounding_boxes_subscriber_);
//    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2,));
    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3, _4,_5));

//    cout << "2.ImageCallback" << endl;

    mCompleted = 0;
//    completed_publisher_ = node_handle.advertise<std_msgs::Int32MultiArray> ("/orb_slam2_rgbd/dataset_completed", 100);
}

RGBDNode::~RGBDNode () {
    delete rgb_subscriber_;
    delete depth_subscriber_;
    delete thermal_img_subscriber_;
    delete temp_img_subscriber_;
    delete sync_;
}

/*void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,
        const sensor_msgs::ImageConstPtr& msgD,
        const sensor_msgs::ImageConstPtr& msgThermalImg,
        const sensor_msgs::ImageConstPtr& msgTempImg,
        const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes) {
    // Copy the ros image message to cv::Mat.

//    cout << "1.ImageCallback" << endl;
//    cout << " msgRGB->header.stamp.toSec() " << msgRGB->header.stamp << endl;
//    cout << " msgD->header.stamp.toSec() " << msgD->header.stamp << endl;
//    cout << " msgThermal->header.stamp.toSec() " << msgThermalImg->header.stamp << endl;
//    cout << " msgTemp->header.stamp.toSec() " << msgTempImg->header.stamp<< endl;
//    cout << " msgBoxes->header.stamp.toSec() " << msgBoxes->header.stamp << endl;


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
    std::vector<Object> vecObjects;// 2D检测结果
    GetObjects(msgBoxes, vecObjects);
    current_frame_time_ = msgRGB->header.stamp;

//    cv::imshow("rgb", cv_ptrRGB->image);
//    cv::imshow("depth", cv_ptrD->image);
//    cv::imshow("thermal", cv_ptrThermalImg->image);
//    cv::imshow("temp", cv_ptrTempImg->image);
//    cv::waitKey(10);
//    cout << "cv_ptrRGB->image" << cv_ptrRGB->image.size() << endl;

//    cv::waitKey(10);

    orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrThermalImg->image, cv_ptrTempImg->image,vecObjects,
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
}*/


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,
                              const sensor_msgs::ImageConstPtr& msgD,
                              const sensor_msgs::ImageConstPtr& msgThermalImg,
                              const sensor_msgs::ImageConstPtr& msgTempImg,
                              const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes) {
    // Copy the ros image message to cv::Mat.

    cout << "1.ImageCallback" << endl;
//    cout << " msgRGB->header.stamp.toSec() " << msgRGB->header.stamp << endl;
//    cout << " msgD->header.stamp.toSec() " << msgD->header.stamp << endl;
//    cout << " msgThermal->header.stamp.toSec() " << msgThermalImg->header.stamp << endl;
//    cout << " msgTemp->header.stamp.toSec() " << msgTempImg->header.stamp<< endl;
//    cout << " msgBoxes->header.stamp.toSec() " << msgBoxes->header.stamp << endl;


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
//        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_16UC1);

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
    std::vector<Object2D> vecObjects;// 2D检测结果
    GetObjects(msgBoxes, vecObjects);
    current_frame_time_ = msgRGB->header.stamp;



    cv::Mat rgb_img, thermal_img, depth_img, temp_img;
    cv_ptrRGB->image.copyTo(rgb_img);
    cv_ptrD->image.copyTo(depth_img);
    cv_ptrThermalImg->image.copyTo(thermal_img);
    cv_ptrTempImg->image.copyTo(temp_img);

    cout << "cv_ptrD->image: " << cv_ptrD->image.type() << endl;
    cout << "depth_img: " << depth_img.type() << endl;

    cv::imshow("rgb", rgb_img);
    cv::imshow("depth", depth_img);
    cv::imshow("thermal", thermal_img);
    cv::imshow("temp", temp_img);
    cv::waitKey(10);
    cout << "cv_ptrRGB->image" << rgb_img.size() << endl;

    depthMapFactor = 1.0f/depthMapFactor;

    if((fabs(depthMapFactor-1.0f)>1e-5) || depth_img.type()!=CV_32F)
        depth_img.convertTo(depth_img,CV_32F,depthMapFactor);

    cout << "RemovePerson!" << endl;
//    PointCloud::Ptr personPointCloud( new PointCloud() );
//    pcl::visualization::CloudViewer viewer ("Viewer");//直接创造一个显示窗口
        // get person mask in thermal image
        personMask = cv::Mat::zeros(temp_img.rows, temp_img.cols, CV_8UC1);
        static int num = 0;
        // remove features in person area
        for(int i = 0; i < vecObjects.size(); i++) {
            Object2D obj = vecObjects[i];
            if ((obj.object_name == "refrigerator") && (obj.prob >= personProbTh)) {
                std::cout << "1.ExtractORBRemovePerson.refrigerator! " << std::endl;
                cout << obj.x_min << "  " << obj.x_max << "  " << obj.y_min << "  " << obj.y_max << endl;
                for (int p = obj.y_min; p < obj.y_max; p++)
                    for (int q = obj.x_min - 15; q < obj.x_max+10; q++) {
                        int temp_value = temp_img.at<ushort>(p, q);
//                                cout << "ExtractORBRemovePerson.temp: " << temp << endl;
                        if ((temp_value >= personThermalMaskMin) && (temp_value <= personThermalMaskMax)) {
                            personMask.at<uchar>(p, q) = 255;

                            float d = depth_img.ptr<float>(p)[q];
//                            cout << "d: " << d << endl;

                            if (d < 0.20 || d > 6 || isnan(d)) // bool isnan( float arg );
                                continue;
                            PointT tempPoint;
                            tempPoint.z = d;
                            tempPoint.x = ( q - cx) * tempPoint.z / fx;
                            tempPoint.y = ( p - cy) * tempPoint.z / fy;

                            // normal rgb map
//                            tempPoint.r = rgb_img.ptr<uchar>(p)[q*3];
//                            tempPoint.g = rgb_img.ptr<uchar>(p)[q*3+1];
//                            tempPoint.b = rgb_img.ptr<uchar>(p)[q*3+2];

                            // for thermal map
                            tempPoint.b = thermal_img.ptr<uchar>(p)[q*3];
                            tempPoint.g = thermal_img.ptr<uchar>(p)[q*3+1];
                            tempPoint.r = thermal_img.ptr<uchar>(p)[q*3+2];


                            personPointCloud->points.push_back(tempPoint);

                        }
                    }
            }
        }
    cv::imshow("personMask", personMask);

    personPointCloud->height = 1;
    personPointCloud->width = personPointCloud->points.size();
    cout << "point cloud size = " << personPointCloud->points.size() << endl;
    personPointCloud->is_dense = false;
    viewer.showCloud (personPointCloud);

    if (personPointCloud->points.size() == 0){
        cout << "point cloud size = " << personPointCloud->points.size() << endl;
        return;
    }

    if (personPointCloud->size() != 0){
        pcl::io::savePCDFile ("/home/gxc/dataset/ObjectDection/box_cloud.pcd", *personPointCloud);

        cout << "save person_point_cloud to /home/gxc/dataset/ObjectDection " << endl;


    }

    char key = ' ';
    key = cv::waitKey(500);
    if (key ==  'q')
    {
        return ;
    }



/*    //for picture point cloud

    cv::Mat rgb_hsv_img, thermal_hsv_img;
    cvtColor(rgb_img, rgb_hsv_img, cv::COLOR_RGB2HSV);
    cvtColor(thermal_img, thermal_hsv_img, cv::COLOR_RGB2HSV);
    vector<cv::Mat> rgb_hsv_channels;
    split(rgb_hsv_img, rgb_hsv_channels);

    vector<cv::Mat> thermal_hsv_channels;
    split(thermal_hsv_img, thermal_hsv_channels);
    vector<cv::Mat> fused_img_channels(3);
    fused_img_channels[0] = thermal_hsv_channels[0];
    fused_img_channels[1] = thermal_hsv_channels[1];
    fused_img_channels[2] = rgb_hsv_channels[2];
    cv::Mat fused_hsv_img, fused_img;
    merge(fused_img_channels, fused_hsv_img);
//        cvtColor(fused_hsv_img, fused_img, COLOR_HSV2BGR);
    cvtColor(fused_hsv_img, fused_img, cv::COLOR_HSV2RGB);

    cv::imshow("fused_img", fused_img);
//    cv::imwrite("/home/gxc/dataset/fused_img.png", fused_img);

    PointCloud::Ptr picture_pointcloud( new PointCloud() );
    for ( int m=0; m<depth_img.rows; m++ )
    {
        for ( int n=0; n<depth_img.cols; n++ )
        {
            float d = depth_img.ptr<float>(m)[n];

            //            cout << "d = " << d << endl;

            int value = personMask.ptr<uchar>(m)[n];
//            cout << "value: " << value << endl;
//            if (value != 255)
            {
                // if (d < 0.01 || d>10)
                if (d < 0.20 || d > 6 || isnan(d)) // bool isnan( float arg );
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - cx) * p.z / fx;
                p.y = ( m - cy) * p.z / fy;

//            p.b = color.ptr<uchar>(m)[n*3];
//            p.g = color.ptr<uchar>(m)[n*3+1];
//            p.r = color.ptr<uchar>(m)[n*3+2];

                // normal rgb map
//                p.r = rgb_img.ptr<uchar>(m)[n*3];
//                p.g = rgb_img.ptr<uchar>(m)[n*3+1];
//                p.b = rgb_img.ptr<uchar>(m)[n*3+2];

                // for thermal map
                p.b = fused_img.ptr<uchar>(m)[n*3];
                p.g = fused_img.ptr<uchar>(m)[n*3+1];
                p.r = fused_img.ptr<uchar>(m)[n*3+2];
                picture_pointcloud->points.push_back(p);
            }
        }
    }

    picture_pointcloud->height = 1;
    picture_pointcloud->width = picture_pointcloud->points.size();
    cout << "point cloud size = " << personPointCloud->points.size() << endl;
    picture_pointcloud->is_dense = false;

    if (picture_pointcloud->size() != 0){
        pcl::io::savePCDFile ("/home/gxc/dataset/RGBDT/picture_cloud.pcd", *picture_pointcloud);
    }*/


/*    // Create the filtering object
    // 建立kd-tree对象用来搜索 .
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(personPointCloud);

    // Euclidean 聚类对象.
    pcl::EuclideanClusterExtraction<PointT> clustering;
    // 设置聚类的最小值 2cm (small values may cause objects to be divided
    // in several clusters, whereas big values may join objects in a same cluster).
    clustering.setClusterTolerance(0.1);
    // 设置聚类的小点数和最大点云数
    clustering.setMinClusterSize(100);
    clustering.setMaxClusterSize(25000);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(personPointCloud);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);

    // For every cluster...
    int currentClusterNum = 1;
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        //添加所有的点云到一个新的点云中
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(personPointCloud->points[*point]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // 保存
        if (cluster->points.size() <= 0)
            break;
        std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
        std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
        pcl::io::savePCDFileASCII(fileName, *cluster);

        currentClusterNum++;
    }

    //CPC
    cv::FileStorage fsSettings("/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/orb_slam2/config/RealSenseRGBD.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: /config/RealSenseRGBD.yaml" << endl;
        exit(-1);
    }



    //设定结晶参数
    // Supervoxel Stuff
    float voxel_resolution = fsSettings["segmetation.voxel_resolution"];
    float seed_resolution = fsSettings["segmetation.seed_resolution"];
    float color_importance = fsSettings["segmetation.color_importance"];
    float spatial_importance = fsSettings["segmetation.spatial_importance"];
    float normal_importance = fsSettings["segmetation.normal_importance"];
    bool use_single_cam_transform = false ;
    bool use_supervoxel_refinement = true;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = fsSettings["segmetation.concavity_tolerance_threshold"];
    float smoothness_threshold = fsSettings["segmetation.smoothness_threshold"];
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;

    // CPCSegmentation Stuff
    float min_cut_score = fsSettings["segmetation.min_cut_score"];
    unsigned int max_cuts = 25;
    unsigned int cutting_min_segments = 400;
    bool use_local_constrain = true;
    bool use_directed_cutting = true;
    bool use_clean_cutting = false;
    unsigned int ransac_iterations =10000;

    unsigned int k_factor = 0;
    if (use_extended_convexity)
        k_factor = 1;

    PCL_INFO ("Extracting supervoxels\n");
    /// Preparation of Input: Supervoxel Oversegmentation
    //生成结晶器
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    //和点云形式有关
    super.setUseSingleCameraTransform (use_single_cam_transform);
    //输入点云及结晶参数
    super.setInputCloud (personPointCloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    //输出结晶分割结果：结果是一个映射表
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
    super.extract (supervoxel_clusters);
    super.refineSupervoxels (2, supervoxel_clusters);
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

    /// Set paramters for LCCP preprocessing and CPC (CPC inherits from LCCP, thus it includes LCCP's functionality)

    //生成CPC分割器
    PCL_INFO ("Starting Segmentation\n");
    pcl::CPCSegmentation<PointT> cpc;
    //输入超体聚类结果
    cpc.setConcavityToleranceThreshold (concavity_tolerance_threshold);
    cpc.setSanityCheck (use_sanity_criterion);
    cpc.setCutting (max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
    cpc.setRANSACIterations (ransac_iterations);
    cpc.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
    cpc.setKFactor (k_factor);
    cpc.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
    cpc.setMinSegmentSize (min_segment_size);
    cpc.segment ();

    PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
    pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared ();
    cpc.relabelCloud (*cpc_labeled_cloud);
    SuperVoxelAdjacencyList sv_adjacency_list;
    cpc.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

    bool output_specified = true;
    bool add_label_field = true;
    bool sv_output_specified = true;
    /// Creating Colored Clouds and Output
    if (cpc_labeled_cloud->size () == personPointCloud->size ())
    {
        if (output_specified)
        {
            PCL_INFO ("Saving output\n");
            pcl::io::savePCDFile (  "cpc_out.pcd", *cpc_labeled_cloud);

            if (sv_output_specified)
            {
                pcl::io::savePCDFile ("cpc_svcloud.pcd", *sv_centroid_normal_cloud);
            }
        }
    }
    else
    {
        PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
    }

    cv::Mat tempMask = cv::Mat::zeros(temp_img.rows, temp_img.cols, CV_8UC1);

    for (unsigned int i=0; i< cpc_labeled_cloud->size(); i++) {
        pcl::PointXYZL tmp;
        tmp = cpc_labeled_cloud->points[i];

        float d = tmp.z;
        int mask_x = tmp.x * fx/tmp.z + cx;
        int mask_y = tmp.y * fy/tmp.z + cy;

//        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
        tempMask.at<uchar>(mask_y, mask_x) = 255;

//        tempPoint.z = d;
//        tempPoint.x = ( q - cx) * tempPoint.z / fx;
//        tempPoint.y = ( p - cy) * tempPoint.z / fy;
    }

    cv::imwrite("tempMask.png", tempMask);
    personPointCloud->clear();

    cv::imshow("tempMask" , tempMask);
    num++;*/

//    cv::waitKey(10);

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

        cout << "object position: " << object.x_min << "  " << object.y_min << "  " << object.x_max << "  " << object.y_max <<  endl;

    }
    cout << "\n" << endl;
}