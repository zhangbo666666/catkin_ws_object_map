#include "ObjectsNode.h"
#include <sys/time.h>
#define Random(x) (rand() % x)

// 计时
long _getTimeUsec()
{
    struct timeval t;
    gettimeofday(&t,0);
    return (long)((long)t.tv_sec*1000*1000 + t.tv_usec);
}

// 使用C++11新特性，实现用字符串作为switch的case子句
typedef std::uint64_t hash_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

hash_t hash_(char const* str)
{
    hash_t ret{basis};

    while(*str){
        ret ^= *str;
        ret *= prime;
        str++;
    }

    return ret;
}
constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis)
{
    return *str ? hash_compile_time(str+1, (*str ^ last_value) * prime) : last_value;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Objects");
    ros::start();
    ros::Rate loop_rate(1);
    ros::NodeHandle node_handle;
    cout << "Main:Start to calculate" << endl;

    ObjectsNode objects(node_handle);

    // ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
        cout << "\n" <<"------------------------3----------------------------"<< endl;
        // Draw bounding boxes of objects, publish to rviz
        std::vector<Cluster> clusters = objects.mpObjectsBoundingBoxes->mpObjectDatabase->getObjectClusters();

        static int num = 0;
        // Publish point clouds of objects
        if(objects.mDatasetCompleted)
        {
            if(num < 3)
            {
                objects.flagReceiveObject = true;
                objects.PublishObjectsBoundingBoxes(clusters);
                num++;
            }
        }
        else
        {
            if (clusters.size() > 0){
                cout << "Step3: Publish object BBOX" << endl;
                objects.PublishObjectsBoundingBoxes(clusters);
            }
        }

        if(clusters.size() > 0){
            cout << "Step3: Publish object pointcloud" << endl;
            objects.PublishObjectsPointCloud(clusters);
        } else{
            cout << "Step3: No objects! " << endl;
        }

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

ObjectsNode::ObjectsNode (ros::NodeHandle &node_handle)
{
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_markers"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    mClustersAll_.clear();
    mClustersCurrentKeyframe_.clear();

    cout << "ObjectsNode: Sleeping 2 seconds before running demo" << endl;
    ros::Duration(2.0).sleep();

    mIsObjectDatabaseUpdate = true; // true: database is updated, false: database is not updated
    flagReceiveObject = false;

    mKeyframeNum = 1;
    mDatasetCompleted = false;
    mMaskRcnnDetectedObjectsNum = 0;

    ObjectDatabase db;
    mvCocoNames_ = db.mvCocoNames;

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    // calculate bounding boxes of objects
    mpObjectsBoundingBoxes = new ObjectsBoundingBoxes();

    //mpObjects3DBB = new Objects3DBB();

    // Get the topics with the same time stamp
//    cout << "Waiting to subscribe keyframe topics ......" << endl;


    keyframe_rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle,
                                                                                    "/orb_slam2_rgbd/keyframe_image_rgb",
                                                                                    100);
    keyframe_depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle,
                                                                                      "/orb_slam2_rgbd/keyframe_image_depth",
                                                                                      100);

//    bounding_boxes_subscriber_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(node_handle,
//                                                                                                  "/darknet_ros/bounding_boxes",
//                                                                                                  20);

//    mask_rcnn_subscriber_ = new message_filters::Subscriber<mask_rcnn_ros::Result>(node_handle,
//                                                                                   "/mask_rcnn/result",
//                                                                                   100);

    yolact_subscriber_ = new message_filters::Subscriber<yolact_ros_msgs::Detections>(node_handle,
                                                                                   "/yolact_ros/detections",
                                                                                   100);

    yolact_visualization_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle,
                                                                                            "/yolact_ros/visualization",
                                                                                            100);

    keyframe_transform_subscriber_ = new message_filters::Subscriber<object_map_msgs::KeyframeTrans>(node_handle,
                                                                                                     "/orb_slam2_rgbd/keyframe_transform",
                                                                                                     100);

    // To synchronize four topics by time stamp
    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(30),
                                                         *keyframe_rgb_subscriber_,
                                                         *keyframe_depth_subscriber_,
                                                         //*bounding_boxes_subscriber_,
                                                          //*mask_rcnn_subscriber_,
                                                         *yolact_subscriber_,
                                                         *keyframe_transform_subscriber_);

    sync_->registerCallback(boost::bind(&ObjectsNode::KeyframeCallback, this, _1, _2, _3, _4));

    // Publish the point clouds of objects
    objects_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/objects_point_cloud", 100, this);
    single_object_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/single_object_point_cloud", 100, this);

    //object_manhattan_frame_subscriber_ = node_handle.subscribe<std_msgs::Float64MultiArray>("/orb_slam2_rgbd/object_MFE", 100, &ObjectsNode::ObjectMFECallback, this); // “this”是指向当前实例
    dataset_completed_subscriber_ = node_handle.subscribe<std_msgs::Int32MultiArray>("/orb_slam2_rgbd/dataset_completed", 10, &ObjectsNode::DatasetCompletedCallback, this);
}

ObjectsNode::~ObjectsNode ()
{
    delete keyframe_rgb_subscriber_;
    delete keyframe_depth_subscriber_;
//    delete bounding_boxes_subscriber_;
//    delete mask_rcnn_subscriber_;
//    delete mask_rcnn_visualization_subscriber_;
    delete yolact_subscriber_;
    delete yolact_visualization_subscriber_;
    delete keyframe_transform_subscriber_;
    delete sync_;
    delete mpObjectsBoundingBoxes;
}


//----------------------Step1 Subscribe images, transform matrix and object detection results from topics----------------------//
// Get RGB, Depth, transform matrix, and object detection results from topics
void ObjectsNode::KeyframeCallback (const sensor_msgs::ImageConstPtr& msgRGB,
                                    const sensor_msgs::ImageConstPtr& msgD,
                                    //const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
                                    // const mask_rcnn_ros::ResultConstPtr& msgMasks,
                                    const yolact_ros_msgs::DetectionsConstPtr& msgDetections,
                                    const object_map_msgs::KeyframeTransConstPtr& msgKeyframeTrans)
{
    cout <<"\n" << "------------------------1----------------------------"<< endl;
    cout << "Step1: Subscribe images, transform matrix and object detection results from topics"  << endl;
    cout << "Step1: Receive a keyframe, number: ***************************  " << mKeyframeNum << "  ***************************" << endl;
    mKeyframeNum++;
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

//    cout << " msgRGB->header.stamp.toSec() " << msgRGB->header.stamp << endl;
//    cout << " msgD->header.stamp.toSec() " << msgD->header.stamp << endl;
//    cout << " msgDetections->header.stamp.toSec() " << msgDetections->header.stamp << endl;

    // Get objects from darknet_ros_msgs
    std::vector<Object> vecObjects;// 2D检测结�?1?7
    //GetObjects(msgBoxes, vecObjects);
    //GetObjects(msgMasks, vecObjects);
    GetObjects(msgDetections, vecObjects);

    mMaskRcnnDetectedObjectsNum += vecObjects.size();

    // Get transform of keyframe
    Eigen::Isometry3d trans_ = Eigen::Isometry3d::Identity();
    GetKeyframeTrans(msgKeyframeTrans, trans_);

    //cv::imwrite("keyframe_rgb.png", cv_ptrRGB->image);
    //cv::imwrite("keyframe_d.png", cv_ptrD->image);

    long time = _getTimeUsec(); // 开始计�?1?7

    //cvtColor(cv_ptrRGB->image, cv_ptrRGB->image, cv::COLOR_RGB2BGR);

    Update(cv_ptrRGB->image, cv_ptrD->image, trans_, vecObjects);
    time = _getTimeUsec() - time; // 结束计时, ms
    cout << "         Update keyframe time: " << time / 1000.0 << " ms" << endl; // 显示检测时�?1?7
    mvTimeKeyframeUpdate.push_back(time / 1000.0);
}


// Convert the tf::StampedTransform to Eigen::Isometry3d
void ObjectsNode::Convert2Transform(tf::StampedTransform transform, Eigen::Isometry3d& T)
{
    double roll, yaw, pitch;
    tf::Quaternion q;
    q = transform.getRotation();
    tf::Matrix3x3 m(q);
    m.getRPY(roll,yaw,pitch);

    ROS_INFO("angles in degree: roll >%f<, yaw >%f<, pitch >%f<", roll, yaw, pitch);

    T = Eigen::Isometry3d::Identity();

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            T(i, j) = m[i][j];
        }
    }

    T(0, 3) = transform.getOrigin().x();
    T(1, 3) = transform.getOrigin().y();
    T(2, 3) = transform.getOrigin().z();
    cout << " T = " << T.matrix() << endl;

}

// Get objects from darknet_ros_msgs
/*void ObjectsNode::GetObjects(const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
                             std::vector<Object>& vecObjects)
{
    int objects_num = msgBoxes->bounding_boxes.size();

    vecObjects.clear();
    for (int i=0; i<objects_num; i++)
    {
        darknet_ros_msgs::BoundingBox boundingBox;
        boundingBox = msgBoxes->bounding_boxes[i];
        Object object;
        object.prob = boundingBox.probability;
        object.object_name = std::string(boundingBox.Class);
        object.x_min = boundingBox.xmin;
        object.y_min = boundingBox.ymin;
        object.x_max = boundingBox.xmax;
        object.y_max = boundingBox.ymax;

        vecObjects.push_back(object);

//        cout << "object_name = " << object.object_name << endl;
//        cout << "prob = " << object.prob << "\n" << endl;

        cout << object.object_name << "   " << object.prob << endl;
    }
    cout << "\n" << endl;
}*/

// Get objects from darknet_ros_msgs
/*void ObjectsNode::GetObjects(const darknet_ros_msgs::BoundingBoxesConstPtr& msgBoxes,
                             std::vector<Object>& vecObjects)
{
    int objects_num = msgBoxes->bounding_boxes.size();

    vecObjects.clear();
    for (int i=0; i<objects_num; i++)
    {
        darknet_ros_msgs::BoundingBox boundingBox;
        boundingBox = msgBoxes->bounding_boxes[i];
        Object object;
        object.object_name = std::string(boundingBox.Class);
        object.prob = boundingBox.probability;
        object.x_min = boundingBox.xmin;
        object.y_min = boundingBox.ymin;
        object.x_max = boundingBox.xmax;
        object.y_max = boundingBox.ymax;

        // get the class id in the coco names
        std::vector<string>::iterator iter_   = mvCocoNames_.begin() - 1;
        std::vector<string>::iterator it_end_ = mvCocoNames_.end();
        iter_ = std::find(++iter_, it_end_, object.object_name);// 按照名字查找
        object.class_id = std::distance(mvCocoNames_.begin(), iter_);

        vecObjects.push_back(object);

        cout << object.class_id << "  " << object.object_name << "  " << object.prob << endl;
    }
    cout << "\n" << endl;
}*/

// Get objects from mask_rcnn_ros/Result.msg
/*
void ObjectsNode::GetObjects(const mask_rcnn_ros::ResultConstPtr& msgMasks,
                             std::vector<Object>& vecObjects)
{
    int objects_num = msgMasks->class_ids.size();

    vecObjects.clear();
    for (int i=0; i<objects_num; i++)
    {
        Object object;
        object.class_id = msgMasks->class_ids[i];
        object.object_name = std::string(msgMasks->class_names[i]);
        object.prob = msgMasks->scores[i];
        object.x_min = msgMasks->boxes[i].x_offset;
        object.y_min = msgMasks->boxes[i].y_offset;
        object.x_max = msgMasks->boxes[i].x_offset + msgMasks->boxes[i].width;
        object.y_max = msgMasks->boxes[i].y_offset + msgMasks->boxes[i].height;
        object.objectMask = Array2Mat(msgMasks->masks[i].data);

        vecObjects.push_back(object);

        cout << object.class_id << "  " << object.object_name << "  " << object.prob << "  " << object.x_min << "  " << object.y_min << "  " << object.x_max << "  " << object.y_max << endl;
    }
    cout << "\n" << endl;
}
*/

// Get objects from yolact_ros/Detections.msg
void ObjectsNode::GetObjects(const yolact_ros_msgs::DetectionsConstPtr& msgDetectios,
                             std::vector<Object>& vecObjects)
{
    std::cout << "Step1: GetObjects"<< std::endl;
    int objects_num = msgDetectios->detections.size();

    vecObjects.clear();

    cv::Mat All = cv::Mat::zeros(480, 640, CV_8UC1);

    for (int i=0; i<objects_num; i++)
    {
        yolact_ros_msgs::Detection detection;
        detection =  msgDetectios->detections[i];
        Object object;

        object.prob = detection.score;
        object.object_name = std::string(detection.class_name);
        //edit by dx
        if(object.object_name == "tv")
            object.object_name = "tvmonitor";
        object.x_min = detection.box.x1;
        object.y_min = detection.box.y1;
        object.x_max = detection.box.x2;
        object.y_max = detection.box.y2;
        object.objectMask = YolactMask2Mat(detection.mask, object.x_min, object.y_min);
        //cv::imwrite("/home/gxc/dataset/SceneNN/mask"+to_string(i)+".jpeg",object.objectMask);
        vecObjects.push_back(object);

        cout << "       Objects info:   " << object.object_name << "  " << object.prob << "  " << object.x_min << "  " << object.y_min << "  " << object.x_max << "  " << object.y_max << endl;
    }
    
}


cv::Mat ObjectsNode::Array2Mat(vector<uchar> array)
{
    cv::Mat M(480, 640, CV_8UC1);
    for (int i = 0; i < M.rows; ++i)
    {
        uchar *p = M.ptr<uchar>(i);
        for (int j = 0; j < M.cols; ++j)
            p[j] = array[i*M.cols + j];
    }
    //    imshow("Object Mask", M);
    //    cv::waitKey(100);
    return M;
}

cv::Mat ObjectsNode::YolactMask2Mat(const yolact_ros_msgs::Mask &mask, int x_min, int y_min)
{
//    std::cout << "Step1: YolactMask2Mat" << std::endl;

    //edit by gxc&dx
    cv::Mat M = cv::Mat::zeros(480, 640, CV_8UC1);
//    std::cout << "x_min, y_min" << x_min << ", " << y_min << std::endl;
//    std::cout << "mask_width, mask.height" << mask.width << ", " << mask.height << std::endl;

    for (int x = 0; x < mask.width; ++x) {
        for (int y = 0; y < mask.height; ++y){
            size_t index = y * mask.width + x;
            size_t byte_ind = index / 8;
            size_t bit_ind = 7 - (index % 8); // bitorder 'big'

//            std::cout << "----YolactMask2Mat1----" << std::endl;
//            std::cout << "mask.mask[byte_ind] & (1 << bit_ind)" << (mask.mask[byte_ind] & (1 << bit_ind)) << std::endl;
//            std::cout << " y, x " << y_min+y << ", " << x_min+x << std::endl;

            if(bool((mask.mask[byte_ind] & (1 << bit_ind))))
                M.at<uchar>(y_min+y, x_min+x)= 255;
        }
    }
    // static int i = 0;
    
   
    //     imshow("Object Mask", M);
    //     cv::waitKey(30);
    //     if(i==0)
    // {
    //     cv::imwrite("/home/gxc/dataset/SceneNN/mask0.jpeg",M);
    //     i++;
    // }
    return M;
}

// Get transform of keyframe
void ObjectsNode::GetKeyframeTrans(const object_map_msgs::KeyframeTransConstPtr& msgKeyframeTrans, Eigen::Isometry3d& T)
{
    std_msgs::Float64MultiArray trans = msgKeyframeTrans->keyframeTrans;
    //Eigen::VectorXd trans = msgKeyframeTrans->keyframeTrans.data[0];

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            T(i, j) = trans.data[i*4+j];
        }
    }
    // cout << "Keyframe transform = \n" << T.matrix() << endl;
    // cout << "--------------------------------------\n" << endl;
}

/*
void ObjectsNode::Update(cv::Mat img_rgb,
                         cv::Mat img_depth,
                         Eigen::Isometry3d trans2Map,
                         std::vector<Object> vecObjects)
{
    bool object_publish = false;
    if(mIsObjectDatabaseUpdate)// true: database is updated, false: database is not updated
    {
        mpObjectsBoundingBoxes->extractObjectsPointClouds(img_rgb, img_depth, trans2Map, vecObjects, mClustersCurrentKeyframe_);
        cout << "mClustersCurrentKeyframe_.size() = " << mClustersCurrentKeyframe_.size() << endl;

        if(!mClustersCurrentKeyframe_.empty())
        {
            // the first keyframe satisfied with the detected objects(prob>threshold), objects  are all added in the database, the late keyframes are merged.
            static int first_keyframe_with_objects = 1;

            for(std::vector<Cluster>::iterator itCluster = mClustersCurrentKeyframe_.begin();
                itCluster != mClustersCurrentKeyframe_.end(); ++itCluster)
            {
                //--------------------
                if(first_keyframe_with_objects == 1)
                    itCluster->first_keyframe_with_objects = true;
                else
                    itCluster->first_keyframe_with_objects = false;

                mpObjectsBoundingBoxes->mpObjectDatabase->objectPointCloudMerge(*itCluster, mIsObjectDatabaseUpdate);

                if(itCluster->flag_new_object)
                {
                    PointCloud7D::Ptr single_object_points( new PointCloud7D );
                    single_object_points = itCluster->object_point_cloud_after_seg;
                    ColorSingleObject(single_object_points);
                    *itCluster->object_point_cloud_after_seg = *single_object_points;
                    sensor_msgs::PointCloud2 cloud = ObjectsPointsToPointCloud(single_object_points);
                    std::cout << "\n" << "object point cloud for MFE 1: " << single_object_points->points.size ()  << " data points." << std::endl;

                    single_object_point_cloud_publisher_.publish(cloud);
                    object_publish = true;
                }
                else
                {
                    PointCloud7D::Ptr single_object_cloud_before_seg( new PointCloud7D );
                    single_object_cloud_before_seg = itCluster->object_point_cloud_before_seg;
                    PointCloud7D::Ptr single_object_cloud_after_filter( new PointCloud7D );
                    PointCloud7D::Ptr single_object_cloud_after_seg( new PointCloud7D );

                    // point cloud filter: 体素格滤�?1?7, 直通滤波器, 统计滤波�?1?7
                    if(!(mpObjectsBoundingBoxes->pointCloudFilter(single_object_cloud_before_seg, single_object_cloud_after_filter)))
                    {
                        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
                        continue;
                    }

                    // point cloud segmentation using LCCP
                    mpObjectsBoundingBoxes->pointCloudLCCP(single_object_cloud_after_filter, single_object_cloud_after_seg);

                    std::cout << "\n" << "object point cloud for MFE 2: " << single_object_cloud_after_seg->points.size ()  << " data points." << std::endl;

                    if(single_object_cloud_after_seg->points.size() < 20)
                    {
                        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
                        continue;
                    }

                    ColorSingleObject(single_object_cloud_after_seg);
                    *itCluster->object_point_cloud_after_seg = *single_object_cloud_after_seg;

                    sensor_msgs::PointCloud2 cloud = ObjectsPointsToPointCloud(single_object_cloud_after_seg);
                    std::cout << "\n" << "Merged object point cloud for MFE 3: " << single_object_cloud_after_seg->points.size ()  << " data points." << std::endl;

                    single_object_point_cloud_publisher_.publish(cloud);
                    object_publish = true;
                }
                mClustersAll_.push_back(*itCluster);
            }
            first_keyframe_with_objects++;
        }
    }
    if(!object_publish)
        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
}

*/

//----------------------Step2 Get object pointclouds and BBOX----------------------//
// Obtain object point cloud and calculate the 3DBB
void ObjectsNode::Update(cv::Mat img_rgb,
                         cv::Mat img_depth,
                         Eigen::Isometry3d trans2Map,
                         std::vector<Object> vecObjects)
{
    cout << "\n" <<"------------------------2----------------------------"<< endl;
    cout << "Step2: Get object pointclouds and BBOX"  << endl;
    cout << "       mIsObjectDatabaseUpdate: "  <<  mIsObjectDatabaseUpdate << endl;


    bool object_publish = false;
    if(mIsObjectDatabaseUpdate)// true: database is updated, false: database is not updated
    {
        //----------------------Step2.1 Extract Objects PointClouds----------------------//
        mpObjectsBoundingBoxes->extractObjectsPointClouds(img_rgb, img_depth, trans2Map, vecObjects, mClustersCurrentKeyframe_);
        cout << "Step2.1: Extract " << mClustersCurrentKeyframe_.size() << " objects" << endl;

        if(!mClustersCurrentKeyframe_.empty())
        {
            // the first keyframe satisfied with the detected objects(prob>threshold), objects  are all added in the database, the late keyframes are merged.
            static int first_keyframe_with_objects = 1;

            // obtain the point cloud of each object and publish to calculate the MF
            for(std::vector<Cluster>::iterator itCluster = mClustersCurrentKeyframe_.begin();
                itCluster != mClustersCurrentKeyframe_.end(); ++itCluster)
            {
                //--------------------
                if(first_keyframe_with_objects == 1)
                    itCluster->first_keyframe_with_objects = true;
                else
                    itCluster->first_keyframe_with_objects = false;



                PointCloud7D::Ptr single_object_points( new PointCloud7D );
                single_object_points = itCluster->object_point_cloud_after_seg;

                // int color_r = Random(255);
                // int color_g = Random(255);
                // int color_b = Random(255);

                ColorSingleObject(single_object_points, itCluster->centroid);
                itCluster->point_color_r = single_object_points->points[0].r;
                itCluster->point_color_g = single_object_points->points[0].g;
                itCluster->point_color_b = single_object_points->points[0].b;

                *itCluster->object_point_cloud_after_seg = *single_object_points;
                object_publish = true;

                /*
                if(itCluster->flag_new_object)
                {
                    PointCloud7D::Ptr single_object_points( new PointCloud7D );
                    single_object_points = itCluster->object_point_cloud_after_seg;
                    ColorSingleObject(single_object_points);
                    *itCluster->object_point_cloud_after_seg = *single_object_points;
                    sensor_msgs::PointCloud2 cloud = ObjectsPointsToPointCloud(single_object_points);
                    std::cout << "\n" << "object point cloud for MFE 1: " << single_object_points->points.size ()  << " data points." << std::endl;

                    single_object_point_cloud_publisher_.publish(cloud);
                    object_publish = true;
                }
                else
                {
                    PointCloud7D::Ptr single_object_cloud_before_seg( new PointCloud7D );
                    single_object_cloud_before_seg = itCluster->object_point_cloud_before_seg;
                    PointCloud7D::Ptr single_object_cloud_after_filter( new PointCloud7D );
                    PointCloud7D::Ptr single_object_cloud_after_seg( new PointCloud7D );

                    // point cloud filter: 体素格滤�?1?7, 直通滤波器, 统计滤波�?1?7
                    if(!(mpObjectsBoundingBoxes->pointCloudFilter(single_object_cloud_before_seg, single_object_cloud_after_filter)))
                    {
                        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
                        continue;
                    }

                    // point cloud segmentation using LCCP
                    mpObjectsBoundingBoxes->pointCloudLCCP(single_object_cloud_after_filter, single_object_cloud_after_seg);

                    std::cout << "\n" << "object point cloud for MFE 2: " << single_object_cloud_after_seg->points.size ()  << " data points." << std::endl;

                    if(single_object_cloud_after_seg->points.size() < 20)
                    {
                        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
                        continue;
                    }

                    ColorSingleObject(single_object_cloud_after_seg);
                    *itCluster->object_point_cloud_after_seg = *single_object_cloud_after_seg;

                    sensor_msgs::PointCloud2 cloud = ObjectsPointsToPointCloud(single_object_cloud_after_seg);
                    std::cout << "\n" << "Merged object point cloud for MFE 3: " << single_object_cloud_after_seg->points.size ()  << " data points." << std::endl;

                    single_object_point_cloud_publisher_.publish(cloud);
                    object_publish = true;
                }
                */
                //mClustersAll_.push_back(*itCluster);

                Cluster cluster;
                cluster = *itCluster;
                //----------------------Step2.2 Calculate the 3DBB----------------------//
                // calculate the 3DBB
                cout << "\n" << "Step2.2: Calculate the 3DBB"  << endl;
                Calculate3DBB(single_object_points, cluster);
                cout << "\n" << "Step2.3: Update object database"  << endl;

                //----------------------Step2.3 Update object database----------------------//
                mpObjectsBoundingBoxes->mpObjectDatabase->objectPointCloudMerge(cluster, mIsObjectDatabaseUpdate);
                cout << "         Cluster.object_update_num = " << cluster.object_update_num << endl;

//                mIsObjectDatabaseUpdate = true;
                if(cluster.flag_new_object)
                {
                    cout << "         New object, add to database" <<endl;
                    mpObjectsBoundingBoxes->mpObjectDatabase->addObject(cluster); // add to the object database
                }
                else
                {
                    cout << "         Old object, update database" <<endl;

                    PointCloud7D::Ptr single_object_cloud_before_seg( new PointCloud7D );
                    *single_object_cloud_before_seg = *cluster.object_point_cloud_before_seg;
                    PointCloud7D::Ptr single_object_cloud_after_filter( new PointCloud7D );
                    PointCloud7D::Ptr single_object_cloud_after_seg( new PointCloud7D );
                    // point cloud filter: 体素格滤�?1?7, 直通滤波器, 统计滤波�?1?7
                    if(mpObjectsBoundingBoxes->pointCloudFilter(single_object_cloud_before_seg, single_object_cloud_after_filter))
                    {
                        // point cloud segmentation using LCCP
                        //mpObjectsBoundingBoxes->pointCloudLCCP(single_object_cloud_after_filter, single_object_cloud_after_seg);
                        mpObjectsBoundingBoxes->pointCloudEuclidean(single_object_cloud_after_filter, single_object_cloud_after_seg);
                        if(single_object_cloud_after_seg->points.size() >= 20)
                        {

                            ColorSingleObject(single_object_cloud_after_seg, itCluster->centroid);
                            *cluster.object_point_cloud_after_seg = *single_object_cloud_after_seg;

                            // calculate the 3DBB
                            Calculate3DBB(single_object_points, cluster);
                            if(cluster.object_update_num != 10000)// after object fusion, need to update the objct database, the default value of cluster.object_update_num is 10000
                            {
                                mpObjectsBoundingBoxes->mpObjectDatabase->addObject(cluster); // update the object database
                            }
                            object_publish = true;
                        }
                    }
                }
                static int obj_num = 1;
                pcl::io::savePCDFileASCII("/home/zb/dataset/sceneNN/object_before_seg" + std::to_string(obj_num) + ".pcd", *itCluster->object_point_cloud_before_seg);
                pcl::io::savePCDFileASCII("/home/zb/dataset/sceneNN/object_LCCP_" + std::to_string(obj_num) + ".pcd", *itCluster->object_point_cloud_after_seg);
                obj_num++;
            }
            flagReceiveObject = true;
            first_keyframe_with_objects++;
        }
    }
    if(!object_publish)
        mIsObjectDatabaseUpdate = true; // for the next to enter the Update() function
}


//Step2.2 Main program: Calculate3DBB
void ObjectsNode::Calculate3DBB(PointCloud7D::Ptr cloud, Cluster& cluster)
{

    std::vector<autosense::ObjectPtr> allObjects;
    autosense::ObjectPtr singleObject;
    autosense::PointICloudPtr cloud_object(new autosense::PointICloud);
    std::vector<autosense::PointICloudPtr> cloud_clusters;

    int points_num = cloud->points.size();

    // delete the points beyond the scope limit in the point cloud
    for(int i = 0; i < points_num; i++)
    {
        autosense::PointI p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.intensity = (cloud->points[i].r +cloud->points[i].g + cloud->points[i].b)/3.0;
        cloud_object->points.push_back(p);
    }

    cloud_clusters.push_back(cloud_object);

    // calculate the 3DBB
    allObjects = Obtain3DBB(cloud_clusters);

    for(int i = 0; i < allObjects.size(); i++)
    {
        singleObject = allObjects[i];
//        cout << "------------------------------" << endl;
//        cout << "         The " << i << "-th object: " ;
//        cout << "direction = \n" << singleObject->direction << endl;
        cout << "         l, w, h = " << singleObject->length << " "
             << singleObject->width << " "
             << singleObject->height << endl;
//        cout << "------------------------------" << endl;
    }
//    Objects3DBB temp;
//    autosense::ObjectPtr singleObject;
//    mpObjects3DBB->CalculateSingleObject3DBB(cloud, singleObject);

    // convert the 3DBB to the required format
    mpObjectsBoundingBoxes->calculateObjectBoundingBox(cluster, singleObject, mIsObjectDatabaseUpdate);
//    cout << "Calculate3DBB: cluster.object_update_num = " << cluster.object_update_num << endl;
}

std::vector<autosense::ObjectPtr> ObjectsNode::Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters)
{
    // define object builder
    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;

    // create object builder by manager
    object_builder_ = autosense::object_builder::createObjectBuilder();

    // build 3D orientation bounding box for clustering point cloud
    //std::vector<PointICloudPtr> cloud_clusters;
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);
    return objects;
}

//----------------------Step3 Publish results----------------------//
void ObjectsNode::PublishObjectsBoundingBoxes(std::vector<Cluster> clusters)
{
    if(flagReceiveObject) // object database updates
    {
        int objnumber = clusters.size();

        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();

        std::cout<< "Step3: Object Database size: " << objnumber << std::endl;

        rviz_visual_tools::colors object_color;

        for( int m=0; m<objnumber; m++)
        {
            Cluster & cluster = clusters[m];
            Eigen::Vector3d size  = cluster.size;
            Eigen::Vector3d boxCenter  = cluster.boxCenter;

            std::cout << "       Object: " << cluster.object_name << " prob: " << cluster.prob << " boxCenter: "
                      << boxCenter[0] << " " << boxCenter[1] << " " << boxCenter[2] << " size: "
                      << size[0] << " " << size[1] << " " << size[2] << "\n"
                      << std::endl;

            string object_name = cluster.object_name;
            switch (hash_(object_name.data()))
            {
            case hash_compile_time("television"):
                object_color = rviz_visual_tools::LIME_GREEN;
                break;
            case hash_compile_time("bottle"):
                object_color = rviz_visual_tools::CYAN;
                break;
            case hash_compile_time("couch"):
                object_color = rviz_visual_tools::GREEN;
                break;
            case hash_compile_time("chair"):
                object_color = rviz_visual_tools::RED;
                break;
            case hash_compile_time("laptop"):
                object_color = rviz_visual_tools::YELLOW;
                break;
            case hash_compile_time("refrigerator"):
                object_color = rviz_visual_tools::DARK_GREY;
                break;
            case hash_compile_time("bag"):
                object_color = rviz_visual_tools::PURPLE;
                break;
            case hash_compile_time("keyboard"):
                object_color = rviz_visual_tools::ORANGE;
                break;
            case hash_compile_time("book"):
                object_color = rviz_visual_tools::PINK;
                break;
            case hash_compile_time("bed"):
                object_color = rviz_visual_tools::MAGENTA;
                break;
            case hash_compile_time("dining table"):
                object_color = rviz_visual_tools::BLUE;
                break;
            case hash_compile_time("toilet"):
                object_color = rviz_visual_tools::GREY;
                break;
            default:
                object_color = rviz_visual_tools::BROWN;
            }

            // Publish the bounding boxes to rviz
            Eigen::Vector3d min_point, max_point;
            min_point << -cluster.size(0) / 2.0, -cluster.size(1) / 2.0, -cluster.size(2) / 2.0;
            max_point << cluster.size(0) / 2.0, cluster.size(1) / 2.0, cluster.size(2) / 2.0;

            visual_tools_->publishWireframeCuboid(cluster.coordinate_system_t,
                                                  min_point, max_point, object_color,
                                                  rviz_visual_tools::LARGE); // modify the wireframe thickness
            PublishLabel(cluster.coordinate_system_t_label, cluster.object_name);

            //cout << "cluster.coordinate_system_t = \n" << cluster.coordinate_system_t.matrix() << endl;

            //visual_tools_->publishAxis(cluster.coordinate_system_t, rviz_visual_tools::LARGE);
            //visual_tools_->publishAxisLabeled(cluster.coordinate_system_t, ".", rviz_visual_tools::LARGE);
        }

        // Don't forget to trigger the publisher!
        visual_tools_->trigger();

        flagReceiveObject = false;
    }
}

void ObjectsNode::PublishLabel(const Eigen::Affine3d& pose, const std::string& label)
{
    Eigen::Affine3d pose_copy = pose;
    //pose_copy.translation().x() -= 0.1;
    visual_tools_->publishText(pose_copy, label, rviz_visual_tools::BLACK, rviz_visual_tools::XXXLARGE, false);
}

sensor_msgs::PointCloud2 ObjectsNode::PointsToPointCloud (PointCloud7D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
    }
    Point7D tmp;
    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 6; // x y z r g b

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = pointCloud->points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z", "r", "g", "b"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[6];
    for (unsigned int i=0; i<cloud.width; i++) {
        tmp = pointCloud->points.at(i);

        data_array[0] = tmp.x; //x. Do the transformation by just reading at the position of z instead of x
        data_array[1] = tmp.y; //y. Do the transformation by just reading at the position of x instead of y
        data_array[2] = tmp.z; //z. Do the transformation by just reading at the position of y instead of z
        //std::cout<<(int)tmp.r<<" "<<(int)tmp.g<<std::endl;
        data_array[3] = (float)((int)tmp.r / 255.0);
        data_array[4] = (float)((int)tmp.g / 255.0);
        data_array[5] = (float)((int)tmp.b / 255.0);

        memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 6*sizeof(float));
    }
    //std::cout<<std::endl<<std::endl;

    return cloud;
}

// Publish an object point cloud with the sequence number
sensor_msgs::PointCloud2 ObjectsNode::ObjectsPointsToPointCloud (PointCloud7D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "PointsToPointCloud: Point cloud vector is empty!" << std::endl;
    }
    Point7D tmp;
    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    static uint object_num = 0;

    cout << "Publish object num = " << object_num << endl;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.header.seq = object_num;
    cloud.height = 1;
    cloud.width = pointCloud->points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[3];
    for (unsigned int i=0; i<cloud.width; i++) {
        tmp = pointCloud->points.at(i);

        data_array[0] = tmp.x;
        data_array[1] = tmp.y;
        data_array[2] = tmp.z;

        memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 3*sizeof(float));
    }
    object_num++;

    return cloud;
}

// Publish all object point clouds for display in rviz
void ObjectsNode::PublishObjectsPointCloud(std::vector<Cluster> clusters)
{
    // PointCloud7D::Ptr object_points = mpObjectsBoundingBoxes->mObjectPoints;
    //    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(object_points);
    //    objects_point_cloud_publisher_.publish(cloud);

    PointCloud7D::Ptr all_objects_points( new PointCloud7D );
    AllObjectsPointCloud(clusters, all_objects_points);

    //    if(all_objects_points->points.size() > 0)
    //        pcl::io::savePCDFileASCII("objects.pcd", *all_objects_points);

    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(all_objects_points);
    objects_point_cloud_publisher_.publish(cloud);
}

// add all single object point cloud to one
void ObjectsNode::AllObjectsPointCloud(std::vector<Cluster> clusters, PointCloud7D::Ptr& all_objects_points)
{
    int objnumber = clusters.size();

    for( int m=0; m < objnumber; m++)
    {
        Cluster & cluster = clusters[m];
        PointCloud7D::Ptr single_object_points( new PointCloud7D );
        single_object_points = cluster.object_point_cloud_after_seg;
        *all_objects_points += *single_object_points;
    }
}

// set the color of the single object
// void ObjectsNode::ColorSingleObject(PointCloud7D::Ptr& single_object_points, int color_r, int color_g, int color_b)
// {
// //    int color_R = Random(255);
// //    int color_G = Random(255);
// //    int color_B = Random(255);

// #pragma omp parallel for   // =======================omp 多线�?1?7 并行处理

//     for (int i = 0; i < single_object_points->size(); i++)
//     {
//         single_object_points->points[i].r = color_r;
//         single_object_points->points[i].g = color_g;
//         single_object_points->points[i].b = color_b;
//     }
// }
//edit by LTX
void ObjectsNode::ColorSingleObject(PointCloud7D::Ptr& single_object_points, Eigen::Vector3d pc)
{
//    int color_R = Random(255);
//    int color_G = Random(255);
//    int color_B = Random(255);

#pragma omp parallel for   // =======================omp 多线�?1?7 并行处理

    double max = 0;
    double min = 10000000;
    for (int i = 0; i < single_object_points->size(); i++)
    {
        Eigen::Vector3d p(single_object_points->points[i].x,single_object_points->points[i].y,single_object_points->points[i].z);
        double a = (p-pc).norm();
        // a = a>=3.0?3.0:a;
        // a = a*255.0/3.0;
        //int b = (int)a;
        // single_object_points->points[i].r = 255-a;
        // single_object_points->points[i].g = a;
        // single_object_points->points[i].b = 0;
        max = a>max?a:max;
        min = a<min?a:min;
    }
#pragma omp parallel for
    for (int i = 0; i < single_object_points->size(); i++)
    {
        Eigen::Vector3d p(single_object_points->points[i].x,single_object_points->points[i].y,single_object_points->points[i].z);
        double a = (p-pc).norm();
        a = 255.0*(a-min)/(max-min);
        single_object_points->points[i].r = 255-a;
        single_object_points->points[i].g = a;
        single_object_points->points[i].b = 0;
    }
}
//----------------------Step4 Save Result----------------------//
// judge if the dataset is completed, if completed, save the results.
void ObjectsNode::DatasetCompletedCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    unique_lock<mutex> lock(mMutexSaveDatabase);
    mDatasetCompleted = (bool)(msg->data[1]);
    cout << "complete = " << mDatasetCompleted;

    if(msg->data[1]) // the dataset is completed
    {
        cout << "\n" <<"------------------------4----------------------------"<< endl;
        cout << "Step4: Save Reslt"  << endl;
        cout << "       The dataset is completed, save the object database and object point clouds to /home/gxc/dataset/SceneNN/000" << endl;

        string dataset_num;
        if(msg->data[0] < 100)
            dataset_num = "0" + std::to_string(msg->data[0]);
        else
            dataset_num = std::to_string(msg->data[0]);
        
        //attention save path
        dataset_num = "000";
   
//        string filePath = "./" + dataset_num;
//        if(mkdir(filePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) // mkdir(), 成功返回0，错误返�?1?7-1
//            cout << "\n Create a folder, named " << dataset_num << endl;
//        else
//            cout << "\n Fail to create a folder, named " << dataset_num << endl;

        string beforeFusion = "/home/zb/dataset/sceneNN/" + dataset_num + "/objectDatabaseBeforeFusion.txt";
        string afterFusion = "/home/zb/dataset/sceneNN/" + dataset_num + "/objectDatabaseAfterFusion.txt";

        cout << "       Save point cloud and 3DBB before fusion" << endl;
        ofstream writeObject3DBB;
        writeObject3DBB.open("/home/zb/dataset/sceneNN/" + dataset_num + "/object3DBB.txt");
        ofstream writeParameters;
        writeParameters.open("/home/zb/dataset/sceneNN/" + dataset_num + "/parameters.txt");

        std::vector<Cluster> clusters_ = mpObjectsBoundingBoxes->mpObjectDatabase->getObjectClusters();
        WriteDatabase2File(beforeFusion, clusters_); // write object database to text

        // save all objects point cloud before fusion
        string all_objects_before_fusion = "/home/zb/dataset/sceneNN/" + dataset_num + "/objectDatabaseBeforeFusion.pcd";
        SaveAllObjectsPointCloud(clusters_, all_objects_before_fusion);

        cout << "       Database Merging" << endl;
        mpObjectsBoundingBoxes->mpObjectDatabase->objectDatabaseMerge();

        cout << "       Save point cloud and 3DBB after fusion" << endl;
        std::vector<Cluster> clusters = mpObjectsBoundingBoxes->mpObjectDatabase->getObjectClusters();
        WriteDatabase2File(afterFusion, clusters);

        // save all objects point cloud after fusion
        string all_objects_after_fusion = "/home/zb/dataset/sceneNN/" + dataset_num + "/objectDatabaseAfterFusion.pcd";
        SaveAllObjectsPointCloud(clusters, all_objects_after_fusion);

        cout << "       Save single object to PCD file" << endl;

        for(int i = 0; i < clusters.size(); i++)
        {
            Cluster cluster = clusters[i];

            // save each object point cloud
            if(cluster.object_point_cloud_after_seg->points.size() > 0)
            {
                pcl::io::savePCDFileASCII("/home/zb/dataset/sceneNN/" + dataset_num + "/object_" + std::to_string(cluster.object_id) + "_" + cluster.object_name + ".pcd", *cluster.object_point_cloud_after_seg);
            }

            writeObject3DBB << cluster.object_name;
            for(int n = 0; n < 10; n++)
            {
                writeObject3DBB << " " << cluster.bb3d[n];
            }
            writeObject3DBB << endl;

        }
        writeObject3DBB.close();

        cout << "       Save parameters." << endl;
        int num1 = mMaskRcnnDetectedObjectsNum;
        int num2 = mpObjectsBoundingBoxes->mObjectsProbBeyondThreshold;
        int num3 = mpObjectsBoundingBoxes->mpObjectDatabase->mDeleteObjectNum; // delete small objects
        int num4 = mpObjectsBoundingBoxes->mpObjectDatabase->mUpdateObjectNum; // update the object with the same label
        int num5 = mpObjectsBoundingBoxes->mpObjectDatabase->mDeleteIntersectionNum; // delete the small object with intersection
        //int num4 = mpObjectsBoundingBoxes->mpObjectDatabase->mUpdateIntersectionNum; // new object, with the different object label, update

        writeParameters << "mKeyframeNum = " << mKeyframeNum << endl;
        writeParameters << "mMaskRcnnDetectedObjectsNum = " << num1 << endl;
        writeParameters << "mObjectsProbBeyondThreshold = " << num2 << endl;
        writeParameters << "mReceivedMFENum = " << mReceivedMFENum << endl; // all received MFE number
        writeParameters << "mDeleteObjectNum = " << num3 << endl;
        writeParameters << "mUpdateObjectNum = " << num4 << endl;
        writeParameters << "mDeleteIntersectionNum = " << num5 << endl;

        cout << "----------------------------" << endl << endl;
        cout << "mKeyframeNum = " << mKeyframeNum << endl;
        cout << "mMaskRcnnDetectedObjectsNum = " << num1 << endl;
        cout << "mObjectsProbBeyondThreshold = " << num2 << endl;
        cout << "mReceivedMFENum = " << mReceivedMFENum << endl; // all received MFE number
        cout << "mDeleteObjectNum = " << num3 << endl;
        cout << "mUpdateObjectNum = " << num4 << endl;
        cout << "mDeleteIntersectionNum = " << num5 << endl;
        //cout << "mUpdateIntersectionNum = " << num4 << endl;

        writeParameters << "Mean keyframe update time = " << MeanTime(mvTimeKeyframeUpdate) << " ms" << endl;
        writeParameters << "Mean MFE time = " << MeanTime(mvTimeMFE) << " ms" << endl;
        writeParameters << "Mean object extraction and filter time = " << MeanTime(mpObjectsBoundingBoxes->mvTimeExtractPointCloudAndFilter) << " ms" << endl;
        writeParameters << "Mean object LCCP time = " << MeanTime(mpObjectsBoundingBoxes->mvTimeLCCP) << " ms" << endl;
        writeParameters << "Mean object fusion time = " << MeanTime(mpObjectsBoundingBoxes->mvTimeFusion) << " ms" << endl;

        cout << "----------------------------" << endl << endl;
        cout << "Mean keyframe update time: " << MeanTime(mvTimeKeyframeUpdate) << " ms" << endl;
        cout << "Mean MFE time: " << MeanTime(mvTimeMFE) << " ms" << endl;
        cout << "Mean object extraction and filter time: " << MeanTime(mpObjectsBoundingBoxes->mvTimeExtractPointCloudAndFilter) << " ms" << endl;
        cout << "Mean object LCCP time: " << MeanTime(mpObjectsBoundingBoxes->mvTimeLCCP) << " ms" << endl;
        cout << "Mean object fusion time: " << MeanTime(mpObjectsBoundingBoxes->mvTimeFusion) << " ms\n" << endl;

        writeParameters.close();
    }
}

void ObjectsNode::WriteDatabase2File(string file_path, std::vector<Cluster> clusters)
{
    ofstream file;
    file.open(file_path);

    for(int i = 0; i < clusters.size(); i++)
    {
        Cluster cluster = clusters[i];

        Eigen::Affine3d T1(cluster.coordinate_system_t);
        Eigen::Affine3d T2(cluster.coordinate_system_t_label);

        file << cluster.object_name << " " \
             << "size" << " " \
             << cluster.size(0) << " " << cluster.size(1) << " " << cluster.size(2) << " " \
             << "boxCenter" << " " \
             << cluster.boxCenter(0) << " " << cluster.boxCenter(1) << " " << cluster.boxCenter(2) << " " \
             << "cluster.coordinate_system_t" << " " \
             << T1(0, 0) << " " << T1(0, 1) << " " << T1(0, 2) << " " << T1(0, 3) << " " \
             << T1(1, 0) << " " << T1(1, 1) << " " << T1(1, 2) << " " << T1(1, 3) << " " \
             << T1(2, 0) << " " << T1(2, 1) << " " << T1(2, 2) << " " << T1(2, 3) << " " \
             << T1(3, 0) << " " << T1(3, 1) << " " << T1(3, 2) << " " << T1(3, 3) << " " \
             << "cluster.coordinate_system_t_label" << " " \
             << T2(0, 0) << " " << T2(0, 1) << " " << T2(0, 2) << " " << T2(0, 3) << " " \
             << T2(1, 0) << " " << T2(1, 1) << " " << T2(1, 2) << " " << T2(1, 3) << " " \
             << T2(2, 0) << " " << T2(2, 1) << " " << T2(2, 2) << " " << T2(2, 3) << " " \
             << T2(3, 0) << " " << T2(3, 1) << " " << T2(3, 2) << " " << T2(3, 3) << " " \
             << endl;

    }
    file.close();
}


double ObjectsNode::MeanTime(vector<double> time)
{
    double totaltime = 0;
    int num = 1;
    for(int n = 0; n < time.size(); n++)
    {
        if(time[n] > 0)
        {
            totaltime += time[n];
            num++;
        }
    }
    double mean_time = totaltime / num;
    return mean_time;
}

void ObjectsNode::SaveAllObjectsPointCloud(std::vector<Cluster> clusters, string point_cloud_name)
{
    PointCloud7D::Ptr all_objects_points( new PointCloud7D );
    AllObjectsPointCloud(clusters, all_objects_points);

    cout << "       All points size: " << all_objects_points->points.size()<< endl;

    if(all_objects_points->points.size() > 0)
    {
        //cout << "---------00---------" << endl;
        pcl::io::savePCDFileASCII(point_cloud_name, *all_objects_points);
        cout << "       Save all objects point cloud successfully." << endl;
    }
    else
    {
        cout << "        Fail to save all objects point cloud." << endl;

    }
}


