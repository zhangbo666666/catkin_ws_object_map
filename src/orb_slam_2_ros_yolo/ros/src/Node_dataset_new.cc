#include "Node_dataset_new.h"
#include <sys/time.h>
#include <iostream>

// 计时
long _getTimeUsec()
{
    struct timeval t;
    gettimeofday(&t,0);
    return (long)((long)t.tv_sec*1000*1000 + t.tv_usec);
}

NodeDatasetNew::NodeDatasetNew (ros::NodeHandle &node_handle,
                                image_transport::ImageTransport &image_transport)
{
    name_of_node_ = ros::this_node::getName();
    //orb_slam_ = pSLAM;
    node_handle_ = node_handle;
    globalMap = boost::make_shared< PointCloud7D >( );

    InitParameters ();

    //rendered_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/debug_image", 1);
    keyframe_rgb_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_rgb", 100);
    keyframe_depth_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_depth", 100);
    keyframe_transform_publisher_ = node_handle_.advertise<object_map_msgs::KeyframeTrans> ("/orb_slam2_rgbd/keyframe_transform", 100);
    keyframe_thermal_image_publisher_= image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_thermal", 100);
    keyframe_temp_image_publisher_= image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_temp", 100);
    //    if (publish_pointcloud_param_) {
    //        map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/orb_slam2_rgbd/map_points", 1);
    //    }

    point_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/orb_slam2_rgbd/map_point_cloud", 100);

    //声明advertise，octomap rviz plug in 默认接受topic为octomap_full的message
    octomap_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>("/octomap_full", 1, true);
    color_octomap_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>("/color_octomap_full", 1, true);

    //Check settings file
    cv::FileStorage fsSettings("/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/orb_slam2/config/ASUS.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << "orb_slam_2_ros_yolo/orb_slam2/config/RealSenseRGBD.yaml" << endl;
        exit(-1);
    }

    // for point cloud resolution
    mResolution = fsSettings["PointCloudMapping.Resolution"];
    mFx = fsSettings["Camera.fx"];
    mFy = fsSettings["Camera.fy"];
    mCx = fsSettings["Camera.cx"];
    mCy = fsSettings["Camera.cy"];

    // mDepthMapFactor = fsSettings["DepthMapFactor"]; // run using a RGB-D sensor
    mDepthMapFactor = fsSettings["DepthMapFactorForDataset"]; // run in the dataset
    mDepthMapFactor = 1.0f / mDepthMapFactor;
}

NodeDatasetNew::~NodeDatasetNew () {

}

void NodeDatasetNew::Update(const cv::Mat & msgRGB,
                            const cv::Mat & msgD,
                            const 
                            bool bIsKeyframe, // is keyframe?
                            cv::Mat Tcw) // transform matrix
//void NodeDatasetNew::Update(const cv::Mat & msgRGB,
//                  const cv::Mat & msgD,
//                  bool bIsKeyframe, // is keyframe?
//                  cv::Mat Tcw) // transform matrix
{
    //cv::Mat position = orb_slam_->GetCurrentPosition();

    // Get the RGB and Depth images of keyframes
    if(bIsKeyframe)
    {
        PublishKeyFrameRGBImage(msgRGB);
        PublishKeyFrameDepthImage(msgD);
    }

    //    if (!position.empty()) {
    //        PublishPositionAsTransform (position);
    //    }

    // Publish the keyframe transform matrix with time stamp
    if (!Tcw.empty()) {
        //        tf::Transform transform = TransformFromMat (Tcw); // Transform Tcw to the map
        //        PublishKeyframeTransform2Map (transform);

        PublishKeyframeTransform(Tcw);
    }

    //PublishRenderedImage (orb_slam_->DrawCurrentFrame());

    //    if (publish_pointcloud_param_) {
    //        PublishMapPoints (orb_slam_->GetAllMapPoints());
    //    }

    // Publish point cloud
    //PublishPointCloud (orb_slam_->GetAllPointCloud());

    UpdateParameters ();
}

void NodeDatasetNew::Update(string datasetNum,
                            int updateNum,
                            const cv::Mat & msgRGB,
                            const cv::Mat & msgD,
                            bool bIsKeyframe, // is keyframe?
                            cv::Mat Tcw) // transform matrix
{
    //cv::Mat position = orb_slam_->GetCurrentPosition();

    // Get the RGB and Depth images of keyframes
    if(bIsKeyframe)
    {
        PublishKeyFrameRGBImage(msgRGB);
        PublishKeyFrameDepthImage(msgD);
    }

    //    if (!position.empty()) {
    //        PublishPositionAsTransform (position);
    //    }

    // Publish the keyframe transform matrix with time stamp
    if (!Tcw.empty()) {
        //        tf::Transform transform = TransformFromMat (Tcw); // Transform Tcw to the map
        //        PublishKeyframeTransform2Map (transform);

        PublishKeyframeTransform(Tcw);
    }

    //PublishRenderedImage (orb_slam_->DrawCurrentFrame());

    //    if (publish_pointcloud_param_) {
    //        PublishMapPoints (orb_slam_->GetAllMapPoints());
    //    }

    // Publish point cloud
    PointCloud7D::Ptr p(new PointCloud7D());
    p = generatePointCloud(msgRGB, msgD, Tcw);

    *globalMap += *p;

    PointCloud7D::Ptr tmp(new PointCloud7D());

    if(globalMap->points.size() > 100)
    {
        voxel.setLeafSize(mResolution, mResolution, mResolution);
        voxel.setInputCloud(globalMap);
        voxel.filter(*tmp);
        globalMap->swap(*tmp);
    }

    //cout << "show global map, size = " << globalMap->points.size() << endl;

    //    static int num = 1;
    //    if((num % 10 == 0) && (globalMap->points.size() > 0))
    //    {
    //        pcl::io::savePCDFileASCII("map.pcd", *globalMap);
    //    }
    //    num++;

    static int num = 0;
    //cout << "num = " << num << endl;
    //cout << "updateNum = " << updateNum << endl;

    std::cout << "datasetNum: " << datasetNum << std::endl;
    if((num == updateNum) && (globalMap->points.size() > 0))
    {
        cout << "Save global map, size = " << globalMap->points.size() << endl;
        pcl::io::savePCDFileASCII("./" + datasetNum + "/map.pcd", *globalMap);

        SaveOctoMap(globalMap, 0.05, "./" + datasetNum + "/octomap.bt");
        SaveColorOctoMap(globalMap, 0.05, "./" + datasetNum + "/octomap_color.ot");
    }
    else
        num++;

    PublishPointCloud(globalMap);

    //    long time = _getTimeUsec(); // 开始计�?

    //    PublishOctoMap(globalMap, 0.05);
    //    // PublishColorOctoMap(globalMap, 0.05);

    //    time = _getTimeUsec() - time; // 结束计时, ms
    //    cout << "Create octomap time: " << time / 1000.0 << " ms" << endl;

    UpdateParameters();
}

void NodeDatasetNew::Update(string datasetNum,
                            int updateNum,
                            const cv::Mat & msgRGB,
                            const cv::Mat & msgD,
                            const cv::Mat & msgThermal,
                            const cv::Mat & msgTemp,
                            bool bIsKeyframe, // is keyframe?
                            cv::Mat Tcw) // transform matrix
{
    //cv::Mat position = orb_slam_->GetCurrentPosition();

    // Get the RGB and Depth images of keyframes
    if(bIsKeyframe)
    {
        struct timeval time_t;
        gettimeofday(&time_t,NULL);
        double START_TIME=time_t.tv_usec*1e-3;
        cout<<endl;
        cout<<START_TIME<<endl;
        PublishKeyFrameRGBImage(msgRGB);
        PublishKeyFrameDepthImage(msgD);

        PublishKeyFrameThermalImage(msgThermal);
        PublishKeyFrameTempImage(msgTemp);
    }

    //    if (!position.empty()) {
    //        PublishPositionAsTransform (position);
    //    }

    // Publish the keyframe transform matrix with time stamp
    if (!Tcw.empty()) {
        //        tf::Transform transform = TransformFromMat (Tcw); // Transform Tcw to the map
        //        PublishKeyframeTransform2Map (transform);

        PublishKeyframeTransform(Tcw);
    }

    //PublishRenderedImage (orb_slam_->DrawCurrentFrame());

    //    if (publish_pointcloud_param_) {
    //        PublishMapPoints (orb_slam_->GetAllMapPoints());
    //    }

//    // Publish point cloud
    PointCloud7D::Ptr p(new PointCloud7D());

    // cout << "rows = " << msgD.rows << endl;
    // cout << "cols = " << msgRGB.cols << endl;
    
    p = generatePointCloud(msgRGB, msgD, Tcw);
    // cout<<"1111"<<endl;

    *globalMap += *p;

    PointCloud7D::Ptr tmp(new PointCloud7D());

    if(globalMap->points.size() > 100)
    {
        voxel.setLeafSize(mResolution, mResolution, mResolution);
        voxel.setInputCloud(globalMap);
        voxel.filter(*tmp);
        globalMap->swap(*tmp);
    }

//     //cout << "show global map, size = " << globalMap->points.size() << endl;

//     //    static int num = 1;
//     //    if((num % 10 == 0) && (globalMap->points.size() > 0))
//     //    {
//     //        pcl::io::savePCDFileASCII("map.pcd", *globalMap);
//     //    }
//     //    num++;

//     static int num = 0;
//     //cout << "num = " << num << endl;
//     cout << "updateNum = " << updateNum << endl;
//     cout << "datasetNum = " << datasetNum << endl;
    
//     if((num == updateNum) && (globalMap->points.size() > 0))
//     {
//         cout << "Save global map, size = " << globalMap->points.size() << endl;
//         //attention save path
//         pcl::io::savePCDFileASCII("/home/gxc/dataset/SceneNN/" + datasetNum + "/map.pcd", *globalMap);

// //        SaveOctoMap(globalMap, 0.05, "./" + datasetNum + "/octomap.bt");
// //        SaveColorOctoMap(globalMap, 0.05, "./" + datasetNum + "/octomap_color.ot");
//     }
//     else
//         num++;
//         cout<<"num is "<<num<<endl;

    PublishPointCloud(globalMap);

//        long time = _getTimeUsec(); // 开始计�?

//        PublishOctoMap(globalMap, 0.05);
//        // PublishColorOctoMap(globalMap, 0.05);

//        time = _getTimeUsec() - time; // 结束计时, ms
//        cout << "Create octomap time: " << time / 1000.0 << " ms" << endl;

    UpdateParameters();
}

// 将pcd点云转换为octomap
void NodeDatasetNew::SaveOctoMap(PointCloud7D::Ptr cloud, double resolution, string outputFile)
{
    // 创建八叉树对象，参数为分辨率(m)
    octomap::OcTree tree(resolution);

    for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap�?
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary(outputFile);
    cout<<"\n Save point cloud into octomap ... \n"<<endl;
}

// 将pcd点云转换为color octomap
void NodeDatasetNew::SaveColorOctoMap(PointCloud7D::Ptr cloud, double resolution, string outputFile)
{
    // 创建八叉树对象，参数为分辨率(m)
    octomap::ColorOcTree tree(resolution);

    for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap�?
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 设置颜色
    for (auto p:cloud->points)
    {
        tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.write(outputFile); // 存储octomap, 注意要存�?.ot文件而非.bt文件

    cout<<"\n Save point cloud into octomap with color ... \n"<<endl;
}

// 将pcd点云转换为octomap
void NodeDatasetNew::PublishOctoMap(PointCloud7D::Ptr cloud, double resolution)
{
    octomap::OcTree* octomap_ = new octomap::OcTree(resolution);
    for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap�?
        octomap_->updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    octomap_->updateInnerOccupancy();

    //声明message
    octomap_msgs::Octomap map_msg;
    //设置header
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time::now();
    //fullMapToMsg负责转换成message
    if (octomap_msgs::fullMapToMsg(*octomap_, map_msg))
        //转换成功，可以发布了
        octomap_publisher_.publish(map_msg);
    else
        ROS_ERROR("Error serializing OctoMap");
}

// 将pcd点云转换为color octomap
void NodeDatasetNew::PublishColorOctoMap(PointCloud7D::Ptr cloud, double resolution)
{
    octomap::ColorOcTree* octomap_ = new octomap::ColorOcTree(resolution);

    for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap�?
        octomap_->updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 设置颜色
    for (auto p:cloud->points)
    {
        octomap_->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    // 更新octomap
    octomap_->updateInnerOccupancy();

    //声明message
    octomap_msgs::Octomap map_msg;
    //设置header
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time::now();
    //fullMapToMsg负责转换成message
    if (octomap_msgs::fullMapToMsg(*octomap_, map_msg))
        //转换成功，可以发布了
        color_octomap_publisher_.publish(map_msg);
    else
        ROS_ERROR("Error serializing OctoMap");
}


//void NodeDatasetNew::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
//    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
//    map_points_publisher_.publish (cloud);
//}

void NodeDatasetNew::PublishPointCloud (PointCloud7D::Ptr pointCloud) {
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud (pointCloud);
    point_cloud_publisher_.publish (cloud);
}

//void NodeDatasetNew::PublishPositionAsTransform (cv::Mat position) {
//    tf::Transform transform = TransformFromMat (position);
//    static tf::TransformBroadcaster tf_broadcaster;
//    tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
//}

// Publish the transform of keyframe to map with the time stamp
void NodeDatasetNew::PublishKeyframeTransform2Map (tf::Transform transform) {
    object_map_msgs::KeyframeTrans keyframeTrans_;
    keyframeTrans_.header.stamp = current_frame_time_;
    keyframeTrans_.header.frame_id = map_frame_id_param_;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    tf::Quaternion q;
    q = transform.getRotation();
    tf::Matrix3x3 m(q);

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

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            keyframeTrans_.keyframeTrans.data.push_back(T(i, j));

    keyframe_transform_publisher_.publish(keyframeTrans_);
}


// Publish the transform of keyframe with the time stamp, not to the map
void NodeDatasetNew::PublishKeyframeTransform (cv::Mat pose) {
    object_map_msgs::KeyframeTrans keyframeTrans_;
    keyframeTrans_.header.stamp = current_frame_time_;
    keyframeTrans_.header.frame_id = map_frame_id_param_;

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            keyframeTrans_.keyframeTrans.data.push_back(pose.at<float>(i, j));

    keyframe_transform_publisher_.publish(keyframeTrans_);
}

//void Node::PublishRenderedImage (cv::Mat image) {
//    std_msgs::Header header;
//    header.stamp = current_frame_time_;
//    header.frame_id = map_frame_id_param_;
//    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
//    rendered_image_publisher_.publish(rendered_image_msg);
//}

// Publish the RGB image of a keyframe
// void NodeDatasetNew::PublishKeyFrameRGBImage (const cv::Mat &imRGB) {
void NodeDatasetNew::PublishKeyFrameRGBImage (const sensor_msgs::ImageConstPtr& msgRGB) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    //const sensor_msgs::ImagePtr keyframe_rgb_msg = cv_bridge::CvImage(header, "bgr8", imRGB).toImageMsg();
    keyframe_rgb_image_publisher_.publish(msgRGB);
}

void NodeDatasetNew::PublishKeyFrameRGBImage (const cv::Mat &imRGB) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_rgb_msg = cv_bridge::CvImage(header, "bgr8", imRGB).toImageMsg();
    keyframe_rgb_image_publisher_.publish(keyframe_rgb_msg);
}

// Publish the depth image of a keyframe
//void NodeDatasetNew::PublishKeyFrameDepthImage (const cv::Mat &imDepth) {
void NodeDatasetNew::PublishKeyFrameDepthImage (const sensor_msgs::ImageConstPtr& msgD) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    //const sensor_msgs::ImagePtr keyframe_depth_msg = cv_bridge::CvImage(header, "mono16", imDepth).toImageMsg();
    keyframe_depth_image_publisher_.publish(msgD);
}

void NodeDatasetNew::PublishKeyFrameDepthImage (const cv::Mat &imDepth) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_depth_msg = cv_bridge::CvImage(header, "mono16", imDepth).toImageMsg();
    keyframe_depth_image_publisher_.publish(keyframe_depth_msg);
}

void NodeDatasetNew::PublishKeyFrameThermalImage (const cv::Mat &imThermal) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_thermal_msg = cv_bridge::CvImage(header, "bgr8", imThermal).toImageMsg();
    keyframe_thermal_image_publisher_.publish(keyframe_thermal_msg);
}

void NodeDatasetNew::PublishKeyFrameTempImage (const cv::Mat &imTemp) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_temp_msg = cv_bridge::CvImage(header, "mono16", imTemp).toImageMsg();
    keyframe_temp_image_publisher_.publish(keyframe_temp_msg);
}


tf::Transform NodeDatasetNew::TransformFromMat (cv::Mat position_mat) {
    cv::Mat rotation(3,3,CV_32F);
    cv::Mat translation(3,1,CV_32F);

    rotation = position_mat.rowRange(0,3).colRange(0,3).t();
    translation = rotation*position_mat.rowRange(0,3).col(3);

    tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                      rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                      rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                      );

    tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

    const tf::Matrix3x3 Rx (1, 0, 0,
                            0, 0, -1,
                            0, 1, 0);

    const tf::Matrix3x3 Rz (0, -1, 0,
                            1, 0, 0,
                            0, 0, 1);

    const tf::Matrix3x3 invX (-1, 0, 0,
                              0, 1, 0,
                              0, 0, 1);

    const tf::Matrix3x3 invYZ (1, 0, 0,
                               0, -1, 0,
                               0, 0, -1);

    tf_camera_rotation = Rx*tf_camera_rotation;
    tf_camera_rotation = Rz*tf_camera_rotation;
    tf_camera_translation = Rx*tf_camera_translation;
    tf_camera_translation = Rz*tf_camera_translation;

    tf_camera_rotation = invYZ*tf_camera_rotation;
    tf_camera_translation = invX*tf_camera_translation;

    return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


//sensor_msgs::PointCloud2 NodeDatasetNew::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
//    if (map_points.size() == 0) {
//        std::cout << "Map point vector is empty!" << std::endl;
//    }

//    sensor_msgs::PointCloud2 cloud;

//    const int num_channels = 3; // x y z

//    cloud.header.stamp = current_frame_time_;
//    cloud.header.frame_id = map_frame_id_param_;
//    cloud.height = 1;
//    cloud.width = map_points.size();
//    cloud.is_bigendian = false;
//    cloud.is_dense = true;
//    cloud.point_step = num_channels * sizeof(float);
//    cloud.row_step = cloud.point_step * cloud.width;
//    cloud.fields.resize(num_channels);

//    std::string channel_id[] = { "x", "y", "z"};
//    for (int i = 0; i<num_channels; i++) {
//        cloud.fields[i].name = channel_id[i];
//        cloud.fields[i].offset = i * sizeof(float);
//        cloud.fields[i].count = 1;
//        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
//    }

//    cloud.data.resize(cloud.row_step * cloud.height);

//    unsigned char *cloud_data_ptr = &(cloud.data[0]);

//    float data_array[3];
//    for (unsigned int i=0; i<cloud.width; i++) {
//        if (!map_points.at(i)->isBad()) {
////            data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
////            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
////            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
//            data_array[0] = map_points.at(i)->GetWorldPos().at<float> (0); //x. Do the transformation by just reading at the position of z instead of x
//            data_array[1] = map_points.at(i)->GetWorldPos().at<float> (1); //y. Do the transformation by just reading at the position of x instead of y
//            data_array[2] = map_points.at(i)->GetWorldPos().at<float> (2); //z. Do the transformation by just reading at the position of y instead of z


//            //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

//            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 3*sizeof(float));
//        }
//    }

//    return cloud;
//}

sensor_msgs::PointCloud2 NodeDatasetNew::PointsToPointCloud (PointCloud7D::Ptr pointCloud) {
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
    }
    Point7D tmp;
    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 6; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = map_frame_id_param_;
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

        //        data_array[0] = tmp.z; //x. Do the transformation by just reading at the position of z instead of x
        //        data_array[1] = -tmp.x; //y. Do the transformation by just reading at the position of x instead of y
        //        data_array[2] = -tmp.y; //z. Do the transformation by just reading at the position of y instead of z

        data_array[0] = tmp.x; //x. Do the transformation by just reading at the position of z instead of x
        data_array[1] = tmp.y; //y. Do the transformation by just reading at the position of x instead of y
        data_array[2] = tmp.z; //z. Do the transformation by just reading at the position of y instead of z

        data_array[3] = (float)((int)tmp.r / 255.0);
        data_array[4] = (float)((int)tmp.g / 255.0);
        data_array[5] = (float)((int)tmp.b / 255.0);
        //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

        memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 6*sizeof(float));
    }

    return cloud;
}

void NodeDatasetNew::InitParameters () {
    node_handle_.param(name_of_node_+"/publish_pointcloud", publish_pointcloud_param_, true);
    node_handle_.param(name_of_node_+"/localize_only", localize_only_param_, false);
    node_handle_.param(name_of_node_+"/reset_map", reset_map_param_, false);
    node_handle_.param<std::string>(name_of_node_+"/pointcloud_frame_id", map_frame_id_param_, "map");
    node_handle_.param<std::string>(name_of_node_+"/camera_frame_id", camera_frame_id_param_, "camera_link");
    node_handle_.param(name_of_node_+"/min_num_kf_in_map", minimum_num_of_kf_in_map_param_, 5);

    //orb_slam_->SetMinimumKeyFrames (minimum_num_of_kf_in_map_param_);
}


void NodeDatasetNew::UpdateParameters () {
    node_handle_.param(name_of_node_+"/localize_only", localize_only_param_, false);
    //orb_slam_->EnableLocalizationOnly (localize_only_param_);

    node_handle_.param(name_of_node_+"/reset_map", reset_map_param_, false);
    //    if (reset_map_param_) {
    //        orb_slam_->Reset();
    //        node_handle_.setParam (name_of_node_+"/reset_map", false);
    //    }

    node_handle_.param(name_of_node_+"/min_num_kf_in_map", minimum_num_of_kf_in_map_param_, 5);
    //orb_slam_->SetMinimumKeyFrames (minimum_num_of_kf_in_map_param_);
}

PointCloud7D::Ptr NodeDatasetNew::generatePointCloud(cv::Mat color, cv::Mat depth, cv::Mat Tcw)
{
    PointCloud7D::Ptr tmp( new PointCloud7D() );

    // Convert the depth image using mDepthMapFactor
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || depth.type()!=CV_32F)
    {
        depth.convertTo(depth, CV_32F, mDepthMapFactor);
    }
    
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d > 5.5)
            {
                continue;
            }

            Point7D p;
            p.z = d;
            // cout << "z = " << d << endl;

            p.x = ( n - mCx) * p.z / mFx;
            p.y = ( m - mCy) * p.z / mFy;

            // cout << "111 = " << endl;

            // for dataset, to display the right color
//            p.b = color.ptr<uchar>(m)[n*3];
//            p.g = color.ptr<uchar>(m)[n*3+1];
//            p.r = color.ptr<uchar>(m)[n*3+2];

            // for real experiments
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
            // cout << "222 = " << endl;

            tmp->points.push_back(p);
        }
    }

//    tmp->width = 1;
//    tmp->height = tmp->points.size();

//    static int num = 0;
//    pcl::io::savePCDFileASCII("map_" + to_string(num) + ".pcd", *tmp);
//    num++;

    Eigen::Isometry3d T =  Eigen::Isometry3d::Identity();

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            T(i, j) = Tcw.at<float>(i, j);
        }
    }

    PointCloud7D::Ptr cloud(new PointCloud7D);
    if(tmp->points.size() > 0)
    {
        //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
        pcl::transformPointCloud(*tmp, *cloud, T.matrix());
    }
    else
        cout << "The input point cloud is empty!" << endl;

    // 对点云坐标做变换，绕x轴旋�?90度，将z轴指向上�?
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    //trans.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)));
 
    trans.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(1,0,0)));
    pcl::transformPointCloud(*cloud, *cloud, trans);

    cloud->is_dense = false;

    cout << "generate point cloud size = " << cloud->points.size() << endl;
    return cloud;
}

