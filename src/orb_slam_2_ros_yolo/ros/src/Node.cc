#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle,
            image_transport::ImageTransport &image_transport)
{
    name_of_node_ = ros::this_node::getName();
    orb_slam_ = pSLAM;
    node_handle_ = node_handle;

    InitParameters ();

    rendered_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/debug_image", 1);
    keyframe_rgb_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_rgb", 1);
    keyframe_depth_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/keyframe_image_depth", 1);
    keyframe_transform_publisher_ = node_handle_.advertise<object_map_msgs::KeyframeTrans> ("/orb_slam2_rgbd/keyframe_transform", 1);

    // if (publish_pointcloud_param_) {
        map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/orb_slam2_rgbd/map_points", 1);
    // }

    point_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/orb_slam2_rgbd/map_point_cloud", 1);
}

Node::~Node () {

}

void Node::Update(const sensor_msgs::ImageConstPtr& msgRGB,
                  const sensor_msgs::ImageConstPtr& msgD,
                  bool bIsKeyframe, // is keyframe?
                  cv::Mat Tcw) // transform matrix
//void Node::Update(const cv::Mat & msgRGB,
//                  const cv::Mat & msgD,
//                  bool bIsKeyframe, // is keyframe?
//                  cv::Mat Tcw) // transform matrix
{
    if(bIsKeyframe)
    {
        cv::Mat position = orb_slam_->GetCurrentPosition();

        // Get the RGB and Depth images of keyframes
        PublishKeyFrameRGBImage(msgRGB);
        PublishKeyFrameDepthImage(msgD);

        if (!position.empty()) {
            PublishPositionAsTransform (position);
        }

        // Publish the keyframe transform matrix with time stamp
        if (!Tcw.empty()) {
            //        tf::Transform transform = TransformFromMat (Tcw); // Transform Tcw to the map
            //        PublishKeyframeTransform2Map (transform);

            PublishKeyframeTransform(Tcw);
        }

        //PublishRenderedImage (orb_slam_->DrawCurrentFrame());

            //    if (publish_pointcloud_param_) {
                   PublishMapPoints (orb_slam_->GetAllMapPoints());
            //    }

        // Publish point cloud
        PublishPointCloud (orb_slam_->GetAllPointCloud());

        UpdateParameters ();
    }
}

void Node::Update(const cv::Mat & msgRGB,
                  const cv::Mat & msgD,
                  bool bIsKeyframe, // is keyframe?
                  cv::Mat Tcw) // transform matrix
{
    if(bIsKeyframe)
    {
        cv::Mat position = orb_slam_->GetCurrentPosition();

        // Get the RGB and Depth images of keyframes

        PublishKeyFrameRGBImage(msgRGB);
        PublishKeyFrameDepthImage(msgD);


        if (!position.empty()) {
            PublishPositionAsTransform (position);
        }

        // Publish the keyframe transform matrix with time stamp
        if (!Tcw.empty()) {
            //        tf::Transform transform = TransformFromMat (Tcw); // Transform Tcw to the map
            //        PublishKeyframeTransform2Map (transform);

            PublishKeyframeTransform(Tcw);
        }

        //    PublishRenderedImage (orb_slam_->DrawCurrentFrame());

        //    if (publish_pointcloud_param_) {
               PublishMapPoints (orb_slam_->GetAllMapPoints());
        //    }

        // Publish point cloud
        PublishPointCloud (orb_slam_->GetAllPointCloud());

        UpdateParameters ();
    }
}

void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
    map_points_publisher_.publish (cloud);
}

void Node::PublishPointCloud (PointCloud7D::Ptr pointCloud) {
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud (pointCloud);
    point_cloud_publisher_.publish (cloud);
}

void Node::PublishPositionAsTransform (cv::Mat position) {
    tf::Transform transform = TransformFromMat (position);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
}

// Publish the transform of keyframe to map with the time stamp
void Node::PublishKeyframeTransform2Map (tf::Transform transform) {
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
void Node::PublishKeyframeTransform (cv::Mat pose) {
    object_map_msgs::KeyframeTrans keyframeTrans_;
    keyframeTrans_.header.stamp = current_frame_time_;
    keyframeTrans_.header.frame_id = map_frame_id_param_;

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            keyframeTrans_.keyframeTrans.data.push_back(pose.at<float>(i, j));

    keyframe_transform_publisher_.publish(keyframeTrans_);
}

void Node::PublishRenderedImage (cv::Mat image) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(rendered_image_msg);
}

// Publish the RGB image of a keyframe
// void Node::PublishKeyFrameRGBImage (const cv::Mat &imRGB) {
void Node::PublishKeyFrameRGBImage (const sensor_msgs::ImageConstPtr& msgRGB) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    //const sensor_msgs::ImagePtr keyframe_rgb_msg = cv_bridge::CvImage(header, "bgr8", imRGB).toImageMsg();
    keyframe_rgb_image_publisher_.publish(msgRGB);
}

void Node::PublishKeyFrameRGBImage (const cv::Mat &imRGB) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_rgb_msg = cv_bridge::CvImage(header, "bgr8", imRGB).toImageMsg();
    keyframe_rgb_image_publisher_.publish(keyframe_rgb_msg);
}

// Publish the depth image of a keyframe
//void Node::PublishKeyFrameDepthImage (const cv::Mat &imDepth) {
void Node::PublishKeyFrameDepthImage (const sensor_msgs::ImageConstPtr& msgD) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    //const sensor_msgs::ImagePtr keyframe_depth_msg = cv_bridge::CvImage(header, "mono16", imDepth).toImageMsg();
    keyframe_depth_image_publisher_.publish(msgD);
}

void Node::PublishKeyFrameDepthImage (const cv::Mat &imDepth) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr keyframe_depth_msg = cv_bridge::CvImage(header, "mono16", imDepth).toImageMsg();
    keyframe_depth_image_publisher_.publish(keyframe_depth_msg);
}

tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
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


sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = map_frame_id_param_;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
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
        if (!map_points.at(i)->isBad()) {
            //            data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
            //            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
            //            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
            data_array[0] = map_points.at(i)->GetWorldPos().at<float> (0); //x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = map_points.at(i)->GetWorldPos().at<float> (1); //y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = map_points.at(i)->GetWorldPos().at<float> (2); //z. Do the transformation by just reading at the position of y instead of z


            //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 3*sizeof(float));
        }
    }

    return cloud;
}

sensor_msgs::PointCloud2 Node::PointsToPointCloud (PointCloud7D::Ptr pointCloud) {
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

void Node::InitParameters () {
    node_handle_.param(name_of_node_+"/publish_pointcloud", publish_pointcloud_param_, true);
    node_handle_.param(name_of_node_+"/localize_only", localize_only_param_, false);
    node_handle_.param(name_of_node_+"/reset_map", reset_map_param_, false);
    node_handle_.param<std::string>(name_of_node_+"/pointcloud_frame_id", map_frame_id_param_, "map");
    node_handle_.param<std::string>(name_of_node_+"/camera_frame_id", camera_frame_id_param_, "camera_link");
    node_handle_.param(name_of_node_+"/min_num_kf_in_map", minimum_num_of_kf_in_map_param_, 5);

    orb_slam_->SetMinimumKeyFrames (minimum_num_of_kf_in_map_param_);
}


void Node::UpdateParameters () {
    node_handle_.param(name_of_node_+"/localize_only", localize_only_param_, false);
    orb_slam_->EnableLocalizationOnly (localize_only_param_);

    node_handle_.param(name_of_node_+"/reset_map", reset_map_param_, false);
    if (reset_map_param_) {
        orb_slam_->Reset();
        node_handle_.setParam (name_of_node_+"/reset_map", false);
    }

    node_handle_.param(name_of_node_+"/min_num_kf_in_map", minimum_num_of_kf_in_map_param_, 5);
    orb_slam_->SetMinimumKeyFrames (minimum_num_of_kf_in_map_param_);
}
