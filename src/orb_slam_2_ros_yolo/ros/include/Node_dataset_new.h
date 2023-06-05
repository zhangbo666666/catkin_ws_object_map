#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <object_map_msgs/KeyframeTrans.h>

#include <assert.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;

typedef pcl::PointXYZRGBA Point7D;
typedef pcl::PointCloud<Point7D> PointCloud7D;

class NodeDatasetNew
{
  public:
    NodeDatasetNew (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~NodeDatasetNew ();

  protected:
    //void Update ();
//    void Update(bool mbIsKeyframe,
//                const cv::Mat &imRGB,
//                const cv::Mat &imDepth);

    void Update(const cv::Mat & msgRGB,
                const cv::Mat & msgD,
                bool bIsKeyframe, // is keyframe?
                cv::Mat Tcw); // transform of keyframe

    void Update(string datasetNum,
                int updateNum,
                const cv::Mat & msgRGB,
                const cv::Mat & msgD,
                bool bIsKeyframe, // is keyframe?
                cv::Mat Tcw); // transform matrix

    void Update(string datasetNum,
                int updateNum,
                const cv::Mat & msgRGB,
                const cv::Mat & msgD,
                const cv::Mat & msgThermal,
                const cv::Mat & msgTemp,
                bool bIsKeyframe, // is keyframe?
                cv::Mat Tcw); // transform matrix

//    ORB_SLAM2::System* orb_slam_;
//    ORB_SLAM2::Tracking* orb_slam_tracking_;

    ros::Time current_frame_time_;

    double mResolution;
    float mFx, mFy, mCx, mCy;
    float mDepthMapFactor;

    pcl::VoxelGrid<Point7D>  voxel;

  private:
    //void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    void UpdateParameters ();

    // publish the RGB and depth images of keyframes
    void PublishKeyFrameRGBImage(const sensor_msgs::ImageConstPtr& msgRGB);
    void PublishKeyFrameDepthImage(const sensor_msgs::ImageConstPtr& msgD);
    void PublishKeyFrameRGBImage (const cv::Mat &imRGB);
    void PublishKeyFrameDepthImage (const cv::Mat &imDepth);
    void PublishKeyFrameThermalImage (const cv::Mat &imThermal);
    void PublishKeyFrameTempImage (const cv::Mat &imTemp);

//    void PublishKeyFrameRGBImage (const cv::Mat &imRGB);
//    void PublishKeyFrameDepthImage (const cv::Mat &imDepth);

    // Publish the transform of keyframe to map with the time stamp
    void PublishKeyframeTransform2Map(tf::Transform transform);

    void PublishKeyframeTransform (cv::Mat pose); // not to map

    tf::Transform TransformFromMat (cv::Mat position_mat);
    //sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void InitParameters ();

    image_transport::Publisher rendered_image_publisher_;

    image_transport::Publisher keyframe_rgb_image_publisher_;
    image_transport::Publisher keyframe_depth_image_publisher_;
    image_transport::Publisher keyframe_thermal_image_publisher_;
    image_transport::Publisher keyframe_temp_image_publisher_;

    //ros::Publisher map_points_publisher_;
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
    void PublishPointCloud (PointCloud7D::Ptr pointCloud);

    sensor_msgs::PointCloud2 PointsToPointCloud (PointCloud7D::Ptr pointCloud);
    ros::Publisher point_cloud_publisher_;

    ros::Publisher octomap_publisher_;
    ros::Publisher color_octomap_publisher_;

    PointCloud7D::Ptr globalMap;
    PointCloud7D::Ptr generatePointCloud(cv::Mat color, cv::Mat depth, cv::Mat Tcw);

    void SaveOctoMap(PointCloud7D::Ptr cloud, double resolution, string outputFile); // 转换point cloud到octomap
    void SaveColorOctoMap(PointCloud7D::Ptr cloud, double resolution, string outputFile); // 加入色彩信息

    void PublishOctoMap(PointCloud7D::Ptr cloud, double resolution);
    void PublishColorOctoMap(PointCloud7D::Ptr cloud, double resolution);

};
