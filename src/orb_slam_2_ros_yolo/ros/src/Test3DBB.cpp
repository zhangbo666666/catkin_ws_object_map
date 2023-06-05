#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>

// include files
#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

using namespace std;

typedef pcl::PointXYZRGBA Point7D;// 点类型 xyzrgba 点+颜色
typedef pcl::PointCloud<Point7D> PointCloud7D;// 点云类型

//ros::Publisher path_pub;
//nav_msgs::Path path;


// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher path_pub)
//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//    ros::Time current_time = ros::Time::now();
//    geometry_msgs::PoseStamped this_pose_stamped;
//    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
//    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

//    this_pose_stamped.pose.orientation = msg->pose.pose.orientation;
//    std::cout << "x = " << this_pose_stamped.pose.position.x << std::endl;
//    std::cout << "y = " << this_pose_stamped.pose.position.y << std::endl;

//    this_pose_stamped.header.stamp = current_time;

//    this_pose_stamped.header.frame_id="odom";
//    path.poses.push_back(this_pose_stamped);
//    path.header.stamp = current_time;
//    path.header.frame_id = "odom";
//    path_pub.publish(path);
//}

std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters);

int main (int argc, char **argv)
{ 
    //    ros::init (argc, argv, "test3DBB");
    //    ros::NodeHandle ph;
    //    path_pub = ph.advertise<nav_msgs::Path>("trajectory", 10, true);
    //    //ros::Subscriber odom_sub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, boost::bind(&odomCallback, _1, path_pub));
    //    ros::Subscriber odom_sub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, &odomCallback);

    //    ros::Rate loop_rate(50);
    //    while (ros::ok())
    //    {
    //        ros::spinOnce();
    //        loop_rate.sleep();
    //    }

    PointCloud7D::Ptr cloud(new PointCloud7D);

    //读取点云文件
    cloud->points.clear();
    pcl::io::loadPCDFile("/home/zb/catkin_ws_object_map/src/backup/orb_slam_2_ros_yolo_0512/experiments/results/object_pcd/object_5.pcd", *cloud);
    cout << "Load the pcd file successfully" << endl;

    std::vector<autosense::ObjectPtr> objects;
    autosense::ObjectPtr object;
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
    objects = Obtain3DBB(cloud_clusters);

    for(int i = 0; i < objects.size(); i++)
    {
        object = objects[i];
        cout << "------------------------------" << endl;
        cout << "The " << i << "-th object:" << endl;
        cout << "direction = \n" << object->direction << endl;
        cout << "l, w, h = " << object->length << " "
                             << object->width << " "
                             << object->height << endl;
        cout << "------------------------------" << endl;
    }

    cout << "Calculate the 3DBB" << endl;

    return 0;
}


std::vector<autosense::ObjectPtr> Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters)
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

