#include <ros/ros.h> 
#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 
#include <nav_msgs/Odometry.h>

ros::Publisher path_pub;
nav_msgs::Path path;

// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher path_pub)
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

    this_pose_stamped.pose.orientation = msg->pose.pose.orientation;
    std::cout << "x = " << this_pose_stamped.pose.position.x << std::endl;
    std::cout << "y = " << this_pose_stamped.pose.position.y << std::endl;

    this_pose_stamped.header.stamp = current_time;

    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    path.header.stamp = current_time;
    path.header.frame_id = "odom";
    path_pub.publish(path);
}


int main (int argc, char **argv) 
{ 
    ros::init (argc, argv, "showpath");
    ros::NodeHandle ph;
    path_pub = ph.advertise<nav_msgs::Path>("trajectory", 10, true);
    //ros::Subscriber odom_sub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, boost::bind(&odomCallback, _1, path_pub));
    ros::Subscriber odom_sub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, &odomCallback);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
