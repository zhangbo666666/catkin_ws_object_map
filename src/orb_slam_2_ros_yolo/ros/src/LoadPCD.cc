#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

#include <unistd.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

typedef pcl::PointXYZRGB Point6D;// 点类型 xyzrgba 点+颜色
typedef pcl::PointCloud<Point6D> PointCloud6D;// 点云类型
typedef std::pair<float, int> PointsDistance;


int main(int argc, char **argv)
{
    std::string object_point_cloud_name = argv[1];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string object_point_cloud_filename = "/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/experiments/results/object_pcd/" + object_point_cloud_name;

    std::string object_point_cloud_LCCP_name = argv[2];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_point_cloud_LCCP(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string object_point_cloud_LCCP_filename = "/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/experiments/results/object_pcd/" + object_point_cloud_LCCP_name;

    if (-1 == pcl::io::loadPCDFile(object_point_cloud_filename, *object_point_cloud)) {
        cout << "error input!" << endl;
        return -1;
    }
    if (-1 == pcl::io::loadPCDFile(object_point_cloud_LCCP_filename, *object_point_cloud_LCCP)) {
        cout << "error input!" << endl;
        return -1;
    }

    int v1(0), v2(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewr"));

    cloud_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    cloud_viewer->setBackgroundColor(255,255,255);
    cloud_viewer->addPointCloud<pcl::PointXYZRGB>(object_point_cloud, "object_point_cloud",v1);
    cloud_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    cloud_viewer->setBackgroundColor(255,255,255);
    cloud_viewer->addPointCloud<pcl::PointXYZRGB>(object_point_cloud_LCCP, "object_point_cloud_LCCP", v2);

    cloud_viewer->spin();

    return 0;
}



