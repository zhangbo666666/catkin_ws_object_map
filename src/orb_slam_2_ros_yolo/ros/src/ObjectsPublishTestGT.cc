#include "ObjectsPublishTestGT.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/StdVector>
#define Random(x) (rand() % x)

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Objects");
    ros::start();
    ros::Rate loop_rate(2);
    ros::NodeHandle node_handle;
    cout << "Start to publish objects ......" << endl;

//    if(argc != 0)
//    {
//        ROS_ERROR ("The input parameters are not right.");
//        ros::shutdown();
//        return 1;
//    }

    string dataset_num = argv[1];
    int objects_num = std::stoi(argv[2]);
//    string object_file_name = argv[2];
//    string objects_cloud_name = argv[3];

    std::vector<Cluster> clusters;
    ObjectsPublish objects(node_handle);

    // ros::spin();

    // obtain objects
    std::string obbox_file_path = "/home/wl/projects/3d_segmentation_evaluation/data/" + dataset_num + "/ground_truth/" + dataset_num + "_obbox.yml";

    cv::FileStorage fsRead(obbox_file_path, cv::FileStorage::READ);
    if(!fsRead.isOpened())
    {
        cerr << "Failed to open settings file " << endl;
        exit(-1);
    }



    for (int i = 1; i <= objects_num; i++)
    {
        obbox obbox_tmp;
        std::string object_name;
        object_name = "object_" + std::to_string(i);
        cout <<  "object_name: " << object_name << endl;
        ///01
        cv::FileNode struct_node = fsRead[object_name];
        obbox_tmp.label = (std::string)struct_node["label"];
        obbox_tmp.x = struct_node["x"];
        obbox_tmp.y = (float)struct_node["y"];
        obbox_tmp.z = (float)struct_node["z"];
        obbox_tmp.l = (float)struct_node["l"];
        obbox_tmp.w = (float)struct_node["w"];
        obbox_tmp.h= (float)struct_node["h"];
        obbox_tmp.q1 = (float)struct_node["q1"];
        obbox_tmp.q2 = (float)struct_node["q2"];
        obbox_tmp.q3 = (float)struct_node["q3"];
        obbox_tmp.q4 = (float)struct_node["q4"];

        cout << "obbox_tmp.label: " << obbox_tmp.label << endl;
        cout << "obbox_tmp.x: " << obbox_tmp.x << endl;
        cout << "obbox_tmp.y: " << obbox_tmp.y << endl;
        cout << "obbox_tmp.z: " << obbox_tmp.z << endl;
        cout << "obbox_tmp.l: " << obbox_tmp.l << endl;
        cout << "obbox_tmp.w: " << obbox_tmp.w << endl;
        cout << "obbox_tmp.h: " << obbox_tmp.h << endl;
        cout << "obbox_tmp.q1: " << obbox_tmp.q1 << endl;
        cout << "obbox_tmp.q2: " << obbox_tmp.q2 << endl;
        cout << "obbox_tmp.q3: " << obbox_tmp.q3 << endl;
        cout << "obbox_tmp.q4: " << obbox_tmp.q4 << endl;

        Cluster cluster_tmp;
        Eigen::Vector3d size_tmp;
//        size_tmp(0) = obbox_tmp.l;
//        size_tmp(1) = obbox_tmp.w;
//        size_tmp(2) = obbox_tmp.h;

        size_tmp << obbox_tmp.l, obbox_tmp.w, obbox_tmp.h;

        cluster_tmp.size = size_tmp;

        cluster_tmp.object_name = obbox_tmp.label;

        Eigen::Vector3d boxCenter_tmp;
//        boxCenter_tmp(0) = obbox_tmp.x;
//        boxCenter_tmp(1) = obbox_tmp.y;
//        boxCenter_tmp(2) = obbox_tmp.z;

        boxCenter_tmp << obbox_tmp.x, obbox_tmp.y, obbox_tmp.z;


        Eigen::Quaterniond q;
        q.x() = obbox_tmp.q1;
        q.y() = obbox_tmp.q2;
        q.z() = obbox_tmp.q3;
        q.w() = obbox_tmp.q4;

//        q.x() = obbox_tmp.q2;
//        q.y() = obbox_tmp.q3;
//        q.z() = obbox_tmp.q4;
//        q.w() = obbox_tmp.q1;


        cout << "q=\n"<<q.coeffs()<<endl<<endl;//coeffs的顺�?:(x,y,z,w)
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R = Eigen::Matrix3d(q);
        cout<<"R=\n"<<R<<endl<<endl;

        Eigen::Affine3d coordinate_system_t;
//        Eigen::Affine3d coordinate_system_t_label;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                coordinate_system_t(i, j) = R(i, j);
            }
            coordinate_system_t(i, 3) = boxCenter_tmp(i);
        }

        for (int i = 0; i < 3; i++)
        {
            coordinate_system_t(3,i) = 0;
        }
        coordinate_system_t(3,3) = 1;
//        cluster_tmp.coordinate_system_t = coordinate_system_t;
        cluster_tmp.coordinate_system_t = coordinate_system_t.inverse();

        cout << "cluster_tmp.coordinate_system_t: " << endl << cluster_tmp.coordinate_system_t.matrix() << endl;
        clusters.push_back(cluster_tmp);
    }

    fsRead.release();

//    char temp[500] = {0};
//
//    while(file.getline(temp, sizeof(temp)))
//    {
//        Cluster cluster;
//        Eigen::Vector3d size;
//        string data_;
//        vector<string> data;
//
//        stringstream word(temp);//采用字符流格式将读取的str进行空格分隔，并放入str word�?
//
//        cout << "temp: " << temp << endl;
//
//        for(int i = 0; i < 39; i++)
//        {
//            word >> data_;
//            data.push_back(data_);
//        }
//        cluster.object_name = data[0];
//
//        size(0) = atof(data[2].c_str());
//        size(1) = atof(data[3].c_str());
//        size(2) = atof(data[4].c_str());
//        cluster.size = size;
//
//        Eigen::Affine3d coordinate_system_t;
//        Eigen::Affine3d coordinate_system_t_label;
//
//        for(int k = 6; k < 22; k++)
//        {
//            coordinate_system_t((k-6)/4, (k-6)%4) = atof(data[k].c_str());
//        }
//        cluster.coordinate_system_t = coordinate_system_t;
//
//        for(int k = 23; k < 39; k++)
//        {
//            coordinate_system_t_label((k-23)/4, (k-23)%4) = atof(data[k].c_str());
//        }
//        cluster.coordinate_system_t_label = coordinate_system_t_label;
//
//        clusters.push_back(cluster);
//
//        cout << "cluster.object_name =\n" << cluster.object_name << endl;
//        cout << "cluster.size =\n" << cluster.size << endl;
//        cout << "cluster.coordinate_system_t = \n" << cluster.coordinate_system_t.matrix() << endl;
//        cout << "cluster.coordinate_system_t_label = \n" << cluster.coordinate_system_t_label.matrix() << endl;
//    }
//    file.close();

    PointCloud6D::Ptr cloud(new PointCloud6D);
    PointCloud6D::Ptr objects_cloud(new PointCloud6D);

    //读取点云文件
    cloud->points.clear();
    string map_path = "/home/wl/projects/3d_segmentation_evaluation/data/" + dataset_num + "/ground_truth/" + dataset_num + ".pcd";
    cout << "map_path = " << map_path << endl;
    pcl::io::loadPCDFile(map_path, *cloud);

//    int v1(0);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewr"));
//    cloud_viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//    cloud_viewer->setBackgroundColor(255,255,255);
//    cloud_viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "ground_truth",v1);
//    cloud_viewer->spin();


//    objects_cloud->points.clear();
//    string objects_cloud_path = "./" + dataset_num + "/" + objects_cloud_name;
//    cout << "object_cloud_path = " << objects_cloud_path << endl;
//    pcl::io::loadPCDFile(objects_cloud_path, *objects_cloud);

    while(ros::ok())
    {
        ros::spinOnce();

        // Publish map point cloud
        objects.PublishMapPointCloud(cloud);

        // Publish objects point cloud
//        objects.PublishObjectsPointCloud(objects_cloud);

        // Draw bounding boxes of objects, publish to rviz
        objects.PublishObjectsBoundingBoxes(clusters);

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

ObjectsPublish::ObjectsPublish (ros::NodeHandle &node_handle)
{
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_markers"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    // ROS_INFO("Sleeping 5 seconds before running demo");
    ros::Duration(5.0).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    map_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/map_point_cloud", 10, this);
    objects_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/objects_point_cloud", 10, this);

    // Publish the point clouds of objects
    //objects_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/objects_point_cloud", 1, this);
    //single_object_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/single_object_point_cloud", 1, this);

}

ObjectsPublish::~ObjectsPublish ()
{

}

// 使用C++11新特性，实现用字符串作为switch的case子句
//----------------------
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
//----------------------

void ObjectsPublish::PublishObjectsBoundingBoxes(std::vector<Cluster> clusters)
{
    int objnumber = clusters.size();

    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();

    std::cout << "Object Database size: " << objnumber << std::endl;

    rviz_visual_tools::colors object_color;

    for( int m=0; m<objnumber; m++)
    {
        Cluster & cluster = clusters[m];
        Eigen::Vector3d size  = cluster.size;
        //Eigen::Vector3d boxCenter  = cluster.boxCenter;

        std::cout << "Object: " << cluster.object_name << " size: "
                  << size[0] << " " << size[1] << " " << size[2] << " "
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
    }

    // Don't forget to trigger the publisher!
    visual_tools_->trigger();
}

void ObjectsPublish::PublishLabel(const Eigen::Affine3d& pose, const std::string& label)
{
    Eigen::Affine3d pose_copy = pose;
    //pose_copy.translation().x() -= 0.1;
    visual_tools_->publishText(pose_copy, label, rviz_visual_tools::BLACK, rviz_visual_tools::XXXLARGE, false);
}

sensor_msgs::PointCloud2 ObjectsPublish::PointsToPointCloud (PointCloud6D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
    }
    Point6D tmp;
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

        data_array[3] = (float)((int)tmp.r / 255.0);
        data_array[4] = (float)((int)tmp.g / 255.0);
        data_array[5] = (float)((int)tmp.b / 255.0);

        memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 6*sizeof(float));
    }

    return cloud;
}

// Publish an object point cloud with the sequence number
sensor_msgs::PointCloud2 ObjectsPublish::ObjectsPointsToPointCloud (PointCloud6D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
    }
    Point6D tmp;
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

// Publish map point clouds for display in rviz
void ObjectsPublish::PublishMapPointCloud(PointCloud6D::Ptr mapCloud)
{
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(mapCloud);
    map_point_cloud_publisher_.publish(cloud);
}

void ObjectsPublish::PublishObjectsPointCloud(PointCloud6D::Ptr objectsCloud)
{
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(objectsCloud);
    objects_point_cloud_publisher_.publish(cloud);
}
