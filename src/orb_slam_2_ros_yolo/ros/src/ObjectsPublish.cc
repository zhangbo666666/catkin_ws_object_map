#include "ObjectsPublish.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#define Random(x) (rand() % x)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Objects");
    ros::start();
    ros::Rate loop_rate(20);
    ros::NodeHandle node_handle;
    cout << "Start to publish objects ......" << endl;

    string dataset_path = "/home/gxc/dataset/SceneNN/";

    if(argc != 4)
    {
        ROS_ERROR ("The input parameters are not right.");
        ros::shutdown();
        return 1;
    }

    string dataset_num = argv[1];
    string object_file_name = argv[2];
    string objects_cloud_name = argv[3];

    std::vector<Cluster> clusters;

    ObjectsPublish objects(node_handle);

    // ros::spin();

    // obtain objects
    ifstream file(dataset_path + dataset_num + "/" + object_file_name, std::ios::in);
    char temp[500] = {0};

    while(file.getline(temp, sizeof(temp)))
    {
        Cluster cluster;
        Eigen::Vector3d size;
        Eigen::Vector3d boxCenter;
        string data_;
        vector<string> data;

        stringstream word(temp);//采用字符流格式将读取的str进行空格分隔，并放入str word�?

        for(int i = 0; i < 43; i++) // the number of words in each row is 43
        {
            word >> data_;
            data.push_back(data_);
        }
        cluster.object_name = data[0];

        size(0) = atof(data[2].c_str());
        size(1) = atof(data[3].c_str());
        size(2) = atof(data[4].c_str());
        cluster.size = size;

        boxCenter(0) = atof(data[6].c_str());
        boxCenter(1) = atof(data[7].c_str());
        boxCenter(2) = atof(data[8].c_str());
        cluster.boxCenter = boxCenter;

        Eigen::Affine3d coordinate_system_t;
        Eigen::Affine3d coordinate_system_t_label;

        for(int k = 10; k < 26; k++)
        {
            coordinate_system_t((k-10)/4, (k-10)%4) = atof(data[k].c_str());
        }
        cluster.coordinate_system_t = coordinate_system_t;

        for(int k = 27; k < 43; k++)
        {
            coordinate_system_t_label((k-27)/4, (k-27)%4) = atof(data[k].c_str());
        }
        cluster.coordinate_system_t_label = coordinate_system_t_label;

        clusters.push_back(cluster);

        cout << "cluster.object_name =\n" << cluster.object_name << endl;
        cout << "cluster.size =\n" << cluster.size << endl;
        cout << "cluster.boxCenter =\n" << cluster.boxCenter << endl;
        cout << "cluster.coordinate_system_t = \n" << cluster.coordinate_system_t.matrix() << endl;
        cout << "cluster.coordinate_system_t_label = \n" << cluster.coordinate_system_t_label.matrix() << endl;
    }
    file.close();

    PointCloud7D::Ptr cloud(new PointCloud7D);
    PointCloud7D::Ptr objects_cloud(new PointCloud7D);

    //读取点云文件
    cloud->points.clear();
    string map_path = dataset_path + dataset_num + "/map.pcd";
    cout << "map_path = " << map_path << endl;
    pcl::io::loadPCDFile(map_path, *cloud);

    int points_num = static_cast<int>(cloud->points.size());
    PointCloud7D::iterator index = cloud->begin();

    // delete the points beyond the scope limit in the point cloud
    for(int i = 0; i < points_num; i++)
    {
        if(cloud->points[i].z > 1.3)
        {
            cloud->erase(index + i);
        }
    }

    objects_cloud->points.clear();
    string objects_cloud_path = dataset_path + dataset_num + "/" + objects_cloud_name;
    cout << "object_cloud_path = " << objects_cloud_path << endl;
    pcl::io::loadPCDFile(objects_cloud_path, *objects_cloud);
    cout << "object_cloud completed" << endl;
    while(ros::ok())
    {
        cout << "Spin start" << endl;
        ros::spinOnce();
        cout << "Spin completed" << endl;
        // Publish map point cloud
        objects.PublishMapPointCloud(cloud);

        // Publish objects point cloud
        objects.PublishObjectsPointCloud(objects_cloud);

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

    map_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/map_point_cloud", 10,
                                                                                 static_cast<bool>(this));
    objects_point_cloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/orb_slam2_rgbd/objects_point_cloud", 10,
                                                                                     static_cast<bool>(this));

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
    int objnumber = static_cast<int>(clusters.size());

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
        case static_cast<int>(hash_compile_time("tvmonitor")):
            object_color = rviz_visual_tools::LIME_GREEN;
            break;
        case static_cast<int>(hash_compile_time("bottle")):
            object_color = rviz_visual_tools::CYAN;
            break;
        case static_cast<int>(hash_compile_time("couch")):
            object_color = rviz_visual_tools::GREEN;
            break;
        case static_cast<int>(hash_compile_time("chair")):
            object_color = rviz_visual_tools::RED;
            break;
        case static_cast<int>(hash_compile_time("laptop")):
            object_color = rviz_visual_tools::YELLOW;
            break;
        case static_cast<int>(hash_compile_time("refrigerator")):
            object_color = rviz_visual_tools::DARK_GREY;
            break;
        case static_cast<int>(hash_compile_time("bag")):
            object_color = rviz_visual_tools::PURPLE;
            break;
        case static_cast<int>(hash_compile_time("keyboard")):
            object_color = rviz_visual_tools::ORANGE;
            break;
        case static_cast<int>(hash_compile_time("book")):
            object_color = rviz_visual_tools::PINK;
            break;
        case static_cast<int>(hash_compile_time("bed")):
            object_color = rviz_visual_tools::MAGENTA;
            break;
        case static_cast<int>(hash_compile_time("dining table")):
            object_color = rviz_visual_tools::BLUE;
            break;
        case static_cast<int>(hash_compile_time("toilet")):
            object_color = rviz_visual_tools::GREY;
            break;
        default:
            object_color = rviz_visual_tools::RED;
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

sensor_msgs::PointCloud2 ObjectsPublish::PointsToPointCloud (PointCloud7D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
    }
    Point7D tmp;
    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 6; // x y z r g b

    cloud.header.stamp = ros::Time::now();
//    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = static_cast<unsigned int>(pointCloud->points.size());
    cloud.is_bigendian = static_cast<unsigned char>(false);
    cloud.is_dense = static_cast<unsigned char>(true);
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
sensor_msgs::PointCloud2 ObjectsPublish::ObjectsPointsToPointCloud (PointCloud7D::Ptr pointCloud)
{
    if (pointCloud->points.size() == 0) {
        std::cout << "point cloud vector is empty!" << std::endl;
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
    cloud.width = static_cast<unsigned int>(pointCloud->points.size());
    cloud.is_bigendian = static_cast<unsigned char>(false);
    cloud.is_dense = static_cast<unsigned char>(true);
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
void ObjectsPublish::PublishMapPointCloud(PointCloud7D::Ptr mapCloud)
{
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(mapCloud);
    map_point_cloud_publisher_.publish(cloud);
}

void ObjectsPublish::PublishObjectsPointCloud(PointCloud7D::Ptr objectsCloud)
{
    sensor_msgs::PointCloud2 cloud = PointsToPointCloud(objectsCloud);
    objects_point_cloud_publisher_.publish(cloud);
}

// test 3DBB
//std::vector<autosense::ObjectPtr> ObjectsPublish::Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters)
//{
//    // 2.define object builder
//    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;

//    // 3.create object builder by manager
//    object_builder_ = autosense::object_builder::createObjectBuilder();

//    // 4.build 3D orientation bounding box for clustering point cloud
//    //std::vector<PointICloudPtr> cloud_clusters;
//    std::vector<autosense::ObjectPtr> objects;
//    object_builder_->build(cloud_clusters, &objects);
//    return objects;
//}

