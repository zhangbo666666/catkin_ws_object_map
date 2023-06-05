#include "ObjectsGraphExtraction.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/filters/extract_indices.h>

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

        stringstream word(temp);//采用字符流格式将读取的str进行空格分隔，并放入str word中

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

    cout<<"*****start to get wall***"<<endl;
    objects.GetWallObject(cloud, clusters);


    objects_cloud->points.clear();
    string objects_cloud_path = dataset_path + dataset_num + "/" + objects_cloud_name;
    cout << "object_cloud_path = " << objects_cloud_path << endl;
    pcl::io::loadPCDFile(objects_cloud_path, *objects_cloud);

    while(ros::ok())
    {
        ros::spinOnce();

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
        case static_cast<int>(hash_compile_time("table")):
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

//for global map to extract wall
void ObjectsPublish::GetWallObject(PointCloud7D::Ptr mapCloud, std::vector<Cluster>& clusters){

    PointCloud7D::Ptr wall_cloud(new PointCloud7D);
    PointCloud7D::Ptr wall_cloud_temp(new PointCloud7D);

    std::cout << "mapCloud has: " << mapCloud->points.size () << "  " << endl;
    std::cout << "clusters has: " << clusters.size () << "  " << endl;

    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(mapCloud);
    float voxelResolution = 0.05;
    if(mapCloud->points.size() > 100000)
    {
        vg.setLeafSize(voxelResolution, voxelResolution, voxelResolution);// 体素格子 尺寸
        vg.filter(*wall_cloud);
        std::cout << "After voxel filter has: " << wall_cloud->points.size () << "  " << endl;
    }

    if(mapCloud->points.size () > 0)
    {
        //直通滤波器，在Z轴方向上剪除多余的点
        pcl::PassThrough<pcl::PointXYZRGBA> pass;   //创建滤波器对象
        pass.setInputCloud (mapCloud);     //设置待滤波的点云
        pass.setFilterFieldName ("z");             //设置在Z轴方向上进行滤波
        pass.setFilterLimits (-0.05, 0.2);           //设置滤波范围为0.2~6.0,在范围之外的点会被剪除
        pass.setFilterLimitsNegative(true);
        pass.filter (*wall_cloud);           //存储
        std::cout << "After pass through has: " << wall_cloud->points.size () << "  " << endl;
    }

    std::cout << "After PassThrough, wall cloud size : " << wall_cloud->points.size () << std::endl;

    if(wall_cloud->points.size () > 5000)
    {
        //统计滤波器，删除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> Static;   //创建滤波器对象
        Static.setInputCloud (wall_cloud);               //设置待滤波的点云
        Static.setMeanK (100);                               //设置在进行统计时考虑查询点临近点数
        Static.setStddevMulThresh (1.5);                      //设置判断是否为离群点的阀值, The distance threshold will be equal to: mean + stddev_mult * stddev.
        Static.filter (*wall_cloud);                    //存储
        // std::cout << "Object PointCloud after statistic filtering has: " << object_cloud_tmp->points.size ()  << " data points." << std::endl;
        std::cout << "After outlier filtering: " << wall_cloud->points.size () << std::endl;
    }

    pointCloudEuclidean(wall_cloud, wall_cloud_temp);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>),cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDWriter writer;

    //采样一致性分割算法的目的主要是从原点云中提取目标模型，比如说面，球体，圆柱等等，从而为后续的目标识别或者点云匹配等等做准备
    //segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    //索引
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);

    // Create the filtering object,按点云索引提取点云子集
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    int i = 0, nr_points = (int) wall_cloud_temp->points.size ();
    // While 30% of the original cloud is still there
    while (wall_cloud_temp->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (wall_cloud_temp);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (wall_cloud_temp);
        extract.setIndices (inliers);
        extract.setNegative (false);//如果设为true,可以提取指定inliers之外的点云
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;

        //attention save path
        ss << "/home/gxc/dataset/SceneNN/000/plane_" << i << ".pcd";
        writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_p, false);

        Cluster cluster;
        cluster.object_name = "wall"; // 名字
        Calculate3DBB(cloud_p, cluster);
        clusters.push_back(cluster);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        wall_cloud_temp.swap (cloud_f);
        i++;
    }
    std::cout << "After extracting wall, clusters has: " << clusters.size () << "  " << endl;
     //attention save path,edit by dx
    WriteDatabase2File("/home/gxc/dataset/SceneNN/000/objects_with_wall.txt", clusters);


/*///Projected to x-y,and compute the convex hull
    // 我们使用一个平面模型ax + by + cz + d = 0，其中a,b,c,d分别对应下面四个通道，对值为1的维度进行降维打击，或者换句话说，假如a = b = d = 0，c = 1，则投影到XY平面
    //或者可以这么理解，画个坐标系把上面方程画出来，假如a = b =1，c = d=0，方程为x=-y,意味着投影在x=-y的平面上。
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>());
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//    coefficients->values.resize(4);
//    coefficients->values[0] = 0;
//    coefficients->values[1] = 0;
//    coefficients->values[2] = 1;
//    coefficients->values[3] = 0;
//
//
//    // Create the filtering object
//    pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
//    proj.setModelType(pcl::SACMODEL_PLANE);   //创建投影类型，投影到平面上
//    proj.setInputCloud(wall_cloud_temp);
//    proj.setModelCoefficients(coefficients);
//    proj.filter(*cloud_projected);

//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>());
////省略ground的io操作，向ground读入数据
//    vector<int> inliers;//用于存放合群点的vector
//    pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr model(
//            new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>(wall_cloud_temp));//定义待拟合平面的model，并使用待拟合点云初始化
//    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model);//定义RANSAC算法模型
//    ransac.setDistanceThreshold(0.05);//设定阈值
//    ransac.computeModel();//拟合
//    ransac.getInliers(inliers);//获取合群点
//    vector<int> tmp;
//    Eigen::VectorXf coeff;
//    ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
//    cout<<"coeff "<<coeff[0]<<" "<<coeff[1]<<" "<<coeff[2]<<" "<<coeff[3]<<endl;
//    ransac.getModel(tmp);
//    pcl::copyPointCloud<pcl::PointXYZRGBA>(*wall_cloud_temp, inliers, *final);


//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::ConvexHull<pcl::PointXYZRGBA> chull;
//    chull.setInputCloud (cloud_projected);
//    chull.reconstruct (*cloud_hull);
//
//    std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

//    pcl::PCDWriter writer;
//    writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);


//    int v1(0), v2(0);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewr"));
//
//    cloud_viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//    cloud_viewer->setBackgroundColor(255,255,255);
//    cloud_viewer->addPointCloud<pcl::PointXYZRGBA>(wall_cloud, "cloud_hull",v1);
//    cloud_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//    cloud_viewer->setBackgroundColor(255,255,255);
//    cloud_viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_filtered, "wall_cloud_temp_projected", v2);

//    cloud_viewer->spin();

*/
}

void ObjectsPublish::pointCloudEuclidean(PointCloud7D::Ptr cloud,
                                               PointCloud7D::Ptr& objectCloud)
{
    pcl::search::KdTree<Point7D>::Ptr kdtree(new pcl::search::KdTree<Point7D>);
    kdtree->setInputCloud(cloud);

    // Euclidean 聚类对象.
    pcl::EuclideanClusterExtraction<Point7D> clustering;
    // 设置聚类的最小值 2cm (small values may cause objects to be divided
    // in several clusters, whereas big values may join objects in a same cluster).
    clustering.setClusterTolerance(0.05);
    // 设置聚类的小点数和最大点云数
    clustering.setMinClusterSize(800);
    clustering.setMaxClusterSize(110000);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr maxCluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // For every cluster...
    int currentClusterNum = 1;
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        //添加所有的点云到一个新的点云中
        pcl::PointCloud<Point7D>::Ptr cluster(new pcl::PointCloud<Point7D>);
        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(cloud->points[*point]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        if (currentClusterNum == 1){
            for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            {
                maxCluster->points.push_back(cloud->points[*point]);
                objectCloud->points.push_back(cloud->points[*point]);
            }

            objectCloud->width = cluster->points.size();
            objectCloud->height = 1;
            objectCloud->is_dense = true;

            maxCluster->width = cluster->points.size();
            maxCluster->height = 1;
            maxCluster->is_dense = true;
        }
        // 保存
        if (cluster->points.size() <= 0)
            break;
//        std::cout << "Step2.1.3: Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
//        std::string fileName = "/home/gxc/dataset/SceneNN/m_result/LCCP/" + boost::to_string(currentClusterNum) + ".pcd";
//        pcl::io::savePCDFileASCII(fileName, *cluster);
//
        currentClusterNum++;
    }
    std::cout << "After  pointCloudEuclidean  object has " << objectCloud->points.size() << " points." << std::endl;
}


//Calculate3DBB
void ObjectsPublish::Calculate3DBB(PointCloud7D::Ptr cloud, Cluster& cluster)
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

    Object3d object3d;
    boundingBoxEstimation(singleObject, object3d);

    cluster.size        = object3d.sizePt; // 尺寸
    cluster.boxCenter   = object3d.boxCenter; // 中心点  // max_it->boxCenter; // 包围盒中心点
    cluster.minPt       = object3d.minPt;
    cluster.maxPt       = object3d.maxPt;
    cluster.coordinate_system_t = object3d.coordinate_system_t;
    cluster.coordinate_system_t_label = object3d.coordinate_system_t_label;
//    cluster.eightCorners = object3d.eightCorners;

//    for(int i = 0; i < 10; i++)
//    {
//        cluster.bb3d[i] = object3d.bb3d[i];
//    }
//    Objects3DBB temp;
//    autosense::ObjectPtr singleObject;
//    mpObjects3DBB->CalculateSingleObject3DBB(cloud, singleObject);

//    cout << "Calculate3DBB: cluster.object_update_num = " << cluster.object_update_num << endl;
}

std::vector<autosense::ObjectPtr> ObjectsPublish::Obtain3DBB(std::vector<autosense::PointICloudPtr> cloud_clusters)
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

void ObjectsPublish::boundingBoxEstimation(autosense::ObjectPtr singleObject, Object3d& object3d)
{
    // 3d边框
    double l, w, h;
    l = singleObject->length;
    w = singleObject->width;
    h = singleObject->height;
    object3d.sizePt = Eigen::Vector3d(l, // x
                                      w, // y
                                      h); // z

    object3d.minPt = Eigen::Vector3d(-l/2.0, -w/2.0, -h/2.0);
    object3d.maxPt = Eigen::Vector3d(l/2.0, w/2.0, h/2.0);

    // 3d边框中心, in the world coordinate system
    object3d.boxCenter = Eigen::Vector3d(singleObject->ground_center[0], // ground center of the object (cx, cy, z_min)
                                         singleObject->ground_center[1],
                                         (singleObject->ground_center[2] + singleObject->height / 2.0));


    object3d.translation << object3d.boxCenter[0], object3d.boxCenter[1], object3d.boxCenter[2];

    // oriented boundingbox information: main direction vector(x, y, 0)
    // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
    Eigen::Vector3d direction;
    direction = singleObject->direction;
    double theta = 0;
    if(direction[0] != 0)
    {
        theta = atan(direction[1] / direction[0]);
    }
    else
    {
        theta = 3.1415 / 2;
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform_label = Eigen::Matrix4d::Identity();

    transform(0, 0) = cos(theta);
    transform(0, 1) = -sin(theta);
    transform(1, 0) = sin(theta);
    transform(1, 1) = cos(theta);
    transform(0, 3) = object3d.boxCenter[0];
    transform(1, 3) = object3d.boxCenter[1];
    transform(2, 3) = object3d.boxCenter[2];

    Eigen::Affine3d a3f_transform(transform.cast<double>());
//    cout << "a3f_transform = \n" << a3f_transform.matrix() << endl;
    object3d.coordinate_system_t = a3f_transform;

    Eigen::MatrixXd boundingBoxCorners(4, 8);
    Eigen::MatrixXd boundingBoxCorners_sys(4, 8);

    // Eight corners sequence: 1-2-3-4, 5-6-7-8
    //    5-------6
    //   /|      /|
    //  8-|-----7 |
    //  | 1-----| 2
    //  |/      |/
    //  4-------3
    boundingBoxCorners <<  object3d.minPt[0], object3d.minPt[0], object3d.maxPt[0], object3d.maxPt[0], object3d.minPt[0], object3d.minPt[0], object3d.maxPt[0], object3d.maxPt[0],
            object3d.minPt[1], object3d.maxPt[1], object3d.maxPt[1], object3d.minPt[1], object3d.minPt[1], object3d.maxPt[1], object3d.maxPt[1], object3d.minPt[1],
            object3d.minPt[2], object3d.minPt[2], object3d.minPt[2], object3d.minPt[2], object3d.maxPt[2], object3d.maxPt[2], object3d.maxPt[2], object3d.maxPt[2],
            1.0,               1.0,               1.0,               1.0,               1.0,               1.0,               1.0,               1.0;

    cout << "transform =\n" << transform << endl;

    cout << "boundingBoxCorners =\n" << boundingBoxCorners << endl;
    boundingBoxCorners_sys = transform * boundingBoxCorners;
    cout << "boundingBoxCorners_sys =\n" << boundingBoxCorners_sys << endl;

    //------------------------
    //    Eigen::Vector4d vec_min, vec_max;
    //    vec_min = boundingBoxCorners_sys.rowwise().minCoeff();
    //    vec_max = boundingBoxCorners_sys.rowwise().maxCoeff();

    //    // save bounding box four corners, maximal height and minimal height
    //    pcl::PointXYZ _corner;
    //    _corner.x = 0.0;
    //    _corner.y = 0.0;
    //    _corner.z = 0.0;
    //    object3d.fourCorners.clear();// clear the data
    //    for(int p = 0; p < 4; p++){
    //        _corner.x = boundingBoxCorners_sys(0, p);
    //        _corner.y = boundingBoxCorners_sys(1, p);
    //        _corner.z = boundingBoxCorners_sys(2, p);
    //        object3d.fourCorners.push_back(_corner);
    //    }

    //    object3d.H_min = vec_min[2];
    //    object3d.H_max = vec_max[2];
    //------------------------

    // save bounding box eight corners
    pcl::PointXYZ _corner;
    _corner.x = 0.0;
    _corner.y = 0.0;
    _corner.z = 0.0;
    object3d.eightCorners.clear();// clear the data
    for(int p = 0; p < 8; p++){
        _corner.x = boundingBoxCorners_sys(0, p);
        _corner.y = boundingBoxCorners_sys(1, p);
        _corner.z = boundingBoxCorners_sys(2, p);
        object3d.eightCorners.push_back(_corner);
    }

    // object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
    double bb3d[10] = {0.0};
    for(int i = 0; i < 4; i++)
    {
        bb3d[i * 2 + 1] = object3d.eightCorners[i].y;
    }

    // when eight corners are converted to system, the sequence may be 1-2-3-4 or 1-2-6-5
    //    5-------6
    //   /|      /|
    //  8-|-----7 |
    //  | 1-----|-2
    //  |/      |/
    //  4-------3
    //    4-------3
    //   /|      /|
    //  8-|-----7 |
    //  | 1-----|-2
    //  |/      |/
    //  5-------6
    double temp1 = abs(object3d.eightCorners[0].x - object3d.eightCorners[2].x);
    double temp2 = abs(object3d.eightCorners[0].x - object3d.eightCorners[4].x);

    if(temp1 >= temp2)
    {
        bb3d[0] = (object3d.eightCorners[0].x + object3d.eightCorners[4].x) / 2;
//        bb3d[2] = bb3d[0];
        bb3d[2] = (object3d.eightCorners[1].x + object3d.eightCorners[5].x) / 2;
        bb3d[4] = (object3d.eightCorners[2].x + object3d.eightCorners[6].x) / 2;
//        bb3d[6] = bb3d[4];
        bb3d[6] = (object3d.eightCorners[3].x + object3d.eightCorners[7].x) / 2;

    }
    if(temp1 < temp2)
    {
        bb3d[0] = (object3d.eightCorners[0].x + object3d.eightCorners[2].x) / 2;
//        bb3d[2] = bb3d[0];
        bb3d[2] = (object3d.eightCorners[1].x + object3d.eightCorners[3].x) / 2;
        bb3d[4] = (object3d.eightCorners[4].x + object3d.eightCorners[6].x) / 2;
//        bb3d[6] = bb3d[4];
        bb3d[2] = (object3d.eightCorners[5].x + object3d.eightCorners[7].x) / 2;
    }

    double minH = 100.0;
    double maxH = -100.0;
    for(int i = 0; i < 8; i++)
    {
        if(object3d.eightCorners[i].z <= minH)
            minH = object3d.eightCorners[i].z;
        if(object3d.eightCorners[i].z >= maxH)
            maxH = object3d.eightCorners[i].z;
    }

    bb3d[8] = minH;
    bb3d[9] = maxH;

    for(int i = 0; i < 10; i++)
    {
        object3d.bb3d[i] = bb3d[i];
        cout << "bb3d  " << i << "  " << bb3d[i] << endl;
    }

    // label position
    Eigen::Vector4d bb_label_sys(object3d.eightCorners[1].x, object3d.eightCorners[1].y, object3d.eightCorners[1].z + 0.1, 1);
    transform_label = transform;
    // Convert to world coordinate system, label position
    transform_label(0, 3) = bb_label_sys(0);
    transform_label(1, 3) = bb_label_sys(1);
    transform_label(2, 3) = bb_label_sys(2);

    Eigen::Affine3d a3f_transform_label(transform_label.cast<double>());
    object3d.coordinate_system_t_label = a3f_transform_label;
}


void ObjectsPublish::WriteDatabase2File(string file_path, std::vector<Cluster> clusters)
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

