#include "ObjectsBasedSemanticNavigation.h"
#include <pcl/io/pcd_io.h>

#define Random(x) (rand() % x)

bool flag_find_object = false;

std::vector<Cluster> clusters;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

void SemanticCommandCallback(const std_msgs::Int32::ConstPtr& msg, ros::Publisher& voice_pub, ros::Publisher& nav_goal_pub)
{
    std_msgs::String msg_voice;
    std::stringstream ss;

    ss << "Start to find the object";
    msg_voice.data = ss.str();

    ROS_INFO("%s", msg_voice.data.c_str());
    voice_pub.publish(msg_voice);
    ros::spinOnce();


    int command_num = msg->data;

    //    //tell the action client that we want to spin a thread by default
    //    MoveBaseClient ac("move_base", true);

    //    //wait for the action server to come up
    //    while(!ac.waitForServer(ros::Duration(5.0))){
    //      ROS_INFO("Waiting for the move_base action server to come up");
    //    }

    //    move_base_msgs::MoveBaseGoal goal;

    //    //we'll send a goal to the robot to move 1 meter forward
    //    // goal.target_pose.header.frame_id = "base_link";
    //    goal.target_pose.header.frame_id = "map";
    //    goal.target_pose.header.stamp = ros::Time::now();

    int objnumber = clusters.size();
    std::cout << "Object Database size: " << objnumber << std::endl;

    //bool flag_find_object = false;
    for( int m = 0; m < objnumber; m++)
    {
        Cluster & cluster = clusters[m];
        Eigen::Vector3d boxCenter = cluster.boxCenter;
        Eigen::Vector3d size = cluster.size;
        Eigen::Affine3d object_transform = cluster.coordinate_system_t;
        string object_name = cluster.object_name;

        if(((command_num == 100) && (object_name == "keyboard")) ||
                ((command_num == 101) && (object_name == "chair"))) // chair
        {
            flag_find_object = true;
            std::cout << "Object: " << cluster.object_name << " boxCenter: "
                      << boxCenter[0] << " " << boxCenter[1] << " " << boxCenter[2] << " "
                      << std::endl;

            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                static int cnt = 0;
                // publish object infomation
                // object position x, object position y, object size width, object size length, angle_z
                std_msgs::Float64MultiArray msg;

                msg.data.push_back(boxCenter[0]);//自己写的，可行
                msg.data.push_back(boxCenter[1]);
                msg.data.push_back(size[0]);
                msg.data.push_back(size[1]);
                cout << "boxCenter = " << boxCenter[0] << " " << boxCenter[1] << endl;
                cout << "size = " << size[0] << " " << size[1] << endl;

                Eigen::Matrix3d object_rot = object_transform.rotation();
                Eigen::Vector3d euler = object_rot.eulerAngles(0, 1, 2);
                double angle = euler(2) * 180 / 3.14159265359;
                cout << "angle_z = " << angle << endl;
                msg.data.push_back(angle);

                nav_goal_pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
                cnt++;
                if(cnt > 10)
                    return;
            }

            //            goal.target_pose.pose.position.x = boxCenter[0];
            //            goal.target_pose.pose.position.y = boxCenter[1];
            //            goal.target_pose.pose.orientation.w = 1.0;
        }
    }


    //    if(flag_find_object)
    //    {
    //        ROS_INFO("Succeed to Find the goal object");
    //        ROS_INFO("Sending goal");
    //        ac.sendGoal(goal);

    //        ac.waitForResult();

    //        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //          ROS_INFO("Hooray, the base moved 1 meter forward");
    //        else
    //          ROS_INFO("The base failed to move forward 1 meter for some reason");
    //    }
    //    else
    //    {
    //        ROS_INFO("Fail to find the goal object");
    //    }
}

void NavGoalObjectUpdateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    cout << "Receive new object position........." << endl;
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    // goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg->data[0];
    goal.target_pose.pose.position.y = msg->data[1];
    goal.target_pose.pose.orientation.w = 1.0;

    cout << "update_object_pos = " << msg->data[0] << "  " << msg->data[1] << endl;

    if(flag_find_object)
    {
        ROS_INFO("Succeed to Find the goal object");
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    else
    {
        ROS_INFO("Fail to find the goal object");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SemanticNavigation");
    ros::start();
    ros::Rate loop_rate(5);
    ros::NodeHandle node_handle;
    cout << "Start to conduct semantic navigation ......" << endl;

    ros::Subscriber semantic_command_subscriber_;
    //    ros::Publisher navigation_goal_publisher_;
    ros::Publisher voice_pub = node_handle.advertise<std_msgs::String>("/robot/voice_tts", 1000);

    // object position x, object position y, object size width, object size length, angle_z
    ros::Publisher nav_goal_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/nav_goal/object", 1000);


    // semantic_command_subscriber_ = node_handle.subscribe<std_msgs::Int32>("/semantic_command", 10, &SemanticCommandCallback);
    //semantic_command_subscriber_ = node_handle.subscribe<std_msgs::Int32>("/semantic_command", 10, boost::bind(&SemanticCommandCallback, _1, voice_pub));

    semantic_command_subscriber_ = node_handle.subscribe<std_msgs::Int32>("/semantic_command", 10, boost::bind(&SemanticCommandCallback, _1, voice_pub, nav_goal_pub));

    //    semantic_command_subscriber_ = node_handle.subscribe<std_msgs::Int32>("/semantic_command", 10, boost::bind(&SemanticCommandCallback, _1, navigation_goal_publisher_), this);
    //    navigation_goal_publisher_ = node_handle.advertise<move_base_msgs::MoveBaseGoal>("/move_base/goal", 10, this);


    ros::Subscriber update_goal_object_subscriber_ = node_handle.subscribe<std_msgs::Float64MultiArray>("/nav_goal/object/update", 10, &NavGoalObjectUpdateCallback);

    if(argc != 4)
    {
        ROS_ERROR ("The input parameters are not right.");
        ros::shutdown();
        return 1;
    }

    string dataset_num = argv[1];
    string object_file_name = argv[2];
    string objects_cloud_name = argv[3];

    ObjectsPublish objects(node_handle);

    // ros::spin();

    // obtain objects
    ifstream file("./" + dataset_num + "/" + object_file_name, std::ios::in);
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
    string map_path = "./" + dataset_num + "/map.pcd";
    cout << "map_path = " << map_path << endl;
    pcl::io::loadPCDFile(map_path, *cloud);

    objects_cloud->points.clear();
    string objects_cloud_path = "./" + dataset_num + "/" + objects_cloud_name;
    cout << "object_cloud_path = " << objects_cloud_path << endl;
    pcl::io::loadPCDFile(objects_cloud_path, *objects_cloud);

    while(ros::ok())
    {
        ros::spinOnce();

        // Publish map point cloud
        //objects.PublishMapPointCloud(cloud);

        // Publish objects point cloud
        //objects.PublishObjectsPointCloud(objects_cloud);

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

void ObjectsPublish::PublishObjectsBoundingBoxes(std::vector<Cluster> clusters)
{
    int objnumber = clusters.size();

    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();

    std::cout << "Object Database size: " << objnumber << std::endl;

    rviz_visual_tools::colors object_color;

    for( int m = 0; m < objnumber; m++)
    {
        Cluster & cluster = clusters[m];
        Eigen::Vector3d size  = cluster.size;
        //Eigen::Vector3d boxCenter  = cluster.boxCenter;

//        std::cout << "Object: " << cluster.object_name << " size: "
//                  << size[0] << " " << size[1] << " " << size[2] << " "
//                  << std::endl;

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
        case hash_compile_time("table"):
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

sensor_msgs::PointCloud2 ObjectsPublish::PointsToPointCloud (PointCloud7D::Ptr pointCloud)
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
