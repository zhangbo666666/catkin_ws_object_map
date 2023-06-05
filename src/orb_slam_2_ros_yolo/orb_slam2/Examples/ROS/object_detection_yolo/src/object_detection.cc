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

#include <pcl/features/impl/integral_image_normal.hpp> // 点云积分图 法线估计
#include <pcl/features/impl/normal_3d.hpp>             // 3d点云法线估计
#include <pcl/filters/impl/extract_indices.hpp>        // 根据点云索引提取对应的点云
#include <pcl/filters/impl/voxel_grid.hpp>             // 体素格滤波
#include <pcl/kdtree/impl/kdtree_flann.hpp>            // 二叉树搜索
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>  // 分割器 提取点云
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp> // 多平面分割
#include <pcl/segmentation/plane_coefficient_comparator.h>              // 平面分割系数参数
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>    // 欧式距离 平面分割系数
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>          // rgb颜色 平面分割系数
#include <pcl/segmentation/edge_aware_plane_comparator.h>               // 边缘 平面分割系数
#include <pcl/segmentation/euclidean_cluster_comparator.h>              // 欧式距离分割
#include <pcl/filters/passthrough.h>  //直通滤波相关

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


#include <unistd.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/cpc_segmentation.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "base_object_builder.hpp"
#include "object_builder_manager.hpp"

typedef pcl::PointXYZRGBA Point7D;// 点类型 xyzrgba 点+颜色
typedef pcl::PointCloud<Point7D> PointCloud7D;// 点云类型

#define Random(x) (rand() % x)

typedef pcl::PointXYZRGBA PointT;// 点类型 xyzrgba 点+颜色
typedef pcl::PointCloud<PointT> PointCloudT;// 点云类型
typedef std::pair<float, int> PointsDistance;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

const double camera_factor = 1000;
const double cx = 325.5 - 175;
const double cy = 253.5 - 100;
const double fx = 518.0;
const double fy = 519.0;



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

int main(int argc, char **argv)
{
/*

    std::string pcd_file_path = "/home/gxc/dataset/ObjectDection/box_cloud.pcd";
    std::string pcd_file_path = "/home/gxc/dataset/SceneNN/000/objectDatabaseBeforeFusion.pcd";

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr person_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

//    std::string object_point_cloud_LCCP_name = argv[2];
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_point_cloud_LCCP(new pcl::PointCloud<pcl::PointXYZRGB>);
//    std::string object_point_cloud_LCCP_filename = "/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/experiments/results/object_pcd/" + object_point_cloud_LCCP_name;

    if (-1 == pcl::io::loadPCDFile(pcd_file_path, *person_point_cloud)) {
        cout << "error input!" << endl;
        return -1;
    }
//    if (-1 == pcl::io::loadPCDFile(object_point_cloud_LCCP_filename, *object_point_cloud_LCCP)) {
//        cout << "error input!" << endl;
//        return -1;
//    }
    if (person_point_cloud->points.size() == 0){
        cout << "point cloud size = " << person_point_cloud->points.size() << endl;
        return 0;
    }

    cout << "point cloud size = " << person_point_cloud->points.size() << endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr box_points_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for(int i = 0; i < person_point_cloud->points.size(); i++) {

//        cout << "r = " << (int)person_point_cloud->points[i].r << endl;
//        cout << "g = " << (int)person_point_cloud->points[i].g << endl;
//        cout << "b = " << (int)person_point_cloud->points[i].b << endl;

        if ((int)person_point_cloud->points[i].r == 209 && (int)person_point_cloud->points[i].g == 19 && (int)person_point_cloud->points[i].b == 27)
        {
            box_points_cloud->push_back(person_point_cloud->points[i]);
        }
    }

    box_points_cloud->width = box_points_cloud->points.size();
    box_points_cloud->height = 1;
    box_points_cloud->is_dense = true;

    std::cout << "box_points_cloud->points.size(): " << box_points_cloud->points.size() << std::endl;
    pcl::io::savePCDFileASCII("/home/gxc/dataset/ObjectDection/box.pcd", *box_points_cloud);

    // 体素格滤波======
    PointCloudT::Ptr person_cloud_tmp_1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    PointCloudT::Ptr person_cloud_tmp_2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    PointCloudT::Ptr person_cloud_tmp_3(new pcl::PointCloud<pcl::PointXYZRGBA>);


    if (person_point_cloud->points.size () > 0){
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(person_point_cloud);
        float voxelResolution = 0.001;

        vg.setLeafSize(voxelResolution, voxelResolution, voxelResolution);// 体素格子 尺寸
        vg.filter(*person_cloud_tmp_1);

        std::cout << "after voxel filtering has: " << person_cloud_tmp_1->points.size ()  << "  " << endl;
        person_cloud_tmp_1->width = person_cloud_tmp_1->points.size();
        person_cloud_tmp_1->height = 1;
        person_cloud_tmp_1->is_dense = true;

        pcl::io::savePCDFileASCII("/home/gxc/dataset/ObjectDection/voxel.pcd", *person_cloud_tmp_1);
    }

*/
/*    cv::Mat person_mask_tmp_1 = cv::Mat::zeros(225, 288, CV_8UC1);
    for (unsigned int i=0; i< person_cloud_tmp_1->size(); i++) {
        pcl::PointXYZRGBA tmp;
        tmp = person_cloud_tmp_1->points[i];

        float d = tmp.z;
        int mask_x = tmp.x * fx/tmp.z + cx;
        int mask_y = tmp.y * fy/tmp.z + cy;

//        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
        person_mask_tmp_1.at<uchar>(mask_y, mask_x) = 255;
    }
    cv::imshow("person_mask_tmp_1", person_mask_tmp_1);
    cv::imwrite("/home/gxc/dataset/ObjectDection/person_mask_tmp_1.png", person_mask_tmp_1);*/

/*
    if(person_point_cloud->points.size () > 0)
    {
        //直通滤波器，在Z轴方向上剪除多余的点
        pcl::PassThrough<pcl::PointXYZRGBA> pass;   //创建滤波器对象
        pass.setInputCloud (person_point_cloud);     //设置待滤波的点云
        pass.setFilterFieldName ("z");             //设置在Z轴方向上进行滤波
        pass.setFilterLimits (0.2, 6.0);           //设置滤波范围为0.2~6.0,在范围之外的点会被剪除
        pass.filter (*person_cloud_tmp_2);           //存储

        std::cout << "after pass through has: " << person_cloud_tmp_2->points.size () << "  " << endl;
        person_cloud_tmp_2->width = person_cloud_tmp_2->points.size();
        person_cloud_tmp_2->height = 1;
        person_cloud_tmp_2->is_dense = true;

        pcl::io::savePCDFileASCII("/home/gxc/dataset/PersonSegmentation/Scene2_196/pass.pcd", *person_cloud_tmp_1);

    }

    cv::Mat person_mask_tmp_2 = cv::Mat::zeros(225, 288, CV_8UC1);
    for (unsigned int i=0; i< person_cloud_tmp_2->size(); i++) {
        pcl::PointXYZRGBA tmp;
        tmp = person_cloud_tmp_2->points[i];

        float d = tmp.z;
        int mask_x = tmp.x * fx/tmp.z + cx;
        int mask_y = tmp.y * fy/tmp.z + cy;

//        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
        person_mask_tmp_2.at<uchar>(mask_y, mask_x) = 255;
    }
    cv::imshow("person_mask_tmp_2", person_mask_tmp_2);
    cv::imwrite("/home/gxc/dataset/PersonSegmentation/Scene2_196/person_mask_tmp_2.png", person_mask_tmp_2);
*/

/*
    if(person_point_cloud->points.size () > 5000)
    {
        //统计滤波器，删除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> Static;   //创建滤波器对象
        Static.setInputCloud (person_point_cloud);               //设置待滤波的点云
//        Static.setInputCloud (person_cloud_tmp_1);               //设置待滤波的点云
        Static.setMeanK (50);                               //设置在进行统计时考虑查询点临近点数
        Static.setStddevMulThresh (0.2);                      //设置判断是否为离群点的阀值, The distance threshold will be equal to: mean + stddev_mult * stddev.
        Static.filter (*person_cloud_tmp_3);                    //存储
        // std::cout << "Object PointCloud after statistic filtering has: " << object_cloud_tmp->points.size ()  << " data points." << std::endl;
        std::cout << "after statistical filtering: " << person_cloud_tmp_3->points.size ()  << " data points." << std::endl;
        person_cloud_tmp_3->width = person_cloud_tmp_3->points.size();
        person_cloud_tmp_3->height = 1;
        person_cloud_tmp_3->is_dense = true;

        pcl::io::savePCDFileASCII("/home/gxc/dataset/ObjectDection/statistical.pcd", *person_cloud_tmp_3);

    }
*/

/*    cv::Mat person_mask_tmp_3 = cv::Mat::zeros(225, 288, CV_8UC1);
    for (unsigned int i=0; i< person_cloud_tmp_3->size(); i++) {
        pcl::PointXYZRGBA tmp;
        tmp = person_cloud_tmp_3->points[i];

        float d = tmp.z;
        int mask_x = tmp.x * fx/tmp.z + cx;
        int mask_y = tmp.y * fy/tmp.z + cy;

//        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
        person_mask_tmp_3.at<uchar>(mask_y, mask_x) = 255;
    }
    cv::imshow("person_mask_tmp_3", person_mask_tmp_3);
    cv::imwrite("/home/gxc/dataset/ObjectDection/person_mask_tmp_3.png", person_mask_tmp_3);

    cv::Mat final;
    int dilation_size =2;
    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                             cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                             cv::Point( dilation_size, dilation_size ) );
    cv::dilate(person_mask_tmp_3, final, element,cv::Point(-1, -1),1);
    cv::erode(final, final, element);
    cv::imshow("final", final);
    cv::imwrite("/home/gxc/dataset/ObjectDection/final.png", final);*/

/* std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(final,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point());
    cv::Mat imageContours=cv::Mat::zeros(person_mask_tmp_3.size(),CV_8UC1);
    cv::Mat Contours=cv::Mat::zeros(person_mask_tmp_3.size(),CV_8UC1);  //绘制

    for(int i=0;i<contours.size();i++)
    {
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
        for(int j=0;j<contours[i].size();j++)
        {
            //绘制出contours向量内所有的像素点
            cv::Point P=cv::Point(contours[i][j].x,contours[i][j].y);
            Contours.at<uchar>(P)=255;
        }

        //输出hierarchy向量内容
        char ch[256];
        sprintf(ch,"%d",i);
        std::string str=ch;
        cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;

        //绘制轮廓
        drawContours(imageContours,contours,i,cv::Scalar(255),1,8,hierarchy);
    }

    imshow("Contours Image",imageContours); //轮廓
    imshow("Point of Contours",Contours);*/

/*        // Create the filtering object
      // 建立kd-tree对象用来搜索 .
      pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
      kdtree->setInputCloud(person_cloud_tmp_1);

      // Euclidean 聚类对象.
      pcl::EuclideanClusterExtraction<PointT> clustering;
      // 设置聚类的最小值 2cm (small values may cause objects to be divided
      // in several clusters, whereas big values may join objects in a same cluster).
      clustering.setClusterTolerance(0.05);
      // 设置聚类的小点数和最大点云数
      clustering.setMinClusterSize(800);
      clustering.setMaxClusterSize(110000);
      clustering.setSearchMethod(kdtree);
      clustering.setInputCloud(person_cloud_tmp_1);
      std::vector<pcl::PointIndices> clusters;
      clustering.extract(clusters);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr maxCluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

      // For every cluster...
      int currentClusterNum = 1;
      for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
      {
          //添加所有的点云到一个新的点云中
          pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
          for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
              cluster->points.push_back(person_point_cloud->points[*point]);
          cluster->width = cluster->points.size();
          cluster->height = 1;
          cluster->is_dense = true;

          if (currentClusterNum == 1){
              for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                  maxCluster->points.push_back(person_point_cloud->points[*point]);
              maxCluster->width = cluster->points.size();
              maxCluster->height = 1;
              maxCluster->is_dense = true;
          }
          // 保存
          if (cluster->points.size() <= 0)
              break;
          std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
          std::string fileName = "/home/gxc/dataset/ObjectDection/" + boost::to_string(currentClusterNum) + ".pcd";
          pcl::io::savePCDFileASCII(fileName, *cluster);

          currentClusterNum++;
      }
*/


    PointCloud7D::Ptr cloud(new PointCloud7D);
    //读取点云文件
    cloud->points.clear();
    std::string pointcloud_path =  argv[1];
    pcl::io::loadPCDFile(pointcloud_path, *cloud);

//    pcl::io::loadPCDFile("/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/experiments/results/object_pcd/object_LCCP_12.pcd", *cloud);
//    pcl::io::loadPCDFile("/home/gxc/dataset/ObjectDection/statistical.pcd", *cloud);
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

    std::cout << " objects[i].polygon: " << object->polygon << std::endl;

    cout << "Calculate the 3DBB" << endl;

 /*     cv::Mat tempMask1 = cv::Mat::zeros(225, 288, CV_8UC1);

      for (unsigned int i=0; i< maxCluster->size(); i++) {
          pcl::PointXYZRGBA tmp;
          tmp = maxCluster->points[i];

          float d = tmp.z;
          int mask_x = tmp.x * fx/tmp.z + cx;
          int mask_y = tmp.y * fy/tmp.z + cy;

  //        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
          tempMask1.at<uchar>(mask_y, mask_x) = 255;
      }

      for (unsigned int i=0; i< maxCluster->size(); i++) {
          pcl::PointXYZRGBA tmp;
          tmp = maxCluster->points[i];

          float d = tmp.z;
          int mask_x = tmp.x * fx/tmp.z + cx;
          int mask_y = tmp.y * fy/tmp.z + cy;

  //        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
          tempMask1.at<uchar>(mask_y, mask_x) = 255;
      }

      cv::imwrite("/home/gxc/dataset/PersonSegmentation/Scene2_196/tempMask1.png", tempMask1);

      cv::Mat dst;
      int dilation_size =2;
      cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );
      cv::dilate(tempMask1, dst, element,cv::Point(-1, -1),1);
      cv::erode(dst, dst, element);
      cv::imshow("dst", dst);
      cv::imwrite("/home/gxc/dataset/PersonSegmentation/Scene2_196/dst.png", dst);
  */
//    cv::imwrite("/home/gxc/dataset/PersonSegmentation/Scene2_196/tempMask1.png", tempMask1);

    ///  Default values of parameters before parsing
    // Supervoxel Stuff
    /*float voxel_resolution = 0.02;
    float seed_resolution = 0.1;
    float color_importance = 5.0;
    float spatial_importance = 5.0;
    float normal_importance = 5.0;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.5;
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;

    ///  Parse Arguments needed for computation
    float normals_scale = seed_resolution / 2.0;

    unsigned int k_factor = 0;
    if (use_extended_convexity)
        k_factor = 1;

    /// Preparation of Input: Supervoxel Oversegmentation
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform (use_single_cam_transform);
    super.setInputCloud (person_point_cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    PCL_INFO ("Extracting supervoxels\n");
    super.extract (supervoxel_clusters);

    if (use_supervoxel_refinement)
    {
        PCL_INFO ("Refining supervoxels\n");
        super.refineSupervoxels (2, supervoxel_clusters);
    }
    std::stringstream temp;
    temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
    PCL_INFO (temp.str ().c_str ());

    PCL_INFO ("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);

    /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

    /// The Main Step: Perform LCCPSegmentation

    PCL_INFO ("Starting Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
    lccp.setSanityCheck (use_sanity_criterion);
    lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor (k_factor);
    lccp.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize (min_segment_size);
    lccp.segment ();

    PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
    lccp.relabelCloud (*lccp_labeled_cloud);
    SuperVoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

    bool output_specified = true;
    bool add_label_field = true;
    bool sv_output_specified = true;

    /// Creating Colored Clouds and Output
    if (lccp_labeled_cloud->size () == person_point_cloud->size ())
    {
        if (output_specified)
        {
            PCL_INFO ("Saving output\n");

                pcl::io::savePCDFile ("/home/gxc/dataset/PersonSegmentation/Scene2_196/lccp_out.pcd", *lccp_labeled_cloud);

            if (sv_output_specified)
            {
                pcl::io::savePCDFile ("/home/gxc/dataset/PersonSegmentation/Scene2_196/svcloud.pcd", *sv_centroid_normal_cloud);
            }
        }
    }
    else
    {
        PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
    }

    int label_max = 0;
    for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
        if (lccp_labeled_cloud->points[i].label > label_max)
            label_max = lccp_labeled_cloud->points[i].label;
    }
    cout << "LCCP segments cloud into : " << label_max << endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr LCCPSegmentation(new pcl::PointCloud<pcl::PointXYZRGBA>);
    LCCPSegmentation->height = 1;
    LCCPSegmentation->width = lccp_labeled_cloud->size();
    LCCPSegmentation->resize(lccp_labeled_cloud->size());
    std::vector<int> subCloudPointsNum;

    for (int i = 0; i < label_max; i++) {
        int color_R = Random(255);
        int color_G = Random(255);
        int color_B = Random(255);
        int cloudPointsNum = 0; // count the points in each subcloud

        for (uint32_t j = 0; j < lccp_labeled_cloud->size(); j++) {
            if (lccp_labeled_cloud->points[j].label == i) {
                LCCPSegmentation->points[j].x = lccp_labeled_cloud->points[j].x;
                LCCPSegmentation->points[j].y = lccp_labeled_cloud->points[j].y;
                LCCPSegmentation->points[j].z = lccp_labeled_cloud->points[j].z;
                LCCPSegmentation->points[j].r = color_R;
                LCCPSegmentation->points[j].g = color_G;
                LCCPSegmentation->points[j].b = color_B;

                cloudPointsNum++;
            }
        }
        subCloudPointsNum.push_back(cloudPointsNum);
    }


    //print all values of vector to screen
    //copy (subCloudPointsNum.begin(), subCloudPointsNum.end(), ostream_iterator<int> (cout, "\n"));

    std::vector<int>::iterator pMaxNum = std::max_element(std::begin(subCloudPointsNum), std::end(subCloudPointsNum));
    int maxNumPosition = std::distance(std::begin(subCloudPointsNum), pMaxNum);
    //std::cout << "Max element is " << *pMaxNum << " at position " << maxNumPosition << std::endl;

    // Extract the object's points
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    objectCloud->height = 1;
    uint32_t pointsNum = subCloudPointsNum[maxNumPosition];
    objectCloud->width = pointsNum;
    objectCloud->resize(pointsNum);

    int n = 0;
    for (int k = 0; k < lccp_labeled_cloud->size(); k++) {
        if (lccp_labeled_cloud->points[k].label == maxNumPosition) {
            objectCloud->points[n].x = LCCPSegmentation->points[k].x;
            objectCloud->points[n].y = LCCPSegmentation->points[k].y;
            objectCloud->points[n].z = LCCPSegmentation->points[k].z;
            objectCloud->points[n].r = LCCPSegmentation->points[k].r;
            objectCloud->points[n].g = LCCPSegmentation->points[k].g;
            objectCloud->points[n].b = LCCPSegmentation->points[k].b;
            n++;
        }
    }


    cv::Mat tempMask = cv::Mat::zeros(225, 288, CV_8UC1);

    for (unsigned int i=0; i< objectCloud->size(); i++) {
        pcl::PointXYZRGBA tmp;
        tmp = objectCloud->points[i];

        float d = tmp.z;
        int mask_x = tmp.x * fx/tmp.z + cx;
        int mask_y = tmp.y * fy/tmp.z + cy;

//        cout << "mask (x,y): " << mask_x << " " << mask_y << endl;
        tempMask.at<uchar>(mask_y, mask_x) = 255;
    }

    cv::imwrite("/home/gxc/dataset/PersonSegmentation/Scene2_196/tempMask.png", tempMask);

    cv::imshow("tempMask" , tempMask);

    cout << "tempMask.size(): "<< tempMask.size() << endl;






    int v1(0), v2(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewr"));

    cloud_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    cloud_viewer->setBackgroundColor(0,0,0);
    cloud_viewer->addPointCloud<pcl::PointXYZRGBA>(person_point_cloud, "object_point_cloud",v1);
    cloud_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    cloud_viewer->setBackgroundColor(0,0,0);
    cloud_viewer->addPointCloud<pcl::PointXYZRGBA>(objectCloud, "object_point_cloud_LCCP", v2);

    cloud_viewer->spin();*/


    cv::waitKey(0);
    return 0;
}



