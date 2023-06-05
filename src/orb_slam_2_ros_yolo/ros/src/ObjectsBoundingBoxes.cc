/* This file is part of ORB-SLAM2-SSD-Semantic.
2d & 3d 融合算法
点云 多平面分割、聚�??
反向投影获取点云团的2d�??
和目标检�??2d�?? 匹配
获取每个点云团的类别信息，以�??3d边框信息
返回 cluster 数组信息


pcl/apps/src/organized_segmentation_demo.cpp.

*/

#include "ObjectsBoundingBoxes.h"

#define Random(x) (rand() % x)

ObjectsBoundingBoxes::ObjectsBoundingBoxes():
    // 平面分割
    plane_comparator_(new pcl::PlaneCoefficientComparator<Point7D, pcl::Normal>),
    euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<Point7D, pcl::Normal>),
    rgb_comparator_(new pcl::RGBPlaneCoefficientComparator<Point7D, pcl::Normal>),
    edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<Point7D, pcl::Normal>),
    // 欧式距离聚类分割
    euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<Point7D, pcl::Normal, pcl::Label>)//,
  // mObjectPoints(new PointCloudT())
{
    mPlanComparator = kPlaneCoefficientComparator;// 平面系数 RANSAC采样
    mPlanNormal_angle_threshold = 3.0;// 同一个平面法线角度差值阈�??(�??)  2.0 0.01 45.0
    normal_distance_threshold = 0.02;  // 法线方向的垂直距离阈�??(�??)      0.02  0.001  0.1
    mMin_plane_inliers = 10000;// 随机采样一致性，平面最少内点数�??  1000 - 10000

    mPlane_minimum_points=10000;// 平面点云�?? 点云最少数�??  1000 2000 3000 - 20000

    mNormal_max_depth_change = 0.02;      // 法线计算参数 深度变化阈�?(�??)        0.02  0.001  0.1
    mNormal_smooth_size = 20.0f;          // 法线 区域面积 平滑参数大小           30.0  1.0  100.0

    mEuclidean_distance_threshold = 0.01f;// 欧式距离聚类分割参数(�??) 超过认为�?? 不同的点云团 0.01 0.001 0.1

    mObject_minimum_points = 1000;// 物体点云�?? 点云最少数�?? 50 100 200 - 5000

    applyConfig();// 0. 实时配置分割器参�??=====

    mpObjectDatabase = new(ObjectDatabase);// 新建目标数据�??
    mObjectsProbBeyondThreshold = 0;

    // LCCP设定结晶参数
    voxel_resolution = 0.03; // The resolution (in meters) of voxels used
    seed_resolution = 0.05; // The average size (in meters) of resulting supervoxels
    color_importance = 2.0;
    spatial_importance = 2.0;
    normal_importance = 2.0;

    concavity_tolerance_threshold = 10;
    // Two supervoxels are unsmooth if their plane-to-plane distance DIST > (expected_distance + smoothness_threshold_*voxel_resolution_).
    smoothness_threshold = 0.5;
    min_segment_size = 0;

    // set step to count points in the bounding box
    step = 0.05; // unit: m
    threshold = 100; // the number of points in a step area
    bValid = false; // whether to use histogram to remove points, true: to use, false: not to use

    // Read the camera parameters
    cv::FileStorage fsSettings("/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/orb_slam2/config/ASUS.yaml", cv::FileStorage::READ);

    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fy"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    // mDepthMapFactor = fsSettings["DepthMapFactor"]; // run using a RGB-D sensor

    mDepthMapFactor = fsSettings["DepthMapFactorForDataset"]; // run in the dataset


    cout << "\n------------------------------" << endl;
    cout << "DepthMapFactor = " << mDepthMapFactor << endl;

    mDepthMapFactor = 1.0f / mDepthMapFactor;

    prob_threshold = fsSettings["ObjectsBoundingBoxes.ObjectProbThreshold"]; // threshold probability of 2D object to generate 3D bounding box

    // Voxel filter in LCCP segmentation
    objectVoxelResolution = fsSettings["ObjectsBoundingBoxes.ObjectVoxelResolution"];

    cout << "Camera.fx = " << fx << endl;
    cout << "Camera.fy = " << fy << endl;
    cout << "Camera.cx = " << cx << endl;
    cout << "Camera.cy = " << cy << endl;
    cout << "ObjectProbThreshold = " << prob_threshold << endl;
    cout << "ObjectVoxelResolution = " << objectVoxelResolution << endl;
    cout << "------------------------------\n" << endl;

    //    cout << "Start to initialize MFE lib ......" << endl;

    //    if(!mclInitializeApplication(NULL,0)){
    //        cout<<"initial error"<<endl;
    //        //return 1;
    //    }

    //    // 初始化库
    //    libMFEInitialize();
    //    cout<<"MFE lib Initialized ......"<<endl;

}

ObjectsBoundingBoxes::~ObjectsBoundingBoxes()
{
    //  delete plane_comparator_;
    //  delete euclidean_comparator_;
    //  delete rgb_comparator_;
    //  delete edge_aware_comparator_;
    //  delete euclidean_cluster_comparator_;


    //    // 后面是一些终止调用的程序
    //    // terminate the lib
    //    libMFETerminate();
    //    // terminate MCR
    //    mclTerminateApplication();

    delete mpObjectDatabase;// 目标数据�??====
}

// Calculate bounding boxes of objects using LCCP and Manhattan Frame
//void ObjectsBoundingBoxes::calculateObjectsBoundingBoxes(cv::Mat img_rgb,
//                                   cv::Mat img_depth,
//                                   Eigen::Isometry3d transform, // The transform matrix
//                                   std::vector<Object>& objects)

void ObjectsBoundingBoxes::calculateObjectBoundingBox(Cluster& cluster,
                                                      Eigen::Matrix3d object_MF,
                                                      bool& isObjectDatabaseUpdate)
{
    Object3d object3d;
    Eigen::Matrix3d rot_MF_Aligned = Eigen::Matrix3d::Identity();

    // rot_MF_Aligned = object_MF;
    //cout << "object_MF = \n " << object_MF << endl;

    rot_MF_Aligned = ManhattanFrameAlign(object_MF); // Align one axis of MF to the z axis of RGB-D camera coordinate system
    //rot_MF_Aligned = object_MF;

    boundingBoxEstimation(cluster.object_point_cloud_after_seg,
                          rot_MF_Aligned, step, threshold, bValid, object3d);

    cout << "Bounding box estimation is finished." << endl;

    cluster.size        = object3d.sizePt; // 尺寸
    cluster.boxCenter   = object3d.boxCenter; // 中心�??  // max_it->boxCenter; // 包围盒中心点
    cluster.minPt       = object3d.minPt;
    cluster.maxPt       = object3d.maxPt;
    cluster.coordinate_system_t = object3d.coordinate_system_t;
    cluster.coordinate_system_t_label = object3d.coordinate_system_t_label;
    cluster.eightCorners = object3d.eightCorners;

    for(int i = 0; i < 10; i++)
    {
        cluster.bb3d[i] = object3d.bb3d[i];
    }

    long time = _getTimeUsec(); // 开始计�??

    // mpObjectDatabase->addObject(cluster); // 加入数据�??======

    time = _getTimeUsec() - time; // 结束计时, ms
    cout << "Object fusion time: " << time / 1000.0 << " ms" << endl; // 显示检测时�??
    mvTimeFusion.push_back(time / 1000.0);

    isObjectDatabaseUpdate = true;
}

// new method to calculate the 3DBB
void ObjectsBoundingBoxes::calculateObjectBoundingBox(Cluster& cluster,
                                                      autosense::ObjectPtr singleObject,
                                                      bool& isObjectDatabaseUpdate)
{
    Object3d object3d;
    boundingBoxEstimation(singleObject, object3d);

    cluster.size        = object3d.sizePt; // 尺寸
    cluster.boxCenter   = object3d.boxCenter; // 中心�??  // max_it->boxCenter; // 包围盒中心点
    cluster.minPt       = object3d.minPt;
    cluster.maxPt       = object3d.maxPt;
    cluster.coordinate_system_t = object3d.coordinate_system_t;
    cluster.coordinate_system_t_label = object3d.coordinate_system_t_label;
    cluster.eightCorners = object3d.eightCorners;

    for(int i = 0; i < 10; i++)
    {
        cluster.bb3d[i] = object3d.bb3d[i];
    }

    long time = _getTimeUsec(); // 开始计�??
    // mpObjectDatabase->addObject(cluster); // 加入数据�??======
    time = _getTimeUsec() - time; // 结束计时, ms
//    cout << "Object fusion time: " << time / 1000.0 << " ms" << endl; // 显示检测时�??
    mvTimeFusion.push_back(time / 1000.0);
    isObjectDatabaseUpdate = true;
}


/*
void ObjectsBoundingBoxes::calculateObjectBoundingBox(Cluster cluster,
                                                      Eigen::Matrix3d object_MF,
                                                      bool& isObjectDatabaseUpdate)
{
    Object3d object3d_MF_Not_Aligned, object3d_MF_Aligned;
    Eigen::Matrix3d rot_MF_Aligned = Eigen::Matrix3d::Identity();

    // rot_MF_Aligned = object_MF;
    //cout << "object_MF = \n " << object_MF << endl;

    rot_MF_Aligned = ManhattanFrameAlign(object_MF); // Align one axis of MF to the y axis of RGB-D camera coordinate system
    //rot_MF_Aligned = object_MF;

    boundingBoxEstimation(cluster.object_point_cloud_after_seg,
                          object_MF, step, threshold, bValid, object3d_MF_Not_Aligned);

    boundingBoxEstimation(cluster.object_point_cloud_after_seg,
                          rot_MF_Aligned, step, threshold, bValid, object3d_MF_Aligned);

    cluster.size        = object3d_MF_Not_Aligned.sizePt; // 尺寸
    cluster.boxCenter   = object3d_MF_Not_Aligned.boxCenter; // 中心�??  // max_it->boxCenter; // 包围盒中心点
    cluster.minPt       = object3d_MF_Not_Aligned.minPt;
    cluster.maxPt       = object3d_MF_Not_Aligned.maxPt;
    cluster.coordinate_system_t = object3d_MF_Not_Aligned.coordinate_system_t;
    cluster.coordinate_system_t_label = object3d_MF_Not_Aligned.coordinate_system_t_label;
    cluster.eightCorners = object3d_MF_Not_Aligned.eightCorners;

    for(int i = 0; i < 10; i++)
    {
        cluster.bb3d[i] = object3d_MF_Aligned.bb3d[i];
    }

    long time = _getTimeUsec(); // 开始计�??

    mpObjectDatabase->addObject(cluster); // 加入数据�??======

    time = _getTimeUsec() - time; // 结束计时, ms
    cout << "Object fusion time: " << time / 1000.0 << " ms" << endl; // 显示检测时�??
    mvTimeFusion.push_back(time / 1000.0);

    isObjectDatabaseUpdate = true;
}
*/

// Extract 3D objects in each keyframe
//void ObjectsBoundingBoxes::extractObjects(cv::Mat img_rgb,
//                               cv::Mat img_depth,
//                               Eigen::Isometry3d transform, // The transform matrix to system
//                               std::vector<Object>& objects, // 2d detection objects
//                               std::vector<Cluster>& clusters)
void ObjectsBoundingBoxes::extractObjectsPointClouds(cv::Mat img_rgb,
                                                     cv::Mat img_depth,
                                                     Eigen::Isometry3d transform, // The transform matrix to world system
                                                     std::vector<Object>& objects, // 2d detection objects
                                                     std::vector<Cluster>& clusters)
{
    cout << "Step2.1: Extract Objects PointClouds"  << endl;
    cout << "         Received objects size:" << objects.size() << endl;
    int counttt = 0;
    clusters.clear();// 数据库清�??
    //cout << "Number of objects = " << objects.size() << endl;

    // Convert the depth image using mDepthMapFactor

    // cout << "ori = " << img_depth.ptr<uint16_t>(200)[300] << endl;

    // if((fabs(mDepthMapFactor-1.0f)>1e-5) || img_depth.type()!=CV_32F)
    //     img_depth.convertTo(img_depth, CV_32F, mDepthMapFactor);
    // cout << "ori = " << img_depth.ptr<float_t>(200)[300] << endl;
    int iii=0;
    for (std::vector<Object>::iterator obj2d_it = objects.begin();
         obj2d_it != objects.end(); ++obj2d_it)// 每一�??2d物体
    {
        iii++;
        // the probability of object >= threshold
        if(obj2d_it->prob >= prob_threshold)
        {
            mObjectsProbBeyondThreshold++;
            long time = _getTimeUsec(); // 开始计�??

            //Object3d object3d;

            // 1. Obtain the point cloud for each 2D object
            // 使用智能指针，创建一个空点云。这种指针用完会自动释放, need to initialise: ( new PointCloud7D )
            PointCloud7D::Ptr object_cloud ( new PointCloud7D );
            PointCloud7D::Ptr object_cloud_tmp ( new PointCloud7D );
            PointCloud7D::Ptr object_cloud_seg ( new PointCloud7D ); // after LCCP segmentation

#pragma omp parallel for   // =======================omp 多线�?? 并行处理

            PointCloud7D::Ptr tmp( new PointCloud7D() );
            // point cloud is null ptr

            // YOLO 2D object detection
            /*int x_min = obj2d_it->x_min;
            int y_min = obj2d_it->y_min;
            int x_max = obj2d_it->x_max;
            int y_max = obj2d_it->y_max;

            for (int m=y_min; m<y_max; m++)
            {
                for (int n=x_min; n<x_max; n++)
                {
                    float d = img_depth.ptr<float>(m)[n];
                    if (d<0.01 || d>6)
                    {
                        continue;
                    }
                    Point7D p;
                    p.z = d;
                    p.x = (n - cx) * p.z / fx;
                    p.y = (m - cy) * p.z / fy;

                    //                    p.z = -(m - cy) * p.z / fy;
                    //                    p.x = d;
                    //                    p.y = -(n - cx) * p.z / fx;

                    p.r = img_rgb.ptr<uchar>(m)[n*3];
                    p.g = img_rgb.ptr<uchar>(m)[n*3+1];
                    p.b = img_rgb.ptr<uchar>(m)[n*3+2];

                    tmp->points.push_back(p);
                }
            }*/

            counttt =0;
            // Mask_rcnn and yolact
            double xc,yc,zc;
            double xmax = 0;
            double xmin = 1000000;
            double ymax = 0;
            double ymin = 1000000;
            double zmax = 0;
            double zmin = 1000000;
            cv::Mat objectMask = obj2d_it->objectMask;

            //edit by gxc
            for (int m=0; m<480; m++)
            {
                for (int n=0; n<640; n++)
                {
                    if(objectMask.ptr<uchar>(m)[n] == 255)
                    {
                        counttt++;
                        float d = img_depth.ptr<uint16_t>(m)[n] / 1000.0;
                        // cout << m << " " << n << " " << d << endl;
                        if (d < 0.20 || d > 6 || isnan(d)) // bool isnan( float arg );
                        {
                            continue;
                        }

                        Point7D p;
                        p.z = d;
                        p.x = (n - cx) * p.z / fx;
                        p.y = (m - cy) * p.z / fy;

                        //cout << "p.x = " << p.x << endl;
                        //cout << "p.y = " << p.y << endl;
                        //cout << "p.z = " << p.z << endl;
                        // p.z = -(m - cy) * p.z / fy;
                        // p.x = d;
                        // p.y = -(n - cx) * p.z / fx;

                        p.r = img_rgb.ptr<uchar>(m)[n*3] *0;
                        p.g = img_rgb.ptr<uchar>(m)[n*3+1] *0;
                        p.b = img_rgb.ptr<uchar>(m)[n*3+2] *0 + 255;

                        // cout << p.r << " " << p.g << " " << p.b << endl;
                        xmax = p.x>xmax?p.x:xmax;
                        ymax = p.y>ymax?p.y:ymax;
                        zmax = p.z>zmax?p.z:zmax;
                        xmin = p.x<xmin?p.x:xmin;
                        ymin = p.y<ymin?p.y:ymin;
                        zmin = p.z<zmin?p.z:zmin;
                        // xc += p.x;
                        // yc += p.y;
                        // zc += p.z;
                        tmp->points.push_back(p);
                    }
                }
            }
            cout << "Step2.1.1: mask size = " << counttt << endl;
            cout << "Step2.1.1: Object point size = " << tmp->points.size() << endl;
            xc =(xmin+xmax)/2.0;
            yc =(ymin+ymax)/2.0;
            zc =(zmin+zmax)/2.0 ;

            if(tmp->points.size() > 0)
            {
                // for real environment experiments, the transformation is in the camera coodinate system
                //pcl::transformPointCloud( *tmp, *object_cloud, transform.inverse().matrix());

                // for SceneNN dataset, the transformation is in the world coodinate system
                pcl::transformPointCloud( *tmp, *object_cloud, transform.matrix());

                // 对点云坐标做变换，绕x轴旋�??90度，将z轴指向上�??
                Eigen::Affine3f trans = Eigen::Affine3f::Identity();
                //trans.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)));
                trans.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(1,0,0)));
                pcl::transformPointCloud(*object_cloud, *object_cloud, trans);

                object_cloud->is_dense = false;

                std::cout << "Step2.1.2: After transformation, object point size: " << object_cloud->points.size () << "  " <<endl; //  << " data points." << std::endl;

                if(object_cloud->points.size () > 200)
                {
                    // 2. LCCP segmentation
                    object_cloud->height = 1;
                    object_cloud->width = object_cloud->points.size();
                    // std::string fileName = "/home/gxc/dataset/SceneNN/m_result/LCCP/beforefilter" + boost::to_string(iii) + ".pcd";
                   // pcl::io::savePCDFileASCII(fileName, *object_cloud);
                   //gxc
                    // if(!pointCloudFilter(object_cloud, object_cloud_tmp))
                    //     continue;
//                    pcl::io::savePCDFileASCII("/home/gxc/dataset/SceneNN/m_result/filter/after_filter.pcd", *object_cloud_tmp);


                    time = _getTimeUsec() - time; // 结束计时, ms
                    mvTimeExtractPointCloudAndFilter.push_back(time / 1000.0);
                    cout << "           Extract an object and filter time: " << time / 1000.0 << " ms" << endl; // 显示检测时�??

                    // if(object_cloud_tmp->points.size () > 50)
                    if(object_cloud->points.size () > 50)
                    {
                        long time_lccp = _getTimeUsec(); // 开始计�??

                        // object point cloud segmentation
                        //pointCloudLCCP(object_cloud_tmp, object_cloud_seg);
                        // std::string fileName = "/home/gxc/dataset/SceneNN/m_result/LCCP/beforeclust" + boost::to_string(iii) + ".pcd";
                     //   pcl::io::savePCDFileASCII(fileName, *object_cloud_tmp);
                        // // //gxc
                        // pointCloudEuclidean(object_cloud_tmp, object_cloud_seg);

                        time_lccp = _getTimeUsec() - time_lccp; // 结束计时, ms
                        mvTimeLCCP.push_back(time_lccp / 1000.0);
                        cout << "           Object Euclidean time: " << time_lccp / 1000.0 << " ms" << endl; // 显示检测时�??
//                        pcl::io::savePCDFileASCII("/home/gxc/dataset/SceneNN/m_result/LCCP/after_lccp.pcd", *object_cloud_tmp);
                        //gxc
                        // if(object_cloud_seg->points.size() < 20)
                        // {
                        //     continue;
                        // }

                        // 3d对象=========
                        Cluster cluster;
                        cluster.centroid(0) = xc;
                        cluster.centroid(1) = yc;
                        cluster.centroid(2) = zc;
                        cluster.object_name = obj2d_it->object_name; // 名字
                        // cluster.class_id    = obj2d_it->class_id;    // 类别id
                        cluster.prob        = obj2d_it->prob; // 置信�??

                        // gxc
                        // cluster.object_point_cloud_before_seg = object_cloud_tmp;
                        // cluster.object_point_cloud_after_seg = object_cloud_seg;
                        cluster.object_point_cloud_before_seg = object_cloud;
                        cluster.object_point_cloud_after_seg = object_cloud;
                        clusters.push_back(cluster);
                    }
                    else
                    {
                        cout << "Step2.1.3: The points of an object point cloud after filtering is too small!" << endl;
                        continue;
                    }
                }
                else
                {
                    cout << "Step2.1.2: The points of an object point cloud is too small!" << endl;
                    continue;
                }
            }
            else
            {
                cout << "Step2.1.1:The object point cloud from Mask R-CNN is empty!" << endl;
                continue;
            }
        }
        else{
            cout << "Step2.1.1: Object " << obj2d_it->object_name  << " prob is so small! " << endl;
        }
        cout << "-----------------------" << endl;
    }
}

// point cloud filter: 体素格滤�??, 直通滤波器, 统计滤波�??
bool ObjectsBoundingBoxes::pointCloudFilter(PointCloud7D::Ptr object_cloud, PointCloud7D::Ptr object_cloud_tmp)
{
    if(object_cloud->points.size() > 0)
    {
        // 体素格滤�??======
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(object_cloud);
        float voxelResolution = 0.01;
        if(object_cloud->points.size() > 100000)
        {
            //voxelResolution = objectVoxelResolution * 1.8;
            voxelResolution = objectVoxelResolution;
            vg.setLeafSize(voxelResolution, voxelResolution, voxelResolution);// 体素格子 尺寸
            vg.filter(*object_cloud_tmp);
        }
        if((object_cloud->points.size() <= 100000) && (object_cloud->points.size() >= 20000))
        {
            //voxelResolution = objectVoxelResolution * 1.5;
            voxelResolution = objectVoxelResolution;
            vg.setLeafSize(voxelResolution, voxelResolution, voxelResolution);// 体素格子 尺寸
            vg.filter(*object_cloud_tmp);
        }
        if((object_cloud->points.size() <= 20000) && (object_cloud->points.size() >= 3000))
        {
            voxelResolution = objectVoxelResolution;
            vg.setLeafSize(voxelResolution, voxelResolution, voxelResolution);// 体素格子 尺寸
            vg.filter(*object_cloud_tmp);
        }
        if(object_cloud->points.size() < 3000)
        {
            *object_cloud_tmp = *object_cloud;
        }

        std::cout << "           After voxel filtering has: " << object_cloud_tmp->points.size ()  << "  " << endl;


//edit by ,used by global map
        // if(object_cloud_tmp->points.size () > 0)
        // {
        //         //直通滤波器，在Z轴方向上剪除多余的点
        //         pcl::PassThrough<pcl::PointXYZRGBA> pass;   //创建滤波器对�??
        //         pass.setInputCloud (object_cloud_tmp);     //设置待滤波的点云
        //         pass.setFilterFieldName ("z");             //设置在Z轴方向上进行滤波
        //         pass.setFilterLimits (-0.05, 0.2);           //设置滤波范围�??0.2~6.0,在范围之外的点会被剪�??
        //         pass.setFilterLimitsNegative(true);
        //         //pass.filter (*object_cloud_tmp);           //存储

        //         std::cout << "           After pass through has: " << object_cloud_tmp->points.size () << "  " << endl;
        //     }
// //
        // if(object_cloud_tmp->points.size () > 5000)
        // {
        //     //统计滤波器，删除离群�??
        //     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> Static;   //创建滤波器对�??
        //     Static.setInputCloud (object_cloud_tmp);               //设置待滤波的点云
        //     Static.setMeanK (50);                               //设置在进行统计时考虑查询点临近点�??
        //     Static.setStddevMulThresh (0.5);                      //设置判断是否为离群点的阀�??, The distance threshold will be equal to: mean + stddev_mult * stddev.
        //     Static.filter (*object_cloud_tmp);                    //存储
        //     // std::cout << "Object PointCloud after statistic filtering has: " << object_cloud_tmp->points.size ()  << " data points." << std::endl;
        //     std::cout << "           After outlier filtering: " << object_cloud_tmp->points.size () << std::endl;
        // }

        if(object_cloud_tmp->points.size () > 300)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        cout << "Step2.1.2: Point cloud is empty!" << endl;
        return false;
    }
}

void ObjectsBoundingBoxes::pointCloudEuclidean(PointCloud7D::Ptr cloud,
                                          PointCloud7D::Ptr& objectCloud)
{
    pcl::search::KdTree<Point7D>::Ptr kdtree(new pcl::search::KdTree<Point7D>);
    kdtree->setInputCloud(cloud);

    // Euclidean 聚类对象.
    pcl::EuclideanClusterExtraction<Point7D> clustering;
    // 设置聚类的最小�? 2cm (small values may cause objects to be divided
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
    std::cout<<"clusters.size() is "<<clusters.size()<<endl;
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        
        //添加所有的点云到一个新的点云中
        pcl::PointCloud<Point7D>::Ptr cluster(new pcl::PointCloud<Point7D>);
        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(cloud->points[*point]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        std::cout << "Cluster  has " << cluster->points.size() << " points." << std::endl;
        std::cout<<"whether comes in 1.0 "<<endl;
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

        std::cout<<"whether comes in 2.0 "<<endl;
        // 保存
        if (cluster->points.size() <= 0)
            break;
        std::cout << "Step2.1.3: Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
        std::string fileName = "/home/zb/dataset/" + boost::to_string(currentClusterNum) + ".pcd";
        pcl::io::savePCDFileASCII(fileName, *cluster);
//
        currentClusterNum++;
    }
    std::cout << "           After  pointCloudEuclidean  object has " << objectCloud->points.size() << " points." << std::endl;
}

// point cloud segmentation using LCCP
void ObjectsBoundingBoxes::pointCloudLCCP(PointCloud7D::Ptr cloud,
                                          PointCloud7D::Ptr& objectCloud)
{
    if(cloud->points.size() > 10)
    {
        //------------------------------
        // LCCP
        //------------------------------
        bool use_single_cam_transform = false;
        bool use_supervoxel_refinement = false;

        //设定结晶参数
        //    float voxel_resolution = 0.03; // The resolution (in meters) of voxels used
        //    float seed_resolution = 0.05; // The average size (in meters) of resulting supervoxels
        //    float color_importance = 2.0;
        //    float spatial_importance = 2.0;
        //    float normal_importance = 2.0;

        pcl::SupervoxelClustering<Point7D> super(voxel_resolution, seed_resolution);
        super.setUseSingleCameraTransform(use_single_cam_transform);
        //super.setInputCloud(input_cloud_ptr);
        super.setInputCloud(cloud);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        std::map<uint32_t, pcl::Supervoxel<Point7D>::Ptr> supervoxel_clusters;
        super.extract(supervoxel_clusters);

        //PCL_INFO("Getting supervoxel adjacency\n");
        std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

        // Store the over segmentation point cloud
        /*
    pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();
    ofstream outFile1("过分�??.txt", std::ios_base::out);
    for (int i = 0; i < overseg->size(); i++) {
        outFile1 << overseg->points[i].x << "\t" << overseg->points[i].y << "\t" << overseg->points[i].z << "\t" << overseg->points[i].label << endl;
    }
    int label_max1 = 0;
    for (int i = 0; i< overseg->size(); i++) {
        if (overseg->points[i].label > label_max1)
            label_max1 = overseg->points[i].label;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr superSegmentation(new pcl::PointCloud<pcl::PointXYZRGB>);
    superSegmentation->height = 1;
    superSegmentation->width = overseg->size();
    superSegmentation->resize(overseg->size());
    for (int i = 0; i < label_max1; i++) {
        int color_R = Random(255);
        int color_G = Random(255);
        int color_B = Random(255);

        for (int j = 0; j < overseg->size(); j++) {
            if (overseg->points[j].label == i) {
                superSegmentation->points[j].x = overseg->points[j].x;
                superSegmentation->points[j].y = overseg->points[j].y;
                superSegmentation->points[j].z = overseg->points[j].z;
                superSegmentation->points[j].r = color_R;
                superSegmentation->points[j].g = color_G;
                superSegmentation->points[j].b = color_B;
            }
        }
    }
    //pcl::io::savePCDFileASCII("过分�??.pcd", *superSegmentation);
*/
        //LCCP分割
        /** \brief Set normal threshold
      *  \param[in] concavity_tolerance_threshold_arg the concavity tolerance angle in [deg] to set */
        //    float concavity_tolerance_threshold = CONCAVITY_TOLERANCE_TH;
        //    // Two supervoxels are unsmooth if their plane-to-plane distance DIST > (expected_distance + smoothness_threshold_*voxel_resolution_).
        //    float smoothness_threshold = SMOOTHNESS_TH;
        //    uint32_t min_segment_size = MIN_SEGMENT_SIZE;

        // Set the value used for k convexity.
        // For k>0 convex connections between p_i and p_j require k common neighbors of these patches
        // that have a convex connection to both.
        unsigned int k_factor = 0;
        bool use_extended_convexity = false;
        bool use_sanity_criterion = false;
        //PCL_INFO("Starting Segmentation\n");
        pcl::LCCPSegmentation<Point7D> lccp;
        lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
        lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
        lccp.setKFactor(k_factor);
        lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
        lccp.setMinSegmentSize(min_segment_size);
        lccp.segment();

        //PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

        pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
        pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
        lccp.relabelCloud(*lccp_labeled_cloud);
        SuperVoxelAdjacencyList sv_adjacency_list;
        lccp.getSVAdjacencyList(sv_adjacency_list);

        /*
    ofstream outFile2("分割后合�??.txt", std::ios_base::out);
    for (int i = 0; i < lccp_labeled_cloud->size();i++) {
        outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << endl;
    }
    */
        int label_max2 = 0;
        for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
            if (lccp_labeled_cloud->points[i].label > label_max2)
                label_max2 = lccp_labeled_cloud->points[i].label;
        }
        //cout << "LCCP segments cloud into : " << label_max2 << endl;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr LCCPSegmentation(new pcl::PointCloud<pcl::PointXYZRGBA>);
        LCCPSegmentation->height = 1;
        LCCPSegmentation->width = lccp_labeled_cloud->size();
        LCCPSegmentation->resize(lccp_labeled_cloud->size());
        vector<int> subCloudPointsNum;

        for (int i = 0; i < label_max2; i++) {
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

        vector<int>::iterator pMaxNum = std::max_element(std::begin(subCloudPointsNum), std::end(subCloudPointsNum));
        int maxNumPosition = std::distance(std::begin(subCloudPointsNum), pMaxNum);
        //std::cout << "Max element is " << *pMaxNum << " at position " << maxNumPosition << std::endl;

        // Extract the object's points
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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

        //std::cout << "PointCloud after LCCP has: " << objectCloud->points.size ()  << " data points." << std::endl;

        //    pcl::io::savePCDFileASCII("分割后合�??.pcd", *LCCPSegmentation);
        //    pcl::io::savePCDFileASCII("object_LCCP.pcd", *objectCloud);
    }
    else
    {
        cout << "The input point cloud for LCCP is too small." << endl;
    }
}


void ObjectsBoundingBoxes::normalEstimation (PointCloud7D::Ptr cloud,
                                             Eigen::MatrixXd& objectNormals)
{
    // Normal estimation*
    //法向计算
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    //建立kdtree来进行近邻点集搜�??
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    //为kdtree添加点云数据
    tree->setInputCloud (cloud);

    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    //点云法向计算时，需要搜索的近邻点大�??
    n.setKSearch (20);
    //开始进行法向计�??
    n.compute (*normals);

    int pointCloudSize;
    pointCloudSize = normals->size();
    //cout << "There are total " << pointCloudSize << " point cloud normals!" << endl;

    Eigen::MatrixXd Normals(pointCloudSize, 3);
    objectNormals = Normals;

    for(int i = 0; i < pointCloudSize; i++)
    {
        objectNormals(i, 0) = normals->points[i].normal_x;
        objectNormals(i, 1) = normals->points[i].normal_y;
        objectNormals(i, 2) = normals->points[i].normal_z;
    }
}

/*
// Calculate the Manhattan frame of object point cloud
int ObjectsBoundingBoxes::objectMFE(Eigen::MatrixXd normals,
                                    Eigen::Matrix3d& rotMF)
{

    // initialize lib，这里必须做初始化！
    // 初始化函数库
    //    if(!mclInitializeApplication(NULL,0)){
    //        cout<<"initial error"<<endl;
    //        return 1;
    //    }
    //    // 初始化库
    //    libMFEInitialize();

    // 为变量分配内存空间，可以查帮助mwArray

    mwArray mwInput(normals.rows(), normals.cols(), mxDOUBLE_CLASS); // 1�??1表示矩阵的大小（所有maltab只有一种变量，就是矩阵，为了和Cpp变量接轨，设置成1*1的矩阵，mxDOUBLE_CLASS表示变量的精度）
    mwArray mwOutput(1, 1, mxDOUBLE_CLASS);
    double *normalsArray = new double[normals.rows() * normals.cols()]();
    for(int i = 0; i < normals.rows(); i++){
        for(int j = 0; j < normals.cols(); j++){
            //cout << normals(i, j) << endl;
            normalsArray[i * normals.cols() + j] = normals(i, j);
        }
    }
    mwInput.SetData(normalsArray, normals.rows() * normals.cols());
    //cout << "mwInput = " << mwInput << endl;
    delete[] normalsArray;
    // 调用自己写的函数
    MFE(1, mwOutput, mwInput);
    //cout << " mwOutput = \n" << mwOutput << endl;
    // get data
    double mf_matrix[9] = {0.0};
    mwOutput.GetData(mf_matrix, 9);

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            rotMF(i, j) = mf_matrix[i * 3 + j];
        }
    }

    //cout << " rotMF = \n" << rotMF << endl;

    //    // 后面是一些终止调用的程序
    //    // terminate the lib
    //    libMFETerminate();
    //    // terminate MCR
    //    mclTerminateApplication();
    return 0;
}
*/

// Get minimum and maximum values in x, y, z axes
void ObjectsBoundingBoxes::getMinMax3DPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                                 pcl::PointXYZ &min,
                                                 pcl::PointXYZ &max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->points.resize(cloud->size());

    for (size_t i = 0; i < cloud->points.size(); i++){
        cloud_xyz->points[i].x = cloud->points[i].x;
        cloud_xyz->points[i].y = cloud->points[i].y;
        cloud_xyz->points[i].z = cloud->points[i].z;
    }

    pcl::getMinMax3D(*cloud_xyz, min, max);
}

// 统计点云沿三个坐标轴方向的直方图
// step: slide window length
// threshold: number of point cloud in a step area
// valid: whether to use histogram to remove points
void ObjectsBoundingBoxes::StatisticalPointCloudHistogram(PointCloud7D::Ptr cloud,
                                                          float step,
                                                          int threshold,
                                                          bool bValid,
                                                          Object3d& object3d)
{
    // Obtain the max and min values
    pcl::PointXYZ min;//用于存放三个轴的最小�?
    pcl::PointXYZ max;//用于存放三个轴的最大�?
    //pcl::PointXYZ center;//new center of the bounding box

    getMinMax3DPointCloud(cloud, min, max);

    //    object3d.min_not_cut_down = min;
    //    object3d.max_not_cut_down = max;

    //    cout<<"min.x = "<<min.x<<"\n"<<endl;
    //    cout<<"max.x = "<<max.x<<"\n"<<endl;
    //    cout<<"min.y = "<<min.y<<"\n"<<endl;
    //    cout<<"max.y = "<<max.y<<"\n"<<endl;
    //    cout<<"min.z = "<<min.z<<"\n"<<endl;
    //    cout<<"max.z = "<<max.z<<"\n"<<endl;

    //    object3d.center_not_cut_down.x = (max.x + min.x) / 2.0;
    //    object3d.center_not_cut_down.y = (max.y + min.y) / 2.0;
    //    object3d.center_not_cut_down.z = (max.z + min.z) / 2.0;

    // calculate the size of bounding box
    //    object3d.width_not_cut_down = fabs(max.x - min.x);
    //    object3d.height_not_cut_down = fabs(max.y - min.y);
    //    object3d.depth_not_cut_down = fabs(max.z - min.z);

    //    object3d.width_not_cut_down = fabs(max.x - min.x);
    //    object3d.height_not_cut_down = fabs(max.y - min.y);
    //    object3d.depth_not_cut_down = fabs(max.z - min.z);

    //    cout << "width = "<< object3d.width_not_cut_down << endl;
    //    cout << "height = "<< object3d.height_not_cut_down << endl;
    //    cout << "depth = "<< object3d.depth_not_cut_down << endl;

    int stepNum_x = ceil(fabs(max.x - min.x) / step);
    int stepNum_y = ceil(fabs(max.y - min.y) / step);
    int stepNum_z = ceil(fabs(max.z - min.z) / step);

    int *stepPointsNum_x = new int[stepNum_x]();
    int *stepPointsNum_y = new int[stepNum_y]();
    int *stepPointsNum_z = new int[stepNum_z]();

    if(bValid)
    {
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            int n_x = 0, n_y = 0, n_z = 0;

            while(!((cloud->points[i].x >= (min.x + n_x * step)) &&
                    (cloud->points[i].x < (min.x + (n_x + 1) * step))))
            {
                n_x++;
            }
            stepPointsNum_x[n_x]++;

            while(!((cloud->points[i].y >= (min.y + n_y * step)) &&
                    (cloud->points[i].y < (min.y + (n_y + 1) * step))))
            {
                n_y++;
            }
            stepPointsNum_y[n_y]++;

            while(!((cloud->points[i].z >= (min.z + n_z * step)) &&
                    (cloud->points[i].z < (min.z + (n_z + 1) * step))))
            {
                n_z++;
            }
            stepPointsNum_z[n_z]++;
        }

        //cout << "stepNum_x = " << stepNum_x << endl;
        for(int k = 0; k < stepNum_x; k++)
        {
            cout << stepPointsNum_x[k] << endl;
        }
        //cout << "stepNum_y = " << stepNum_y << endl;
        for(int k = 0; k < stepNum_y; k++)
        {
            cout << stepPointsNum_y[k] << endl;
        }
        //cout << "stepNum_z = " << stepNum_z << endl;
        for(int k = 0; k < stepNum_z; k++)
        {
            cout << stepPointsNum_z[k] << endl;
        }

        pcl::PointXYZ min_temp;
        pcl::PointXYZ max_temp;

        // decrease the size of bounding box
        // decrease the size in x axis
        bool flag_min_x = false;
        for(int k = 0; k < ceil(stepNum_x / 2.0); k++)
        {
            if((stepPointsNum_x[k] < threshold))
            {
                flag_min_x = true;
                min_temp.x = min.x + (k + 1) * step;
            }
        }
        if(flag_min_x == false)
        {
            min_temp.x = min.x;
        }
        bool flag_max_x = false;
        for(int k = 0; k < ceil(stepNum_x / 2.0); k++)
        {
            if((stepPointsNum_x[stepNum_x - k - 1] < threshold))
            {
                flag_max_x = true;
                max_temp.x = max.x - (k + 1) * step;
            }
        }
        if(flag_max_x == false)
        {
            max_temp.x = max.x;
        }
        // decrease the size in y axis
        bool flag_min_y = false;
        for(int k = 0; k < ceil(stepNum_y / 2.0); k++)
        {
            if((stepPointsNum_y[k] < threshold))
            {
                flag_min_y = true;
                min_temp.y = min.y + (k + 1) * step;
            }
        }
        if(flag_min_y == false)
        {
            min_temp.y = min.y;
        }
        bool flag_max_y = false;
        for(int k = 0; k < ceil(stepNum_y / 2.0); k++)
        {
            if((stepPointsNum_y[stepNum_y - k - 1] < threshold))
            {
                flag_max_y = true;
                max_temp.y= max.y - (k + 1) * step;
            }
        }
        if(flag_max_y == false)
        {
            max_temp.y = max.y;
        }
        // decrease the size in z axis
        bool flag_min_z = false;
        for(int k = 0; k < ceil(stepNum_z / 2.0); k++)
        {
            if((stepPointsNum_z[k] < threshold))
            {
                flag_min_z = true;
                min_temp.z = min.z + (k + 1) * step;
            }
        }
        if(flag_min_z == false)
        {
            min_temp.z = min.z;
        }
        bool flag_max_z = false;
        for(int k = 0; k < ceil(stepNum_z / 2.0); k++)
        {
            if((stepPointsNum_z[stepNum_z - k - 1] < threshold))
            {
                flag_max_z = true;
                max_temp.z = max.z - (k + 1) * step;
            }
        }
        if(flag_max_z == false)
        {
            max_temp.z = max.z;
        }

        min = min_temp;
        max = max_temp;
    }

    // original point in world coordinate system, not in MF
    object3d.minPt = Eigen::Vector3d(min.x, min.y, min.z);
    object3d.maxPt = Eigen::Vector3d(max.x, max.y, max.z);

    // 3d边框
    object3d.sizePt = Eigen::Vector3d(fabs(max.x - min.x),
                                      fabs(max.y - min.y),
                                      fabs(max.z - min.z));
    // 3d边框中心, in the MF coodinate system
    object3d.boxCenter = Eigen::Vector3d((max.x + min.x) / 2.0,
                                         (max.y + min.y) / 2.0,
                                         (max.z + min.z) / 2.0);
    //    cout<<"min.x = "<<min.x<<"\n"<<endl;
    //    cout<<"max.x = "<<max.x<<"\n"<<endl;
    //    cout<<"min.y = "<<min.y<<"\n"<<endl;
    //    cout<<"max.y = "<<max.y<<"\n"<<endl;
    //    cout<<"min.z = "<<min.z<<"\n"<<endl;
    //    cout<<"max.z = "<<max.z<<"\n"<<endl;

    //    cout << "width = "<< fabs(max.x - min.x) << endl;
    //    cout << "height = "<< fabs(max.y - min.y) << endl;
    //    cout << "depth = "<< fabs(max.z - min.z) << endl;

    delete[] stepPointsNum_x;
    delete[] stepPointsNum_y;
    delete[] stepPointsNum_z;
}


void ObjectsBoundingBoxes::boundingBoxEstimation(PointCloud7D::Ptr cloud,
                                                 Eigen::Matrix3d rot_MF,
                                                 float step, // step: slide window length
                                                 int threshold, // threshold: number of point cloud in a step area
                                                 bool bValid,
                                                 Object3d& object3d)
{
    // calculate the transform matrix to convert point cloud from the camera coordinate system to MF coordinate system
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform_1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform_label = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d transform_inv = Eigen::Matrix4d::Identity();

    for(int m = 0; m < 3; m++)
        for(int n =0; n < 3; n++)
            transform(m, n) = rot_MF(m, n);

    transform_1 = transform;
    transform_label = transform;

    //std::cout << transform << std::endl;

    // Executing the transformation
    PointCloud7D::Ptr transformed_cloud (new PointCloud7D ());

    transform_inv = transform.inverse();
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_inv);

    StatisticalPointCloudHistogram(transformed_cloud, step, threshold, bValid, object3d);
    Eigen::Vector4d bb_center_(object3d.boxCenter[0], object3d.boxCenter[1], object3d.boxCenter[2], 1);
    //Eigen::Vector4d bb_label_(object3d.minPt[0], object3d.minPt[1], object3d.maxPt[2] + 0.1, 1);

    Eigen::Vector4d bb_center_sys;

    bb_center_sys = transform * bb_center_;
    //bb_label_sys = transform * bb_label_;

    // Convert to world coordinate system
    object3d.boxCenter = Eigen::Vector3d(bb_center_sys(0), bb_center_sys(1), bb_center_sys(2));

    object3d.translation << bb_center_sys(0), bb_center_sys(1), bb_center_sys(2);
    transform_1(0, 3) = bb_center_sys(0);
    transform_1(1, 3) = bb_center_sys(1);
    transform_1(2, 3) = bb_center_sys(2);

    Eigen::Affine3d a3f_transform(transform_1.cast<double>());
    //cout << "a3f_transform = \n" << a3f_transform.matrix() << endl;
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

    // cout << "transform =\n" << transform << endl;

    //cout << "boundingBoxCorners =\n" << boundingBoxCorners << endl;
    boundingBoxCorners_sys = transform * boundingBoxCorners;
    //cout << "boundingBoxCorners_sys =\n" << boundingBoxCorners_sys << endl;

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
        bb3d[2] = bb3d[0];
        bb3d[4] = (object3d.eightCorners[2].x + object3d.eightCorners[6].x) / 2;
        bb3d[6] = bb3d[4];
    }
    if(temp1 < temp2)
    {
        bb3d[0] = (object3d.eightCorners[0].x + object3d.eightCorners[2].x) / 2;
        bb3d[2] = bb3d[0];
        bb3d[4] = (object3d.eightCorners[4].x + object3d.eightCorners[6].x) / 2;
        bb3d[6] = bb3d[4];
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
        //cout << "bb3d  " << i << "  " << bb3d[i] << endl;
    }

    // label position
    Eigen::Vector4d bb_label_sys(object3d.eightCorners[1].x, object3d.eightCorners[1].y, object3d.eightCorners[1].z + 0.1, 1);
    // Convert to world coordinate system, label position
    transform_label(0, 3) = bb_label_sys(0);
    transform_label(1, 3) = bb_label_sys(1);
    transform_label(2, 3) = bb_label_sys(2);

    Eigen::Affine3d a3f_transform_label(transform_label.cast<double>());
    object3d.coordinate_system_t_label = a3f_transform_label;
}

// new method to calculate the 3DBB
void ObjectsBoundingBoxes::boundingBoxEstimation(autosense::ObjectPtr singleObject, Object3d& object3d)
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


/*
// Align one axis of MF to the y axis of RGB-D camera coordinate system
Eigen::Matrix3d ObjectsBoundingBoxes::ManhattanFrameAlign(Eigen::Matrix3d rot_MF)
{
    Eigen::Matrix3d rot_MF_Aligned = Eigen::Matrix3d::Identity();

    Eigen::Vector3d vector_i(0, 0, 0), vector_j(0, 0, 0), vector_k(0, 0, 0);
    Eigen::Vector3d vector_i_new(0, 0, 0), vector_j_new(0, 1.0, 0), vector_k_new(0, 0, 0);
    Eigen::Vector3d j(0, 1.0, 0);

    vector_i = rot_MF.col(0);
    vector_j = rot_MF.col(1);
    vector_k = rot_MF.col(2);

    double m = 0.0, n = 0.0, p = 0.0, min_value = 0.0;

    m = fabs(j.dot(vector_i));
    n = fabs(j.dot(vector_j));
    p = fabs(j.dot(vector_k));

    min_value = m;
    if(n < min_value){
        min_value = n;
    }
    else if(p < min_value){
        min_value = p;
    }

    if(min_value == m){
        vector_i_new = j.cross(vector_i);
    }
    else if(min_value == n){
        vector_i_new = j.cross(vector_j);
    }
    else if(min_value == p){
        vector_i_new = j.cross(vector_k);
    }

    vector_k_new = j.cross(vector_i_new);

    rot_MF_Aligned.col(0) = vector_i_new;
    rot_MF_Aligned.col(1) = vector_j_new;
    rot_MF_Aligned.col(2) = vector_k_new;
    //cout << "rot_MF_Aligned = \n" << rot_MF_Aligned << endl;

    return rot_MF_Aligned;
}
*/

// Align one axis of MF to the z axis of RGB-D camera coordinate system
Eigen::Matrix3d ObjectsBoundingBoxes::ManhattanFrameAlign(Eigen::Matrix3d rot_MF)
{
    Eigen::Matrix3d rot_MF_Aligned = Eigen::Matrix3d::Identity();

    Eigen::Vector3d vector_i(0, 0, 0), vector_j(0, 0, 0), vector_k(0, 0, 0);
    Eigen::Vector3d vector_i_new(0, 0, 0), vector_j_new(0, 0, 0), vector_k_new(0, 0, 1);
    Eigen::Vector3d k(0, 0, 1);

    vector_i = rot_MF.col(0);
    vector_j = rot_MF.col(1);
    vector_k = rot_MF.col(2);

    double m = 0.0, n = 0.0, p = 0.0, min_value = 0.0;

    m = fabs(k.dot(vector_i));
    n = fabs(k.dot(vector_j));
    p = fabs(k.dot(vector_k));

    min_value = m;
    if(n < min_value){
        min_value = n;
    }
    else if(p < min_value){
        min_value = p;
    }

    if(min_value == m){
        vector_i_new = k.cross(vector_i);
    }
    else if(min_value == n){
        vector_i_new = k.cross(vector_j);
    }
    else if(min_value == p){
        vector_i_new = k.cross(vector_k);
    }

    vector_j_new = k.cross(vector_i_new);

    rot_MF_Aligned.col(0) = vector_i_new;
    rot_MF_Aligned.col(1) = vector_j_new;
    rot_MF_Aligned.col(2) = vector_k_new;
    cout << "rot_MF_Aligned = \n" << rot_MF_Aligned << endl;

    return rot_MF_Aligned;
}

//// 点云分割聚类 点云团反向投�?? �?? 2d检测框 相似�??
//// 获取每个点云团的类别信息，以�??3d边框信息，返�?? cluster 数组信息
//void ObjectsBoundingBoxes::extract(std::vector<Object>& objects,
//                        PointCloudT::Ptr point_ptr,    // 有序点云=====
//                        std::vector<Cluster>& clusters)
//{
//    clusters.clear();// 数据库清�??======
//    // 1. 点云团分割，获取点云�??=================================
//    std::vector<pcl::PointIndices> cluster_indices; // 点云团索�?? 数组=====
//    PointCloudT::Ptr cloud_segment(new PointCloudT);// 保存的点云，有序点云====

//    segment(point_ptr, cloud_segment, cluster_indices);

//    //std::cout << "cluster  size  " << cluster_indices.size() << std::endl;

//    std::vector<Object3d> object3ds;// 3d点云�??

//    for ( std::vector<pcl::PointIndices>::iterator indices_it = cluster_indices.begin();
//          indices_it != cluster_indices.end(); ++indices_it )
//    {
//        try
//        {
//            pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
//            copyPointCloud(cloud_segment, indices_it->indices, seg);// 由点�?? indices 除以 / 取余 图像宽度 得到像素坐标
//            // 求取点云范围=====
//            cv::Rect_<float> obj_roi;
//            bool ok = getProjectedROI(seg, obj_roi); // 获取点云反投影的 2d�??====
//            if(!ok) continue;

//            //std::cout << "3dobject_roi: " << obj_roi.x     << " "
//            //                            << obj_roi.y     << " "
//            //                            << obj_roi.width << " "
//            //                            << obj_roi.height << std::endl;

//            // lambda 函数
//            auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.x < r.x; };
//            auto minmax_x = std::minmax_element(seg->begin(), seg->end(), cmp_x);// 点云x最�??

//            auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.y < r.y; };
//            auto minmax_y = std::minmax_element(seg->begin(), seg->end(), cmp_y);// 点云y最�??

//            auto cmp_z = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.z < r.z; };
//            auto minmax_z = std::minmax_element(seg->begin(), seg->end(), cmp_z);// 点云z最�??

//            // 边框中心�??=====
//            auto sum_x = [](double sum_x, PointXYZPixel const& l){return sum_x + l.x;};
//            auto sumx = std::accumulate(seg->begin(), seg->end(), 0.0, sum_x);
//            double mean_x =  sumx / seg->size(); //均�?

//            auto sum_y = [](double sum_y, PointXYZPixel const& l){return sum_y + l.y;};
//            auto sumy = std::accumulate(seg->begin(), seg->end(), 0.0, sum_y);
//            double mean_y =  sumy / seg->size(); //均�?

//            auto sum_z = [](double sum_z, PointXYZPixel const& l){return sum_z + l.z;};
//            auto sumz = std::accumulate(seg->begin(), seg->end(), 0.0, sum_z);
//            double mean_z =  sumz / seg->size(); //均�?

//            Object3d object3d;
//            object3d.rect     = obj_roi; // 3d点云对应�?? 2d图像边框====
//            object3d.minPt    = Eigen::Vector3d(minmax_x.first->x, minmax_y.first->y, minmax_z.first->z);
//            object3d.maxPt    = Eigen::Vector3d(minmax_x.second->x,minmax_y.second->y,minmax_z.second->z);
//            object3d.centroid = Eigen::Vector3d(mean_x, mean_y, mean_z); // 均值中�??
//            // 3d边框
//            object3d.sizePt   = Eigen::Vector3d(object3d.maxPt[0]-object3d.minPt[0],
//                    object3d.maxPt[1]-object3d.minPt[1],
//                    object3d.maxPt[2]-object3d.minPt[2]);
//            // 3d边框中心===
//            object3d.boxCenter= Eigen::Vector3d(object3d.minPt[0]+object3d.sizePt[0]/2.0,
//                    object3d.minPt[1]+object3d.sizePt[1]/2.0,
//                    object3d.minPt[2]+object3d.sizePt[2]/2.0);

//            object3ds.push_back(object3d);
//        }
//        catch (std::exception& e)
//        {
//            std::cout << e.what() << std::endl;// 错误信息
//        }
//    }

//    // std::cout << "object3ds size  " << object3ds.size() << std::endl;
//    // 没有 分割后的目标=== 参数===
//    // 2. �??2d检测框融合，生成带有标签的3d目标物体
//    findMaxIntersectionRelationships(objects, object3ds, clusters);
//}


//// 2d物体 �?? 3d物体 关系 ==============================================
//// 遍历 每一�??2d物体
////     遍历   每一�??3d物体
////        计算 2d物体边框 �??3d物体投影2d边框的相似度  两边框的匹配相似�?? match = IOU * distance /  AvgSize
////        记录�?? �??2d边框最相似�?? 3d物体id
//void ObjectsBoundingBoxes::findMaxIntersectionRelationships(std::vector<Object>& objects,   // 2d 目标检测框
//                                                 std::vector<Object3d>& object3d,// 3d点云�?? �??2d投影�??
//                                                 std::vector<Cluster>& clusters) // 3d点云�?? 带类别信�??
//{
//    for (std::vector<Object>::iterator obj2d_it = objects.begin();
//         obj2d_it != objects.end(); ++obj2d_it)// 每一�??2d物体
//    {
//        std::vector<Object3d>::iterator max_it = object3d.begin();// 3d物体id
//        double max = 0;
//        cv::Rect_<float>  rect2d = obj2d_it->rect; // 2d边框


//        //std::cout << "2dobject_roi: " << rect2d.x     << " "
//        //                              << rect2d.y     << " "
//        //                              << rect2d.width << " "
//        //                              << rect2d.height << std::endl;



//        for (std::vector<Object3d>::iterator it = max_it;
//             it != object3d.end(); ++it)// 每一�??3d物体
//        {
//            cv::Rect_<float> rect3d = it->rect;    // 3d物体 roi 3d点云投影�??2d平面后的2d边框
//            double area = getMatch(rect2d, rect3d);// 两边�?? �?? 匹配相似�??   IOU * distance /  AvgSize

//            // std::cout << "match: " << area << std::endl;

//            if (area < max)
//            {
//                continue;
//            }

//            max = area;
//            max_it = it;  // 为每一�?? 2d边框 寻找 一�?? 匹配度最高的 3d物体===========
//        }

//        if (max <= 0)
//        {
//            std::cout << "Cannot find correlated 3D object " << std::endl;
//            continue;
//        }

//        // 3d对象=========

//        Cluster cluster;
//        /*
//    cluster.object.rect = obj2d_it->rect;      // 2d对象
//    cluster.object.prob = obj2d_it->prob;// 2d对象
//    cluster.object.object_name = obj2d_it->object_name;// 2d对象

//    cluster.centroid  = max_it->centroid;// 点云中心
//    cluster.minPt     = max_it->minPt;   // 最小的x值，y值，z�??
//    cluster.maxPt     = max_it->maxPt;   // 最大的x值，y值，z�??
//    cluster.sizePt    = max_it->sizePt;// 长宽�??
//    cluster.boxCenter = max_it->boxCenter;// 包围盒中心点
//*/


//        cluster.object_name = obj2d_it->object_name;// 名字
//        cluster.class_id    = obj2d_it->class_id;    // 类别id
//        cluster.prob        = obj2d_it->prob;// 置信�??

//        cluster.size        = max_it->sizePt;// 尺寸
//        cluster.centroid    = max_it->centroid; // 中心�??  // max_it->boxCenter; // 包围盒中心点

//        clusters.push_back(cluster);

//        object3d.erase(max_it);// 删除已经匹配�??3d点云物体
//    }
//}



//  2d 像素点集 获取 对应的roi边框 min_x, min_y, max_x-min_x, max_y-min_y================
//  �?? 3d+2d点云团里获取 2droi边框       ======
// PointXYZPixel = 3d�?? + 2d 像素点坐�??======
// 计算 3d物体点云集合 投影�?? 相机平面上的 2d�?? ROI=====
bool ObjectsBoundingBoxes::getProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr& point_cloud,// 新类型点�?? x,y,z,px,py
                                           cv::Rect_<float> & roi)// 3d点云�?? 对应像素2d边框
{
    // lambda 函数
    auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_x < r.pixel_x; };

    auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);// 点云对应像素点坐标， pixel_x的最大最小�?

    roi.x = minmax_x.first->pixel_x;// x_offset 框的左上角点, x坐标最小�?
    auto max_x = minmax_x.second->pixel_x;// x坐标最大�?
    if(roi.x >= 0 && max_x >= roi.x)
    {
        roi.width = max_x - roi.x;// 2d�?? 宽度   max_x - min_x

        auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_y < r.pixel_y; };
        auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);// 点云对应像素点坐标， pixel_y的最大最小�?
        roi.y = minmax_y.first->pixel_y; // y_offset 框的左上角点, y坐标最小�?
        auto max_y = minmax_y.second->pixel_y;// y坐标最大�?
        if(roi.y >= 0 && max_y >= roi.y)
        {
            roi.height = max_y - roi.y;       //  2d�?? 高度   max_y - min_x
            return true;
        }
        return false;
    }
    return false;
}

// 两边�?? �?? 匹配相似�??   IOU * distance /  AvgSize===============
double ObjectsBoundingBoxes::getMatch(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2)
{
    cv::Rect2i ir1(r1), ir2(r2);
    /* calculate center of rectangle #1  边框中心�?? */
    cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));// 边框 中心�??1
    /* calculate center of rectangle #2  边框中心�?? */
    cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));// 边框 中心�??2

    double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();// opencv �?? 矩形支持 &并集 运算�??
    /* calculate the overlap rate*/
    double overlap = a0 / (a1 + a2 - a0);// IOU 交并�??
    /* calculate the deviation between centers #1 and #2*/
    double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));// 边框中心�?? 距离 距离近相�??
    /* calculate the length of diagonal for the rectangle in average size*/
    // 使用 平均尺寸  进行匹配�?? 加权 =====================================================
    double len_diag = sqrt(powf(((ir1.width + ir2.width) >> 1), 2) + powf(((ir1.height + ir2.height) >> 1), 2));

    /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/

    return overlap * len_diag / deviate;
}


// 点云分割相关算法==============================
void ObjectsBoundingBoxes::segment(const PointCloud7D::ConstPtr& cloud, // 输入点云
                                   PointCloud7D::Ptr& cloud_segment,    // 保存的点�??
                                   std::vector<pcl::PointIndices>& cluster_indices)// 点云团索�?? 数组
{

    std::cout << "Total original point size = " << cloud->size() << std::endl;

    pcl::copyPointCloud(*cloud, *cloud_segment);  // 保存的点云，有序点云====

    applyConfig();// 0. 实时配置分割器参�??=====

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);//法线
    estimateNormal(cloud, normal_cloud);// 1. 估计法线=======

    std::vector<pcl::PlanarRegion<Point7D>, Eigen::aligned_allocator<pcl::PlanarRegion<Point7D> > > regions;// 平面区域?
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);// 点云标签，属于那个平�??====
    std::vector<pcl::PointIndices> label_indices;// 平面点云团索�??===

    segmentPlanes(cloud, normal_cloud, regions, labels, label_indices);// 2. 分割平面============

    std::cout << "find plane : " << label_indices.size() << std::endl; // 平面数量 ====

    segmentObjects(cloud, regions, labels, label_indices, cluster_indices);// 3. 分割目标对象====

}

// 1. 估计法线=======
void ObjectsBoundingBoxes::estimateNormal(const PointCloud7D::ConstPtr& cloud,
                                          pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud)
{
    normal_estimation_.setInputCloud(cloud);// 输入点云
    normal_estimation_.compute(*normal_cloud);// 计算点云法线

    // 设置边缘平面分割系数??
    float* distance_map = normal_estimation_.getDistanceMap();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<Point7D, pcl::Normal> > eapc =
            boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<Point7D, pcl::Normal> >(edge_aware_comparator_);
    eapc->setDistanceMap(distance_map);
    eapc->setDistanceThreshold(0.01f, false);

}

// 2. 分割平面===============================
void ObjectsBoundingBoxes::segmentPlanes(
        const PointCloud7D::ConstPtr& cloud,    // 输入点云
        const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,// 点云法线
        std::vector<pcl::PlanarRegion<Point7D>, Eigen::aligned_allocator<pcl::PlanarRegion<Point7D> > >& regions,// 平面区域
        pcl::PointCloud<pcl::Label>::Ptr labels,       // 标签
        std::vector<pcl::PointIndices>& label_indices) // 索引
{

    double mps_start = pcl::getTime ();
    std::vector<pcl::ModelCoefficients> model_coefficients;// 平面模型系数
    std::vector<pcl::PointIndices> inlier_indices;    // 平面上的�?? 的索�??
    std::vector<pcl::PointIndices> boundary_indices;  // 其他索引

    plane_segmentation_.setInputNormals(normal_cloud);// 输入点云法线
    plane_segmentation_.setInputCloud(cloud);         // 输入点云
    // 执行平面分割====
    plane_segmentation_.segmentAndRefine(regions,
                                         model_coefficients,
                                         inlier_indices,
                                         labels,
                                         label_indices,
                                         boundary_indices);
    // mps.segment (regions); // not refinement ====
    double mps_end = pcl::getTime ();
    std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;
}

// 3. 分割目标对象=============================
void ObjectsBoundingBoxes::segmentObjects(
        const PointCloud7D::ConstPtr& cloud,//输入点云
        std::vector<pcl::PlanarRegion<Point7D>, Eigen::aligned_allocator<pcl::PlanarRegion<Point7D> > >& regions,//平面区域
        pcl::PointCloud<pcl::Label>::Ptr labels, // �?? 属于那个平面�?? 标签
        std::vector<pcl::PointIndices>& label_indices,  // 平面 标签索引
        std::vector<pcl::PointIndices>& cluster_indices)// 索引
{

    // 更新平面对象=========================
    std::vector<bool> plane_labels;// 平面�??? 数量过少，不认为是平�??=====
    plane_labels.resize(label_indices.size(), false);// 默认设置�?? �??
    for (size_t i = 0; i < label_indices.size(); i++)
    {
        // 平面点最少数�??===
        if (label_indices[i].indices.size() > mPlane_minimum_points)
        {
            plane_labels[i] = true;// 该点湍数量大于阈值，设置�?? 平面点与�??====
        }
    }

    // 欧式距离聚类分割
    euclidean_cluster_comparator_->setInputCloud(cloud);// 点云
    euclidean_cluster_comparator_->setLabels(labels);   // �?? 属于那个平面�?? 标签
    euclidean_cluster_comparator_->setExcludeLabels(plane_labels);// 平面是否为平面的标签

    pcl::PointCloud<pcl::Label> euclidean_labels;// 聚类分割点云�??

    // 点云聚类分割�??===========
    pcl::OrganizedConnectedComponentSegmentation<Point7D, pcl::Label>
            euclidean_segmentation(euclidean_cluster_comparator_);

    euclidean_segmentation.setInputCloud(cloud);// 输入点云
    euclidean_segmentation.segment(euclidean_labels, cluster_indices);// 点云团索�??=====

    // 匿名函数== 判断点云团是否足够大===
    auto func = [this](pcl::PointIndices indices) { return indices.indices.size() < this->mObject_minimum_points; };
    // 删除过小的点云团
    cluster_indices.erase(std::remove_if(cluster_indices.begin(), cluster_indices.end(), func), cluster_indices.end());

    PCL_INFO ("Got %d euclidean clusters!\n", cluster_indices.size ());

}

// 0. 实时配置分割器参�??=================================
void ObjectsBoundingBoxes::applyConfig()
{

    // 法线估计器参�??=====
    //normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
    normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
    normal_estimation_.setMaxDepthChangeFactor(mNormal_max_depth_change);// 法线计算参数 深度变化阈�?(�??)
    normal_estimation_.setNormalSmoothingSize(mNormal_smooth_size);      // 平滑因子，法�?? 区域面积 平滑参数大小

    // 欧式距离聚类分割器参�??=====
    // 欧式距离聚类分割参数(�??) 超过认为�?? 不同的点云团 0.02 0.001 0.1
    euclidean_cluster_comparator_->setDistanceThreshold(mEuclidean_distance_threshold, false);


    // 平面分割器参�??=====
    plane_segmentation_.setMinInliers(mMin_plane_inliers);// 采样一致性算法，最少内点数�??
    plane_segmentation_.setAngularThreshold(pcl::deg2rad(mPlanNormal_angle_threshold));// 平面相邻点法线角度差阈�?
    plane_segmentation_.setDistanceThreshold(normal_distance_threshold);          // 法线方向的垂直距离阈�??

    if (mPlanComparator == kPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(plane_comparator_);// 平面参数
    }
    else if (mPlanComparator == kEuclideanPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(euclidean_comparator_);// 欧式距离分割 平面
    }
    else if (mPlanComparator == kRGBPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(rgb_comparator_);// 颜色分割平面
    }
    else if (mPlanComparator == kEdgeAwarePlaneComaprator)
    {
        plane_segmentation_.setComparator(edge_aware_comparator_);// 边缘分割平面
    }
}

// XYZRGB�??+颜色 点云  拷贝�?? XYZ+像素点坐�?? 点云
void ObjectsBoundingBoxes::copyPointCloud(const PointCloud7D::ConstPtr& original,
                                          const std::vector<int>& indices,
                                          pcl::PointCloud<PointXYZPixel>::Ptr& dest)
{
    pcl::copyPointCloud(*original, indices, *dest);// 拷贝 3d点坐�??
    uint32_t width = original->width;              // 有序点云，相当于图像宽度
    for (uint32_t i = 0; i < indices.size(); i++)  // 子点云序�??
    {
        dest->points[i].pixel_x = indices[i] % width;// 列坐�??
        dest->points[i].pixel_y = indices[i] / width;// 行坐�??
    }
}

// 计时
long ObjectsBoundingBoxes::_getTimeUsec()
{
    struct timeval t;
    gettimeofday(&t,0);
    return (long)((long)t.tv_sec*1000*1000 + t.tv_usec);
}
