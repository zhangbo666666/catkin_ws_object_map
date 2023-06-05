/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 语义目标数据库=======
 * 添加，删除，融合目标数据
 */

#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

#include "System.h"

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <mutex>

//#include "mclmcrrt.h"
//#include "mclmcr.h"
//#include "mclcppclass.h"
//#include "matrix.h"
//#include "libcalculateBb3dOverlap.h"

typedef pcl::PointXYZRGBA Point7D;// 点类型 xyzrgba 点+颜色+透明度
typedef pcl::PointCloud<Point7D> PointCloud7D;// 点云类型

// 目标语义信息
typedef struct Cluster
{
    Eigen::Vector3d size;    // 3d框尺寸
    Eigen::Vector3d centroid;// 点云中心点
    float prob;              // 置信度
    std::string object_name; // 物体类别名
    int class_id;            // 对应类别id
    int object_id;           // 物体编号
    bool flag_new_object; // true: add the cluster, false: update the object
    int object_update_num = 10000; // if flag_new_object is false, update the cluster in the database in the object_update_num position
    bool operator == (const std::string &x);

    Eigen::Vector3d minPt; // 所有点中最小的x值，y值，z值,original point in world coordinate system, not in MF
    Eigen::Vector3d maxPt; // 所有点中最大的x值，y值，z值
    Eigen::Vector3d boxCenter; // 包围盒中心点

    int point_color_r;
    int point_color_g;
    int point_color_b;

    // save bounding box eight corners
    // Eight corners sequence: 1-2-3-4, 5-6-7-8
    //    5-------6
    //   /|      /|
    //  8-|-----7 |
    //  | 1-----|-2
    //  |/      |/
    //  4-------3
    vector<pcl::PointXYZ> eightCorners;

    // object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
    double bb3d[10];

    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Affine3d coordinate_system_t; // translation + rotation

    Eigen::Affine3d coordinate_system_t_label; // label position

    // object point cloud extracted from a 2D bounding box, merge different clouds of the same object
    PointCloud7D::Ptr object_point_cloud_before_seg;
    PointCloud7D::Ptr object_point_cloud_after_seg; // object point cloud after LCCP segmentation

    bool first_keyframe_with_objects; // the first keyframe with the detected object(prob > threshold)

} Cluster;

typedef struct Bb3dIntersection{
    double volume1;
    double volume2;
    double intersection;
    double union_bb;
    double IoU3d;
    double volumeRatioBB1;
    double volumeRatioBB2;
}Bb3dIntersection;

class ObjectDatabase
{
public:

    ObjectDatabase();
    ~ObjectDatabase();

    void addObject(Cluster& cluster);

    void objectDatabaseMerge();
    int databaseDeleteSmallObjects();
    int databaseDeleteIntersectionObjects();

    void objectPointCloudMerge(Cluster& cluster, bool& isObjectDatabaseUpdate); // merge objects with multi-views

    // calculate intersection of two bounding boxes
    // object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
    Bb3dIntersection calculateIntersectionBB3D(double* bb1, double* bb2);

    cv::Scalar  getObjectColor(int class_id); // 定义的物体颜色
    float getObjectSize(int class_id);        // 定义的物体尺寸

    std::vector<Cluster>  getObjectByName(std::string objectName);// 返回数据库中 同名字的物体数据

    std::vector<Cluster> mClusters;   // 语义点云目标数组

    // Get the object clusters
    std::vector<Cluster> getObjectClusters();

    int mDeleteObjectNum; // delete small objects
    int mUpdateObjectNum; // update the object with the same label
    int mDeleteIntersectionNum; // delete the small object with intersection
    std::vector<string> mvCocoNames;

protected:

    std::vector<cv::Scalar> mvColors;// 每种物体的颜色
    std::vector<float>      mvDistanceTh; // 每种物体的centroid distance threshold
    std::vector<Eigen::Vector3d>      mvSizes; // 每种物体的大小

    int DataBaseSize;

    //---------
    std::mutex mMutexClusters;
    float mVolumeThMin; // threshold for object volume
    float mVolumeThMax; // threshold for object volume
    float mMinEdgeTh; // threshold for object minimum edge
    float mMinIntersectionTh; // threshold for minimum intersection between two object bounding boxes

    // calculate the 3D bounding box of two object bounding boxes
    //    mwArray mwABoundingBox1; // object bounding box
    //    mwArray mwABoundingBox2;
    //    mwArray mwAVolume1; // volume of the bounding box
    //    mwArray mwAVolume2;
    //    mwArray mwAIntersection; // intersection volume of two bounding boxes
    //    mwArray mwAUnion; // Volume1 + Volume2 - Intersection
    //    mwArray mwAIoU3d; // 3d IoU

};

#endif // OBJECTDATABASE_H
