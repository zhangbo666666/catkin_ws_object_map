/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 语义数据�?===========
 * 添加，删除，融合目标数据
 */

#include "ObjectDatabase.h"
#include <pcl/common/centroid.h>

#include <stdexcept>
#include <PolygonIntersection.hpp>

// 重写 Cluster的等号操作符，方便按名字查找
bool Cluster::operator ==(const std::string &x){
    return(this->object_name == x);
} 

// 类构造函�?=======
ObjectDatabase::ObjectDatabase()
{
//    cout << "Start to initialize CulateBb3dOverlap lib ......" << endl;
    // initialize lib
//    if(!mclInitializeApplication(NULL,0)){
//        cout<<"CulateBb3dOverlap initial error"<<endl;
        //return 1;
//    }
//    cout<<"Succeed to initialize......"<<endl;

//    libcalculateBb3dOverlapInitialize();
//    cout<<"CulateBb3dOverlap lib Initialized ......"<<endl;

    cout << "Skip initializing CulateBb3dOverlap lib ......" << endl;

    DataBaseSize = 0;
    mClusters.clear();

    //Check settings file
    cv::FileStorage fsSettings("/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/orb_slam2/config/ASUS.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << "orb_slam_2_ros_yolo/orb_slam2/config/ASUS.yaml" << endl;
        exit(-1);
    }

    // for point cloud resolution
    mVolumeThMin = fsSettings["ObjectDatabase.mVolumeThMin"];
    mVolumeThMax = fsSettings["ObjectDatabase.mVolumeThMax"];

    mMinEdgeTh = fsSettings["ObjectDatabase.mMinEdgeTh"];
    mMinIntersectionTh = fsSettings["ObjectDatabase.mMinIntersectionTh"];

    cout << "mVolumeThMin = " << mVolumeThMin << endl;
    cout << "mVolumeThMax = " << mVolumeThMax << endl;

    cout << "mMinEdgeTh = " << mMinEdgeTh << endl;

    // 不同颜色对应不同物体 ================================================
    for (int i = 0; i < 81; i++) // 带有背景
    {  // COCO数据�? 80类物�?=======
        //mvColors.push_back(cv::Scalar(i*10 + 40, i*10 + 40, i*10 + 40));
        mvColors.push_back(cv::Scalar(i*3 + 15, i*3 + 15, i*3 + 15));

        //mvColors.push_back(cv::Scalar(255,0,0));
        // VOC 数据�? 20类物�?
        // "background","aeroplane", "bicycle", "bird", "boat",
        // "bottle", "bus", "car", "cat", "chair",
        // "cow", "diningtable", "dog", "horse","motorbike", "person",
        // "pottedplant","sheep", "sofa", "train", "tvmonitor"

        // COCO数据�? 80类物�?
    }
    mvColors[1] = cv::Scalar(0,0,255);  // �? 红色
    mvColors[40] = cv::Scalar(255,0,255); // 瓶子 粉色    rgb
    mvColors[42] = cv::Scalar(255,0,255); // cup
    mvColors[57] = cv::Scalar(255,0,0);   // 椅子 蓝色
    mvColors[58] = cv::Scalar(255,255,0);   // sofa Yellow
    mvColors[60] = cv::Scalar(255,105,180);  // bed HotPink
    mvColors[61] = cv::Scalar(255,105,180);  // dining table
    mvColors[63] = cv::Scalar(0,255,0);  // 显示�? 绿色
    mvColors[64] = cv::Scalar(155,48,255);  // laptop Purple1
    mvColors[65] = cv::Scalar(255,64,64);  // mouse Brown1
    mvColors[67] = cv::Scalar(255,0,255);  // keyboard Magenta
    mvColors[72] = cv::Scalar(173,255,47);  // sink GreenYellow
    mvColors[73] = cv::Scalar(0,245,255);  // refrigerator Turquoise1
    mvColors[74] = cv::Scalar(255,106,106);  // book IndianRed1

    // 物体尺寸大小 =========================================================
    Eigen::Vector3d size(0.3, 0.3, 0.3);

    for (int i = 0; i < 81; i++)
    { // coco数据�? 80类物�?=======
        mvDistanceTh.push_back(0.50);
        mvSizes.push_back(size);
    }

/*    mvDistanceTh[1] = 0.45;    // person
    mvDistanceTh[40] = 0.20;   // bottle
    mvDistanceTh[42] = 0.20;   // cup
    mvDistanceTh[56] = 0.50;    // chair
    mvDistanceTh[57] = 0.70;    // sofa
    mvDistanceTh[59] = 1.60;   // bed
    mvDistanceTh[61] = 0.50;  // table
    mvDistanceTh[62] = 0.45;  // television
    mvDistanceTh[63] = 0.50;  // laptop
    mvDistanceTh[64] = 0.20;  // mouse
    mvDistanceTh[66] = 0.50;  // keyboard
    mvDistanceTh[71] = 0.55;  // sink
    mvDistanceTh[72] = 0.50;  // refrigerator
    mvDistanceTh[73] = 0.10;  // book*/

    mvDistanceTh[1] = 0.45;    // person
    mvDistanceTh[40] = 0.20;   // bottle
    mvDistanceTh[42] = 0.20;   // cup
    mvDistanceTh[57] = 0.50;    // chair
    mvDistanceTh[58] = 0.70;    // sofa
    mvDistanceTh[60] = 1.60;   // bed
    mvDistanceTh[61] = 0.50;  // table
    mvDistanceTh[63] = 0.45;  // television
    mvDistanceTh[64] = 0.50;  // laptop
    mvDistanceTh[65] = 0.20;  // mouse
    mvDistanceTh[67] = 0.50;  // keyboard
    mvDistanceTh[72] = 0.55;  // sink
    mvDistanceTh[73] = 0.50;  // refrigerator
    mvDistanceTh[74] = 0.10;  // book

    // Length, Width, Hight
    mvSizes[1] << 0.4, 0.4, 1.7;    // person`
    mvSizes[40] << 0.10, 0.10, 0.20;   // bottle
    mvSizes[42] << 0.10, 0.10, 0.20;   // cup
    mvSizes[57] << 0.50, 0.50, 0.8;    // chair
    mvSizes[58] << 0.6, 0.6, 0.6;    // sofa
    mvSizes[60] << 1.50, 1.80, 0.50;   // bed
    mvSizes[61] << 0.80, 0.50, 0.50;  // table
    mvSizes[63] << 0.45, 0.25, 0.45;  // television
    mvSizes[64] << 0.50, 0.35, 0.40;  // laptop
    mvSizes[65] << 0.10, 0.10, 0.10;  // mouse
    mvSizes[67] << 0.50, 0.20, 0.05;  // keyboard
    mvSizes[72] << 0.55, 0.55, 0.50;  // sink
    mvSizes[73] << 0.75, 0.75, 1.50;  // refrigerator
    mvSizes[74] << 0.20, 0.10, 0.05;  // book
    // obtain coco names
    fstream file;
    string temp;
    //string filename = "../../../../orb_slam_2_ros_mask_rcnn/coco_names_mask_rcnn.txt";
    string filename = "/home/zb/catkin_ws_object_map/src/orb_slam_2_ros_yolo/coco_names.txt";

    file.open(filename, ios::in);
    mvCocoNames.push_back("BG"); // background
    for(int i = 0; i < 80; i++)
    {
        getline(file, temp);
        mvCocoNames.push_back(temp);
        cout << mvCocoNames[i+1] << endl;
    }
    file.close();

    mDeleteObjectNum = 1; // delete small objects
    mUpdateObjectNum = 1; // update the object with the same label during fusion
    mDeleteIntersectionNum = 1; // delete the small object with intersection
}
// 类析构函�?===============
ObjectDatabase::~ObjectDatabase()
{
    //    delete mwABoundingBox1;
    //    delete mwABoundingBox2;
    //    delete mwAVolume1;
    //    delete mwAVolume2;
    //    delete mwAIntersection;
    //    delete mwAUnion;
    //    delete mwAIoU3d;

    // 后面是一些终止调用的程序
    // terminate the lib
//    libcalculateBb3dOverlapTerminate();
    // terminate MCR
//    mclTerminateApplication();
}
// 返回 定义的物体颜�?====================
cv::Scalar  ObjectDatabase::getObjectColor(int class_id)
{
    return mvColors[class_id];
}
// 返回 定义的物体尺�?==================
float ObjectDatabase::getObjectSize(int class_id)
{
    return mvDistanceTh[class_id];
}       
// 返回数据库中 同名字的物体数据=========
std::vector<Cluster>  ObjectDatabase::getObjectByName(std::string objectName)
{
    // 按名字查物体是否在数据库
    std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
    std::vector<Cluster>::iterator it_end = mClusters.end();
    std::vector<Cluster> sameName;// 同名字的物体
    while(true)
    {
        iter = find(++iter, it_end, objectName);// 按照名字查找
        if (iter != it_end )// 找到一个，存放起来
            sameName.push_back(*iter);
        else//已经找不到了
            break;
    }
    return sameName; // 这里可以传递引用过来，减少赋值时�?===
}

/*
void ObjectDatabase::addObject(Cluster& cluster)
{
    unique_lock<mutex> lock(mMutexClusters);

    // 1. 查看总数�?,数据库为空直接加�?
    if(!mClusters.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mClusters.push_back(cluster);
        return;
    }
    else
    {
        // 2. 数据库内已经存在物体了，查找新物体是否在数据库内已经存在
        std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
        std::vector<Cluster>::iterator it_end = mClusters.end();
        std::vector<std::vector<Cluster>::iterator> likely_obj;// 同名字的物体的迭代器
        while(true)
        {
            iter = find(++iter, it_end, cluster.object_name);// 按照名字查找
            if (iter != it_end )// 找到一个，存放起来
                likely_obj.push_back(iter);
            else//已经找不到了
                break;
        }

        // 3. 如果没找到，则直接添�? 进数据库
        std::vector<Cluster>::iterator best_close;// 最近的索引
        float center_distance = 100;// 对应的距�?
        if(!likely_obj.size())
        {
            DataBaseSize++;
            cluster.object_id = DataBaseSize;
            mClusters.push_back(cluster);
            return;
        }
        else//找到多个和数据库里同名字的物�?
        {
            // 4. 遍例每一个同名字的物体，找到中心点最近的一�?
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;
                //Eigen::Vector3d dis_vec = cluster.centroid - temp_cluster.centroid;// 中心点连接向�?
                Eigen::Vector3d dis_vec = cluster.boxCenter - temp_cluster.boxCenter;// 中心点连接向�?

                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // 最短的距离
                    best_close      = temp_iter;// 对应的索�?
                }
            }
            // 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息// 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息
            // if(center_distance < mvSizes[cluster.class_id])
            if(center_distance < mvSizes[0])
            {
                best_close->prob    = (best_close->prob + cluster.prob)/2.0; // 综合置信�?
                // best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // 中心平均
                best_close->boxCenter = (best_close->boxCenter + cluster.boxCenter)/2.0; // 中心平均

                best_close->size     = (best_close->size + cluster.size)/2.0; // 中心尺寸
            }
            else
            {
                // 6. 如果距离超过物体尺寸则认为是不同位置的同一种物体，直接放入数据�?
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mClusters.push_back(cluster);
            }
        }
    }

    // 数据库大小限�?======，超过一定大小，删除�? 置信度低的目�?====
    return;
}
*/

// keyframe_num = 1; // the detected objects in the first keyframe are all added in the database,
// the late keyframes are merged.
void ObjectDatabase::addObject(Cluster& cluster)
{
    unique_lock<mutex> lock(mMutexClusters);

    std::vector<string>::iterator iter_   = mvCocoNames.begin() - 1;
    std::vector<string>::iterator it_end_ = mvCocoNames.end();
    iter_ = std::find(++iter_, it_end_, cluster.object_name);// 按照名字查找
    cluster.class_id = std::distance(mvCocoNames.begin(), iter_);

    if(cluster.flag_new_object) // new object
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mClusters.push_back(cluster); // add cluster
        return;
    }
    else // old object, update
    {
        cluster.object_id = mClusters[cluster.object_update_num].object_id;
        cluster.prob = max(mClusters[cluster.object_update_num].prob, cluster.prob); // max prob
        mClusters[cluster.object_update_num] = cluster; // update the cluster

        cout << "Step2.3: Total object updated number: " << mUpdateObjectNum << "\n" << endl;
        mUpdateObjectNum++;
    }
    return;
}

// Merge object database when running the dataset
/*
// keyframe_num = 1; // the detected objects in the first keyframe are all added in the database,
// the late keyframes are merged.
void ObjectDatabase::addObject(Cluster& cluster)
{
    unique_lock<mutex> lock(mMutexClusters);

    Eigen::Vector3d objectSize(cluster.size(0), cluster.size(1), cluster.size(2));
    Eigen::Vector3d objectSizePriority(mvSizes[cluster.class_id]);

    double vol1, vol2;
    vol1 = objectSize[0] * objectSize[1] * objectSize[2];
    vol2 = objectSizePriority[0] * objectSizePriority[1] * objectSizePriority[2];

    double minEdge = min(min(objectSize[0], objectSize[1]), objectSize[2]);
    double minEdgePriority = min(min(objectSizePriority[0], objectSizePriority[1]), objectSizePriority[2]);

    // delete small objects

    cout << "vol1 = " << vol1 << endl;
    cout << "mVolumeTh * vol2 = " << mVolumeTh * vol2 << endl;
    cout << "minEdge = " << minEdge << endl;
    cout << "mMinEdgeTh * minEdgePriority = " << mMinEdgeTh * minEdgePriority << endl;

    cout << "----------0000-----------" << endl;

    // The judgement condition: volume and minimum edge
    if((vol1 < mVolumeTh * vol2) || (minEdge < mMinEdgeTh * minEdgePriority))
    {
        cout << "Object is too small, delete it, delete object number: " << mDeleteObjectNum << "\n" << endl;
        mDeleteObjectNum++;
        return;
    }
    else
    {
        cout << "----------00-----------" << endl;

        if(cluster.flag_new_object) // new object
        {
            cout << "----------0-----------" << endl;

            bool flagIntersection = false; // no intersection with all object bounding boxes
            vector<vector<Cluster>::iterator> likely_update_cluster; // store the cluster to update
            vector<double> volumeRatio;
            likely_update_cluster.clear();
            volumeRatio.clear();
            cout << "----------1-----------" << endl;

            vector<Cluster>::iterator iter;
            for(iter = mClusters.begin(); iter != mClusters.end(); iter++)
            {
                cout << "----------2-----------" << endl;

                cout << "cluster.object_name = " << cluster.object_name << endl;
                cout << "iter->object_name = " << iter->object_name << endl;

                for(int i = 0; i < 10; i++)
                {
                    cout << "cluster.bb3d = " << cluster.bb3d[i] << endl;
                }
                for(int i = 0; i < 10; i++)
                {
                    cout << "iter->bb3d = " << iter->bb3d[i] << endl;
                }
                cout << "----------3-----------" << endl;

                Bb3dIntersection temp1;
                temp1 = calculateIntersectionBB3D(cluster.bb3d, iter->bb3d);
                cout << "----------4-----------" << endl;

                double volume1, volume2, intersection, union_bb, IoU3d, volumeRatioBB1, volumeRatioBB2;
                volume1 = temp1.volume1;
                volume2 = temp1.volume2;
                intersection = temp1.intersection;
                union_bb = temp1.union_bb;
                IoU3d = temp1.IoU3d;
                volumeRatioBB1 = temp1.volumeRatioBB1;
                volumeRatioBB2 = temp1.volumeRatioBB2;

                cout << "volume1 = " << volume1 << endl;
                cout << "volume2 = " << volume2 << endl;
                cout << "intersection = " << intersection << endl;
                cout << "union = " << union_bb << endl;
                cout << "IoU3d = " << IoU3d << endl;
                cout << "volumeRatioBB1 = " << volumeRatioBB1 << endl;
                cout << "volumeRatioBB2 = " << volumeRatioBB2 << endl;

                if((volumeRatioBB1 < mMinIntersectionTh) && (volumeRatioBB2 < mMinIntersectionTh)) // add the new object bounding box
                {
                    continue;
                }
                else
                {
                    flagIntersection = true;
                    if(volumeRatioBB1 > volumeRatioBB2) // delete the new object bounding box
                    {
                        cout << "Delete intersection object number: " << mDeleteIntersectionNum << "\n" << endl;
                        mDeleteIntersectionNum++;
                        continue;
                    }
                    else
                    {
                        likely_update_cluster.push_back(iter);
                        volumeRatio.push_back(volumeRatioBB2);
                    }
                }
            }

            // add the object bounding box to the dataset
            if(!flagIntersection)
            {
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mClusters.push_back(cluster); // add cluster
                cout << "cluster.object_name = " << cluster.object_name << endl;

                cout << "Add a new object to the databse ..." << "\n" << endl;
            }

            // new object, with the different object label, update
            if(!volumeRatio.empty())
            {
                vector<double>::iterator maxRatio = std::max_element(volumeRatio.begin(), volumeRatio.end());
                cout << "maxRatio = " << *maxRatio << "  " << std::distance(volumeRatio.begin(), maxRatio) << endl;
                Cluster& temp_cluster = *likely_update_cluster[std::distance(volumeRatio.begin(), maxRatio)];
                cluster.object_id = temp_cluster.object_id;

                cout << "cluster.object_id = " << cluster.object_id << endl;

                mClusters[temp_cluster.object_id - 1] = cluster; // update the cluster
                cout << "Update an object with different label in the databse, the object number: " << cluster.object_id << endl;

                cout << "Update intersection object number: " << mUpdateIntersectionNum << "\n" << endl;
                mUpdateIntersectionNum++;
            }
            return;
        }
        else // old object, with the same object label, update
        {
            cout << "----------old-----------" << endl;

            vector<int> delete_num;

            // update the old object

            cout << "mClusters.size() = " << mClusters.size() << endl;
            cout << "cluster.object_update_num = " << cluster.object_update_num << endl;

            cluster.object_id = mClusters[cluster.object_update_num].object_id;
            cluster.prob = max(mClusters[cluster.object_update_num].prob, cluster.prob); // max prob
            mClusters[cluster.object_update_num] = cluster; // update the cluster
            cout << "\n" << "Update an object in the databse with same label, the object number: " << cluster.object_id << endl;

            cout << "Object updated number: " << mUpdateObjectNum << "\n" << endl;
            mUpdateObjectNum++;

            // judge the intersection between the updated object with database


            for(int n = 0; n < mClusters.size(); n++)
            {
                if(mClusters[n].object_id != cluster.object_id)
                {
                    Bb3dIntersection temp2;

                    temp2 = calculateIntersectionBB3D(cluster.bb3d, mClusters[n].bb3d);

                    double volume1, volume2, intersection, union_bb, IoU3d, volumeRatioBB1, volumeRatioBB2;
                    volume1 = temp2.volume1;
                    volume2 = temp2.volume2;
                    intersection = temp2.intersection;
                    union_bb = temp2.union_bb;
                    IoU3d = temp2.IoU3d;
                    volumeRatioBB1 = temp2.volumeRatioBB1;
                    volumeRatioBB2 = temp2.volumeRatioBB2;

                    cout << "volume1 = " << volume1 << endl;
                    cout << "volume2 = " << volume2 << endl;
                    cout << "intersection = " << intersection << endl;
                    cout << "union = " << union_bb << endl;
                    cout << "IoU3d = " << IoU3d << endl;
                    cout << "volumeRatioBB1 = " << volumeRatioBB1 << endl;
                    cout << "volumeRatioBB2 = " << volumeRatioBB2 << endl;

                    if((volumeRatioBB1 < mMinIntersectionTh) && (volumeRatioBB2 < mMinIntersectionTh)) // add the new object bounding box
                    {
                        continue;
                    }
                    else
                    {
                        if(volumeRatioBB1 > volumeRatioBB2) // delete the updated object bounding box
                        {
                            delete_num.push_back(cluster.object_id - 1);
                            continue;
                        }
                        else
                        {
                            delete_num.push_back(mClusters[n].object_id - 1);
                        }
                    }
                }
            }

            // delete the object bounding box with big intersection
            for(int n = 0; n < delete_num.size(); n++)
            {
                mClusters.erase(mClusters.begin() + delete_num[n]);
                cout << "After updating old object, delete the object number: " << delete_num[n] << "\n" << endl;
            }

            // update the object id
            for(int n = 0; n < mClusters.size(); n++)
            {
                mClusters[n].object_id = n + 1;
            }

        }
        return;
    }
}
*/

// Merge object database after running the dataset
// merge the whole object database when the dataset is completed
// (1) delete small objects
// (2) delete intersection objects
void ObjectDatabase::objectDatabaseMerge()
{

//edit by
    cout << "****************Delete small objects in database" << endl;
    // delete small objects in database
//    int delete_num_1 = databaseDeleteSmallObjects();
//    cout << "       delete_num_1 = " << delete_num_1 << endl;

//    while(delete_num_1 > 0)
//    {
//        cout << "       delete_num_1 = " << delete_num_1 << endl;
//        cout << "       Delete small objects again ..." << endl;
//        delete_num_1 = databaseDeleteSmallObjects();
//    }
//    cout << "       delete_num_1 = " << delete_num_1 << endl;



    int delete_num_2 = databaseDeleteIntersectionObjects();
//
    while(delete_num_2 > 0)
    {
        cout << "         delete_num_2 = " << delete_num_2 << endl;
        cout << "         Delete intersection objects again ..." << endl;
        delete_num_2 = databaseDeleteIntersectionObjects();
    }
    cout << "         delete_num_2 = " << delete_num_2 << endl;
    cout << "         Finish to merge the objects ... " << endl;
}

// delete small objects in database
int ObjectDatabase::databaseDeleteSmallObjects()
{
    //cout << "\n" << "Delete small objects in the database ..." << endl;
    vector<int> delete_num;
    for(int n = 0; n < mClusters.size(); n++)
    {
        Eigen::Vector3d objectSize(mClusters[n].size(0), mClusters[n].size(1), mClusters[n].size(2));
        Eigen::Vector3d objectSizePriority(mvSizes[mClusters[n].class_id]);



        double vol1, vol2;
        vol1 = objectSize[0] * objectSize[1] * objectSize[2];
        vol2 = objectSizePriority[0] * objectSizePriority[1] * objectSizePriority[2];

        double minEdge = min(min(objectSize[0], objectSize[1]), objectSize[2]);
        double minEdgePriority = min(min(objectSizePriority[0], objectSizePriority[1]), objectSizePriority[2]);

        // delete small objects
        cout << "*****object_name = " << mClusters[n].object_name << endl;
        cout << "       vol1 = " << vol1 << endl;
        cout << "       vol2.class.id = " << mClusters[n].class_id << endl;
        cout << "       vol2_0 = " << objectSizePriority[0] << endl;
        cout << "       vol2_1 = " << objectSizePriority[1] << endl;
        cout << "       vol2_2 = " << objectSizePriority[2] << endl;

        cout << "       vol2 = " << vol2 << endl;
        cout << "       mVolumeThMax * vol2 = " << mVolumeThMax * vol2 << endl;
        cout << "       mVolumeThMin * vol2 = " << mVolumeThMin * vol2 << endl;
        cout << "       minEdge = " << minEdge << endl;
        cout << "       mMinEdgeTh = " << mMinEdgeTh << endl;
        cout << "       minEdgePriority = " << minEdgePriority << endl;
        cout << "       mMinEdgeTh * minEdgePriority = " << mMinEdgeTh * minEdgePriority << "\n" << endl;

        // The judgement condition: volume and minimum edge
        if((vol1 <= mVolumeThMin * vol2) || (vol1 >= mVolumeThMax * vol2) || (minEdge < mMinEdgeTh * minEdgePriority))
        {
            if(vol1 <= mVolumeThMin * vol2)
            {
                cout << "       Object is too small, delete it, delete object number: " << mDeleteObjectNum << endl;
            }
            if(minEdge < mMinEdgeTh * minEdgePriority)
            {
                cout << "       Object is too thin, delete it, delete object number: " << mDeleteObjectNum << endl;
            }
           if(vol1 >= mVolumeThMax * vol2)
           {
               cout << "       Object is too big, delete it, delete object number: " << mDeleteObjectNum << endl;
           }

            cout << "*****object_name = " << mClusters[n].object_name << "\n" << endl;
            mDeleteObjectNum++;
            delete_num.push_back(n);
        }
    }

    cout << "\n Before delete small objects:" << endl;
    for(int m = 0; m < mClusters.size(); m++)
    {
        cout << mClusters[m].object_name << endl;
    }

    // delete small objects
    for(int n = 0; n < delete_num.size(); n++)
    {
        //cout << "delete_num = " << delete_num[n] << endl;
        mClusters.erase(mClusters.begin() + delete_num[n] - n);
    }

    cout << "\n After delete small objects:" << endl;

    for(int m = 0; m < mClusters.size(); m++)
    {
        cout << mClusters[m].object_name << endl;
    }

    // update the object id
    for(int n = 0; n < mClusters.size(); n++)
    {
        mClusters[n].object_id = n + 1;
    }
    return delete_num.size();
}

// delete intersection objects in the database
int ObjectDatabase::databaseDeleteIntersectionObjects()
{
    cout << "****************Delete intersection objects in the database ..." << endl;

    bool flagIntersection = false; // no intersection with all object bounding boxes
    //vector<vector<Cluster>::iterator> likely_update_cluster; // store the cluster to update

    //vector<double> volumeRatio;
    //likely_update_cluster.clear();
    //volumeRatio.clear();

    vector<int> delete_num;
    for(int m = 0; m < mClusters.size(); m++)
    {
        for(int n = m + 1; n < mClusters.size(); n++)
        {
            //edit by dx1
           cout << "mClusters[m].object_name = " << mClusters[m].object_name << endl;
           cout << "mClusters[n].object_name = " << mClusters[n].object_name << endl;

           for(int i = 0; i < 10; i++)
           {
               cout << "mClusters[m].bb3d = " << mClusters[m].bb3d[i] << endl;
           }
           for(int i = 0; i < 10; i++)
           {
               cout << "mClusters[n].bb3d = " << mClusters[n].bb3d[i] << endl;
           }
           cout << "---------------------" << endl;

            Bb3dIntersection temp1;
            temp1 = calculateIntersectionBB3D(mClusters[m].bb3d, mClusters[n].bb3d);
//            cout << "---------------------" << endl;

            double volume1, volume2, intersection, union_bb, IoU3d, volumeRatioBB1, volumeRatioBB2;
            volume1 = temp1.volume1;
            volume2 = temp1.volume2;
            intersection = temp1.intersection;
            union_bb = temp1.union_bb;
            IoU3d = temp1.IoU3d;
            volumeRatioBB1 = temp1.volumeRatioBB1;
            volumeRatioBB2 = temp1.volumeRatioBB2;

//            cout << "volume1 = " << volume1 << endl;
//            cout << "volume2 = " << volume2 << endl;
//            cout << "intersection = " << intersection << endl;
//            cout << "union = " << union_bb << endl;
//            cout << "IoU3d = " << IoU3d << endl;
//            cout << "volumeRatioBB1 = " << volumeRatioBB1 << endl;
//            cout << "volumeRatioBB2 = " << volumeRatioBB2 << endl;

            // the intersection is very small
            if((volumeRatioBB1 < mMinIntersectionTh) && (volumeRatioBB2 < mMinIntersectionTh))
            {
                continue;
            }
            else
            {
                flagIntersection = true;
                if(volumeRatioBB1 > volumeRatioBB2) // delete the object bounding box
                {
                    //                        cout << "Delete intersection object number: " << mDeleteIntersectionNum << "\n" << endl;
                    //                        mDeleteIntersectionNum++;
                    //                        continue;
                    //likely_update_cluster.push_back(mClusters.begin() + m);
                    //volumeRatio.push_back(volumeRatioBB1);
                    delete_num.push_back(m);
                }
                else
                {
                    //likely_update_cluster.push_back(mClusters.begin() + n);
                    //volumeRatio.push_back(volumeRatioBB2);
                    delete_num.push_back(n);
                }
            }

//            if(!volumeRatio.empty())
//            {
////                vector<double>::iterator maxRatio = std::max_element(volumeRatio.begin(), volumeRatio.end());
////                cout << "maxRatio = " << *maxRatio << "  " << std::distance(volumeRatio.begin(), maxRatio) << endl;
//                //Cluster& temp_cluster = *likely_update_cluster[std::distance(volumeRatio.begin(), maxRatio)];
//            }

        }
    }

//    for(int m = 0; m < delete_num.size(); m++)
//    {
//        cout << "delete_num[m] = " << delete_num[m] << endl;
//    }

    std::sort(delete_num.begin(), delete_num.end()); //升序排列

//    for(int m = 0; m < delete_num.size(); m++)
//    {
//        cout << "delete_num[m] = " << delete_num[m] << endl;
//    }

    for(int i = 0; i < delete_num.size(); i++)
    {
        for(int j = i + 1; j < delete_num.size(); j++)
        {
            if(delete_num[i] == delete_num[j])
            {
                delete_num.erase(delete_num.begin() + j);
                j = j - 1;
            }
        }
    }

//    for(int n = 0; n < delete_num.size(); n++)
//    {
//        cout << "delete_num[n] = " << delete_num[n] << endl;
//    }

    cout << "\n Before delete intersection objects:" << endl;
    for(int m = 0; m < mClusters.size(); m++)
    {
        cout << mClusters[m].object_name << endl;
    }

    // delete small objects
    for(int n = 0; n < delete_num.size(); n++)
    {
        cout << "delete_num[n] = " << delete_num[n] << endl;
        mClusters.erase(mClusters.begin() + delete_num[n] - n);
    }

    mDeleteIntersectionNum += delete_num.size();
    cout << "Delete intersection object number: " << mDeleteIntersectionNum << "\n" << endl;

    cout << "\n After delete intersection objects:" << endl;
    for(int m = 0; m < mClusters.size(); m++)
    {
        cout << mClusters[m].object_name << endl;
    }

    // update the object id
    for(int n = 0; n < mClusters.size(); n++)
    {
        mClusters[n].object_id = n + 1;
    }
    return delete_num.size();
}



// calculate intersection of two bounding boxes
// object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
/*Bb3dIntersection ObjectDatabase::calculateIntersectionBB3D(double* bb1, double* bb2)
{
    // fusion the intersection bounding box in the same object, but with different label or size
    // 为变量分配内存空间，1�?10表示矩阵的大小（所有maltab只有一种变量，就是矩阵，为了和Cpp变量接轨，设置成1*1的矩阵，mxDOUBLE_CLASS表示变量的精度）
//    cout << "Start to calculate intersection ... " << endl;
//    mwArray mwABoundingBox1(1, 10, mxDOUBLE_CLASS);
//    mwArray mwABoundingBox2(1, 10, mxDOUBLE_CLASS); // object bounding box
//    mwArray mwAVolume1(1, 5, mxDOUBLE_CLASS); // volume of the bounding box
//    mwArray mwAVolume2(1, 5, mxDOUBLE_CLASS);
//    mwArray mwAIntersection(1, 5, mxDOUBLE_CLASS); // intersection volume of two bounding boxes
//    mwArray mwAUnion(1, 5, mxDOUBLE_CLASS); // Volume1 + Volume2 - Intersection
//    mwArray mwAIoU3d(1, 5, mxDOUBLE_CLASS); // 3d IoU

    // object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
    //            double bb1[10] = {1, 1, 2, 1, 2, 2, 1, 2, 0, 1};
    //            double bb2[10] = {0, 1, 1, 2, 2, 1, 1, 0, 0, 1};
    //double volume1, volume2, intersection, union_bb, IoU3d;

    //            mwABoundingBox1.SetData(bb1, 10);
    //            mwABoundingBox2.SetData(bb2, 10);

//    mwABoundingBox1.SetData(bb1, 10);
//    mwABoundingBox2.SetData(bb2, 10);

    // calculate the overlap of two bounding boxws
//    calculateBb3dOverlap(5, mwAVolume1, mwAVolume2, mwAIntersection, mwAUnion, mwAIoU3d, mwABoundingBox1, mwABoundingBox2);

//    Bb3dIntersection temp;
    // get data
//    temp.volume1 = mwAVolume1.Get(1, 1);
//    temp.volume2 = mwAVolume2.Get(1, 1);
//    temp.intersection = mwAIntersection.Get(1, 1);
//    temp.union_bb = mwAUnion.Get(1, 1);
//    temp.IoU3d = mwAIoU3d.Get(1, 1);
//    temp.volumeRatioBB1 = temp.intersection / temp.volume1;
//    temp.volumeRatioBB2 = temp.intersection / temp.volume2;

//    return temp;
}*/

// calculate intersection of two bounding boxes
// object bounding box format : x1 y1 x2 y2 x3 y3 x4 y4 zMin zMax
// New method: delete intersection objects in the database
/*Bb3dIntersection ObjectDatabase::calculateIntersectionBB3D(double* bb1, double* bb2)
{
    std::vector<cv::Point2f> bb1Points;
    std::vector<cv::Point2f> bb2Points;
    cv::Point2f bb1Point1 = cv::Point2f(bb1[0], bb1[1]);
    cv::Point2f bb1Point2 = cv::Point2f(bb1[2], bb1[3]);
    cv::Point2f bb1Point3 = cv::Point2f(bb1[4], bb1[5]);
    cv::Point2f bb1Point4 = cv::Point2f(bb1[6], bb1[7]);
    bb1Points.push_back(bb1Point1);
    bb1Points.push_back(bb1Point2);
    bb1Points.push_back(bb1Point3);
    bb1Points.push_back(bb1Point4);
    cv::Point2f bb2Point1 = cv::Point2d(bb2[0], bb2[1]);
    cv::Point2f bb2Point2 = cv::Point2d(bb2[2], bb2[3]);
    cv::Point2f bb2Point3 = cv::Point2d(bb2[4], bb2[5]);
    cv::Point2f bb2Point4 = cv::Point2d(bb2[6], bb2[7]);
    bb2Points.push_back(bb2Point1);
    bb2Points.push_back(bb2Point2);
    bb2Points.push_back(bb2Point3);
    bb2Points.push_back(bb2Point4);

    cv::Vec2f vecs1[2];
    vecs1[0] = cv::Vec2f(bb1Point1 - bb1Point2);
    vecs1[1] = cv::Vec2f(bb1Point2 - bb1Point3);
    // check that given sides are perpendicular
    cout << " rect1 perpendicular = " << abs(vecs1[0].dot(vecs1[1])) / (norm(vecs1[0]) * norm(vecs1[1])) << endl;

    cv::Vec2f vecs2[2];
    vecs2[0] = cv::Vec2f(bb2Point1 - bb2Point2);
    vecs2[1] = cv::Vec2f(bb2Point2 - bb2Point3);
    cout << " rect2 perpendicular = " << abs(vecs2[0].dot(vecs2[1])) / (norm(vecs2[0]) * norm(vecs2[1])) << endl;



    for (vector<cv::Point2f>::iterator it = bb1Points.begin(); it != bb1Points.end(); it++)
    {
        cout << *it << " ";
    }

    cout << "       RotatedRect1" << endl;
    for (vector<cv::Point2f>::iterator it = bb2Points.begin(); it != bb2Points.end(); it++)
    {
        cout << *it << " ";
    }

    cout << "       RotatedRect2" << endl;

    cv::RotatedRect rect_1 = cv::RotatedRect(cv::Point2f(bb1[0], bb1[1]), cv::Point2f(bb1[2], bb1[3]),cv::Point2f(bb1[4], bb1[5]));
    cout << "       RotatedRect1 OK" << endl;

    cv::RotatedRect rect_2 = cv::RotatedRect(cv::Point2f(bb2[0], bb2[1]), cv::Point2f(bb2[2], bb2[3]),cv::Point2f(bb2[4], bb2[5]));

    cout << "       RotatedRect2 OK" << endl;
//20201008 OpenCV Error: Assertion failed (abs(vecs[0].dot(vecs[1])) / (norm(vecs[0]) * norm(vecs[1])) <= 1.19209289550781250000e-7F) in RotatedRect

//    cv::RotatedRect rect_1 = cv::RotatedRect(cv::Point2f(bb1[0], bb1[1]), cv::Point2f(bb1[2], bb1[3]),cv::Point2f(bb1[4], bb1[5]));
//    cv::RotatedRect rect_2 = cv::RotatedRect(cv::Point2f(bb2[0], bb2[1]), cv::Point2f(bb2[2], bb2[3]),cv::Point2f(bb2[4], bb2[5]));
    std::vector<cv::Point2f> vertices;
    int intersectionType = cv::rotatedRectangleIntersection(rect_1, rect_2,vertices);
    double intersectionArea = cv::contourArea(vertices);

    cout << "       IntersectionArea is " << intersectionArea << endl;

    std::vector<double> vect = {bb1[8],bb1[9],bb2[8],bb2[9]};
    for (vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
    std::sort(vect.begin(),vect.end());

    for (vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
    {
        cout << *it << " ";
    }

    double intersectionHeight = fabs(vect[1]-vect[2]);

    Bb3dIntersection temp;

    temp.volume1 = cv::contourArea(bb1Points) * fabs(bb1[8]-bb1[9]);
    temp.volume2 = cv::contourArea(bb2Points) * fabs(bb2[8]-bb2[9]);
    temp.intersection = intersectionArea * intersectionHeight;
    temp.union_bb = temp.volume1 + temp.volume2 - temp.intersection;
    temp.IoU3d = temp.intersection / temp.union_bb;
    temp.volumeRatioBB1 = temp.intersection / temp.volume1;
    temp.volumeRatioBB2 = temp.intersection / temp.volume2;

    return temp;
}*/


Bb3dIntersection ObjectDatabase::calculateIntersectionBB3D(double* bb1, double* bb2)
{
    std::vector<cv::Point2f> bb1Points;
    std::vector<cv::Point2f> bb2Points;
    cv::Point2f bb1Point1 = cv::Point2f(bb1[0], bb1[1]);
    cv::Point2f bb1Point2 = cv::Point2f(bb1[2], bb1[3]);
    cv::Point2f bb1Point3 = cv::Point2f(bb1[4], bb1[5]);
    cv::Point2f bb1Point4 = cv::Point2f(bb1[6], bb1[7]);
    bb1Points.push_back(bb1Point1);
    bb1Points.push_back(bb1Point2);
    bb1Points.push_back(bb1Point3);
    bb1Points.push_back(bb1Point4);
    cv::Point2f bb2Point1 = cv::Point2d(bb2[0], bb2[1]);
    cv::Point2f bb2Point2 = cv::Point2d(bb2[2], bb2[3]);
    cv::Point2f bb2Point3 = cv::Point2d(bb2[4], bb2[5]);
    cv::Point2f bb2Point4 = cv::Point2d(bb2[6], bb2[7]);
    bb2Points.push_back(bb2Point1);
    bb2Points.push_back(bb2Point2);
    bb2Points.push_back(bb2Point3);
    bb2Points.push_back(bb2Point4);

    Polygon poly[2];
    Polygon inter;
    poly[0].clear();
    poly[0].add(bb1Point1);
    poly[0].add(bb1Point2);
    poly[0].add(bb1Point3);
    poly[0].add(bb1Point4);

    poly[1].clear();
    poly[1].add(bb2Point1);
    poly[1].add(bb2Point2);
    poly[1].add(bb2Point3);
    poly[1].add(bb2Point4);
    intersectPolygonSHPC(poly[0],poly[1],inter);
    double intersectionArea = inter.area();

    double area1 = sqrt((bb1Point1.x-bb1Point2.x) * (bb1Point1.x-bb1Point2.x) + (bb1Point1.y-bb1Point2.y)*(bb1Point1.y-bb1Point2.y)) *
                   sqrt((bb1Point1.x-bb1Point3.x) * (bb1Point1.x-bb1Point3.x) + (bb1Point1.y-bb1Point3.y)*(bb1Point1.y-bb1Point3.y));

    double area2 = sqrt((bb2Point1.x-bb2Point2.x) * (bb2Point1.x-bb2Point2.x) +  (bb2Point1.y-bb2Point2.y)*(bb2Point1.y-bb2Point2.y)) *
                   sqrt((bb2Point1.x-bb2Point3.x) * (bb2Point1.x-bb2Point3.x) +  (bb2Point1.y-bb2Point3.y)*(bb2Point1.y-bb2Point3.y));
    cout << "       Area1 is " << intersectionArea << endl;
    cout << "       Area2 is " << intersectionArea << endl;
    cout << "       IntersectionArea is " << intersectionArea << endl;

    std::vector<double> vect = {bb1[8],bb1[9],bb2[8],bb2[9]};
    for (vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
    std::sort(vect.begin(),vect.end());

    for (vector<double>::iterator it = vect.begin(); it != vect.end(); it++)
    {
        cout << *it << " ";
    }

    double intersectionHeight = fabs(vect[1]-vect[2]);

    Bb3dIntersection temp;

    temp.volume1 = area1 * fabs(bb1[8]-bb1[9]);
    temp.volume2 = area2 * fabs(bb2[8]-bb2[9]);
    temp.intersection = intersectionArea * intersectionHeight;
    temp.union_bb = temp.volume1 + temp.volume2 - temp.intersection;
    temp.IoU3d = temp.intersection / temp.union_bb;
    temp.volumeRatioBB1 = temp.intersection / temp.volume1;
    temp.volumeRatioBB2 = temp.intersection / temp.volume2;

    return temp;
}

// for each keyframe, after 2D detection, obtain the point cloud of each object,
// judge whether the object is new or old
// if the objct is new,
// if the object is old, merge point clouds

// keyframe_num = 1; // the detected objects in the first keyframe are all added in the database,
// the late keyframes are merged.
void ObjectDatabase::objectPointCloudMerge(Cluster& cluster, bool& isObjectDatabaseUpdate)
{
    unique_lock<mutex> lock(mMutexClusters);

    isObjectDatabaseUpdate = false; // true: database is updated, false: database is not updated

    std::vector<string>::iterator iter_   = mvCocoNames.begin() - 1;
    std::vector<string>::iterator it_end_ = mvCocoNames.end();
    iter_ = std::find(++iter_, it_end_, cluster.object_name);// 按照名字查找
    cluster.class_id = std::distance(mvCocoNames.begin(), iter_);

    // 1. 查看总数�?,数据库为�?
    //if((!mClusters.size()) || cluster.first_keyframe_with_objects == true)
    if(cluster.first_keyframe_with_objects == true)
    {
        cluster.flag_new_object = true;
        return;
    }
    else
    {
        // 2. 数据库内已经存在物体了，查找新物体是否在数据库内已经存在
        std::vector<Cluster>::iterator iter   = mClusters.begin() - 1;
        std::vector<Cluster>::iterator it_end = mClusters.end();
        std::vector<std::vector<Cluster>::iterator> likely_obj;// 同名字的物体的迭代器
        while(true)
        {
            iter = find(++iter, it_end, cluster.object_name);// 按照名字查找
            if (iter != it_end )// 找到一个，存放起来
                likely_obj.push_back(iter);
            else//已经找不到了
                break;
        }

        // 3. 如果没找到，则直接添加进数据�?
        std::vector<Cluster>::iterator best_close;// 最近的索引
        float center_distance = 100;// 对应的距�?
        if(!likely_obj.size())
        {
            cluster.flag_new_object = true;
        }
        else//找到多个和数据库里同名字的物�?
        {
            // 4. 遍例每一个同名字的物体，找到中心点最近的一�?
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;

                //Eigen::Vector3d dis_vec = cluster.boxCenter - temp_cluster.boxCenter;// 中心点连接向�?

//                // 计算点云重心
//                Eigen::Vector4d centroid_database_object, centroid_object;
//                pcl::compute3DCentroid(*temp_cluster.object_point_cloud_after_seg, centroid_database_object);
//                pcl::compute3DCentroid(*cluster.object_point_cloud_after_seg, centroid_object);
//                Eigen::Vector4d dis_vec = centroid_database_object - centroid_object;

                // 3D bounding box centroid distance
                Eigen::Vector3d centroid_database_object(temp_cluster.boxCenter), centroid_object(cluster.boxCenter);
                Eigen::Vector3d dis_vec = centroid_database_object - centroid_object;

//                std::cout << "The XYZ coordinates of the centroid are: ("
//                          << centroid_database_object[0] << ", "
//                          << centroid_database_object[1] << ", "
//                          << centroid_database_object[2] << ")." << std::endl;

//                std::cout << "The XYZ coordinates of the centroid are: ("
//                          << centroid_object[0] << ", "
//                          << centroid_object[1] << ", "
//                          << centroid_object[2] << ")." << std::endl;

                float dist = dis_vec.norm();
//                cout << "dist = " << dist << endl;

                if(dist < center_distance)
                {
                    center_distance = dist; // 最短的距离
                    best_close      = temp_iter;// 对应的索�?
                }
            }

//            cout << "center_distance_min = " << center_distance << endl;

            // 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息

            if(center_distance < mvDistanceTh[cluster.class_id])
                // if(center_distance < mvSizes[0])
            {
//                cout << "cluster.class_id = " << cluster.class_id << endl;
//                cout << "cluster.object_name = " << cluster.object_name << endl;
//                cout << "object_size = " << mvDistanceTh[cluster.class_id] << endl;
//                cout << "Object fusion..." << endl;

                cluster.flag_new_object = false;

                // merge object point clouds
                *cluster.object_point_cloud_before_seg += *best_close->object_point_cloud_before_seg;
                cluster.point_color_r = best_close->point_color_r;
                cluster.point_color_g = best_close->point_color_g;
                cluster.point_color_b = best_close->point_color_b;

                cluster.object_update_num = best_close->object_id - 1; // to update the object in the database, object_id is from 1
            }
            else
            {
                // 6. 如果距离超过物体尺寸则认为是不同位置的同一种物体，直接放入数据�?
                cluster.flag_new_object = true;
//                cout << "cluster.object_name = " << cluster.object_name << endl;
//                cout << "cluster.class_id = " << cluster.class_id << endl;
//                cout << "object_size = " << mvDistanceTh[cluster.class_id] << endl;

//                cout << "Exceed the distance limit..." << endl;
            }
        }
        return;
    }
}

// Get the object clusters
std::vector<Cluster> ObjectDatabase::getObjectClusters()
{
    unique_lock<mutex> lock(mMutexClusters);
    return mClusters;
}
