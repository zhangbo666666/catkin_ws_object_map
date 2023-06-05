//
// Created by ichigoi7e on 17/07/2018.
//

#include "Converter.h"
#include "PointCloudMapping.h"

#include <KeyFrame.h>
#include <boost/make_shared.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( ); // Segmentation Fault

    // viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    // viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    cout << "color.rows = " << color.rows << "color.cols = " <<  color.cols << endl;
    cout << "depth.rows = " << depth.rows << "depth.cols = " <<  depth.cols << endl;
    for ( int m=0; m<depth.rows; m++ )
    {
        for ( int n=0; n<depth.cols; n++ )
        {
            float d = depth.ptr<float>(m)[n];

                        //cout << "d = " << d << endl;

            // if (d < 0.01 || d>10)
            
            if (d < 0.10 || d > 3 || isnan(d)) // bool isnan( float arg );
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

        //    p.b = color.ptr<uchar>(m)[n*3];
        //    p.g = color.ptr<uchar>(m)[n*3+1];
        //    p.r = color.ptr<uchar>(m)[n*3+2];
            
            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    if(tmp->points.size() > 0)
    {
        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
        PointCloud::Ptr cloud(new PointCloud);
        pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());

        //--------------------------
        // 对点云坐标做变换，绕x轴旋�????90度，将z轴指向上�????
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        //trans.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)));
        trans.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(1,0,0)));
        pcl::transformPointCloud(*cloud, *cloud, trans);
        //--------------------------

        cloud->is_dense = false;
        cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
        return cloud;
    }
    else
        cout << "The input point cloud is empty!" << endl;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        // cout << "++++=" << endl;
        // // PointCloud::Ptr tmp(new PointCloud());
        // cout << "++=" << endl;
        // if(globalMap->points.size() > 0)
        // {
            // voxel.setInputCloud( globalMap );
            // voxel.filter( *tmp );
            // globalMap->swap( *tmp );
            viewer.showCloud( globalMap );
        // }

        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
    //pcl::io::savePCDFileASCII("/home/gxc/dataset/SceneNN/000/map.pcd", *globalMap);
    // pcl::io::savePCDFileASCII("/home/zb/dataset/435/map.pcd", *globalMap);

    //pcl::io::savePCDFileASCII("map.pcd", *globalMap);
}

// 返回所有地图点
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GetAllPointCloud()
{
    return globalMap;
}
