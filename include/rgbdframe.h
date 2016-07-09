/*
 * rgbdframe.h
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */

#ifndef RGBDFRAME_H_
#define RGBDFRAME_H_


#include "common.h"
#include "parameter_reader.h"

//#include"Thirdparty/DBoW2/DBoW2/FORB.h"
//#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor{

//帧
class RGBDFrame
{
public:
    typedef shared_ptr<RGBDFrame> Ptr;

public:
    RGBDFrame() {}
    // 方法
    // 给定像素点，求3D点坐标
    cv::Point3f project2dTo3dLocal( const int& u, const int& v  ) const
    {
        if (depth.data == nullptr)
            return cv::Point3f(0,0,0);
        ushort d = depth.ptr<ushort>(v)[u];
        if (d == 0)
            return cv::Point3f(0,0,0);
        cv::Point3f p;
        p.z = double( d ) / camera.scale;
        p.x = ( u - camera.cx) * p.z / camera.fx;
        p.y = ( v - camera.cy) * p.z / camera.fy;
        return p;
    }

    cv::Point3f project2dTo3dLocal1( const int& u, const int& v, CAMERA_INTRINSIC_PARAMETERS camera1) const
    {
        if (depth.data == nullptr)
            return cv::Point3f(0,0,0);
        ushort d = depth.ptr<ushort>(v)[u];
        if (d == 0)
            return cv::Point3f(0,0,0);
        cv::Point3f p;
        p.z = double( d ) / camera1.scale;
        p.x = ( u - camera1.cx) * p.z / camera1.fx;
        p.y = ( v - camera1.cy) * p.z / camera1.fy;
        return p;
    }

public:
    // 数据成员
    int id  = -1;            //-1表示该帧不存在

    // 彩色图和深度图
    cv::Mat rgb, depth;
    // 该帧位姿
    // 定义方式为：x_local = T * x_world 注意也可以反着定义；
    Eigen::Isometry3d       T=Eigen::Isometry3d::Identity();

    // 特征
    vector<cv::KeyPoint>    keypoints;/*
    vector<cv::KeyPoint>    keypoints1;
    vector<cv::KeyPoint>    keypoints2;
    vector<cv::KeyPoint>    keypoints3;
    vector<cv::KeyPoint>    keypoints4;*/
    cv::Mat                 descriptor;/*
    cv::Mat                 descriptor1;
    cv::Mat                 descriptor2;
    cv::Mat                 descriptor3;
    cv::Mat                 descriptor4;*/
    vector<cv::Point3f>     kps_3d;
    Eigen::Vector3d         translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond      rotation = Eigen::Quaterniond::Identity();

    // 相机
    // 默认所有的帧都用一个相机模型
    CAMERA_INTRINSIC_PARAMETERS camera;

    // BoW回环特征
//    DBoW2::BowVector bowVec;

};

// FrameReader
// 从TUM数据集中读取数据的类
class FrameReader
{
public:
	vector<string>  rgbFiles, depthFiles;
	vector<string>  rgbTimes, depthTimes;
	void FrameWriter(RGBDFrame frame);
	FILE *fp;
    FrameReader( const rgbd_tutor::ParameterReader& para )
        : parameterReader( para )
    {
        init_tum( );
    	fp=fopen("/home/wei/workspace/slam/pose","r+");
    	if(fp == NULL)
    		cout << "Open file error!" << endl;
        ofstream out("/home/wei/workspace/slam/pose");
    }

    // 获得下一帧
    RGBDFrame::Ptr   next();

    // 重置index
    void    reset()
    {
        cout<<"重置 frame reader"<<endl;
        currentIndex = start_index;
    }

    // 根据index获得帧
    RGBDFrame::Ptr   get( const int& index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
    // 初始化tum数据集
    void    init_tum( );
protected:

    // 当前索引
    int currentIndex =0;
    // 起始索引
    int start_index  =0;

    const   ParameterReader&    parameterReader;

    // 文件名序列
//    vector<string>  rgbFiles, depthFiles;

    // 数据源
    string  dataset_dir;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS     camera;
};

};



#endif /* RGBDFRAME_H_ */
