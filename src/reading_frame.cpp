/*
 * reading_frame.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */
#include "rgbdframe.h"
#include <FeatureDetect.h>
#include <opencv2/core/eigen.hpp>      //for cv2eigen

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace rgbd_tutor;
using namespace cv;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


void use_eigen_tran(RESULT_OF_PNP result, Eigen::Isometry3d &T)
{
	T = Eigen::Isometry3d::Identity();
	cv::Mat R;
	cv::Rodrigues(result.rvec, R);
	Eigen::Matrix3d r;
	cv::cv2eigen(R, r);
	Eigen::AngleAxisd angle(r);
	//cout << "translation" << endl;
	//cout <<"0: "<< result.tvec.at<double>(0, 0) << endl;
	//cout <<"1: " << result.tvec.at<double>(1, 0) << endl;
	//cout <<"2: " << result.tvec.at<double>(2, 0) << endl;
	//double a, b, c;
	//a = result.tvec.at<double>(0, 0);
	//b = result.tvec.at<double>(1, 0);
	//c = result.tvec.at<double>(2, 0);
//	Eigen::Translation<double, 3> trans(a,b,c);
//	Eigen::Translation<double, 3> trans(result.tvec.at<double>(0, 0), result.tvec.at<double>(0, 1), result.tvec.at<double>(0, 2));
	T = angle;
	T(0, 3) = result.tvec.at<double>(0, 0);
	T(1, 3) = result.tvec.at<double>(1, 0);
	T(2, 3) = result.tvec.at<double>(2, 0);
	//T(1, 3) = result.tvec.at<double>(0, 1);
	//T(2, 3) = result.tvec.at<double>(0, 2);
}



PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// »ñÈ¡Éî¶ÈÍŒÖÐ(m,n)ŽŠµÄÖµ
			ushort d = depth.ptr<ushort>(m)[n];
			// d ¿ÉÄÜÃ»ÓÐÖµ£¬ÈôÈçŽË£¬Ìø¹ýŽËµã
			if (d == 0)
				continue;
			// d ŽæÔÚÖµ£¬ÔòÏòµãÔÆÔöŒÓÒ»žöµã
			PointT p;

			// ŒÆËãÕâžöµãµÄ¿ÕŒä×ø±ê
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;

			// ŽÓrgbÍŒÏñÖÐ»ñÈ¡ËüµÄÑÕÉ«
			// rgbÊÇÈýÍšµÀµÄBGRžñÊœÍŒ£¬ËùÒÔ°ŽÏÂÃæµÄË³Ðò»ñÈ¡ÑÕÉ«
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// °ÑpŒÓÈëµœµãÔÆÖÐ
			cloud->points.push_back(p);
		}
	// ÉèÖÃ²¢±£ŽæµãÔÆ
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

	return cloud;
}


int main()
{
    ParameterReader para;
    FrameReader     fr(para);
    RGBDFrame::Ptr old_frame(new RGBDFrame);
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera = para.getCamera();
	RESULT_OF_PNP result;
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	PointCloud::Ptr cloud;
	PointCloud::Ptr output(new PointCloud());

	FeatureDetect fd;

	old_frame = fr.next();
	fd.Detect_orb(old_frame);
	cloud = image2PointCloud(old_frame->rgb, old_frame->depth, camera);
//	pcl::visualization::CloudViewer viewer("Cloud viewer");

    while( RGBDFrame::Ptr frame = fr.next() )
    {
    	fd.Detect_orb(frame);
    	fd.Match_orb(old_frame, frame, frame->camera);
    	Mat temp;
    	drawKeypoints(frame->rgb, frame->keypoints, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        cv::imshow( "image", temp);//frame->rgb );
        cv::waitKey(20);

        result = fd.Match_orb(old_frame, frame, camera);
        if(!fd.Key_Frame_Judge(result)) {
        	cout << frame->id << "  " << fd.Key_Frame_Judge(result) << endl;
           	use_eigen_tran(result,T);
        }
        else
        	continue;


 /*   //		cloud2 = image2PointCloud(dst.rgb, dst.depth, cam_params);
    		//cout << result.rvec << endl;
    		//cout << result.tvec << endl;
    	use_eigen_tran(result, T);


        PointCloud::Ptr newCloud = image2PointCloud(frame->rgb, frame->depth, camera);
        pcl::transformPointCloud( *cloud, *output, T.matrix() );
        *newCloud += *output;
        cloud = newCloud;
        viewer.showCloud( cloud );
*/
        old_frame->rgb = frame->rgb.clone();
        old_frame->depth = frame->depth.clone();
        old_frame->descriptor = frame->descriptor.clone();
       	old_frame->keypoints.clear();
       	for(int i = 0; i < frame->keypoints.size(); i++) {
       		old_frame->keypoints.push_back(frame->keypoints[i]);
        }
    }

    return 0;
}


