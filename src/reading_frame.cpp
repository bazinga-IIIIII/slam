/*
 * reading_frame.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */
#include "rgbdframe.h"


#include <opencv2/legacy/legacy.hpp>   //for BruteForceMatcher
#include <opencv2/core/eigen.hpp>      //for cv2eigen

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace rgbd_tutor;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};

void computeKeyPointsAndDesp_orb(RGBDFrame::Ptr& frame)
{
	ORB orb;
	vector<KeyPoint> keypoint_1;
	Mat descriptors_1;
	orb(frame->rgb, Mat(), frame->keypoints, frame->descriptor);
	return;
}

RESULT_OF_PNP estimateMotion_orb(RGBDFrame::Ptr& frame1, RGBDFrame::Ptr&frame2, CAMERA_INTRINSIC_PARAMETERS cam)
{
	RESULT_OF_PNP result;
	vector< cv::DMatch > matches;
//	cv::FlannBasedMatcher matcher;
	BruteForceMatcher<HammingLUT> matcher;
	/*
	cout << frame1->keypoints.size() << endl;
	cout << frame1->descriptor.size() << endl;
	cout << frame2->keypoints.size() << endl;
	cout << frame2->descriptor.size() << endl;
	*/
	matcher.match(frame1->descriptor, frame2->descriptor, matches);

	cout << "find total " << matches.size() << " matches." << endl;
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;
	double good_match_threshold = 4;
	for (size_t i = 0; i<matches.size(); i++)
	{
		if (matches[i].distance < minDis)
			minDis = matches[i].distance;
	}

	for (size_t i = 0; i<matches.size(); i++)
	{
		if (matches[i].distance < good_match_threshold*minDis)
			goodMatches.push_back(matches[i]);
	}

	if (goodMatches.size() <= 5)
	{
		result.inliers = -1;
		return result;
	}

	vector<cv::Point3f> pts_obj;
	vector< cv::Point2f > pts_img;
	int flag = 0;
	for (size_t i = 0; i<goodMatches.size(); i++)
	{
		cv::Point2f p = frame1->keypoints[goodMatches[i].queryIdx].pt;
		ushort d = frame1->depth.ptr<ushort>(int(p.y))[int(p.x)];
		if (d == 0)
			continue;
		pts_img.push_back(cv::Point2f(frame2->keypoints[goodMatches[i].trainIdx].pt));
		cv::Point3f pt(p.x, p.y, d);
		cv::Point3f pd = frame2->project2dTo3dLocal(pt.x, pt.y);
		pts_obj.push_back(pd);
		flag++;
	}

	if (flag <= 5)
	{
		result.inliers = -1;
		return result;
	}

	Mat img_matches;
    drawMatches(frame1->rgb,frame1->keypoints,frame2->rgb,frame2->keypoints,matches,img_matches);
    imshow("matches",img_matches);

	double camera_matrix_data[3][3] = {
		{ cam.fx, 0, cam.cx },
		{ 0, cam.fy, cam.cy },
		{ 0, 0, 1 }
	};

	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

	cout << "here" << endl;
	cout << camera_matrix_data[0][0] << endl;
	cout << rvec << endl;
	cout << tvec << endl;
	cout << inliers.rows << endl;

	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;
	cout << result.inliers << endl;

	return result;
}

double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}

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
    int flag = 0;
	RESULT_OF_PNP result;
	int min_inliers = 5;
	double max_norm = 0.3;
	double keyframe_threshold = 0.1;
	double norm;
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	PointCloud::Ptr cloud;
	PointCloud::Ptr output(new PointCloud());

	old_frame = fr.next();
	computeKeyPointsAndDesp_orb(old_frame);
	cloud = image2PointCloud(old_frame->rgb, old_frame->depth, camera);
	pcl::visualization::CloudViewer viewer("Cloud viewer");

    while( RGBDFrame::Ptr frame = fr.next() )
    {
    	computeKeyPointsAndDesp_orb(frame);

        cv::imshow( "image", frame->rgb );
        cv::waitKey(1);

        result = estimateMotion_orb(old_frame, frame, camera);
    	if ( result.inliers < min_inliers )
    	{
    		cout << "11111111" << endl;
    		continue;
    	}
    	norm = normofTransform(result.rvec, result.tvec);
    	if ( norm < keyframe_threshold )
    	{
    		cout << "22222222" << endl;
    		continue;
    	}
    	if ( norm > max_norm )
    	{
    		cout << "33333333" << endl;
    		continue;
    	}
    //		cloud2 = image2PointCloud(dst.rgb, dst.depth, cam_params);
    		//cout << result.rvec << endl;
    		//cout << result.tvec << endl;
    	use_eigen_tran(result, T);


        PointCloud::Ptr newCloud = image2PointCloud(frame->rgb, frame->depth, camera);
        pcl::transformPointCloud( *cloud, *output, T.matrix() );
        *newCloud += *output;
        cloud = newCloud;
        viewer.showCloud( cloud );

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


