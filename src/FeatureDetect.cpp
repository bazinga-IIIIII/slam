/*
 * FeatureDetect.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: wei
 */

#include <FeatureDetect.h>

FeatureDetect::FeatureDetect() {
	// TODO Auto-generated constructor stub
	good_match_threshold = 4.0;
}

FeatureDetect::~FeatureDetect() {
	// TODO Auto-generated destructor stub
}

void FeatureDetect::Detect_orb(RGBDFrame::Ptr& frame) {
//	ORB orb;
//	orb(frame->rgb, Mat(), frame->keypoints, frame->descriptor);
	Mat grayimage;
	cvtColor(frame->rgb, grayimage, CV_BGR2GRAY);
	feature_detector.detect(grayimage, frame->keypoints);
	feature_extractor.compute(grayimage, frame->keypoints, frame->descriptor);
}

void FeatureDetect::Detect_sift(RGBDFrame::Ptr& frame) {
	// TODO
}

void FeatureDetect::Detect_surf(RGBDFrame::Ptr& frame) {
	// TODO
}

RESULT_OF_PNP FeatureDetect::Match_orb(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {

	RESULT_OF_PNP result;
	vector< cv::DMatch > matches;
//	cv::FlannBasedMatcher matcher;
	BruteForceMatcher<HammingLUT> matcher;
	matcher.match(src->descriptor, dst->descriptor, matches);

	cout << "find total " << matches.size() << " matches." << endl;
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;

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
		cv::Point2f p = src->keypoints[goodMatches[i].queryIdx].pt;
		ushort d = src->depth.ptr<ushort>(int(p.y))[int(p.x)];
		if (d == 0)
			continue;
		pts_img.push_back(cv::Point2f(dst->keypoints[goodMatches[i].trainIdx].pt));
		cv::Point3f pt(p.x, p.y, d);
		cv::Point3f pd = dst->project2dTo3dLocal(pt.x, pt.y);
		pts_obj.push_back(pd);
		flag++;
	}

	if (flag <= 5)
	{
		result.inliers = -1;
		return result;
	}

	Mat img_matches;
    drawMatches(src->rgb,src->keypoints,dst->rgb,dst->keypoints,matches,img_matches);
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

RESULT_OF_PNP FeatureDetect::Match_sift(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {
	// TODO
}

RESULT_OF_PNP FeatureDetect::Match_surf(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {
	// TODO
}
