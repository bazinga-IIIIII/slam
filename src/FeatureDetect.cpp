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
	min_inliers = 5;
	keyframe_threshold = 0.1;
	max_norm = 0.5;

}

FeatureDetect::~FeatureDetect() {
	// TODO Auto-generated destructor stub
}

void FeatureDetect::Detect_orb(RGBDFrame::Ptr& frame) {
	OrbFeatureDetector feature_detector(500);
	OrbDescriptorExtractor feature_extractor;
//	ORB orb;
//	orb(frame->rgb, Mat(), frame->keypoints, frame->descriptor);
	Mat grayimage;
	cvtColor(frame->rgb, grayimage, CV_BGR2GRAY);
	feature_detector.detect(grayimage, frame->keypoints);
	feature_extractor.compute(grayimage, frame->keypoints, frame->descriptor);
//	cout << "size" << frame->keypoints.size() << endl;
}

void FeatureDetect::Detect_sift(RGBDFrame::Ptr& frame) {

	SiftFeatureDetector detector(300);
	detector.detect( frame->rgb, frame->keypoints );
	SiftDescriptorExtractor extractor;
	extractor.compute( frame->rgb, frame->keypoints, frame->descriptor );
}

void FeatureDetect::Detect_surf(RGBDFrame::Ptr& frame) {

	SurfFeatureDetector detector(300,1);
	detector.detect( frame->rgb, frame->keypoints );
	SurfDescriptorExtractor extractor;
	extractor.compute( frame->rgb, frame->keypoints, frame->descriptor );
}
/*
void FeatureDetect::Detect_surf_block(RGBDFrame::Ptr& frame) {

	SurfFeatureDetector detector(100);
	SurfDescriptorExtractor extractor;

//	detector.detect( frame->rgb, frame->keypoints );
//	extractor.compute( frame->rgb, frame->keypoints, frame->descriptor );

	detector.detect( frame->rgb(Rect(0,0,320,240)), frame->keypoints1 );
	extractor.compute( frame->rgb(Rect(0,0,320,240)), frame->keypoints1, frame->descriptor1 );

	detector.detect( frame->rgb(Rect(320,0,320,240)), frame->keypoints2 );
	extractor.compute( frame->rgb(Rect(320,0,320,240)), frame->keypoints2, frame->descriptor2 );

	detector.detect( frame->rgb(Rect(0,240,320,240)), frame->keypoints3 );
	extractor.compute( frame->rgb(Rect(0,240,320,240)), frame->keypoints3, frame->descriptor3 );

	detector.detect( frame->rgb(Rect(320,240,320,240)), frame->keypoints4 );
	extractor.compute( frame->rgb(Rect(320,240,320,240)), frame->keypoints4, frame->descriptor4 );

	for (size_t i = 0; i<frame->keypoints2.size(); i++)
	{
		frame->keypoints2.at(i).pt.x += 320;
	}
	for (size_t i = 0; i<frame->keypoints3.size(); i++)
	{
		frame->keypoints3.at(i).pt.y += 240;
	}
	for (size_t i = 0; i<frame->keypoints4.size(); i++)
	{
		frame->keypoints4.at(i).pt.x += 320;
		frame->keypoints4.at(i).pt.y += 240;
	}

}*/

RESULT_OF_PNP FeatureDetect::Match_orb(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {

	RESULT_OF_PNP result;
	vector< cv::DMatch > matches;
//	cv::FlannBasedMatcher matcher;
	BruteForceMatcher<HammingLUT> matcher;
	matcher.match(src->descriptor, dst->descriptor, matches);

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

	vector<cv::Point3f> pts_src;
	vector< cv::Point2f > pts_dst;
	int flag = 0;
	for (size_t i = 0; i < goodMatches.size(); i++)
	{
		cv::Point2f p = src->keypoints[goodMatches[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints[goodMatches[i].trainIdx].pt));
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
    cv::waitKey(1);

	double camera_matrix_data[3][3] = {
		{ cam.fx, 0, cam.cx },
		{ 0, cam.fy, cam.cy },
		{ 0, 0, 1 }
	};

	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_src, pts_dst, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;

	return result;
}

RESULT_OF_PNP FeatureDetect::Match_sift(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {
	// TODO
	RESULT_OF_PNP result;
	return result;
}

RESULT_OF_PNP FeatureDetect::Match_surf(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {

	RESULT_OF_PNP result;
	vector< cv::DMatch > matches;
	FlannBasedMatcher matcher;
	matcher.match(src->descriptor, dst->descriptor, matches);
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

	vector<cv::Point3f> pts_src;
	vector< cv::Point2f > pts_dst;
	int flag = 0;
	for (size_t i = 0; i < goodMatches.size(); i++)
	{
		cv::Point2f p = src->keypoints[goodMatches[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints[goodMatches[i].trainIdx].pt));
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
    cv::waitKey(1);

	double camera_matrix_data[3][3] = {
		{ cam.fx, 0, cam.cx },
		{ 0, cam.fy, cam.cy },
		{ 0, 0, 1 }
	};

	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_src, pts_dst, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;


	return result;
}
/*
RESULT_OF_PNP FeatureDetect::Block_match(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam) {

	cout << "hello" << endl;
	RESULT_OF_PNP result;
	FlannBasedMatcher matcher;
	vector< cv::DMatch > goodMatches1;
	vector< cv::DMatch > goodMatches2;
	vector< cv::DMatch > goodMatches3;
	vector< cv::DMatch > goodMatches4;
	double minDis;


	vector< cv::DMatch > matches1;
	matcher.match(src->descriptor1, dst->descriptor1, matches1);
	minDis = 9999;
	for (size_t i = 0; i<matches1.size(); i++)
	{
		if (matches1[i].distance < minDis)
			minDis = matches1[i].distance;
	}
	for (size_t i = 0; i<matches1.size(); i++)
	{
		if (matches1[i].distance < good_match_threshold*minDis)
			goodMatches1.push_back(matches1[i]);
	}


	vector< cv::DMatch > matches2;
	matcher.match(src->descriptor2, dst->descriptor2, matches2);
	minDis = 9999;
	for (size_t i = 0; i<matches2.size(); i++)
	{
		if (matches2[i].distance < minDis)
			minDis = matches2[i].distance;
	}
	for (size_t i = 0; i<matches2.size(); i++)
	{
		if (matches2[i].distance < good_match_threshold*minDis)
			goodMatches2.push_back(matches2[i]);
	}


	vector< cv::DMatch > matches3;
	matcher.match(src->descriptor3, dst->descriptor3, matches3);
	minDis = 9999;
	for (size_t i = 0; i<matches3.size(); i++)
	{
		if (matches3[i].distance < minDis)
			minDis = matches3[i].distance;
	}
	for (size_t i = 0; i<matches3.size(); i++)
	{
		if (matches3[i].distance < good_match_threshold*minDis)
			goodMatches3.push_back(matches3[i]);
	}


	vector< cv::DMatch > matches4;
	matcher.match(src->descriptor4, dst->descriptor4, matches4);
	minDis = 9999;
	for (size_t i = 0; i<matches4.size(); i++)
	{
		if (matches4[i].distance < minDis)
			minDis = matches4[i].distance;
	}
	for (size_t i = 0; i<matches4.size(); i++)
	{
		if (matches4[i].distance < good_match_threshold*minDis)
			goodMatches4.push_back(matches4[i]);
	}

	if (goodMatches1.size()+goodMatches2.size()+goodMatches3.size()+goodMatches4.size() <= 5)
	{
		result.inliers = -1;
		return result;
	}

	cout << "hello1" << endl;
	vector<cv::Point3f> pts_src;
	vector< cv::Point2f > pts_dst;
	int flag = 0;
	cout << goodMatches1.size() << endl;
	for (size_t i = 0; i < goodMatches1.size(); i++)
	{
	//	cout << i << endl;
//		cout << src->keypoints[goodMatches1[i].queryIdx].pt.x << endl;
		cv::Point2f p = src->keypoints1[goodMatches1[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints1[goodMatches1[i].trainIdx].pt));
		flag++;
	}
	cout << "hello2" << endl;
	for (size_t i = 0; i < goodMatches2.size(); i++)
	{
		cv::Point2f p = src->keypoints2[goodMatches2[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints2[goodMatches2[i].trainIdx].pt));
		flag++;
	}
	cout << "hello3" << endl;
	for (size_t i = 0; i < goodMatches3.size(); i++)
	{
		cv::Point2f p = src->keypoints3[goodMatches3[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints3[goodMatches3[i].trainIdx].pt));
		flag++;
	}
	for (size_t i = 0; i < goodMatches4.size(); i++)
	{
		cv::Point2f p = src->keypoints4[goodMatches4[i].queryIdx].pt;
		cv::Point3f pd = src->project2dTo3dLocal1(p.x, p.y, cam);
		if(pd == cv::Point3f(0,0,0))
			continue;

		pts_src.push_back(pd);
		pts_dst.push_back(cv::Point2f(dst->keypoints4[goodMatches4[i].trainIdx].pt));
		flag++;
	}
	if (flag <= 5)
	{
		result.inliers = -1;
		return result;
	}
	cout << "hello3" << endl;
//	Mat img_matches;
//    drawMatches(src->rgb,src->keypoints,dst->rgb,dst->keypoints,matches,img_matches);
//    imshow("matches",img_matches);
	Mat temp;
	drawKeypoints(dst->rgb, dst->keypoints1, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(temp, dst->keypoints2, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(temp, dst->keypoints3, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(temp, dst->keypoints4, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	imshow("temp",temp);
	cv::waitKey(1);

	double camera_matrix_data[3][3] = {
		{ cam.fx, 0, cam.cx },
		{ 0, cam.fy, cam.cy },
		{ 0, 0, 1 }
	};
	cout << "hello4" << endl;
	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_src, pts_dst, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;


	return result;
}*/

int FeatureDetect::Key_Frame_Judge(RESULT_OF_PNP result_of_pnp) {
	if(result_of_pnp.inliers < min_inliers)
		return 1;

	double norm = normofTransform(result_of_pnp.rvec, result_of_pnp.tvec);
	if(norm < keyframe_threshold)
		return 2;
	if(norm > max_norm)
		return 3;

	return 0;
}

double FeatureDetect::Key_Frame_Local(RESULT_OF_PNP result_of_pnp) {
	double norm = normofTransform(result_of_pnp.rvec, result_of_pnp.tvec);
	if(norm > max_norm)
		return 1;
	else
		return norm;
}
