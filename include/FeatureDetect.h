/*
 * FeatureDetect.h
 *
 *  Created on: Mar 30, 2016
 *      Author: wei
 */

#ifndef FEATUREDETECT_H_
#define FEATUREDETECT_H_

#include "rgbdframe.h"

using namespace rgbd_tutor;
using namespace cv;

class FeatureDetect {
public:
	FeatureDetect();
	virtual ~FeatureDetect();

public:
//	OrbFeatureDetector feature_detector;
//	OrbDescriptorExtractor feature_extractor;
	double good_match_threshold;
	int min_inliers;
	double keyframe_threshold;
	double max_norm;

public:
	void Detect_orb(RGBDFrame::Ptr& frame);
	void Detect_sift(RGBDFrame::Ptr& frame);
	void Detect_surf(RGBDFrame::Ptr& frame);
	void Detect_surf_block(RGBDFrame::Ptr& frame);
	RESULT_OF_PNP Match_orb(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	RESULT_OF_PNP Match_sift(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	RESULT_OF_PNP Match_surf(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	RESULT_OF_PNP Block_match(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	int Key_Frame_Judge(RESULT_OF_PNP result_of_pnp);

public:
	double normofTransform(cv::Mat rvec, cv::Mat tvec) {
		return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
	}
};

#endif /* FEATUREDETECT_H_ */
