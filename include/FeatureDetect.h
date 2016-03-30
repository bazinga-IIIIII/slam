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
	OrbFeatureDetector feature_detector;
	OrbDescriptorExtractor feature_extractor;
	double good_match_threshold;

public:
	void Detect_orb(RGBDFrame::Ptr& frame);
	void Detect_sift(RGBDFrame::Ptr& frame);
	void Detect_surf(RGBDFrame::Ptr& frame);
	RESULT_OF_PNP Match_orb(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	RESULT_OF_PNP Match_sift(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
	RESULT_OF_PNP Match_surf(RGBDFrame::Ptr& src, RGBDFrame::Ptr& dst, CAMERA_INTRINSIC_PARAMETERS cam);
};

#endif /* FEATUREDETECT_H_ */
