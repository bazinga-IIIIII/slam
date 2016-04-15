/*
 * Transform.h
 *
 *  Created on: Apr 11, 2016
 *      Author: wei
 */

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include "common.h"
#include "rgbdframe.h"
#include <opencv2/core/eigen.hpp>

using namespace rgbd_tutor;

class Transform {
public:
	Transform();
	virtual ~Transform();

public:
	Eigen::Isometry3d PnpoutputToIsometry(RESULT_OF_PNP result);
	Eigen::Matrix4d IsometryToMatrix4d(Eigen::Isometry3d T);
	Eigen::Matrix4d PnpoutputToMatrix4d(RESULT_OF_PNP result);
	Eigen::Matrix4d TransformToMatrix4d(Eigen::Vector3d v, Eigen::Quaterniond q);

	Eigen::Quaterniond Matrix9ToQuaternion(Eigen::Matrix3d m);
	Eigen::Quaterniond Matrix16ToQuaternion(Eigen::Matrix4d m);
	Eigen::Quaterniond IsometryToQuaternion(Eigen::Isometry3d T);
	Eigen::Vector3d Matrix16ToTranslation(Eigen::Matrix4d m);

	void GetFrameTransform(RGBDFrame &frame, RGBDFrame &old_frame, RESULT_OF_PNP result);

	void PrintQuar(Eigen::Quaterniond q);
	void PrintIsometry(Eigen::Isometry3d T);

//	FILE *fp;
};

#endif /* TRANSFORM_H_ */
