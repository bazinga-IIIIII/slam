/*
 * Transform.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: wei
 */

#include <Transform.h>

Transform::Transform() {
	// TODO Auto-generated constructor stub

}

Transform::~Transform() {
	// TODO Auto-generated destructor stub
}

Eigen::Isometry3d Transform::PnpoutputToIsometry(RESULT_OF_PNP result) {
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	cv::Mat R;
	cv::Rodrigues(result.rvec, R);
	Eigen::Matrix3d r;
	cv::cv2eigen(R, r);
	Eigen::AngleAxisd angle(r);
	T = angle;
	T(0, 3) = result.tvec.at<double>(0, 0);
	T(1, 3) = result.tvec.at<double>(1, 0);
	T(2, 3) = result.tvec.at<double>(2, 0);
	return T;
}

Eigen::Matrix4d Transform::IsometryToMatrix4d(Eigen::Isometry3d T) {
	Eigen::Matrix4d m;
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			m(i,j) = T(i,j);
	for(i=0;i<3;i++) {
		m(i,3) = T(i,3);
		m(3,i) = 0;
	}
	m(3,3) = 1;
	return m;
}

Eigen::Matrix4d Transform::PnpoutputToMatrix4d(RESULT_OF_PNP result) {
	Eigen::Matrix4d m;
	Eigen::Isometry3d T;
	T = PnpoutputToIsometry(result);
	m = IsometryToMatrix4d(T);
	return m;
}

Eigen::Matrix4d Transform::TransformToMatrix4d(Eigen::Vector3d v, Eigen::Quaterniond q) {
	Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
	Eigen::Matrix3d m3 = q.toRotationMatrix();
	m.block(0,0,3,3) = m3;
	m.block<3,1>(0,3) = v;
	return m;
}

Eigen::Quaterniond Transform::Matrix9ToQuaternion(Eigen::Matrix3d m) {
	Eigen::Quaterniond q(m);
//	q = m;
	return q;
}

Eigen::Quaterniond Transform::Matrix16ToQuaternion(Eigen::Matrix4d m) {
	Eigen::Quaterniond q;
	q = Matrix9ToQuaternion(m.block<3,3>(0,0));
	return q;
}

Eigen::Quaterniond Transform::IsometryToQuaternion(Eigen::Isometry3d T) {
	Eigen::Quaterniond q;
	Eigen::Matrix3d m;
	m = T.rotation();
	q = Matrix9ToQuaternion(m);
	return q;
}

Eigen::Vector3d Transform::Matrix16ToTranslation(Eigen::Matrix4d m) {
	Eigen::Vector3d v;
	v = m.block<3,1>(0,3);
	return v;
}

void Transform::GetFrameTransform(RGBDFrame &frame, RGBDFrame &old_frame, RESULT_OF_PNP result) {
	Eigen::Matrix4d m;
	m = PnpoutputToMatrix4d(result);
	Eigen::Matrix4d m_old;
	m_old = TransformToMatrix4d(old_frame.translation, old_frame.rotation);
	frame.rotation = Matrix16ToQuaternion(m * m_old);
	frame.translation = Matrix16ToTranslation(m * m_old);
}

void Transform::PrintQuar(Eigen::Quaterniond q) {
	cout << "Quaternions: ";
	cout << q.w() <<"  ";
	cout << q.x() <<"  ";
	cout << q.y() <<"  ";
	cout << q.z() << endl;
}

void Transform::PrintIsometry(Eigen::Isometry3d T) {
	cout << "The Rotation Matrix:" << endl;
	cout << T.rotation() << endl;
	cout << "The Translation Matrix:" << endl;
	cout << T.translation() << endl;
}

