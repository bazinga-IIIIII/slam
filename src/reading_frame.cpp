/*
 * reading_frame.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */
#include "rgbdframe.h"
#include "Transform.h"
#include "FeatureDetect.h"
#include <opencv2/core/eigen.hpp>      //for cv2eigen

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace rgbd_tutor;
using namespace cv;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


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
	Eigen::Quaterniond q;
	PointCloud::Ptr cloud;
	PointCloud::Ptr output(new PointCloud());

	FeatureDetect fd;
	Transform tf;

	old_frame = fr.next();
	fd.Detect_orb(old_frame);
	Eigen::Matrix3d m;
	tf.Matrix9ToQuaternion(m);

	cloud = image2PointCloud(old_frame->rgb, old_frame->depth, camera);
//	pcl::visualization::CloudViewer viewer("Cloud viewer");

    while( RGBDFrame::Ptr frame = fr.next() )
    {
//    	break;
    	fd.Detect_orb(frame);
    	fd.Match_orb(old_frame, frame, frame->camera);
    	Mat temp;
    	drawKeypoints(frame->rgb, frame->keypoints, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        cv::imshow( "image", temp);//frame->rgb );
        cv::waitKey(1);

        result = fd.Match_orb(old_frame, frame, camera);

        if(!fd.Key_Frame_Judge(result)) {
            cout << frame->id << "  " << fd.Key_Frame_Judge(result) << endl;
        	tf.GetFrameTransform(*frame, *old_frame, result);
 //       	cout << fr.rgbFiles[frame->id] << endl;
        	fr.FrameWriter(*frame);
 //           cout << rgbFiles[currentIndex] << endl;
 //       	tf.PrintQuar(frame->rotation);
  //      	cout << frame->translation << endl;
 //       	Eigen::Matrix4d temp = tf.PnpoutputToMatrix4d(result);
 //       	frame->rotation = tf.Matrix16ToQuaternion(temp);
//        	cout << frame->id << "  " << fd.Key_Frame_Judge(result) << endl;
 //       	T = tf.PnpoutputToIsometry(result);
 //       	tf.IsometryToMatrix4d(T);
  //      	q = tf.Matrix16ToQuaternion(T);
  //      	cout << frame->translation << endl;
 //       	tf.PrintQuar(frame->rotation);
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
        old_frame->rotation = frame->rotation;
        old_frame->translation = frame->translation;
        old_frame->descriptor = frame->descriptor.clone();
       	old_frame->keypoints.clear();
       	for(int i = 0; i < frame->keypoints.size(); i++) {
       		old_frame->keypoints.push_back(frame->keypoints[i]);
        }
    }

    return 0;
}


