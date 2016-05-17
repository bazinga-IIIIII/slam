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
//#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cstdlib>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <g2o/core/g2o_core_api.h>
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/marginal_covariance_cholesky.h>

using namespace rgbd_tutor;
using namespace cv;
using namespace g2o;


//g2o::SparseOptimizer m_globalOptimizer;
g2o::SparseOptimizer m_globalOptimizer1;
/*
typedef BlockSolver_6_3 SlamBlockSolver;
typedef LinearSolverCSparse< BlockSolver_6_3::PoseMatrixType > SlamLinearSolver;

	SlamLinearSolver* linearSolver;
	SlamBlockSolver* blockSolver;
	OptimizationAlgorithmLevenberg* solver;
/////// 		SparseOptimizer m_globalOptimizer;
	VertexSE3* v;
	Eigen::Matrix<double, 6, 6> information;

void init_g2o()
{
	linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering( false );
	blockSolver = new SlamBlockSolver( linearSolver );
	solver = new OptimizationAlgorithmLevenberg( blockSolver );
	m_globalOptimizer.setAlgorithm( solver );
	m_globalOptimizer.setVerbose( false );

	v = new VertexSE3();
	v->setId( 1 );
	v->setEstimate( Eigen::Isometry3d::Identity() );
	v->setFixed( true );
	m_globalOptimizer.addVertex( v );
	}
*/


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


int temp1()//int main()
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


bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
	protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

//Doing non-trivial things in signal handler is bad. If you want to pause,
//do it in another thread.
//Though libusb operations are generally thread safe, I cannot guarantee
//everything above is thread safe when calling start()/stop() while
//waitForNewFrame().
void sigusr1_handler(int s)
{
	if (devtopause == 0)
		return;
	/// [pause]
	if (protonect_paused)
		devtopause->start();
	else
		devtopause->stop();
	protonect_paused = !protonect_paused;
	/// [pause]
}

int main() {
	/// [context]
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	/// [context]

	std::string serial = "";

	bool viewer_enabled = true;
	bool enable_rgb = true;
	bool enable_depth = true;
	int deviceId = -1;
	size_t framemax = -1;
	if (!enable_rgb && !enable_depth)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	/// [discovery]
	if(freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	if (serial == "")
	{
		serial = freenect2.getDefaultDeviceSerialNumber();
	}
	/// [discovery]

	if(pipeline)
	{
		/// [open]
		dev = freenect2.openDevice(serial, pipeline);
		/// [open]
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}

	if(dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause = dev;

	signal(SIGINT,sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb)
		types |= libfreenect2::Frame::Color;
	if (enable_depth)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb && enable_depth)
	{
		if (!dev->start())
			return -1;
	}
	else
	{
		if (!dev->startStreams(enable_rgb, enable_depth))
			return -1;
	}

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	/// [start]


	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;

//	init_g2o();

	Mat rgb_image(1080, 1920, CV_8UC4);
	Mat depth_image(424, 512, CV_32FC1);
	Mat rgb_image1, rgb_image2;

	////////////////////////////////////////////////////////////////////////////////////////add
	int start_flag = 1;

	RGBDFrame::Ptr old_frame(new RGBDFrame);
	RGBDFrame::Ptr frame(new RGBDFrame);

	FeatureDetect fd;
	Transform tf;
	RESULT_OF_PNP result;


    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = 261.696594;
    camera.cy = 202.522202;
    camera.fx = 368.096588;
    camera.fy = 368.096588;
    camera.scale=1000.0;



	////////////////////////////////////////////////////////////////////////////////////////add

	/// [loop start]
	while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb && enable_depth)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			/// [registration]
		}

		rgb_image.cols = rgb->width;
		rgb_image.rows = rgb->height;
		rgb_image.data = rgb->data;
//		imshow("rgb", rgb_image);


		depth_image.cols = depth->width;
		depth_image.rows = depth->height;
		depth_image.data = depth->data;
		imshow("depth",depth_image);

		resize(rgb_image, rgb_image1, Size(512,424), 512/1920, 424/1080, INTER_AREA);
		cvtColor(rgb_image1, rgb_image2, CV_BGRA2BGR);
		imshow("rgb1",rgb_image2);
		waitKey(10);


		if(start_flag) {
	        old_frame->rgb = rgb_image2.clone();
	        old_frame->depth = depth_image.clone();
	    	fd.Detect_orb(old_frame);
	        start_flag = 0;
		}
		else {
	        frame->rgb = rgb_image2.clone();
	        frame->depth = depth_image.clone();

	    	fd.Detect_orb(frame);
//	    	cout << frame->keypoints.size() << endl;
	    	drawKeypoints(frame->rgb, frame->keypoints, frame->rgb, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	        cv::imshow( "image", frame->rgb );
	        cv::waitKey(10);

	        result = fd.Match_orb(old_frame, frame, camera);
	        int result_k = fd.Key_Frame_Judge(result);
	        cout << "k   " << result_k << endl;
	        if(!result_k) {
//	            cout << "lalalalalal  " << fd.Key_Frame_Judge(result) << endl;
	        	tf.GetFrameTransform(*frame, *old_frame, result);

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


		}


		framecount++;
		if (!viewer_enabled)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		/// [loop end]
		listener.release(frames);
		/** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
	}
	/// [loop end]
	dev->stop();
	dev->close();
	/// [stop]

	delete registration;

	return 0;

	cout << "hello" << endl;
}
