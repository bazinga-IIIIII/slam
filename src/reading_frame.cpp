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

#include <Ferns.h>



using namespace rgbd_tutor;
using namespace cv;
using namespace g2o;


g2o::SparseOptimizer m_globalOptimizer;

typedef BlockSolver_6_3 SlamBlockSolver;
typedef LinearSolverCSparse< BlockSolver_6_3::PoseMatrixType > SlamLinearSolver;

SlamLinearSolver* linearSolver;
SlamBlockSolver* blockSolver;
OptimizationAlgorithmLevenberg* solver;
VertexSE3* v;
Eigen::Matrix<double, 6, 6> information;

void init_g2o() {
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

    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
}
/**/


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct for_g2o
{
	Mat rgb;
	Mat depth;
	int id;
};

PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if (d == 0)
				continue;
			PointT p;

			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;

			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			cloud->points.push_back(p);
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

	return cloud;
}


int main()//int main()
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

	std::vector<RGBDFrame::Ptr> keyframes;

	FeatureDetect fd;
	Transform tf;
	Ferns ferns(500, 7000, 0.2);//500颗随机蕨。深度值上限7000mm。阈值0.2


	old_frame = fr.next();
//	fd.Detect_orb(old_frame);
	fd.Detect_surf(old_frame);
//	fd.Detect_surf_block(old_frame);
	keyframes.push_back(old_frame);
	ferns.addFrame(old_frame, 0.4);

	Eigen::Matrix3d m;
	tf.Matrix9ToQuaternion(m);

	cloud = image2PointCloud(old_frame->rgb, old_frame->depth, camera);
//	pcl::visualization::CloudViewer viewer("Cloud viewer");


    while( RGBDFrame::Ptr frame = fr.next())//time 17-32误差极大
    {
//    	break;

//    	fd.Detect_orb(frame);
//    	result = fd.Match_orb(old_frame, frame, camera);//frame->camera);
    	fd.Detect_surf(frame);
//    	fd.Detect_surf_block(frame);
    	result = fd.Match_surf(old_frame, frame, camera);//frame->camera);
//    	result = fd.Block_match(old_frame, frame, camera);//frame->camera);

 //   	cout << frame->id << endl;
 //   	cout << fd.Key_Frame_Judge(result) << endl;
 //   	Mat temp;
//    	drawKeypoints(frame->rgb, frame->keypoints, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
 //       cv::imshow( "image", temp);//frame->rgb );
  //      cv::waitKey(1);

 //       result = fd.Match_orb(old_frame, frame, camera);
//        cout << fd.Key_Frame_Judge(result) << endl;
        cout << frame->id << "  " << fd.Key_Frame_Judge(result) << endl;
        if(!fd.Key_Frame_Judge(result)) {
//            cout << frame->id << "  " << fd.Key_Frame_Judge(result) << endl;
            keyframes.push_back(frame);
        	ferns.addFrame(frame, 0.4);
        	tf.GetFrameTransform(*frame, *old_frame, result);
        	fr.FrameWriter(*frame);
        }
/*        else if(fd.Key_Frame_Judge(result)==3 && fd.Key_Frame_Judge(fd.Match_orb(keyframes.at(ferns.findFrame(frame)), frame, camera))==0) {
        	cout << "find pnp" << endl;
        }*/
        else if(fd.Key_Frame_Judge(result) == 3) {
        	cout << "relocalization" << "  id:" << frame->id << endl;
 //       	cout <<ferns.findFrame(frame)<< endl;
        	int temp = ferns.findFrame(frame);
//        	result = fd.Match_orb(keyframes.at(temp), frame, camera);
        	result = fd.Match_surf(keyframes.at(temp), frame, camera);
//        	result = fd.Block_match(keyframes.at(temp), frame, camera);
        	if(!fd.Key_Frame_Judge(result)) {
        		cout << "find pnp" << endl;
        		//add key frame
            	tf.GetFrameTransform(*frame, *keyframes.at(temp), result);
                keyframes.push_back(frame);
            	ferns.addFrame(frame, 0.4);
            	fr.FrameWriter(*frame);
        	}
        	else
        		continue;
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
        old_frame->descriptor1 = frame->descriptor1.clone();
        old_frame->descriptor2 = frame->descriptor2.clone();
        old_frame->descriptor3 = frame->descriptor3.clone();
        old_frame->descriptor4 = frame->descriptor4.clone();
       	old_frame->keypoints.clear();
       	old_frame->keypoints1.clear();
       	old_frame->keypoints2.clear();
       	old_frame->keypoints3.clear();
       	old_frame->keypoints4.clear();
       	for(int i = 0; i < frame->keypoints.size(); i++) {
       		old_frame->keypoints.push_back(frame->keypoints[i]);
        }
       	for(int i = 0; i < frame->keypoints1.size(); i++) {
       		old_frame->keypoints1.push_back(frame->keypoints1[i]);
        }
       	for(int i = 0; i < frame->keypoints2.size(); i++) {
       		old_frame->keypoints2.push_back(frame->keypoints2[i]);
        }
       	for(int i = 0; i < frame->keypoints3.size(); i++) {
       		old_frame->keypoints3.push_back(frame->keypoints3[i]);
        }
       	for(int i = 0; i < frame->keypoints4.size(); i++) {
       		old_frame->keypoints4.push_back(frame->keypoints4[i]);
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

int temp2() {  //main function robocup@home demo
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

	cout << dev->getColorCameraParams().cx << endl;
	cout << dev->getIrCameraParams().cx << endl;
	cout << dev->getIrCameraParams().cy << endl;
	cout << dev->getIrCameraParams().fx << endl;
	cout << dev->getIrCameraParams().fy << endl;

	size_t framecount = 0;

	init_g2o();

	vector<PointCloud::Ptr> v_cloud;

    vector<for_g2o> zz;
    for_g2o cs;
    int keyframe_flag = 1;

	Mat rgb_image(1080, 1920, CV_8UC4);
	Mat depth_image(424, 512, CV_16UC1);
	Mat depth_image1(424, 512, CV_32FC1);
	Mat rgb_image1, rgb_image2;
	Mat t1(424, 512, CV_8UC1);
	Mat t2;

	////////////////////////////////////////////////////////////////////////////////////////add
	int start_flag = 1;

	RGBDFrame::Ptr old_frame(new RGBDFrame);
	RGBDFrame::Ptr frame(new RGBDFrame);

	FeatureDetect fd;
	Transform tf;
	RESULT_OF_PNP result;


    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = 258.801;
    camera.cy = 207/629;
    camera.fx = 364.878;
    camera.fy = 364.878;
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


		t2.cols = registered.width;
		t2.rows = registered.height;
		t2.data = registered.data;

		imshow("t2", t2);




		rgb_image.cols = rgb->width;
		rgb_image.rows = rgb->height;
		rgb_image.data = rgb->data;
//		imshow("rgb", rgb_image);


		depth_image1.cols = depth->width;
		depth_image1.rows = depth->height;
		depth_image1.data = depth->data;
		imshow("depth",depth_image1);


		for(int i=0;i<depth_image1.rows;i++)
				for(int j=0;j<depth_image1.cols;j++)
					depth_image.at<short>(i,j) = depth_image1.at<float>(i,j);

		imshow("depth1",depth_image);

		for(int i=0;i<t2.rows;i++)
				for(int j=0;j<t2.cols;j++)
					t1.at<uchar>(i,j) = depth_image1.at<float>(i,j);
		imshow("t1", t1);

		resize(rgb_image, rgb_image1, Size(512,424), 512/1920, 424/1080, INTER_AREA);
		cvtColor(rgb_image1, rgb_image2, CV_BGRA2BGR);
		imshow("rgb1",rgb_image2);
		waitKey(10);

		if(keyframe_flag > 35)
			break;

		if(start_flag) {
	        old_frame->rgb = rgb_image2.clone();
	        old_frame->depth = depth_image.clone();

//	        cs.rgb = old_frame->rgb;
//	        cs.depth = old_frame->depth;
//	        cs.id = 1;
//	        zz.push_back(cs);
	        v_cloud.push_back(image2PointCloud(old_frame->rgb, old_frame->depth, camera));

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
//	        cout << "k   " << result_k << endl;
	        if(!result_k) {
//	            cout << "lalalalalal  " << fd.Key_Frame_Judge(result) << endl;
	        	tf.GetFrameTransform(*frame, *old_frame, result);

	        	keyframe_flag++;
//		        cs.rgb = frame->rgb;
//		        cs.depth = frame->depth;
//		        cs.id = keyframe_flag;
//		        zz.push_back(cs);
	        	v_cloud.push_back(image2PointCloud(frame->rgb, frame->depth, camera));

		        VertexSE3* rd_v = new g2o::VertexSE3();
		        EdgeSE3* rd_edge = new g2o::EdgeSE3();

	            rd_v->setId(keyframe_flag);
	            rd_v->setEstimate(Eigen::Isometry3d::Identity());
	            m_globalOptimizer.addVertex(rd_v);

	            rd_edge->vertices()[0] = m_globalOptimizer.vertex(keyframe_flag-1);//(m_kpkeyframe.size() - 1);
	            rd_edge->vertices()[1] = m_globalOptimizer.vertex(keyframe_flag);
	            static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
	            rd_edge->setRobustKernel(robustKernel);

	            rd_edge->setInformation(information);

	            Eigen::Isometry3d m_parament = Eigen::Isometry3d::Identity();
	            m_parament = tf.PnpoutputToIsometry(result);
	            rd_edge->setMeasurement(m_parament);
	            m_globalOptimizer.addEdge(rd_edge);


	            cout<<"optimizing pose graph, vertices: "<<m_globalOptimizer.vertices().size()<<endl;
	            m_globalOptimizer.save("result_before.g2o");
	            m_globalOptimizer.initializeOptimization();
	            m_globalOptimizer.optimize( 100 ); //可以指定优化步数
	            m_globalOptimizer.save( "result_after.g2o" );
	            cout<<"Optimization done."<<endl;
/**/

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


	pcl::visualization::CloudViewer viewer("Cloud viewer");
	PointCloud::Ptr cloud(new PointCloud());
	PointCloud::Ptr newCloud(new PointCloud());
	PointCloud::Ptr output(new PointCloud());

	int i;
	for(i=0;i<35;i++) {
		VertexSE3* vertex = dynamic_cast<VertexSE3*>(m_globalOptimizer.vertex(i+1));
		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		pose = vertex -> estimate();
		cout << pose.matrix() << endl;
		cout << v_cloud.size() << endl;

		newCloud = v_cloud[i];
		pcl::transformPointCloud( *newCloud, *newCloud, pose.matrix() );
		*output += *newCloud;
/*		if ( i == 0 )
		{
			output->clear();
			*newCloud = *v_cloud[i];
//			pcl::transformPointCloud(*newCloud, *output, pose.matrix());
		}
		else
		{
			newCloud = v_cloud[i];
			pcl::transformPointCloud(*output, *output, pose.matrix());
			*output += *newCloud;
		}
*/

	}
	viewer.showCloud( newCloud );
	while(1)
	{}


	delete registration;

	return 0;

	cout << "hello" << endl;
}
