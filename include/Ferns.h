/*
 * Ferns.h
 *
 *  Created on: Jun 15, 2016
 *      Author: wei
 */

#ifndef FERNS_H_
#define FERNS_H_

#include <random>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <limits>

#include "rgbdframe.h"
using namespace rgbd_tutor;

class Ferns {
public:
    Ferns(int n, int maxDepth, const float photoThresh);
    virtual ~Ferns();

    bool addFrame(RGBDFrame::Ptr frame, const float threshold);

//    Eigen::Matrix4f findFrame(RGBDFrame::Ptr frame);
    int findFrame(RGBDFrame::Ptr frame);

    class Fern
    {
        public:
            Fern()
            {}

            Eigen::Vector2i pos;
            Eigen::Vector4i rgbd;
            std::vector<int> ids[16];//一个向量数组？
    };

    std::vector<Fern> conservatory;

    class Frame
    {
        public:
        Frame(int n, int id, int global_id)//, cv::Mat rgb, cv::Mat depth)
         : goodCodes(0),
           id(id),
		   global_id(global_id)
//		   rgb(rgb),
//		   depth(depth)
        {
            codes = new unsigned char[n];
        }

            virtual ~Frame()
            {
                delete [] codes;
            }

            unsigned char * codes;
            int goodCodes;
            const int id;
 //           cv::Mat rgb;
 //           cv::Mat depth;
            const int global_id;
    };

    std::vector<Frame*> frames;

    const int num;
    std::mt19937 random;  //mt19937是Mersenne Twister算法，译为马特赛特旋转演算法。用来生成伪随机数，使用需要初始化
    const int factor;	  //resize的缩放因子
    const int width;
    const int height;
    const int maxDepth;
    const float photoThresh;

//产生均匀分布的整数
    std::uniform_int_distribution<int32_t> widthDist;
    std::uniform_int_distribution<int32_t> heightDist;
    std::uniform_int_distribution<int32_t> rgbDist;
    std::uniform_int_distribution<int32_t> dDist;

    int lastClosest;
    const unsigned char badCode;

    void generateFerns();

    float blockHD(const Frame * f1, const Frame * f2);
    float blockHDAware(const Frame * f1, const Frame * f2);


};

#endif /* FERNS_H_ */
