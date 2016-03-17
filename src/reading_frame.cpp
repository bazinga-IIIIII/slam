/*
 * reading_frame.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: wei
 */
#include "rgbdframe.h"

using namespace rgbd_tutor;

int main()
{
    ParameterReader para;
    FrameReader     fr(para);
    while( RGBDFrame::Ptr frame = fr.next() )
    {
        cv::imshow( "image", frame->rgb );
        cv::waitKey(1);
    }

    return 0;
}


