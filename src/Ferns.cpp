/*
 * Ferns.cpp
 *
 *  Created on: Jun 15, 2016
 *      Author: wei
 */

#include <Ferns.h>

Ferns::Ferns(int n, int maxDepth, const float photoThresh)  //  冒号后面为初始化Ferns的成员变量
 : num(n),
   factor(8),
   width(640),
   height(480),
   maxDepth(maxDepth),
   photoThresh(photoThresh),
   widthDist(0, width - 1),
   heightDist(0, height - 1),
   rgbDist(0, 255),
   dDist(400, maxDepth),
   lastClosest(-1),
   badCode(255)
{
    random.seed(time(0));  //随机数生成初始化
    generateFerns();
}

Ferns::~Ferns() {
	// TODO Auto-generated destructor stub
    for(size_t i = 0; i < frames.size(); i++)
    {
        delete frames.at(i);
    }
}

void Ferns::generateFerns()
{
    for(int i = 0; i < num; i++)
    {
        Fern f;

        f.pos(0) = widthDist(random);   //初始化widthDist(0, width - 1),   均匀生成【0，width-1】之间的随机数
        f.pos(1) = heightDist(random);

        f.rgbd(0) = rgbDist(random);
        f.rgbd(1) = rgbDist(random);
        f.rgbd(2) = rgbDist(random);
        f.rgbd(3) = dDist(random);

        conservatory.push_back(f);
    }
}

bool Ferns::addFrame(RGBDFrame::Ptr i_frame, const float threshold)
{
//	cout << "id  " << i_frame->id << endl;
    Frame * frame = new Frame(num, frames.size(), i_frame->id);//, i_frame->rgb, i_frame->depth);
//    cout << "size " << frames.size() << endl;

    int * coOccurrences = new int[frames.size()];
    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;
	    //判断当有深度值的时候，进行编码

        int row = conservatory.at(i).pos(1);
        int col = conservatory.at(i).pos(0);
        if(i_frame->depth.at<short>(row, col) >0) {
	    //将pos点的rgb值赋给pix
            code = (i_frame->rgb.at<cv::Vec3b>(row, col)[0] > conservatory.at(i).rgbd(0)) << 3 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[1] > conservatory.at(i).rgbd(1)) << 2 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[2] > conservatory.at(i).rgbd(2)) << 1 |
                   (i_frame->depth.at<short>(row, col) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;
	        //这里code值为0000-1111即0-15				     //500颗蕨中，每颗蕨的ids中都存了1,2,3,4......frame_size
     //       cout << "size " << conservatory.at(i).ids[code].size() << endl;
            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)  //假设编码为10，查找第i颗蕨中ids=10的所有frame编号
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;         //co-occurrences数组里，查到编号的位置对应+1
            }
        }
        //每一个frame存500个code作为frame的成员变量
        frame->codes[i] = code;
//        if(code != 255)
//        	printf("%d   %u\n", i, code);
    }

    float minimum = std::numeric_limits<float>::max();//产生一个大数

    if(frame->goodCodes > 0)  //计算最小dissim   //最近邻？
    {
        for(size_t i = 0; i < frames.size(); i++)    //在所有历史帧里，计算dissim
        {
            float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);    //分母是和第i帧相比，取当前帧和第i帧的goodcode的最小值，其实就是500个蕨中有效蕨的个数

            float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;      //coOccurrences[i]=第i帧中有多少个蕨与当前帧相同
										  //dissim也就是差异度，dissim越小表示相差越少
//            cout << "here  " << maxCo << "  " << coOccurrences[i] << "  " << frame->goodCodes << "  " << frames.at(i)->goodCodes << endl;
            if(dissim < minimum)
            {
                minimum = dissim;
            }
        }
    }
//    cout << "delete" << endl;
    delete [] coOccurrences;
//    cout << minimum << endl;

//    if((minimum > threshold || frames.size() == 0) && frame->goodCodes > 0)
    if(frame->goodCodes > 0)     //在goodcode>0即帧有效的情况下，首帧和dissim大于与阈值的帧将会被push进容器
    {
        for(int i = 0; i < num; i++)						//然后将新添加帧的帧号 push到 有效蕨的ids容器中
        {
            if(frame->codes[i] != badCode)
            {
                conservatory.at(i).ids[frame->codes[i]].push_back(frame->id);
            }
        }
//        cout << "size " << frames.size() << endl;
        frames.push_back(frame);
//        cout << "size " << frames.size() << endl;

        return true;
    }
    else
    {
 //   	cout << "fffffffffffffff" << endl;
        delete frame;

        return false;
    }
}


int Ferns::findFrame(RGBDFrame::Ptr i_frame)
{
    lastClosest = -1;
    Frame * frame = new Frame(num, frames.size(), i_frame->id);

    int * coOccurrences = new int[frames.size()];
    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        int row = conservatory.at(i).pos(1);
        int col = conservatory.at(i).pos(0);
        if(i_frame->depth.at<short>(row, col) >0) {
	    //将pos点的rgb值赋给pix
            code = (i_frame->rgb.at<cv::Vec3b>(row, col)[0] > conservatory.at(i).rgbd(0)) << 3 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[1] > conservatory.at(i).rgbd(1)) << 2 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[2] > conservatory.at(i).rgbd(2)) << 1 |
                   (i_frame->depth.at<short>(row, col) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

    float minimum = std::numeric_limits<float>::max();
    int minId = -1;

    for(size_t i = 0; i < frames.size()-1; i++)
    {
        float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

        float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

        if(dissim < minimum )//&& 1==2)//time - frames.at(i)->srcTime > 300)    //查找差异度最小的帧，300ms以内的帧不考虑，差异最小帧帧号赋值给minID
        {
            minimum = dissim;
            minId = i;
        }
    }

    delete [] coOccurrences;

    Eigen::Matrix4f estPose = Eigen::Matrix4f::Identity();
									//blockHDAware返回值【0,1】，数值越大，相似度越高
//    cout << "minID:" << minId << endl;
//    cout << "global_ID:" << frames.at(minId)->global_id << endl;
    if(minId != -1 && blockHDAware(frame, frames.at(minId)) > 0.3)      //差异最小帧帧号非空，并且块间汉明距离大于阈值，blabla
    {
    	delete frame;
    	return minId;
    }
    else {
    	delete frame;
    	return -1;
    }
//    delete frame;
}

void Ferns::findFrame_k(RGBDFrame::Ptr i_frame, int a[5])
{
//	int a[5] = {-1,-1,-1,-1,-1};

    lastClosest = -1;
    Frame * frame = new Frame(num, frames.size(), i_frame->id);

    int * coOccurrences = new int[frames.size()];
    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        int row = conservatory.at(i).pos(1);
        int col = conservatory.at(i).pos(0);
        if(i_frame->depth.at<short>(row, col) >0) {
	    //将pos点的rgb值赋给pix
            code = (i_frame->rgb.at<cv::Vec3b>(row, col)[0] > conservatory.at(i).rgbd(0)) << 3 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[1] > conservatory.at(i).rgbd(1)) << 2 |
                   (i_frame->rgb.at<cv::Vec3b>(row, col)[2] > conservatory.at(i).rgbd(2)) << 1 |
                   (i_frame->depth.at<short>(row, col) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

//    int minId = -1;

    std::vector<Frame*> frames_temp;
    for(int i=0; i < frames.size()-1; i++)
    	frames_temp.push_back(frames.at(i));

    cout << "find match id1:"<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<" "<<a[4]<<endl;
    for(int i=0; i < 5; i++) {
    	float minimum = std::numeric_limits<float>::max();
    	for(int j=0; j < frames_temp.size(); j++) {
            float maxCo = std::min(frame->goodCodes, frames_temp.at(j)->goodCodes);
            float dissim = (float)(maxCo - coOccurrences[j]) / (float)maxCo;
            if(dissim < minimum ) {
                minimum = dissim;
                a[i] = j;
            }
    	}
    	coOccurrences[a[i]] = 0;
 //   	frames_temp.at(a[i])->goodCodes = -1;
    }
    cout << "find match id2:"<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<" "<<a[4]<<endl;
    if(frames_temp.size() < 5)
    	for(int i=0; i<5-frames_temp.size(); i++)
    		a[frames_temp.size()+i] = -1;

    delete [] coOccurrences;

	cout << "find match id3:"<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<" "<<a[4]<<endl;
    for(int i=0; i < 5; i++) {
    	if(a[i] != -1 && blockHDAware(frame, frames.at(a[i])) <= 0.3)
    		a[i] = -1;
    }
	cout << "find match id4:"<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<" "<<a[4]<<endl;
//    for(size_t i = 0; i < frames_temp.size(); i++)
//    {
//        delete frames_temp.at(i);
//    }
	frames_temp.clear();
    delete frame;

//    cout << "find match id:"<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<" "<<a[4]<<endl;
 //   return a;
}

float Ferns::blockHDAware(const Frame * f1, const Frame * f2)
{
    int count = 0;
    float val = 0;

    for(int i = 0; i < num; i++)
    {
        if(f1->codes[i] != badCode && f2->codes[i] != badCode)
        {
            count++;

            if(f1->codes[i] == f2->codes[i])
            {
                val += 1.0f;
            }
        }
    }

    return val / (float)count;
}

