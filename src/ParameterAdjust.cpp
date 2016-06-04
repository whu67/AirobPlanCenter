/*
 * ParameterAdjust.cpp
 *
 *  Created on: May 25, 2016
 *      Author: root
 */
#include "ParameterAdjust.h"

#include <queue>
#include <set>
#include <map>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <math.h>
#include <stdlib.h>
using namespace cv;

#define DEBUGPHOTO

CvCapture* capture;
std::map<int, Mat> PhotoMap;

ParameterAdjust::ParameterAdjust()
{
	// TODO Auto-generated constructor stub
	pthread_mutex_init(&AdjustThreadmutex, NULL);
	pthread_cond_init(&AdjustThreadEvent, NULL);
	pthread_mutex_init(&RevAdjustOrderQueuemutex, NULL);
}

ParameterAdjust::~ParameterAdjust()
{
	// TODO Auto-generated destructor stub
}

int ParameterAdjust::open(MainProgram* lp)
{
	m_MainProgram = lp;
	return 1;
}

bool ParameterAdjust::init()
{
	PhotoMap.clear();
	maxError = 0.06f;

	capture = cvCreateCameraCapture(CV_CAP_ANY);
	if(NULL == capture)
		return false;
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 360);
	return true;
}

void ParameterAdjust::ClearPhotoMap()
{
	PhotoMap.clear();
}

void ParameterAdjust::AddToAdjustQueue(const char* data, int data_len, int priority)
{

}

bool ParameterAdjust::GetPhoto(unsigned char Xindex, unsigned char Yindex)
{
	IplImage* frame;
	frame = cvQueryFrame(capture);
	if(!frame)
	{
		//return failure of get photo to robot
		//m_MainProgram->m_CommHelper.SendData();
		return false;
	}
	else
	{
#ifdef DEBUGPHOTO
		cvShowImage("vedio",frame);
		return true;
#endif
		Mat img(frame,true);
		int x = Xindex;
		int y = Yindex;
		char indexbuffer[10];
		int index;
		sprintf(indexbuffer, "%d%d", x, y);
		index = atoi(indexbuffer);
		fprintf(stdout, "photo index is %d.\n", index);
		PhotoMap.insert(std::pair<int, Mat>(index, img));
	}
}

void *ParameterAdjust::ParameterAdjustThreadFunc(void * lparam)
{
	ParameterAdjust *pAdjust;
	//得到ParameterAdjust实例指针
	pAdjust = (ParameterAdjust*) lparam;

	while (true)
	{

		if (pAdjust->RevAdjustDataQueue.size() <= 0)
		{
			pthread_mutex_lock(&pAdjust->AdjustThreadmutex);
			pthread_cond_wait(&pAdjust->AdjustThreadEvent, &pAdjust->AdjustThreadmutex);
			pthread_mutex_unlock(&pAdjust->AdjustThreadmutex);

		}
		else
		{
			while (!pAdjust->RevAdjustDataQueue.empty())
			{
				pthread_mutex_lock(&pAdjust->RevAdjustOrderQueuemutex);

//				pAdjust->MsgProcess(&pAdjust->RevAdjustDataQueue.top());
//
//				if (pAdjust->RevAdjustDataQueue.top().data != NULL)
//				{
//					delete (pAdjust->RevAdjustDataQueue.top().data);
//				}
//
//				pAdjust->RevAdjustDataQueue.pop();

				pthread_mutex_unlock(&pAdjust->RevAdjustOrderQueuemutex);
			}
		}

	}
	pthread_exit(NULL);
	return 0;
}

bool ParameterAdjust::CreateAdjustThread()
{
	int ret = pthread_create(&m_AdjustThreadHandle, NULL, ParameterAdjustThreadFunc, (void*) this);
	if (ret)
	{
		fprintf(stdout, "Create ParameterAdjustThreadFunc thread error!\n");
		return false;
	}
	return true;
}
