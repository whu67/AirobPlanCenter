/*
 * ParameterAdjust.h
 *
 *  Created on: May 25, 2016
 *      Author: root
 */

#ifndef SRC_PARAMETERADJUST_H_
#define SRC_PARAMETERADJUST_H_

//#include <queue>
//#include <set>
//#include <map>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgproc/types_c.h"
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/legacy/legacy.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
//#include <math.h>
//#include <stdlib.h>
//using namespace cv;

#include <stdlib.h>
#include <queue>
#include <pthread.h>

class MainProgram;

struct AdjustData
{
	friend bool operator <(AdjustData n1, AdjustData n2)
	{
		return n1.priority > n2.priority;
	}
	int priority;
	char *data;
	int data_len;
	AdjustData()
	{
		this->priority = 10;
		this->data_len = 0;
		this->data = NULL;
	}
	void free()
	{
		if (data != NULL)
		{
			delete (data);
			data = NULL;
		}
		data_len = 0;
	}
};

class ParameterAdjust
{
public:
	ParameterAdjust();
	virtual ~ParameterAdjust();

	int open(MainProgram *lp);
	bool init();

	void ClearPhotoMap();
	bool GetPhoto(unsigned char Xindex, unsigned char Yindex);
	void GetPhotoFromMap();

	void AddToAdjustQueue(const char * data, int data_len, int priority = 10);
	bool CreateAdjustThread();

	float maxError;// = 0.06f;

public:
	MainProgram* m_MainProgram;

	std::priority_queue<AdjustData> RevAdjustDataQueue;

private:
	pthread_mutex_t AdjustThreadmutex;
	pthread_mutex_t RevAdjustOrderQueuemutex;
	pthread_cond_t AdjustThreadEvent;

	pthread_t m_AdjustThreadHandle; //通讯线程句柄

private:
	static void *ParameterAdjustThreadFunc(void * lparam);
};

#endif /* SRC_PARAMETERADJUST_H_ */
