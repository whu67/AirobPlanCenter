/*
 * ParameterAdjust.h
 *
 *  Created on: May 25, 2016
 *      Author: root
 */

#ifndef SRC_PARAMETERADJUST_H_
#define SRC_PARAMETERADJUST_H_

#include <stdlib.h>
#include <queue>
#include <pthread.h>
#include "CommHelper.h"
#include "unit.h"


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

class MainProgram;

class ParameterAdjust
{
public:
	ParameterAdjust();
	virtual ~ParameterAdjust();

	int open(CommHelper *lp);
	bool init();

	void ClearPhotoMap();
	bool GetTmpPhoto(unsigned char Xindex, unsigned char Yindex);
	void RemoveFirstPhoto();
	bool GetPhoto(unsigned char Xindex, unsigned char Yindex);
	void GetPhotoFromMap();

	void AddToAdjustQueue(const char * data, int data_len, int priority = 10);
	bool CreateAdjustThread();
	int AdjustProcess(const AdjustData* msg);

	float maxError;// = 0.06f;

	int Adjustcount;
private:
	MainProgram* m_MainProgram;
	CommHelper* m_CommHelperPtr;

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
