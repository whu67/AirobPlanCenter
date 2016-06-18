/*
 * MapDealer.h
 *
 *  Created on: May 17, 2016
 *      Author: root
 */

#ifndef MAPDEALER_H_
#define MAPDEALER_H_

#include <sys/types.h>
#include <pthread.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <memory.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>

#include "unit.h"
#include "astar_algorithm.h"

#define MAX_MAP_GRID 160

using namespace std;

struct MapData
{
	friend bool operator <(MapData n1, MapData n2)
	{
		return n1.priority > n2.priority;
	}
	int priority;
	char *data;
	int data_len;
	MapData()
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

class MapDealer
{
public:
	MapDealer();
	virtual ~MapDealer();

	unsigned char Fix[MAX_MAP_GRID][MAX_MAP_GRID];
	unsigned char tmpFix[MAX_MAP_GRID][MAX_MAP_GRID];

	int open(MainProgram *lp);
	void init();
	void AddToRevMapDataQueue(const char * data, int data_len, int priority = 10);
	void AddToRevPlanOrderQueue(const char * data, int data_len, int priority = 10);
	bool CreateUpdateMapandPlanThread();
	int UpdateMapProcess(const MapData* msg);
	int RoadPlanProcess(const MapData* msg);

	void ClearMap();

	int mapdump;
	int pathdump;
public:
	MainProgram* m_MainProgram;

	priority_queue<MapData> RevMapDataQueue;
	priority_queue<MapData> RevPlanOrderQueue;

	pthread_mutex_t RevMapDataQueuemutex;
	pthread_mutex_t RevPlanOrderQueuemutex;
	pthread_mutex_t MapFixmutex;

private:
	pthread_mutex_t UpdateMapThreadmutex;
	pthread_cond_t UpdateMapThreadEvent;

	pthread_mutex_t RoadPlanThreadmutex;
	pthread_cond_t RoadPlanThreadEvent;

	pthread_t m_UpdateMapThreadHandle; //通讯线程句柄
	pthread_t m_RoadPlanThreadHandle; //通讯线程句柄

private:
	static void * UpdateMapFunc(void * lparam);
	static void * RoadPlanFunc(void * lparam);

};

#endif /* MAPDEALER_H_ */
