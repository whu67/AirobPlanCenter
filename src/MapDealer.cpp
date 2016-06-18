/*
 * MapDealer.cpp
 *
 *  Created on: May 17, 2016
 *      Author: root
 */
#include "main.h"
#include "MapDealer.h"

MapDealer::MapDealer()
{
	// TODO Auto-generated constructor stub
	pthread_mutex_init(&RevMapDataQueuemutex, NULL);
	pthread_mutex_init(&RevPlanOrderQueuemutex, NULL);
	pthread_mutex_init(&MapFixmutex, NULL);

	pthread_cond_init(&UpdateMapThreadEvent, NULL);
	pthread_mutex_init(&UpdateMapThreadmutex, NULL);

	pthread_cond_init(&RoadPlanThreadEvent, NULL);
	pthread_mutex_init(&RoadPlanThreadmutex, NULL);
}

MapDealer::~MapDealer()
{
	// TODO Auto-generated destructor stub
}

void MapDealer::init()
{
	mapdump = 0;
	pathdump = 0;
	for(int i = 0; i < 2; i++)
	{
		for(int t = 0; t < MAX_MAP_GRID; t++)
			Fix[i][t] = 0x00;
	}
	return;
}

int MapDealer::open(MainProgram* lp)
{
	m_MainProgram = lp;
	return 1;
}

void MapDealer::AddToRevMapDataQueue(const char * data, int data_len, int priority)
{
	MapData m_RevMapData;
	m_RevMapData.data = new char[data_len + 1];
	Hexstrncpy(m_RevMapData.data, data, data_len);
	m_RevMapData.data[data_len] = 0;
	m_RevMapData.data_len = data_len;
	m_RevMapData.priority = priority;
	pthread_mutex_lock(&RevMapDataQueuemutex);
	RevMapDataQueue.push(m_RevMapData);
	pthread_mutex_unlock(&RevMapDataQueuemutex);

	pthread_mutex_lock(&UpdateMapThreadmutex);
	pthread_cond_signal(&UpdateMapThreadEvent);
	pthread_mutex_unlock(&UpdateMapThreadmutex);
	return;
}

void MapDealer::AddToRevPlanOrderQueue(const char * data, int data_len, int priority)
{
	MapData m_RevMapData;
	m_RevMapData.data = new char[data_len + 1];
	Hexstrncpy(m_RevMapData.data, data, data_len);
	m_RevMapData.data[data_len] = 0;
	m_RevMapData.data_len = data_len;
	m_RevMapData.priority = priority;
	pthread_mutex_lock(&RevPlanOrderQueuemutex);
	RevPlanOrderQueue.push(m_RevMapData);
	pthread_mutex_unlock(&RevPlanOrderQueuemutex);

	pthread_mutex_lock(&RoadPlanThreadmutex);
	pthread_cond_signal(&RoadPlanThreadEvent);
	pthread_mutex_unlock(&RoadPlanThreadmutex);
	return;
}

bool MapDealer::CreateUpdateMapandPlanThread()
{
	int ret = pthread_create(&m_UpdateMapThreadHandle, NULL, UpdateMapFunc,	(void*) this);
	if (ret)
	{
		fprintf(stdout, "Create UpdateMapFunc thread error!/n");
			return false;
	}

	ret = pthread_create(&m_RoadPlanThreadHandle, NULL, RoadPlanFunc,	(void*) this);
	if (ret)
	{
		fprintf(stdout, "Create RoadPlanFunc thread error!/n");
			return false;
	}
	return true;
}

int MapDealer::UpdateMapProcess(const MapData* msg)
{
	int X_index = msg->data[2];
	int Y_index = msg->data[3];
	unsigned char value = msg->data[6];
	Fix[X_index][Y_index] = value;
	return 1;
}

void * MapDealer::UpdateMapFunc(void* lparam)
{
	MapDealer * pMapDealer;
	pMapDealer = (MapDealer*) lparam;

	while(true)
	{
		if(pMapDealer->RevMapDataQueue.size() <= 0)
		{
			pthread_mutex_lock(&pMapDealer->UpdateMapThreadmutex);
			pthread_cond_wait(&pMapDealer->UpdateMapThreadEvent, &pMapDealer->UpdateMapThreadmutex);
			pthread_mutex_unlock(&pMapDealer->UpdateMapThreadmutex);
		}
		else
		{
			while(!pMapDealer->RevMapDataQueue.empty())
			{
				pthread_mutex_lock(&pMapDealer->RevMapDataQueuemutex);

				pthread_mutex_lock(&pMapDealer->MapFixmutex);
				pMapDealer->UpdateMapProcess(&pMapDealer->RevMapDataQueue.top());
				pthread_mutex_unlock(&pMapDealer->MapFixmutex);

				if(pMapDealer->RevMapDataQueue.top().data != NULL)
				{
					delete (pMapDealer->RevMapDataQueue.top().data);
				}
				pMapDealer->RevMapDataQueue.pop();
				pthread_mutex_unlock(&pMapDealer->RevMapDataQueuemutex);
			}
		}
	}

	pthread_exit(NULL);
	return 0;
}

void * MapDealer::RoadPlanFunc(void* lparam)
{
	MapDealer * pMapDealer;
	pMapDealer = (MapDealer*) lparam;

	while(true)
	{
		if(pMapDealer->RevPlanOrderQueue.size() <= 0)
		{
			pthread_mutex_lock(&pMapDealer->RoadPlanThreadmutex);
			pthread_cond_wait(&pMapDealer->RoadPlanThreadEvent, &pMapDealer->RoadPlanThreadmutex);
			pthread_mutex_unlock(&pMapDealer->RoadPlanThreadmutex);
		}
		else
		{
			while(!pMapDealer->RevPlanOrderQueue.empty())
			{
				pthread_mutex_lock(&pMapDealer->RevPlanOrderQueuemutex);
				pMapDealer->RoadPlanProcess(&pMapDealer->RevPlanOrderQueue.top());
				if(pMapDealer->RevPlanOrderQueue.top().data != NULL)
				{
					delete (pMapDealer->RevPlanOrderQueue.top().data);
				}
				pMapDealer->RevPlanOrderQueue.pop();
				pthread_mutex_unlock(&pMapDealer->RevPlanOrderQueuemutex);
			}
		}
	}
	return 0;
}

int MapDealer::RoadPlanProcess(const MapData* msg)
{
	int X_StartIndex,Y_StartIndex;
	int X_EndIndex,Y_EndIndex;
	int map[MAX_MAP_GRID][MAX_MAP_GRID];

	X_StartIndex = msg->data[2];
	Y_StartIndex = msg->data[3];
	X_EndIndex = msg->data[4];
	Y_EndIndex = msg->data[5];

	memcpy(tmpFix, Fix, MAX_MAP_GRID*MAX_MAP_GRID);
	tmpFix[X_StartIndex][Y_StartIndex] |= 0x01;
/*
	ifstream getmapstream;
	int si,sj;
	int ei,ej;
	getmapstream.open("//home/jacky/map", ios::in);

	if(getmapstream.is_open())
	{
		char linecontent[330];
		getmapstream.getline(linecontent,330);
		int ytag = 0;
		int xtag = 0;
		while(!getmapstream.fail())
		{
			xtag = 0;
			for(int i = 0; i < 330; i+=2,xtag++)
			{
				if('1' == linecontent[i])
				{
					map[ytag][xtag] = UNAVAIL;
				}
				else if('0' == linecontent[i])
				{
					map[ytag][xtag] = AVAIL;
				}
				else if('5' == linecontent[i])
				{
					map[ytag][xtag] = START;
					si = ytag;
					sj = i/2;
				}
				else if('6' == linecontent[i])
				{
					map[ytag][xtag] = END;
					ei = ytag;
					ej = i/2;
				}
				else
				{
					map[ytag][xtag] = UNAVAIL;
				}
			}
			ytag++;
			if(160 == ytag)
				break;
			getmapstream.getline(linecontent,330);
		}
	}
	getmapstream.close();

	ofstream outplanmapstr;
	mapdump++;
	char dumppath[20];
	sprintf(dumppath, "//home/%d.map", mapdump);
	outplanmapstr.open(dumppath, ios::out);

	for(int i  = 0; i < MAX_MAP_GRID; i++)
	{
		for(int j = 0; j < MAX_MAP_GRID; j++)
		{
			if((X_StartIndex == i)&&(Y_StartIndex == j))
				outplanmapstr<<" "<<5;
			else if((X_EndIndex == i)&&(Y_EndIndex == j))
				outplanmapstr<<" "<<6;
			else
				outplanmapstr<<" "<<map[i][j];
		}
		outplanmapstr<<endl;
	}
	outplanmapstr.close();
*/

#ifdef DEBUG
	ofstream outplanmapstr;
	mapdump++;
	char dumppath[20];
	sprintf(dumppath, "//home/%d.map", mapdump);
	outplanmapstr.open(dumppath, ios::out);
	outplanmapstr<<X_StartIndex<<","<<Y_StartIndex<<"   "<<X_EndIndex<<","<<Y_EndIndex<<endl;
	//Road Plan under 3 is blocked!
	for(int i  = 0; i < MAX_MAP_GRID; i++)
	{
		for(int j = 0; j < MAX_MAP_GRID; j++)
		{
			if(0x00 == tmpFix[i][j])
			{
				map[i][j] = UNAVAIL;
			}
			else if(0x01 == tmpFix[i][j])
			{
				map[i][j] = AVAIL;
			}
			else if(0x02 == tmpFix[i][j])
			{
				map[i][j] = UNAVAIL;
			}
			else if(0x03 == tmpFix[i][j])
			{
				map[i][j] = UNAVAIL;
			}
			else
			{
				map[i][j] = AVAIL;
			}
			if((X_StartIndex == i)&&(Y_StartIndex == j))
				outplanmapstr<<" "<<5;
			else if((X_EndIndex == i)&&(Y_EndIndex == j))
				outplanmapstr<<" "<<6;
			else
				outplanmapstr<<" "<<map[i][j];
		}
		outplanmapstr<<endl;
	}
	outplanmapstr.close();
#endif

	Node* Path_list;
	fprintf(stdout, "start: %d,%d  end: %d,%d.\n", X_StartIndex, Y_StartIndex, X_EndIndex, Y_EndIndex);
	Path_list = Road_Plan(map,X_StartIndex, Y_StartIndex, X_EndIndex, Y_EndIndex);
	if(!Path_list)
	{
		//Road Plan under 3 is passed!
		destroy_openlist();
		destroy_closelist();
		Path_list = Road_Plan(map,X_StartIndex, Y_StartIndex, X_EndIndex, Y_EndIndex);
	}

	if(Path_list)
	{
		unsigned char PathBuffer[2048];
		memset(PathBuffer, 0, 2048);
		//serial path to buffer
		Node* tmp_ptr = Path_list;
		int tag = 0;
		while(tmp_ptr)
		{
			PathBuffer[tag] = tmp_ptr->j;
			tag++;
			PathBuffer[tag] = tmp_ptr->i;
			tag++;
			tmpFix[tmp_ptr->i][tmp_ptr->j] = 0xFF;
			tmp_ptr = tmp_ptr->parent;
		}

		destroy_openlist();
		destroy_closelist();
//		ofstream outplanmapstr;
//		mapdump++;
//		char dumppath[20];
//		sprintf(dumppath, "//home/%d.map", mapdump);
//		outplanmapstr.open(dumppath, ios::out);
//		outplanmapstr<<X_StartIndex<<","<<Y_StartIndex<<"   "<<X_EndIndex<<","<<Y_EndIndex<<endl;
//		for(int i  = 0; i < MAX_MAP_GRID; i++)
//		{
//			for(int j = 0; j < MAX_MAP_GRID; j++)
//			{
//				if(0x00 == tmpFix[i][j])
//				{
//					map[i][j] = UNAVAIL;
//				}
//				else if(0x01 == tmpFix[i][j])
//				{
//					map[i][j] = AVAIL;
//				}
//				else if(0x02 == tmpFix[i][j])
//				{
//					map[i][j] = UNAVAIL;
//				}
//				else if(0x03 == tmpFix[i][j])
//				{
//					map[i][j] = UNAVAIL;
//				}
//				else
//				{
//					map[i][j] = AVAIL;
//				}
//	//			if((X_StartIndex == i)&&(Y_StartIndex == j))
//	//				outplanmapstr<<" "<<5;
//	//			else if((X_EndIndex == i)&&(Y_EndIndex == j))
//	//				outplanmapstr<<" "<<6;
//	//			else
//					outplanmapstr<<" "<<map[i][j];
//			}
//			outplanmapstr<<endl;
//		}
//		outplanmapstr.close();

#ifdef DEBUG
		ofstream outplanpathstr;
		pathdump++;
		char dumppath[20];
		sprintf(dumppath, "//home/%d-%d.path", pathdump, mapdump);
		outplanpathstr.open(dumppath, ios::out);
		outplanpathstr<<X_StartIndex<<","<<Y_StartIndex<<"   "<<X_EndIndex<<","<<Y_EndIndex<<endl;
		//Road Plan under 3 is blocked!
		for(int i  = 0; i < MAX_MAP_GRID; i++)
		{
			for(int j = 0; j < MAX_MAP_GRID; j++)
			{
				if(0x00 == tmpFix[i][j])
				{
					map[i][j] = UNAVAIL;
				}
				else if(0x01 == tmpFix[i][j])
				{
					map[i][j] = AVAIL;
				}
				else if(0x02 == tmpFix[i][j])
				{
					map[i][j] = UNAVAIL;
				}
				else if(0x03 == tmpFix[i][j])
				{
					map[i][j] = UNAVAIL;
				}
				else if(0xFF == tmpFix[i][j])
				{
					map[i][j] = 8;
				}
				else
				{
					map[i][j] = AVAIL;
				}
				if((X_StartIndex == i)&&(Y_StartIndex == j))
					outplanpathstr<<" "<<5;
				else if((X_EndIndex == i)&&(Y_EndIndex == j))
					outplanpathstr<<" "<<6;
				else
					outplanpathstr<<" "<<map[i][j];
			}
			outplanpathstr<<endl;
		}
		outplanpathstr.close();
#endif

		int AllSendNode = (tag>143)? 144:tag;
		unsigned char * RealSend = new unsigned char[AllSendNode];
		memcpy(RealSend, &PathBuffer[tag-AllSendNode], AllSendNode);
		//add path to Commhelper send queue
		int circles = 0;
		for(int t = AllSendNode; t > 0; t-=12)
		{
			if(t > 12)
			{
				unsigned char* RoadPlanSendBuffer = new unsigned char[16];
				memset(RoadPlanSendBuffer, 0, 16);
				RoadPlanSendBuffer[0] = 0xFA;
				RoadPlanSendBuffer[1] = 0xFA;
				RoadPlanSendBuffer[2] = circles;
				for(int i = 0; i < 6; i++)
				{
					RoadPlanSendBuffer[3+i] = RealSend[AllSendNode-(circles*12)-(i*2)-1];
					RoadPlanSendBuffer[9+i] = RealSend[AllSendNode-(circles*12)-(i*2)-2];
				}
				unsigned char CRC = RoadPlanSendBuffer[0];
				for(int i = 1; i < 15; i++)
				{
					CRC = CRC ^ RoadPlanSendBuffer[i];
				}
				RoadPlanSendBuffer[15] = CRC;

				m_MainProgram->m_CommHelper.SendData((char*)RoadPlanSendBuffer, 16);
			}
			else
			{
				//the last send buffer
				unsigned char* RoadPlanSendBuffer = new unsigned char[16];
				memset(RoadPlanSendBuffer, 0xFF, 16);
				RoadPlanSendBuffer[0] = 0xFA;
				RoadPlanSendBuffer[1] = 0xFA;
				RoadPlanSendBuffer[2] = circles;
				RoadPlanSendBuffer[2] |= 0xE0;

				for(int i = 0; i < t/2; i++)
				{
					RoadPlanSendBuffer[3+i] = RealSend[AllSendNode-(circles*12)-(i*2)-1];
					RoadPlanSendBuffer[9+i] = RealSend[AllSendNode-(circles*12)-(i*2)-2];
				}
				unsigned char CRC = RoadPlanSendBuffer[0];
				for(int i = 1; i < 15; i++)
				{
					CRC = CRC ^ RoadPlanSendBuffer[i];
				}
				RoadPlanSendBuffer[15] = CRC;
				m_MainProgram->m_CommHelper.SendData((char*)RoadPlanSendBuffer, 16);
			}
			circles++;
			usleep(30000);
		}
		//add path to TCPClient send queue
		int TCPcircles = 0;
		for(int t = AllSendNode; t > 0; t-=8)
		{
			if(t > 8)
			{
				unsigned char* RoadPlanSendBuffer = new unsigned char[10];
				memset(RoadPlanSendBuffer, 0, 10);
				RoadPlanSendBuffer[0] = 0xF8;
				RoadPlanSendBuffer[1] = 0xF8;
				for(int i = 0; i < 4; i++)
				{
					RoadPlanSendBuffer[2+(i*2)] = RealSend[AllSendNode-(TCPcircles*8)-(i*2)-1];
					RoadPlanSendBuffer[2+(i*2)+1] = RealSend[AllSendNode-(TCPcircles*8)-(i*2)-2];
				}
				m_MainProgram->m_tcpClient.SendData((char*)RoadPlanSendBuffer, 10);
			}
			else
			{
				//the last send buffer
				unsigned char* RoadPlanSendBuffer = new unsigned char[10];
				memset(RoadPlanSendBuffer, 0xFF, 10);
				RoadPlanSendBuffer[0] = 0xF8;
				RoadPlanSendBuffer[1] = 0xF8;

				for(int i = 0; i < t/2; i++)
				{
					RoadPlanSendBuffer[2+(i*2)] = RealSend[AllSendNode-(TCPcircles*8)-(i*2)-1];
					RoadPlanSendBuffer[2+(i*2)+1] = RealSend[AllSendNode-(TCPcircles*8)-(i*2)-2];
				}
				m_MainProgram->m_tcpClient.SendData((char*)RoadPlanSendBuffer, 10);
			}
			TCPcircles++;
		}

		free(RealSend);
	}
	else
	{
		//add 0xff to Commhelper send queue, there is no path to go
		destroy_openlist();
		destroy_closelist();
		unsigned char* RoadPlanSendBuffer = new unsigned char[16];
		memset(RoadPlanSendBuffer, 0xFF, 16);
		RoadPlanSendBuffer[0] = 0xFA;
		RoadPlanSendBuffer[1] = 0xFA;

		unsigned char CRC = RoadPlanSendBuffer[0];
		for(int i = 1; i < 15; i++)
		{
			CRC = CRC ^ RoadPlanSendBuffer[i];
		}
		RoadPlanSendBuffer[15] = CRC;
		m_MainProgram->m_CommHelper.SendData((char*)RoadPlanSendBuffer, 16);
	}
	return 1;
}

void MapDealer::ClearMap()
{
	memset(Fix, 0, MAX_MAP_GRID*MAX_MAP_GRID);
}































