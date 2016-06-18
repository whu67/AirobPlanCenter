/*
 * TCPClient.cpp
 *
 *  Created on: 2014-7-17
 *      Author: root
 */
#include "main.h"
#include "TCPClient.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//构造函数
TCPClient::TCPClient()
{
	ConnectionStatus = false;
	RevThreadFuncStatus = false;
	ExitFlag = false;
	m_socket = -1;

	pthread_mutex_init(&SendDataQueuemutex, NULL);
	pthread_mutex_init(&RevDataQueuemutex, NULL);

	pthread_cond_init(&MsgProcessThreadEvent, NULL);
	pthread_mutex_init(&MsgProcessThreadmutex, NULL);

	pthread_cond_init(&SendQueueThreadEvent, NULL);
	pthread_mutex_init(&SendQueueThreadmutex, NULL);
}

//析构函数
TCPClient::~TCPClient()
{
	//释放socket资源
}

/*--------------------------------------------------------------------
 【函数介绍】:  此线程用于监听TCP客户端通讯的事件，例如当接收到数据、
 连接断开和通讯过程发生错误等事件
 【入口参数】:  lparam:无类型指针，可以通过此参数，向线程中传入需要用到的资源。
 在这里我们将TCPClient类实例指针传进来
 【出口参数】:  (无)
 【返回  值】:  返回值没有特别的意义，在此我们将返回值设为0。
 ---------------------------------------------------------------------*/
void *TCPClient::SocketThreadFunc(void * lparam)
{
	TCPClient *pSocket;
	//得到TCPClient实例指针
	pSocket = (TCPClient*) lparam;
	//定义读事件集合
	fd_set fdRead;
	int ret;
	//定义事件等待时间
	struct timeval aTime;

	while (true)
	{
		if (pSocket->ExitFlag)
		{
			fprintf(stdout, "收到退出接收线程信号.\n");
			break;
		}

		if (!pSocket->ConnectionStatus)
		{
			pSocket->ReConnect();
		}
		//收到退出事件，结束线程
		//if (WaitForSingleObject(pSocket->m_exitThreadEvent,0) == WAIT_OBJECT_0)
		//{
		//	break;
		//}
		aTime.tv_sec = 1;
		aTime.tv_usec = 0;
		//置空fdRead事件为空
		FD_ZERO(&fdRead);
		//给客户端socket设置读事件
		FD_SET(pSocket->m_socket,&fdRead);
		//调用select函数，判断是否有读事件发生
		ret = select(pSocket->m_socket + 1, &fdRead, NULL, NULL, &aTime);

		if (ret == -1)
		{
			pSocket->ReConnect();
			continue;
			//break;
		}

		if (ret > 0)
		{
			if (FD_ISSET(pSocket->m_socket,&fdRead))
			{
				//发生读事件
				char recvBuf[1024];
				int recvLen;
				//ZeroMemory(recvBuf,1024);
				memset(recvBuf, 0, 1024);
				//接收数据
				recvLen = recv(pSocket->m_socket, recvBuf, 1024, 0);
				if (recvLen == -1)
				{
					pSocket->ReConnect();
					continue;
				}
				else if (recvLen == 0)
				{
					printf("服务器端断开.\n");
					pSocket->ReConnect();
					continue;
				}
				else
				{
					//触发数据接收事件
					pSocket->ExtractData(recvBuf, recvLen);

				}
			}
		}
	}
	pSocket->RevThreadFuncStatus = false;
	return 0;
}

int TCPClient::MsgProcess(const ServerData* msg)
{

	fprintf(stdout, "TCP Rev  (Hex):[%02X", msg->data[0] & 0xff);
	for (int i = 1; i < msg->data_len; i++)
	{
		fprintf(stdout, " %02X", msg->data[i] & 0xff);
	}
	fprintf(stdout, "]\n");
	fprintf(stdout, "TCP Rev  (Str):[%s]\n", msg->data);

	m_MainProgram->m_CommHelper.SendData(msg->data, msg->data_len);

	return 1;

}

int TCPClient::ExtractData(const char *data, int data_len)
{

	//print_hex(data, data_len, "TCPClient收到数据");
	int ret = 0;

	AddToRevQueue(data, data_len);

	ret = data_len;
	return ret;
}

void *TCPClient::MsgProcessThreadFunc(void * lparam)
{
	TCPClient *pSocket;
	//得到TCPServer实例指针
	pSocket = (TCPClient*) lparam;

	while (true)
	{

		if (pSocket->RevDataQueue.size() <= 0)
		{
			pthread_mutex_lock(&pSocket->MsgProcessThreadmutex);
			pthread_cond_wait(&pSocket->MsgProcessThreadEvent,
					&pSocket->MsgProcessThreadmutex);
			pthread_mutex_unlock(&pSocket->MsgProcessThreadmutex);

		}
		else
		{
			while (!pSocket->RevDataQueue.empty())
			{
				pthread_mutex_lock(&pSocket->RevDataQueuemutex);

				pSocket->MsgProcess(&pSocket->RevDataQueue.top());

				if (pSocket->RevDataQueue.top().data != NULL)
				{
					delete (pSocket->RevDataQueue.top().data);
				}

				pSocket->RevDataQueue.pop();

				pthread_mutex_unlock(&pSocket->RevDataQueuemutex);
			}
		}

	}
	pthread_exit(NULL);
	return 0;
}

void *TCPClient::SendQueueThreadFunc(void * lparam)
{
	TCPClient *pSocket;
	//得到TCPServer实例指针
	pSocket = (TCPClient*) lparam;

	while (true)
	{

		if (pSocket->SendDataQueue.size() <= 0)
		{
			pthread_mutex_lock(&pSocket->SendQueueThreadmutex);
			pthread_cond_wait(&pSocket->SendQueueThreadEvent,
					&pSocket->SendQueueThreadmutex);
			pthread_mutex_unlock(&pSocket->SendQueueThreadmutex);

		}
		else
		{
			while (!pSocket->SendDataQueue.empty())
			{

				pthread_mutex_lock(&pSocket->SendDataQueuemutex);

				if (pSocket->getConnectionStatus())
				{
					pSocket->SendData(pSocket->SendDataQueue.top().data,
							pSocket->SendDataQueue.top().data_len);
				}
				if (pSocket->SendDataQueue.top().data != NULL)
				{
					delete (pSocket->SendDataQueue.top().data);
				//	delete[] pSocket->SendDataQueue.top().data;
				}
				pSocket->SendDataQueue.pop();
				pthread_mutex_unlock(&pSocket->SendDataQueuemutex);
			}
		}

	}

	pthread_exit(NULL);
	return 0;
}

/*--------------------------------------------------------------------
 【函数介绍】: 用于打开客户端socket
 【入口参数】: (无)
 【出口参数】: (无)
 【返回  值】: TRUE:打开成功;FALSE:打开失败
 ---------------------------------------------------------------------*/
bool TCPClient::CreateRevThread()
{

	if (RevThreadFuncStatus)
		return true;

	int ret = pthread_create(&m_tcpThreadHandle, NULL, SocketThreadFunc,
			(void*) this);
	if (ret)
	{
		close(m_socket);
		fprintf(stdout, "Create ServerSocketThreadFunc thread error!\n");
		RevThreadFuncStatus = false;
		return false;
	}

	ret = pthread_create(&m_MsgProcessThreadHandle, NULL, MsgProcessThreadFunc,
			(void*) this);

	if (ret)
	{
		fprintf(stdout, "Create MsgProcessThreadFunc thread error!/n");
		return -1;
	}

	ret = pthread_create(&m_SendQueueThreadHandle, NULL, SendQueueThreadFunc,
			(void*) this);

	if (ret)
	{
		fprintf(stdout, "Create SendQueueThreadFunc thread error!/n");
		return -1;
	}

	RevThreadFuncStatus = true;
	return true;
}

/*--------------------------------------------------------------------
 【函数介绍】:  打开TCP服务
 【入口参数】:  (无)
 【出口参数】:  (无)
 【返回  值】:  <=0:打开TCP服务失败; =1:打开TCP服务成功
 ---------------------------------------------------------------------*/
int TCPClient::Open(MainProgram * lp)
{
	m_MainProgram = lp;
	return 1;
}

/*--------------------------------------------------------------------
 【函数介绍】: 用于关闭客户端socket
 【入口参数】:  (无)
 【出口参数】:  (无)
 【返回  值】: TRUE:关闭成功;FALSE:关闭失败
 ---------------------------------------------------------------------*/
bool TCPClient::Close()
{

	if (m_socket < 0)
		return true;
	int err = close(m_socket);
	if (err == -1)
	{
		return false;
	}
	m_socket = -1;
	return true;
}

/*--------------------------------------------------------------------
 【函数介绍】: 用于建立与TCP服务器连接
 【入口参数】: (无)
 【出口参数】: (无)
 【返回  值】: TRUE:建立连接成功;FALSE:建立连接失败
 ---------------------------------------------------------------------*/
bool TCPClient::Connect()
{
	if (ConnectionStatus)
		return true;

	struct sockaddr_in addr;

	addr.sin_family = AF_INET;
	addr.sin_port = htons(m_port);

	addr.sin_addr.s_addr = inet_addr(m_remoteHost);

	if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		m_socket = -1;
		return false;
	}

	// 设置接收缓冲区
	int nRecvBuf = 32 * 1024;//设置为32K
	setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, (const char*) &nRecvBuf,
			sizeof(int));
	// 设置发送缓冲区
	int nSendBuf = 32 * 1024;//设置为32K
	setsockopt(m_socket, SOL_SOCKET, SO_SNDBUF, (const char*) &nSendBuf,
			sizeof(int));

	if (-1 == connect(m_socket, (struct sockaddr *) (&addr), sizeof(addr)))
	{
		m_socket = -1;
		printf("Connect Fail!\n");
		return false;
	}

	int flags;
	if ((flags = fcntl(m_socket, F_GETFL, 0)) < 0)
		fprintf(stdout, "获取通讯模式出错\n");
	//出错处理
	flags |= O_NONBLOCK;
	if ((fcntl(m_socket, F_SETFL, flags)) < 0)
		fprintf(stdout, "设置为非阻塞模式出错\n");

	ConnectionStatus = true;

	if (!CreateRevThread())
	{
		exit(0);
		return false;
	}

	//
	usleep(200 * 1000);

	return true;
}

bool TCPClient::ReConnect()
{
	ConnectionStatus = false;
	sleep(1);
	while (true)
	{
		int CloseTimes = 3;
		int ReConnectTimes = 3;
		while (CloseTimes--)
		{
			//fprintf(stdout, "[%d]CloseTimes...\n", 3 - CloseTimes);
			if (Close())
				break;
		}

		while (ReConnectTimes--)
		{
			fprintf(stdout, "[%d]重新连接中...\n", 3 - ReConnectTimes);
			if (Connect())
			{
				ConnectionStatus = true;
				fprintf(stdout, "重新连接成功.\n");

				return true;
			}
			sleep(1);
		}
		fprintf(stdout, "重新连接失败,5秒后重新尝试.\n");
		sleep(5);

	}
	return false;
}

bool TCPClient::getConnectionStatus()
{
	return ConnectionStatus;
}

void TCPClient::AddToSendQueue(const char * data, int data_len,int priority)
{
	ServerData m_SendData;
	m_SendData.data = new char[data_len + 1];
	Hexstrncpy(m_SendData.data, data, data_len);
	m_SendData.data[data_len] = 0;
	m_SendData.data_len = data_len;
	m_SendData.priority = priority;
	pthread_mutex_lock(&SendDataQueuemutex);
	SendDataQueue.push(m_SendData);
	pthread_mutex_unlock(&SendDataQueuemutex);

	pthread_mutex_lock(&SendQueueThreadmutex);
	pthread_cond_signal(&SendQueueThreadEvent);
	pthread_mutex_unlock(&SendQueueThreadmutex);
}

void TCPClient::AddToRevQueue(const char * data, int data_len, int priority)
{

	ServerData m_RevData;
	m_RevData.data = new char[data_len + 1];
	Hexstrncpy(m_RevData.data, data, data_len);
	m_RevData.data[data_len] = 0;
	m_RevData.data_len = data_len;
	pthread_mutex_lock(&RevDataQueuemutex);
	RevDataQueue.push(m_RevData);
	pthread_mutex_unlock(&RevDataQueuemutex);

	pthread_mutex_lock(&MsgProcessThreadmutex);
	pthread_cond_signal(&MsgProcessThreadEvent);
	pthread_mutex_unlock(&MsgProcessThreadmutex);
}

void TCPClient::setExitFlag(bool flag)
{
	ExitFlag = flag;
}

bool TCPClient::ConnectServer()
{

	//建立与服务器端连接
	if (Connect()) {
		//fprintf(stdout, "m_tcpClient建立连接成功.\n");
		return true;
	} else {
		//fprintf(stdout, "m_tcpClient建立连接失败.\n");
		if (!CreateRevThread())
			return false;
	}
	return false;
}

/*--------------------------------------------------------------------
 【函数介绍】: 向服务器端发送数据
 【入口参数】: buf:待发送的数据
 len:待发送的数据长度
 【出口参数】: (无)
 【返回  值】: TRUE:发送数据成功;FALSE:发送数据失败
 ---------------------------------------------------------------------*/
bool TCPClient::SendData(const char * buf, int len)
{
	if (!ConnectionStatus)
	{
		fprintf(stdout, "Tcp Send Fail!\n");
		return false;
	}

	int nBytes = 0;
	int nSendBytes = 0;

	while (nSendBytes < len)
	{
		nBytes = send(m_socket, buf + nSendBytes, len - nSendBytes,
				MSG_NOSIGNAL);
		if (nBytes == -1)
		{

			fprintf(stdout, "Tcp Send Fail!\n");
			return false;
		}

		nSendBytes = nSendBytes + nBytes;

		if (nSendBytes < len)
		{
			usleep(10 * 1000);
			continue;
		}
	}
	fprintf(stdout, "TCP Send(Hex):[%02X", buf[0] & 0xff);
	for (int i = 1; i < len; i++)
	{
		fprintf(stdout, " %02X", buf[i] & 0xff);
	}
	fprintf(stdout, "]\n");
	fprintf(stdout, "Tcp Send(Str):[%s]\n", buf);
	return true;
}
