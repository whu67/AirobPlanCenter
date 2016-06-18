/*
 * CommHelper.cpp
 *
 *  Created on: 2014-4-11
 *      Author: root
 */
#include "main.h"
#include "CommHelper.h"

bool Serials_RecieveData_Valid[SERIALS_RECIEVEDATA_BUFFERSLOT];

CommHelper::CommHelper()
{
//	strcpy(dev, "/dev/");
//	Speed = 115200;
	memset(PhotoReply, 0xFD, 16);
	for(int i = 3; i < 15; i++)
	{
		PhotoReply[i] = 0xFF;
	}
	unsigned char ReplyCRC = PhotoReply[0];
	for(int i = 1; i < 15; i++)
	{
		ReplyCRC ^= PhotoReply[i];
	}
	PhotoReply[15] = ReplyCRC;
 }

CommHelper::~CommHelper()
{
	tcsetattr(fd, TCSANOW, &oldtio);
	if (fd > 0)
		close(fd);
}

int CommHelper::Close()
{
	return 1;
}

int CommHelper::SendData(char * str, int len)
{
	int result, TryTimes = 3;
	while (TryTimes--)
	{
		result = write(fd, str, len);
		if (result < 0)
		{
			usleep(10 * 1000);
		}
		else
		{
			//printf("com send :(Str)[%s]\n", str);
			fprintf(stdout, "COM send (Hex):[%02X", (unsigned char) str[0]);
			for (int i = 1; i < len; i++)
			{
				fprintf(stdout, " %02X", (unsigned char) str[i]);
			}
			fprintf(stdout, "]\n");

			return 1;
		}
	}
	delete (str);
	print_hex(str , len , "Write to serial port failed");
	return -1;
}

int CommHelper::init_dev()
{
	struct termios newtio;

	fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		printf("open serial port : %s failed.\n", dev);
		return -1;
	} else
		//set to block;
		fcntl(fd, F_SETFL, 0);

	printf("open serial port : %s success.\n", dev);

	tcgetattr(fd, &oldtio); //save current serial port settings

	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

	int speed_arr[] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200,
			B300, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
	int name_arr[] = { 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,
			38400, 19200, 9600, 4800, 2400, 1200, 300, };
	int i;
	for (i = 0; i < (int) sizeof(speed_arr) / (int) sizeof(int); i++)
	{
		if (Speed == name_arr[i])
		{
			cfsetispeed(&newtio, speed_arr[i]);
			cfsetospeed(&newtio, speed_arr[i]);
			i = -1;
			break;
		}
	}
	if (i != -1)
	{
		printf("configure the serial port speed : %d failed.\n", Speed);
		return -1;
	}
	newtio.c_cflag |= CLOCAL | CREAD;
	/*8N1*/
	newtio.c_cflag &= ~CSIZE; /* Mask the character size bits */
	newtio.c_cflag |= CS8; /* Select 8 data bits */
	newtio.c_cflag &= ~PARENB;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cflag &= ~CRTSCTS;//disable hardware flow control;
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*raw input*/
	newtio.c_oflag &= ~OPOST; /*raw output*/
	tcflush(fd, TCIFLUSH);//clear input buffer
	newtio.c_cc[VTIME] = 100; /* inter-character timer unused */
	newtio.c_cc[VMIN] = 0; /* blocking read until 0 character arrives */
	tcsetattr(fd, TCSANOW, &newtio);

	for(i = 0; i < SERIALS_RECIEVEDATA_BUFFERSLOT; i++);
	{
		if(i < SERIALS_RECIEVEDATA_CMDCOUNTERS)
			Serials_RecieveData_Valid[i] = true;
		else
			Serials_RecieveData_Valid[i] = false;
	}
	return 1;
}

int CommHelper::StripCMD(const char * data, int data_len)
{
	unsigned char cmdbuff[2];

	if(10 != data_len)
		return -1;

	cmdbuff[0] = data[0];
	cmdbuff[1] = data[1];

	if((0xf1 == cmdbuff[0])&&(0xf1 == cmdbuff[1]))
	{
		return 1;
	}
	else if((0xF2 == cmdbuff[0])&&(0xF2 == cmdbuff[1]))
	{
		return 2;
	}
	else if((0xf3 == cmdbuff[0])&&(0xf3 == cmdbuff[1]))
	{
		return 3;
	}
	else if((0xf4 == cmdbuff[0])&&(0xf4 == cmdbuff[1]))
	{
		return 4;
	}
	else if((0xf5 == cmdbuff[0])&&(0xf5 == cmdbuff[1]))
	{
		return 5;
	}
	else if((0xf6 == cmdbuff[0])&&(0xf6 == cmdbuff[1]))
	{
		return 6;
	}
	else if((0xf7 == cmdbuff[0])&&(0xf7 == cmdbuff[1]))
	{
		return 7;
	}
	return -1;
}

//消息
int CommHelper::MsgProcess(const char * data, int data_len)
{
//	print_hex(data, data_len, "串口有效数据");
	int circles;
	circles = data_len/10;

	for(int i = 0; i < circles; i++)
	{
		char * realdata = new char[10];
		unsigned char * Replybuffer = new unsigned char[16];
		memcpy(realdata, (data+i*10), 10);
		int cmd = StripCMD(realdata, 10);
		//cmd：1-地图数据 2-规划数据 3-调试数据1 4-调试数据2 5-拍照指令 6-上位机状态查询指令


		//debug communication data
//		memset(Replybuffer, 0xFD, 16);
//		Replybuffer[0] = 0xFA;
//		Replybuffer[1] = 0xFA;
//		m_MainProgram->m_CommHelper.SendData((char*)Replybuffer, 16);

		//begin to analyze data and assign to specific thread to deal with
		switch (cmd)
		{
			case 1:
				//m_MainProgram->m_MapDealer.AddToRevMapDataQueue(realdata, 10);
				m_MainProgram->m_MapDealer.UpdateMapDirectly(realdata);
				m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
				break;
			case 2:
				memset(Replybuffer, 0xFD, 16);
				Replybuffer[0] = 0xFA;
				Replybuffer[1] = 0xFA;
				m_MainProgram->m_CommHelper.SendData((char*)Replybuffer, 16);
//				usleep(30000);
				m_MainProgram->m_MapDealer.AddToRevPlanOrderQueue(realdata, 10);
				m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
				break;
			case 3:
				m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
				break;
			case 4:
				m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
				break;
			case 5:
				if(0x11 == realdata[7])
				{
					//Get a Photo
					if(!m_MainProgram->m_ParameterAdjuster.GetPhoto(realdata[4], realdata[5]))
					{
						//failed to get photo
						memset(Replybuffer, 0x00, 16);
						Replybuffer[0] = 0xF5;
						Replybuffer[1] = 0xF5;
					}
					else
					{
						memcpy(Replybuffer, PhotoReply, 16);
					}
					m_MainProgram->m_CommHelper.SendData((char*)Replybuffer, 16);
				}
				else
				{
					//Parameter Adjust
					if(!m_MainProgram->m_ParameterAdjuster.GetTmpPhoto(realdata[4], realdata[5]))
					{
						//failed to get photo
						memset(Replybuffer, 0x00, 16);
						Replybuffer[0] = 0xF5;
						Replybuffer[1] = 0xF5;
					}
					else
					{
						memcpy(Replybuffer, PhotoReply, 16);
					}
					m_MainProgram->m_CommHelper.SendData((char*)Replybuffer, 16);
					m_MainProgram->m_ParameterAdjuster.AddToAdjustQueue(realdata, 10);
				}
				break;
			case 6:
				break;
			case 7:
				m_MainProgram->m_MapDealer.ClearMap();
				m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
				break;
			default:
				break;
		}
		//end to analyze data and assign to specifice thread to deal with

		//m_MainProgram->m_tcpClient.AddToSendQueue(realdata, 10);
	}

	return 1;
}

//返回的值为已经使用了多少字节
int CommHelper::ExtractData(const char * data, int data_len)
{
	int ret = 0;

	MsgProcess(data , data_len) ;

	ret = data_len ;
	return ret;
}

void *CommHelper::CommReadThreadFunc(void * lparam)
{

	CommHelper *pCommHelper;
	//得到CommHelper实例指针
	pCommHelper = (CommHelper*) lparam;

//	printf("CommReadThreadFunc start in device fd %d.\n", pCommHelper->fd);
//	int n, max_fd, len, count = 0;
//	fd_set input;
//	max_fd = pCommHelper->fd + 1;
//	/* Do the select */
//	struct timeval timeout;
//	char buffer[1024];
//	char * ptr = buffer;
//	int ret = 0;
//
//	while (true)
//	{
//		timeout.tv_sec = 0;
//		timeout.tv_usec = 30 * 1000; //20ms
//		FD_ZERO(&input);
//		FD_SET(pCommHelper->fd, &input);
//		n = select(max_fd, &input, NULL, NULL, &timeout);
//		if (n < 0)
//			perror("select failed");
//
//		else if (n == 0)
//		{
//			if ( (count - ret )> 0)
//			{   ///
//				buffer[count] = 0;
//
//				printf("com rev :(Str)[%s]\n", buffer);
//				fprintf(stdout, "com rev :(Hex)[%02X",
//						(unsigned char) buffer[0]);
//				for (int i = 1; i < count; i++)
//				{
//					fprintf(stdout, " %02X", (unsigned char) buffer[i]);
//				}
//				fprintf(stdout, "]\n");
//
//				ret = pCommHelper->ExtractData(buffer, count);
//				count = count - ret;
//				if (count < 512)
//				{
//					Hexstrncpy(buffer, buffer + ret, count);
//					ptr = buffer + count;
//				}
//				else
//				{
//					count = 0;
//					ptr = buffer;
//				}
//				ret = count ; //用来标记下次是否有数据增加
//			}
//		}
//		else
//		{
//			ioctl(pCommHelper->fd, FIONREAD, &len);
//			if (!len)
//			{
//				fprintf(stderr, "Communication closed by server\n");
//				break;
//			}
//
//			len = read(pCommHelper->fd, ptr, len);
//			ptr += len;
//			count += len;
//		}
//	}

	int inputfd = pCommHelper->fd;
	unsigned char TemporaryBuffer[ 2 ];
	char RecieveBuffer[SERIALS_RECIEVEDATA_BUFFERLENGTH];

	int readcounter;
	while(true)
	{
		while((readcounter = read(inputfd, TemporaryBuffer, 1)) > 0)
		{
			if(((TemporaryBuffer[0]^0xF0) <= SERIALS_RECIEVEDATA_CMDCOUNTERS)&&((TemporaryBuffer[0]^0xF0) != 0))
			{
				if((readcounter = read(inputfd, &TemporaryBuffer[1], 1)) > 0)
				{
					if(TemporaryBuffer[0] == TemporaryBuffer[1])
					{
						bool GoOnGet = true;
						int Recieveindex = 2;
						RecieveBuffer[0] = TemporaryBuffer[0];
						RecieveBuffer[1] = TemporaryBuffer[1];
						while(GoOnGet)
						{
							if(Recieveindex == 10)
							{
								pCommHelper->ExtractData(RecieveBuffer, 10);
								usleep(1 * 1000);
								break;
							}
							else
							{
								if((readcounter = read(inputfd, &RecieveBuffer[Recieveindex], 1)) > 0)
								{
									Recieveindex++;
								//	usleep(1000);
								}
							}
						}
					}
				}
			}
		}
//		usleep(10*1000);
	}
	printf("CommReadThreadFunc exit.\n");
	return 0;
}

int CommHelper::Open(MainProgram *lp, char * device, int speed = 115200)
{
	m_MainProgram = lp;
	strcat(dev, device);
	this->Speed = speed;
	if (init_dev() < 0)
		return -1;

	//创建一个线程进行处理
	pthread_attr_t child_thread_attr;
	if (0 != pthread_attr_init(&child_thread_attr))
		printf("CommRead pthread_attr_init failed\n");

	if (0 != pthread_attr_setdetachstate(&child_thread_attr,
			PTHREAD_CREATE_DETACHED)) {//此处设置成分离线程内存，一旦线程结束，立刻回收内存
		perror("CommRead pthread_attr_setdetachstate failed");
		return -1;
	}

	int result = pthread_create(&m_CommThreadHandle, &child_thread_attr,
			CommReadThreadFunc, (void*) this);
	if (result == -1) {
		printf("CommRead work pthread create failed\n");
	}

	return 1;
}

