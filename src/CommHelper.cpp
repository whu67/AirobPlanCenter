/*
 * CommHelper.cpp
 *
 *  Created on: 2014-4-11
 *      Author: root
 */
#include "main.h"
#include "CommHelper.h"

CommHelper::CommHelper()
{
	strcpy(dev, "/dev/");
	Speed = 115200;
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

	return 1;
}

//消息
int CommHelper::MsgProcess(const char * data, int data_len)
{

	print_hex(data, data_len, "串口有效数据");

	m_MainProgram->m_tcpClient.AddToSendQueue(data, data_len);

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

	//printf("CommReadThreadFunc start in device fd %d.\n", pCommHelper->fd);
	int n, max_fd, len, count = 0;
	fd_set input;
	max_fd = pCommHelper->fd + 1;
	/* Do the select */
	struct timeval timeout;
	char buffer[1024];
	char * ptr = buffer;
	int ret = 0;

	while (true)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 30 * 1000; //20ms
		FD_ZERO(&input);
		FD_SET(pCommHelper->fd, &input);
		n = select(max_fd, &input, NULL, NULL, &timeout);
		if (n < 0)
			perror("select failed");

		else if (n == 0)
		{
			if ( (count - ret )> 0)
			{   ///
				buffer[count] = 0;

				printf("com rev :(Str)[%s]\n", buffer);
				fprintf(stdout, "com rev :(Hex)[%02X",
						(unsigned char) buffer[0]);
				for (int i = 1; i < count; i++)
				{
					fprintf(stdout, " %02X", (unsigned char) buffer[i]);
				}
				fprintf(stdout, "]\n");

				ret = pCommHelper->ExtractData(buffer, count);
				count = count - ret;
				if (count < 512)
				{
					Hexstrncpy(buffer, buffer + ret, count);
					ptr = buffer + count;
				}
				else
				{
					count = 0;
					ptr = buffer;
				}
				ret = count ; //用来标记下次是否有数据增加
			}
		}
		else
		{
			ioctl(pCommHelper->fd, FIONREAD, &len);
			if (!len)
			{
				fprintf(stderr, "Communication closed by server\n");
				break;
			}

			len = read(pCommHelper->fd, ptr, len);
			ptr += len;
			count += len;
		}
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
