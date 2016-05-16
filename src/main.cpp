/*
 #define Dameon true
 #define MAXFILE 65535
 */
#include "main.h"

MainProgram::MainProgram()
{
	RemotePort = 60000;
	strcpy(RemoteHost, "192.168.99.200"); //192.168.99.200
	strcpy(Serdevice, "ttyS1");
	SerSpeed = 115200;
}

MainProgram::~MainProgram()
{

}

int MainProgram::init()
{
	return 1;
}

int MainProgram::start()
{

	//
	//strcpy(m_NetworkConfig.wifi_ssid, "TESTTEST");
	//strcpy(m_NetworkConfig.wifi_key, "87654321");
	//m_NetworkConfig.uci_wifi_config_set();
	//

	if (m_CommHelper.Open(this, Serdevice, SerSpeed) < 0)
	{
		printf("Device open failed.\n");
		exit(1);
	}
	else
	{
		printf("Comm Speed is %d.\n", m_CommHelper.Speed);
	}

	//设置m_tcpClient属性
	strcpy(m_tcpClient.m_remoteHost, RemoteHost); //

	m_tcpClient.m_port = RemotePort;
	m_tcpClient.Open(this);

	if (m_tcpClient.ConnectServer())
	{
		fprintf(stdout, "TCP建立连接成功!\n");
	}
	else
	{
		fprintf(stdout, "TCP建立连接失败!.\n");
		//return -1;
	}

	return 0;
}

int background = 0; /* 后台运行标记  */
int Outlog = 0; //后台运行时保存输出信息
pthread_cond_t m_exitEvent;
pthread_mutex_t mutex; /*互斥锁*/

int main(int argc, char *argv[])
{

	MainProgram m_MainProgram;

	//初始化并解释程序的启动参数
	extern char *optarg;
	int c;

	while ((c = getopt(argc, argv, "e:p:s:bh")) != EOF)
	{
		switch (c)
		{
		case 'e':
			strcpy(m_MainProgram.Serdevice, optarg);
			break;
		case 'p':
			m_MainProgram.RemotePort = atoi(optarg);
			break;
		case 's':
			strcpy(m_MainProgram.RemoteHost, optarg);
			break;
		case 'b':
			background = 1;
			break;
		case '?':
		case 'h':
			printf(
					"Usage: %s [-e <Serdevice>] [-p <RemotePort>] [-s <RemoteHost>] [-b <Background>]\n",
					argv[0]);
			exit(0);
		}
	}

	if (background)
	{
		//创建退出事件句柄
		pthread_mutex_init(&mutex, NULL);
		pthread_cond_init(&m_exitEvent, NULL);

		/*		参数：
		 当 nochdir为零时，当前目录变为根目录，否则不变；
		 当 noclose为零时，标准输入、标准输出和错误输出重导向为/dev/null，也就是不输出任何信 息，否则照样输出。
		 返回值：
		 deamon()调用了fork()，如果fork成功，那么父进程就调用_exit(2)退出，所以看到的错误信息 全部是子进程产生的。如果成功函数返回0，否则返回-1并设置errno。
		 */
		if (daemon(1, 0) != 0) {
			perror("Daemon error!\n");
			exit(1);
		}

		char device[10];
		strcpy(device, "ttyATH0");
		strcpy(m_MainProgram.Serdevice, device);
		/*		for (int i = '0'; i <= '9'; i++) { //自动查找一个可用的设备
		 device[11] = i;
		 //printf ("可用的devce：[%s]\n",device.c_str()) ;
		 if (access(device, F_OK) != -1) {
		 printf("可用的devce：[%s]\n", device + 5);
		 strcpy(m_MainProgram.Serdevice, device + 5);
		 break;
		 }
		 }*/
	}

	m_MainProgram.start();

	if (!background)
	{
		char usercmd[128];
		while (true)
		{
			cin.getline(usercmd, 127);
			switch (usercmd[0])
			{
			case 'q':
			case 'Q':
			{
				if (m_MainProgram.m_tcpClient.Close() == 1)
					fprintf(stdout, "Exit success.\n");
				return 0;
			}
			case '!':
			{
				if (strlen(usercmd) < 2)
				{
					fprintf(stdout, "没有ASCII数据要发送!\n");
					break;
				}
				char Strtmp[128];
				strncpy(Strtmp, usercmd + 1, strlen(usercmd) - 1);
				Strtmp[strlen(usercmd) - 1] = 0;
				m_MainProgram.m_tcpClient.SendData(Strtmp, strlen(Strtmp));
				break;
			}
			case '@':
			{
				if (strlen(usercmd) < 2)
				{
					fprintf(stdout, "没有Hex数据要发送!\n");
					break;
				}
				char SendBuf[64] = { 0 };
				char StrTmp[3];
				int tmp = 0, count = 0;
				char *pusercmd = usercmd;
				pusercmd += 1;
				for (int i = 1; i < (int) strlen(usercmd); i += 3)
				{
					strncpy(StrTmp, pusercmd, 2);
					StrTmp[2] = 0;
					sscanf(StrTmp, "%02X", &tmp);
					pusercmd += 3;
					SendBuf[count++] = tmp;
				}
				m_MainProgram.m_tcpClient.SendData(SendBuf, count);

			}
				break;
			case '#':
			{
				if (strlen(usercmd) < 2)
				{
					fprintf(stdout, "没有ASCII数据要发送到串口!\n");
					break;
				}
				char Strtmp[128];
				strncpy(Strtmp, usercmd + 1, strlen(usercmd) - 1);
				Strtmp[strlen(usercmd) - 1] = 0;
				m_MainProgram.m_CommHelper.SendData(Strtmp, strlen(Strtmp));
				break;
			}
			case '$':
			{
				if (strlen(usercmd) < 2)
				{
					fprintf(stdout, "没有Hex数据要发送到串口!\n");
					break;
				}
				char SendBuf[64] = { 0 };
				char StrTmp[3];
				int tmp = 0, count = 0;
				char *pusercmd = usercmd;
				pusercmd += 1;
				for (int i = 1; i < (int) strlen(usercmd); i += 3)
				{
					strncpy(StrTmp, pusercmd, 2);
					StrTmp[2] = 0;
					sscanf(StrTmp, "%02X", &tmp);
					pusercmd += 3;
					SendBuf[count++] = tmp;
				}
				m_MainProgram.m_CommHelper.SendData(SendBuf, count);
				m_MainProgram.m_CommHelper.ExtractData(SendBuf, count);
			}
				break;
			default:
				fprintf(stdout, "命令未知!请输入 h | H 查看帮助.\n");
				break;
			}
		}
	}
	else
	{
		//sleep(1);
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&m_exitEvent, &mutex);
		pthread_mutex_unlock(&mutex);
		fprintf(stdout, "收到退出信号.\n");
		pthread_cond_destroy(&m_exitEvent);
	}

	return 0;
}
