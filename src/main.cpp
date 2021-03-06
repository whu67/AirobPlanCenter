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
	//begin to read config.ini
	if(!read_profile_string("MapHost", "IP", RemoteHost, 20, "192.168.199.23", "./config.ini"))
	{
		printf("Read ini file failed!, Programme exit!\n");
		return -1;
	}
	else
	{
		printf("Map Host = %s \n", RemoteHost);
	}
	RemotePort = read_profile_int("MapPort", "PORT", 10001, "./config.ini");
	printf("Map Port = %d \n", RemotePort);

	if(!read_profile_string("ModifyHost", "IP", ModifyHost, 20, "192.168.199.23", "./config.ini"))
	{
		printf("Read ini file failed!, Programme exit!\n");
		return -1;
	}
	else
	{
		printf("Modify Host = %s \n", ModifyHost);
	}
	ModifyPort = read_profile_int("ModifyPort", "PORT", 10001, "./config.ini");
	printf("Map Port = %d \n", ModifyPort);

	if(!read_profile_string("Device", "Serial", Serdevice, 20, "/dev/ttyS1", "./config.ini"))
	{
		printf("Read ini file failed!, Programme exit!\n");
		return -1;
	}
	else
	{
		printf("Serial Device = %s \n", Serdevice);
	}

	SerSpeed = read_profile_int("Speed", "SerSpeed", 115200, "./config.ini");
	//end to read config.ini

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
		fprintf(stdout, "TCP connected successfully!\n");
	}
	else
	{
		fprintf(stdout, "TCP connected failed!.\n");
		//return -1;
	}

	m_MapDealer.open(this);
	m_MapDealer.init();
	if(m_MapDealer.CreateUpdateMapandPlanThread())
	{
		fprintf(stdout, "Map Update and Road Plan thread create successfully!\n");
	}
	else
	{
		fprintf(stdout, "Map Update and Road Plan thread create failed!\n");
		exit(1);
	}

	m_ParameterAdjuster.open(&m_CommHelper);//this);
	if(!m_ParameterAdjuster.init())
	{
		fprintf(stdout, "Parameter Adjuster initialize failed!\n");
		exit(1);
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
			case 'd':
			case 'D':
			{
				//output Fix array to /home/map.txt
				unsigned char tmpoutfix[MAX_MAP_GRID][MAX_MAP_GRID];
				memcpy(tmpoutfix, m_MainProgram.m_MapDealer.Fix, MAX_MAP_GRID*MAX_MAP_GRID);
				ofstream outputstr;
				outputstr.open("//home/map.txt", ios::out);
				for(int i  = 0; i < MAX_MAP_GRID; i++)
				{
					for(int t = 0; t < MAX_MAP_GRID; t++)
					{
						outputstr<<(int)tmpoutfix[i][t]<<",";
					}
					outputstr<<endl;
				}
				outputstr.close();
				fprintf(stdout, "Dump map data to  /home/map.txt done!");
			}
				break;
			case 'p':
			case 'P':
			{
				//Get a Photo by m_ParameterAdjster
				m_MainProgram.m_ParameterAdjuster.RemoveFirstPhoto();
				if(m_MainProgram.m_ParameterAdjuster.GetPhoto(0x00,0x00,0x00))
				{
					fprintf(stdout, "Get photo successfully!\n");
				}
				else
				{
					fprintf(stdout, "Get photo failed!\n");
				}
			}
				break;
			case 'a':
			case 'A':
			{
				if(m_MainProgram.m_ParameterAdjuster.GetTmpPhoto(0x00,0x00,0x00))
				{
					fprintf(stdout, "Get adjust photo successfully!\n");
				}
				else
				{
					fprintf(stdout, "Get adjust photo failed!\n");
				}
//				unsigned char test[10];
//				memset(test, 0x00, 10);
//				test[0] = 0xF5;
//				test[1] = 0xF5;
//				AdjustData m_RevAdjustData;
//				m_RevAdjustData.data = new char[11];
//				Hexstrncpy(m_RevAdjustData.data, (char*)test, 10);
//				m_RevAdjustData.data[10] = 0;
//				m_RevAdjustData.data_len = 10;
//				m_RevAdjustData.priority = 10;
// 				m_MainProgram.m_ParameterAdjuster.AdjustProcess(&m_RevAdjustData);
			}
				break;
			default:
				fprintf(stdout, "unkown order!enter h | H for manual.\n");
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
		fprintf(stdout, "Roger exit signal.\n");
		pthread_cond_destroy(&m_exitEvent);
	}

	return 0;
}
