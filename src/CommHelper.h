/*
 * CommHelper.h
 *
 *  Created on: 2014-4-11
 *      Author: root
 */

#ifndef COMMHELPER_H_
#define COMMHELPER_H_

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <memory>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <pthread.h>

#include "unit.h"

#define SERIALS_RECIEVEDATA_BUFFERSLOT  	16
#define SERIALS_RECIEVEDATA_CMDCOUNTERS		7
#define SERIALS_RECIEVEDATA_BUFFERLENGTH	10

using namespace std;

class MainProgram;

class CommHelper
{

public:
	CommHelper();
	~CommHelper();
	struct termios tio;
	char dev[15];
	int fd;
	int Speed ;
	pthread_t m_CommThreadHandle; //通讯线程句柄

	int SendData(char * str , int len);
	int init_dev();
	int Open(MainProgram *lp ,char * device , int speed);
	int Close();
	int ExtractData(const char * data, int data_len);
	int StripCMD(const char * data, int data_len);
	int MsgProcess(const char * data, int data_len);

private:
	MainProgram * m_MainProgram; //引用
	struct termios oldtio;
	static void *CommReadThreadFunc(void * lparam);

};

#endif /* COMMHELPER_H_ */
