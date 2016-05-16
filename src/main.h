/*
 * main.h
 *
 *  Created on: 2014-8-26
 *      Author: root
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <sys/stat.h>
#include <pthread.h>
#include <signal.h>
#include <assert.h>

#include "unit.h"
#include "TCPClient.h"
#include "CommHelper.h"


using namespace std;

class MainProgram
{
 public:
	TCPClient m_tcpClient;
	CommHelper m_CommHelper;

	int RemotePort;
	char RemoteHost[20];
	int SerSpeed ;
	char Serdevice[10];
 public:
	MainProgram();
	~MainProgram();
	int init();
	int start();
};


#endif /* MAIN_H_ */
