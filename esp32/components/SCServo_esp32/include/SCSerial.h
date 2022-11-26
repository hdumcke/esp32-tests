/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者: 
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

// #include "HardwareSerial.h"

#include "SCS.h"

class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);

protected:
	int writeSCS(unsigned char *nDat, int nLen);//输出nLen字节
	int readSCS(unsigned char *nDat, int nLen);//输入nLen字节
	int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut);
	int writeSCS(unsigned char bDat);//输出1字节
	void rFlushSCS();//
	void wFlushSCS();//
public:
	unsigned long IOTimeOut;//输入输出超时
	int uart_port_num;//串口number
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

#endif
