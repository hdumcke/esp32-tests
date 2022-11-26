/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.5.24
 * 作者: 
 */

#include "SCSerial.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

SCSerial::SCSerial()
{
	IOTimeOut = 10;
	uart_port_num = 0;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 10;
	uart_port_num = 0;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 10;
	uart_port_num = 0;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut)
{

	return uart_read_bytes(uart_port_num, nDat, nLen, TimeOut / portTICK_RATE_MS);
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	//uart_read_bytes(uart_port_num, nDat, nLen, IOTimeOut / portTICK_RATE_MS);
	return uart_read_bytes(uart_port_num, nDat, nLen, IOTimeOut / portTICK_RATE_MS);
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	// uart_write_bytes(uart_port_num, nData, nlen);
	return uart_write_bytes(uart_port_num, nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	// uart_write_bytes(uart_port_num, &bDat, 1);
	return uart_write_bytes(uart_port_num, &bDat, 1);
}

void SCSerial::rFlushSCS()
{
	uart_flush(uart_port_num);
}

void SCSerial::wFlushSCS()
{
}