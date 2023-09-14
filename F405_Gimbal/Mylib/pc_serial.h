#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "pc_uart.h"
#include "algorithmOfCRC.h"
#include "ins.h"

#include "FreeRTOS.h"
#include "task.h"

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

extern PCRecvData pc_recv_data;

#endif // !_PC_SERIAL_H
