
#ifndef __GLOBALDEFINE_H
#define __GLOBALDEFINE_H

#include <stdint.h>

#define COMM_BUFFER_SIZE 128 
#define COMM_BUFFER_NUM  32

//通信帧格式
typedef struct COMM_FRAM_TAG
{
	uint16_t crc;							//等于0表示不使用CRC，我们默认不使用
	uint8_t id;								//板子ID，或者说是目标板地址
	uint8_t cmd;							//命令类型0x00(0x80):写(读)ARM命令，0x01(0x81)：写(读)FPGA命令；
	uint16_t adr;							//起始地址
	uint16_t len;							//数据长度
	uint8_t  Bbuf[COMM_BUFFER_SIZE];		//数据缓冲区,字节方式读写
//uint16_t Dbuf[COMM_BUFFER_SIZE/2];		//数据缓冲区,字方式读写
	
}COMM_FRAM;

#endif /* __GLOBALDEFINE_H */
