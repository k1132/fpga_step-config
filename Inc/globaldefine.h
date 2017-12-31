
#ifndef __GLOBALDEFINE_H
#define __GLOBALDEFINE_H

#include <stdint.h>

#define COMM_BUFFER_SIZE 128 
#define COMM_BUFFER_NUM  32

//ͨ��֡��ʽ
typedef struct COMM_FRAM_TAG
{
	uint16_t crc;							//����0��ʾ��ʹ��CRC������Ĭ�ϲ�ʹ��
	uint8_t id;								//����ID������˵��Ŀ����ַ
	uint8_t cmd;							//��������0x00(0x80):д(��)ARM���0x01(0x81)��д(��)FPGA���
	uint16_t adr;							//��ʼ��ַ
	uint16_t len;							//���ݳ���
	uint8_t  Bbuf[COMM_BUFFER_SIZE];		//���ݻ�����,�ֽڷ�ʽ��д
//uint16_t Dbuf[COMM_BUFFER_SIZE/2];		//���ݻ�����,�ַ�ʽ��д
	
}COMM_FRAM;

#endif /* __GLOBALDEFINE_H */
