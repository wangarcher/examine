#ifndef _UARTCOM_H
#define _UARTCOM_H

union chartofloat
{
    char ch[4];
	float fl;
};

union chtosh
{
    short sh;
    unsigned char ch[2];
};

int UART_Init_Baud(int fd,int baud);

unsigned char GET_IMU_CRC(unsigned char *imu_data_p,int len);


#endif
