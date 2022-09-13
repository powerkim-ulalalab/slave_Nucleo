/*
 * vibration_sen.h
 *
 *  Created on: Aug 25, 2022
 *      Author: KimGiun
 */

#ifndef INC_VIBRATION_SEN_H_
#define INC_VIBRATION_SEN_H_
#include "main.h"

#define GYRO_DEV (0x68 << 1)   //0x68이지만 한비트 좌쉬프트 시킨다.

void i2c_device_read(uint8_t sel, unsigned char dev_addr, unsigned char reg, unsigned char *wr_buf, unsigned char size);
void i2c_device_write(uint8_t sel, unsigned char dev_addr, unsigned char reg, unsigned char value );
void ReadI2C(uint8_t sel);  //전역변수에 저장한다.

void i2c_start(uint8_t sel);
void i2c_stop(uint8_t sel);
unsigned char i2c_write(uint8_t sel, unsigned char value );
unsigned char i2c_read(uint8_t sel, unsigned char ack );
short ss_Kalman(short fval, unsigned char portnum);



#endif /* INC_VIBRATION_SEN_H_ */
