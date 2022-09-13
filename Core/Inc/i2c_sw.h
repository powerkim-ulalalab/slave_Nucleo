#ifndef __I2C_SW_H
#define __I2C_SW_H

/* includes */
#include "main.h"

/* defines */
#define  SW_I2C_WAIT_TIME  2000/*1000:500us*/

#define   GPIO_SW_I2C1_SCL			GPIOB
#define   GPIO_SW_I2C1_SDA			GPIOB
#define   GPIO_SW_I2C1_SCL_PIN		GPIO_SCL1_Pin//
#define   GPIO_SW_I2C1_SDA_PIN		GPIO_SDA1_Pin//

#define   GPIO_SW_I2C2_SCL			GPIOB
#define   GPIO_SW_I2C2_SDA			GPIOB
#define   GPIO_SW_I2C2_SCL_PIN		GPIO_SCL2_Pin
#define   GPIO_SW_I2C2_SDA_PIN		GPIO_SDA2_Pin

/*
#define   GPIO_SW_I2Cx_SCL			GPIOx
#define   GPIO_SW_I2Cx_SDA			GPIOx
#define   GPIO_SW_I2Cx_SCL_PIN		GPIO_PIN_x
#define   GPIO_SW_I2Cx_SDA_PIN		GPIO_PIN_x
*/

#define SW_I2C1		1
#define SW_I2C2		2

//#define SW_I2Cx		x


void sda_high(uint8_t sel);
void sda_low(uint8_t sel);

void scl_low(uint8_t sel);
void scl_high(uint8_t sel);
void TIMER__Wait_us(__IO uint32_t nCount);

void SW_I2C_ReadControl_8Bit_Test(uint8_t sel, uint8_t IICID);


void TIMER__Wait_us(__IO uint32_t nCount);
/* functions */
void SW_I2C_initial(void);

void i2c_port_initial(uint8_t sel);		

uint8_t SW_I2C_ReadVal_SDA(uint8_t sel);

void SW_I2C_Write_Data(uint8_t sel, uint8_t data);
uint8_t SW_I2C_Read_Data(uint8_t sel);

uint8_t SW_I2C_WriteControl_8Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint8_t data);
uint8_t SW_I2C_WriteControl_8Bit_OnlyRegAddr(uint8_t sel, uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_WriteControl_16Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint16_t data);

uint8_t SW_I2C_ReadControl_8Bit_OnlyRegAddr(uint8_t sel, uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_ReadControl_8Bit_OnlyData(uint8_t sel, uint8_t IICID);
uint16_t SW_I2C_ReadControl_16Bit_OnlyData(uint8_t sel, uint8_t IICID);
uint8_t SW_I2C_ReadControl_8Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr);
uint16_t SW_I2C_ReadControl_16Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr);

uint8_t SW_I2C_ReadnControl_8Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));
uint8_t SW_I2C_Multi_ReadnControl_8Bit(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));
uint8_t SW_I2C_Check_SlaveAddr(uint8_t sel, uint8_t IICID);

uint8_t SW_I2C_UTIL_WRITE(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint8_t data);
uint8_t SW_I2C_UTIL_Read(uint8_t sel, uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_UTIL_Read_Multi(uint8_t sel, uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));

#endif  /* __I2C_SW_H */
