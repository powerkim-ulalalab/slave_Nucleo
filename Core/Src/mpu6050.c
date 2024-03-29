#include "main.h"

#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;


uint8_t MPU6050_Init(uint8_t sel)
{
	printf("MPU6050_Init\n");

    uint8_t check;
    uint8_t Data;

    check = SW_I2C_ReadControl_8Bit(2, GYRO_DEV, WHO_AM_I_REG);

    if (check == 104) /* 0x68 will be returned by the sensor if everything goes well */
    {
    	printf("Read_ok\n");

    	Data = 0x80;
    	SW_I2C_WriteControl_8Bit(sel, GYRO_DEV, PWR_MGMT_1_REG, Data);
		HAL_Delay(10);
		printf("PWR_MGMT_1_REG RESET_OK\n");
		/* power management register 0X6B we should write all 0's to wake the sensor up */

		Data = 0;
        SW_I2C_WriteControl_8Bit(sel, GYRO_DEV, PWR_MGMT_1_REG, Data); /* PWR_MGMT_1_REG: 0x6B  */
        HAL_Delay(10);
        printf("PWR_MGMT_1_REG OK\n");

        /* Set DATA RATE of 1KHz by writing SMPLRT_DIV register */
        Data = 0x07;
        SW_I2C_WriteControl_8Bit(sel, GYRO_DEV, SMPLRT_DIV_REG, Data); /* SMPLRT_DIV_REG: 0x19 */
        printf("SMPLRT_DIV_REG OK\n");

        /* Set accelerometer configuration in ACCEL_CONFIG Register
        XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g */

        Data = 0x00;
        SW_I2C_WriteControl_8Bit(sel, GYRO_DEV, ACCEL_CONFIG_REG, Data); /* ACCEL_CONFIG_REG: 0x1C */
        printf("ACCEL_CONFIG_REG OK\n");

        /* Set Gyroscopic configuration in GYRO_CONFIG Register
        XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s */
        Data = 0x00;
        SW_I2C_WriteControl_8Bit(sel, GYRO_DEV, GYRO_CONFIG_REG, Data); /* GYRO_CONFIG_REG: 0x1B */
        printf("GYRO_CONFIG_REG OK\n");

        return 0;
    }
    return 1;
}


void MPU6050_Read_Accel(uint8_t sel, uint8_t IICID)
{
    uint16_t raw_accel_x = 0;
    uint16_t raw_accel_y = 0;
    uint16_t raw_accel_z = 0;
    int16_t temp[3] = {0,0,0};
    short mpu6050[3] = {0,0,0};
    static uint8_t CntReset = 0, CntInit = 0;

    int i = 0;
   /* Read 6 BYTES of data starting from ACCEL_XOUT_H register */

//    raw_accel_x = SW_I2C_ReadControl_16Bit(sel, IICID, ACCEL_XOUT_H_REG);
//    raw_accel_y = SW_I2C_ReadControl_16Bit(sel, IICID, ACCEL_YOUT_H_REG);
//    raw_accel_z = SW_I2C_ReadControl_16Bit(sel, IICID, ACCEL_ZOUT_H_REG);


    raw_accel_x = (SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_XOUT_H_REG) << 8 | SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_XOUT_L_REG) );
    raw_accel_y = (SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_YOUT_H_REG) << 8 | SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_YOUT_L_REG) );
    raw_accel_z = (SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_ZOUT_H_REG) << 8 | SW_I2C_ReadControl_8Bit(sel, IICID, ACCEL_ZOUT_L_REG) );

    temp[0] = (int16_t)raw_accel_x;
    temp[1] = (int16_t)raw_accel_y;
    temp[2] = (int16_t)raw_accel_z;


    for (i = 0;  i < 3; i++)
    {
    	mpu6050[i] = ss_Kalman(temp[i], i);
	}

    if( (mpu6050[0] == mpu6050[1]) && (mpu6050[1] == mpu6050[2]) && (mpu6050[0] != 0) )
    {
		SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x80);
		HAL_Delay(10);

		SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x00);
		HAL_Delay(10);

		SW_I2C_WriteControl_8Bit(sel, IICID, SMPLRT_DIV_REG,	0x00);
		SW_I2C_WriteControl_8Bit(sel, IICID, GYRO_CONFIG_REG,	0x00);
		SW_I2C_WriteControl_8Bit(sel, IICID, ACCEL_CONFIG_REG,	0x00);

		//i2c_device_write(GYRO_DEV, 0x1C, 0x18); //ACCE CONFIG

		CntReset ++;
    }


    else if( (mpu6050[0] == mpu6050[1]) || (mpu6050[1] == mpu6050[2]) )  //-32767을 띄는 형태
    {
		if( (mpu6050[0] == -32767) || (mpu6050[2] == -32767) )
		{
			SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x80);
			HAL_Delay(10);

			SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x00);
			HAL_Delay(10);

			SW_I2C_WriteControl_8Bit(sel, IICID, SMPLRT_DIV_REG,	0x00);
			SW_I2C_WriteControl_8Bit(sel, IICID, GYRO_CONFIG_REG,	0x00);
			SW_I2C_WriteControl_8Bit(sel, IICID, ACCEL_CONFIG_REG,	0x00);

			//i2c_device_write(GYRO_DEV, 0x1C, 0x18); //ACCE CONFIG

			CntReset ++;
		}

    }
    else
    {
	  CntReset = 0;
    }

	if( (mpu6050[0] == mpu6050[1]) && (mpu6050[1] == mpu6050[2]) && (mpu6050[0] == 0) ) //중간에 뺏따가 꽂았을 때
	{
		CntInit ++;
	}

	if( CntInit > 50 )
	{
		CntInit = 0;

		SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x80);
		HAL_Delay(10);

		SW_I2C_WriteControl_8Bit(sel, IICID, PWR_MGMT_1_REG,	0x00);
		HAL_Delay(10);

		SW_I2C_WriteControl_8Bit(sel, IICID, SMPLRT_DIV_REG,	0x00);
		SW_I2C_WriteControl_8Bit(sel, IICID, GYRO_CONFIG_REG,	0x00);
		SW_I2C_WriteControl_8Bit(sel, IICID, ACCEL_CONFIG_REG,	0x00);

		HAL_Delay(100);
	}

    for (i = 0;  i < 3; i++)
	{
    	printf("%d/",mpu6050[i]);
	}
    printf("\n");

}























































