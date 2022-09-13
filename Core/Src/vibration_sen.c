#include "main.h"

#define TRUE	1
#define FALSE	0


short MPU6050[5][7] = {0,};    //전역변수로 사용한다.

//for MPU-6050
float g_xPredict_MPU[6] = {0,0,0,0,0,0};
float g_x_MPU[6] = {0,0,0,0,0,0};
float g_PPredict_MPU[6] = {0,0,0,0,0,0};
float g_P_MPU[6] = {1,1,1,1,1,1};
float g_K_MPU[6] = {0,0,0,0,0,0};
float g_z_MPU[6] = {0,0,0,0,0,0};

short ss_Kalman(short fval, unsigned char portnum)
{
    float l_z ;

    // predict..
    g_xPredict_MPU[portnum]=g_x_MPU[portnum];
    g_PPredict_MPU[portnum]=g_P_MPU[portnum] + kal_Q;

    // Correct..
    g_K_MPU[portnum]=g_PPredict_MPU[portnum]/(g_PPredict_MPU[portnum] + kal_R);
    l_z=(float)fval;
    g_x_MPU[portnum]=g_xPredict_MPU[portnum]+g_K_MPU[portnum]*(l_z-g_xPredict_MPU[portnum]);
    g_P_MPU[portnum]=(1 - g_K_MPU[portnum])*g_PPredict_MPU[portnum];

    return (short)g_x_MPU[portnum];
}

void i2c_device_read(uint8_t sel, unsigned char dev_addr, unsigned char reg, unsigned char *wr_buf, unsigned char size)
{
	unsigned char ix;
	uint8_t returnack = 0;

	i2c_start_condition(sel);

    SW_I2C_Write_Data(sel,(uint8_t)dev_addr);
    if (!i2c_check_ack(sel))
    {
        returnack = FALSE;
    }

    SW_I2C_Write_Data(sel,(uint8_t)reg);

    if (!i2c_check_ack(sel))
    {
        returnack = FALSE;
    }

    i2c_start_condition(sel);

    SW_I2C_Write_Data(sel,(uint8_t)dev_addr|0x01);
	if (!i2c_check_ack(sel))
	{
		returnack = FALSE;
	}

	for( ix=1; ix<=size; ix++ )
	{
		*wr_buf = SW_I2C_Read_Data(sel);

		if (!i2c_check_ack(sel))
		{
			returnack = FALSE;
		}
		wr_buf++;
	}

	i2c_stop_condition(sel);             // if no stop() can get continuous read

}

void i2c_device_write(uint8_t sel, unsigned char dev_addr, unsigned char reg, unsigned char value )
{

	i2c_start(sel);
    SW_I2C_Write_Data(sel, dev_addr);
    SW_I2C_Write_Data(sel, reg);
    SW_I2C_Write_Data(sel, value);

//    i2c_write(dev_addr);    // send chip addr and set data to write mode
//    i2c_write(reg);      // send 1st address
//    i2c_write(value);       // write data

    i2c_stop(sel);
}

void ReadI2C(uint8_t sel)  //전역변수에 저장한다.
{
	int16_t temp[7] = {0,0,0,0,0,0,0};
	uint8_t raw_data[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t ix = 0;
	uint8_t iy = 0;

	static uint8_t CntReset = 0, CntInit = 0;

	i2c_device_read(sel,GYRO_DEV, 0x3B, raw_data, 14); //burst로 전체를 읽어들인다.

	temp[0] = ( (raw_data[0] << 8) | raw_data[1] );
	temp[1] = ( (raw_data[2] << 8) | raw_data[3] );
	temp[2] = ( (raw_data[4] << 8) | raw_data[5] );
	temp[3] = ( (raw_data[8] << 8) | raw_data[9]  );
	temp[4] = ( (raw_data[10]<< 8) | raw_data[11] );
	temp[5] = ( (raw_data[12]<< 8) | raw_data[13] );
	temp[6] = ( (raw_data[6] << 8) | raw_data[7] ); //온도데이터는 마지막으로


	//osi_LockObjLock(&g_LockObjCommBoard, OSI_WAIT_FOREVER);
	//센서가 이상할때는 가속도가 3점이 전부 상한을 친다. 32766, 32763 등이 나온다.
	//if( (temp[0] < 32760) && (temp[1] < 32760) && (temp[2] < 32760) )

		for( iy = 0 ; iy < 6; iy ++ )       //6개 진동데이터는 칼만을 통과한다.
		{
			//MPU6050[ix][iy] = temp[iy];   //최종데이터
			MPU6050[ix][iy] = ss_Kalman(temp[iy], iy);   //최종데이터

			//printf("%d/%d\n",ix,MPU6050[ix][iy]);
		}
		MPU6050[ix][iy] = temp[6] / 340 + 36.53; //온도계산

		for(int i = 0 ; i < 8; i++)
				{
					printf("%d/",temp[i]);
				}
				printf("\n");
		ix ++;



		if( ix == 5 ) ix = 0;

	//osi_LockObjUnlock(&g_LockObjCommBoard);

	if( (MPU6050[0][0] == MPU6050[0][1]) && (MPU6050[0][1] == MPU6050[0][2]) && (MPU6050[0][0] != 0) )
	{
		i2c_device_write(sel,GYRO_DEV, 0x6B, 0x80); //ACCE CONFIG
		HAL_Delay(10);
		//osi_Sleep(10);
		i2c_device_write(sel,GYRO_DEV, 0x6B, 0x00); //ACCE CONFIG
		HAL_Delay(10);
		//osi_Sleep(10);
		i2c_device_write(sel,GYRO_DEV, 0x19, 0x00); //샘플링 속도
		i2c_device_write(sel,GYRO_DEV, 0x1B, 0x00); //GYRO CONFIG
		i2c_device_write(sel,GYRO_DEV, 0x1C, 0x00); //ACCE CONFIG
		//i2c_device_write(GYRO_DEV, 0x1C, 0x18); //ACCE CONFIG

		CntReset ++;
	}
	else if( (MPU6050[0][0] == MPU6050[0][1]) || (MPU6050[0][1] == MPU6050[0][2]) )  //-32767을 띄는 형태
	{
		if( (MPU6050[0][0] == -32767) || (MPU6050[0][2] == -32767) )
		{
			i2c_device_write(sel,GYRO_DEV, 0x6B, 0x80); //ACCE CONFIG
			HAL_Delay(10);
			//osi_Sleep(10);
			i2c_device_write(sel,GYRO_DEV, 0x6B, 0x00); //ACCE CONFIG
			HAL_Delay(10);
			//osi_Sleep(10);
			i2c_device_write(sel,GYRO_DEV, 0x19, 0x00); //샘플링 속도
			i2c_device_write(sel,GYRO_DEV, 0x1B, 0x00); //GYRO CONFIG
			i2c_device_write(sel,GYRO_DEV, 0x1C, 0x00); //ACCE CONFIG
			//i2c_device_write(GYRO_DEV, 0x1C, 0x18); //ACCE CONFIG

			CntReset ++;
		}
	}
	else
	{
		CntReset = 0;
	}

	if( (MPU6050[0][0] == MPU6050[0][1]) && (MPU6050[0][1] == MPU6050[0][2]) && (MPU6050[0][0] == 0) ) //중간에 뺏따가 꽂았을 때
	{
		CntInit ++;
	}
	if( CntInit > 50 )
	{
		CntInit = 0;
		i2c_device_write(sel, GYRO_DEV, 0x6B, 0x80); //ACCE CONFIG
		HAL_Delay(10);
		//osi_Sleep(10);
		i2c_device_write(sel, GYRO_DEV, 0x6B, 0x00); //ACCE CONFIG
		HAL_Delay(10);
		//osi_Sleep(10);
		i2c_device_write(sel, GYRO_DEV, 0x19, 0x00); //샘플링 속도
		i2c_device_write(sel, GYRO_DEV, 0x1B, 0x00); //GYRO CONFIG
		i2c_device_write(sel, GYRO_DEV, 0x1C, 0x00); //ACCE CONFIG
		HAL_Delay(100);
		//osi_Sleep(100);
	}
}

void i2c_start(uint8_t sel)
{
	sda_out_mode(sel);

	scl_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	sda_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);

	scl_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	sda_low(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	scl_low(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);

}

void i2c_stop(uint8_t sel)
{
	sda_out_mode(sel);

	sda_low(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	scl_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	sda_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);
	scl_high(sel);
	TIMER__Wait_us(SW_I2C_WAIT_TIME);

#if 0
    SDAOutMode(1);
    //stop condition
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, 0 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 1 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, 1 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 1 );
    //UtilsDelay(ul_delay);
#endif

}

unsigned char i2c_write(uint8_t sel, unsigned char value )
{
    unsigned char  ix, ack=0;

    //I2C_SDA를 출력으로 전환한다.
	sda_out_mode(sel);

    for(ix=0x80; ix>0; ix/=2)
    {
    	if(ix & value)
    	{
    		sda_high(sel);
    	}
    	else
    	{
    		sda_low(sel);
    	}
    	scl_high(sel);
		TIMER__Wait_us(SW_I2C_WAIT_TIME);
		scl_low(sel);
    }

    sda_high(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    sda_in_mode(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_high(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    ack = (unsigned char)SW_I2C_ReadVal_SDA(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low(sel);

    sda_out_mode(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_low(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    return ack;

}

unsigned char i2c_read(uint8_t sel, unsigned char ack )
{
    unsigned char  ix, rd_data=0;

    sda_high(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_in_mode(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

#if 0
    // 여기서 I2C_SDA를 입력으로 전환한다.
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, 1 );
    //UtilsDelay(ul_delay);
    SDAOutMode(0);
    //UtilsDelay(ul_delay);
#endif

    for(ix=0x80; ix>0; ix/=2)               // shift bit for masking
    {


    	scl_high(sel);
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    	if( !(SW_I2C_ReadVal_SDA(sel)) ) rd_data = (rd_data | ix);
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    	scl_low(sel);
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);

#if 0
        GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 1 );                      // clk for SENSI-BUS
        //UtilsDelay(ul_delay);
        if ( GPIO_IF_Get( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin ) ) rd_data = (rd_data | ix);   // read bit
        //UtilsDelay(ul_delay);
        GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 0 );
        //UtilsDelay(ul_delay);
#endif

    }

    sda_out_mode(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_high(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    if(ack)
    {
    	sda_low(sel);
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
    else
    {
    	sda_high(sel);
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }

    scl_high(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    scl_low(sel);
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    return( rd_data );
#if 0
	sda_high(sel);
    // 다시 I2C_SDA를 출력으로 전환한다.
    SDAOutMode(1);  // SDA -> Output
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, 1 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, (ack)? 0 : 1);    // in case of "ack==1" pull down DATA-Line
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 1 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SCK, g_uiI2CsckPort, g_ucI2CsckPin, 0 );
    //UtilsDelay(ul_delay);
    GPIO_IF_Set( GPIO_I2C_SDA, g_uiI2CsdaPort, g_ucI2CsdaPin, 1);    // release DATA-line

    return( rd_data );

#endif
}










