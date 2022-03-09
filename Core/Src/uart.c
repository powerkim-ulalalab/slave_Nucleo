#include "uart.h"
#include "Timer.h"
#include "Modbus.h"
#include <stdlib.h>

/******************RX_SlaveData*************************/
extern volatile uint16_t RX_DataCnt;
extern uint8_t MB_RxBuf[RXBUFLEN];
extern uint8_t RX_Data;
/***************************************************/
extern uint8_t RxTimerFlag;

extern UART_HandleTypeDef huart2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2) // Modbus Slave TX
	{
		RxTimerFlag = ON;

		if(RxTimerFlag)
		{
			MB_RxBuf[RX_DataCnt++] = RX_Data;
		}

		HAL_UART_Receive_IT(&huart2, &RX_Data, 1);
	}

}

