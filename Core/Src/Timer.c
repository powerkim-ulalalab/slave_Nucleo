#include "timer.h"
#include "Modbus.h"

uint8_t RxTimerFlag = 0;
volatile uint16_t RxTimerCnt = 0;


void Rx_TimerStartStop()
{
	if(RxTimerFlag)
	{
		HAL_TIM_Base_Start_IT(&htim1);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim1);
	}

}


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)//0.1ms
{
  if(htim == &htim1)
  {
	  if(RxTimerCnt++ > 600)
	  {

		  RxTimerCnt = 0;
		  RxTimerFlag = 0;

	  }
  }

}


