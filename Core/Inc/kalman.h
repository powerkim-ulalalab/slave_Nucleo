/*
 * kalman.h
 *
 *  Created on: 2022. 8. 26.
 *      Author: KimGiun
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"


#define	KALMAN_SIZE	10
#define kal_Q 0.01
#define kal_R 0.25

typedef struct _KALMAN_HandleTypeDef
{
	float x_predict[KALMAN_SIZE];	/* x: adulation Value*/
	float x[KALMAN_SIZE];

	float p_predict[KALMAN_SIZE];	/* p: Error_Covariance Value*/
	float p[KALMAN_SIZE];

	float k[KALMAN_SIZE];			/* k: Kalman Benefit*/

	float *ptr_adulation_value;

	uint8_t kalman_ai_ch;

}KALMAN_HandleTypeDef;

//extern KALMAN_HandleTypeDef hkalman_cv;
//extern float kalman_cv_value[CV_CH];
//
//extern KALMAN_HandleTypeDef hkalman_kth;
//extern float kalman_kth_value[KTH_CH];
//
//void Action_Kalman_Init(KALMAN_HandleTypeDef *hkalman, float *p_ai_kalman);
//uint8_t u16_Kalman(KALMAN_HandleTypeDef *hkalman, uint8_t ai_ch);
//float Action_Apply_Kalman_Filter(float raw_data, uint8_t ai_ch, KALMAN_HandleTypeDef *hkalman);


#endif /* INC_KALMAN_H_ */
