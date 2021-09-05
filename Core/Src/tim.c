/*
 * tim.c
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#include "tim.h"

void tim_init(TIM_HandleTypeDef* htim) {
	HAL_TIM_Base_Start_IT(htim);
}
