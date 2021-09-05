/*
 * dac.c
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#include "dac.h"

void dac_init(DAC_HandleTypeDef* hdac) {
	HAL_DAC_Start(hdac, DAC_CHANNEL_1);
}
