/*
 * adc.h
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

#define ADC_BUF_LEN 4096

uint32_t adc1_avg;
uint32_t adc2_avg;

void adc_init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, uint32_t* adc1_buf, uint32_t* adc2_buf);
void adc_buffer_half_full(ADC_HandleTypeDef* hadc, uint32_t* adc1, uint32_t* adc2);
void adc_buffer_full(ADC_HandleTypeDef* hadc, uint32_t* adc1, uint32_t* adc2);

#endif /* INC_ADC_H_ */
