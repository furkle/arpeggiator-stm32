/*
 * adc.c
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#include "main.h"
#include "adc.h"

extern uint32_t adc1_avg;
extern uint32_t adc2_avg;

void adc_init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, uint32_t* adc1_buf, uint32_t* adc2_buf) {
	HAL_ADC_Start_DMA(hadc1, adc1_buf, ADC_BUF_LEN);
	HAL_ADC_Start_DMA(hadc2, adc2_buf, ADC_BUF_LEN);
}

void adc_buffer_half_full(ADC_HandleTypeDef* hadc, uint32_t* adc1_buf, uint32_t* adc2_buf) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	uint32_t adc1_sum = 0;
	uint32_t adc2_sum = 0;

	uint16_t i;
	if (hadc->Instance == ADC1) {
	  for (i = 0; i < ADC_BUF_LEN / 2; i += 1) {
		  adc1_sum += (uint32_t)&adc1_buf[i];
	  }

	  adc1_avg = adc1_sum / ADC_BUF_LEN / 2;
	} else if (hadc->Instance == ADC2) {
	  for (i = 0; i < ADC_BUF_LEN / 2; i += 1) {
		  adc2_sum += (uint32_t)&adc2_buf[i];
	  }

	  adc2_avg = adc2_sum / ADC_BUF_LEN / 2;
	}
}

void adc_buffer_full(ADC_HandleTypeDef* hadc, uint32_t* adc1_buf, uint32_t* adc2_buf) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	uint32_t adc1_sum = 0;
	uint32_t adc2_sum = 0;
	uint16_t i;
	if (hadc->Instance == ADC1) {
	  for (i = ADC_BUF_LEN / 2; i < ADC_BUF_LEN; i += 1) {
		  adc1_sum += (uint32_t)adc1_buf[i];
	  }

	  adc1_avg = adc1_sum / ADC_BUF_LEN / 2;
	} else if (hadc->Instance == ADC2) {
	  for (i = ADC_BUF_LEN / 2; i < ADC_BUF_LEN; i += 1) {
		  adc2_sum += (uint32_t)adc2_buf[i];
	  }

	  adc2_avg = adc2_sum / ADC_BUF_LEN / 2;
	}
}
