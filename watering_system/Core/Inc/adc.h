/*
 * adc.h
 *
 *  Created on: May 27, 2025
 *      Author: vladimir
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

// Public variables
extern uint16_t lux;
extern uint16_t moisture;
extern uint8_t convCompleted;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// Function prototypes

void My_ADC_Init(void);
void ADC_StartConversion(void);
void ADC_GetValues(uint16_t* lux_val, uint16_t* moisture_val);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* INC_ADC_H_ */
