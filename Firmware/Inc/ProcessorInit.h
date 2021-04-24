/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef PROCESSOR_INIT_H
#define PROCESSOR_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void SystemClock_Config(void);	
void Error_Handler(void);
void assert_failed(uint8_t *file, uint32_t line);

#ifdef __cplusplus
}
#endif

#endif 
