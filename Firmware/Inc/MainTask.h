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

#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((__packed__)) {
	uint32_t Timestamp;

	float current_setpoint;
	float current_measurement;
	float current_filtered;

	float omega_measurement;
	float omega_filtered;

	float integral;
	float PI_out;
	float duty;

} ControllerDebug_t;

void MainTask(void * pvParameters);

#ifdef __cplusplus
}
#endif

#endif 
