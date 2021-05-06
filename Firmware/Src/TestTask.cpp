/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "TestTask.h"

// FreeRTOS include
#ifdef USE_FREERTOS_CMSIS
#include <cmsis_os.h>
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#endif
#include <Priorities.h>

// High-res timebase
#include <PrecisionSysTick/PrecisionSysTick.h> // for HAL_GetHighResTick()

/* Include Periphiral drivers */
#include <IO/IO.hpp>
#include <PWM/PWM.hpp>

#include <cmath>


void TestTask(void * pvParameters)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */
	/*UART * uart = new UART(UART::PORT_UART2, 1612800, 100, true);
	LSPC * lspc = new LSPC(uart, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY);
	Debug::AssignDebugCOM((void*)lspc);

	TaskHandle_t CPULoadTaskHandle;
	xTaskCreate(CPU_Load, (char *)"CPU Load", 256, (void*) lspc, CPULOAD_PRIORITY, &CPULoadTaskHandle);*/

	//IO * led = new IO(GPIOC, GPIO_PIN_6);
	//IO * debug = new IO(GPIOA, GPIO_PIN_15);
	//Encoder * encoder = new Encoder(Encoder::TIMER4, true);
	PWM led(PWM::TIMER3, PWM::CH1, 500);
    double value = 0;

    while (1)
    {
        //led->Toggle();
        led.Set(fmod(value, 1.0));
        value += 0.1;
        osDelay(500);
    }

	while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}
}