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

#include "MainTask.h"
#include "cmsis_os.h"
#include "Priorities.h"
#include "stm32g4xx_hal_timebase_tim.h" // for HAL_GetHighResTick()

/* Include Periphiral drivers */
#include "UART.h"
#include "IO.h"
#include "Encoder.h"

/* Include Drivers */

/* Include Module libraries */
#include "Debug.h"

/* Include Application-layer libraries */

/* Miscellaneous includes */
#include <stdlib.h>
#include <string.h>
#include <vector>

void UART_TestCallback(void * param, uint8_t * buffer, uint32_t bufLen);

void MainTask(void * pvParameters)
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

	UART * uart = new UART(UART::PORT_UART2, 115200, 100, true);
	Debug::AssignDebugCOM((void*)uart);

	IO * led = new IO(GPIOC, GPIO_PIN_6);
	Encoder * encoder = new Encoder(Encoder::TIMER4);

	while (1)
	{
		Debug::printf("Encoder = %d\n", encoder->Get());
		osDelay(100);
	}

#if 0
	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(100);
	while (1)
	{
		vTaskGetRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

		// Split into multiple packages and send
		uint16_t txIdx = 0;
		uint16_t remainingLength = strlen(pcWriteBuffer);
		uint16_t txLength;

		/*while (remainingLength > 0) {
			txLength = remainingLength;
			if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
				txLength = LSPC_MAXIMUM_PACKAGE_LENGTH-1;
				while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
				if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				else txLength++;
			}
			lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

			txIdx += txLength;
			remainingLength -= txLength;
		}*/
	}
#endif

	while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}
}

void UART_TestCallback(void * param, uint8_t * buffer, uint32_t bufLen)
{
	UART * uart = (UART*)param;
	uart->WriteBlocking(buffer, bufLen);
	uart->Write('\n');
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}
