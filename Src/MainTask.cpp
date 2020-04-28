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
#include "CANBus.h"
#include "Timer.h"
#include "SyncedPWMADC.h"

/* Include Drivers */

/* Include Module libraries */
#include "Debug.h"

/* Include Device libraries */
#include "LSPC.hpp"

/* Include Application-layer libraries */

/* Miscellaneous includes */
#include <stdlib.h>
#include <string.h>
#include <vector>

void UART_TestCallback(void * param, uint8_t * buffer, uint32_t bufLen);
void CAN_Callback(void * param, const CANBus::package_t& package);
void LSPC_Callback(void * param, const std::vector<uint8_t>& data);
void Timer_Callback(void * param);

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
	LSPC * lspc = new LSPC(uart, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY);
	//Debug::AssignDebugCOM((void*)uart);

	IO * led = new IO(GPIOC, GPIO_PIN_6);
	IO * debug = new IO(GPIOA, GPIO_PIN_15);
	Encoder * encoder = new Encoder(Encoder::TIMER4);

	lspc->registerCallback(0x02, LSPC_Callback, led);

	CANBus * can = new CANBus();
	can->registerCallback(0x55, CAN_Callback, uart);

	Timer * timer = new Timer(Timer::TIMER6, 10000);
	timer->RegisterInterruptSoft(5, Timer_Callback, led);

	SyncedPWMADC * motor = new SyncedPWMADC();
	motor->Debug_SetSamplingPin(debug);
	motor->SetSamplingInterval(10);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0.99);
	osDelay(2000);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0);
	osDelay(2000);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0.99);
	osDelay(2000);
	motor->SetOperatingMode(SyncedPWMADC::COAST);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0);

	//motor->SetTimerFrequencyWith50pctDutyCycle(400, true);
	//motor->ADC_ConfigureBackEMFSampling();
	//motor->Timer_Configure(20000, 1000);
	//motor->SetTimerFrequencyWith50pctDutyCycle(20000, true);
	//motor->SetTimerFrequencyWith50pctDutyCycle(400, false);

	while (1)
	{
		int32_t encoderValue = encoder->Get();
		//Debug::printf("Encoder = %d\n", encoderValue);
		can->Transmit(0x01, (uint8_t *)&encoderValue, sizeof(encoderValue));
		lspc->TransmitAsync(0x01, (uint8_t *)&encoderValue, sizeof(encoderValue));
		osDelay(1000);
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

void CAN_Callback(void * param, const CANBus::package_t& package)
{
	UART * uart = (UART*)param;
	Debug::print("Received CAN package with content: ");
	uart->WriteBlocking((uint8_t *)package.Data, package.DataLength);
	uart->Write('\n');
}

void LSPC_Callback(void * param, const std::vector<uint8_t>& data)
{
	IO * led = (IO*)param;
	int32_t encoderValue = *((int32_t*)data.data());
}

void Timer_Callback(void * param)
{
	IO * led = (IO*)param;
	led->Toggle();
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}
