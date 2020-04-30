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
#include <math.h>

void UART_TestCallback(void * param, uint8_t * buffer, uint32_t bufLen);
void CAN_Callback(void * param, const CANBus::package_t& package);
void SetPWM_Callback(void * param, const std::vector<uint8_t>& data);
void Timer_Callback(void * param);

typedef struct {
	uint32_t time;
	int32_t value;
} tx_data_t;

SyncedPWMADC::Sample samples_vin[10];
SyncedPWMADC::Sample samples_current[10];
tx_data_t tx_data[10];

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

	UART * uart = new UART(UART::PORT_UART2, 460800, 100, true);
	LSPC * lspc = new LSPC(uart, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY);
	//Debug::AssignDebugCOM((void*)uart);

	IO * led = new IO(GPIOC, GPIO_PIN_6);
	IO * debug = new IO(GPIOA, GPIO_PIN_15);
	Encoder * encoder = new Encoder(Encoder::TIMER4);

	CANBus * can = new CANBus();
	can->registerCallback(0x55, CAN_Callback, uart);

	Timer * timer = new Timer(Timer::TIMER6, 10000);
	timer->RegisterInterruptSoft(5, Timer_Callback, led);

	SyncedPWMADC * motor = new SyncedPWMADC(5000);
	motor->Debug_SetSamplingPin(debug);
	motor->SetSamplingInterval(10);
	motor->SetDutyCycle_MiddleSamplingOnce(0.4);

	lspc->registerCallback(0x01, SetPWM_Callback, motor);

	/*osDelay(2000);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0);
	osDelay(2000);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0.99);
	osDelay(2000);
	motor->SetOperatingMode(SyncedPWMADC::COAST);
	motor->SetTimerFrequencyAndDutyCycle_MiddleSampling(400, 0);*/

	//motor->SetTimerFrequencyWith50pctDutyCycle(400, true);
	//motor->ADC_ConfigureBackEMFSampling();
	//motor->Timer_Configure(20000, 1000);
	//motor->SetTimerFrequencyWith50pctDutyCycle(20000, true);
	//motor->SetTimerFrequencyWith50pctDutyCycle(400, false);

	/*
	int32_t encoderValue = encoder->Get();
	//Debug::printf("Encoder = %d\n", encoderValue);
	can->Transmit(0x01, (uint8_t *)&encoderValue, sizeof(encoderValue));
	*/

	tx_data_t * test = new tx_data_t[10];

	while (1)
	{
		for (uint8_t i = 0; i < 10; i++) {
			uint32_t timestamp = HAL_GetHighResTick();
			samples_current[i] = motor->GetCurrent();
			samples_vin[i] = motor->GetVin();

			osDelay(1);
		}

		for (uint8_t i = 0; i < 10; i++) {
			tx_data[i].time = samples_vin[i].Timestamp;
			tx_data[i].value = (int32_t)(roundf(samples_vin[i].ValueON * 1000));
		}
		lspc->TransmitAsync(0x01, (uint8_t *)&tx_data, 10*sizeof(tx_data_t));

		for (uint8_t i = 0; i < 10; i++) {
			float current = (samples_current[i].ValueON-2.0571f) / 0.1371f;
			tx_data[i].time = samples_current[i].Timestamp;
			tx_data[i].value = (int32_t)(roundf(current * 1000));
		}
		lspc->TransmitAsync(0x02, (uint8_t *)&tx_data, 10*sizeof(tx_data_t));

		for (uint8_t i = 0; i < 10; i++) {
			float current = (samples_current[i].ValueOFF-2.0571f) / 0.1371f;
			tx_data[i].time = samples_current[i].Timestamp;
			tx_data[i].value = (int32_t)(roundf(current * 1000));
		}
		lspc->TransmitAsync(0x03, (uint8_t *)&tx_data, 10*sizeof(tx_data_t));
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

void SetPWM_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 2) {
		int16_t duty = *((int16_t*)data.data());
		float dutyCycle = (float)duty / 1000;
		motor->SetDutyCycle_MiddleSamplingOnce(dutyCycle);
	}
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
