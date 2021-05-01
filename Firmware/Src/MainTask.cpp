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

#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>

/* Include Periphiral drivers */
#include <UART/UART.hpp>
#include <IO/IO.hpp>
#include <Encoder/Encoder.hpp>
#include <CANBus/CANBus.hpp>
#include <Timer/Timer.hpp>
#include <SyncedPWMADC/SyncedPWMADC.hpp>

/* Include Drivers */

/* Include Module libraries */
#include <Debug/Debug.h>

/* Include Device libraries */
#include "LSPC.hpp"

/* Include Application-layer libraries */

/* Miscellaneous includes */
#include <Matrix/Matrix.hpp>
#include <MathLib/MathLib.h>
#include <LeastSquares/LeastSquares.hpp>
#include <NonlinearLeastSquares/NonlinearLeastSquares.hpp>
#include <FirstOrderLPF/FirstOrderLPF.hpp>
#include <CircularBuffer/CircularBuffer.hpp>

void CPU_Load(void * pvParameters);
void TaskRunTimeStats( char *pcWriteBuffer );
void UART_TestCallback(void * param, uint8_t * buffer, uint32_t bufLen);
void CAN_Callback(void * param, const CANBus::package_t& package);
void SetDutyCycle_Callback(void * param, const std::vector<uint8_t>& data);
void SetFreq_Callback(void * param, const std::vector<uint8_t>& data);
void SetAveraging_Callback(void * param, const std::vector<uint8_t>& data);
void Timer_Callback(void * param);
void SetOperatingMode_Callback(void * param, const std::vector<uint8_t>& data);
void SetActiveInactive_Callback(void * param, const std::vector<uint8_t>& data);
void StartFrequencySweep_Callback(void * param, const std::vector<uint8_t>& data);
void FrequencySweepThread(void * pvParameters);
void StartDutySweep_Callback(void * param, const std::vector<uint8_t>& data);
void DutySweepThread(void * pvParameters);
void StartSampleLocationSweep_Callback(void * param, const std::vector<uint8_t>& data);
void SampleLocationSweepThread(void * pvParameters);
void TestFitExponentialDecay(void);

typedef enum {
	NONE,
	CURRENT,
	VBUS,
	BEMF,
	// See "Motor Parameter Estimation steps" in "Estimator designs"
	Ke,
	R_Ke,
	R_L,
	L,
	R_L_Ke,
	B_tau_c,
	J
} calibrationMode;
calibrationMode currentCalibrationMode = NONE;
SemaphoreHandle_t captureCalibrationSamples;
float calibration_value_ref = 0;
typedef struct {
	float measured;
	float reference;
} calibrationSample_t;
calibrationSample_t * calibration_samples = 0;
uint32_t calibration_captured_samples = 0;
uint32_t calibration_max_samples = 0;
float calibration_BemfHighRangeValue = 0, calibration_BemfLowRangeValue = 0;
LineFitting * calibration_lsq = 0;
void Calibration_Callback(void * param, const std::vector<uint8_t>& data);

typedef struct {
	uint32_t time;
	int32_t value;
} tx_data_t;

typedef struct {
	SyncedPWMADC * motor;
	LSPC * lspc;
} ControllerParameters_t;

bool reset_tx_packing = true;
size_t freeHeapBytes = 0;

bool CurrentControllerEnabled = false;
float CurrentSetpoint = 0.0f;
float Controller_Kp = 4.0;
float Controller_Ti = 12.0;
void SetCurrentSetpoint_Callback(void * param, const std::vector<uint8_t>& data);
void SetCurrentKpTi_Callback(void * param, const std::vector<uint8_t>& data);
void StartCurrentController_Callback(ControllerParameters_t * motor);
void CurrentControllerThread(void * pvParameters);
void StartCurrentSweep_Callback(void * param, const std::vector<uint8_t>& data);
void CurrentSweepThread(void * pvParameters);

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
	UART * uart = new UART(UART::PORT_UART2, 1612800, 100, true);
	LSPC * lspc = new LSPC(uart, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY);
	Debug::AssignDebugCOM((void*)lspc);

	/*TaskHandle_t CPULoadTaskHandle;
	xTaskCreate(CPU_Load, (char *)"CPU Load", 256, (void*) lspc, CPULOAD_PRIORITY, &CPULoadTaskHandle);*/

	IO * led = new IO(GPIOC, GPIO_PIN_6);
	IO * debug = new IO(GPIOA, GPIO_PIN_15);
	Encoder * encoder = new Encoder(Encoder::TIMER4, true);

	CANBus * can = new CANBus();
	can->registerCallback(0x55, CAN_Callback);

	Timer * timer = new Timer(Timer::TIMER6, 10000);
	timer->RegisterInterruptSoft(5, Timer_Callback, led);

	uint16_t PWM_freq = 5000;
	uint16_t Sample_freq = 100;
	SyncedPWMADC * motor = new SyncedPWMADC(PWM_freq);
	motor->AssignEncoder(encoder);
	motor->Debug_SetSamplingPin(debug);
	motor->SetSamplingInterval(PWM_freq/Sample_freq);
	motor->DetermineCurrentSenseOffset();
	motor->SetNumberOfAveragingSamples(1);
	motor->SetDutyCycle_EndSampling(0);

	ControllerParameters_t ControllerParameters = { motor, lspc };

	lspc->registerCallback(0x01, SetDutyCycle_Callback, motor);
	lspc->registerCallback(0x02, SetFreq_Callback, motor);
	lspc->registerCallback(0x03, SetOperatingMode_Callback, motor);
	lspc->registerCallback(0x04, SetActiveInactive_Callback, motor);
	lspc->registerCallback(0x05, SetAveraging_Callback, motor);
	lspc->registerCallback(0x06, StartFrequencySweep_Callback, motor);
	lspc->registerCallback(0x07, StartDutySweep_Callback, motor);
	lspc->registerCallback(0x08, Calibration_Callback, motor);
	lspc->registerCallback(0x09, StartSampleLocationSweep_Callback, motor);
	lspc->registerCallback(0x0A, SetCurrentSetpoint_Callback, &ControllerParameters);
	lspc->registerCallback(0x0B, SetCurrentKpTi_Callback);
	lspc->registerCallback(0x0C, StartCurrentSweep_Callback, &ControllerParameters);
	captureCalibrationSamples = xSemaphoreCreateBinary();

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

	SyncedPWMADC::CombinedSample_t sample;
	uint8_t * TX_Buffer = (uint8_t*)pvPortMalloc(LSPC_MAXIMUM_PACKAGE_LENGTH);
	uint16_t TX_Buffer_WritePos = 0;

	while (1)
	{
		if (currentCalibrationMode != NONE && calibration_samples) {
			if (xSemaphoreTake( captureCalibrationSamples, ( TickType_t ) 0 )) {
				xSemaphoreGive(captureCalibrationSamples);
				if (currentCalibrationMode == BEMF) {
					while (true) {
						calibration_BemfHighRangeValue = 0;
						calibration_BemfLowRangeValue = 0;

						for (uint8_t i = 0; i < 2; i++) {
							if ( xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
								if (sample.BemfHighRange)
									calibration_BemfHighRangeValue = sample.Bemf;
								else
									calibration_BemfLowRangeValue = sample.Bemf;
							}
						}

						if (calibration_BemfHighRangeValue != 0 && calibration_BemfLowRangeValue != 0) {
							if (calibration_BemfLowRangeValue < 3.20f) { // < BEMF_SWITCH_TO_HIGH_RANGE_HYSTERESIS_THRESHOLD
								if (calibration_captured_samples < calibration_max_samples) {
									calibration_samples[calibration_captured_samples].measured = calibration_BemfHighRangeValue;
									calibration_samples[calibration_captured_samples].reference = calibration_BemfLowRangeValue;
									calibration_captured_samples++;
								}

								if (calibration_lsq && calibration_BemfHighRangeValue > 0.3) // we are mainly interested in fitting the upper range, since this is where we will be using the conversion
									calibration_lsq->AddMeasurement(calibration_BemfHighRangeValue, calibration_BemfLowRangeValue); // estimate linear mapping (a*x+b) between low and high range measurements
								else
									break; // stop sampling
							}

							for (uint8_t samplesToSkip = 20; samplesToSkip > 0; samplesToSkip--) {
								xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY );
							}
						}
					}
					xQueueReset(motor->SampleQueue);
				}
				else {
					for (uint8_t samplesToCapture = 10; (samplesToCapture > 0 && calibration_captured_samples < calibration_max_samples); samplesToCapture--) {
						if ( xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
							calibration_samples[calibration_captured_samples].reference = calibration_value_ref;
							if (currentCalibrationMode == CURRENT)
								calibration_samples[calibration_captured_samples].measured = sample.Current.ON;
							else if (currentCalibrationMode == VBUS)
								calibration_samples[calibration_captured_samples].measured = sample.VbusOFF;

							if (calibration_lsq)
								calibration_lsq->AddMeasurement(calibration_samples[calibration_captured_samples].reference, calibration_samples[calibration_captured_samples].measured);

							calibration_captured_samples++;
						}
					}
				}
				xSemaphoreTake( captureCalibrationSamples, ( TickType_t ) 0 );
			}
		}

#if 0 // 1 = send debug data for logging
#if 0 // 1 = do not pack data
		if ( xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
			while (1) {
				if (lspc->TransmitAsync(0x04, (uint8_t*)&sample, sizeof(SyncedPWMADC::CombinedSample_t)-1)) {
					break;
				}
				osDelay(1);
			}
		}
#else
		// Pack data before sending
		while (true) {
			if ((TX_Buffer_WritePos + sizeof(SyncedPWMADC::CombinedSample_t)-1) < LSPC_MAXIMUM_PACKAGE_LENGTH) {
				if ( xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
					memcpy(&TX_Buffer[TX_Buffer_WritePos], (uint8_t*)&sample, sizeof(SyncedPWMADC::CombinedSample_t)-1);
					TX_Buffer_WritePos += sizeof(SyncedPWMADC::CombinedSample_t)-1; // minus 1 because of the extra internal variable boolean
				} else {
					break;
				}
			} else {
				break;
			}
		}
		if (TX_Buffer_WritePos > 0) {
			while (1) {
				if (lspc->TransmitAsync(0x04, TX_Buffer, TX_Buffer_WritePos)) {
					TX_Buffer_WritePos = 0;
					break;
				} else {
					osDelay(1);
				}
			}
		}
#endif
#else
		freeHeapBytes = xPortGetFreeHeapSize();
		osDelay(200);
#endif
	}

	while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}
}

void CPU_Load(void * pvParameters)
{
	LSPC * lspc = (LSPC*)pvParameters;

	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(500); // Approx 40 bytes pr. task
	while (1)
	{
		TaskRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

		// Split into multiple packages and send
		uint16_t txIdx = 0;
		uint16_t remainingLength = strlen(pcWriteBuffer);
		uint16_t txLength;

		while (remainingLength > 0) {
			txLength = remainingLength;
			if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
				txLength = LSPC_MAXIMUM_PACKAGE_LENGTH-1;
				while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
				if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				else txLength++;
			}
			lspc->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

			txIdx += txLength;
			remainingLength -= txLength;
		}

		osDelay(1000);
	}
}


static char *WriteTaskNameToBuffer( char *pcBuffer, const char *pcTaskName )
{
	size_t x;

	/* Start by copying the entire string. */
	strcpy( pcBuffer, pcTaskName );

	/* Pad the end of the string with spaces to ensure columns line up when
	printed out. */
	for( x = strlen( pcBuffer ); x < ( size_t ) ( configMAX_TASK_NAME_LEN - 1 ); x++ )
	{
		pcBuffer[ x ] = ' ';
	}

	/* Terminate. */
	pcBuffer[ x ] = 0x00;

	/* Return the new end of string. */
	return &( pcBuffer[ x ] );
}

void TaskRunTimeStats( char *pcWriteBuffer )
{
	// Based on vTaskGetRunTimeStats
	static TaskStatus_t *pxTaskStatusArrayPrev = 0;
	static UBaseType_t uxArraySizePrev = 0;
	static uint32_t ulTotalTimePrev = 0;

	uint32_t ulTotalTime, ulStatsAsPercentage;
	TaskStatus_t *pxTaskStatusArray;
	UBaseType_t uxArraySize;

	*pcWriteBuffer = 0x00; // ensure that we start clean

	uxArraySize = uxTaskGetNumberOfTasks();
	pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );

		ulTotalTime /= 100UL; // For percentage calculations.

		if( ulTotalTime > 0 && pxTaskStatusArrayPrev )
		{
			UBaseType_t x;
			for( x = 0; x < uxArraySize; x++ )
			{
				ulStatsAsPercentage = (pxTaskStatusArray[ x ].ulRunTimeCounter - pxTaskStatusArrayPrev[ x ].ulRunTimeCounter) /
									  (ulTotalTime - ulTotalTimePrev);

				if (ulStatsAsPercentage > 100)
					ulStatsAsPercentage = 0;

				pcWriteBuffer = WriteTaskNameToBuffer( pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName );

				if( ulStatsAsPercentage > 0UL )
				{
						sprintf( pcWriteBuffer, "\t%u\t\t%u%%\r\n", ( unsigned int ) pxTaskStatusArray[ x ].ulRunTimeCounter, ( unsigned int ) ulStatsAsPercentage );
				}
				else
				{
						sprintf( pcWriteBuffer, "\t%u\t\t<1%%\r\n", ( unsigned int ) pxTaskStatusArray[ x ].ulRunTimeCounter );
				}

				pcWriteBuffer += strlen( pcWriteBuffer );
			}
		}

		if (pxTaskStatusArrayPrev)
			vPortFree( pxTaskStatusArrayPrev );
		pxTaskStatusArrayPrev = pxTaskStatusArray;
		uxArraySizePrev = uxArraySize;
		ulTotalTimePrev = ulTotalTime;
	}
}

void CAN_Callback(void * param, const CANBus::package_t& package)
{
	Debug::printf("Received CAN package with ID: %d", package.ID);
}

void SetDutyCycle_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 2) {
		int16_t duty = *((int16_t*)data.data());
		float dutyCycle = (float)duty / 1000;
		CurrentControllerEnabled = false;

		motor->SetDutyCycle_MiddleSampling(dutyCycle);

		reset_tx_packing = true;
	}
}

void SetFreq_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 4) {
		uint16_t* values = (uint16_t*)data.data();
		uint16_t PWM_freq = values[0];
		uint16_t Sample_freq = values[1];

		uint16_t interval = (uint16_t)roundf((float)PWM_freq/Sample_freq);
		uint16_t resulting_sample_freq = PWM_freq / interval;
		if (resulting_sample_freq > 2000) {
			resulting_sample_freq = 2000; // limited by UART baud rate
			interval = (uint16_t)floorf((float)PWM_freq/Sample_freq);
		}

		CurrentControllerEnabled = false;
		motor->SetPWMFrequency(PWM_freq);
		motor->SetSamplingInterval(interval);
		motor->SetDutyCycle_MiddleSampling(motor->GetCurrentDutyCycle());

		reset_tx_packing = true;
	}
}

void SetAveraging_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 2) {
		uint16_t* values = (uint16_t*)data.data();
		uint16_t AveragingCount = values[0];

		CurrentControllerEnabled = false;
		motor->SetNumberOfAveragingSamples(AveragingCount);
	}
}

void SetOperatingMode_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (!motor) return;
	CurrentControllerEnabled = false;

	if (data.size() == 1) {
		if (data[0] == 0x00) //
			motor->SetOperatingMode(SyncedPWMADC::BRAKE);
		else if (data[0] == 0x01)
			motor->SetOperatingMode(SyncedPWMADC::COAST);
	}
}

void SetActiveInactive_Callback(void * param, const std::vector<uint8_t>& data)
{
	static bool active_state = true;
	static float active_duty_cycle;
	static SyncedPWMADC::OperatingMode_t active_mode;

	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (!motor) return;
	CurrentControllerEnabled = false;

	if (data.size() == 1) {
		if (data[0] == 0x00 && active_state) {
			active_mode = motor->GetOperatingMode();
			active_duty_cycle = motor->GetCurrentDutyCycle();

			//motor->SetBemfRange(false, true);
			motor->SetBemfRange(true, false);
			motor->SetOperatingMode(SyncedPWMADC::COAST);
			motor->SetDutyCycle_MiddleSamplingOnce(0.0f);
			active_state = false;
		}
		else if (data[0] == 0x01 && !active_state) {
			motor->SetOperatingMode(active_mode);
			motor->SetDutyCycle_MiddleSamplingOnce(active_duty_cycle);
			active_state = true;
		}
	}
}


void StartFrequencySweep_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 0) {
		TaskHandle_t FrequencySweepTaskHandle;
	    // Obtain the handle of a task from its name.
		FrequencySweepTaskHandle = xTaskGetHandle( "Frequency sweep" );

		if (FrequencySweepTaskHandle != 0) {
			TaskStatus_t xTaskDetails;

			// Use the handle to obtain further information about the task.
			vTaskGetInfo( FrequencySweepTaskHandle,
						  &xTaskDetails,
						  pdTRUE, // Include the high water mark in xTaskDetails.
						  eInvalid ); // Include the task state in xTaskDetails.

			if (xTaskDetails.eCurrentState != eDeleted && xTaskDetails.eCurrentState != eInvalid) {
				return; // Frequency sweep task is already running
			}
		}
		CurrentControllerEnabled = false;
		xTaskCreate(FrequencySweepThread, (char *)"Frequency sweep", 256, (void*) motor, 6, &FrequencySweepTaskHandle);
	}
}


void FrequencySweepThread(void * pvParameters)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)pvParameters;
	motor->SetOperatingMode(SyncedPWMADC::BRAKE);
	motor->SetSamplingInterval(2);
	motor->SetNumberOfAveragingSamples(1);

	uint32_t freq = 250;
	motor->SetPWMFrequency(freq); // configure prescaler using starting frequency
	motor->WaitForNewQueuedSample();
	freq = 250;
	bool switch_sampling = false;
	while (freq <= 5000)
	{
		if (freq >= 2500 && !switch_sampling)  {
			motor->SetSamplingInterval(4);
			motor->SetNumberOfAveragingSamples(1);
			switch_sampling = true;
		}
		motor->SetPWMFrequency(freq, false);
		//motor->SetDutyCycle_EndSampling(0.5);
		float duty_step = 0.4;
		for (float duty = 0.1; duty < 0.91; duty += duty_step) {
			motor->SetDutyCycle_CustomSampling(duty, duty-0.025, 0.975); // these sample locations also fulfills the "outside of ripple"-zone up to 5 KHz for 50% duty (for 30% duty the first will be 0.275)
			for (uint8_t samples = 0; samples < 50; samples++)
				motor->WaitForNewQueuedSample();
		}
		for (float duty = 0.9 - duty_step; duty > 0.09; duty -= duty_step) {
			motor->SetDutyCycle_CustomSampling(duty, duty-0.025, 0.975); // these sample locations also fulfills the "outside of ripple"-zone up to 5 KHz for 50% duty (for 30% duty the first will be 0.275)
			for (uint8_t samples = 0; samples < 50; samples++)
				motor->WaitForNewQueuedSample();
		}
		freq += 100;
	}

	motor->SetDutyCycle_EndSampling(0);
	motor->SetPWMFrequency(250);

	vTaskDelete(NULL); // delete/stop this current task
}

void FrequencySweepThread2(void * pvParameters)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)pvParameters;
	motor->SetOperatingMode(SyncedPWMADC::BRAKE);
	motor->SetSamplingInterval(2);
	motor->SetNumberOfAveragingSamples(1);

	uint32_t freq = 250;
	bool switch_sampling = false;
	motor->SetPWMFrequency(freq); // configure prescaler using starting frequency
	motor->WaitForNewQueuedSample();
	for (float duty = 0.1; duty < 0.91; duty += 0.1) {
		freq = 250;
		switch_sampling = false;
		motor->SetSamplingInterval(2);
		motor->SetNumberOfAveragingSamples(1);
		while (freq <= 5000) {
			if (freq >= 2500 && !switch_sampling)  {
				motor->SetSamplingInterval(4);
				motor->SetNumberOfAveragingSamples(1);
				switch_sampling = true;
			}

			motor->SetPWMFrequency(freq, false);
			//motor->SetDutyCycle_EndSampling(0.5);
			motor->SetDutyCycle_CustomSampling(duty, duty-0.025, 0.975); // these sample locations also fulfills the "outside of ripple"-zone up to 5 KHz for 50% duty (for 30% duty the first will be 0.275)

			for (uint8_t samples = 0; samples < 10; samples++)
				motor->WaitForNewQueuedSample();

			freq += 50;
		}
	}

	motor->SetDutyCycle_EndSampling(0);
	motor->SetPWMFrequency(250);

	vTaskDelete(NULL); // delete/stop this current task
}


void StartDutySweep_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 0) {
		TaskHandle_t DutySweepTaskHandle;
	    // Obtain the handle of a task from its name.
		DutySweepTaskHandle = xTaskGetHandle( "Duty sweep" );

		if (DutySweepTaskHandle != 0) {
			TaskStatus_t xTaskDetails;

			// Use the handle to obtain further information about the task.
			vTaskGetInfo( DutySweepTaskHandle,
						  &xTaskDetails,
						  pdTRUE, // Include the high water mark in xTaskDetails.
						  eInvalid ); // Include the task state in xTaskDetails.

			if (xTaskDetails.eCurrentState != eDeleted && xTaskDetails.eCurrentState != eInvalid) {
				return; // Frequency sweep task is already running
			}
		}
		CurrentControllerEnabled = false;
		xTaskCreate(DutySweepThread, (char *)"Duty sweep", 256, (void*) motor, CALIBRATION_SWEEP_PRIORITY, &DutySweepTaskHandle);
	}
}


void DutySweepThread(void * pvParameters)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)pvParameters;
	motor->SetOperatingMode(SyncedPWMADC::BRAKE);

	bool OperatingFrequency = true;
	uint32_t freq;
	if (OperatingFrequency) {
		freq = 10000;
		motor->SetSamplingInterval(16);
		motor->SetNumberOfAveragingSamples(1);
	} else {
		freq = 500;
		motor->SetSamplingInterval(2);
		motor->SetNumberOfAveragingSamples(1);
	}

	motor->SetPWMFrequency(freq); // configure prescaler using starting frequency
	if (OperatingFrequency) motor->SetDutyCycle_MiddleSamplingOnce(0);
	else motor->SetDutyCycle_EndSampling(0);
	motor->WaitForNewQueuedSample();
	for (float duty = 0.1; duty < 1.0; duty += 0.05) {
		if (OperatingFrequency) motor->SetDutyCycle_MiddleSampling(duty);
		else motor->SetDutyCycle_EndSampling(duty);
		osDelay(500);
	}
	for (float duty = 0.9; duty > -1.0; duty -= 0.05) {
		if (OperatingFrequency) motor->SetDutyCycle_MiddleSampling(duty);
		else motor->SetDutyCycle_EndSampling(duty);
		osDelay(500);
	}
	for (float duty = -0.9; duty < 1.0; duty += 0.05) {
		if (OperatingFrequency) motor->SetDutyCycle_MiddleSampling(duty);
		else motor->SetDutyCycle_EndSampling(duty);
		osDelay(500);
	}
	for (float duty = 0.9; duty >= 0.1; duty -= 0.05) {
		if (OperatingFrequency) motor->SetDutyCycle_MiddleSampling(duty);
		else motor->SetDutyCycle_EndSampling(duty);
		osDelay(500);
	}

	motor->SetDutyCycle_EndSampling(0);
	motor->SetPWMFrequency(250);

	vTaskDelete(NULL); // delete/stop this current task
}

void StartSampleLocationSweep_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 0) {
		TaskHandle_t SampleLocationSweepTaskHandle;
	    // Obtain the handle of a task from its name.
		SampleLocationSweepTaskHandle = xTaskGetHandle( "Sample location sweep" );

		if (SampleLocationSweepTaskHandle != 0) {
			TaskStatus_t xTaskDetails;

			// Use the handle to obtain further information about the task.
			vTaskGetInfo( SampleLocationSweepTaskHandle,
						  &xTaskDetails,
						  pdTRUE, // Include the high water mark in xTaskDetails.
						  eInvalid ); // Include the task state in xTaskDetails.

			if (xTaskDetails.eCurrentState != eDeleted && xTaskDetails.eCurrentState != eInvalid) {
				return; // Frequency sweep task is already running
			}
		}
		CurrentControllerEnabled = false;
		xTaskCreate(SampleLocationSweepThread, (char *)"Sample location sweep", 256, (void*) motor, CALIBRATION_SWEEP_PRIORITY, &SampleLocationSweepTaskHandle);
	}
}

void SampleLocationSweepThread(void * pvParameters)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)pvParameters;
	motor->SetOperatingMode(SyncedPWMADC::BRAKE);
	//motor->SetPWMFrequency(200);
	//motor->SetSamplingInterval(3);
	//motor->SetNumberOfAveragingSamples(2);
	motor->SetDutyCycle_EndSampling(0.5);

	for (uint32_t freq = 200; freq <= 2000; freq += 200) {
		motor->SetPWMFrequency(freq);
		motor->SetSamplingInterval(3);
		motor->SetNumberOfAveragingSamples(2);
		motor->SetCustomSamplingLocations(0.02);
		for (float sampleLocation = 0.04; sampleLocation <= 0.98; sampleLocation += 0.02) {
			motor->WaitForNewQueuedSample();
			motor->SetCustomSamplingLocations(sampleLocation);
		}
	}

	motor->SetDutyCycle_EndSampling(0);

	vTaskDelete(NULL); // delete/stop this current task
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


void TestFitExponentialDecay(void)
{
	float coeffs[3];
	float x[10];
	float y[10];

	float a_ = -12.2f;
	float b_ = 30.f;
	float c_ = -0.2f;
	for (uint8_t i = 0; i < 10; i++) {
		x[i] = 2 + i;
		y[i] = a_ + b_ * expf(c_ * x[i]);
	}

	Matrix x2(10, 1, x);
	Matrix y2(10, 1, y);
	uint32_t t0 = HAL_tic();
	NonlinearLeastSquares::fitExponentialDecay(x2, y2, coeffs);
	float dt = HAL_toc(t0);
}

void Calibration_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)param;

	if (data.size() == 1) {
		if (currentCalibrationMode == NONE) {
			if (data[0] == 0x01) {
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
				// Allocate sample memory - 100 samples depth = 10 different values since we store 10 samples from each value reference
				calibration_samples = (calibrationSample_t*)calloc(100, sizeof(calibrationSample_t));
				if (!calibration_samples) return;
				calibration_max_samples = 100;
				calibration_captured_samples = 0;

				if (calibration_lsq) {
					delete calibration_lsq;
					calibration_lsq = 0;
				}
				calibration_lsq = new LineFitter<LeastSquares1D>; // fit only slope

				currentCalibrationMode = CURRENT;
				//motor->ChannelCalibrations.CurrentSense.Enabled = false;
				motor->ChannelCalibrations.CurrentSense.Enabled = true;
				if (motor->GetCurrentDirection())
					motor->ChannelCalibrations.CurrentSense.VSENSE1.Scale = 1.0f;
				else
					motor->ChannelCalibrations.CurrentSense.VSENSE3.Scale = 1.0f;

				motor->SetOperatingMode(SyncedPWMADC::BRAKE);
				motor->SetPWMFrequency(500);
				motor->SetSamplingInterval(100); // 5 Hz sampling
				motor->SetNumberOfAveragingSamples(50); // 100 ms averaging interval
				motor->SetDutyCycle_EndSampling(motor->GetCurrentDutyCycle());
			}

			else if (data[0] == 0x02) {
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
				// Allocate sample memory - 100 samples depth = 10 different values since we store 10 samples from each value reference
				calibration_samples = (calibrationSample_t*)calloc(100, sizeof(calibrationSample_t));
				if (!calibration_samples) return;
				calibration_max_samples = 100;
				calibration_captured_samples = 0;

				if (calibration_lsq) {
					delete calibration_lsq;
				}
				calibration_lsq = new LineFitter<LeastSquares2D>; // fit slope and offset

				currentCalibrationMode = VBUS;
				motor->ChannelCalibrations.Vbus.Enabled = false;

				motor->SetOperatingMode(SyncedPWMADC::BRAKE);
				motor->SetPWMFrequency(500);
				motor->SetSamplingInterval(100); // 5 Hz sampling
				motor->SetNumberOfAveragingSamples(50); // 100 ms averaging interval
				motor->SetDutyCycle_EndSampling(0);
			}

			else if (data[0] == 0x03) {
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
				// Allocate sample memory - 100 samples depth = 10 different values since we store 10 samples from each value reference
				calibration_samples = (calibrationSample_t*)calloc(100, sizeof(calibrationSample_t));
				if (!calibration_samples) return;
				calibration_max_samples = 100;
				calibration_captured_samples = 0;

				if (calibration_lsq) {
					delete calibration_lsq;
					calibration_lsq = 0;
				}
				calibration_lsq = new LineFitter<LeastSquares2D>; // fit slope and offset

				calibration_BemfHighRangeValue = 0;
				calibration_BemfLowRangeValue = 0;
				currentCalibrationMode = BEMF;
				motor->ChannelCalibrations.Bemf.Enabled = false;

				motor->SetOperatingMode(SyncedPWMADC::COAST);
				motor->SetBemfRange(false, false, true);
				motor->SetPWMFrequency(20000);
				motor->SetSamplingInterval(2); // 10000 Hz sampling  (BEMF sampling needs to be fast since we are alternating the Voltage divider = high-range/low-range)
				motor->SetNumberOfAveragingSamples(1); // no averaging
				motor->SetDutyCycle_MiddleSamplingOnce(0.0f);

				// Capture samples
				xSemaphoreGive(captureCalibrationSamples);
			}
		}

		else if (data[0] == 0x00) {
			if (currentCalibrationMode == CURRENT) {
				while (uxSemaphoreGetCount(captureCalibrationSamples)) osDelay(1);

				if (calibration_captured_samples > 0) {
					LineFitting::lineParameters_t calibration_params_est;
					if (calibration_lsq) {
						calibration_params_est = calibration_lsq->GetEstimate();
						if (calibration_params_est.valid) {
							// current_vsense = a*current + b
							// current = 1/a * (vbus_sense - b)     [this is the way the calibrated scale and offset correction is applied in SyncedPWMADC]
							if (motor->GetCurrentDirection()) {
								motor->ChannelCalibrations.CurrentSense.VSENSE1.Scale = 1 / calibration_params_est.slope; // 1/a
								//motor->ChannelCalibrations.CurrentSense.VSENSE1.Offset = -calibration_params_est.offset; // -b
							} else {
								motor->ChannelCalibrations.CurrentSense.VSENSE3.Scale = 1 / calibration_params_est.slope; // 1/a
								//motor->ChannelCalibrations.CurrentSense.VSENSE3.Offset = -calibration_params_est.offset; // -b
							}
						}
					}
				}
				if (calibration_lsq) {
					delete calibration_lsq;
					calibration_lsq = 0;
				}

				currentCalibrationMode = NONE;
				motor->ChannelCalibrations.CurrentSense.Enabled = true;

				// Free the allocated sample memory
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
			}
			else if (currentCalibrationMode == VBUS) {
				while (uxSemaphoreGetCount(captureCalibrationSamples)) osDelay(1);
				currentCalibrationMode = NONE;

				if (calibration_captured_samples > 0) {
					LineFitting::lineParameters_t calibration_params_est;
					if (calibration_lsq) {
						calibration_params_est = calibration_lsq->GetEstimate();
						if (calibration_params_est.valid) {
							// vbus_sense = a*vbus + b
							// vbus = 1/a * (vbus_sense - b)     [this is the way the calibrated scale and offset correction is applied in SyncedPWMADC]
							motor->ChannelCalibrations.Vbus.VBUS.Scale = 1 / calibration_params_est.slope; // 1/a
							motor->ChannelCalibrations.Vbus.VBUS.Offset = -calibration_params_est.offset; // -b
						}
					}
				}
				if (calibration_lsq) {
					delete calibration_lsq;
					calibration_lsq = 0;
				}

				motor->ChannelCalibrations.Vbus.Enabled = true;

				// Free the allocated sample memory
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
			}
			else if (currentCalibrationMode == BEMF) {
				while (uxSemaphoreGetCount(captureCalibrationSamples)) osDelay(1);
				currentCalibrationMode = NONE;

				motor->SetOperatingMode(SyncedPWMADC::BRAKE);
				motor->SetDutyCycle_EndSampling(0.0f);

				if (calibration_captured_samples > 0) {
					LineFitting::lineParameters_t calibration_params_est;
					if (calibration_lsq) {
						calibration_params_est = calibration_lsq->GetEstimate();
						if (calibration_params_est.valid) {
							// bemf = a_low * bemf_low + b_low
							// bemf_low = a_high * bemf_high + b_high
							//
							// bemf = a_low * (a_high * bemf_high + b_high) + b_low
							// bemf = a_low*a_high * bemf_high + a_low*b_high + b_low
							//
							// bemf = a_low*a_high * (bemf_high + (a_low*b_high + b_low)/(a_low*a_high))       [this is the way the calibrated scale and offset correction is applied in SyncedPWMADC]
							if (motor->GetCurrentDirection()) {
								motor->ChannelCalibrations.Bemf.BEMF3.LowRange.Scale = 1; // calibration of low-range Bemf-sense requires precise external voltage input
								motor->ChannelCalibrations.Bemf.BEMF3.LowRange.Offset = 0;
								motor->ChannelCalibrations.Bemf.BEMF3.HighRange.Scale = motor->ChannelCalibrations.Bemf.BEMF3.LowRange.Scale * calibration_params_est.slope; // a_low*a_high
								motor->ChannelCalibrations.Bemf.BEMF3.HighRange.Offset = (motor->ChannelCalibrations.Bemf.BEMF3.LowRange.Scale * calibration_params_est.offset + motor->ChannelCalibrations.Bemf.BEMF3.LowRange.Offset) / motor->ChannelCalibrations.Bemf.BEMF3.HighRange.Scale; // (a_low*b_high + b_low)/(a_low*a_high)
							} else {
								motor->ChannelCalibrations.Bemf.BEMF1.LowRange.Scale = 1; // calibration of low-range Bemf-sense requires precise external voltage input
								motor->ChannelCalibrations.Bemf.BEMF1.LowRange.Offset = 0;
								motor->ChannelCalibrations.Bemf.BEMF1.HighRange.Scale = motor->ChannelCalibrations.Bemf.BEMF1.LowRange.Scale * calibration_params_est.slope; // a_low*a_high
								motor->ChannelCalibrations.Bemf.BEMF1.HighRange.Offset = (motor->ChannelCalibrations.Bemf.BEMF1.LowRange.Scale * calibration_params_est.offset + motor->ChannelCalibrations.Bemf.BEMF1.LowRange.Offset) / motor->ChannelCalibrations.Bemf.BEMF1.HighRange.Scale; // (a_low*b_high + b_low)/(a_low*a_high)
							}
						}
					}
				}
				if (calibration_lsq) {
					delete calibration_lsq;
					calibration_lsq = 0;
				}

				motor->SetBemfRange(true, false, false);
				motor->ChannelCalibrations.Bemf.Enabled = true;

				// Free the allocated sample memory
				if (calibration_samples) {
					free(calibration_samples);
					calibration_samples = 0;
					calibration_captured_samples = 0;
					calibration_max_samples = 0;
				}
			}
		}
	} else if (data.size() == 3 && currentCalibrationMode != NONE) {
		int16_t valueRaw = *((int16_t*)&data[1]);
		float value = (float)valueRaw / 1000;

		if (!uxSemaphoreGetCount(captureCalibrationSamples) && calibration_captured_samples < calibration_max_samples) {
			// Capture n-samples
			calibration_value_ref = value;
			xSemaphoreGive(captureCalibrationSamples);
		}
	}
}

void SetCurrentSetpoint_Callback(void * param, const std::vector<uint8_t>& data)
{
	ControllerParameters_t * params = (ControllerParameters_t*)param;
	if (data.size() == 2) {
		int16_t currentSetpointRaw = *((int16_t*)data.data());
		CurrentSetpoint = (float)currentSetpointRaw / 1000;

		StartCurrentController_Callback(params);
	}
}

void SetCurrentKpTi_Callback(void * param, const std::vector<uint8_t>& data)
{
	if (data.size() == 4) {
		int16_t * currentSetpointRaw = ((int16_t*)data.data());
		Controller_Kp = (float)currentSetpointRaw[0] / 100;
		Controller_Ti = (float)currentSetpointRaw[1] / 100;
	}
}

void StartCurrentController_Callback(ControllerParameters_t * params)
{
	TaskHandle_t CurrentControllerTaskHandle;

	// Obtain the handle of a task from its name.
	CurrentControllerTaskHandle = xTaskGetHandle( "Current controller" );

	if (CurrentControllerTaskHandle != 0) {
		TaskStatus_t xTaskDetails;

		// Use the handle to obtain further information about the task.
		vTaskGetInfo( CurrentControllerTaskHandle,
					  &xTaskDetails,
					  pdTRUE, // Include the high water mark in xTaskDetails.
					  eInvalid ); // Include the task state in xTaskDetails.

		if (xTaskDetails.eCurrentState != eDeleted && xTaskDetails.eCurrentState != eInvalid) {
			return; // Current controller task is already running
		}
	}
	xTaskCreate(CurrentControllerThread, (char *)"Current controller", 400, (void*) params, CURRENT_CONTROLLER_PRIORITY, &CurrentControllerTaskHandle);
}

void CurrentControllerThread(void * pvParameters)
{
	ControllerParameters_t * params = (ControllerParameters_t*)pvParameters;
	SyncedPWMADC * motor = params->motor;
	LSPC * lspc = params->lspc;

	{
		FirstOrderLPF LPF_Omega(1.f/2000, 20);
		FirstOrderLPF LPF_Current(1.f/2000, 500);

		CircularBuffer<float> Encoder_Times(40, true);
		CircularBuffer<int32_t> Encoder_Ticks(40, true);

		motor->SetOperatingMode(SyncedPWMADC::BRAKE);
		motor->SetPWMFrequency(20000);
		motor->SetSamplingInterval(10);
		motor->SetNumberOfAveragingSamples(8);
		motor->SetDutyCycle_MiddleSamplingOnce(0.0f);

		motor->WaitForNewSample();
		SyncedPWMADC::CombinedSample_t sample = motor->GetLatestSample();
		ControllerDebug_t controllerDebug;

		uint32_t prev_encoder = sample.Encoder;
		float prev_time = sample.Timestamp;

		float integral = 0.f;
		float prev_error = 0.f;

		bool TransmitDebugDownsample = true;

		SyncedPWMADC::OperatingMode_t PreviousMode = SyncedPWMADC::BRAKE;
		CurrentControllerEnabled = true;
		while (CurrentControllerEnabled)
		{
			//motor->WaitForNewQueuedSample();
			motor->WaitForNewSample();
			sample = motor->GetLatestSample();

			// Average current
			// Vmot = R*i + Ke*omega
			// duty*Vin = R*i + Ke*omega
			// duty = 1/Vin * (R*i + Ke*omega)
			// including current error term
			// duty = 1/Vin * (R*i_setpoint + Ke*omega) - R/Vin*(i_meas-i_setpoint)
			// duty = 1/Vin * (R*i_setpoint + Ke*omega) - Kp*i_err
			//    Kp = R/Vin
			//    i_err = i_meas - i_setpoint
			/*if (motor->Samples.Encoder.Updated &&
				motor->Samples.Vbus.Updated &&
				motor->Samples.CurrentSense)*/
			if ( CurrentControllerEnabled && ((PreviousMode == SyncedPWMADC::COAST) || ((sample.Current.ON != 0 || sample.Current.OFF != 0) && (sample.VbusON != 0 || sample.VbusOFF != 0))) )
			{
				float current = 0;
				float vin = 0;
				if (fabsf(sample.Current.ON) > fabsf(sample.Current.OFF)) {
					current = sample.Current.ON;
					vin = sample.VbusON;
				} else {
					current = sample.Current.OFF;
					vin = sample.VbusOFF;
				}

				float current_filtered = LPF_Current.Filter(current);

				int32_t encoder = sample.Encoder;
				float time = sample.Timestamp / 100000.f;

				Encoder_Times.Push(time);
				Encoder_Ticks.Push(encoder);

				int32_t delta_encoder = encoder - prev_encoder; // OBS. We might need to consider wrapping if the motor will be running for a long time?
				float delta_time = time - prev_time;
				prev_encoder = encoder;
				prev_time = time;

				float omega = M_2PI * ((float)delta_encoder / motor->MotorParameters.TicksAfterGearing) / delta_time;

				delta_time = Encoder_Times.Front() - Encoder_Times.Back();
				delta_encoder = Encoder_Ticks.Front() - Encoder_Ticks.Back(); // OBS. We might need to consider wrapping if the motor will be running for a long time?
				float omega_filtered = M_2PI * ((float)delta_encoder / motor->MotorParameters.TicksAfterGearing) / delta_time;
				omega_filtered = LPF_Omega.Filter(omega_filtered);

				/* PI Controller */
				float Kp = Controller_Kp;
				float Ki = Controller_Kp * Controller_Ti;

				float error = CurrentSetpoint - current;
				integral += Ki * delta_time/2.f * (error + prev_error);
				if (integral > vin) {
					integral = vin;
				} else if (integral < -vin) {
					integral = -vin;
				}

				float PI_out = Kp*error + integral;
				prev_error = error;

				// Add feedforward term
				float u = PI_out + motor->MotorParameters.Ke * omega_filtered; // + motor->MotorParameters.R * CurrentSetpoint; // reference feedforward currently disabled (handled by integral term)
				// With the reference feedforward the gains has to be heavily reduced but the bandwidth seem to increase as the lag is reduced
				// Use the following gains with reference feedforward enabled: Kp=1, Ti=2

				// Convert to duty cycle
				float duty = u / vin;

	#if 0
				if (fabs(CurrentSetpoint) > 0) {
					duty += 1/vin * (motor->MotorParameters.R*CurrentSetpoint + motor->MotorParameters.Ke*omega_filtered);
				}

				/*if (CurrentSetpoint > 0) {
					PreviousDirection = true;
				}
				else if (CurrentSetpoint < 0) {
					PreviousDirection = false;
				}*/

				if ((CurrentSetpoint > 0 && duty < 0) ||
					(CurrentSetpoint < 0 && duty > 0))
				{
					duty = 0.0f;
				}

				float i_err = current_filtered - CurrentSetpoint;
				const float Kp = Controller_Kp;
				duty -= Kp * motor->MotorParameters.R/vin * i_err;

				if ((CurrentSetpoint > 0 && duty < 0) ||
								(CurrentSetpoint < 0 && duty > 0) ||
								(CurrentSetpoint == 0))
				{
					duty = 0.0f;
				}
	#endif

				if (CurrentSetpoint == 0) {
					duty = 0.0f;
					integral = 0;
				}

				if (duty > 1.0f) {
					duty = 1.0f;
				} else if (duty < -1.0f) {
					duty = -1.0f;
				}

				/*if (CurrentSetpoint == 0) {
					if (PreviousMode != SyncedPWMADC::COAST) {
						PreviousMode = SyncedPWMADC::COAST;
						motor->SetOperatingMode(PreviousMode);
					}
				} else {
					if (PreviousMode != SyncedPWMADC::BRAKE) {
						PreviousMode = SyncedPWMADC::BRAKE;
						motor->SetOperatingMode(PreviousMode);
					}
				}*/
				motor->SetDutyCycle_MiddleSamplingOnce(duty);

				//if (TransmitDebugDownsample) {
					controllerDebug.Timestamp = sample.Timestamp;
					controllerDebug.current_setpoint = CurrentSetpoint;
					controllerDebug.omega_measurement = omega;
					controllerDebug.omega_filtered = omega_filtered;
					controllerDebug.current_measurement = current;
					controllerDebug.current_filtered = current_filtered;
					controllerDebug.integral = integral;
					controllerDebug.PI_out = PI_out;
					controllerDebug.duty = duty;
					lspc->TransmitAsync(0x05, (uint8_t *)&controllerDebug, sizeof(ControllerDebug_t));
				/*}
				TransmitDebugDownsample = !TransmitDebugDownsample;*/
			}
		}

		//motor->SetDutyCycle_MiddleSampling(0.0f);
	}
	vTaskDelete(NULL); // delete/stop this current task
	// OBS: Destructor of this function will never be called! Therefore all objects need to be wrapped in a bracketed group.
}

void StartCurrentSweep_Callback(void * param, const std::vector<uint8_t>& data)
{
	ControllerParameters_t * params = (ControllerParameters_t*)param;
	if (data.size() == 0) {
		TaskHandle_t CurrentSweepTaskHandle;
	    // Obtain the handle of a task from its name.
		CurrentSweepTaskHandle = xTaskGetHandle( "Current sweep" );

		if (CurrentSweepTaskHandle != 0) {
			TaskStatus_t xTaskDetails;

			// Use the handle to obtain further information about the task.
			vTaskGetInfo( CurrentSweepTaskHandle,
						  &xTaskDetails,
						  pdTRUE, // Include the high water mark in xTaskDetails.
						  eInvalid ); // Include the task state in xTaskDetails.

			if (xTaskDetails.eCurrentState != eDeleted && xTaskDetails.eCurrentState != eInvalid) {
				return; // Frequency sweep task is already running
			}
		}
		StartCurrentController_Callback(params);
		xTaskCreate(CurrentSweepThread, (char *)"Current sweep", 256, (void*)0, CALIBRATION_SWEEP_PRIORITY, &CurrentSweepTaskHandle);
	}
}

void CurrentSweepThread(void * pvParameters)
{
	float amplitude = 2.f; // A
	float freq = 200; // Hz

	while (CurrentControllerEnabled)
	{
		float time = HAL_GetTime();
		CurrentSetpoint = amplitude * sinf(2*M_PI*freq * time);

		osDelayMs(0.5);
	}

	vTaskDelete(NULL); // delete/stop this current task
	// OBS: Destructor of this function will never be called! Therefore all objects need to be wrapped in a bracketed group.
}
