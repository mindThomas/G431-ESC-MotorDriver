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
#include "Matrix.hpp"
#include "LeastSquares.h"
#include "NonlinearLeastSquares.hpp"


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
void TestFitExponentialDecay(void);

typedef enum {
	NONE,
	CURRENT,
	VBUS,
	BEMF
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

bool reset_tx_packing = true;

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

	UART * uart = new UART(UART::PORT_UART2, 921600, 100, true);
	LSPC * lspc = new LSPC(uart, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY);
	Debug::AssignDebugCOM((void*)lspc);

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

	lspc->registerCallback(0x01, SetDutyCycle_Callback, motor);
	lspc->registerCallback(0x02, SetFreq_Callback, motor);
	lspc->registerCallback(0x03, SetOperatingMode_Callback, motor);
	lspc->registerCallback(0x04, SetActiveInactive_Callback, motor);
	lspc->registerCallback(0x05, SetAveraging_Callback, motor);
	lspc->registerCallback(0x06, StartFrequencySweep_Callback, motor);
	lspc->registerCallback(0x07, StartDutySweep_Callback, motor);
	lspc->registerCallback(0x08, Calibration_Callback, motor);
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

		if ( xQueueReceive( motor->SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
			while (1) {
				if (lspc->TransmitAsync(0x04, (uint8_t *)&sample, sizeof(SyncedPWMADC::CombinedSample_t)-1)) { // minus 1 because of the extra internal variable boolean
					break;
				}
				osDelay(1);
			}
		}
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
		motor->SetDutyCycle_EndSampling(dutyCycle);

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

		motor->SetPWMFrequency(PWM_freq);
		motor->SetSamplingInterval(PWM_freq/Sample_freq);
		motor->SetDutyCycle_EndSampling(motor->GetCurrentDutyCycle());

		reset_tx_packing = true;
	}
}

void SetAveraging_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (data.size() == 2) {
		uint16_t* values = (uint16_t*)data.data();
		uint16_t AveragingCount = values[0];

		motor->SetNumberOfAveragingSamples(AveragingCount);
	}
}

void SetOperatingMode_Callback(void * param, const std::vector<uint8_t>& data)
{
	SyncedPWMADC * motor = (SyncedPWMADC*)param;
	if (!motor) return;

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
	while (freq <= 10000) {
		motor->SetPWMFrequency(freq, false);
		motor->SetDutyCycle_EndSampling(0.5);

		freq += 10;

		for (uint8_t samples = 0; samples < 20; samples++)
			motor->WaitForNewQueuedSample();
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
		xTaskCreate(DutySweepThread, (char *)"Duty sweep", 256, (void*) motor, 6, &DutySweepTaskHandle);
	}
}


void DutySweepThread(void * pvParameters)
{
	SyncedPWMADC * motor = (SyncedPWMADC *)pvParameters;
	motor->SetOperatingMode(SyncedPWMADC::BRAKE);
	motor->SetSamplingInterval(2);
	motor->SetNumberOfAveragingSamples(1);

	uint32_t freq = 500;
	motor->SetPWMFrequency(freq); // configure prescaler using starting frequency
	motor->WaitForNewQueuedSample();
	for (float duty = 0.1; duty < 1.0; duty += 0.05) {
		motor->SetDutyCycle_EndSampling(duty);
		osDelay(500);
	}

	motor->SetDutyCycle_EndSampling(0);
	motor->SetPWMFrequency(250);

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
