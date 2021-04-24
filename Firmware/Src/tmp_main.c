#include "stm32g4xx_hal_fdcan.h"
#include "FirstOrderLPF.h"
#include <math.h>
#include <string.h>

void SetTimerFrequencyWith50pctDutyCycle(uint32_t freq);
void SetTimerFrequencyAndDutyCycle_MiddleSampling(uint32_t freq, float dutyPct);
void SetTimerFrequencyAndDutyCycle_EndSampling(uint32_t freq, float dutyPct);
void SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(uint32_t freq, float dutyPct);

/* USER CODE BEGIN 4 */
uint8_t sampleIndex = 0;
uint16_t ADC_Samples[4];
uint16_t Vref = 0;
float currentSenseOffset = 0;
uint32_t currentSenseOffset_uint = 0;
const uint32_t CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED = 1000;
uint32_t currentSenseOffsetSampleCount = 10000;
float currentSense = 0;

_Bool recordSamples = 0;
_Bool DMA_Transfer_Ongoing = 0;

_Bool DirectionForward = 0;

int32_t Encoder_Get();
#define SAMPLE_BUFFER_SIZE 100
uint16_t sample_index = 0;
uint32_t times_low[SAMPLE_BUFFER_SIZE];
uint32_t times_high[SAMPLE_BUFFER_SIZE];
int16_t samples_low[SAMPLE_BUFFER_SIZE];
int16_t samples_high[SAMPLE_BUFFER_SIZE];

uint32_t time_vin;
int16_t sample_vin;

uint32_t freq = 500;
float duty = 0.5;

//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		//if ( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC)) {
		/*
		if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC) && sampleIndex < 4) {
			ADC_Samples[sampleIndex] = HAL_ADC_GetValue(hadc);
			sampleIndex++;
			GPIOA->ODR ^= GPIO_PIN_15;
		}
		*/
		//if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
			DMA_Transfer_Ongoing = 0;
			/*TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_5, TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_6, TIM_CCx_DISABLE);*/
			//GPIOA->ODR ^= GPIO_PIN_15;
			/*if (recordSamples) {
				GPIOA->BRR = GPIO_PIN_15;
			}*/
			if (recordSamples) {
				time_vin = HAL_GetHighResTick();
				sample_vin = HAL_ADC_GetValue(hadc);
				GPIOA->BRR = GPIO_PIN_15;
			}

			if (currentSenseOffsetSampleCount < CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED) {
				if ((sampleIndex % 2) == 0) {
					currentSenseOffset_uint += HAL_ADC_GetValue(hadc);
					currentSenseOffsetSampleCount++;
				}
			}
			if (recordSamples) {
				//int32_t tmp = (int32_t)HAL_ADC_GetValue(hadc) - (int32_t)currentSenseOffset_uint;
				int32_t tmp = (int32_t)HAL_ADC_GetValue(hadc);
				//int32_t tmp = __LL_ADC_CALC_VREFANALOG_VOLTAGE((int32_t)HAL_ADC_GetValue(hadc), LL_ADC_RESOLUTION_12B); // for Vref reading conversion
				if (sample_index < 2*SAMPLE_BUFFER_SIZE) {
					if ((sampleIndex % 2) == 1) {
						times_low[(sample_index / 2)] = HAL_GetHighResTick();
						samples_low[(sample_index / 2)] = tmp;
					} else {
						times_high[(sample_index / 2)] = HAL_GetHighResTick();
						samples_high[(sample_index / 2)] = tmp;
					}
				}

				sample_index++;
				/*if (sample_index == SAMPLE_BUFFER_SIZE) {
					TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE); // Enable Coast mode
				}*/
			}

			sampleIndex++;
		//}
	}
	else if (hadc->Instance == ADC2) {
		/*if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
			if (recordSamples) {
				time_vin = HAL_GetHighResTick();
				sample_vin = HAL_ADC_GetValue(hadc);
				GPIOA->BRR = GPIO_PIN_15;
			}
		}*/
		DMA_Transfer_Ongoing = 0;
		if (recordSamples) {
			GPIOA->BRR = GPIO_PIN_15;
		}
	}
}


FirstOrderLPF currentSenseLPF;
void ADC_Sample_Processing()
{
	// Make a local copy of the ADC samples
	uint16_t _ADC_Samples_[sizeof(ADC_Samples) / sizeof(uint16_t)];
	memcpy(_ADC_Samples_, ADC_Samples, sizeof(ADC_Samples));


	if (currentSenseOffsetSampleCount < CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED) {
		float vSense = _ADC_Samples_[0];
		currentSense = FirstOrderLPF_Filter(&currentSenseLPF, vSense);
		currentSenseOffset += currentSense;
		currentSenseOffsetSampleCount++;
	} else if (currentSenseOffsetSampleCount > CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED) {
		float vSense = (float)_ADC_Samples_[0] - currentSenseOffset;
		currentSense = FirstOrderLPF_Filter(&currentSenseLPF, vSense);
		//currentSense = _ADC_Samples_[0] - currentSenseOffset;
	} else {
		float vSense = _ADC_Samples_[0];
		currentSense = FirstOrderLPF_Filter(&currentSenseLPF, vSense);
	}

	Vref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(_ADC_Samples_[2], ADC_RESOLUTION_12B);
}

const float Kp = 0.001f;
const float Ki = 1.0f;
const float Kd = 0.0f;
const float max_integral = 1.0;
float PID(float err, float dt)
{
	static float err_prev = 0;
	static float integral = 0;

	integral += err * dt;
	if (integral < -max_integral) integral = -max_integral;
	if (integral > max_integral) integral = max_integral;

	float err_diff = (err - err_prev) / dt;
	float out = Kp * err + Ki * integral + Kd * err_diff;

	err_prev = err;

	return out;
}

void setPWM(int16_t duty)
{
	if (duty > 1000) duty = 1000;
	if (duty < -1000) duty = -1000;

	if (duty > 0) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -duty);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	}

	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
}

int32_t EncoderOffset = -(0x10000); // a single interrupt will be fired the first time the encoder is enabled - this initial offset is thus to compensate for that
int32_t Encoder_Get()
{
	//return -( (uint16_t)__HAL_TIM_GET_COUNTER(&_hRes->handle) + _hRes->offsetValue); // invert direction
	return (uint16_t)__HAL_TIM_GET_COUNTER(&htim4) + EncoderOffset;
}

void SetMaxDuty(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**TIM1 GPIO Configuration
    PC13     ------> TIM1_CH1N
    PB15     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // TIM1_CH1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // TIM1_CH1N
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // TIM1_CH3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // TIM1_CH3N
}

void AllOff(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**TIM1 GPIO Configuration
    PC13     ------> TIM1_CH1N
    PB15     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // TIM1_CH1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // TIM1_CH1N
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // TIM1_CH2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // TIM1_CH2N
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // TIM1_CH3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // TIM1_CH3N
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
float currentSense_mV = 0;
float currentSense_mA = 0;
float currentSetpoint_mA = 200;
float PWM = 0;

int32_t EncoderTicks = 0;

__IO uint16_t ADC_DMA_Samples1[33]; // should be +1 due to the first sample always being from the previous ADC sample when starting the DMA sequence
__IO uint16_t ADC_DMA_Samples2[33]; // should be +1 due to the first sample always being from the previous ADC sample when starting the DMA sequence

uint8_t UART_Data[4608] = {0};
uint8_t UART_RX_Data[10] = {0};

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	// Disable TIM1 CH3N (set low) to force OUT3 to be toggle between high and floating depending on PWM (instead of toggling between high and low)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_InitStruct.Pin, GPIO_PIN_RESET);*/

	FirstOrderLPF_Init(&currentSenseLPF, 1.f / 8500, 0.01f);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    //HAL_ADCEx_InjectedStart_IT(&hadc1);
    //HAL_ADC_Start_IT(&hadc1);
    //__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOS);
    //HAL_ADC_Start_IT(&hadc2);

    //HAL_ADC_Start_IT(&hadc2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_Samples1, sizeof(ADC_DMA_Samples1)/sizeof(ADC_DMA_Samples1[0]));
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR); // disable Overrun interrupt since it will be triggered all the time during the "idle" period between DMA reads
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&ADC_DMA_Samples2, sizeof(ADC_DMA_Samples2)/sizeof(ADC_DMA_Samples2[0]));
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_OVR); // disable Overrun interrupt since it will be triggered all the time during the "idle" period between DMA reads

	// Start PWM interface
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_6);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4); // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

	// Coast mode requires disconnecting the motor in the OFF state
	// With all MOSFETs operating normally (in Brake mode) when the PWM is in OFF state both bottom MOSFETs are on,
	// hence shorting the motor serving as brake mode
	// With the MOSFET manager chips (L6387ED) we can disable the bottom
    // Disable TIM1 CH3N (set low) to force OUT3 to be toggle between high and floating depending on PWM (instead of toggling between high and low)
    //TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

#if 0
	SetTimerFrequencyWith50pctDutyCycle(100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	osDelay(2000); // wait for stabilization

	currentSenseOffsetSampleCount = 0; // sample offset
	while (currentSenseOffsetSampleCount < CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED)
		osDelay(1);

	currentSenseOffset_uint = currentSenseOffset_uint / currentSenseOffsetSampleCount;
	currentSenseOffsetSampleCount = CURRENT_SENSE_OFFSET_SAMPLE_COUNT_FINISHED + 1; // marks that offset computation has finished
#endif

	osDelay(1000); // wait for stabilization
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

#if 0
	//SetTimerFrequencyWith50pctDutyCycle(100);
	//SetTimerFrequencyAndDutyCycle(100, 0.5);
	SetTimerFrequencyAndDutyCycle_EndSampling(10000, 0.8);
	osDelay(500); // wait for stabilization

	recordSamples = 1;

	int16_t setpoint = 2842;

	uint32_t currentTime = HAL_GetTick();
	uint8_t sampleTick = 0;
	//while ((HAL_GetTick() - currentTime) < 10000) {
	while (sample_index < SAMPLE_BUFFER_SIZE) {
		/*int32_t sense_sum = 0;
		for (uint8_t i = 0; i < 10; i++) {
			sense_sum += sample_low[i];
			sense_sum += sample_high[i];
		}

		volatile int16_t current_sense = sense_sum / 20;
		volatile float error = setpoint - current_sense;
		volatile float duty = PID(error, 0.001);

		if (duty < 0.1) duty = 0.1;
		if (duty > 0.9) duty = 0.9;

		SetTimerFrequencyAndDutyCycle_MiddleSampling(10000, duty);

		if ((++sampleTick % 5) == 0) {
			times[sample_index] = HAL_GetHighResTick();
			samples_current_sense[sample_index] = current_sense;
			samples_encoder[sample_index] = Encoder_Get();
			sample_index++;
		}

		EncoderTicks = Encoder_Get();*/

		osDelay(1);
	}

	recordSamples = 0;
#endif


#if 0
	SetTimerFrequencyAndDutyCycle_EndSampling(500, 0.8);
	osDelay(1000); // wait for stabilization

	SetTimerFrequencyAndDutyCycle_EndSampling(500, 0.5);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // Disable TIM1 CH3N (set low) to force OUT3 to be toggle between high and floating depending on PWM (instead of toggling between high and low)
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

	recordSamples = 1;

	uint32_t currentTime = HAL_GetTick();
	while (sample_index < SAMPLE_BUFFER_SIZE)
	{
		times[sample_index] = HAL_GetHighResTick();
		samples_bemf[sample_index] = sample_vin;
		samples_encoder[sample_index] = Encoder_Get();
		sample_index++;

		if (sample_index > 100 && sample_vin < 700) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		}

		osDelay(2);
	}

	recordSamples = 0;
#endif

#if 0
	SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(10000, 0);
	osDelay(500);

	recordSamples = 1;

	uint32_t currentTime = HAL_GetTick();
	while (sample_index < SAMPLE_BUFFER_SIZE)
	{
		int32_t sense_sum = 0;
		for (uint8_t i = 0; i < 10; i++) {
			sense_sum += sample_low[i];
			//sense_sum += sample_high[i];
		}

		volatile int16_t current_sense = sense_sum / 20;

		times[sample_index] = HAL_GetHighResTick();
		samples_current_sense[sample_index] = current_sense;
		samples_encoder[sample_index] = Encoder_Get();
		sample_index++;

		if ((sample_index % 200) == 0) {
			SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(10000, (float)sample_index / 2000.f);
			osDelay(500);
		}

		osDelay(1);
	}

	recordSamples = 0;
#endif

#if 0
	SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(10000, 0);
	recordSamples = 1;

	uint32_t currentTime = HAL_GetTick();
	while (sample_index < SAMPLE_BUFFER_SIZE)
	{
		SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(10000, (float)sample_index / 2300.f);
		osDelay(500);

		for (uint8_t i = 0; i < 100; i++) {
			times[sample_index] = time_middle[i];
			samples_current_sense[sample_index] = sample_middle[i];
			samples_encoder[sample_index] = encoder_middle[i];
			sample_index++;
		}
	}

	recordSamples = 0;
#endif

#if 1
	//TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE); // Enable Coast mode
	SetTimerFrequencyAndDutyCycle_EndSampling(freq, duty);
	osDelay(1000);
	recordSamples = 1;

	uint32_t currentTime = HAL_GetTick();
	while (sample_index < SAMPLE_BUFFER_SIZE)
	{
		DirectionForward = 1;
		SetTimerFrequencyAndDutyCycle_EndSampling(freq, duty);
		osDelay(2000);
		DirectionForward = 0;
		SetTimerFrequencyAndDutyCycle_EndSampling(freq, duty);
		osDelay(2000);
	}

	recordSamples = 0;
#endif

	//SetTimerFrequencyAndDutyCycle(20000, 0.1);
	//osDelay(3000);

	// Disable motor and change to coast mode
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);
	TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, 0);

	while (1)
		osDelay(1);


	/* Infinite loop */
	while (1)
	{
		/* Remember to disable InternalOutput of OPAMPs */
		currentSense_mV = Vref * currentSense / __LL_ADC_DIGITAL_SCALE(ADC_RESOLUTION_12B);
		currentSense_mA = 36.458f * currentSense_mV;

		float err = currentSetpoint_mA - currentSense_mA;
		float out = PID(err, 0.001);
		if (out < 120) out = 120;
		if (out > 600) out = 600;
		PWM = out;
		setPWM(PWM);

		osDelay(1);
	}
}
