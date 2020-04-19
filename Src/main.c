/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FirstOrderLPF.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef uart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART2_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void SetTimerFrequencyWith50pctDutyCycle(uint32_t freq);
void SetTimerFrequencyAndDutyCycle_MiddleSampling(uint32_t freq, float dutyPct);
void SetTimerFrequencyAndDutyCycle_EndSampling(uint32_t freq, float dutyPct);
void SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(uint32_t freq, float dutyPct);
void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Copied from stm32fg4xx_hal_tim_ex.c
void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  SystemClock_Config();  // needs to be done as the first thing
  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#if 0 // do not include the SystemClock_Config() below
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_UART2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV5;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;//RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // Take ADC PLL-based clock (68 MHz) and divide with this Prescaler
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#if 0
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE BEGIN ADC1_Init 2 */
  // For VrefInt
  /*LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);*/
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // Take ADC PLL-based clock (68 MHz) and divide with this Prescaler
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  //sConfig.Channel = ADC_CHANNEL_1; // Vin
  sConfig.Channel = ADC_CHANNEL_14; // BEMF3
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


#if 0
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP; //TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 49;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF_RISING_OC6REF_RISING; //TIM_TRGO2_OC5REF_RISING_OC6REF_RISING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

  sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  //sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  //sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF; // 16-bit timers
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}


/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART2_Init(void)
{/* USER CODE BEGIN LPUART1_Init 0 */

	  /* USER CODE END LPUART1_Init 0 */

	  /* USER CODE BEGIN LPUART1_Init 1 */

	  /* USER CODE END LPUART1_Init 1 */
	  uart2.Instance = USART2;
	  uart2.Init.BaudRate = 115200;
	  uart2.Init.WordLength = UART_WORDLENGTH_8B;
	  uart2.Init.StopBits = UART_STOPBITS_1;
	  uart2.Init.Parity = UART_PARITY_NONE;
	  uart2.Init.Mode = UART_MODE_TX_RX;
	  uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  uart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  uart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&uart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_UARTEx_SetTxFifoThreshold(&uart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_UARTEx_SetRxFifoThreshold(&uart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_UARTEx_DisableFifoMode(&uart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN LPUART1_Init 2 */

	  /* USER CODE END LPUART1_Init 2 */
}

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

int32_t Encoder_Get();
#define SAMPLE_BUFFER_SIZE 2000
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
			if (recordSamples) {
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
				if (sample_index == SAMPLE_BUFFER_SIZE) {
					TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE); // Enable Coast mode
				}
			}

			sampleIndex++;
		//}
	}
	else if (hadc->Instance == ADC2) {
		if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
			if (recordSamples) {
				time_vin = HAL_GetHighResTick();
				sample_vin = HAL_ADC_GetValue(hadc);
			}
		}
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		//if (hadc1.DMA_Handle->State == HAL_DMA_STATE_BUSY) {
		if (DMA_Transfer_Ongoing) {
			GPIOA->ODR ^= GPIO_PIN_15;
		}
		//GPIOA->BSRR = GPIO_PIN_15;
		/*
		GPIOA->BSRR = GPIO_PIN_15;
		uint8_t i;
		for (i = 0; i < 10; i++) asm("nop");
		GPIOA->BRR = GPIO_PIN_15;
		*/
	}
}

void SetTimerFrequencyWith50pctDutyCycle(uint32_t freq)
{
	const uint32_t PCLK = 170000000;
	const uint16_t PRESCALER = 32;
	const float ADC_SAMPLE_TIME_US = 5; //20; // 16 us // See MATLAB script: 'ADC_Configuration.m'

	uint32_t ARR = (PCLK / (uint32_t)(freq*(PRESCALER+1))) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim1, ARR);

	uint16_t END = ARR + 1;
	uint16_t CENTER = END / 2;
	uint16_t ADCsampleCnt = (END*freq) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER/2 - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - CENTER/2 - SampleDuty); // sample location

    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - SampleDuty); // sample location

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = CENTER - ADCsampleCnt;
	uint16_t sampleLocation2 = END - ADCsampleCnt;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, sampleLocation1);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, sampleLocation2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sampleLocation2);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CENTER);
}

void SetTimerFrequencyAndDutyCycle_MiddleSampling(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = 170000000; // timer PCLK
	const uint16_t PRESCALER = 32; // timer prescaler (should match configuration above)
	const float ADC_SAMPLE_TIME_US = 5; //20; // 16 us // See MATLAB script: 'ADC_Configuration.m'

	uint32_t ARR = (PCLK / (uint32_t)(freq*(PRESCALER+1))) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim1, ARR);

	if (dutyPct > 1) dutyPct = 1.0;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = END * dutyPct;
	uint16_t ADCsampleCnt = (END*freq) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - SampleDuty); // sample location
	uint16_t sampleLocation1 = 1;
	uint16_t sampleLocation2 = 0;

	if (ADCsampleCnt < CENTER/2) {
		sampleLocation1 = CENTER/2 - ADCsampleCnt; // sample location - at the midpoint of the ON-period minus the sample duration
	} // else sampleLocation1 = 1;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, sampleLocation1);

	if (((END - CENTER) / 2 + CENTER - ADCsampleCnt) > (sampleLocation1 + ADCsampleCnt)) {
		sampleLocation2 = (END - CENTER) / 2 + CENTER - ADCsampleCnt;  // sample location - at the midpoint of the OFF-period minus the sample duration
	}
	else if ((END - ADCsampleCnt) > (sampleLocation1 + ADCsampleCnt)) {
		sampleLocation2 = (END - ADCsampleCnt);
	} // else sampleLocation2 = 0; // disabled
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, sampleLocation2); // sample location - at the midpoint of the OFF-period minus the sample duration
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sampleLocation2); // sample location - at the midpoint of the OFF-period minus the sample duration

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CENTER); // control the duty cycle with the other side of the motor
}

void SetTimerFrequencyAndDutyCycle_EndSampling(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = 170000000;
	const uint16_t PRESCALER = 32;
	const float ADC_SAMPLE_TIME_US = 5; //20; // 16 us // See MATLAB script: 'ADC_Configuration.m'

	uint32_t ARR = (PCLK / (uint32_t)(freq*(PRESCALER+1))) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim1, ARR);

	if (dutyPct > 1) dutyPct = 1.0;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = END * dutyPct;
	uint16_t ADCsampleCnt = (END*freq) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	//(CCRx / (ARR + 1)) = dutyPct

    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER/2 - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - CENTER/2 - SampleDuty); // sample location

    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, CENTER - SampleDuty); // sample location
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, END - SampleDuty); // sample location

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = 1;
	uint16_t sampleLocation2 = 0;

	if (CENTER > ADCsampleCnt) {
		sampleLocation1 = CENTER - ADCsampleCnt;
	}
	if (END > ADCsampleCnt) {
		sampleLocation2 = END - ADCsampleCnt;
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, sampleLocation1);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, sampleLocation2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sampleLocation2);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CENTER);
}

void SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = 170000000; // timer PCLK
	const uint16_t PRESCALER = 32; // timer prescaler (should match configuration above)
	const float ADC_SAMPLE_TIME_US = 5; //20; // 16 us // See MATLAB script: 'ADC_Configuration.m'

	uint32_t ARR = (PCLK / (uint32_t)(freq*(PRESCALER+1))) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim1, ARR);

	if (dutyPct > 1) dutyPct = 1.0;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = END * dutyPct;
	uint16_t ADCsampleCnt = (END*freq) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	uint16_t sampleLocation = 1;

	if (dutyPct > 0.5) {
		sampleLocation = CENTER/2 - ADCsampleCnt; // sample location - at the midpoint of the ON-period minus the sample duration
	} else {
		sampleLocation = (END - CENTER) / 2 + CENTER - ADCsampleCnt;  // sample location - at the midpoint of the OFF-period minus the sample duration
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, sampleLocation);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, sampleLocation);

	// Only sample once
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CENTER); // control the duty cycle with the other side of the motor
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
/* USER CODE END 4 */

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

__IO uint16_t ADC_DMA_Samples[33]; // should be +1 due to the first sample always being from the previous ADC sample when starting the DMA sequence

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Disable TIM1 CH3N (set low) to force OUT3 to be toggle between high and floating depending on PWM (instead of toggling between high and low)
    /*GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_InitStruct.Pin, GPIO_PIN_RESET);*/

	// Enable Encoder interface
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE); // Enable overflow (update) interrupt

	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	FirstOrderLPF_Init(&currentSenseLPF, 1.f / 8500, 0.01f);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    //HAL_ADCEx_InjectedStart_IT(&hadc1);
    //HAL_ADC_Start_IT(&hadc1);
    //__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOS);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_Samples, sizeof(ADC_DMA_Samples)/sizeof(ADC_DMA_Samples[0]));
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR); // disable Overrun interrupt since it will be triggered all the time during the "idle" period between DMA reads

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_IT(&hadc2);

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

    // Disable TIM1 CH3N (set low) to force OUT3 to be toggle between high and floating depending on PWM (instead of toggling between high and low)
    //TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

    // Enable BEMF sense
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_InitStruct.Pin, GPIO_PIN_RESET);

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
	while (sample_index < 2*SAMPLE_BUFFER_SIZE)
	{
		osDelay(1);
	}

	recordSamples = 0;
#endif

	//SetTimerFrequencyAndDutyCycle(20000, 0.1);
	//osDelay(3000);

	// Disable motor and change to coast mode
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6, 0);

	HAL_UART_Transmit(&uart2, times_low, sizeof(times_low), 0xFFFF);
	HAL_UART_Transmit(&uart2, samples_low, sizeof(samples_low), 0xFFFF);
	HAL_UART_Transmit(&uart2, times_high, sizeof(times_high), 0xFFFF);
	HAL_UART_Transmit(&uart2, samples_high, sizeof(samples_high), 0xFFFF);
	//HAL_UART_Transmit(&uart2, time_vin, sizeof(time_vin), 0xFFFF);
	//HAL_UART_Transmit(&uart2, sample_vin, sizeof(sample_vin), 0xFFFF);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

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
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM1) {
	  //sampleIndex = 0;
	  //GPIOA->BSRR = GPIO_PIN_15;
      /* Start the DMA channel */
	  if (hadc1.DMA_Handle->State != HAL_DMA_STATE_READY) {
		  HAL_DMA_Abort(hadc1.DMA_Handle);
	  }
	  __HAL_ADC_CLEAR_FLAG(&hadc1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	  /*uint32_t tmp = hadc1.Instance->DR; // Read existing data in ADC (if any), to ensure that DMA starts clean
	  tmp = hadc1.Instance->ISR;
	  __HAL_ADC_CLEAR_FLAG(&hadc1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	  hadc1.DMA_Handle->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hadc1.DMA_Handle->ChannelIndex & 0x1FU));*/
	  /*TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_5, TIM_CCx_ENABLE);
	  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_6, TIM_CCx_ENABLE);*/
	  // Even though I try clearing all the above, the DMA sample will still include the previous/most recent ADC sample as the first one, which is thus a sample captured at the OFF end.
      HAL_DMA_Start_IT(hadc1.DMA_Handle, (uint32_t)&hadc1.Instance->DR, (uint32_t*)&ADC_DMA_Samples, sizeof(ADC_DMA_Samples)/sizeof(ADC_DMA_Samples[0]));
      DMA_Transfer_Ongoing = 1;
	  //ADC_Sample_Processing();
  }
  else if (htim->Instance == TIM4) {
	// Encoder overflow detected
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) // encoder is turning negative
		EncoderOffset -= 0x10000;
	else // encoder is turning positive
		EncoderOffset += 0x10000;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
