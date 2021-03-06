/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#ifdef USE_PRECISION_SYSTICK
#include <PrecisionSysTick/PrecisionSysTick.h>
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
float convertRunTimeCounterValue(unsigned long ticks);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
void configureTimerForRunTimeStats(void)
{
    /* only needed for openOCD or Segger FreeRTOS thread awareness. It needs the symbol uxTopUsedPriority present after linking */
    {
        extern const int uxTopUsedPriority;
        __attribute__((__unused__)) volatile uint8_t dummy_value_for_openocd;
        dummy_value_for_openocd = uxTopUsedPriority;
    }
#if( configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H == 1 && configUSE_TRACE_FACILITY==1)
    /* reference FreeRTOSDebugConfig, otherwise it might get removed by the linker or optimizations */
    {
        extern const uint8_t FreeRTOSDebugConfig[];
        if (FreeRTOSDebugConfig[0]==0) { /* just use it, so the linker cannot remove FreeRTOSDebugConfig[] */
            for(;;); /* FreeRTOSDebugConfig[0] should always be non-zero, so this should never happen! */
        }
    }
#endif
#if configHEAP_SCHEME_IDENTIFICATION
    extern const uint8_t freeRTOSMemoryScheme; /* constant for NXP Kernel Awareness to indicate heap scheme */
  if (freeRTOSMemoryScheme>100) { /* reference/use variable so it does not get optimized by the linker */
    for(;;);
  }
#endif
}

inline unsigned long getRunTimeCounterValue(void)
{
#ifdef USE_PRECISION_SYSTICK
	return HAL_GetHighResTick();
#else
	return HAL_GetTick();
#endif
}

float convertRunTimeCounterValue(unsigned long ticks)
{
#ifdef USE_PRECISION_SYSTICK
    return HAL_Tick2Time(ticks);
#else
    return (float)ticks / HAL_GetTickFreq();
#endif
}

/* USER CODE END 1 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Arduino-similar function to get milli seconds count based on FreeRTOS ticks, hence only in FreeRTOS tick resolution */
uint32_t millis()
{
	 return (1000 * xTaskGetTickCount()) / configTICK_RATE_HZ;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
