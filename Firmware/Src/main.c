/* Copyright (C) 2020- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mallocTracker.h"

// FreeRTOS include
#ifdef USE_FREERTOS_CMSIS
#include <cmsis_os.h>
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#endif

/* Private includes ----------------------------------------------------------*/
#include "MainTask.h"
#include "MemoryManagement.h"
#include "Priorities.h"
#include "ProcessorInit.h"
#include "TestTask.h"
#include <MATLABCoderInit/MATLABCoderInit.h>

/* Private variables ---------------------------------------------------------*/
TaskHandle_t mainTaskHandle;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    /* Configure the system clock - needs to be done as the first thing */
    SystemClock_Config();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    MATLABCoder_initialize();

    enableMallocTracking();

    /* Create the main thread which creates objects and spawns the rest of the threads */
    xTaskCreate(MainTask, "mainTask", 300, (void*)NULL, MAIN_TASK_PRIORITY, &mainTaskHandle);

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    while (1) {}
}

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
    LL_PWR_DisableDeadBatteryPD();
}