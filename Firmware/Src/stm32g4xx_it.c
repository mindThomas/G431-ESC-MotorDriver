/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
struct xREGISTER_STACK {
	uint32_t spare0[ 8 ];
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr; /* Link register. */
	uint32_t pc; /* Program counter. */
	uint32_t psr; /* Program status register. */
	uint32_t spare1[ 8 ];
};

volatile struct xREGISTER_STACK *pxRegisterStack = NULL;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}


#if 0
void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	/* 'pxRegisterStack' can be inspected in a break-point. */
		pxRegisterStack = ( struct xREGISTERSTACK *)
			( pulFaultStackAddress - 8 );

	// Check the Link Return (LR) address to find the function pointer where the code was executing before the crash

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " bl prvGetRegistersFromStack                               \n"
    );
}
#endif

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void __attribute__( ( naked ) ) HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
		" bl prvGetRegistersFromStack                               \n"
        /*" ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"*/
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	/* These are volatile to try and prevent the compiler/linker optimising them
	away as the variables never actually get used.  If the debugger won't show the
	values of the variables, make them global my moving their declaration outside
	of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    if (lr == 0) // no return address, probably a hard-fault due to reset back from DFU bootloader
		NVIC_SystemReset();

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}

void DisableHardware(void)
{
	__HAL_RCC_TIM1_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	__HAL_RCC_TIM3_CLK_DISABLE();
	__HAL_RCC_TIM4_CLK_DISABLE();
	__HAL_RCC_TIM6_CLK_DISABLE();
	__HAL_RCC_TIM7_CLK_DISABLE();
	__HAL_RCC_TIM8_CLK_DISABLE();
	__HAL_RCC_TIM15_CLK_DISABLE();
	__HAL_RCC_TIM16_CLK_DISABLE();
	__HAL_RCC_TIM17_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOE_CLK_DISABLE();
	__HAL_RCC_GPIOF_CLK_DISABLE();
	__HAL_RCC_GPIOG_CLK_DISABLE();
	__HAL_RCC_USART1_CLK_DISABLE();
	__HAL_RCC_USART2_CLK_DISABLE();
	__HAL_RCC_USART3_CLK_DISABLE();
	__HAL_RCC_UART4_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_I2C3_CLK_DISABLE();
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_SPI2_CLK_DISABLE();
	__HAL_RCC_SPI3_CLK_DISABLE();
}

volatile signed char * StackOverflowTaskName;
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    //Debug::printf("stack overflow in task id %lu, name: %s n", (uint32t)xTask, pcTaskName);
	// pxCurrentTCB contains a pointer to the currently running task

	StackOverflowTaskName = pcTaskName;

	DisableHardware();
    for( ;; );
}

void vApplicationMallocFailedHook( void )
{
	DisableHardware();
	for( ;; );
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
