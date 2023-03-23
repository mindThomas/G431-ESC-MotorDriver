#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_MspInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */