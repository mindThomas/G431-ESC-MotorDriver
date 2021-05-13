#ifndef __DEFAULT_INTERRUPTS
#define __DEFAULT_INTERRUPTS

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef DEFAULTINTERRUPTS_USE_SYSTICK
void SysTick_Handler(void);
#endif

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // __DEFAULT_INTERRUPTS