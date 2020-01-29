/**
 * @file startup.c
 * @author Ian Frosst
 * @brief STM32F767ZI startup code
 * @date 2020-01-22
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <stdint.h>
#include <string.h>

#include "system_stm32f7xx.h"

#ifndef __HEAP_SIZE
#define __HEAP_SIZE 0
#endif

#ifndef __STACK_SIZE
#define __STACK_SIZE 0xc00
#endif

#define DEFINE_DEFAULT_ISR(name) \
  __attribute__((interrupt,weak,alias("Default_Handler"))) \
  void name(void);

typedef void (*const ISR_t)(void);

/* Function declarations */
extern void _start(void);

/* Symbols defined in linker script */
extern uint8_t __StackTop;

extern uint8_t __data_start[];
extern uint8_t __data_loadaddr[];
extern uint8_t __data_size[];

extern uint8_t __ramfunc_start[];
extern uint8_t __ramfunc_loadaddr[];
extern uint8_t __ramfunc_size[];

// Hook functions called by _start (_mainCRTStartup) in newlib crt0.o
void hardware_init_hook(void) {
  SystemInit(); // Call CMSIS `SystemInit` to set up clocks, etc.
}

void software_init_hook(void) {
  // Copy `.data` section into ram
  memcpy(__data_start, __data_loadaddr, (size_t)__data_size);
  if (__ramfunc_size != 0) {
    memcpy(__ramfunc_start, __ramfunc_loadaddr, (size_t)__ramfunc_size);
  }
}

__attribute__((naked)) void Default_Handler(void) {
  asm (
  "  tst  lr, #4      \n"
  "  ite  eq          \n"
  "  mrseq  r0, msp   \n"
  "  mrsne  r0, psp   \n"
  "  b  panic         \n"
  );
}

#if __HEAP_SIZE != 0
__attribute__((used,section(".heap"))) uint8_t __heap[__HEAP_SIZE];
#endif

__attribute__((used,section(".stack"))) uint8_t __stack_dummy[__STACK_SIZE];

DEFINE_DEFAULT_ISR(NMI_Handler);
DEFINE_DEFAULT_ISR(HardFault_Handler);
DEFINE_DEFAULT_ISR(MemManage_Handler);
DEFINE_DEFAULT_ISR(BusFault_Handler);
DEFINE_DEFAULT_ISR(UsageFault_Handler);
DEFINE_DEFAULT_ISR(SVC_Handler);
DEFINE_DEFAULT_ISR(DebugMon_Handler);
DEFINE_DEFAULT_ISR(PendSV_Handler);
DEFINE_DEFAULT_ISR(SysTick_Handler);

DEFINE_DEFAULT_ISR(WWDG_IRQHandler);
DEFINE_DEFAULT_ISR(PVD_IRQHandler);
DEFINE_DEFAULT_ISR(TAMP_STAMP_IRQHandler);
DEFINE_DEFAULT_ISR(RTC_WKUP_IRQHandler);
DEFINE_DEFAULT_ISR(FLASH_IRQHandler);
DEFINE_DEFAULT_ISR(RCC_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI0_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI1_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI2_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI3_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI4_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream0_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream1_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream2_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream3_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream4_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream5_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream6_IRQHandler);
DEFINE_DEFAULT_ISR(ADC_IRQHandler);
DEFINE_DEFAULT_ISR(CAN1_TX_IRQHandler);
DEFINE_DEFAULT_ISR(CAN1_RX0_IRQHandler);
DEFINE_DEFAULT_ISR(CAN1_RX1_IRQHandler);
DEFINE_DEFAULT_ISR(CAN1_SCE_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI9_5_IRQHandler);
DEFINE_DEFAULT_ISR(TIM1_BRK_TIM9_IRQHandler);
DEFINE_DEFAULT_ISR(TIM1_UP_TIM10_IRQHandler);
DEFINE_DEFAULT_ISR(TIM1_TRG_COM_TIM11_IRQHandler);
DEFINE_DEFAULT_ISR(TIM1_CC_IRQHandler);
DEFINE_DEFAULT_ISR(TIM2_IRQHandler);
DEFINE_DEFAULT_ISR(TIM3_IRQHandler);
DEFINE_DEFAULT_ISR(TIM4_IRQHandler);
DEFINE_DEFAULT_ISR(I2C1_EV_IRQHandler);
DEFINE_DEFAULT_ISR(I2C1_ER_IRQHandler);
DEFINE_DEFAULT_ISR(I2C2_EV_IRQHandler);
DEFINE_DEFAULT_ISR(I2C2_ER_IRQHandler);
DEFINE_DEFAULT_ISR(SPI1_IRQHandler);
DEFINE_DEFAULT_ISR(SPI2_IRQHandler);
DEFINE_DEFAULT_ISR(USART1_IRQHandler);
DEFINE_DEFAULT_ISR(USART2_IRQHandler);
DEFINE_DEFAULT_ISR(USART3_IRQHandler);
DEFINE_DEFAULT_ISR(EXTI15_10_IRQHandler);
DEFINE_DEFAULT_ISR(RTC_Alarm_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_FS_WKUP_IRQHandler);
DEFINE_DEFAULT_ISR(TIM8_BRK_TIM12_IRQHandler);
DEFINE_DEFAULT_ISR(TIM8_UP_TIM13_IRQHandler);
DEFINE_DEFAULT_ISR(TIM8_TRG_COM_TIM14_IRQHandler);
DEFINE_DEFAULT_ISR(TIM8_CC_IRQHandler);
DEFINE_DEFAULT_ISR(DMA1_Stream7_IRQHandler);
DEFINE_DEFAULT_ISR(FMC_IRQHandler);
DEFINE_DEFAULT_ISR(SDMMC1_IRQHandler);
DEFINE_DEFAULT_ISR(TIM5_IRQHandler);
DEFINE_DEFAULT_ISR(SPI3_IRQHandler);
DEFINE_DEFAULT_ISR(UART4_IRQHandler);
DEFINE_DEFAULT_ISR(UART5_IRQHandler);
DEFINE_DEFAULT_ISR(TIM6_DAC_IRQHandler);
DEFINE_DEFAULT_ISR(TIM7_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream0_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream1_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream2_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream3_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream4_IRQHandler);
DEFINE_DEFAULT_ISR(ETH_IRQHandler);
DEFINE_DEFAULT_ISR(ETH_WKUP_IRQHandler);
DEFINE_DEFAULT_ISR(CAN2_TX_IRQHandler);
DEFINE_DEFAULT_ISR(CAN2_RX0_IRQHandler);
DEFINE_DEFAULT_ISR(CAN2_RX1_IRQHandler);
DEFINE_DEFAULT_ISR(CAN2_SCE_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_FS_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream5_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream6_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2_Stream7_IRQHandler);
DEFINE_DEFAULT_ISR(USART6_IRQHandler);
DEFINE_DEFAULT_ISR(I2C3_EV_IRQHandler);
DEFINE_DEFAULT_ISR(I2C3_ER_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_HS_EP1_OUT_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_HS_EP1_IN_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_HS_WKUP_IRQHandler);
DEFINE_DEFAULT_ISR(OTG_HS_IRQHandler);
DEFINE_DEFAULT_ISR(DCMI_IRQHandler);
DEFINE_DEFAULT_ISR(RNG_IRQHandler);
DEFINE_DEFAULT_ISR(FPU_IRQHandler);
DEFINE_DEFAULT_ISR(UART7_IRQHandler);
DEFINE_DEFAULT_ISR(UART8_IRQHandler);
DEFINE_DEFAULT_ISR(SPI4_IRQHandler);
DEFINE_DEFAULT_ISR(SPI5_IRQHandler);
DEFINE_DEFAULT_ISR(SPI6_IRQHandler);
DEFINE_DEFAULT_ISR(SAI1_IRQHandler);
DEFINE_DEFAULT_ISR(LTDC_IRQHandler);
DEFINE_DEFAULT_ISR(LTDC_ER_IRQHandler);
DEFINE_DEFAULT_ISR(DMA2D_IRQHandler);
DEFINE_DEFAULT_ISR(SAI2_IRQHandler);
DEFINE_DEFAULT_ISR(QUADSPI_IRQHandler);
DEFINE_DEFAULT_ISR(LPTIM1_IRQHandler);
DEFINE_DEFAULT_ISR(CEC_IRQHandler);
DEFINE_DEFAULT_ISR(I2C4_EV_IRQHandler);
DEFINE_DEFAULT_ISR(I2C4_ER_IRQHandler);
DEFINE_DEFAULT_ISR(SPDIF_RX_IRQHandler);
DEFINE_DEFAULT_ISR(DFSDM1_FLT0_IRQHandler);
DEFINE_DEFAULT_ISR(DFSDM1_FLT1_IRQHandler);
DEFINE_DEFAULT_ISR(DFSDM1_FLT2_IRQHandler);
DEFINE_DEFAULT_ISR(DFSDM1_FLT3_IRQHandler);
DEFINE_DEFAULT_ISR(SDMMC2_IRQHandler);
DEFINE_DEFAULT_ISR(CAN3_TX_IRQHandler);
DEFINE_DEFAULT_ISR(CAN3_RX0_IRQHandler);
DEFINE_DEFAULT_ISR(CAN3_RX1_IRQHandler);
DEFINE_DEFAULT_ISR(CAN3_SCE_IRQHandler);
DEFINE_DEFAULT_ISR(JPEG_IRQHandler);
DEFINE_DEFAULT_ISR(MDIOS_IRQHandler);


__attribute__((used,section(".isr_vector")))
const ISR_t g_pfnVectors[128] = {
    (ISR_t)(&__StackTop),
    _start,    /* Alias for _mainCRTStartup */

    /* Core Interrupts */
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* External Interrupts */
    WWDG_IRQHandler,          /* Window WatchDog              */
    PVD_IRQHandler,           /* PVD through EXTI Line detection */
    TAMP_STAMP_IRQHandler,    /* Tamper and TimeStamps through the EXTI line */
    RTC_WKUP_IRQHandler,      /* RTC Wakeup through the EXTI line */
    FLASH_IRQHandler,         /* FLASH                        */
    RCC_IRQHandler,           /* RCC                          */
    EXTI0_IRQHandler,         /* EXTI Line0                   */
    EXTI1_IRQHandler,         /* EXTI Line1                   */
    EXTI2_IRQHandler,         /* EXTI Line2                   */
    EXTI3_IRQHandler,         /* EXTI Line3                   */
    EXTI4_IRQHandler,         /* EXTI Line4                   */
    DMA1_Stream0_IRQHandler,  /* DMA1 Stream 0                */
    DMA1_Stream1_IRQHandler,  /* DMA1 Stream 1                */
    DMA1_Stream2_IRQHandler,  /* DMA1 Stream 2                */
    DMA1_Stream3_IRQHandler,  /* DMA1 Stream 3                */
    DMA1_Stream4_IRQHandler,  /* DMA1 Stream 4                */
    DMA1_Stream5_IRQHandler,  /* DMA1 Stream 5                */
    DMA1_Stream6_IRQHandler,  /* DMA1 Stream 6                */
    ADC_IRQHandler,           /* ADC1, ADC2 and ADC3s         */
    CAN1_TX_IRQHandler,       /* CAN1 TX                      */
    CAN1_RX0_IRQHandler,      /* CAN1 RX0                     */
    CAN1_RX1_IRQHandler,      /* CAN1 RX1                     */
    CAN1_SCE_IRQHandler,      /* CAN1 SCE                     */
    EXTI9_5_IRQHandler,       /* External Line[9:5]s          */
    TIM1_BRK_TIM9_IRQHandler, /* TIM1 Break and TIM9          */
    TIM1_UP_TIM10_IRQHandler, /* TIM1 Update and TIM10        */
    TIM1_TRG_COM_TIM11_IRQHandler, /* TIM1 Trigger and Commutation and TIM11 */
    TIM1_CC_IRQHandler,            /* TIM1 Capture Compare         */
    TIM2_IRQHandler,               /* TIM2                         */
    TIM3_IRQHandler,               /* TIM3                         */
    TIM4_IRQHandler,               /* TIM4                         */
    I2C1_EV_IRQHandler,            /* I2C1 Event                   */
    I2C1_ER_IRQHandler,            /* I2C1 Error                   */
    I2C2_EV_IRQHandler,            /* I2C2 Event                   */
    I2C2_ER_IRQHandler,            /* I2C2 Error                   */
    SPI1_IRQHandler,               /* SPI1                         */
    SPI2_IRQHandler,               /* SPI2                         */
    USART1_IRQHandler,             /* USART1                       */
    USART2_IRQHandler,             /* USART2                       */
    USART3_IRQHandler,             /* USART3                       */
    EXTI15_10_IRQHandler,          /* External Line[15:10]s        */
    RTC_Alarm_IRQHandler,          /* RTC Alarm (A and B) through EXTI Line */
    OTG_FS_WKUP_IRQHandler,        /* USB OTG FS Wakeup through EXTI line */
    TIM8_BRK_TIM12_IRQHandler,     /* TIM8 Break and TIM12         */
    TIM8_UP_TIM13_IRQHandler,      /* TIM8 Update and TIM13        */
    TIM8_TRG_COM_TIM14_IRQHandler, /* TIM8 Trigger and Commutation and TIM14 */
    TIM8_CC_IRQHandler,            /* TIM8 Capture Compare         */
    DMA1_Stream7_IRQHandler,       /* DMA1 Stream7                 */
    FMC_IRQHandler,                /* FMC                          */
    SDMMC1_IRQHandler,             /* SDMMC1                       */
    TIM5_IRQHandler,               /* TIM5                         */
    SPI3_IRQHandler,               /* SPI3                         */
    UART4_IRQHandler,              /* UART4                        */
    UART5_IRQHandler,              /* UART5                        */
    TIM6_DAC_IRQHandler,           /* TIM6 and DAC1&2 underrun errors */
    TIM7_IRQHandler,               /* TIM7                         */
    DMA2_Stream0_IRQHandler,       /* DMA2 Stream 0                */
    DMA2_Stream1_IRQHandler,       /* DMA2 Stream 1                */
    DMA2_Stream2_IRQHandler,       /* DMA2 Stream 2                */
    DMA2_Stream3_IRQHandler,       /* DMA2 Stream 3                */
    DMA2_Stream4_IRQHandler,       /* DMA2 Stream 4                */
    ETH_IRQHandler,                /* Ethernet                     */
    ETH_WKUP_IRQHandler,           /* Ethernet Wakeup through EXTI line */
    CAN2_TX_IRQHandler,            /* CAN2 TX                      */
    CAN2_RX0_IRQHandler,           /* CAN2 RX0                     */
    CAN2_RX1_IRQHandler,           /* CAN2 RX1                     */
    CAN2_SCE_IRQHandler,           /* CAN2 SCE                     */
    OTG_FS_IRQHandler,             /* USB OTG FS                   */
    DMA2_Stream5_IRQHandler,       /* DMA2 Stream 5                */
    DMA2_Stream6_IRQHandler,       /* DMA2 Stream 6                */
    DMA2_Stream7_IRQHandler,       /* DMA2 Stream 7                */
    USART6_IRQHandler,             /* USART6                       */
    I2C3_EV_IRQHandler,            /* I2C3 event                   */
    I2C3_ER_IRQHandler,            /* I2C3 error                   */
    OTG_HS_EP1_OUT_IRQHandler,     /* USB OTG HS End Point 1 Out   */
    OTG_HS_EP1_IN_IRQHandler,      /* USB OTG HS End Point 1 In    */
    OTG_HS_WKUP_IRQHandler,        /* USB OTG HS Wakeup through EXTI */
    OTG_HS_IRQHandler,             /* USB OTG HS                   */
    DCMI_IRQHandler,               /* DCMI                         */
    0,                             /* Reserved                     */
    RNG_IRQHandler,                /* RNG                          */
    FPU_IRQHandler,                /* FPU                          */
    UART7_IRQHandler,              /* UART7                        */
    UART8_IRQHandler,              /* UART8                        */
    SPI4_IRQHandler,               /* SPI4                         */
    SPI5_IRQHandler,               /* SPI5                         */
    SPI6_IRQHandler,               /* SPI6                         */
    SAI1_IRQHandler,               /* SAI1                         */
    LTDC_IRQHandler,               /* LTDC                         */
    LTDC_ER_IRQHandler,            /* LTDC error                   */
    DMA2D_IRQHandler,              /* DMA2D                        */
    SAI2_IRQHandler,               /* SAI2                         */
    QUADSPI_IRQHandler,            /* QUADSPI                      */
    LPTIM1_IRQHandler,             /* LPTIM1                       */
    CEC_IRQHandler,                /* HDMI_CEC                     */
    I2C4_EV_IRQHandler,            /* I2C4 Event                   */
    I2C4_ER_IRQHandler,            /* I2C4 Error                   */
    SPDIF_RX_IRQHandler,           /* SPDIF_RX                     */
    0,                             /* Reserved                     */
    DFSDM1_FLT0_IRQHandler,        /* DFSDM1 Filter 0 global Interrupt */
    DFSDM1_FLT1_IRQHandler,        /* DFSDM1 Filter 1 global Interrupt */
    DFSDM1_FLT2_IRQHandler,        /* DFSDM1 Filter 2 global Interrupt */
    DFSDM1_FLT3_IRQHandler,        /* DFSDM1 Filter 3 global Interrupt */
    SDMMC2_IRQHandler,             /* SDMMC2                       */
    CAN3_TX_IRQHandler,            /* CAN3 TX                      */
    CAN3_RX0_IRQHandler,           /* CAN3 RX0                     */
    CAN3_RX1_IRQHandler,           /* CAN3 RX1                     */
    CAN3_SCE_IRQHandler,           /* CAN3 SCE                     */
    JPEG_IRQHandler,               /* JPEG                         */
    MDIOS_IRQHandler,              /* MDIOS                        */
};
