/*******************************************************************************
 * @file stm32h5xx_it_tactical.c
 * @brief Interrupt handlers for the imc_tactical build.
 *
 * SVC_Handler, PendSV_Handler, and SysTick_Handler are intentionally absent —
 * they are provided by the FreeRTOS port via the #defines in FreeRTOSConfig.h:
 *   vPortSVCHandler    → SVC_Handler
 *   xPortPendSVHandler → PendSV_Handler
 *   xPortSysTickHandler→ SysTick_Handler
 *
 * HAL_IncTick() is called from vApplicationTickHook() in freertos_hooks.c.
 ******************************************************************************/

#include "main.h"
#include "stm32h5xx_it.h"

/* ---------------------------------------------------------------------------
 * DMA handles declared in the peripheral init files
 * -------------------------------------------------------------------------*/
extern DMA_NodeTypeDef   Node_GPDMA1_Channel0;
extern DMA_QListTypeDef  List_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel2;
extern DMA_QListTypeDef  List_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel1;
extern DMA_QListTypeDef  List_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern UART_HandleTypeDef huart4;
extern PCD_HandleTypeDef  hpcd_USB_DRD_FS;

/* ---------------------------------------------------------------------------
 * Cortex-M33 core exception handlers
 * -------------------------------------------------------------------------*/
void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void) {}

/* ---------------------------------------------------------------------------
 * GPIO EXTI handlers
 * -------------------------------------------------------------------------*/
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI2_IMU_INT_1_Pin);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI3_MOTOR_0_FAULT_Pin);
}

void EXTI7_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI7_MOTOR_1_FAULT_Pin);
}

void EXTI8_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI8_IMU_INT2_Pin);
}

void EXTI13_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI13_DEBUG_BUTTON_Pin);
}

/* ---------------------------------------------------------------------------
 * DMA handlers
 * -------------------------------------------------------------------------*/
void GPDMA1_Channel0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
}

void GPDMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
}

void GPDMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
}

/* ---------------------------------------------------------------------------
 * Peripheral handlers
 * -------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}
