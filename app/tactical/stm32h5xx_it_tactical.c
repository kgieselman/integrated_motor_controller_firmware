/*******************************************************************************
 * @file stm32h5xx_it_tactical.c
 * @brief Interrupt handlers for the imc_tactical build.
 *
 * SVC_Handler and PendSV_Handler are intentionally absent —
 * they are provided by the FreeRTOS port via the #defines in FreeRTOSConfig.h:
 *   vPortSVCHandler    → SVC_Handler
 *   xPortPendSVHandler → PendSV_Handler
 *
 * SysTick_Handler is defined below. HAL_Init() starts SysTick long before the
 * scheduler does, and the FreeRTOS tick handler dereferences task lists that do
 * not exist until the first xTaskCreate(), so it must not run early. Until the
 * scheduler starts the handler only advances the HAL tick; after that it hands
 * over to xPortSysTickHandler(), whose tick hook calls HAL_IncTick() itself.
 ******************************************************************************/

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
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
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern PCD_HandleTypeDef  hpcd_USB_DRD_FS;

/* ---------------------------------------------------------------------------
 * Cortex-M33 core exception handlers
 * -------------------------------------------------------------------------*/
/* CPU fault reporting. Any fault prints the stacked registers and the fault
 * status registers on the console (USART1, polled) and then waits for the
 * watchdog (or a reset) to restart the board. Decode PC with:
 *   arm-none-eabi-addr2line -f -e build/imc_tactical.elf <PC> */
static void faultPutHex(const char* label, uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[24];
    uint32_t n = 0U;
    while (*label != '\0' && n < 12U) { buf[n++] = *label++; }
    buf[n++] = '=';
    buf[n++] = '0';
    buf[n++] = 'x';
    for (int i = 28; i >= 0; i -= 4) { buf[n++] = hex[(v >> (uint32_t)i) & 0xFU]; }
    buf[n++] = '\r';
    buf[n++] = '\n';
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100U);
}

void faultDump(uint32_t* sp)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n*** FAULT ***\r\n", 17U, 100U);
    faultPutHex("PC", sp[6]);
    faultPutHex("LR", sp[5]);
    faultPutHex("XPSR", sp[7]);
    faultPutHex("R0", sp[0]);
    faultPutHex("R1", sp[1]);
    faultPutHex("R2", sp[2]);
    faultPutHex("R3", sp[3]);
    faultPutHex("R12", sp[4]);
    faultPutHex("SP", (uint32_t)sp);
    faultPutHex("CFSR", SCB->CFSR);
    faultPutHex("HFSR", SCB->HFSR);
    faultPutHex("BFAR", SCB->BFAR);
    faultPutHex("MMFAR", SCB->MMFAR);
    __disable_irq();
    for (;;)
    {
    }
}

#define FAULT_ENTRY(name)                                   \
    __attribute__((naked)) void name(void)                  \
    {                                                       \
        __asm volatile("tst lr, #4\n"                       \
                       "ite eq\n"                           \
                       "mrseq r0, msp\n"                    \
                       "mrsne r0, psp\n"                    \
                       "b faultDump\n");                    \
    }


void NMI_Handler(void)
{
    while (1) {}
}

FAULT_ENTRY(HardFault_Handler)

FAULT_ENTRY(MemManage_Handler)

FAULT_ENTRY(BusFault_Handler)

FAULT_ENTRY(UsageFault_Handler)

/* SysTick: HAL tick only until the scheduler runs, then the FreeRTOS tick. */
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        HAL_IncTick();
    }
    else
    {
        xPortSysTickHandler();
    }
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
/*
 * Console receive, USART1. HAL_UART_Receive_IT() arms one byte at a time from
 * telemetryTaskInit(); without this vector the RXNE interrupt lands on the weak
 * Default_Handler in cubemx/startup_stm32h563xx.s, which is an infinite loop.
 * Console output is blocking transmit, so the banner prints either way and
 * nothing else reports the missing receive path.
 *
 * USART1_IRQn is not listed in cubemx/integrated_motor_controller.ioc, so this
 * handler and the matching HAL_NVIC_EnableIRQ() in main_tactical.cpp both live
 * in files this project owns and survive a CubeMX regeneration.
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}
