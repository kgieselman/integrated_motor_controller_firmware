/*******************************************************************************
 * @file freertos_hooks.c
 * @brief FreeRTOS application hooks and static idle/timer task memory.
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h5xx_hal.h"

/* Called from the FreeRTOS SysTick handler each tick.
 * Keeps HAL_GetTick() / HAL_Delay() functional under the RTOS. */
void vApplicationTickHook(void)
{
    HAL_IncTick();
}

void vApplicationIdleHook(void)
{
    __WFI();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

/* ---------------------------------------------------------------------------
 * Static memory for the idle and timer-daemon tasks.
 * Required because configSUPPORT_STATIC_ALLOCATION = 1.
 * -------------------------------------------------------------------------*/
static StaticTask_t xIdleTaskTCB;
static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t          **ppxIdleTaskTCBBuffer,
                                   StackType_t           **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *puxIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB;
static StackType_t  uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t          **ppxTimerTaskTCBBuffer,
                                    StackType_t           **ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *puxTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *puxTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}
