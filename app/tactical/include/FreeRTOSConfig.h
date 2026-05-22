/*******************************************************************************
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS configuration for STM32H563xx @ 250 MHz (Cortex-M33).
 ******************************************************************************/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* SystemCoreClock is updated by HAL_RCC_ClockConfig — use it as the clock
 * source so the tick period stays correct across clock changes. */
#ifndef __ASSEMBLER__
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

/* ---------------------------------------------------------------------------
 * Scheduler
 * -------------------------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1   /* Cortex-M33 CLZ support */
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_TIME_SLICING                  1

/* ---------------------------------------------------------------------------
 * Memory
 * -------------------------------------------------------------------------*/
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 64U * 1024U ) )

/* ---------------------------------------------------------------------------
 * Hooks
 * -------------------------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     1   /* calls HAL_IncTick() */
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* ---------------------------------------------------------------------------
 * Debug / safety
 * -------------------------------------------------------------------------*/
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configASSERT( x )  if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* ---------------------------------------------------------------------------
 * Software timers
 * -------------------------------------------------------------------------*/
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* ---------------------------------------------------------------------------
 * Cortex-M33 / ARMv8-M hardware features (required by ARM_CM33_NTZ port)
 * STM32H563 has an FPU (FPv5-SP-D16) and MPU but no TrustZone in this build.
 * -------------------------------------------------------------------------*/
#define configENABLE_FPU         1
#define configENABLE_MPU         0
#define configENABLE_TRUSTZONE   0

/* ---------------------------------------------------------------------------
 * Cortex-M33 interrupt priority.
 * STM32H5 uses 4 priority bits (NVIC_PRIORITYGROUP_4).
 * ISRs that call FreeRTOS FromISR APIs must use a priority >= MAX_SYSCALL.
 * -------------------------------------------------------------------------*/
#define configPRIO_BITS                                 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0xFU
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5U
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY        << ( 8U - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   << ( 8U - configPRIO_BITS ) )

/* ---------------------------------------------------------------------------
 * Map FreeRTOS port handlers to CMSIS-standard names so the port assembler
 * stubs are visible in the vector table without modifying startup code.
 * stm32h5xx_it_tactical.c must NOT define these three handlers.
 * -------------------------------------------------------------------------*/
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* ---------------------------------------------------------------------------
 * Optional API includes
 * -------------------------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1

#endif /* FREERTOS_CONFIG_H */
