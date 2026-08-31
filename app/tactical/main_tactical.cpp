/*******************************************************************************
 * @file main_tactical.cpp
 * @brief Integrated Motor Controller — tactical application entry point.
 *
 * Owns main(), SystemClock_Config() and Error_Handler(). All MX_*_Init()
 * calls are delegated to the CubeMX-generated peripheral sources (gpio.c,
 * adc.c, …), which are compiled into this target; cubemx/Core/Src/main.c is
 * excluded from the build and exists only as a HAL reference.
 *
 * Clock: HSE 25 MHz (bypass) → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK.
 *        HSI48 + CRS → 48 MHz USB clock.
 *
 * Current scope is the phase 0 skeleton: every peripheral the drivers need is
 * brought up, the scheduler starts, a heartbeat proves it is running, and the
 * console proves the board can talk. The control, comms, telemetry and safety
 * tasks land on top of this.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

extern "C"
{
#include "stm32h5xx_hal.h"
#include "adc.h"
#include "gpdma.h"
#include "gpio.h"
#include "i2c.h"
#include "icache.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
}

#include "AdcDma.hpp"
#include "Buzzer.hpp"
#include "Console.hpp"
#include "Led.hpp"
#include "platform/RobotContext.hpp"
#include "platform/Snapshot.hpp"

#include <cstdint>

/* Private declarations ------------------------------------------------------*/

static void SystemClock_Config(void);
static void initPeripherals(void);
static void configureInterruptPriorities(void);

/* File-local constants ------------------------------------------------------*/

/// Heartbeat toggle period. 500 ms on + 500 ms off = a 1 Hz blink.
static constexpr uint32_t kHeartbeatPeriodMs = 500U;

/// Console poll period. Fast enough to feel responsive at a terminal.
static constexpr uint32_t kConsolePollMs = 10U;

/// Task priorities. configMAX_PRIORITIES is 7; the FreeRTOS timer daemon holds
/// 6. Application tasks live at 1–5. See docs/tactical_architecture.md §3.1.
static constexpr UBaseType_t kHeartbeatPriority = 1U;
static constexpr UBaseType_t kConsolePriority   = 2U;

/// NVIC priority for peripheral ISRs. Anything calling a FreeRTOS *FromISR API
/// must sit at 5–14; the kernel cannot mask 0–4.
static constexpr uint32_t kUartIrqPriority = 6U;

/* Driver instances ----------------------------------------------------------*/

// Statically allocated, constructed before main(). No heap anywhere.
static Led    s_ledHeartbeat(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin);
static Led    s_ledLink(DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin);
static Led    s_ledFault(DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin);
static Buzzer s_buzzer(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin);

// Console transport is USART1 on the growth header. This is a workaround for a
// connector part shortage on Rev A — the intended transport is USB-CDC, and
// swapping it back is a change to this one line plus the RX wiring below.
static Console s_console(&huart1);

/// Single-byte RX staging buffer for interrupt-driven console receive.
static uint8_t s_rxByte;

/* Tasks ---------------------------------------------------------------------*/

/**
 * @brief Blink DEBUG_LED_0 to prove the scheduler is running.
 *
 * Becomes the mode indicator once the control task exists: the blink pattern
 * will encode Disabled / Teleop / Auto / Fault.
 *
 * @param pvParameters Unused.
 */
static void vHeartbeatTask(void* pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    s_ledHeartbeat.toggle();
    vTaskDelay(pdMS_TO_TICKS(kHeartbeatPeriodMs));
  }
}

/**
 * @brief Drain complete console lines and dispatch them.
 *
 * @param pvParameters Unused.
 *
 * @todo Console::feed() runs in ISR context while poll() runs here, and the
 *       line buffer is unprotected between them (issue F3). Guard the shared
 *       state before the console is trusted for anything but diagnostics.
 */
static void vConsoleTask(void* pvParameters)
{
  (void)pvParameters;

  s_console.printAbout();
  s_console.printHelp();

  for (;;)
  {
    s_console.poll();
    vTaskDelay(pdMS_TO_TICKS(kConsolePollMs));
  }
}

/* Entry point ---------------------------------------------------------------*/

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  initPeripherals();
  configureInterruptPriorities();

  s_ledHeartbeat.init();
  s_ledLink.init();
  s_ledFault.init();
  s_buzzer.init();

  // Start the ADC1 circular scan. Until this succeeds, Battery and Motor
  // report their readings as unavailable rather than as a plausible zero.
  if (!adcDmaStart(&hadc1))
  {
    s_ledFault.on();
  }

  // Arm the first console RX interrupt — re-armed in HAL_UART_RxCpltCallback.
  HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);

  // One chirp says the board booted far enough to reach the scheduler. Called
  // before the scheduler starts, so the blocking busy-wait costs nothing.
  s_buzzer.chirp();

  xTaskCreate(vHeartbeatTask, "Heartbeat", configMINIMAL_STACK_SIZE * 2U,
              nullptr, kHeartbeatPriority, nullptr);

  xTaskCreate(vConsoleTask, "Console", configMINIMAL_STACK_SIZE * 4U,
              nullptr, kConsolePriority, nullptr);

  vTaskStartScheduler();

  // Only reached if the scheduler could not start — almost always too little
  // heap for the idle or timer task.
  for (;;)
  {
  }
}

/* Interrupt callbacks -------------------------------------------------------*/

/**
 * @brief Feed a received console byte and re-arm the receiver.
 *
 * @param huart UART that raised the interrupt.
 *
 * @note Runs in ISR context. Calls no FreeRTOS API, so its NVIC priority is
 *       unconstrained by the kernel — it is nonetheless set to
 *       kUartIrqPriority so that the policy holds uniformly.
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART1)
  {
    s_console.feed(&s_rxByte, 1U);
    HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
  }
}

/* Private helpers -----------------------------------------------------------*/

/**
 * @brief Bring up every peripheral the driver layer depends on.
 *
 * Order is carried over verbatim from the validated bring-up firmware. GPDMA
 * must precede any peripheral that uses DMA; ICACHE is enabled early because
 * it affects instruction fetch timing everywhere.
 */
static void initPeripherals(void)
{
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();      // Growth header — no driver uses it yet.
  MX_TIM2_Init();      // Servo / ESC PWM
  MX_TIM3_Init();      // Motor PWM
  MX_TIM4_Init();      // Encoder 0 input capture
  MX_TIM8_Init();      // Encoder 1 input capture
  MX_UART4_Init();     // CRSF / ELRS receiver
  MX_USART1_UART_Init(); // Console
  MX_USB_PCD_Init();   // Not used yet — see the console transport note above.
}

/**
 * @brief Apply the NVIC priority policy to peripherals CubeMX configured.
 *
 * CubeMX assigns its own priorities during MX_*_Init(). Any ISR that will call
 * a FreeRTOS *FromISR API must sit at 5–14, so the ones this firmware owns are
 * re-asserted here, after init and before the scheduler starts.
 *
 * @todo Extend to the CRSF UART4 DMA, the motor fault EXTI lines and the IMU
 *       INT line as those drivers are wired in.
 */
static void configureInterruptPriorities(void)
{
  HAL_NVIC_SetPriority(USART1_IRQn, kUartIrqPriority, 0U);
}

/**
 * @brief Fatal error handler — disables interrupts and halts.
 *
 * Called by the HAL on unrecoverable errors. Defined here because
 * cubemx/Core/Src/main.c is excluded from the build.
 */
extern "C" void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

/**
 * @brief Configure the system clock to 250 MHz via PLL1 from HSE.
 *
 * Source: cubemx/Core/Src/main.c (reference — that file is not compiled).
 * HSE 25 MHz bypass → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK.
 * HSI48 enabled for USB; CRS locks it to USB SOF.
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
                                   | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 2;
  RCC_OscInitStruct.PLL.PLLN       = 40;
  RCC_OscInitStruct.PLL.PLLP       = 2;
  RCC_OscInitStruct.PLL.PLLQ       = 4;
  RCC_OscInitStruct.PLL.PLLR       = 2;
  RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN   = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* EOF -----------------------------------------------------------------------*/
