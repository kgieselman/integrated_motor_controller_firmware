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
 * This file's job is deliberately small: bring up the clock and every
 * peripheral the drivers need, apply the NVIC priority policy, construct the
 * board-level indicator hardware, hand the interrupt callbacks to the tasks
 * that own the peripherals behind them, and start the scheduler. The robot
 * itself lives in app/tactical/tasks/ - see Tasks.hpp for the four tasks and
 * docs/tactical_architecture.md §3.1 for the schedule they implement.
 *
 * Nothing here knows what a CRSF frame or a control cycle is. Each task owns
 * its own peripherals and its own snapshots inside its translation unit, which
 * is why the callbacks below only forward.
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
#include "Led.hpp"
#include "platform/Watchdog.hpp"
#include "tasks/CommsTask.hpp"
#include "tasks/ControlTask.hpp"
#include "tasks/HeartbeatTask.hpp"
#include "tasks/Tasks.hpp"
#include "tasks/TelemetryTask.hpp"

#include <cstdint>

/* Private declarations ------------------------------------------------------*/

static void SystemClock_Config(void);
static void initPeripherals(void);
static void configureInterruptPriorities(void);

/* File-local constants ------------------------------------------------------*/

/// NVIC priority for the UART interrupts this firmware owns, and for the UART4
/// receive DMA channel. Anything calling a FreeRTOS *FromISR API must sit at
/// 5-14; the kernel cannot mask 0-4. Six is the row
/// docs/tactical_architecture.md §3.3 assigns to the CRSF path.
static constexpr uint32_t kUartIrqPriority = 6U;

/// NVIC priority for the DRV8874 nFAULT lines on EXTI3 and EXTI7. The top of
/// the kernel-maskable band, per docs/tactical_architecture.md §3.3: a motor
/// driver reporting overcurrent is the most urgent thing this board can be
/// told, and its handler latches a fault and notifies the control task.
static constexpr uint32_t kMotorFaultIrqPriority = 5U;

/// NVIC priority for the IMU data-ready line on EXTI2. §3.3 puts it below the
/// CRSF path deliberately - a late IMU sample degrades a heading estimate,
/// while a late CRSF frame delays the failsafe.
static constexpr uint32_t kImuIrqPriority = 7U;

/* Board hardware ------------------------------------------------------------*/

// Statically allocated, constructed before main(). No heap anywhere.
//
// These four are board-level rather than task-level: they are the debug
// indicators and the transducer, they belong to no mechanism, and main() needs
// two of them before any task exists (the ADC failure path below). They are
// constructed and initialised here and then bound to the heartbeat task, which
// is the only thing that drives them once the scheduler is running.
static Led    s_ledMode(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin);
static Led    s_ledLink(DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin);
static Led    s_ledFault(DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin);
static Buzzer s_buzzer(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin);

/* Entry point ---------------------------------------------------------------*/

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  initPeripherals();
  configureInterruptPriorities();

  s_ledMode.init();
  s_ledLink.init();
  s_ledFault.init();

  // Buzzer::init() returns false when the DWT cycle counter will not run, in
  // which case beep() is disabled and the robot loses its audible mode cues.
  // Not fatal, and not silent either. The control task's overrun measurement
  // has its own counter and its own self-test - see controlTaskInit().
  if (!s_buzzer.init())
  {
    s_ledFault.on();
  }

  // Start the ADC1 circular scan. Until this succeeds, Battery and Motor
  // report their readings as unavailable rather than as a plausible zero.
  if (!adcDmaStart(&hadc1))
  {
    s_ledFault.on();
  }

  // Task init, in dependency order and all of it before any task is created.
  //
  // Two of these arm an interrupt (the CRSF DMA and the console receiver), so
  // an ISR can fire from here on. Both hooks are written to tolerate arriving
  // before their task exists; see commsTaskOnUartRxEvent().
  //
  // controlTaskInit() is last because it runs the boot self-test, which is the
  // §5.1 edge SelfTest -> Fault. A failure there is not a reason to stop: it
  // latches inside SafetyMonitor, and the robot boots into Fault with the
  // reason readable on the console - which is a great deal more useful than a
  // board that sits dark in an unexplained while(1).
  if (!commsTaskInit())
  {
    s_ledFault.on();
  }

  if (!telemetryTaskInit())
  {
    s_ledFault.on();
  }

  heartbeatTaskInit(s_ledMode, s_ledLink, s_ledFault, s_buzzer);

  if (!controlTaskInit())
  {
    s_ledFault.on();
  }

  // One chirp says the board booted far enough to reach the scheduler. Called
  // before the scheduler starts, so the blocking busy-wait costs nothing.
  s_buzzer.chirp();

  if (!tasksCreateAll())
  {
    // Almost always configTOTAL_HEAP_SIZE against the stack depths in
    // Tasks.hpp. Starting the scheduler with a missing task would run a robot
    // with, say, no failsafe, so stop here instead.
    s_ledFault.on();
    Error_Handler();
  }

  // Last statement before the scheduler, deliberately. Started any earlier and
  // the whole init sequence above would have to finish inside one reload
  // period, for no benefit: nothing up to here loops, so nothing up to here can
  // wedge in the way a watchdog exists to catch. From this line on, a control
  // task that stops advancing its liveness counter resets the board in
  // Watchdog::kReloadMs - see docs/tactical_architecture.md §5.2.
  //
  // A failure here is the §5.2 stall row silently not being armed, which is
  // worth a light but not worth refusing to boot over.
  if (!watchdogStart())
  {
    s_ledFault.on();
  }

  vTaskStartScheduler();

  // Only reached if the scheduler could not start — almost always too little
  // heap for the idle or timer task.
  for (;;)
  {
  }
}

/* Interrupt callbacks -------------------------------------------------------*/

/**
 * @brief Forward a completed console byte to the telemetry task.
 *
 * @param huart UART that raised the interrupt.
 *
 * @note Runs in ISR context. The HAL dispatches one callback for every UART, so
 *       this forwards unconditionally and the task decides whether the handle
 *       is its own.
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  telemetryTaskOnRxComplete(huart);
}

/**
 * @brief Forward a UART receive event to the comms task.
 *
 * Raised by the UART4 IDLE line and by the receive DMA channel's half- and
 * full-transfer events. In circular mode @p size is the DMA write POSITION
 * measured from the start of the ring, not a count of new bytes - see the
 * receive-model note at the top of drivers/CRSFReceiver.hpp.
 *
 * @param huart UART that raised the event.
 * @param size  Current DMA write position, in bytes from the start of the ring.
 *
 * @note Runs in ISR context, and the comms task's hook calls
 *       vTaskNotifyGiveFromISR(), so both source interrupts must sit in the
 *       kernel-maskable 5-14 band. configureInterruptPriorities() puts them at
 *       kUartIrqPriority.
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
  commsTaskOnUartRxEvent(huart, size);
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
 * The CRSF receive path raises HAL_UARTEx_RxEventCallback from two different
 * interrupts, and both are set, because setting only one leaves half the path
 * outside the policy:
 *
 *  - UART4_IRQn carries the IDLE-line event, which is what marks the end of a
 *    CRSF frame. CubeMX left it at 8 — legal, but not the row §3.3 assigns.
 *  - GPDMA1_Channel1_IRQn is the UART4 receive DMA channel, and it carries the
 *    half- and full-transfer events. cubemx/Core/Src/gpdma.c both prioritises
 *    it (at 8) and enables it, so this is a re-assertion to the §3.3 row
 *    rather than a missing enable. The EnableIRQ() below is redundant today and
 *    kept so that the policy for this line reads complete in one place.
 *
 * USART1_IRQn is the console receiver. CubeMX does not configure it at all -
 * USART1 is absent from the .ioc's NVIC list - so this is the only place it is
 * prioritised or enabled, and USART1_IRQHandler() in stm32h5xx_it_tactical.c is
 * the only thing standing between HAL_UART_Receive_IT() and Default_Handler.
 *
 * The three EXTI lines are re-asserted rather than inherited. cubemx/Core/Src/
 * gpio.c already enables all three and already happens to put the motor faults
 * at 5, which is the row §3.3 wants - but that file is CubeMX-owned, a
 * regeneration can change any of it silently, and EXTI2 is at 5 there rather
 * than at the 7 §3.3 assigns. Stating the whole policy here means the band the
 * kernel depends on is readable in one place and cannot drift underneath it.
 * Enabling stays gpio.c's job; these lines set priority only, because a line
 * enabled here whose driver has not been wired yet would fire into a handler
 * with nothing behind it.
 */
static void configureInterruptPriorities(void)
{
  HAL_NVIC_SetPriority(USART1_IRQn, kUartIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_NVIC_SetPriority(UART4_IRQn, kUartIrqPriority, 0U);

  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, kUartIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, kMotorFaultIrqPriority, 0U);
  HAL_NVIC_SetPriority(EXTI7_IRQn, kMotorFaultIrqPriority, 0U);

  HAL_NVIC_SetPriority(EXTI2_IRQn, kImuIrqPriority, 0U);
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
