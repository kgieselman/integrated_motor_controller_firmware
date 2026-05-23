/*******************************************************************************
 * @file main_bringup.cpp
 * @brief Integrated Motor Controller bring-up application entry point.
 *
 * Owns main() and SystemClock_Config(). All MX_*_Init() calls are delegated
 * to the CubeMX-generated peripheral source files (gpio.c, adc.c, etc.),
 * which are compiled as part of the build but main.c is excluded — it exists
 * only as a reference for HAL calls.
 *
 * Initial foundation: blinks DEBUG_LED_0 (PC15) at 1 Hz to confirm clocks,
 * GPIO, and toolchain are all functional before any driver work begins.
 *
 * Clock configuration (from cubemx/Core/Src/main.c reference):
 *   HSE 25 MHz (bypass) → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK
 *   HSI48 + CRS → 48 MHz USB clock
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
}

#include "Console.hpp"
#include "tests/BatteryTest.hpp"
#include "tests/CrsfTest.hpp"
#include "tests/EepromTest.hpp"
#include "tests/EncoderTest.hpp"
#include "tests/ImuTest.hpp"
#include "tests/MotorTest.hpp"
#include "tests/ServoTest.hpp"
#include <cstdint>

/* Private declarations ------------------------------------------------------*/

static void SystemClock_Config(void);


/* File-local constants ------------------------------------------------------*/

/// Heartbeat toggle period (500 ms on + 500 ms off = 1 Hz blink).
static constexpr uint32_t kHeartbeatPeriodMs = 500U;

// Console instance — swap to &huart1 once the USART1 connector is populated.
static Console console(&huart4);

// Single-byte RX staging buffer for interrupt-driven receive.
static uint8_t s_rxByte;


/* Entry point ---------------------------------------------------------------*/

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();

  // Arm the first UART RX interrupt — re-armed in HAL_UART_RxCpltCallback.
  HAL_UART_Receive_IT(&huart4, &s_rxByte, 1U);

  registerImuTests(console);
  registerMotorTests(console);
  registerEncoderTests(console);
  registerBatteryTests(console);
  registerEepromTests(console);
  registerServoTests(console);
  registerCrsfTests(console);

  console.printAbout();
  console.printHelp();

  uint32_t lastToggleMs = 0U;

  while (1)
  {
    const uint32_t now = HAL_GetTick();
    if ((now - lastToggleMs) >= kHeartbeatPeriodMs)
    {
      HAL_GPIO_TogglePin(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin);
      lastToggleMs = now;
    }

    console.poll();
  }
}


/* UART RX interrupt callback -------------------------------------------------*/

/**
 * @brief Called by HAL when a UART RX interrupt fires.
 *
 * Feeds the received byte into the console and re-arms for the next byte.
 * Swap huart4 → huart1 (and &huart4 → &huart1) when the USART1 connector
 * is populated.
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == UART4)
  {
    console.feed(&s_rxByte, 1U);
    HAL_UART_Receive_IT(&huart4, &s_rxByte, 1U);
  }
}


/* Private helpers -----------------------------------------------------------*/

/**
 * @brief Fatal error handler — disables interrupts and halts.
 *
 * Called by HAL on unrecoverable errors. Defined here because main.c is
 * excluded from the build (it is a reference only).
 */
extern "C" void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}


/**
 * @brief Configure the system clock to 250 MHz via PLL1 from HSE.
 *
 * Source: cubemx/Core/Src/main.c (reference — do not compile that file).
 * HSE 25 MHz bypass → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK.
 * HSI48 enabled for USB; CRS locks it to USB SOF.
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI48
                                        | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State          = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM            = 2;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 2;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 2;
  RCC_OscInitStruct.PLL.PLLRGE         = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL      = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN       = 0;
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
