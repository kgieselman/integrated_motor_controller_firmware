/*******************************************************************************
 * @file main_tactical.cpp
 * @brief Integrated Motor Controller — tactical application entry point.
 *
 * Clock: HSE 25 MHz (bypass) → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK
 ******************************************************************************/

extern "C"
{
#include "stm32h5xx_hal.h"
#include "gpio.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
}

static void SystemClock_Config(void);

static void vHeartbeatTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        HAL_GPIO_TogglePin(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    xTaskCreate(vHeartbeatTask, "Heartbeat", configMINIMAL_STACK_SIZE * 2U, nullptr, 1U, nullptr);

    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) {}
}

extern "C" void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

/**
 * @brief Configure the system clock to 250 MHz via PLL1 from HSE.
 *
 * HSE 25 MHz bypass → PLL1 (M=2, N=40, P=2) → 250 MHz SYSCLK.
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
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
