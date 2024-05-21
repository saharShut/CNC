#include "clock.h"
#include "stm32f4xx_hal.h"


static void Error_Handler()
{
  while(1)
  {
    ;
  }
}

void SysClockInit()
{
  RCC_OscInitTypeDef RCC_OscInitStr = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStr = {0};
  
  /// Configure the main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStr.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStr.HSEState = RCC_HSE_ON;
  RCC_OscInitStr.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStr.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStr.PLL.PLLM = 8;
  RCC_OscInitStr.PLL.PLLN = 336;
  RCC_OscInitStr.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStr.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStr) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStr.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStr.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStr.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStr.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStr.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStr, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void SysClockDeinit()
{
  HAL_RCC_DeInit();
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


