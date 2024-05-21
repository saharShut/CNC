//Led blinck

#include "grbl.h"
#include "stm32f4xx_hal.h"




void Led_GPIO_Init()
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIOInitStr;
    
    GPIOInitStr.Pin = GPIO_PIN_7;
    GPIOInitStr.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOInitStr.Speed = GPIO_SPEED_FREQ_MEDIUM;
    
    HAL_GPIO_Init(GPIOA, &GPIOInitStr);

}

void LedBlink(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}
