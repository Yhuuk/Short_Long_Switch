/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* PB11(Key_LED) 和 PB7(Mos_drived) 当前不参与本次短按+长按电源逻辑，保持 CubeMX 原有初始化。 */
  HAL_GPIO_WritePin(GPIOB, Key_LED_Pin|Mos_drived_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  /* PA9(power_control) 为电源保持控制脚：高电平使三极管导通，系统保持供电。 */
  HAL_GPIO_WritePin(power_control_GPIO_Port, power_control_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  /* PB5(LED) 为低电平点亮、高电平熄灭；初始化为熄灭，仅用于调试状态提示。 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  /* PB11、PB5、PB7 均按 CubeMX 配置为推挽输出；PB5 只做调试 LED。 */
  GPIO_InitStruct.Pin = Key_LED_Pin|LED_Pin|Mos_drived_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  /* PA9 推挽输出：高电平保持供电，低电平释放保持并导致系统断电。 */
  GPIO_InitStruct.Pin = power_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(power_control_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  /* PA15(Key_Detectd) 为按键输入：上拉输入，松开为高电平，按下为低电平。 */
  GPIO_InitStruct.Pin = Key_Detectd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Key_Detectd_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
