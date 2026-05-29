/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  WAIT_POWER_ON_CONFIRM = 0,  /* 开机确认阶段：第一次短按由硬件完成，软件等待松开后再检测第二次长按。 */
  NORMAL_RUN,                 /* 正常运行阶段：PA9 保持高电平，系统已经自保持供电。 */
  WAIT_SHUTDOWN_CONFIRM,      /* 关机确认阶段：已检测到有效短按，等待窗口内的第二次长按。 */
  POWER_OFF                   /* 断电阶段：PA9 已释放或即将释放，程序通常会随电源关闭停止运行。 */
} PowerState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 按键消抖时间：过小容易受抖动误触发，过大会让按键响应变迟钝。 */
#define KEY_DEBOUNCE_MS              30U
/* 有效短按的最短时间：过滤毛刺或极短误碰。 */
#define SHORT_PRESS_MIN_MS           50U
/* 有效短按的最长时间：超过该时间视为长按/无效按住，直接长按不能触发关机。 */
#define SHORT_PRESS_MAX_MS           500U

/* 短按释放后等待第二次按下的时间窗口：超时则本次“短按 + 长按”组合失效。 */
#define SECOND_PRESS_WINDOW_MS       2000U

/* 开机确认长按时间：第二次按下必须持续达到该时间，才确认开机成功。 */
#define POWER_ON_LONG_PRESS_MS       1000U
/* 关机确认长按时间：第二次按下必须持续达到该时间，才执行关机。 */
#define SHUTDOWN_LONG_PRESS_MS       1200U

/* 开机总确认超时：上电后若一直未完成释放和第二次长按，则释放 PA9 让系统断电。 */
#define POWER_ON_CONFIRM_TIMEOUT_MS  4500U

/* PA15 is pulled up: released is high and a pressed key reads low. */
/* PA15 为上拉输入：松开为高电平，按下为低电平；以后更换硬件电平时集中改这里。 */
#define KEY_PRESSED_LEVEL            GPIO_PIN_RESET
#define KEY_RELEASED_LEVEL           GPIO_PIN_SET

/* PB5 debug LED is active low. */
/* PB5 调试 LED 为低电平点亮、高电平熄灭；它只用于观察流程，不参与真正电源控制。 */
#define LED_ON_LEVEL                 GPIO_PIN_RESET
#define LED_OFF_LEVEL                GPIO_PIN_SET
/* 等待确认阶段的 LED 闪烁周期，用于提示当前处于开机/关机确认流程。 */
#define DEBUG_LED_BLINK_MS           250U
#define Key_LED_BLINK_MS             250U

#define Key_LED_ON_LEVEL               GPIO_PIN_SET
#define Key_LED_OFF_LEVEL              GPIO_PIN_RESET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 电源控制主状态：开机确认、正常运行、关机确认、断电。 */
static PowerState_t power_state;

/* 按键消抖相关变量：raw 为瞬时采样值，stable 为消抖后的稳定电平。 */
static GPIO_PinState key_raw_level;
static GPIO_PinState key_stable_level;
static uint32_t key_raw_change_tick;

/* 按键事件时间：记录按下起点和最近一次释放时的持续时间。 */
static uint32_t key_press_start_tick;
static uint32_t key_last_press_duration_ms;

/* 组合按键时间窗口：分别用于开机总超时和第二次按下窗口。 */
static uint32_t power_on_attempt_tick;
static uint32_t confirm_window_start_tick;

/* 单次循环内的边沿事件：消抖确认后置位，任务处理完后下次 Key_Update 会清零。 */
static uint8_t key_press_event;
static uint8_t key_release_event;

/* 开机/关机确认流程的阶段标志。 */
static uint8_t power_on_first_release_seen;
static uint8_t second_press_started;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Power_Hold_Early_Init(void);
static void Power_Hold_On(void);
static void Power_Hold_Off(void);
static void MOS_Driven_On(void);
static void MOS_Driven_Off(void);
static void Debug_LED_Set(uint8_t led_on);
static void Key_LED_Set(uint8_t led_on);
static void Key_LED_Update(uint32_t now);
static uint8_t Time_Elapsed(uint32_t now, uint32_t start, uint32_t period);
static uint8_t Key_Is_Pressed(void);
static void Key_Update(uint32_t now);
static void Power_Key_Control_Init(void);
static void Power_Key_Control_Task(void);
static void Cancel_Power_On(void);
static void Shutdown_Prepare(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  拉高 PA9，保持系统自保持供电。
  * @note   PA9 输出高电平时三极管导通，按键松开后系统仍能继续供电。
  */
static void Power_Hold_On(void)
{
  HAL_GPIO_WritePin(power_control_GPIO_Port, power_control_Pin, GPIO_PIN_SET);
}

/**
  * @brief  拉低 PA9，释放电源保持。
  * @note   PA9 输出低电平时三极管关断，硬件电源会关闭，系统即将断电。
  */
static void Power_Hold_Off(void)
{
  HAL_GPIO_WritePin(power_control_GPIO_Port, power_control_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  拉高 Mos_drived_Pin，驱动 MOS 管导通。
  * @note   Mos_drived_Pin 输出高电平时 MOS 管导通，允许电流通过。
  */
static void MOS_Driven_On(void)
{
  HAL_GPIO_WritePin(Mos_drived_GPIO_Port, Mos_drived_Pin, GPIO_PIN_SET);
}

/**
  * @brief  拉低 Mos_drived_Pin，关闭 MOS 管。
  * @note   Mos_drived_Pin 输出低电平时 MOS 管关闭，不允许电流通过。
  */
static void MOS_Driven_Off(void)
{
  HAL_GPIO_WritePin(Mos_drived_GPIO_Port, Mos_drived_Pin, GPIO_PIN_RESET);
}


/**
  * @brief  尽早初始化 PA9 并拉高，避免启动确认前掉电。
  * @note   开机时第一次短按由硬件完成；STM32 上电后要先自保持，再等待第二次长按确认。
  */
static void Power_Hold_Early_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*
   * Drive the hold pin before ordinary peripheral initialization.  Setting the
   * output latch first avoids a low pulse while PA9 changes to output mode.
   */
  /* 先打开 GPIOA 时钟，再写 PA9 输出锁存器，减少 PA9 进入输出模式时出现低脉冲的风险。 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  Power_Hold_On();
  GPIO_InitStruct.Pin = power_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(power_control_GPIO_Port, &GPIO_InitStruct);
  Power_Hold_On();
}

/**
  * @brief  设置 PB5 调试 LED 状态。
  * @param  led_on 非 0 点亮 LED，0 熄灭 LED。
  * @note   PB5 为低电平点亮、高电平熄灭；该 LED 只用于调试组合按键状态。
  */
static void Debug_LED_Set(uint8_t led_on)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                    (led_on != 0U) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/**
  * @brief  设置 PB5 调试 LED 状态。
  * @param  led_on 非 0 点亮 LED，0 熄灭 LED。
  * @note   PB5 为低电平点亮、高电平熄灭；该 LED 只用于调试组合按键状态。
  */
static void Key_LED_Set(uint8_t led_on)
{
  HAL_GPIO_WritePin(Key_LED_GPIO_Port, Key_LED_Pin,
                    (led_on != 0U) ? Key_LED_ON_LEVEL : Key_LED_OFF_LEVEL);
}


/**
  * @brief  判断指定毫秒时间是否已经过去。
  * @note   now 通常来自 HAL_GetTick()，该计数由 SysTick 提供，单位为 ms。
  */
static uint8_t Time_Elapsed(uint32_t now, uint32_t start, uint32_t period)
{
  return (((uint32_t)(now - start)) >= period) ? 1U : 0U;
}

/**
  * @brief  读取消抖后的按键状态。
  * @retval 1 按键按下；0 按键松开。
  * @note   PA15 上拉输入，稳定电平为 KEY_PRESSED_LEVEL 时表示按键被按下。
  */
static uint8_t Key_Is_Pressed(void)
{
  return (key_stable_level == KEY_PRESSED_LEVEL) ? 1U : 0U;
}

/**
  * @brief  按键消抖和边沿事件更新。
  * @param  now 当前毫秒计时值，来自 HAL_GetTick()。
  * @note   需要在主循环中周期调用；它只生成按下/释放事件，不直接改变电源状态。
  */
static void Key_Update(uint32_t now)
{
  GPIO_PinState sampled_level;

  /* 每轮先清除边沿事件，只有本轮确认了新的稳定电平变化才重新置位。 */
  key_press_event = 0U;
  key_release_event = 0U;

  /* 读取 PA15 原始电平：上拉输入下，松开为高，按下为低。 */
  sampled_level = HAL_GPIO_ReadPin(Key_Detectd_GPIO_Port, Key_Detectd_Pin);

  if (sampled_level != key_raw_level)
  {
    /* 原始电平发生变化时只记录变化时刻，等电平稳定超过消抖时间后才承认。 */
    key_raw_level = sampled_level;
    key_raw_change_tick = now;
  }

  if ((key_stable_level != key_raw_level) &&
      (Time_Elapsed(now, key_raw_change_tick, KEY_DEBOUNCE_MS) != 0U))
  {
    /* 原始电平持续稳定超过 KEY_DEBOUNCE_MS，才更新为正式按键状态。 */
    key_stable_level = key_raw_level;

    if (Key_Is_Pressed() != 0U)
    {
      /* 确认一次按下边沿，记录按下开始时间，后续用于判断长按时长。 */
      key_press_start_tick = now;
      key_press_event = 1U;
    }
    else
    {
      /* 确认一次释放边沿，计算本次按住多久，后续用于判断是否为有效短按。 */
      key_last_press_duration_ms = (uint32_t)(now - key_press_start_tick);
      key_release_event = 1U;
    }
  }
}

/**
  * @brief  电源按键状态机初始化。
  * @note   GPIO 初始化后调用；会保持 PA9 高电平，并进入开机确认等待状态。
  */
static void Power_Key_Control_Init(void)
{
  uint32_t now = HAL_GetTick();

  /* 再次拉高 PA9，确保系统进入自保持供电；PB5 默认熄灭。 */
  Power_Hold_On();
  Debug_LED_Set(0U);

  key_raw_level = HAL_GPIO_ReadPin(Key_Detectd_GPIO_Port, Key_Detectd_Pin);
  /*
   * At startup hardware has already supplied the first short press.  Begin as
   * "pressed" so a debounced release is required before a second press counts.
   */
  /*
   * 开机时 STM32 原本断电，无法完整检测第一次短按。
   * 因此软件默认“第一次短按已经由硬件完成”，这里只等待用户松开后再做第二次长按确认。
   */
  key_stable_level = KEY_PRESSED_LEVEL;
  key_raw_change_tick = now;
  key_press_start_tick = now;
  key_last_press_duration_ms = 0U;
  key_press_event = 0U;
  key_release_event = 0U;
  power_on_first_release_seen = 0U;
  second_press_started = 0U;
  power_on_attempt_tick = now;
  confirm_window_start_tick = now;
  /* 初始进入开机确认状态；若后续确认失败，会拉低 PA9 自动断电。 */
  power_state = WAIT_POWER_ON_CONFIRM;
}

/**
  * @brief  取消开机确认并释放电源保持。
  * @note   开机确认超时或第二次长按不足时调用，会拉低 PA9，系统即将断电。
  */
static void Cancel_Power_On(void)
{
  /* No valid startup confirmation: release the temporary power latch. */
  /* 没有得到有效开机确认，熄灭调试 LED 后释放 PA9。 */
  Debug_LED_Set(0U);
  power_state = POWER_OFF;
  Power_Hold_Off();
}

/**
  * @brief  关机前准备入口。
  * @note   当前为空实现；以后可在这里保存数据、关闭外设，再由状态机拉低 PA9。
  */
static void Shutdown_Prepare(void)
{
  /* Reserved for saving data or stopping peripherals before power is removed. */
}

/**
  * @brief  根据当前电源状态刷新 PB5 调试 LED。
  * @note   等待确认时闪烁，正常运行时常亮，断电流程中熄灭。
  */
static void Key_LED_Update(uint32_t now)
{
  switch (power_state)
  {
    case WAIT_POWER_ON_CONFIRM:
    case WAIT_SHUTDOWN_CONFIRM:
      /* 开机/关机确认阶段闪烁，提示用户当前正在等待第二次长按。 */
      // Debug_LED_Set((((now / DEBUG_LED_BLINK_MS) & 1U) != 0U) ? 1U : 0U);
      Key_LED_Set((((now / Key_LED_BLINK_MS) & 1U)) != 0U ? 1U : 0U);
      break;

    case NORMAL_RUN:
      /* 正常运行阶段常亮，表示 PA9 已保持高电平，系统供电有效。 */
      // Debug_LED_Set(1U);
      Key_LED_Set(1U);
      break;

    case POWER_OFF:
    default:
      /* 断电阶段熄灭；PB5 不参与电源控制，只做状态提示。 */
      // Debug_LED_Set(0U);
      Key_LED_Set(0U);
      break;
  }
}

/**
  * @brief  电源按键主状态机任务。
  * @note   必须在 while(1) 中周期调用；所有短按、长按、超时判断都在这里完成。
  */
static void Power_Key_Control_Task(void)
{
  /* HAL_GetTick() 基于 SysTick，提供毫秒级计时，用于消抖和长按/超时判断。 */
  uint32_t now = HAL_GetTick();

  Key_Update(now);

  switch (power_state)
  {
    case WAIT_POWER_ON_CONFIRM:
      /* 开机确认：第一次短按由硬件完成，软件只负责等待松开和第二次长按。 */
      /*
       * The hardware-created startup attempt has one total deadline.  Its
       * first stable release arms the time window for the confirming hold.
       */
      if (Time_Elapsed(now, power_on_attempt_tick,
                       POWER_ON_CONFIRM_TIMEOUT_MS) != 0U)
      {
        /* 开机确认总超时：用户没有完成后续长按，释放 PA9 让系统断电。 */
        Cancel_Power_On();
        break;
      }

      if (power_on_first_release_seen == 0U)
      {
        /* 必须先等用户松开硬件触发的第一次按键，避免把同一次按住误判为第二次长按。 */
        if (Key_Is_Pressed() == 0U)
        {
          /* 第一次按键已经松开，从这里开始计算第二次按下的有效时间窗口。 */
          power_on_first_release_seen = 1U;
          confirm_window_start_tick = now;
          second_press_started = 0U;
        }
        break;
      }

      if (second_press_started == 0U)
      {
        /* 第二次按下必须发生在窗口内，超时说明本次开机确认无效。 */
        if (Time_Elapsed(now, confirm_window_start_tick,
                         SECOND_PRESS_WINDOW_MS) != 0U)
        {
          Cancel_Power_On();
          break;
        }

        if (key_press_event != 0U)
        {
          /* 捕获第二次按下边沿，后续开始检测它是否能持续到开机长按时间。 */
          second_press_started = 1U;
        }
        break;
      }

      if (key_release_event != 0U)
      {
        /*
         * An incomplete confirming hold is treated as a failed startup.
         * A new attempt must begin from hardware after power is released.
         */
        /* 第二次长按时间不够就松开，本次开机确认失败，直接释放 PA9。 */
        Cancel_Power_On();
        break;
      }

      if ((Key_Is_Pressed() != 0U) &&
          (Time_Elapsed(now, key_press_start_tick,
                        POWER_ON_LONG_PRESS_MS) != 0U))
      {
        /* 第二次按下持续达到开机确认长按时间，开机成功并进入正常运行。 */
        second_press_started = 0U;

        //在程序进入稳定运行状态，先把MOS通道打开
        MOS_Driven_On();

        power_state = NORMAL_RUN;
      }
      break;

    case NORMAL_RUN:
      /* 正常运行：只有“有效短按释放”才能进入关机确认；直接长按会被忽略。 */
      /*
       * Shutdown can only be armed by a completed short press.  A direct
       * long press ends with a duration above SHORT_PRESS_MAX_MS and is ignored.
       */
      if ((key_release_event != 0U) &&
          (key_last_press_duration_ms >= SHORT_PRESS_MIN_MS) &&
          (key_last_press_duration_ms <= SHORT_PRESS_MAX_MS))
      {
        /* 有效短按：按下时间处于短按范围内，开始等待第二次关机长按。 */
        confirm_window_start_tick = now;
        second_press_started = 0U;
        power_state = WAIT_SHUTDOWN_CONFIRM;
      }
      break;

    case WAIT_SHUTDOWN_CONFIRM:
      /* 关机确认：短按已完成，等待窗口内第二次按下并保持到关机长按时间。 */
      if (second_press_started == 0U)
      {
        /* 短按后超过窗口仍没有第二次按下，取消本次关机组合，回到正常运行。 */
        if (Time_Elapsed(now, confirm_window_start_tick,
                         SECOND_PRESS_WINDOW_MS) != 0U)
        {
          power_state = NORMAL_RUN;
          break;
        }

        if (key_press_event != 0U)
        {
          /* 捕获第二次按下边沿，后续开始检查是否达到关机长按时间。 */
          second_press_started = 1U;
        }
        break;
      }

      if (key_release_event != 0U)
      {
        /* Incomplete second press cancels shutdown; require a fresh short press. */
        /* 第二次按下时间不够就松开，不允许关机，必须重新短按再确认。 */
        second_press_started = 0U;
        power_state = NORMAL_RUN;
        break;
      }

      if ((Key_Is_Pressed() != 0U) &&
          (Time_Elapsed(now, key_press_start_tick,
                        SHUTDOWN_LONG_PRESS_MS) != 0U))
      {
        /* 第二次按下达到关机长按时间：先做关机准备，再拉低 PA9 断电。 */
        Shutdown_Prepare();
        Debug_LED_Set(0U);

        //在单片机断电之前，先把MOS通道断开
        MOS_Driven_Off();

        power_state = POWER_OFF;
        Power_Hold_Off();
      }
      break;

    case POWER_OFF:
    default:
      /* POWER_OFF 下不再做新的按键流程，等待硬件电源真正关闭。 */
      break;
  }

  Key_LED_Update(now);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* HAL_Init() 会启动 SysTick；后续 HAL_GetTick() 用它做毫秒级计时。 */
  /* 这里尽早拉高 PA9，避免开机按键松开后系统立刻掉电。 */
  Power_Hold_Early_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  /* GPIO 全部初始化完成后，进入开机确认状态机。 */
  Power_Key_Control_Init();

  //这两个引脚是临时测试功能使用的
  //这个工程是GitHub下拉的Short_Long_Switch
  // HAL_GPIO_WritePin(GPIOB, Key_LED_Pin, GPIO_PIN_SET);  /* 上电后先点亮 PB5，表示系统已开始运行；后续状态由 Power_Key_Control_Task 刷新。 */
  // HAL_GPIO_WritePin(GPIOB, Mos_drived_Pin, GPIO_PIN_SET);  
  // /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 非阻塞周期任务：持续处理按键消抖、短按+长按组合和电源保持控制。 */
    Power_Key_Control_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 当前工程使用内部 HSI 8MHz，不修改系统时钟；HAL_GetTick() 依赖 SysTick 计时。 */

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
