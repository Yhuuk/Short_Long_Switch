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
  WAIT_POWER_ON_CONFIRM = 0,
  NORMAL_RUN,
  WAIT_SHUTDOWN_CONFIRM,
  POWER_OFF
} PowerState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_DEBOUNCE_MS              30U
#define SHORT_PRESS_MIN_MS           50U
#define SHORT_PRESS_MAX_MS           500U

#define SECOND_PRESS_WINDOW_MS       2000U

#define POWER_ON_LONG_PRESS_MS       1500U
#define SHUTDOWN_LONG_PRESS_MS       1800U

#define POWER_ON_CONFIRM_TIMEOUT_MS  4500U

/* PA15 is pulled up: released is high and a pressed key reads low. */
#define KEY_PRESSED_LEVEL            GPIO_PIN_RESET
#define KEY_RELEASED_LEVEL           GPIO_PIN_SET

/* PB5 debug LED is active low. */
#define LED_ON_LEVEL                 GPIO_PIN_RESET
#define LED_OFF_LEVEL                GPIO_PIN_SET
#define DEBUG_LED_BLINK_MS           250U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static PowerState_t power_state;
static GPIO_PinState key_raw_level;
static GPIO_PinState key_stable_level;
static uint32_t key_raw_change_tick;
static uint32_t key_press_start_tick;
static uint32_t key_last_press_duration_ms;
static uint32_t power_on_attempt_tick;
static uint32_t confirm_window_start_tick;
static uint8_t key_press_event;
static uint8_t key_release_event;
static uint8_t power_on_first_release_seen;
static uint8_t second_press_started;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Power_Hold_Early_Init(void);
static void Power_Hold_On(void);
static void Power_Hold_Off(void);
static void Debug_LED_Set(uint8_t led_on);
static void Debug_LED_Update(uint32_t now);
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
static void Power_Hold_On(void)
{
  HAL_GPIO_WritePin(power_control_GPIO_Port, power_control_Pin, GPIO_PIN_SET);
}

static void Power_Hold_Off(void)
{
  HAL_GPIO_WritePin(power_control_GPIO_Port, power_control_Pin, GPIO_PIN_RESET);
}

static void Power_Hold_Early_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*
   * Drive the hold pin before ordinary peripheral initialization.  Setting the
   * output latch first avoids a low pulse while PA9 changes to output mode.
   */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  Power_Hold_On();
  GPIO_InitStruct.Pin = power_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(power_control_GPIO_Port, &GPIO_InitStruct);
  Power_Hold_On();
}

static void Debug_LED_Set(uint8_t led_on)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                    (led_on != 0U) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

static uint8_t Time_Elapsed(uint32_t now, uint32_t start, uint32_t period)
{
  return (((uint32_t)(now - start)) >= period) ? 1U : 0U;
}

static uint8_t Key_Is_Pressed(void)
{
  return (key_stable_level == KEY_PRESSED_LEVEL) ? 1U : 0U;
}

static void Key_Update(uint32_t now)
{
  GPIO_PinState sampled_level;

  key_press_event = 0U;
  key_release_event = 0U;
  sampled_level = HAL_GPIO_ReadPin(Key_Detectd_GPIO_Port, Key_Detectd_Pin);

  if (sampled_level != key_raw_level)
  {
    key_raw_level = sampled_level;
    key_raw_change_tick = now;
  }

  if ((key_stable_level != key_raw_level) &&
      (Time_Elapsed(now, key_raw_change_tick, KEY_DEBOUNCE_MS) != 0U))
  {
    key_stable_level = key_raw_level;

    if (Key_Is_Pressed() != 0U)
    {
      key_press_start_tick = now;
      key_press_event = 1U;
    }
    else
    {
      key_last_press_duration_ms = (uint32_t)(now - key_press_start_tick);
      key_release_event = 1U;
    }
  }
}

static void Power_Key_Control_Init(void)
{
  uint32_t now = HAL_GetTick();

  Power_Hold_On();
  Debug_LED_Set(0U);

  key_raw_level = HAL_GPIO_ReadPin(Key_Detectd_GPIO_Port, Key_Detectd_Pin);
  /*
   * At startup hardware has already supplied the first short press.  Begin as
   * "pressed" so a debounced release is required before a second press counts.
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
  power_state = WAIT_POWER_ON_CONFIRM;
}

static void Cancel_Power_On(void)
{
  /* No valid startup confirmation: release the temporary power latch. */
  Debug_LED_Set(0U);
  power_state = POWER_OFF;
  Power_Hold_Off();
}

static void Shutdown_Prepare(void)
{
  /* Reserved for saving data or stopping peripherals before power is removed. */
}

static void Debug_LED_Update(uint32_t now)
{
  switch (power_state)
  {
    case WAIT_POWER_ON_CONFIRM:
    case WAIT_SHUTDOWN_CONFIRM:
      Debug_LED_Set((((now / DEBUG_LED_BLINK_MS) & 1U) != 0U) ? 1U : 0U);
      break;

    case NORMAL_RUN:
      Debug_LED_Set(1U);
      break;

    case POWER_OFF:
    default:
      Debug_LED_Set(0U);
      break;
  }
}

static void Power_Key_Control_Task(void)
{
  uint32_t now = HAL_GetTick();

  Key_Update(now);

  switch (power_state)
  {
    case WAIT_POWER_ON_CONFIRM:
      /*
       * The hardware-created startup attempt has one total deadline.  Its
       * first stable release arms the time window for the confirming hold.
       */
      if (Time_Elapsed(now, power_on_attempt_tick,
                       POWER_ON_CONFIRM_TIMEOUT_MS) != 0U)
      {
        Cancel_Power_On();
        break;
      }

      if (power_on_first_release_seen == 0U)
      {
        if (Key_Is_Pressed() == 0U)
        {
          power_on_first_release_seen = 1U;
          confirm_window_start_tick = now;
          second_press_started = 0U;
        }
        break;
      }

      if (second_press_started == 0U)
      {
        if (Time_Elapsed(now, confirm_window_start_tick,
                         SECOND_PRESS_WINDOW_MS) != 0U)
        {
          Cancel_Power_On();
          break;
        }

        if (key_press_event != 0U)
        {
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
        Cancel_Power_On();
        break;
      }

      if ((Key_Is_Pressed() != 0U) &&
          (Time_Elapsed(now, key_press_start_tick,
                        POWER_ON_LONG_PRESS_MS) != 0U))
      {
        second_press_started = 0U;
        power_state = NORMAL_RUN;
      }
      break;

    case NORMAL_RUN:
      /*
       * Shutdown can only be armed by a completed short press.  A direct
       * long press ends with a duration above SHORT_PRESS_MAX_MS and is ignored.
       */
      if ((key_release_event != 0U) &&
          (key_last_press_duration_ms >= SHORT_PRESS_MIN_MS) &&
          (key_last_press_duration_ms <= SHORT_PRESS_MAX_MS))
      {
        confirm_window_start_tick = now;
        second_press_started = 0U;
        power_state = WAIT_SHUTDOWN_CONFIRM;
      }
      break;

    case WAIT_SHUTDOWN_CONFIRM:
      if (second_press_started == 0U)
      {
        if (Time_Elapsed(now, confirm_window_start_tick,
                         SECOND_PRESS_WINDOW_MS) != 0U)
        {
          power_state = NORMAL_RUN;
          break;
        }

        if (key_press_event != 0U)
        {
          second_press_started = 1U;
        }
        break;
      }

      if (key_release_event != 0U)
      {
        /* Incomplete second press cancels shutdown; require a fresh short press. */
        second_press_started = 0U;
        power_state = NORMAL_RUN;
        break;
      }

      if ((Key_Is_Pressed() != 0U) &&
          (Time_Elapsed(now, key_press_start_tick,
                        SHUTDOWN_LONG_PRESS_MS) != 0U))
      {
        Shutdown_Prepare();
        Debug_LED_Set(0U);
        power_state = POWER_OFF;
        Power_Hold_Off();
      }
      break;

    case POWER_OFF:
    default:
      break;
  }

  Debug_LED_Update(now);
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
  Power_Hold_Early_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  Power_Key_Control_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
