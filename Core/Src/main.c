/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "app_health.h"
#include "app_mode_task.h"
#include "auto_run.h"
#include "bluetooth.h"
#include "buzzer.h"
#include "car_mode.h"
#include "car.h"
#include "encoder.h"
#include "follow_task.h"
#include "hx711.h"
#include "motion_control.h"
#include "oled_ui.h"
#include "servo.h"
#include "status_led.h"
#include "tracking.h"
#include "transport_task.h"
#include "ultrasonic.h"
#include "uwb_follow.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ULTRASONIC_PERIOD_MS         100U
#define HX711_WEIGHT_PERIOD_MS       500U
#define HX711_TARE_SAMPLE_PERIOD_MS  100U
#if APP_DEBUG_ENABLE
#define DEBUG_PRINT_PERIOD_MS        1000U
#endif
#define APP_RUN_TASK(task_id, call_expr)                 \
  do                                                     \
  {                                                      \
    uint32_t task_start_tick = HAL_GetTick();            \
    AppHealth_TaskBegin((task_id), task_start_tick);     \
    call_expr;                                           \
    AppHealth_TaskEnd((task_id), HAL_GetTick());         \
  } while (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile CarMode_t g_car_mode = MODE_TRANSPORT;
int16_t g_car_speed = 0;
uint16_t g_distance_cm = ULTRASONIC_TIMEOUT_CM;
int32_t g_weight_g = 0;
static uint32_t s_ultrasonic_last_tick = 0;
static uint32_t s_weight_last_tick = 0;
static uint8_t s_weight_tared = 0;
static int64_t s_weight_tare_sum = 0;
static uint8_t s_weight_tare_count = 0;
#if APP_DEBUG_ENABLE
static uint32_t s_debug_last_tick = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_UpdateUltrasonic(uint32_t now)
{
  uint16_t distance = 0;

  Ultrasonic_Task();

  if (Ultrasonic_TakeDistanceCm(&distance) != 0U)
  {
    g_distance_cm = distance;
  }

  if (((uint32_t)(now - s_ultrasonic_last_tick) >= ULTRASONIC_PERIOD_MS) &&
      (Ultrasonic_IsBusy() == 0U))
  {
    s_ultrasonic_last_tick = now;
    Ultrasonic_Start();
  }
}

static void App_UpdateWeight(uint32_t now)
{
  uint32_t period_ms = (s_weight_tared != 0U) ? HX711_WEIGHT_PERIOD_MS : HX711_TARE_SAMPLE_PERIOD_MS;
  int32_t raw = 0;

  if ((uint32_t)(now - s_weight_last_tick) >= period_ms)
  {
    s_weight_last_tick = now;

    if (HX711_ReadRawIfReady(&raw) != HX711_OK)
    {
      return;
    }

    if (s_weight_tared == 0U)
    {
      s_weight_tare_sum += raw;
      s_weight_tare_count++;

      if (s_weight_tare_count < HX711_DEFAULT_TARE_TIMES)
      {
        g_weight_g = 0;
        return;
      }

      HX711_SetOffset((int32_t)(s_weight_tare_sum / s_weight_tare_count));
      s_weight_tared = 1U;
      g_weight_g = 0;
      return;
    }

    g_weight_g = (int32_t)HX711_RawToWeight(raw);
  }
}

static void App_InitWeight(void)
{
  HX711_Init();
  HX711_SetScale(HX711_DEFAULT_SCALE);
  s_weight_tared = 0U;
  s_weight_tare_sum = 0;
  s_weight_tare_count = 0;
  g_weight_g = 0;
  s_weight_last_tick = HAL_GetTick();
}

static void App_RunMotionTask(void)
{
  switch (g_car_mode)
  {
    case MODE_MANUAL:
      break;

    case MODE_AUTO:
      Auto_Task();
      break;

    case MODE_TRACK:
      Tracking_Task();
      break;

    case MODE_FOLLOW:
      break;

    case MODE_TRANSPORT:
      Transport_Task();
      break;

    case MODE_EMERGENCY_STOP:
    default:
      Transport_EmergencyStop();
      g_car_speed = 0;
      Motion_Submit(MOTION_OWNER_EMERGENCY, MOTION_CMD_STOP, 0, 0U);
      break;
  }
}

static uint8_t App_StatusLedCanFollowMode(StatusLedState_t state)
{
  return ((state == STATUS_LED_IDLE) ||
          (state == STATUS_LED_MANUAL) ||
          (state == STATUS_LED_AUTO) ||
          (state == STATUS_LED_TRACK) ||
          (state == STATUS_LED_TRANSPORT)) ? 1U : 0U;
}

static void App_SyncStatusLedByMode(void)
{
  StatusLedState_t current = StatusLed_GetState();
  StatusLedState_t mode_state = STATUS_LED_MANUAL;

  if (g_car_mode == MODE_EMERGENCY_STOP)
  {
    StatusLed_SetState(STATUS_LED_EMERGENCY);
    return;
  }

  if (AppMode_GetMode() == APP_MODE_FOLLOW_LIGHT)
  {
    StatusLed_SetState(STATUS_LED_FOLLOW);
    return;
  }

  if (App_StatusLedCanFollowMode(current) == 0U)
  {
    return;
  }

  switch (g_car_mode)
  {
    case MODE_AUTO:
      mode_state = STATUS_LED_AUTO;
      break;

    case MODE_TRACK:
      mode_state = STATUS_LED_TRACK;
      break;

    case MODE_FOLLOW:
      mode_state = STATUS_LED_FOLLOW;
      break;

    case MODE_TRANSPORT:
      mode_state = STATUS_LED_TRANSPORT;
      break;

    case MODE_MANUAL:
    default:
      mode_state = STATUS_LED_MANUAL;
      break;
  }

  StatusLed_SetState(mode_state);
}

static void App_RunLedTask(void)
{
  App_SyncStatusLedByMode();
  StatusLed_Task();
}

static void App_DebugPrint(uint32_t now)
{
#if APP_DEBUG_ENABLE
  if ((uint32_t)(now - s_debug_last_tick) >= DEBUG_PRINT_PERIOD_MS)
  {
    s_debug_last_tick = now;
    printf("mode=%d, L=%ld pulse, LS=%ld pulse/s, R=%ld pulse, RS=%ld pulse/s, DIS=%u, WT=%ld\r\n",
           (int)g_car_mode,
           (long)Encoder_GetLeftPulse(),
           (long)Encoder_GetLeftSpeed(),
           (long)Encoder_GetRightPulse(),
           (long)Encoder_GetRightSpeed(),
           g_distance_cm,
           (long)g_weight_g);
  }
#else
  (void)now;
#endif
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Buzzer_Init();
  Car_Init();
  Encoder_Init();
  Ultrasonic_Init();
  Servo_Init();
  App_InitWeight();
  AppHealth_Init();
  StatusLed_Init();
  Tracking_Init();
  Auto_Init();
  UwbFollow_Init();
  Follow_Init();
  OLED_UI_Init();
  Transport_Init();
  Bluetooth_Init();
  AppMode_Init();

#if APP_DEBUG_ENABLE
  printf("\r\nAuto car scheduler start\r\n");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    AppHealth_LoopBegin(now);

    APP_RUN_TASK(APP_HEALTH_TASK_APP_MODE, AppMode_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_BLUETOOTH, Bluetooth_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_LED, App_RunLedTask());
    APP_RUN_TASK(APP_HEALTH_TASK_BUZZER, Buzzer_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_ULTRASONIC, App_UpdateUltrasonic(now));
    APP_RUN_TASK(APP_HEALTH_TASK_ENCODER, Encoder_Update());
    APP_RUN_TASK(APP_HEALTH_TASK_WEIGHT, App_UpdateWeight(now));
    APP_RUN_TASK(APP_HEALTH_TASK_UWB, UwbFollow_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_FOLLOW, Follow_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_MODE_CONTROL, App_RunMotionTask());
    APP_RUN_TASK(APP_HEALTH_TASK_SERVO, Servo_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_MOTION, Motion_Task());
    APP_RUN_TASK(APP_HEALTH_TASK_OLED, OLED_UI_Update());
    APP_RUN_TASK(APP_HEALTH_TASK_DEBUG, App_DebugPrint(now));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Bluetooth_OnRxCpltCallback(huart);
    return;
  }

  if (huart->Instance == USART2)
  {
    UwbFollow_OnRxCpltCallback(huart);
    return;
  }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Bluetooth_OnErrorCallback(huart);
    return;
  }

  if (huart->Instance == USART2)
  {
    UwbFollow_OnErrorCallback(huart);
    return;
  }
}

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
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
