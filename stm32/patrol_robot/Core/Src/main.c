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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdio.h"

#include "speed_PID.h"
#include "simple_CAN.h"
#include "angle_unwrap.h"
#include "car_sport.h"
#include "mix_algorithm.h"

#include "oled.h"
#include "JY901S.h"
#include "brushless_motor.h"
#include "slave.h"
#include "move.h"

/* Delete dubug.h to remove the temporary single-character commands. */
#if defined(__has_include)
#if __has_include("dubug.h")
#include "dubug.h"
#define DUBUG_COMMANDS_ENABLED 1
#endif
#endif

#ifndef DUBUG_COMMANDS_ENABLED
#define DUBUG_COMMANDS_ENABLED 0
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* 电池电压采样与滤波 */
uint16_t input_value;
float input_voltage, filtered_Voltage = 0;
int input_voltage_count = 0;
float filtered_voltage_alpha = 0.05f;

/* 串口接收缓冲区：
 * huart1 连接 HC-05，模拟正式上位机
 * huart3 连接 HC-04，用于 VOFA 调试 */
uint8_t debugReceiveData[64];
uint8_t upperReceiveData[128];
uint16_t dataSize = 0;

/* OLED 显示缓存 */
char data1[50], data2[50], data3[50], data4[50], data5[50];

typedef struct
{
  float channel[11];
  uint32_t tail;
} VofaJustFloatFrame;

_Static_assert(sizeof(VofaJustFloatFrame) == 48U, "VOFA JustFloat frame size must be 48 bytes");

static VofaJustFloatFrame vofa_frame;

/* IMU 解算结果 */
IMUData imuData;

/* 当前底盘目标值 */
Target target = {0,0,0,0,0,0,0,0,0};

/* 调试值 */
int16_t L_voltage = 0, R_voltage = 0;
int start = 0;
float p = 88.0f, i = 28.0f, d = 0.8f;

/* USART1 是否向 HC-05 发送正式里程和状态反馈。 */
static const uint8_t kEnableUpperUartFeedback = 1U;
static uint32_t vofa_last_tx_tick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float NormalizeDisplayAngle(float angle);
static bool HandleVofaTuningCommand(uint8_t *data, uint16_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float NormalizeDisplayAngle(float angle)
{
  while (angle >= 180.0f)
  {
    angle -= 360.0f;
  }

  while (angle < -180.0f)
  {
    angle += 360.0f;
  }

  return angle;
}

static bool HandleVofaTuningCommand(uint8_t *data, uint16_t size)
{
  float value;

  if ((data == NULL) || (size < 3U) || (data[1] != ':'))
  {
    return false;
  }

  value = safe_extract_float((char *)data);

  switch (data[0])
  {
    case 'A':
      straight_yaw_kp = value;
      return true;

    case 'B':
      straight_yaw_kd = value;
      return true;

    case 'C':
      straight_vision_kp = value;
      return true;

    case 'G':
      straight_imu_enabled = (value >= 0.5f) ? 1U : 0U;
      return true;

    case 'V':
      straight_vision_enabled = (value >= 0.5f) ? 1U : 0U;
      return true;

    case 'P':
      p = value;
      PID_change(p, i, d);
      return true;

    case 'I':
      i = value;
      PID_change(p, i, d);
      return true;

    case 'D':
      d = value;
      PID_change(p, i, d);
      return true;

    case 'S':
      target.speed_L = value;
      target.speed_R = value;
      return true;

    default:
      return false;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* TIM4 是 20ms 一次的底盘控制周期 */
  if (htim == &htim4)
  {
    /* 先更新传感器和电机当前状态 */
    Read_IMU_Data();
    Motor_data_calculation();

    /* 再执行上位机运动逻辑：
     * 巡航 / 转向 / 里程反馈 都在这里更新 */
    Move_Process20ms();

    if (start != 0)
    {
      /* 左右轮速度环，输出扭矩 */
      target.torque_L = speed_pid_L(&leftWheel.speed, target.speed_L);
      target.torque_R = speed_pid_R(&rightWheel.speed, target.speed_R);
      /* 把目标扭矩写入电机对象并通过 CAN 发出去 */
      Motor_SetTorque(&leftWheel, target.torque_L);
      Motor_SetTorque(&rightWheel, target.torque_R);
      Motor_SendTask(&leftWheel, &rightWheel, &filtered_Voltage, &L_voltage, &R_voltage);
    }
    else
    {
      /* 停车时持续发 0 扭矩，确保电机真正停下 */
      target.torque_L = 0.0f;
      target.torque_R = 0.0f;
      Motor_SetTorque(&leftWheel, 0.0f);
      Motor_SetTorque(&rightWheel, 0.0f);
      Motor_SendTask(&leftWheel, &rightWheel, &filtered_Voltage, &L_voltage, &R_voltage);
    }
  }

  /* TIM3 用于舵机平滑移动 */
  if (htim == &htim3)
  {
    if (slave_0.status)
    {
      if (slave_0.work_times > slave_0.work)
      {
        slave_0.work++;
        slave_0.crr_value = slave_0.crr_value + slave_0.work_ccr;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, slave_0.crr_value);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)slave_0.PWM);
        reset_slave(&slave_0);
      }
    }

    if (slave_1.status)
    {
      if (slave_1.work_times > slave_1.work)
      {
        slave_1.work++;
        slave_1.crr_value = slave_1.crr_value + slave_1.work_ccr;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, slave_1.crr_value);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)slave_1.PWM);
        reset_slave(&slave_1);
      }
    }

    if (slave_0.status == 0 && slave_1.status == 0)
    {
      HAL_TIM_Base_Stop_IT(&htim3);
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  dataSize = Size;

  /* huart3 连接 HC-04，是 VOFA 调试串口
   *
   * 这里支持的不是正式上位机协议，而是简单调试命令：
   * 1. S:20  让左右轮都按 20cm/s 直行
   * 2. P:xx  改 PID 的 P
   * 3. I:xx  改 PID 的 I
   * 4. D:xx  改 PID 的 D
   * 5. A:xx / B:xx 调直行航向 P / D
   * 6. C:xx 调视觉 P，V:0 / V:1 关闭或开启视觉辅助
   * 7. 也兼容老的二进制双轮速度格式 */
  if (huart == &huart3)
  {
    if (dataSize < sizeof(debugReceiveData))
    {
      debugReceiveData[dataSize] = '\0';
    }
    else
    {
      debugReceiveData[sizeof(debugReceiveData) - 1] = '\0';
    }

    /* 老格式：
     * [0xA5][float speed_L][float speed_R]
     * 直接给左右轮速度 */
    if (debugReceiveData[0] == 0xA5)
    {
      memcpy(&target.speed_L, &debugReceiveData[1], sizeof(target.speed_L));
      memcpy(&target.speed_R, &debugReceiveData[5], sizeof(target.speed_R));
    }

    /* 文本调试命令 */
    if (!HandleVofaTuningCommand(debugReceiveData, dataSize))
    {
      if (debugReceiveData[0] == 1U)
      {
        PID_out_clear();
      }
    }

    /* 重新打开蓝牙串口 DMA 接收 */
    HAL_UART_AbortReceive(&huart3);
    memset(debugReceiveData, 0, sizeof(debugReceiveData));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, debugReceiveData, sizeof(debugReceiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }

  /* huart1 连接 HC-05，是正式上位机协议串口
   * 收到的原始字节流交给 move 模块去拆包和执行 */
  if (huart == &huart1)
  {
#if DUBUG_COMMANDS_ENABLED
    if (!Dubug_HandleCommand(upperReceiveData, Size))
    {
      Move_InputBytes(upperReceiveData, Size);
    }
#else
    Move_InputBytes(upperReceiveData, Size);
#endif
    HAL_UART_AbortReceive(&huart1);
    memset(upperReceiveData, 0, sizeof(upperReceiveData));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, upperReceiveData, sizeof(upperReceiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* 按键 1：人工启动
   * 按键 2：人工急停 */
  HAL_Delay(10);
  if (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(100);
    start++;
    while (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET);
  }

  if (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(100);
    Move_EmergencyStop();
    while (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin) == GPIO_PIN_RESET);
  }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init(&hcan);
  CAN_Filter_Mask_Config(&hcan, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x100, 0x7F8);
  Motor_InitAll();

  HAL_Delay(20);
  OLED_Init();
  /* IMU 走 USART2，HC-04/VOFA 调试走 USART3 */
  Gyroscope_Init(&huart2, &huart3);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&input_value, 1);

  /* 启动两个串口的 DMA 接收 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, debugReceiveData, sizeof(debugReceiveData));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, upperReceiveData, sizeof(upperReceiveData));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* 舵机初始化 + move 模块初始化 + 开启 20ms 控制周期 */
  slave_Init();
  Move_Init(&huart1);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * 把显示给人看的 yaw 角限制到 [-180, 180)。
     * 这里只影响 OLED 和蓝牙调试输出，不改 imuData.yaw 内部值，
     * 因此不会影响 move.c 里的原地转向和边走边转控制。
     */
    float yaw = NormalizeDisplayAngle(imuData.yaw);
    /* OLED 上显示当前速度、输出、电压、姿态等调试信息 */
    sprintf(data1, "%.2f,%.2f,%.2f,%.2f", p, i, d, target.speed_L);
    sprintf(data2, "R%.2f,L%.2f", rightWheel.speed, leftWheel.speed);
    sprintf(data3, "out:%d,%d", L_voltage, R_voltage);
    sprintf(data4, "vol:%.2f", filtered_Voltage);
    sprintf(data5, "y%.2f,r%.2f", yaw, imuData.roll);
    OLED_NewFrame();
    OLED_PrintASCIIString(0, 0, data2, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 12, data1, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 24, data4, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 36, data5, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 48, data3, &afont12x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    HAL_Delay(5);

    /* USART1 通过 HC-05 向正式上位机发送里程和状态反馈。 */
    if (kEnableUpperUartFeedback != 0U)
    {
      Move_UartTask();
    }

    /*
     * HC-04 on USART3 -> VOFA+ JustFloat, 10 Hz, channel order:
     * yaw,target_yaw,yaw_error,imu_correction,
     * vision_error,vision_correction,total_correction,
     * target_left,target_right,actual_left,actual_right
     */
    if (((uint32_t)(HAL_GetTick() - vofa_last_tx_tick) >= 100U) &&
        (huart3.gState == HAL_UART_STATE_READY))
    {
      vofa_frame.channel[0] = imuData.yaw;
      vofa_frame.channel[1] = straight_target_yaw;
      vofa_frame.channel[2] = straight_yaw_error;
      vofa_frame.channel[3] = straight_imu_correction;
      vofa_frame.channel[4] = vision_error;
      vofa_frame.channel[5] = vision_correction;
      vofa_frame.channel[6] = straight_total_correction;
      vofa_frame.channel[7] = target.speed_L;
      vofa_frame.channel[8] = target.speed_R;
      vofa_frame.channel[9] = leftWheel.speed;
      vofa_frame.channel[10] = rightWheel.speed;
      vofa_frame.tail = 0x7F800000U;

      vofa_last_tx_tick = HAL_GetTick();
      HAL_UART_Transmit_DMA(&huart3,
                            (uint8_t *)&vofa_frame,
                            (uint16_t)sizeof(vofa_frame));
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    /* 电池低压告警 */
    input_voltage = (float)input_value / 4095.0f * 3.3f * 5.7f;
    filtered_Voltage = filtered_voltage_alpha * input_voltage + (1.0f - filtered_voltage_alpha) * filtered_Voltage;
    if (filtered_Voltage <= 11.15f)
    {
      input_voltage_count++;
      if (input_voltage_count == 100)
      {
        HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
      }
    }
    else
    {
      input_voltage_count = 0;
      HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
