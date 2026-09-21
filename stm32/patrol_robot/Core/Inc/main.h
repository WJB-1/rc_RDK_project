/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  //陀螺仪数据结构体
  typedef struct {
    float yaw, pitch, roll;			   // ° (Z,Y,X顺序)
    float gyro[3];                 // °/s (Z,Y,X顺序)
    float accel[3];                // m/s² (Z,Y,X顺序)
    float temperature;             // °C
    float quaternion[4];           // 四元数无量纲
    float magnet[3];               // 磁场强度，无标准单位（原始值/150） (Z,Y,X顺序)
  } IMUData;

  //目标量结构体
  typedef struct
  {
    float position;	                // cm
    float speed_L,speed_R,speed;    // cm/s
    float turn_yaw_angle;           // °
    float slave0_angle;             // °
    float slave1_angle;             // °
    float torque_L,torque_R;        // Nm
  } Target;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
  extern IMUData imuData;
  extern Target target;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define buzzer_Pin GPIO_PIN_4
#define buzzer_GPIO_Port GPIOA
#define key1_Pin GPIO_PIN_5
#define key1_GPIO_Port GPIOA
#define key1_EXTI_IRQn EXTI9_5_IRQn
#define key2_Pin GPIO_PIN_6
#define key2_GPIO_Port GPIOA
#define key2_EXTI_IRQn EXTI9_5_IRQn
#define test1_Pin GPIO_PIN_7
#define test1_GPIO_Port GPIOA
#define test2_Pin GPIO_PIN_0
#define test2_GPIO_Port GPIOB
#define slave0_Pin GPIO_PIN_8
#define slave0_GPIO_Port GPIOA
#define slave1_Pin GPIO_PIN_9
#define slave1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
