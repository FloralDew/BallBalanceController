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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "OLED_I2C.h"
#include "MPU6050_I2C.h"
#include "stm32f1xx_it.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 激光测距模块
#define LASER_BUF_SIZE 64
typedef struct {
  uint8_t bufA[LASER_BUF_SIZE];
  uint8_t bufB[LASER_BUF_SIZE];
  uint8_t *activeBuf; // DMA 正在写入的那块(只在 ISR 里改)
  // 已收完整帧交给主循环读的那块（ISR 写，主循环读）
  uint8_t *volatile readyBuf; // pointer, instead of data, is volatile
  volatile bool frameReady;
} Laser;

typedef struct {
  uint16_t count_raw;
  bool modified_flag;
} Rotary_encoder;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
// 激光测距
Laser laser = {
  .activeBuf = laser.bufA,
  .readyBuf = NULL,
  .frameReady = 0
}; // 只初始化了部分成员，其他成员自动置0.（数组则置全0, 指针NULL）
// 全局不能写赋值这样的可执行语句. 只有在函数体内可以
// mpu6050
MPU6050_t MPU6050;
// 旋转编码器
Rotary_encoder rotary_encoder = {
  .count_raw = 0,
  .modified_flag = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************** interrupt callback **********************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // dma接收不定长数据的中断
{
  if (huart == &huart1)
  {
    uint8_t *justFilled = laser.activeBuf; // 刚收完的这块

    // 加字符串终止符（DMA 不会自动加）
    if (Size < LASER_BUF_SIZE)
      justFilled[Size] = '\0';
    else
      justFilled[LASER_BUF_SIZE - 1] = '\0';

    // 交换：刚收完的交给主循环，另块拿去继续收
    laser.activeBuf = (justFilled == laser.bufA) ? laser.bufB : laser.bufA;

    laser.readyBuf = justFilled;
    // laser_readyLen = Size;
    laser.frameReady = 1;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, laser.activeBuf, LASER_BUF_SIZE);
  }
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // tim1 call back
// { 
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
// }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    rotary_encoder.count_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    rotary_encoder.modified_flag = 1;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialization
  OLED_Init();
  OLED_Clear();
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  // OLED_ShowString(0, 0, "MPU Initializing...", 12, 0);
  while (MPU6050_Init(&hi2c1) == 1); // wait for mpu6050 to init
  OLED_ShowString(0, 0, "MPU Calibrating...", 12, 0);
  MPU6050_Calibrate_Gyro(&hi2c1, &MPU6050, 300); // 采集300次样本用于校准陀螺仪零偏，约需要300*2ms=0.6秒
  OLED_Clear();
  // OLED_ShowUint(0, 0, MPU6050_Init(&hi2c1), 3, 16, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 接收不定长数据，64为最大长度. 同样必须打开uart1全局中断
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, laser.activeBuf, LASER_BUF_SIZE);
  int i2c1_fault_cnt = 0;
  float ball_target = 0.0f;
  while (1)
  {
		// OLED_Clear();
    if (laser.frameReady)
    {
      laser.frameReady = 0;
      char *frame = (char *)laser.readyBuf; // 直接用，不需要拷贝

      if (frame != NULL)
      {
        char *pos = strtok(frame, ",");
        if (pos != NULL)
        {
          OLED_printf(0, 0, 12, 0, "%s  ", pos);
        }
      }
    }

    if (rotary_encoder.modified_flag)
    {
      rotary_encoder.modified_flag = 0;
      ball_target = rotary_encoder.count_raw / 4.0f;
      OLED_printf(80, 0, 12, 0, "%g   ", ball_target);
    }

    if (flag_100ms)
    {
      flag_100ms = 0;
      if (MPU6050_Read_All(&hi2c1, &MPU6050) == HAL_OK)
      {
        // sprintf(buf, "ac %.2f %.2f %.2f", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
        // OLED_ShowString(0, 1, buf, 12, 0);
        // sprintf(buf, "gy %.2f %.2f %.2f", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
        // OLED_ShowString(0, 2, buf, 12, 0);
        // sprintf(buf, "temp %.2f", MPU6050.Temperature);
        // OLED_ShowString(0, 3, buf, 12, 0);
        // sprintf(buf, "Euler %.2f %.2f", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);
        // OLED_ShowString(0, 4, buf, 12, 0);
        OLED_printf(0, 1, 12, 0, "%.3f %.3f  ", MPU6050.Ay, MPU6050.Az);
        OLED_printf(0, 2, 12, 0, "%.2f ", MPU6050.KalmanAngleX);
        // 串口发送
        static char buf[256] = {0}; // dma非阻塞，必须设置为static，否则dma搬走的是垃圾
        sprintf(buf, "ac %f %f %f, gy %f %f %f, tmp %f, euler %f %f\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az,
                MPU6050.Gx, MPU6050.Gy, MPU6050.Gz, MPU6050.Temperature, MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);
        // sprintf(buf, "%d,%d,%d\n", MPU6050.Accel_X_RAW, MPU6050.Accel_Y_RAW, MPU6050.Accel_Z_RAW);
        // if (huart2.gState == HAL_UART_STATE_READY)
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, strlen(buf)); // 必须也打开uart2的全局中断
      } else {
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(5);
        MX_I2C1_Init();

        OLED_printf(0, 7, 12, 0, "F %d", i2c1_fault_cnt++);
      }
    }

    if (flag_1s)
    {
      flag_1s = 0;
      // // OLED_Clear();
			// HAL_ADC_Start(&hadc1); // 启动ADC常规序列
      // HAL_ADC_PollForConversion(&hadc1, 5); // 等待转换完成，us级. 超时时间单位为ms
      // uint32_t dr = HAL_ADC_GetValue(&hadc1);
      // float motor_votage = dr * (3.3 - 0.0) / 4095.0;
      // OLED_ShowFloat(16 * 6, 7, motor_votage, 2, 2, 12, 0);
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
