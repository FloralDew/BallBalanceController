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
#include "OLED_I2C.h"
#include "MPU6050_I2C.h"
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LASER_BUF_SIZE 64

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static uint8_t laser_bufA[LASER_BUF_SIZE];
static uint8_t laser_bufB[LASER_BUF_SIZE];
// DMA 正在写入的那块（只在 ISR 里改）
static uint8_t *laser_activeBuf = laser_bufA;
// 已收完整帧交给主循环读的那块（ISR 写，主循环读）
static uint8_t *volatile laser_readyBuf = NULL; // pointer, instead of data, is volatile
static volatile uint16_t laser_readyLen = 0;
static volatile uint8_t laser_frameReady = 0;

MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************** interrupt callback **********************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // dma接收不定长数据的中断
{
  if (huart == &huart1)
  {
    uint8_t *justFilled = laser_activeBuf; // 刚收完的这块

    // 加字符串终止符（DMA 不会自动加）
    if (Size < LASER_BUF_SIZE)
      justFilled[Size] = '\0';
    else
      justFilled[LASER_BUF_SIZE - 1] = '\0';

    // 交换：刚收完的交给主循环，另块拿去继续收
    laser_activeBuf = (justFilled == laser_bufA) ? laser_bufB : laser_bufA;

    laser_readyBuf = justFilled;
    laser_readyLen = Size;
    laser_frameReady = 1;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, laser_activeBuf, LASER_BUF_SIZE);
  }
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // tim1 call back
// { 
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
// }
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
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "MPU Initializing...", 12, 0);
  while (MPU6050_Init(&hi2c1) == 1); // wait for mpu6050 to init
  OLED_ShowString(0, 0, "MPU Calibrating...", 12, 0);
  MPU6050_Calibrate_Gyro(&hi2c1, &MPU6050, 300); // 采集300次样本用于校准陀螺仪零偏，约需要300*2ms=0.6秒
  OLED_Clear();
  // OLED_ShowUint(0, 0, MPU6050_Init(&hi2c1), 3, 16, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, laser_activeBuf, 64); // 接收不定长数据，32为最大长度。同样必须打开uart1全局中断
	int i2c1_fault_cnt = 0;
  while (1)
  {
		// OLED_Clear();
    if (laser_frameReady)
    {
      laser_frameReady = 0;
      char *frame = (char *)laser_readyBuf; // 直接用，不需要拷贝

      if (frame != NULL)
      {
        char *pos = strtok(frame, ",");
        if (pos != NULL)
        {
          OLED_ShowString(0, 0, "pos:", 12, 0);
          OLED_ShowString(6 * 5, 0, pos, 12, 0);
        }
      }
    }
    
    if (flag_100ms)
    {
      flag_100ms = 0;
      if (MPU6050_Read_All(&hi2c1, &MPU6050) == HAL_OK)
      {
        static char buf[256] = {0}; // dma非阻塞，必须设置为static，否则dma搬走的是垃圾
        sprintf(buf, "ac %.2f %.2f %.2f", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
        OLED_ShowString(0, 1, buf, 12, 0);
        sprintf(buf, "gy %.2f %.2f %.2f", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
        OLED_ShowString(0, 2, buf, 12, 0);
        sprintf(buf, "temp %.2f", MPU6050.Temperature);
        OLED_ShowString(0, 3, buf, 12, 0);
        sprintf(buf, "Euler %.2f %.2f", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);
        OLED_ShowString(0, 4, buf, 12, 0);

        // sprintf(buf, "ac %f %f %f, gy %f %f %f, tmp %f, euler %f %f\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az,
        //         MPU6050.Gx, MPU6050.Gy, MPU6050.Gz, MPU6050.Temperature, MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);
        sprintf(buf, "%d,%d,%d\n", MPU6050.Accel_X_RAW, MPU6050.Accel_Y_RAW, MPU6050.Accel_Z_RAW);
        // if (huart2.gState == HAL_UART_STATE_READY)
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, strlen(buf)); // 必须也打开uart2的全局中断
      } else {
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(5);
        MX_I2C1_Init();
        OLED_ShowString(0, 7, "I2C1 Fault", 12, 1);
        i2c1_fault_cnt++;
        OLED_ShowUint(10 * 6, 7, i2c1_fault_cnt, 3, 12, 1);
      }
    }

    if (flag_1s)
    {
      flag_1s = 0;
      // OLED_Clear();
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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
