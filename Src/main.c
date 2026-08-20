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
#include <stdarg.h>
#include <stdlib.h> // atof
#include <math.h>

#include "stm32f1xx_it.h"
#include "controller.h"
#include "button.h"

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
  float ball_target;
  bool modified_flag;
} Rotary_encoder;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define OLED_STATE_POS_COL 0
#define OLED_STATE_POS_ROW 5
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
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
MPU6050_t MPU6050_gw;
// 旋转编码器
Rotary_encoder rotary_encoder = {
  .count_raw = 0,
  .ball_target = 0.0f,
  .modified_flag = 0
};
// 按钮
BTN_HandleTypedef hbutton;
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
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*********************** interrupt callback **********************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // dma接收不定长数据的中断
{
  if (huart == &huart2) // 激光测距
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

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, laser.activeBuf, LASER_BUF_SIZE);
  }
  
  if (huart == &huart3)
  {
    // 张大头反馈
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

/* Other functions */
HAL_StatusTypeDef UART_DMA_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
  static char STRING_BUF[256] = {0};
  va_list ap;

  va_start(ap, fmt);
  vsnprintf(STRING_BUF, sizeof(STRING_BUF), fmt, ap);
  va_end(ap);

  return HAL_UART_Transmit_DMA(huart, (uint8_t *)STRING_BUF, strlen(STRING_BUF));
}


/* ************************ 调度器任务 *********************** */
static void task_get_button(void)
{
  Button_Get(&hbutton);
  BTN_StateTypedef btn_type = hbutton.btn_state;
  hbutton.btn_state = BTN_RELEASE; // 及时复位

  switch (btn_type)
  {
    case BTN_PRESS:
      if (Controller_GetState() == CONTROLLER_IDLE)
      {
        BallStablization_Start(0, rotary_encoder.ball_target * 10.0f); // cm -> mm
        Sched_SetEnable(TASK_BALL_STAB, 1);
        Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1);
      }
      break;
    case BTN_DOUBLEPRESS:
      // OLED_printf(10, 4, 12, 0, "%d", ++cnt[2]);
      break;
    case BTN_LONGPRESS:
      if (Controller_GetState() == CONTROLLER_IDLE)
      {
        ZeroGuideway_Start();
        Sched_SetEnable(TASK_ZERO_GUIDEWAY, 1);
        Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1);
      }
      break;
      
    default:
      break;
  }
}

static void task_laser(void)
{
  if (laser.frameReady)
  {
    laser.frameReady = 0;
    char *frame = (char *)laser.readyBuf; // 直接用，不需要拷贝

    if (frame != NULL)
    {
      char *pos = strtok(frame, ",");
      if (pos != NULL)
      {
        // OLED_Clear(0, 0);
        Guideway_FeedBallPos((float)atof(pos));
        OLED_printf(0, 0, 12, 0, "pos: %-7s", pos);
      }
    }
  }
}

static void task_rotary_encoder(void)
{
  if (rotary_encoder.modified_flag)
  {
    rotary_encoder.modified_flag = 0;
    rotary_encoder.ball_target = rotary_encoder.count_raw / 4.0f + 2.0f;
    OLED_printf(12, 0, 12, 0, "rot: %-4g", rotary_encoder.ball_target);
  }
}

static void task_read_mpu(void)
{
  static int i2c1_fault_cnt = 0;
  if (MPU6050_Read_All(&hi2c1, &MPU6050_gw) == HAL_OK)
  {
    Guideway_FeedAngle(MPU6050_gw.KalmanAngleX); // 外部给予控制器当前角度
  } else {
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(2);
    MX_I2C1_Init();

    OLED_printf(0, 7, 12, 1, "I2C Fault %d", ++i2c1_fault_cnt);
  }
}

static void task_display_uart(void)
{
  // OLED_Clear(1, 2);
  // OLED_printf(0, 1, 12, 0, "ac_yz %.3f %.3f", MPU6050_gw.Ay, MPU6050_gw.Az);
  // OLED_printf(0, 2, 12, 0, "gy %.2f %.2f %.2f", MPU6050_gw.Gx, MPU6050_gw.Gy, MPU6050_gw.Gz);
  // OLED_printf(0, 3, 12, 0, "temp %.2f", MPU6050_gw.Temperature);
  OLED_printf(0, 2, 12, 0, "theta %.2f   ", MPU6050_gw.KalmanAngleX);
  // OLED_printf(0, 3, 12, 0, "euler_x_f %.2f", Guideway_GetAngle());
  // 串口发送
  // if (huart1.gState == HAL_UART_STATE_READY)
  // UART_DMA_printf(&huart1, "ac %f %f %f, gy %f %f %f, tmp %f, euler %f %f\n", MPU6050_gw.Ax, MPU6050_gw.Ay, MPU6050_gw.Az,
  //                 MPU6050_gw.Gx, MPU6050_gw.Gy, MPU6050_gw.Gz, MPU6050_gw.Temperature, MPU6050_gw.KalmanAngleX, MPU6050_gw.KalmanAngleY); // 必须也打开uart2的全局中断
}

static void task_adc(void)
{
  HAL_ADC_Start(&hadc1);                // 启动ADC常规序列
  HAL_ADC_PollForConversion(&hadc1, 1); // 等待转换完成，轮询，us级. 超时时间单位为ms
  uint32_t dr = HAL_ADC_GetValue(&hadc1);
  float motor_votage = dr * (3.3 - 0.0) / 4095.0;
  OLED_printf(15, 7, 12, 0, "%.2fV", motor_votage);
}

static void task_zero_guideway(void)
{
  ZeroGuideway_Poll();
  if (Controller_GetState() == CONTROLLER_IDLE)
  {
    Sched_SetEnable(TASK_ZERO_GUIDEWAY, 0);
    OLED_Clear(5, 5);
    Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1);
  }
}

static void task_get_lut(void) {
  static int pulse = -700; // 只有第一次调用会执行初始化，-700大概在+15度左右
  // static int dir = 1;
  Get_Guideway_LUT_Poll(pulse);
  float angle = Guideway_GetAngle();
  UART_DMA_printf(&huart1, "%d,%f\n", pulse, angle);
  if (angle < -15.0f)
  {
    Controller_SetIDLE();
    Sched_SetEnable(TASK_GET_LUT, 0);
    OLED_Clear(5, 5);
    Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1);
  }
  pulse += 5; // 逐渐往下运动
}

static void task_ball_stab(void)
{
  if (BallStablization_Poll(0) == CONTROLLER_IDLE)
  {
    Sched_SetEnable(TASK_BALL_STAB, 0);
    OLED_Clear(5, 5);
    Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1);
  }
}

/* ---------- 任务表 ---------- */
static Task_t task_list[TASK_COUNT] = {
  [TASK_LASER] =          {task_laser,          0,      1, 0},
  [TASK_ROTARY_ENCODER] = {task_rotary_encoder, 0,      1, 0},
  [TASK_READ_MPU] =       {task_read_mpu,       10,     1, 0}, // mpu6050 init中制定了采集周期为10ms，不能再小了
  [TASK_ZERO_GUIDEWAY] =  {task_zero_guideway,  250,    0, 0}, // 控制周期ms，需 > 传感器延迟 + 运动时间
  [TASK_GET_LUT] =        {task_get_lut,        3000,   0, 0},
  [TASK_BALL_STAB] =      {task_ball_stab,      20,     0, 0},
  [TASK_DISPLAY_UART] =   {task_display_uart,   200,    1, 0},
  [TASK_ADC] =            {task_adc,            1000,   1, 0},
  [TASK_GET_BUTTON] =     {task_get_button,     10,     1, 0},
};
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Initialization
  // OLED初始化
  OLED_Init(0x8f);
  OLED_Clear(0, 7);
  // 旋转编码器初始化
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  // MPU6050初始化
  while (MPU6050_Init(&hi2c1) == 1); // wait for mpu6050 to init
  OLED_printf(0, 0, 12, 0, "MPU Calibrating...");
  MPU6050_Calibrate_Gyro(&hi2c1, &MPU6050_gw, 300); // 采集300次样本用于校准陀螺仪零偏，约需要300*2ms=0.6秒
  OLED_Clear(0, 7);
  // 按钮初始化
  Button_Init(&hbutton, BTN_GPIO_Port, BTN_Pin);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // 接收不定长数据，LASER_BUF_SIZE为最大长度. 同样必须打开uart1全局中断
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, laser.activeBuf, LASER_BUF_SIZE);
  rotary_encoder.modified_flag = 1; // 触发一次旋转编码器显示
  Sched_Init(task_list, sizeof(task_list) / sizeof(task_list[0]));

  Motor_Return_Origin(); // 电机回零，实测会飘0.2度以内
  HAL_Delay(1000); // 等待电机回零完成

  Show_State_On_OLED(OLED_STATE_POS_COL, OLED_STATE_POS_ROW, 12, 1); // idle
  while (1)
  {
    Sched_Run();

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim3.Init.Period = 63;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SPI_DC_GPIO_Port, OLED_SPI_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SPI_RES_GPIO_Port, OLED_SPI_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SPI_DC_Pin */
  GPIO_InitStruct.Pin = OLED_SPI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SPI_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SPI_RES_Pin */
  GPIO_InitStruct.Pin = OLED_SPI_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SPI_RES_GPIO_Port, &GPIO_InitStruct);

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
