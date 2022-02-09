/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for RadioCommand */
osThreadId_t RadioCommandHandle;
uint32_t RadioCommandBuffer[ 128 ];
osStaticThreadDef_t RadioCommandControlBlock;
const osThreadAttr_t RadioCommand_attributes = {
  .name = "RadioCommand",
  .stack_mem = &RadioCommandBuffer[0],
  .stack_size = sizeof(RadioCommandBuffer),
  .cb_mem = &RadioCommandControlBlock,
  .cb_size = sizeof(RadioCommandControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ReadIMU */
osThreadId_t ReadIMUHandle;
uint32_t ReadIMUBuffer[ 128 ];
osStaticThreadDef_t ReadIMUControlBlock;
const osThreadAttr_t ReadIMU_attributes = {
  .name = "ReadIMU",
  .stack_mem = &ReadIMUBuffer[0],
  .stack_size = sizeof(ReadIMUBuffer),
  .cb_mem = &ReadIMUControlBlock,
  .cb_size = sizeof(ReadIMUControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadMAG */
osThreadId_t ReadMAGHandle;
uint32_t ReadMAGBuffer[ 128 ];
osStaticThreadDef_t ReadMAGControlBlock;
const osThreadAttr_t ReadMAG_attributes = {
  .name = "ReadMAG",
  .stack_mem = &ReadMAGBuffer[0],
  .stack_size = sizeof(ReadMAGBuffer),
  .cb_mem = &ReadMAGControlBlock,
  .cb_size = sizeof(ReadMAGControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AttitudeControl */
osThreadId_t AttitudeControlHandle;
uint32_t AttitudeControlBuffer[ 128 ];
osStaticThreadDef_t AttitudeControlControlBlock;
const osThreadAttr_t AttitudeControl_attributes = {
  .name = "AttitudeControl",
  .stack_mem = &AttitudeControlBuffer[0],
  .stack_size = sizeof(AttitudeControlBuffer),
  .cb_mem = &AttitudeControlControlBlock,
  .cb_size = sizeof(AttitudeControlControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CyclicPitchCont */
osThreadId_t CyclicPitchContHandle;
uint32_t CyclicPitchContBuffer[ 128 ];
osStaticThreadDef_t CyclicPitchContControlBlock;
const osThreadAttr_t CyclicPitchCont_attributes = {
  .name = "CyclicPitchCont",
  .stack_mem = &CyclicPitchContBuffer[0],
  .stack_size = sizeof(CyclicPitchContBuffer),
  .cb_mem = &CyclicPitchContControlBlock,
  .cb_size = sizeof(CyclicPitchContControlBlock),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorOutput */
osThreadId_t MotorOutputHandle;
uint32_t MotorOutputBuffer[ 128 ];
osStaticThreadDef_t MotorOutputControlBlock;
const osThreadAttr_t MotorOutput_attributes = {
  .name = "MotorOutput",
  .stack_mem = &MotorOutputBuffer[0],
  .stack_size = sizeof(MotorOutputBuffer),
  .cb_mem = &MotorOutputControlBlock,
  .cb_size = sizeof(MotorOutputControlBlock),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RCdata */
osMessageQueueId_t RCdataHandle;
uint8_t RCdataBuffer[ 1 * sizeof( RC ) ];
osStaticMessageQDef_t RCdataControlBlock;
const osMessageQueueAttr_t RCdata_attributes = {
  .name = "RCdata",
  .cb_mem = &RCdataControlBlock,
  .cb_size = sizeof(RCdataControlBlock),
  .mq_mem = &RCdataBuffer,
  .mq_size = sizeof(RCdataBuffer)
};
/* Definitions for IMUdata */
osMessageQueueId_t IMUdataHandle;
uint8_t IMUdataBuffer[ 1 * sizeof( IMU ) ];
osStaticMessageQDef_t IMUdataControlBlock;
const osMessageQueueAttr_t IMUdata_attributes = {
  .name = "IMUdata",
  .cb_mem = &IMUdataControlBlock,
  .cb_size = sizeof(IMUdataControlBlock),
  .mq_mem = &IMUdataBuffer,
  .mq_size = sizeof(IMUdataBuffer)
};
/* Definitions for MAGdata */
osMessageQueueId_t MAGdataHandle;
uint8_t MAGdataBuffer[ 1 * sizeof( MOTOR ) ];
osStaticMessageQDef_t MAGdataControlBlock;
const osMessageQueueAttr_t MAGdata_attributes = {
  .name = "MAGdata",
  .cb_mem = &MAGdataControlBlock,
  .cb_size = sizeof(MAGdataControlBlock),
  .mq_mem = &MAGdataBuffer,
  .mq_size = sizeof(MAGdataBuffer)
};
/* Definitions for CMDdata */
osMessageQueueId_t CMDdataHandle;
uint8_t CMDdataBuffer[ 1 * sizeof( RPYT_CMD ) ];
osStaticMessageQDef_t CMDdataControlBlock;
const osMessageQueueAttr_t CMDdata_attributes = {
  .name = "CMDdata",
  .cb_mem = &CMDdataControlBlock,
  .cb_size = sizeof(CMDdataControlBlock),
  .mq_mem = &CMDdataBuffer,
  .mq_size = sizeof(CMDdataBuffer)
};
/* Definitions for PWMdata */
osMessageQueueId_t PWMdataHandle;
uint8_t PWMdataBuffer[ 1 * sizeof( PWM_CMD ) ];
osStaticMessageQDef_t PWMdataControlBlock;
const osMessageQueueAttr_t PWMdata_attributes = {
  .name = "PWMdata",
  .cb_mem = &PWMdataControlBlock,
  .cb_size = sizeof(PWMdataControlBlock),
  .mq_mem = &PWMdataBuffer,
  .mq_size = sizeof(PWMdataBuffer)
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartRadioCommand(void *argument);
void StartReadIMU(void *argument);
void StartReadMAG(void *argument);
void StartAttitudeControl(void *argument);
void StartCyclicPitchControl(void *argument);
void StartMotorOutput(void *argument);

/* USER CODE BEGIN PFP */

void MAG_check(void );
void RC_check(void );
void IMU_check(void );
void PWM_check(void );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile unsigned long gTick = 0;

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  MAG_check();
  RC_check();
  IMU_check();
  PWM_check();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RCdata */
  RCdataHandle = osMessageQueueNew (1, sizeof(RC), &RCdata_attributes);

  /* creation of IMUdata */
  IMUdataHandle = osMessageQueueNew (1, sizeof(IMU), &IMUdata_attributes);

  /* creation of MAGdata */
  MAGdataHandle = osMessageQueueNew (1, sizeof(MOTOR), &MAGdata_attributes);

  /* creation of CMDdata */
  CMDdataHandle = osMessageQueueNew (1, sizeof(RPYT_CMD), &CMDdata_attributes);

  /* creation of PWMdata */
  PWMdataHandle = osMessageQueueNew (1, sizeof(PWM_CMD), &PWMdata_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RadioCommand */
  RadioCommandHandle = osThreadNew(StartRadioCommand, NULL, &RadioCommand_attributes);

  /* creation of ReadIMU */
  ReadIMUHandle = osThreadNew(StartReadIMU, NULL, &ReadIMU_attributes);

  /* creation of ReadMAG */
  ReadMAGHandle = osThreadNew(StartReadMAG, NULL, &ReadMAG_attributes);

  /* creation of AttitudeControl */
  AttitudeControlHandle = osThreadNew(StartAttitudeControl, NULL, &AttitudeControl_attributes);

  /* creation of CyclicPitchCont */
  CyclicPitchContHandle = osThreadNew(StartCyclicPitchControl, NULL, &CyclicPitchCont_attributes);

  /* creation of MotorOutput */
  MotorOutputHandle = osThreadNew(StartMotorOutput, NULL, &MotorOutput_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  HAL_TIM_Base_MspInit(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 17000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Base_MspInit(&htim4);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  HAL_UART_Receive_DMA(&huart1, rc_byte_data, 16);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartx){
	if(huartx->Instance == huart1.Instance){
		Spektrum_Read();

		HAL_UART_Receive_DMA(&huart1, rc_byte_data, 16);
	}
}

void RC_check(void){
	for(int check = 0; check < 10;){
		if(rc.channel[THROTTLE].pos == 342)	check++;
		else check = 0;
	}
	for(int check = 0; check < 10;){
		if(SWITCH_POS(rc.channel[AUX1].pos) == 0)	check++;
		else check = 0;
	}

}

void IMU_check(void){
	IMU imu  = { 0, };

	wt931_Init(&hi2c1);

	for(int check = 0; check < 5;){
		imu = readIMU();
		if(imu.theta != 0) check++;
		else check = 0;
	}
}

void MAG_check(void){
	MOTOR mag = {0, };

	as5147_Init(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
	//as5147_setZeroPosition();

	sens_time = 11;

	for(int check = 0; check < 20;){
		updatePosition(&mag);
		float err = mag.pre_ang - mag.rad;
		if((err >= -0.5 && err <= 0.5) && mag.rad != 0) check++;
		else check = 0;

		HAL_Delay(5);
	}
	while(mag.rad > 0.1 && mag.rad < 6.1){
		setOffset(&mag);
		updatePosition(&mag);
	}

}

void PWM_check(void){
	  htim1.Instance->CCR1 = PWM_START;
	  htim1.Instance->CCR2 = PWM_START;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRadioCommand */
/**
  * @brief  Function implementing the RadioCommand thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRadioCommand */
void StartRadioCommand(void *argument)
{
  /* USER CODE BEGIN 5 */

  RC rc_cmd = {0, };
  int imu_key = 1;

  /* Infinite loop */
  for(;;)
  {
	if(rc.channel[THROTTLE].pos != 0){
		rc_cmd.thro = map(rc.channel[THROTTLE].pos, RC_MIN, RC_MAX, PWM_MIN, PWM_MAX);     // thro command 				m/s

		rc_cmd.d_pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, 100, -100);			// d(pi)/dt command			deg/s
		rc_cmd.d_theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -100, 100);		// d(theta)/dt command		deg/s
		rc_cmd.d_psi = map(rc.channel[RUDDER].pos, RC_MIN, RC_MAX, -100, 100);			// d(psi)/dt command		deg/s

		rc_cmd.pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, 70, -70);			// d(pi)/dt command			deg/s
		rc_cmd.theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -70, 70);		// d(theta)/dt command		deg/s

		if(SWITCH_POS(rc.channel[AUX1].pos) > 0) rc_cmd.arm = 1;
		else rc_cmd.arm = 0;

		rc_cmd.mode = SWITCH_POS(rc.channel[AUX2].pos);

		if(SWITCH_POS(rc.channel[AUX3].pos) > 0 && imu_key == 1) {
			setIMUoffset();
			imu_key = 0;
		}else if(SWITCH_POS(rc.channel[AUX3].pos) == 0){
			imu_key = 1;
		}

		osMessageQueuePut(RCdataHandle, &rc_cmd, 0, 0);
	}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReadIMU */
/**
* @brief Function implementing the ReadIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadIMU */
void StartReadIMU(void *argument)
{
  /* USER CODE BEGIN StartReadIMU */

  IMU imu  = { 0, };

  wt931_Init(&hi2c1);

  /* Infinite loop */
  for(;;)
  {
	imu = readIMU();
	osMessageQueuePut(IMUdataHandle, &imu, 0, 0);

    osDelay(1);
  }
  /* USER CODE END StartReadIMU */
}

/* USER CODE BEGIN Header_StartReadMAG */
/**
* @brief Function implementing the ReadMAG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadMAG */
void StartReadMAG(void *argument)
{
  /* USER CODE BEGIN StartReadMAG */

  MOTOR mag = {0, };

  as5147_Init(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
  as5147_setZeroPosition();
  setOffset(&mag);

  sens_time = 0;
  sens_start = 0;
  osDelay(1);

  /* Infinite loop */
  for(;;)
  {
	sens_time = gTick - sens_start;
	updatePosition(&mag);
	sens_start = gTick;
	osMessageQueuePut(MAGdataHandle, &mag, 0, 0);

    osDelay(1);
  }
  /* USER CODE END StartReadMAG */
}

/* USER CODE BEGIN Header_StartAttitudeControl */
/**
* @brief Function implementing the AttitudeControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAttitudeControl */
void StartAttitudeControl(void *argument)
{
  /* USER CODE BEGIN StartAttitudeControl */
  RC rc_cmd = {0, };
  IMU imu = {0, };
  RPYT_CMD rpy_cmd = {0, };

  while(rc_cmd.arm){}

  int roll_deg = regPID(PID, 9, 2, 1.7);
  int pitch_deg = regPID(PID, 10, 2, 2.3);

  int dyaw = regPID(PI, -0.6, -1.8, 0);

  int droll = regPID(P, 0.5, 0, 0);
  int dpitch = regPID(P, 0.5, 0, 0);

  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(RCdataHandle, &rc_cmd, 0, 0);
	osMessageQueueGet(IMUdataHandle, &imu, 0, 0);

	if(rc_cmd.arm){
		switch(rc_cmd.mode){
		case 0: //rare
			rpy_cmd.roll_cmd = rc_cmd.d_pi;
			rpy_cmd.pitch_cmd = rc_cmd.d_theta;
			rpy_cmd.limit_cmd = 100;

			rpy_cmd.yaw_cmd = PWM_MIN + (rc_cmd.thro-PWM_MIN)*0.6 + rc_cmd.d_psi;
			rpy_cmd.throttle_cmd = rc_cmd.thro;
			break;
		case 1: // manual
			rpy_cmd.roll_cmd = rc_cmd.d_pi;
			rpy_cmd.pitch_cmd = rc_cmd.d_theta;
			rpy_cmd.limit_cmd = 100;

			rpy_cmd.yaw_cmd = PWM_MIN + (rc_cmd.thro-PWM_MIN)*0.6 + rc_cmd.d_psi;
			rpy_cmd.throttle_cmd = rc_cmd.thro;
			break;
		case 2: // stabilize
			rpy_cmd.roll_cmd = PIDoutput(droll, imu.P, PIDoutput(roll_deg, imu.pi, rc_cmd.pi));
			rpy_cmd.pitch_cmd = PIDoutput(dpitch, imu.Q, PIDoutput(pitch_deg, imu.theta, rc_cmd.theta));
			rpy_cmd.limit_cmd = 100;

			rpy_cmd.yaw_cmd = PWM_MIN + (rc_cmd.thro-PWM_MIN)*0.6 + PIDoutput(dyaw, imu.R, rc_cmd.d_psi);
			rpy_cmd.throttle_cmd = rc_cmd.thro;
			break;
		}
	}else{
		rpy_cmd.roll_cmd = 0;
		rpy_cmd.pitch_cmd = 0;
		rpy_cmd.limit_cmd = 0;

		rpy_cmd.yaw_cmd = PWM_MIN;
		rpy_cmd.throttle_cmd = PWM_MIN;

		resetState(roll_deg);
		resetState(pitch_deg);
		resetState(dyaw);
		resetState(droll);
		resetState(dpitch);
	}

	osMessageQueuePut(CMDdataHandle, &rpy_cmd, 0, 0);
    osDelay(1);
  }
  /* USER CODE END StartAttitudeControl */
}

/* USER CODE BEGIN Header_StartCyclicPitchControl */
/**
* @brief Function implementing the CyclicPitchCont thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCyclicPitchControl */
void StartCyclicPitchControl(void *argument)
{
  /* USER CODE BEGIN StartCyclicPitchControl */

  RPYT_CMD rpyt_cmd = {0, };
  PWM_CMD pwm_cmd = {0, };
  MOTOR mag = {0, };

  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(CMDdataHandle, &rpyt_cmd, 0, 0);
	osMessageQueueGet(MAGdataHandle, &mag, 0, 0);

	pwm_cmd.main_rotor = rpyt_cmd.throttle_cmd + (getOffset(rpyt_cmd, mag)*((rpyt_cmd.throttle_cmd-PWM_MIN)*0.003));
	pwm_cmd.tail_rotor = rpyt_cmd.yaw_cmd;

	osMessageQueuePut(PWMdataHandle, &pwm_cmd, 0, 0);

    osDelay(1);
  }
  /* USER CODE END StartCyclicPitchControl */
}

/* USER CODE BEGIN Header_StartMotorOutput */
/**
* @brief Function implementing the MotorOutput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorOutput */
void StartMotorOutput(void *argument)
{
  /* USER CODE BEGIN StartMotorOutput */
  PWM_CMD pwm_cmd = {0, };

  htim1.Instance->CCR1 = PWM_MIN;
  htim1.Instance->CCR2 = PWM_MIN;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(PWMdataHandle, &pwm_cmd, 0, 0);

    htim1.Instance->CCR1 = pwm_cmd.main_rotor;
    htim1.Instance->CCR2 = pwm_cmd.tail_rotor;

    osDelay(1);
  }
  /* USER CODE END StartMotorOutput */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {
    gTick++;
  }

  if (htim->Instance == TIM4) {
#if 0
	MOTOR mag;
    uint8_t buffer[50] = {0,};
	osMessageQueueGet(MAGdataHandle, &mag, 0, 0);
    sprintf(buffer,"%f\r\n", mag.lpf);
	HAL_UART_Transmit(&huart2, buffer, 50, HAL_TIMEOUT);
#endif
  }
  /* USER CODE END Callback 1 */
}

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

