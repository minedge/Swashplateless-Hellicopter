/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for RC */
osThreadId_t RCHandle;
uint32_t RCBuffer[ 128 ];
osStaticThreadDef_t RCControlBlock;
const osThreadAttr_t RC_attributes = {
  .name = "RC",
  .stack_mem = &RCBuffer[0],
  .stack_size = sizeof(RCBuffer),
  .cb_mem = &RCControlBlock,
  .cb_size = sizeof(RCControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
uint32_t IMUBuffer[ 128 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_mem = &IMUBuffer[0],
  .stack_size = sizeof(IMUBuffer),
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CyclicPitch */
osThreadId_t CyclicPitchHandle;
uint32_t CyclicPitchBuffer[ 128 ];
osStaticThreadDef_t CyclicPitchControlBlock;
const osThreadAttr_t CyclicPitch_attributes = {
  .name = "CyclicPitch",
  .stack_mem = &CyclicPitchBuffer[0],
  .stack_size = sizeof(CyclicPitchBuffer),
  .cb_mem = &CyclicPitchControlBlock,
  .cb_size = sizeof(CyclicPitchControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MainLoop */
osThreadId_t MainLoopHandle;
uint32_t MainLoopBuffer[ 128 ];
osStaticThreadDef_t MainLoopControlBlock;
const osThreadAttr_t MainLoop_attributes = {
  .name = "MainLoop",
  .stack_mem = &MainLoopBuffer[0],
  .stack_size = sizeof(MainLoopBuffer),
  .cb_mem = &MainLoopControlBlock,
  .cb_size = sizeof(MainLoopControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MAG */
osThreadId_t MAGHandle;
uint32_t MAGBuffer[ 128 ];
osStaticThreadDef_t MAGControlBlock;
const osThreadAttr_t MAG_attributes = {
  .name = "MAG",
  .stack_mem = &MAGBuffer[0],
  .stack_size = sizeof(MAGBuffer),
  .cb_mem = &MAGControlBlock,
  .cb_size = sizeof(MAGControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMUQ */
osMessageQueueId_t IMUQHandle;
uint8_t IMUQBuffer[ 1 * sizeof( IMU ) ];
osStaticMessageQDef_t IMUQControlBlock;
const osMessageQueueAttr_t IMUQ_attributes = {
  .name = "IMUQ",
  .cb_mem = &IMUQControlBlock,
  .cb_size = sizeof(IMUQControlBlock),
  .mq_mem = &IMUQBuffer,
  .mq_size = sizeof(IMUQBuffer)
};
/* Definitions for PWMQ */
osMessageQueueId_t PWMQHandle;
uint8_t PWMQBuffer[ 1 * sizeof( PWM_CMD ) ];
osStaticMessageQDef_t PWMQControlBlock;
const osMessageQueueAttr_t PWMQ_attributes = {
  .name = "PWMQ",
  .cb_mem = &PWMQControlBlock,
  .cb_size = sizeof(PWMQControlBlock),
  .mq_mem = &PWMQBuffer,
  .mq_size = sizeof(PWMQBuffer)
};
/* Definitions for CMDQ */
osMessageQueueId_t CMDQHandle;
uint8_t CMDQBuffer[ 1 * sizeof( PQR_CMD ) ];
osStaticMessageQDef_t CMDQControlBlock;
const osMessageQueueAttr_t CMDQ_attributes = {
  .name = "CMDQ",
  .cb_mem = &CMDQControlBlock,
  .cb_size = sizeof(CMDQControlBlock),
  .mq_mem = &CMDQBuffer,
  .mq_size = sizeof(CMDQBuffer)
};
/* Definitions for MAGQ */
osMessageQueueId_t MAGQHandle;
uint8_t MAGQBuffer[ 1 * sizeof( MOTOR ) ];
osStaticMessageQDef_t MAGQControlBlock;
const osMessageQueueAttr_t MAGQ_attributes = {
  .name = "MAGQ",
  .cb_mem = &MAGQControlBlock,
  .cb_size = sizeof(MAGQControlBlock),
  .mq_mem = &MAGQBuffer,
  .mq_size = sizeof(MAGQBuffer)
};
/* USER CODE BEGIN PV */

IMU imu  = { 0, };
MOTOR mag = { 0, };
PQR_CMD pqr_cmd = { 0, }, pid_cmd = { 0, };			// P Q
PWM_CMD pwm_cmd = { 0, };		// main tail thro

double cyclic_control;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void StartRC(void *argument);
void StartIMU(void *argument);
void StartCyclicPitch(void *argument);
void StartMainLoop(void *argument);
void StartMAG(void *argument);

/* USER CODE BEGIN PFP */
void RC_check(void);
void IMU_check(void);
void MAG_check(void);
void PWM_check(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  MAG_check();
  RC_check();
  IMU_check();
  PWM_check();

  pwm_cmd.cyclic_pitch = PWM_MIN;
  pwm_cmd.tail = PWM_MIN;
  pwm_cmd.throttle = PWM_MIN;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);


  /*======GAIN SET===============================================================================================*/
  setAmplitudeGain(0.0002);
  setShiftGain(0.003054325);
  setPIDGain(0.45, 0.007, 0, 0);//0.01//1.05, 0.007, 1.0, 0
  setPIDGain(0.8, 0.0085, 0, 1);//0.0115//1.2, 0.0085, 1.15, 1
  setPIDGain(1.15, 0.007, 1.05, 2);
  setPIDGain(0.4, 0, 0.95, 3);//0.0115//1.2, 0.0085, 1.15, 1
  setPIDGain(0.4, 0, 0.95, 4);//0.0115//1.2, 0.0085, 1.15, 1

  /*=============================================================================================================*/

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
  /* creation of IMUQ */
  IMUQHandle = osMessageQueueNew (1, sizeof(IMU), &IMUQ_attributes);

  /* creation of PWMQ */
  PWMQHandle = osMessageQueueNew (1, sizeof(PWM_CMD), &PWMQ_attributes);

  /* creation of CMDQ */
  CMDQHandle = osMessageQueueNew (1, sizeof(PQR_CMD), &CMDQ_attributes);

  /* creation of MAGQ */
  MAGQHandle = osMessageQueueNew (1, sizeof(MOTOR), &MAGQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RC */
  RCHandle = osThreadNew(StartRC, NULL, &RC_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(StartIMU, NULL, &IMU_attributes);

  /* creation of CyclicPitch */
  CyclicPitchHandle = osThreadNew(StartCyclicPitch, NULL, &CyclicPitch_attributes);

  /* creation of MainLoop */
  MainLoopHandle = osThreadNew(StartMainLoop, NULL, &MainLoop_attributes);

  /* creation of MAG */
  MAGHandle = osThreadNew(StartMAG, NULL, &MAG_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	as5147_Init(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
	//as5147_setZeroPosition();

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

/* USER CODE BEGIN Header_StartRC */
/**
  * @brief  Function implementing the RC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRC */
void StartRC(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if(rc.channel[THROTTLE].pos != 0){
		rc.thro = map(rc.channel[THROTTLE].pos, RC_MIN, RC_MAX, PWM_MIN, PWM_MAX);     // thro command 				m/s

		rc.d_pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, -100, 100);			// d(pi)/dt command			deg/s
		rc.d_theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -100, 100);		// d(theta)/dt command		deg/s
		rc.d_psi = map(rc.channel[RUDDER].pos, RC_MIN, RC_MAX, -100, 100);			// d(psi)/dt command		deg/s

		rc.pi = map(rc.channel[AILERON].pos, RC_MIN, RC_MAX, -45, 45);			// d(pi)/dt command			deg/s
		rc.theta = map(rc.channel[ELEVATOR].pos, RC_MIN, RC_MAX, -45, 45);		// d(theta)/dt command		deg/s

		if(SWITCH_POS(rc.channel[AUX1].pos) > 0) rc.arm = 1;
		else rc.arm = 0;

		////rc.mode = SWITCH_POS(rc.channel[AUX2].pos);
	}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartIMU */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU */
void StartIMU(void *argument)
{
  /* USER CODE BEGIN StartIMU */
  /* Infinite loop */
  for(;;)
  {
	imu = readIMU();
    osDelay(1);
  }
  /* USER CODE END StartIMU */
}

/* USER CODE BEGIN Header_StartCyclicPitch */
/**
* @brief Function implementing the CyclicPitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCyclicPitch */
void StartCyclicPitch(void *argument)
{
  /* USER CODE BEGIN StartCyclicPitch */
  /* Infinite loop */
  for(;;)
  {
	float amplitude = setAmplitude(pwm_cmd, pid_cmd);		//cogging power
	float start_rad = setCyclicShift(mag, pid_cmd);

	if(start_rad == -1)
		cyclic_control = 0;
	else{
		start_rad -= ((pwm_cmd.throttle - PWM_MIN) * cyclic_shift_gain);
		cyclic_control = sin(mag.rad - start_rad);
	}

	pwm_cmd.cyclic_pitch = pwm_cmd.throttle + (cyclic_control * amplitude);

	if(pwm_cmd.tail < PWM_MIN) pwm_cmd.tail = PWM_MIN;
	else if(pwm_cmd.tail > PWM_MAX) pwm_cmd.tail = PWM_MAX;
	if(pwm_cmd.cyclic_pitch < PWM_MIN) pwm_cmd.cyclic_pitch = PWM_MIN;
	else if(pwm_cmd.cyclic_pitch > PWM_MAX) pwm_cmd.cyclic_pitch = PWM_MAX;

	if(rc.arm){
		htim1.Instance->CCR1 = pwm_cmd.cyclic_pitch;
		htim1.Instance->CCR2 = pwm_cmd.tail;
	}else{
		P_err.sum_error = 0;
		Q_err.sum_error = 0;
		R_err.sum_error = 0;

		htim1.Instance->CCR1 = PWM_MIN;
		htim1.Instance->CCR2 = PWM_MIN;
	}
    osDelay(1);
  }
  /* USER CODE END StartCyclicPitch */
}

/* USER CODE BEGIN Header_StartMainLoop */
/**
* @brief Function implementing the MainLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainLoop */
void StartMainLoop(void *argument)
{
  /* USER CODE BEGIN StartMainLoop */
  /* Infinite loop */
  for(;;)
  {
	pqr_cmd = CommandGennerate(imu, rc);
	pid_cmd = PIDController(pqr_cmd, imu);

	pwm_cmd.throttle = rc.thro;
	pwm_cmd.tail = rc.thro + pid_cmd.R_cmd;

    osDelay(1);
  }
  /* USER CODE END StartMainLoop */
}

/* USER CODE BEGIN Header_StartMAG */
/**
* @brief Function implementing the MAG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMAG */
void StartMAG(void *argument)
{
  /* USER CODE BEGIN StartMAG */
  /* Infinite loop */
  for(;;)
  {
	updatePosition(&mag);
    osDelay(1);
  }
  /* USER CODE END StartMAG */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
