/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EndeffAddress (0x23<<1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t test = 0;


uint8_t EmergencyArray[2] = {1,1};
uint8_t ButtonArray[2] = {1,1};  //[Now, Last] = {UP, UP}
uint64_t _micros = 0;
uint64_t Timestamp = 0;
uint64_t HomeTimestamp = 0;
uint64_t LaserTimestamp = 0;
uint64_t LaserDelay = 5000000;
uint8_t  HomeMode = 0;
uint8_t ProxiArray[2]   = {1,1};

int PWMOut = 0;

uint16_t RobotArm_Position = 0;

float dt    = 0.001;
float K_P	= 1200;
float K_I	= 650;
float K_D	= 0;

float a=0.9;float w=0.103;
float rad=0;float raw=0;
float Q=0;float R=0;float theta_est=0;float omega_est=0;float dt1=0.01;float theta_pd=0;float y=0;float p11=0;
float p12=0;float p21=0;float p22=0;float omega_pd=0;float a0=0;float a1=0;float a2=0;float a3=0;float a4=0;float a5=0;
float sb=0;float sa=0;float tf=0;float vb=0;float sbf=0;float t=0;float Vmax=0;float vcon =0;
float p=0;
float i=0;
float d=0;
float pre_error=0;float error=0;
uint8_t push=0;float n=0;float angle=0;float kalman_theta=0;float rb_pos=0;float float_encode=0;
uint8_t flag_case = 0;
float get_station=0;float get_position=0;float now_postion=0;

uint8_t reverse = 0;
float distance = 0;

uint8_t LaserOpenCommand[1] = {0x45};

typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail;

} UARTStucrture;

UARTStucrture UART2 = { 0 };

float pi = 3.14159265359;

float    Lastest_Angle = 0;
float    Current_Angle = 0;
uint8_t  Current_Station = 0;
uint8_t  Current_Multi_Station = 0; // 1-15
uint8_t  Clockwise = 0;
uint8_t  Goal_Mode = 0; // 1-3
float    Speed = 0;
uint8_t   s = 0;
uint8_t ak[2] = {0};

uint8_t  Serial_Mode;
uint16_t Serial_Angle;
uint8_t  Serial_Speed;

uint16_t Delay = 0;
uint8_t Emergency = 0;
uint8_t Run  = 0;
uint8_t Home = 0;
uint8_t SpecialHome = 0;
uint8_t Laser = 0;
uint8_t MCU_Connected = 0;
uint8_t EndEff_Enable = 0;
float Max_Speed = 0;
float Goal_Angle = 0;
uint8_t Single_Station = 0;

int16_t InputChar      	= 0;
uint8_t InputByte 		= 0;
uint8_t ChkSum			= 0;
uint8_t Data_Frame2[2]  = {0};
uint8_t Data_Frame3[8]  = {0};

uint8_t Multi_Station[16] = {0};
uint8_t Multi_Station_Amount = 0;
uint8_t Multi_Station_Current = 0;

//uint16_t Multi_Station_Angle[11] = {0};
//                                        1 2 3  4  5   6   7   8   9   10
uint16_t Multi_Station_Angle[11] = {0,    0,5,45,65,90,180,210,270,325,355};

uint8_t UART_Mode = 0;

uint8_t UART_Mode_Print = 0;

uint8_t UART_Flow2_Ack = 0;

typedef enum
{
	Start,

	Frame1_CheckSum,

	Frame2_Data1,
	Frame2_Data2,
	Frame2_CheckSum,

	Frame3_Station,
	Frame3_Data,
	Frame3_CheckSum,

} ProtocalState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void NucleoCheck();
void EmergencyCheck();
uint64_t micros();
void ProxiCheck();
void MotorDrive();
void SetHome();
uint32_t EncoderPosition_Update();
float EncoderVelocity_Update();
void ReachGoal();
void pid();
void kalmanfilter();
void planning();

void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);

void UART_Ack1();
void UART_Ack2();
void UART_Flow2();
void UART_Protocal();
void UART_Execute();

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1)
  {
	  HomeMode = 10;
	  SetHome();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /////////////////////////////////////////////////////////////
  while (1)
  {
	  if (micros() - Timestamp >= 10000) //10000us = 0.01s = 100Hz
	  {
		  Timestamp = micros();

		  NucleoCheck();
		  EmergencyCheck();

		  if(UART_Flow2_Ack)
		  {
			while(1)
			{
				while(1)
				{
					InputChar = UARTReadChar(&UART2);
					if(InputChar != -1)
					{
						InputByte = (uint8_t)InputChar;
						ak[0] = InputByte;
						break;
					}
				}
				while(1)
				{
					InputChar = UARTReadChar(&UART2);
					if(InputChar != -1)
					{
						InputByte = (uint8_t)InputChar;
						ak[1] = InputByte;
						break;
					}
				}
				if(ak[0] == 'X' && ak[1] == 'u')
				{
					break;
				}
			}
		  }
		  else
		  {
			  UART_Protocal();
		  }

		  //----------------------------------------------

		  if(Laser)
		  {
			  if(EndEff_Enable) { LaserDelay = 5000000; } //5000000us = 5s
			  else              { LaserDelay = 1000000; } //1000000us = 1s

			  if (micros() - LaserTimestamp >= LaserDelay)
			  {
				  if(Goal_Mode == 3)
				  {
					  if(Current_Multi_Station == Multi_Station_Amount-1)
					  {
						  UART_Ack2();
						  Goal_Mode = 0;
					  }
					  else
					  {
						  Current_Multi_Station += 1;
						  angle = Multi_Station_Angle[ Multi_Station[Current_Multi_Station] ];
						  Lastest_Angle = Current_Angle;
						  Run = 1;
					  }
				  }
				  else
				  {
					  UART_Ack2();
					  Goal_Mode = 0;
				  }
				  Laser = 0;
				  t = 0;
			  }
		  }

		  if(Emergency)
		  {
			  // Do Nothing
		  }
		  else if(Run)
		  {
			  if(angle == 0)
			  {
				  angle = 15;
				  SpecialHome = 1;
			  }
			  else
			  {
				  RobotArm_Position = EncoderPosition_Update();
				  EncoderVelocity_Update();
				  planning();
				  kalmanfilter();
				  pid();
				  MotorDrive();
			  }
		  }

		  else if(Home)
		  {
			  RobotArm_Position = EncoderPosition_Update();
			  kalmanfilter();
			  SetHome();
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /////////////////////////////////////////////////////////////
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 50000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 99;
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

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//******************************************************************

void NucleoCheck()
{
	ButtonArray[1] = ButtonArray[0];
	ButtonArray[0] = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

	if(ButtonArray[0]==1 && ButtonArray[1]==0) //When Released Button
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void EmergencyCheck()
{
	EmergencyArray[1] = EmergencyArray[0];
	EmergencyArray[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

	if(EmergencyArray[0]==0 && EmergencyArray[1]==1) //When Pressed Emergency
	{
		Emergency = 1;
		PWMOut = 0;
	}
	else if(EmergencyArray[0]==1 && EmergencyArray[1]==0) //When Released Emergency
	{
		Emergency = 0;
		Lastest_Angle = Current_Angle;
		t = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}

uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
}

void ProxiCheck()
{
	ProxiArray[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	ProxiArray[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

void MotorDrive()
{
	if(PWMOut >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWMOut);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -PWMOut);
	}
}

void SetHome()
{
	if(HomeMode == 1)
	{
		PWMOut = 750;
		MotorDrive();
		if (micros() - HomeTimestamp >= 200000) //200000us = 0.2s
		{
			HomeMode = 2;
		}
	}
	else if(HomeMode == 2)
	{
		PWMOut = -2500;
		MotorDrive();
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
		{
			HomeMode = 3;
		}
	}
	else if(HomeMode == 3)
	{
		PWMOut = -600;
		MotorDrive();
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
		{
			HomeMode = 4;
			HomeTimestamp = micros();
		}
	}

	else if(HomeMode == 4)
	{
		PWMOut = 0;
		MotorDrive();
		if (micros() - HomeTimestamp >= 500000) //500000us = 0.5s
		{
			htim1.Instance->CNT = 0;
			HomeMode = 0;
			Home = 0;
			Lastest_Angle = 0;
			if(Goal_Mode != 0)
			{
				SpecialHome = 0;
				ReachGoal();
			}
		}
	}

	else if(HomeMode == 10)
	{
		PWMOut = 750;
		MotorDrive();
		HAL_Delay(200);

		PWMOut = -2500;
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)
		{
			MotorDrive();
		}

		PWMOut = -600;
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1)
		{
			MotorDrive();
		}

		PWMOut = 0;
		MotorDrive();
		HAL_Delay(500);
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
		HomeMode = 0;
	}
}

#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 3600
#define  MAX_ENCODER_PERIOD 7200

uint32_t EncoderPosition_Update()
{
	return HTIM_ENCODER.Instance->CNT;
}

float EncoderVelocity_Update()
{   static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();
	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;
	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{EncoderPositionDiff -= MAX_ENCODER_PERIOD;}
	else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{EncoderPositionDiff += MAX_ENCODER_PERIOD;}
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;
	raw =(float)(EncoderPositionDiff * 1000000.00) / (float) EncoderTimeDiff;
	rad = raw* 0.05*2.00*3.141592/360.00;
	return  rad;
}

void kalmanfilter()
{    Q = pow(a,2);
	 R = pow(w,2);
	 theta_est = theta_pd + omega_pd*dt1;
	 omega_est = omega_pd;
	 y = (rad-omega_est);

    p11 = p11 + dt1*p21+ (Q*pow(dt1,4))/4 + (pow(dt1,2))*(p12+dt1*p22)/dt1;
    p12 = p12 + dt1*p22 + (Q*dt1*pow(dt1,2))/2;
    p21 = (2*dt1*p21+Q*pow(dt1,4) + 2*p22*pow(dt1,2))/(2*dt1);
    p22 = Q*pow(dt1,2)+p22;

    theta_est+= (p12*y)/(R+p22);
    omega_est+= (p22*y)/(R+p22);

    p11=p11-(p12*p21)/(R+p22);
    p12=p12-(p22*p21)/(R+p22);
    p21=-p21*(p22/(R+p22)-1);
    p22=-p22*(p22/(R+p22)-1);

    theta_pd=theta_est;
    omega_pd=omega_est;

    kalman_theta=(float)(theta_est*57.2958);
}

void planning()
{
  t=t+0.01;
  Vmax = Max_Speed * 0.10472 * 0.9;     //rad/s
  sb = angle*0.0174533;           //degree -> rad
  sa = Lastest_Angle * 0.0174533; //degree -> rad

  if(sb < sa) { reverse = 1; distance = Lastest_Angle - angle; tf = 15.00*(sa-sb)/(8.00*Vmax); }
  else        { reverse = 0; distance = angle - Lastest_Angle; tf = 15.00*(sb-sa)/(8.00*Vmax); }

  if (distance <=30)
  { flag_case = 3; }

  if (distance > 30 && distance <=60)
  { flag_case = 4; }

  if (distance > 60 && distance <=90)
  { flag_case = 5; }

  if (distance > 90){

	  flag_case = 2;
	  if(0.5>=(5.7335*(sb-sa)/(pow(tf,2))))  //check accerelation
	  {tf=tf;}
	  else{tf=pow((5.7335*(sb-sa)/0.5),0.5);}
	  a0=0;
	  a1=0;
	  a2=0;
	  a3= 10.00*(sb-sa)/(pow(tf,3));
	  a4= -15.00*(sb-sa)/(pow(tf,4));
	  a5= 6.00*(sb-sa)/(pow(tf,5));
	  if(t<=tf){
		  sbf =  a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
		  vb= (float)((3*a3*pow(t,2))+(4*a4*pow(t,3))+(5*a5*pow(t,4)));}
	  else { vb=0; }
  }
}

void ReachGoal()
{
	if(SpecialHome)
	{
	  PWMOut=0;
	  MotorDrive();
	  Run = 0;
	  Home = 1;
	  HomeMode = 1;
	  HomeTimestamp = micros();
	}
	else
	{
		omega_est = 0;
		PWMOut=0;
		MotorDrive();
		Run=0;
		if(EndEff_Enable) { HAL_I2C_Master_Transmit_IT(&hi2c1, EndeffAddress, LaserOpenCommand, 1); }
		Laser = 1;
		LaserTimestamp = micros();
	}
}


void pid()
{
	if  (flag_case == 2)
	{
		 error = vb-omega_est;
		 p = (error);
		 i = i+error;
		 d = error - pre_error;
		 pre_error = error;
		 PWMOut =( (p*K_P)+(i*K_I)+(d*K_D));

		 if(vb==0)
		 {
			if(reverse)
			{
				if((uint16_t)(angle*20) < 30*20 && (RobotArm_Position) > 330*20)
				{
					PWMOut=600;
				}
				else if((RobotArm_Position) < (uint16_t)(angle*20))
				{
					PWMOut=400;
				}
				else if((RobotArm_Position) > (uint16_t)(angle*20))
				{
					PWMOut=-1000;
				}
			}
			else
			{
				if((uint16_t)(angle*20) > 330*20 && (RobotArm_Position) < 30*20)
				{
					PWMOut=-600;
				}
				else if((RobotArm_Position) < (uint16_t)(angle*20))
				{
					PWMOut=1000;
				}
				else if((RobotArm_Position) > (uint16_t)(angle*20))
				{
					PWMOut=-400;
				}
			}

			if((RobotArm_Position) == (uint16_t)(angle*20))
			{
				ReachGoal();
			}
		 }
	}

	else if (flag_case == 3)
	{
		if(reverse)
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=400;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-1000;
			}
		}
		else
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=1000;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-400;
			}
		}

		if((RobotArm_Position) == (uint16_t)(angle*20))
		{
			ReachGoal();
		}
	}
	else if (flag_case == 4)
	{
		if(reverse)
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=300;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-1500;
			}
		}
		else
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=1500;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-300;
			}
		}


		if((RobotArm_Position) == (uint16_t)(angle*20))
		{
			ReachGoal();
		}
	}
	else if (flag_case == 5)
	{
		if(reverse)
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=300;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-1600;
			}
		}
		else
		{
			if((RobotArm_Position) < (uint16_t)(angle*20))
			{
				PWMOut=1600;
			}
			else if((RobotArm_Position) > (uint16_t)(angle*20))
			{
				PWMOut=-300;
			}
		}

		if((RobotArm_Position) == (uint16_t)(angle*20))
		{
			ReachGoal();
		}
	}
}

void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}

uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}

int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;
}

void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);
}

void UART_Ack1()
{
	uint8_t temp[1];
	temp[0] = 'X'; //0x58
	UARTTxWrite(&UART2, temp, 1);
	HAL_Delay(1);
	temp[0] = 'u'; //0b01110101
	UARTTxWrite(&UART2, temp, 1);
	// HAL_Delay(1);
}

void UART_Ack2()
{
	uint8_t temp[1];
	temp[0] = 'F'; //70
	UARTTxWrite(&UART2, temp, 1);
	HAL_Delay(1);
	temp[0] = 'n'; //0o156
	UARTTxWrite(&UART2, temp, 1);
	// HAL_Delay(1);
}

void UART_Flow2()
{
	Serial_Mode  = (0b10010000 | UART_Mode);
	Serial_Angle = (uint16_t)(Current_Angle * 10000 * pi / 180);
	Serial_Speed = (uint8_t) (Speed * 255 / 10);

	ChkSum = Serial_Mode;

	uint8_t temp[1];
	temp[0] = Serial_Mode;
	UARTTxWrite(&UART2, temp, 1);
	HAL_Delay(1);

	if(UART_Mode == 10) { temp[0] = (uint8_t)(Serial_Angle / 256); }
	else                { temp[0] = 0; }
	ChkSum += temp[0];
	UARTTxWrite(&UART2, temp, 1);
	HAL_Delay(1);

	if(UART_Mode == 9)       { temp[0] = Current_Station; }
	else if(UART_Mode == 10) { temp[0] = (uint8_t)(Serial_Angle % 256); }
	else if(UART_Mode == 11) { temp[0] = Serial_Speed; }
	ChkSum += temp[0];
	UARTTxWrite(&UART2, temp, 1);
	HAL_Delay(1);

	ChkSum = ~(ChkSum);
	temp[0] = ChkSum;
	UARTTxWrite(&UART2, temp, 1);
	// HAL_Delay(1);
}


void UART_Protocal()
{
	static ProtocalState State = Start;

	InputChar = UARTReadChar(&UART2);

	if (InputChar != -1)
	{
		InputByte = (uint8_t)InputChar;

		switch(State)
		{
			case Start:
				if( ((InputByte>>4) & 0b00001111) == 0b00001001)
				{
					UART_Mode = InputByte & 0b00001111;
					UART_Mode_Print = UART_Mode;
				}
				if(UART_Mode >=1 && UART_Mode <= 14)
				{

					if(UART_Mode == 2 || UART_Mode == 3 || UART_Mode >= 8)
					{
						State = Frame1_CheckSum;
					}
					else if(UART_Mode == 1 || UART_Mode == 4 || UART_Mode == 5 || UART_Mode == 6)
					{
						State = Frame2_Data1;

					}
					else if(UART_Mode == 7)
					{
						State = Frame3_Station;
					}
				}
				break;

			case Frame1_CheckSum:
				ChkSum = ~(0b10010000 | UART_Mode);
				if(InputByte == ChkSum)
				{
					UART_Ack1();
					UART_Execute();
				}
				ChkSum = 0;
				State = Start;
				break;

			case Frame2_Data1:
				Data_Frame2[0] = InputByte;
				State = Frame2_Data2;
				break;

			case Frame2_Data2:
				Data_Frame2[1] = InputByte;
				State = Frame2_CheckSum;
				break;

			case Frame2_CheckSum:
				ChkSum = ~( (0b10010000 | UART_Mode) + Data_Frame2[0] + Data_Frame2[1] );
				if(InputByte == ChkSum)
				{
					UART_Ack1();
					UART_Execute();
				}
				ChkSum = 0;
				State = Start;
				break;

			case Frame3_Station:
				Multi_Station_Amount = InputByte;
				State = Frame3_Data;
				break;

			case Frame3_Data:
				Data_Frame3   [(int)Multi_Station_Current/2] = InputByte;
				Multi_Station [Multi_Station_Current]        = InputByte & 0b00001111;
				Multi_Station [Multi_Station_Current+1] 	 = InputByte >> 4;
				Multi_Station_Current += 2;
				if(Multi_Station_Current >= Multi_Station_Amount)
				{
					Multi_Station_Current = 0;
					State = Frame3_CheckSum;
				}
				break;

			case Frame3_CheckSum:
				ChkSum = (0b10010000 | UART_Mode);
				ChkSum += Multi_Station_Amount;
				for(int i=0 ; i < (int)((Multi_Station_Amount+1)/2) ; i++)
				{
					ChkSum += Data_Frame3[i];
				}
				ChkSum = ~(ChkSum);
				if(InputByte == ChkSum)
				{
					UART_Ack1();
					UART_Execute();
				}
				ChkSum = 0;
				State = Start;
				break;

			default:
				State = Start;
				break;
		}
	}

}

void UART_Execute()
{
	switch(UART_Mode)
	{
		case 1:
			break;
		case 2:
			MCU_Connected = 1;
			break;
		case 3:
			MCU_Connected = 0;
			break;
		case 4:
			Max_Speed = Data_Frame2[1] * 10 / 255;
			break;
		case 5:
			Goal_Mode = 1;
			Goal_Angle = (Data_Frame2[0] * 256) + Data_Frame2[1];
			Goal_Angle = Goal_Angle / (pi * 10000) * 180.0;
			angle = round(Goal_Angle);
			Lastest_Angle = Current_Angle;
			break;
		case 6:
			Goal_Mode = 2;
			Single_Station = Data_Frame2[1];
			angle = Multi_Station_Angle[Single_Station];
			Lastest_Angle = Current_Angle;
			break;
		case 7:
			Goal_Mode = 3;
			Current_Multi_Station = 0;
			angle = Multi_Station_Angle[ Multi_Station[Current_Multi_Station] ];
			Lastest_Angle = Current_Angle;
			break;
		case 8:
			t = 0;
			Run = 1;
			break;
		case 9:
			UART_Flow2();
			break;
		case 10:
			Current_Angle = ( EncoderPosition_Update() * 360.0 / 7200.0 );
			UART_Flow2();
			break;
		case 11:
			Speed = ( omega_est * 9.5493 );
			if(Speed < 0) { Speed = 0 - Speed; }
			UART_Flow2();
			break;
		case 12:
			EndEff_Enable = 1;
			break;
		case 13:
			EndEff_Enable = 0;
			break;
		case 14:
			Home = 1;
			HomeMode = 1;
			HomeTimestamp = micros();
			break;
		case 15:
			break;
	}
	UART_Mode = 0;
}

//******************************************************************
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
