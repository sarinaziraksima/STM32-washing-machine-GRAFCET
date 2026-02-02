/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
// Libraries
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

//Inputs and Conditions
bool startButton = false, stopButton = false, L0 = false, L1 = false, L2 = false, endOfWashCycle = false;
bool startButton_f = false, stopButton_f = false, L0_f = false, L1_f = false, L2_f = false;
bool system_on = false;
// outputs
bool doorLock = false, valve = false, heater = false, washerMotor = false, pump = false;
// GRAFCET Variables
bool x0 = false, x1 = false, x2 = false, x3 = false, x4 = false;
bool sx0 = false, sx1 = false, sx2 = false, sx3 = false, sx4 = false;
bool rx0 = false, rx1 = false, rx2 = false, rx3 = false, rx4 = false;
// Sub-GRAFCET Variables and Flags
bool time_f = false, iteration_complete = false, motor_en = false, sub_grafcet_f = false;

bool sub_x0 = false, sub_x1 = false, sub_x2 = false, sub_x3 = false, sub_x4 = false, sub_x5 = false, sub_x6 = false, sub_x7 = false;
bool sub_sx0 = false, sub_sx1 = false, sub_sx2 = false, sub_sx3 = false, sub_sx4 = false, sub_sx5 = false, sub_sx6 = false, sub_sx7 = false;
bool sub_rx0 = false, sub_rx1 = false, sub_rx2 = false, sub_rx3 = false, sub_rx4 = false, sub_rx5 = false, sub_rx6 = false, sub_rx7 = false;

// Motor Washing Cycle Variables
uint8_t i = 1;//Iteration Counter
int iteration = 3;

int dutycycle_left = 80;
int dutycycle_right = 20;

//UART Variables
char msg[] = "Hello, please set the Iteration and Duty Cycle\n\rfor the motor's left and right directions.\n\r\n\rExample = 1,20,50\r\n";
uint8_t tx_buffer[27] = "welcome\n\r";
uint8_t rx_index;
uint8_t rx_data[1];
uint8_t rx_buffer[100];

// Functions for States
void GRAFCET(void);
void sub_GRAFCET(void);


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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart2, rx_data, 1); // Start UART Interrupt
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);// Send a message in UART terminal
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  __WFI(); // Sleep until an interrupt
	  if (system_on == true){
		  GRAFCET();
		  system_on = false;
	  }
	  if(sub_grafcet_f){
		  sub_GRAFCET();
		  time_f = false;
		  sub_grafcet_f = false;
	  	  if((sub_x5 && !iteration_complete) || sub_x7 || sub_x0) sub_grafcet_f = true;//This ensures that the states are updated when there is no timer interupt
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000-1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 50000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//USART Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//Prevent unused arguments compilation
	UNUSED(huart);
	if (huart->Instance == USART2)
	{
		if (rx_index == 0)//Gather data and save in buffer
		{
			for (uint16_t j = 0; j < 100; j++)
				rx_buffer[j] = 0;
		}
		if (rx_data[0] != 13) // data is not finished ('\r' is not read)
		{
			rx_buffer[rx_index++] = rx_data[0];
		}
		else
		{
			// CLOSE THE STRING
			rx_buffer[rx_index] = '\0';
			HAL_UART_Transmit(&huart2, (uint8_t *)"Data updated to ", 16, 100);
			HAL_UART_Transmit(&huart2, rx_buffer, rx_index, 100);
			sscanf((char*)rx_buffer, "%d,%d,%d", &iteration, &dutycycle_left, &dutycycle_right);
			HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
			rx_index = 0;
		}
		HAL_UART_Receive_IT(&huart2, rx_data, 1);
	}
}

//Timer Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
    	//If timer interrupt happens ubdate sub-GRAFCET and timer flag
    	time_f = true;
    	sub_grafcet_f = true;
    }
}

// Input buttons Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	startButton_f = false;
	L0_f = false;
	L1_f = false;
	L2_f = false;
	stopButton_f = false;
	    if (GPIO_Pin == GPIO_PIN_8) // startButton
	    {
			startButton_f = true;
			system_on = true;
	    }
	    else if (GPIO_Pin == GPIO_PIN_9) // L0
	    {
			L0_f = true;
			system_on = true;
	    }
	    else if (GPIO_Pin == GPIO_PIN_5) // L1
	    {
			L1_f = true;
			system_on = true;
			sub_grafcet_f = true;
	    }
	    else if (GPIO_Pin == GPIO_PIN_6) // L2
	    {
			L2_f = true;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim3);
			endOfWashCycle = false;
			system_on = true;
	    }
	    else if (GPIO_Pin == GPIO_PIN_7) // stopButton
	    {
	    	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	    	HAL_TIM_Base_Stop_IT(&htim3);
			stopButton_f = true;
			system_on = true;
			sub_x0 = false;
			sub_x1 = false;
			sub_x2 = false;
			sub_x3 = false;
			sub_x4 = false;
			sub_x5 = false;
			sub_x6 = false;
			sub_x7 = false;
	    }
}

// GRAFCET for the system
void sub_GRAFCET(void){

	  //Stage 1 read inputs (no input)

	  //Stage 2 update the set of all states
	  sub_sx0 = (!sub_x0 && !sub_x1 && !sub_x2 && !sub_x3 && !sub_x4 && !sub_x5 && !sub_x6 && !sub_x7);
	  sub_sx1 = (sub_x0 && motor_en) || (sub_x5 && !iteration_complete);
	  sub_sx2 = sub_x1 && time_f;
	  sub_sx3 = sub_x2 && time_f;
	  sub_sx4 = sub_x3 && time_f;
	  sub_sx5 = sub_x4 && time_f;
	  sub_sx6 = sub_x5 && time_f && iteration_complete;
	  sub_sx7 = sub_x6 && time_f;

	  //Stage 3 update the reset of all states
	  sub_rx0 = sub_x0 && motor_en;
	  sub_rx1 = (sub_x1 && time_f) || (sub_x1 && stopButton);
	  sub_rx2 = (sub_x2 && time_f) || (sub_x2 && stopButton);
	  sub_rx3 = (sub_x3 && time_f) || (sub_x3 && stopButton);
	  sub_rx4 = (sub_x4 && time_f) || (sub_x4 && stopButton);
	  sub_rx5 = (sub_x5 && time_f && iteration_complete) || (sub_x5 && stopButton) || (sub_x5 && !iteration_complete);
	  sub_rx6 = (sub_x6 && time_f) || (sub_x6 && stopButton);
	  sub_rx7 = (sub_x7 && L2) || (sub_x7 && stopButton);

	  //Stage 4 update the status of all states
	  sub_x0 = sub_sx0 || (sub_x0 && !sub_rx0);
	  sub_x1 = sub_sx1 || (sub_x1 && !sub_rx1);
	  sub_x2 = sub_sx2 || (sub_x2 && !sub_rx2);
	  sub_x3 = sub_sx3 || (sub_x3 && !sub_rx3);
	  sub_x4 = sub_sx4 || (sub_x4 && !sub_rx4);
	  sub_x5 = sub_sx5 || (sub_x5 && !sub_rx5);
	  sub_x6 = sub_sx6 || (sub_x6 && !sub_rx6);
	  sub_x7 = sub_sx7 || (sub_x7 && !sub_rx7);


	  // Stage 5 calculate output and write output ( set the PWM Duty Cycle, PWM frequency, and Timer)

	  //Set the PWM Output (Duty Cycle)
	  if (!(sub_x1 || sub_x3 || sub_x6))
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	  else if(sub_x3)
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutycycle_right*10-1);//DC = 20
	  else if(sub_x6)
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500-1);//DC = 50
	  else if(sub_x1)
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dutycycle_left*10-1);//DC = 80 left

	  // start timer and pwm
	  if(sub_x1){
	      HAL_TIM_Base_Start_IT(&htim2);
		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  }

	  // Set Period of the Timer and PWM frequency (Sets time between each state)
	  if (sub_x2 || sub_x4 || (sub_x5 && iteration_complete)){
		  htim3.Instance->PSC = 48000;
		  htim3.Instance->EGR = TIM_EGR_UG;
		  __HAL_TIM_SET_AUTORELOAD(&htim2, 1500000-1);//1.5s timer//0.6 0.2
		  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  }
	  else if (sub_x1 || sub_x3){
		  htim3.Instance->PSC = 48000;
		  htim3.Instance->EGR = TIM_EGR_UG;
		  __HAL_TIM_SET_AUTORELOAD(&htim2, 3000000-1);//3s timer
		  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  }
	  else if (sub_x6){
		  htim3.Instance->PSC = 16000;
		  htim3.Instance->EGR = TIM_EGR_UG;
		  __HAL_TIM_SET_AUTORELOAD(&htim2, 7000000-1);//7s timer
		  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  }
	  else if (sub_x7){
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		  HAL_TIM_Base_Stop_IT(&htim2);
		  endOfWashCycle = true;
		  system_on = true;
	  }

	  //Calculate and the Iteration and set the flag
	  if(sub_x3 && i<iteration+1){
		  i++;
		  iteration_complete = false;
	  }
	  if(i==iteration+1){
		  i = 1;
		  iteration_complete = true;
	  }


  }

void GRAFCET(void){

	  //stage 1 read inputs and connect the outputs (we have 5 inputs)
	  L0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) || L0_f;//PB9 - D14 - L0
	  L1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) || L1_f;//PA5 - D13 - L1
	  L2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) || L2_f;//PA6 - D12 - L2
	  startButton = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) || startButton_f;//PB8 - D15 - startButton
	  stopButton = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) || stopButton_f;//PA7 - D11 - stopButton

	  //stage 2 update the set of all states
	  sx0 = (!x0 && !x1 && !x2 && !x3 && !x4) || (x4 && L2);
	  sx1 = x0 && startButton;
	  sx2 = x1 && L0;
	  sx3 = x2 && L1;
	  sx4 = (x3 && stopButton) || (x3 && endOfWashCycle);

	  //stage 3 update the reset of all states
	  rx0 = x0 && startButton;
	  rx1 = x1 && L0;
	  rx2 = x2 && L1;
	  rx3 = (x3 && stopButton) || (x3 && endOfWashCycle);
	  rx4 = x4 && L2;
	  sub_sx6 = !(x4 && L2);

	  //stage 4 update the status of all states
	  x0 = sx0 || (x0 && !rx0);
	  x1 = sx1 || (x1 && !rx1);
	  x2 = sx2 || (x2 && !rx2);
	  x3 = sx3 || (x3 && !rx3);
	  x4 = sx4 || (x4 && !rx4);

	  //stage 5 calculate output and write output
	  doorLock = x1||x2||x3||x4;
	  valve = x1;
	  heater = x2;
	  motor_en = x3;
	  pump = x4;

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, doorLock);//PA0 - A0
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, valve);//PA1 - A1
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, heater);//PA4 - A2
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, pump);//PC1 - A4

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
