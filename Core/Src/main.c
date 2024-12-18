/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t num=0;
uint8_t Mode,Semi,Side_Sensing,Mode_Temp;
uint8_t Speed_Level,speed1,speed2,speed3,speed4,speed5,Speed_Temp;
uint8_t Shearing,Side_Trimmer,Shearing_Value,Shearing_Value_Temp;
uint8_t Lever_Fwd,Lever_Rev,Lever_Value,Lever_Temp,Lever_Right,Lever_Left;
uint8_t Steering_Mode,All_Wheel,Crab,Zero_Turn,Width_In,Width_Out,Steering_Mode_Temp;
uint32_t Steering_Angle,Steering_Val,Steering_Angle_Temp,Steering_Val_Avg,Steering_Val_Temp,Adc;
uint8_t Uart_State_Pin=0,n1;
uint8_t count,Tx[8],ref,Rx[2],Uart_Connection,Prev_Uart_Connection,Prev_Rx,Uart_Check,Data[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Uart_State()
{
	 Uart_State_Pin=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
	
//	if(Uart_State_Pin==0){
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
//	}
//	else{
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
//	}
	Uart_Check=(Uart_State_Pin==1 || Uart_Connection==1)?1:0;
	if(Uart_Check==1 ){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	}
	else if (Uart_Check==0) {
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	}
}
void Main_Battery()
{
	HAL_UART_Receive_IT(&huart1,Rx,sizeof(Rx));
	if(Rx[0]!=1) {
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	  }                                                        // Battery level checking condition
	  else if(Rx[0]==1){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	  }
	
	
}
void Tx_Data()
	{
	Tx[0]=0xAA;
	Tx[1]=Mode;
	Tx[2]=Speed_Level;
	Tx[3]=Steering_Mode;
	Tx[4]=Steering_Angle;
	Tx[5]=Lever_Value;
	Tx[6]=Shearing_Value;
	Tx[7]=0xFF;
	count++;
	HAL_UART_Transmit_IT(&huart1 ,Tx  , sizeof(Tx) );		
}
void Modes()
	{
		Semi=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		Side_Sensing=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		Mode=((Semi==0)? 1: (Side_Sensing==0)? 3: 2);
			
		if(Mode_Temp!=Mode){
			 Tx_Data();
			 Mode_Temp=Mode;
		//	a++;
		 }

   }
void Speed()
  {
	  speed1= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
	  speed2= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
	  speed3= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	  speed4= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
	  speed5= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
	  Speed_Level=((speed1==0)? 1 : (speed2==0)? 2 : (speed3==0)? 3:(speed4==0)? 4:(speed5==0)? 5: 0);
	
	  if(Speed_Temp != Speed_Level){
		  Tx_Data();	
		  Speed_Temp=Speed_Level;
		//	a++;
	  }
  }
		void Shearing_Button()
		{
				Shearing=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
			  Side_Trimmer=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
			  Shearing_Value=((Shearing==0)? 1 :(Side_Trimmer==0)? 3 :2);
			if(Shearing_Value!=Shearing_Value_Temp){
				Tx_Data();
				Shearing_Value_Temp=Shearing_Value;
			//	a++;
				} 
	  }
		void Lever()
	{
		Lever_Fwd=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
		Lever_Left=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
		Lever_Right=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);
		Lever_Rev=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);
		Lever_Value=((Lever_Fwd==0)? 1 : (Lever_Rev==0)? 2 : (Lever_Right==0)? 4: (Lever_Left==0)? 3: 0);
		if(Lever_Temp != Lever_Value){
			Tx_Data();
			Lever_Temp=Lever_Value;
		//	a++;
		}
}
	void Steering_Modes()
	{
		All_Wheel=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
		Crab=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
		Zero_Turn=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
		Width_In=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		Width_Out=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
		Steering_Mode=((All_Wheel==0)? 1 : (Crab==0)? 2 : (Zero_Turn==0)? 3 : (Width_In==0)? 4 : (Width_Out==0)? 5 :0);
		
		if(Steering_Mode_Temp != Steering_Mode){
		  Tx_Data();
		  Steering_Mode_Temp = Steering_Mode;
	//		a++;
		}
  }
		
	void Steering()
  {

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		for(int i=0; i<=100000; i++)															// AVERAGING IT  BY / WITH 100000
	 {                                                           
		 Steering_Val += HAL_ADC_GetValue(&hadc1);
	 }  
	 Steering_Val_Avg = Steering_Val / 100000;
	 if ( ((Steering_Val_Temp+25)< Steering_Val_Avg)|| ((Steering_Val_Temp-25)>Steering_Val_Avg ))
	 {
	   Steering_Angle = (Steering_Val_Avg * 181)/4036;
		 Steering_Angle=(Steering_Angle<=0)?0:(Steering_Angle>180)?180:Steering_Angle;
		 if( Steering_Angle != Steering_Angle_Temp )
		 {
			Tx_Data();
			Steering_Angle_Temp = Steering_Angle;
		//	 a++;
		 }
		 Steering_Val_Temp = Steering_Val_Avg;		 
	 }
	 Steering_Val = 0;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	Data[0]=0xBB;
  Data[7]=0xEE;
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		Uart_State();
		Main_Battery();
		Speed();
		Modes();
		Shearing_Button();
		Lever();
		Steering_Modes();
		Steering();

//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
//		HAL_Delay(1000);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UART_OUT2_Pin|UART_OUT1_Pin|LED_MAIN_OUT2_Pin|LED_MAIN_OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UART_STATE_LED_Pin */
  GPIO_InitStruct.Pin = UART_STATE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UART_STATE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Speed_1_Pin Speed_2_Pin Speed_3_Pin Speed_4_Pin
                           Speed_5_Pin Semi_Pin Lever_Fwd_Pin Lever_Right_Pin
                           Lever_Rev_Pin */
  GPIO_InitStruct.Pin = Speed_1_Pin|Speed_2_Pin|Speed_3_Pin|Speed_4_Pin
                          |Speed_5_Pin|Semi_Pin|Lever_Fwd_Pin|Lever_Right_Pin
                          |Lever_Rev_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Side_Sensing_Pin Shearing_Pin Side_Trimmer_Pin All_Wheel_Pin
                           Crab_Pin Zero_Turn_Pin Width_In_Pin Width_Out_Pin
                           Lever_Left_Pin */
  GPIO_InitStruct.Pin = Side_Sensing_Pin|Shearing_Pin|Side_Trimmer_Pin|All_Wheel_Pin
                          |Crab_Pin|Zero_Turn_Pin|Width_In_Pin|Width_Out_Pin
                          |Lever_Left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : UART_OUT2_Pin UART_OUT1_Pin LED_MAIN_OUT2_Pin LED_MAIN_OUT1_Pin */
  GPIO_InitStruct.Pin = UART_OUT2_Pin|UART_OUT1_Pin|LED_MAIN_OUT2_Pin|LED_MAIN_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
