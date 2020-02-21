/*
 * bsp.c
 *
 *  Created on: Feb 16, 2020
 *      Author: antiloope
 */

#include "bsp.h"
#include "stm32f4xx_hal.h"
#include "../HAL/hd44780_hal.h"
#include "../HAL/hc-sr04_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

// Handlers for both sensors
HCSR04sensor volumeSensor;

/// LCD pins defines
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RW_Pin GPIO_PIN_8
#define LCD_E_Pin GPIO_PIN_9
#define LCD_D4_Pin GPIO_PIN_10
#define LCD_D5_Pin GPIO_PIN_11
#define LCD_D6_Pin GPIO_PIN_12
#define LCD_D7_Pin GPIO_PIN_13
#define LCD_GPIO_Port GPIOE

#define BTN_0 GPIO_PIN_0
#define BTN_1 GPIO_PIN_1
#define BTN_2 GPIO_PIN_2
#define BTN_PORT GPIOA

void vError_Handler(){

}

void vGPIOInit(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();

	  __HAL_RCC_SYSCFG_CLK_ENABLE();
	  //__HAL_RCC_PWR_CLK_ENABLE();

	//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_GPIO_Port,VolumeSensor_Trigger_Pin|Led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Led_Pin */
	GPIO_InitStruct.Pin = VolumeSensor_Trigger_Pin|Led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NEXT */
	GPIO_InitStruct.Pin = BTN_0|BTN_1|BTN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn,6, 6);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_SetPriority(EXTI1_IRQn,6, 6);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_SetPriority(EXTI2_IRQn,6, 6);

	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn,6, 6);
}

static void MX_TIM3_Init(void)
{
	__TIM3_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000000-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		vError_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		vError_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		vError_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		vError_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 7;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		vError_Handler();
	}

}

static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
	__TIM9_CLK_ENABLE();
  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    vError_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    vError_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    vError_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 7;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    vError_Handler();
  }
}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
	__TIM4_CLK_ENABLE();
 /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    vError_Handler();
  }
  //HAL_TIM_Base_Start_IT(&htim4);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    vError_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    vError_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

}

static void MX_TIM5_Init(void)
{
  __TIM5_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    vError_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    vError_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    vError_Handler();
  }
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_SetPriority(TIM5_IRQn, 6, 6);

}

void vSystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    vError_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    vError_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    vError_Handler();
  }
}

float fBSPGetSensor(){
	// Get value from sensor (between 2 and 400 cm)
	float volume = fHCSR04GetDistance(&volumeSensor);
	// Cap measured values
	if(volume > 99) volume = 99;
	if(volume < 5) volume = 5;

	return volume;
}

void TIM3_IRQHandler(){
	vHCSR04CaptureCallbak(&volumeSensor);
}

void TIM5_IRQHandler(){
	if (__HAL_TIM_GET_FLAG(&htim5 , TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
	{
		if (__HAL_TIM_GET_ITSTATUS(&htim5, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
			globalCounter++;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// This function call HC-SR04 callback
	if(htim->Instance == VolumeSensor_Timer.Instance){
		vHCSR04CaptureCallbak(&volumeSensor);
	}
}

void vLCDInit(){
	char selected[] = { 0x00, 0x04, 0x0e, 0x1b, 0x0e, 0x04, 0x00, 0x00 };
	hd44780_cgram(0, selected);
	char volume_empty[] = { 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f };
	hd44780_cgram(1, volume_empty);
	char volume[] = { 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f };
	hd44780_cgram(2, volume);
	char separator[] = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
	hd44780_cgram(3, separator);


	hd44780_init(LCD_GPIO_Port, LCD_RS_Pin, LCD_RW_Pin, LCD_E_Pin, LCD_D4_Pin,
				LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin, HD44780_LINES_2,
				HD44780_FONT_5x8);
}

void vBSPDisplayPrint(uint8_t offset_line1,const char* line1,uint8_t offset_line2, const char* line2){
	hd44780_position(0, offset_line1);
	hd44780_print(line1);
	hd44780_position(1, offset_line2);
	hd44780_print(line2);
	hd44780_display(true, false, false);
}

void vBSPDisplaySymbol(uint8_t line,uint8_t column,display_symbol symbol){
	hd44780_position(line, column);
	hd44780_put(symbol);
	hd44780_display(true, false, false);
}

void vBSPDisplayClear(){
	hd44780_clear();
}


void vBSPInit(){
	// Init the HAL library
	HAL_Init();

	// Configure Clock
	vSystemClock_Config();

	// Init GPIO
	vGPIOInit();

	vLCDInit();

	// Init Timers for HC-SR04 sensor
	MX_TIM3_Init();
	MX_TIM9_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();


	// Init sensors
	volumeSensor = xHCSR04InitSensor(htim3,VolumeSensor_Trigger_GPIO_Port,VolumeSensor_Trigger_Pin);

	globalCounter = 0;
	HAL_TIM_Base_Start_IT(&htim5);
}

void vBSPLedOff(){
	HAL_GPIO_WritePin(Led_GPIO_Port,Led_Pin,GPIO_PIN_RESET);
}

void vBSPLedOn(){
	HAL_GPIO_WritePin(Led_GPIO_Port,Led_Pin,GPIO_PIN_SET);
}

// Handle button interrupts
void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BTN_2);
}

// Handle button interrupts
void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BTN_1);
}

void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BTN_0);
}

/// Define Interrupt function for GPIO
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint8_t button;
	switch(GPIO_Pin){
	case BTN_2:
		button=BUTTON_CALCULATE;
		xQueueSendToBackFromISR(ButtonsQueue,(void *)&button,pdFALSE);
		break;
	case BTN_1:
		button=BUTTON_NEW_MEASURE;
		xQueueSendToBackFromISR(ButtonsQueue,(void *)&button,pdFALSE);
		break;
	case BTN_0:
		button=BUTTON_RESET;
		xQueueSendToBackFromISR(ButtonsQueue,(void *)&button,pdFALSE);
		break;
	}
}


