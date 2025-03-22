/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "lcd_16x2.h";
#include "MAX31865_lib.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TUANAP HAL_GPIO_ReadPin(GPIOB, Tuanap_Pin)
#define MODE HAL_GPIO_ReadPin(GPIOB, MODE_Pin)
#define OKE HAL_GPIO_ReadPin(GPIOB, OKE_Pin)
float PT100_Temperature = 0.0f;
unsigned long thoigian;
char chuoi[12];
char chuoi1[12];
int cong=0;
bool zero_cross_detected = false;
bool tuanap = 0;
bool mode = 0;
bool oke = 0;
bool oke1 = 0;
bool fix = 0;
bool fix_oke=0;
int max_firing_delay =8300;
uint8_t sotuan = 1;

 float setpoint = 40.0f;
//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp = 300;
float ki = 3;
int kd = 400;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t doC[8] = {0x07,0x05,0x07,0x00,0x00,0x00,0x00,0x00};
uint8_t pixeltemp[8] = {0x04,0x0A,0x0A,0x0A,0x15,0x1F,0x15,0x0E};
uint16_t dem = 0;
uint16_t dem1 = 0;
uint16_t dem2 = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  Lcd_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
Lcd_clear_display();
//Lcd_write_string("Khanh dep trai");
//HAL_Delay(500);
MAX31865_Init(3);
Lcd_create_custom_char(0, doC);
Lcd_create_custom_char(1, pixeltemp);
HAL_TIM_Base_Start(&htim2);
//Lcd_clear_display();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Lcd_gotoxy(1, 0);
	  Lcd_write_string("BUONG AP TRUNG");
	  Lcd_gotoxy(1, 1);
	  Lcd_write_string("TUAN AP");
	  Lcd_gotoxy(9, 1);
	  Lcd_write_string("|");
	  Lcd_gotoxy(11, 1);
	  Lcd_write_string("MODE");
		  if(TUANAP == 0)
			  {
				  while(TUANAP==0);
				  Lcd_clear_display();
				  tuanap = 1;
			  }
		  if(MODE == 0)
		  {
			  while(MODE ==0);
			  Lcd_clear_display();
			  mode = 1;
			  Lcd_gotoxy(1,0);
			  Lcd_write_string("CAI DAT: ");
			  Lcd_gotoxy(11,0);
			  Lcd_write_int(setpoint);
			  Lcd_write_custom_char(13, 0, 0);
			  Lcd_gotoxy(14,0);
			  Lcd_write_string("C");
		  }
		  while(mode == 1)
		  {
			  Lcd_gotoxy(11,0);
			  Lcd_write_int(setpoint);
			  Lcd_gotoxy(3,1);
			  Lcd_write_string("START = OKE");
			  if(OKE == 0)
			  			  {
			  				  while(OKE ==0);
			  				  Lcd_clear_display();
			  				  mode = 0;
			  				Lcd_gotoxy(1,1);
			  				Lcd_write_string("CAI DAT: ");
			  				Lcd_gotoxy(10,1);
			  				Lcd_write_int(setpoint);
			  				Lcd_write_custom_char(13, 1, 0);
			  				Lcd_gotoxy(14,1);
			  				Lcd_write_string("C");
			  				  oke1 = 1;
			  				  break;
			  			  }
		  }
		  while(oke1 ==1)
		  {
			  PID_Temp();
			  Lcd_gotoxy(10,1);
			  Lcd_write_int(setpoint);
		  }
		  while(tuanap == 1)
		  {
			  Lcd_gotoxy(0,0);
			  Lcd_write_string("TUAN AP=");
			  Lcd_gotoxy(8,0);
			  Lcd_write_int(sotuan);
			  Lcd_gotoxy(9,0);
			  Lcd_write_string("|");
			  Lcd_write_custom_char(10,0, 1);
			  Lcd_gotoxy(11,0);
			  Lcd_write_string("=");

			  Lcd_gotoxy(3,1);
			  Lcd_write_string("START = OKE");
			  if(TUANAP == 0)
			  {
				  while(TUANAP==0);
				  sotuan++;
				  if(sotuan >3)
				  {
				  	sotuan = 1;

				  }


			  }
			  if(sotuan ==1)
			  {
			 	setpoint = 37.8;
			 	Lcd_gotoxy(12,0);
			 	sprintf(chuoi1,"%.2f",setpoint);
			 	Lcd_write_string(chuoi1);
			 	}
			  else if(sotuan == 2)
			 	{
			 		setpoint = 37.5;
			 		Lcd_gotoxy(12,0);
			 		sprintf(chuoi1,"%.2f",setpoint);
			 		Lcd_write_string(chuoi1);
			 	}
			 else if(sotuan == 3)
			 	{
			 	setpoint = 37.2;
			 	Lcd_gotoxy(12,0);
			 	sprintf(chuoi1,"%.2f",setpoint);
			 	Lcd_write_string(chuoi1);
			 	}
			  if(OKE == 0)
			  {
				  while(OKE ==0);
				  Lcd_clear_display();
				  tuanap = 0;
				  oke = 1;
				  Lcd_gotoxy(0, 1);
				  Lcd_write_string("SET_TEMP: ");
				  Lcd_gotoxy(10,1);
				  sprintf(chuoi1,"%.1f",setpoint);
				  Lcd_write_string(chuoi1);
				  Lcd_write_custom_char(14, 1, 0);
				  	Lcd_gotoxy(15,1);
				  	Lcd_write_string("C");
				  break;
			  }
		  }
		  while(oke == 1)
		  {

			  PID_Temp();

		  }

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  HAL_GPIO_WritePin(GPIOA, CS_Pin|firing_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin firing_pin_Pin */
  GPIO_InitStruct.Pin = CS_Pin|firing_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TANG_Pin GIAM_Pin */
  GPIO_InitStruct.Pin = TANG_Pin|GIAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : zero_coss_Pin */
  GPIO_InitStruct.Pin = zero_coss_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(zero_coss_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Tuanap_Pin OKE_Pin MODE_Pin */
  GPIO_InitStruct.Pin = Tuanap_Pin|OKE_Pin|MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ham PID nhietdo
void PID_Temp()
{
	if (HAL_GetTick() - thoigian >= 500) {
			  timePrev = Time;
			  thoigian = HAL_GetTick();
		  			PT100_Temperature = MAX31865_Get_Temperature();

		  			PID_error = setpoint - PT100_Temperature ;  //Calculate the pid ERROR

		  			    if (PID_error > 3.1)  //integral constant will only affect errors below 30ºC
		  			    { PID_i = 0; }

		  			    PID_p = kp * PID_error;            //Calculate the P value
		  			    PID_i = PID_i + (ki * PID_error);  //Calculate the I value
		  			   // timePrev = Time;                   // the previous time is stored before the actual time read
		  			    Time = HAL_GetTick();                   // actual time read
		  			    elapsedTime = (Time - timePrev) / 1000;
		  			    PID_d = kd * ((PID_error - previous_error) / elapsedTime);  //Calculate the D value
		  			    PID_value = PID_p + PID_i + PID_d;                          //Calculate total PID value

		  			    //We define firing delay range between 0 and 8300. Read above why 8300!!!!!!!
		  			    if (PID_value < 0) { PID_value = 0; }
		  			    if (PID_value > 8300) { PID_value = 8300; }
		  			    realTemp_LCD();
		  			  previous_error = PID_error;
		  }
				  if (zero_cross_detected) {
					  delay_us(max_firing_delay - PID_value);
					  if((PT100_Temperature >= setpoint-0.2)&& fix==0)
			  					  {
			  						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			  						HAL_Delay(3000);
			  						fix = 1;
			  						fix_oke = 1;
			  					  }
					  if(PT100_Temperature >= setpoint && fix_oke == 1){
			  					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			  					  }else{HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);}
			  			delay_us(100);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					  zero_cross_detected = 0;

				  }
}

void realTemp_LCD()
{
	Lcd_write_custom_char(3,0, 1);
	Lcd_gotoxy(5,0);
	Lcd_write_string("=");
	Lcd_gotoxy(7,0);
	sprintf(chuoi,"%.2f",PT100_Temperature);
	Lcd_write_string(chuoi);
	Lcd_write_custom_char(12, 0, 0);
	Lcd_gotoxy(13,0);
	Lcd_write_string("C");
}
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0)
  {
	  zero_cross_detected = 1;
	  cong++;
  }
  /* USER CODE END EXTI0_IRQn 1 */
}


void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(TANG_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
  if(HAL_GPIO_ReadPin(GPIOA, TANG_Pin) == 0){
    	for(int i = 500000; i>0; i--);

    	if(HAL_GPIO_ReadPin(GPIOA, TANG_Pin) == 0){
    	  setpoint+=2;

    	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    	 HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
    	 while(HAL_GPIO_ReadPin(GPIOA, TANG_Pin) == 0);

    }
    }
  /* USER CODE END EXTI3_IRQn 1 */
}
/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GIAM_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
  if(HAL_GPIO_ReadPin(GPIOA, GIAM_Pin) == 0){
  	for(int i = 500000; i>0; i--);

  	if(HAL_GPIO_ReadPin(GPIOA, GIAM_Pin) == 0){
  	  setpoint-=2;

  	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
  	 HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);
  	 while(HAL_GPIO_ReadPin(GPIOA, GIAM_Pin) == 0);

  }
  }
  /* USER CODE END EXTI4_IRQn 1 */
}
void delay_us (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
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
