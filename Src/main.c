/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
MODULE_LICENSE("GPL");	// licence g�n�ral public
MODULE_AUTHOR("Thomas Abgrall, Devaux Axel, Venier Antoine");
MODULE_DESCRIPTION("Control led pin STM32F407VG-DISCOVERY"); 
MODULE_VERSION("Version 1.00"); 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {PARSE_STATUS_OK, PARSE_STATUS_ERR}Parse_STATUS;

typedef struct{
	GPIO_PinState LED14;
	GPIO_PinState LED15;
}LEDs_Stat;

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
static LEDs_Stat LEDs_Stat;
static uint8_t Buff_Data_Reception[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
Parse_STATUS parseUARTData_Reception(uint8_t Data_Reception[8]);
void emitBip(void);
void afficherEtatLEDs(void);
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
	LEDs_Stat.LED14 = GPIO_PIN_RESET;
	LEDs_Stat.LED15 = GPIO_PIN_RESET;
	
	memset(Buff_Data_Reception, 0xCC, 2);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart2,Buff_Data_Reception,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Gestion du bouton qui toggle les deux leds*/
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			
			/* Mise � jour des variables qui tracks l'�tat des LEDs*/
			LEDs_Stat.LED14 = (GPIO_PinState) !LEDs_Stat.LED14;
			LEDs_Stat.LED15 = (GPIO_PinState) !LEDs_Stat.LED15;
			
			/*anti-ebonds du bouton*/
			do{
				HAL_Delay(200);
			}while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
		}
		HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim2.Init.Prescaler = 750;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 249;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX_Data_Reception;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Le timer n�2 (3 secondes) permet de d�clencher le timer 3 */
	if(htim == &htim2){
		afficherEtatLEDs();
		HAL_TIM_Base_Start_IT(&htim3);
		
		// A noter que le Timer 2 ne se relance pas lui m�me
	}
	/* Le timer n�3 (1 seconde) permet de d�clencher l'affichage */
	else if( htim == &htim3){
			afficherEtatLEDs();
			HAL_TIM_Base_Start_IT(&htim3);
	}
}

void HAL_UART_Data_ReceptionCpltCallback(UART_HandleTypeDef *huart){
	/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	/* Cette fonction permet de recevoir les charact�res re�us sur la liaison
	 * s�rie 1 par 1.
	 * Les charat�tres sont stock�s dans la variable stacking.
	*/
	
	uint8_t reset_index = 0;
	static uint8_t stacking[10];
	static char index = 0;
	uint32_t* tim2_cnt = (uint32_t*) 0x40000024;
	
	
	stacking[index] = Buff_Data_Reception[0];
	memset(Buff_Data_Reception, 0xCC, 8);
	
	/* Ce if d�tecte la fin de la commande */
	if(stacking[index] == '\n' || index >= 9){
		Parse_STATUS parse_status = parseUARTData_Reception(stacking);
		if(parse_status == PARSE_STATUS_ERR){
			emitBip();
		}
		
		/* Reset du compteur du timer 2 */ 
		*tim2_cnt = 0x00;
		
		// Comme une commande a �t� d�t�ct�e, on arr�te d'affiche l'�tat des LEDs
		HAL_TIM_Base_Stop_IT(&htim3);
		
		// On red�mare alors le timer 2 au cas ou il a �t� stopp�
		HAL_TIM_Base_Start_IT(&htim2);
		
		reset_index = 1;
		memset(stacking, 0xDD, 10);
	}
	if(!reset_index)
		index ++;
	else
		index = 0;
	
	HAL_UART_Receive_IT(&huart2,Buff_Data_Reception,1);
}

/**
  * @brief Cette fontion permet d'analyser le commande et
	*				d'effectuer les actions sur les LEDs
	* @param Data_Reception : la commande � analyser
	* @retval Parse_STATUS : 
	*								PARSE_STATUS_ERR : Si la commande re�u n'�tait pas valide
	*								PARSE_STATUS_OK : Si la commande re�u �tait valide
  */
Parse_STATUS parseUARTData_Reception(uint8_t Data_Reception[10]){
	uint16_t LED;
	GPIO_PinState etat;
	
	/* if sur les parties constantes des commande valides */
	if(Data_Reception[0] == 'L' && Data_Reception[1] == 'E' && Data_Reception[2] == 'D' && Data_Reception[3] == ' ' &&
		 Data_Reception[5] == ' ' && Data_Reception[6] == 'O')
	{
		/* v�rification sur les parie variable (LED et �tat) */
		switch(Data_Reception[4]){
			case '1':
				LED = GPIO_PIN_14;
				break;
			case '2':
				LED = GPIO_PIN_15;
				break;
			default:
				return PARSE_STATUS_ERR; 
		}
		switch(Data_Reception[7]){
			case 'N':
				etat = GPIO_PIN_SET;
				break;
			case 'F':
				etat = GPIO_PIN_RESET;
				break;
			default :
				return PARSE_STATUS_ERR; 
		}
		
		HAL_GPIO_WritePin(GPIOD, led, etat);
		/* Mise � jour de la structure de tracking de l'�tat des LEDs */
		if(LED == GPIO_PIN_14)
			LEDs_Stat.LED14 = etat;
		else
			LEDs_Stat.LED15 = etat;
		return PARSE_STATUS_OK;
	}
	else
		return PARSE_STATUS_ERR;
}

/**
  * @brief Fonction pour emettre un son sur les �couteurs
  * @param None
  * @retval None
  */
void emitBip(){
	HAL_UART_Transmit(&huart2, (uint8_t*) "BIP\r\n", 6, 10);
}

/**
  * @brief Fonciton qui affiche l'�tat des LEDs sur la liaison s�rie
  * @param None
  * @retval None
  */
void afficherEtatLEDs(){
	HAL_UART_Transmit(&huart2, (uint8_t*) "Etat LEDs : \n", 14, 10);
	HAL_UART_Transmit(&huart2, (uint8_t*) "LED 1 > O", 10, 10);
	if(LEDs_Stat.LED14 == GPIO_PIN_SET)
		HAL_UART_Transmit(&huart2, (uint8_t*) "N\n\r", 4, 10);
	else
		HAL_UART_Transmit(&huart2, (uint8_t*) "FF\n\r", 5, 10);
	
	HAL_UART_Transmit(&huart2, (uint8_t*) "LED 2 > O", 10, 10);
	if(LEDs_Stat.LED15 == GPIO_PIN_SET)
		HAL_UART_Transmit(&huart2, (uint8_t*) "N\n\r", 4, 10);
	else
		HAL_UART_Transmit(&huart2, (uint8_t*) "FF\n\r", 5, 10);
	HAL_UART_Transmit(&huart2, (uint8_t*) "------------\n", 14, 10);
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
	while(1){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_Delay(600);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
