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
#include "projdefs.h"
#include "stm32f1xx_hal_uart.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include "ugui.h"
#include "ugui_colors.h"
#include "user.h"
#include "images.h"
#include "main_menu.h"
#include "esp8266.h"
#include "WiFiConnection.h"
#include <inttypes.h>
#include "extraTools.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEFAULT_FONT FONT_6X8
#define TIM_FREQ 64000000
#define MAX_OBJECTS 10 //uGUI

//FLASH DEFINES
#define FLASH_GAME_START_ADDR   ADDR_FLASH_PAGE_64   /* Start @ of user Flash area */
#define FLASH_GAME_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE   /* End @ of user Flash area */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//test

static USER_INPUT uInput;
static USER_INPUT uInputOld;
static ActiveGameBobOption selected_menu_option = PLAY_GAME;
static ActiveScreen screen = MAIN_MENU_SCREEN;
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

/*SELECT GAME VARIABLES */
static uint8_t nr_of_games = 0;
static uint8_t selected_game = 0;

/* UART BUFFER */

//uint8_t rx[50] = {0};
//volatile uint8_t found_flag = 0;
//uint8_t rx_index = 0;
//volatile uint8_t tx_completed_flag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel);
static void initUserInput(USER_INPUT *uInput);
static void updateSelectedOption(ActiveGameBobOption selected_menu_option);
static void bootGame();
static void MX_TIM1_DeInit();
static void drawPic(PICTURE pic, uint16_t x, uint16_t y);

//POWER OFF AND FW CHANGE
static void deInitPeripherals();
static void powerOFF();

//FLASHING RELATED FUNCTIONS
static HAL_StatusTypeDef eraseOldGame();

//SCREEN RELATED FUNCTIONS
static void drawMainMenu();
static void mainMenuScreenLogic();
static void drawSelectGameScreen();
static void selectGameScreenLogic();
static void drawFailureScreen();
static void updateSelectedGame(uint8_t hightlighted_game);
static void updateDownloadProgress(char *part, char *nr_of_parts);
static void clearScreen();


//WiFi CONNECTION RELATED FUNCTIONS
static uint8_t initWiFi();
static uint8_t connectToWiFi();
static uint8_t getGameList(char *ret_buffer, size_t size );
static uint8_t downloadGame();
static int findDataBlockEnd(int offset);



//uGUI


/* TASK FUNCTIONS */
static void testTask( void *pvParameters );
static void taskMainMenu(void *pvParameters);
static void taskGetUserInput(void *pvParameters);
static void taskGameLogic(void *pvParameters);
TaskHandle_t h_task_main_menu = NULL;
TaskHandle_t h_task_game_logic = NULL;

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
	/*
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
	DWT->CYCCNT = 0;                                // Clear counter
	DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;             // Enable counter
	*/
	BaseType_t xReturned;
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
  initUserInput(&uInput);

  	#ifdef DEBUG_LOG
		static char debug_output[50];
		sprintf(debug_output,"TEST \n");
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)debug_output, strlen(debug_output));
	#endif
  xReturned = xTaskCreate(testTask, "task", 256, NULL, 1, NULL);
  xReturned = xTaskCreate(taskMainMenu,"taskMainMenu",256,NULL,3,&h_task_main_menu);
  xReturned = xTaskCreate(taskGetUserInput,"taskGetUserInput",256,NULL,2,NULL);

  //xReturned = xTaskCreate(taskGameLogic, "taskGameLogic", 512, NULL, 3, NULL);
  vTaskStartScheduler();
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  sConfig.Channel = ADC_CHANNEL_6;
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10006;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  huart3.Init.BaudRate = 9600;
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ESP_EN_Pin|POWER_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ESP_EN_Pin */
  GPIO_InitStruct.Pin = ESP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_OFF_Pin */
  GPIO_InitStruct.Pin = POWER_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_ANALOG_KEY_Pin */
  GPIO_InitStruct.Pin = LEFT_ANALOG_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LEFT_ANALOG_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_ANALOG_KEY_Pin KEY_DOWN_Pin KEY_UP_Pin KEY_RIGHT_Pin
                           KEY_LEFT_Pin */
  GPIO_InitStruct.Pin = RIGHT_ANALOG_KEY_Pin|KEY_DOWN_Pin|KEY_UP_Pin|KEY_RIGHT_Pin
                          |KEY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_B_Pin KEY_A_Pin */
  GPIO_InitStruct.Pin = KEY_B_Pin|KEY_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
   Error_Handler();
  }
}


static void testTask( void *pvParameters ){

	for(;;){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	    vTaskDelay(500);
	}
}


static void drawPic(PICTURE pic, uint16_t x, uint16_t y){
	uint16_t start_x = x;
	uint16_t start_y = y;
	uint16_t cur_x = start_x;
	uint16_t cur_y = start_y;
	for(uint16_t i =0; i<pic.height;i++){
		cur_x = start_x;
		for(uint16_t j=0;j<pic.width;j++){
			uint8_t max_shift = j == pic.width-1 ?  4 :  0;
			for(int8_t k=7;k>=max_shift;k--){
				uint8_t temp = 1;
				temp <<=k;
				uint8_t current_value = pic.pic[i][j];
				uint8_t temp2 = current_value & temp;
				if( temp2 == temp){
					UG_DrawPixel(cur_x, cur_y, C_WHITE);
				}
				else{
					UG_DrawPixel(cur_x, cur_y, C_BLACK);
				}
				cur_x++;
			}
		}
		cur_y+=1;
	}

}


static void drawFailureScreen(){
	//UG_FillScreen(C_WHITE);
	//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
	clearScreen();
	//UG_Update();
	char text_buffer[30];
	sprintf(text_buffer, "Failed. Press A to return");
	LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen(text_buffer) / 2),
			LCD_HEIGHT / 2 - 8, text_buffer, FONT_10X16, C_BLACK, C_WHITE);
	//UG_Update();
}

static void drawSelectGameScreen(){
	//UG_FillScreen(C_WHITE);
	//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
	clearScreen();
	char text_buffer[25];
	sprintf(text_buffer, "Loading Game List...");
	LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen(text_buffer) / 2),
			LCD_HEIGHT / 2 - 8, text_buffer, FONT_10X16, C_BLACK, C_WHITE);
	//UG_Update();


	uint8_t esp8266_ret;

	if(initWiFi() != ESP8266_OK){
		drawFailureScreen();
		return;
	}
	//result = Esp8266_ChangeBaudRate(9600, 8, 1, 0, 0);
	//  result = Esp8266_ListAP(output);
	if(connectToWiFi() != ESP8266_OK){
		drawFailureScreen();
		return;
	}

	char game_list[50] = {0};
	esp8266_ret = getGameList(game_list, 100);
	if(esp8266_ret != ESP8266_OK){
		drawFailureScreen();
		return;
	}
	else{
		//UG_FillScreen(C_WHITE);
		//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
		clearScreen();
		char *token = strtok(game_list," ");
		//char temp_buff[100];
		int i = 0;
		while(token){
			nr_of_games++;
			LCD_PutStr(LCD_WIDTH / 2 - (16 * strlen(token) / 2),
					26+i, token, FONT_16X26, C_BLACK, C_WHITE);
			token = strtok(NULL," ");
			i+=26;
		}
		if(i==0){
			LCD_PutStr(LCD_WIDTH / 2 - (16 * strlen("Nothing to load") / 2),
					LCD_HEIGHT/2, "Nothing to load", FONT_16X26, C_BLACK, C_WHITE);
		}

	}


}

static uint8_t initWiFi(){
	HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(1000));
	uint8_t esp8266_ret = Esp8266_Init(&huart3);
	if(esp8266_ret != ESP8266_OK){
		return esp8266_ret;
	}
	esp8266_ret = Esp8266_SetCwMode(1);
	if(esp8266_ret != ESP8266_OK){
		return esp8266_ret;
	}
	return esp8266_ret;
}


static uint8_t connectToWiFi(){
	char server[] = "172.20.10.4";
	char port[] = "5000";

	uint8_t esp8266_ret = Esp8266_ConnectAP(ssid, pwd); //ssid and pwd inside the WiFiConnection.h
	if(esp8266_ret != ESP8266_OK){
		return esp8266_ret;
	}
	//result = Esp8266_GetIpAddress(output, output_size);
	esp8266_ret = Esp8266_StartTCPIPConnection(server, port);
	vTaskDelay(pdMS_TO_TICKS(1000));
	uint8_t retry_counter = 1;
	while(esp8266_ret == ESP8266_ERROR && retry_counter < 3){
		esp8266_ret = Esp8266_StartTCPIPConnection(server, port);
		retry_counter++;
	}
	if(esp8266_ret != ESP8266_OK){
		return esp8266_ret;
	}
	return esp8266_ret;
}

static uint8_t getGameList(char *ret_buffer, size_t size ){
	char cmd[60];
	char server[] = "172.20.10.4";

	sprintf(cmd, "GET /gameList.txt HTTP/1.1\r\nHost: %s\r\n\r\n", server);
	uint8_t esp8266_ret = Esp8266_SendIpCommand(cmd);
	if(esp8266_ret != ESP8266_OK){
		return esp8266_ret;
	}
	/* Interpret data from server */
	//#ifdef DEBUG_LOG
	//	sprintf(debug_output,"Finding index \n");
	//	HAL_UART_Transmit_IT(&huart2, (uint8_t*)debug_output, strlen(debug_output));
	//#endif

	int data_start_index = find_str((char*)rx_buffer,"DATA START\n", strlen("DATA START\n"));

	#ifdef DEBUG_LOG
		static char debug_output[50];
		sprintf(debug_output,"Found index %d",data_start_index);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)debug_output, strlen(debug_output));
		vTaskDelay(pdMS_TO_TICKS(1000));
	#endif

	char *raw_data = (char*)rx_buffer + data_start_index;
	char game_list[128];
	strcpy(game_list,raw_data);
	char *token = strtok(game_list,"\n");
	char temp_buff[100]={0};
	while(token){
		#ifdef DEBUG_LOG
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)token, strlen(token));
			vTaskDelay(pdMS_TO_TICKS(1000));
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"\n", strlen("\n"));
			vTaskDelay(pdMS_TO_TICKS(100));
		#endif

		token = strtok(NULL,"\n");
		if(token == NULL || strstr(token,"STOP") != NULL)
			break;

		//if(strlen(token)<100-strlen(temp_buff) && find_str(token, "DATA STOP", strlen("DATA STOP"))){

		if(strlen(token)<100-strlen(temp_buff)){// && strstr(token,"STOP") == NULL){

			strcat(temp_buff,token);
			strcat(temp_buff," ");
			//	#ifdef DEBUG_LOG
			//	HAL_UART_Transmit_IT(&huart2, (uint8_t*)temp_buff, strlen(temp_buff));
			//	vTaskDelay(pdMS_TO_TICKS(1000));
			//#endif
		}
	}
		#ifdef DEBUG_LOG
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"While End \n", strlen("While End \n"));
			vTaskDelay(pdMS_TO_TICKS(1000));
		#endif
	if(strlen(temp_buff)<size){
		strcpy(ret_buffer, temp_buff);
	}
	else{
		strcpy(ret_buffer, "ERROR");
	}
	#ifdef DEBUG_LOG
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)ret_buffer, strlen(ret_buffer));
		vTaskDelay(pdMS_TO_TICKS(1000));
	#endif
	return esp8266_ret;

}


static HAL_StatusTypeDef eraseOldGame(){
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_PROC_PAGEERASE;
	EraseInitStruct.PageAddress = FLASH_GAME_START_ADDR;
	EraseInitStruct.NbPages = (FLASH_GAME_END_ADDR-FLASH_GAME_START_ADDR)/FLASH_PAGE_SIZE;
	uint32_t PAGEError;
	status = HAL_FLASH_Unlock();
	if(status!=HAL_OK){
		return status;
	}
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	return status;

}

static void updateDownloadProgress(char *part, char *nr_of_parts){
	//UG_FillScreen(C_WHITE);
	//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
	clearScreen();
	LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen("Downloading game...") / 2),
			LCD_HEIGHT/4, "Downloading game...", FONT_10X16, C_BLACK, C_WHITE);
	char text[10];
	sprintf(text, "%s/%s",part,nr_of_parts);
	LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen(text) / 2),
			LCD_HEIGHT/4+32, text, FONT_10X16, C_BLACK, C_WHITE);
}

static uint8_t downloadGame(){

	char server[] = "172.20.10.4";
	char port[] = "5000";
	uint8_t esp8266_ret;
	HAL_StatusTypeDef status;

	int loop_index=0;
	const char *key = "\r\n+IPD,";
	const char *key_start;
	int key_index = 0;
	int key_start_index;
	int data_block_end;
	uint8_t found_colon_flag;
	uint8_t retry_counter;

	uint32_t received_byte;
	uint32_t byte_to_flash = 0;
	uint32_t address = FLASH_GAME_START_ADDR;
	int byte_counter = 0;




	char cmd[60];
	char part_str[3];
	char nr_of_parts_str[3];
	//char server[] = "172.20.10.4";
	int part = 1;
	int nr_of_parts = 13;
	status = eraseOldGame();
	if(status!=HAL_OK){
		return ESP8266_ERROR;
	}
	intToString(nr_of_parts, nr_of_parts_str, 3);
	while(part<=nr_of_parts){

		esp8266_ret = Esp8266_StartTCPIPConnection(server, port);
		retry_counter = 1;
		while(esp8266_ret == ESP8266_ERROR && retry_counter < 3){
			esp8266_ret = Esp8266_StartTCPIPConnection(server, port);
			retry_counter++;
		}
		intToString(part, part_str, 3);
		updateDownloadProgress(part_str, nr_of_parts_str);
		key_index = 0;
		loop_index = 0;
		sprintf(cmd, "GET /gameBobPong/pong_part%s.bin HTTP/1.1\r\nHost: %s\r\n\r\n", part_str, server);
		esp8266_ret = Esp8266_SendIpCommand(cmd);
		if(esp8266_ret != ESP8266_OK){
			return esp8266_ret;
		}

		/* Interpret data from server */
		do{

			key_start_index = find_str((char*)rx_buffer+key_index, key, strlen(key));
			key_start = (char*)rx_buffer + key_start_index;
			if(key_start_index!=-1){
				key_index = key_start - (char*)rx_buffer + 5; //+5 to skip +IPD: chars
				if(loop_index == 0){ //skip first block of data (header etc...)
					loop_index++;
				}
				else{
					data_block_end = findDataBlockEnd(key_index);
					if(data_block_end==-1){//error in communication
						return ESP8266_ERROR;
					}
					found_colon_flag = 0;
					for(int i=key_index;i<data_block_end;i++){
						if(!found_colon_flag){
							if(rx_buffer[i] == ':'){
								found_colon_flag = 1;
							}
						}
						else{ //RECEIVED BYTE
							received_byte = rx_buffer[i];
							received_byte<<=8*byte_counter;
							byte_to_flash |=received_byte;
							if(byte_counter==3){//Whole word received - ready to flash
								byte_counter = 0;
								status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, byte_to_flash);
								address+=4;
								byte_to_flash = 0;
								if(address == 0x080105B0){
									address = address;
								}

							}
							else{
								byte_counter++;
							}

						}
					}
				}
			}
			else{
				break;
			}

		}while(key_start_index!=-1);
		part++;
	}
	status = HAL_FLASH_Lock();
	if(part >= nr_of_parts){
		//UG_FillScreen(C_WHITE);
		//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
		//HAL_Delay(100);
		clearScreen();
		LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen("Download OK") / 2),
				LCD_HEIGHT/2, "Download OK", FONT_10X16, C_BLACK, C_WHITE);
	}
	else{
		//UG_FillScreen(C_WHITE);
		//UG_FillFrame(0, 0, LCD_WIDTH, LCD_HEIGHT, C_WHITE);
		//HAL_Delay(100);
		clearScreen();
		LCD_PutStr(LCD_WIDTH / 2 - (10 * strlen("Download ERROR") / 2),
				LCD_HEIGHT/2, "Download ERROR", FONT_10X16, C_BLACK, C_WHITE);
	}
	
	return esp8266_ret;
}


static int findDataBlockEnd(int offset){
	const char *key = "\r\n+IPD,";
	int block_end = 0;
	int next_key_index;
	//const char *next_key = strstr((char*)rx_buffer+offset,key);
	next_key_index = find_str((char*)rx_buffer+offset, key, strlen(key));
	const char *next_key = (char*)rx_buffer+next_key_index;
	if(next_key_index != -1){
		return next_key_index;//next_key - (char*)rx_buffer;
	}
	else{
		const char *end_key = "CLOSED";
		next_key_index = find_str((char*)rx_buffer+offset, end_key, strlen(end_key));
		return next_key_index;
	}
}


static void selectGameScreenLogic(){

	if (uInput.keyA == GPIO_PIN_RESET) {
		screen = MAIN_MENU_SCREEN;
		HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET); //Disable ESP8266
		drawMainMenu();
		updateSelectedOption(selected_menu_option);
	}
	if (uInput.keyB == GPIO_PIN_RESET) {
		downloadGame();
	}
	if (uInput.keyDown == GPIO_PIN_RESET && uInput.keyDown != uInputOld.keyDown) {
		if (selected_game < nr_of_games) {
			selected_game++;
			updateSelectedGame(selected_game);
		}
	}
	else if (uInput.keyUp == GPIO_PIN_RESET && uInput.keyUp != uInputOld.keyUp) {
		if (selected_game > 0) {
			selected_game--;
			updateSelectedGame(selected_game);
		}
	}
}


static void drawMainMenu(){
	clearScreen();
	//UG_Update();
	UG_FillFrame(0, 0, LCD_WIDTH, 30, C_PURPLE);
	char text_buffer[25];
	sprintf(text_buffer, "Welcome to GameBob!");
	LCD_PutStr(10, 10, text_buffer, FONT_12X20, C_WHITE, C_PURPLE);
	drawPic(play_game_pic,LCD_WIDTH/2-42, LCD_HEIGHT/2-62);
	drawPic(load_game_pic, LCD_WIDTH/2-42, LCD_HEIGHT/2);
	drawPic(power_off_pic, LCD_WIDTH/2-42, LCD_HEIGHT/2+62);
	//UG_Update();
}


static void clearScreen(){
	for(int i=0;i<3;i++){
		UG_FillScreen(C_WHITE);
	}
}


static void mainMenuScreenLogic(){

	//Select option if confirmed
	if (selected_menu_option == PLAY_GAME && uInput.keyB == GPIO_PIN_RESET) {
		bootGame();
	}
	if (selected_menu_option == SELECT_GAME && uInput.keyB == GPIO_PIN_RESET) {
		screen = SELECT_GAME_SCREEN;
		drawSelectGameScreen();
	}
	if (selected_menu_option == POWER_OFF && uInput.keyB == GPIO_PIN_RESET){
		powerOFF();
	}

	//Check if selected option changed
	if (uInput.keyDown == GPIO_PIN_RESET && uInput.keyDown != uInputOld.keyDown) {
		if (selected_menu_option < POWER_OFF) {
			selected_menu_option++;
			updateSelectedOption(selected_menu_option);
		}
	}
	else if (uInput.keyUp == GPIO_PIN_RESET && uInput.keyUp != uInputOld.keyUp) {
		if (selected_menu_option > PLAY_GAME) {
			selected_menu_option--;
			updateSelectedOption(selected_menu_option);
		}
	}
}

static void taskMainMenu(void *pvParameters){

	drawMainMenu();
	updateSelectedOption(selected_menu_option);

	BaseType_t xReturned;
	//uint8_t game_started_flag = 0;
	for (;;) {
		switch (screen) {
		case (MAIN_MENU_SCREEN):
			mainMenuScreenLogic();
			break;
		case (SELECT_GAME_SCREEN):
			selectGameScreenLogic();
			break;

		default:
			mainMenuScreenLogic();
		}

		vTaskDelay(33);
	}


}

static void taskGetUserInput(void *pvParameters){
	/* Check analog input */
	uint16_t joystick[4] = {0}; //0 - X axis ; 1 - Y axis
	char buffer[20];
	uint32_t adc_channel = ADC_CHANNEL_6;
	uint32_t adc_channels[4] = {ADC_CHANNEL_6,ADC_CHANNEL_7,ADC_CHANNEL_8,ADC_CHANNEL_9};
	for(;;){
		for(int i=0;i<4;i++){
			ADC_SetActiveChannel(&hadc1, adc_channels[i]);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			joystick[i] = HAL_ADC_GetValue(&hadc1);
		}

		uInputOld = uInput;

		uInput.leftXAxis = joystick[0];
		uInput.leftYAxis = joystick[1];
		uInput.rightXAxis = joystick[2];
		uInput.rightYAxis = joystick[3];

		/* Check analog key press */
		uInput.leftAnalogKey = HAL_GPIO_ReadPin(LEFT_ANALOG_KEY_GPIO_Port, LEFT_ANALOG_KEY_Pin);
		uInput.rightAnalogKey = HAL_GPIO_ReadPin(RIGHT_ANALOG_KEY_GPIO_Port, RIGHT_ANALOG_KEY_Pin);

		/* Check buttons state */
		uInput.keyDown = HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin);
		uInput.keyUp = HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin);
		uInput.keyB = HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin);
		uInput.keyA = HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin);
		uInput.keyLeft = HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin);
		uInput.keyRight = HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin);

		/* Get seed */
		ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_10);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		uInput.seed = HAL_ADC_GetValue(&hadc1);



		vTaskDelay(33);
	}
}

static void initUserInput(USER_INPUT *uInput){
	uInput->leftXAxis = 2000;
	uInput->leftYAxis = 2000;
	uInput->rightXAxis = 2000;
	uInput->rightYAxis = 2000;
	uInput->keyLeft = GPIO_PIN_SET;
	uInput->keyRight = GPIO_PIN_SET;
	uInput->keyUp = GPIO_PIN_SET;
	uInput->keyDown = GPIO_PIN_SET;
	uInput->keyA = GPIO_PIN_SET;
	uInput->keyB = GPIO_PIN_SET;

	ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	uInput->seed = HAL_ADC_GetValue(&hadc1);


}

static void MX_TIM1_DeInit(){
	HAL_TIM_PWM_DeInit(&htim1);
}


static void deInitPeripherals(){
	vTaskSuspendAll();
	HAL_UART_DeInit(&huart3);
	HAL_UART_DeInit(&huart2);
	MX_TIM1_DeInit();
	HAL_ADC_DeInit(&hadc1);
	HAL_SPI_DeInit(&hspi1);
	__HAL_RCC_DMA1_CLK_DISABLE();
	HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	//__HAL_RCC_GPIOC_CLK_DISABLE();
	//__HAL_RCC_GPIOA_CLK_DISABLE();
	//__HAL_RCC_GPIOB_CLK_DISABLE();

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_DeInit(LEFT_ANALOG_KEY_GPIO_Port, LEFT_ANALOG_KEY_Pin);
	HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
	HAL_GPIO_DeInit(GPIOB, RIGHT_ANALOG_KEY_Pin|KEY_DOWN_Pin|KEY_UP_Pin);
	HAL_GPIO_DeInit(GPIOC, KEY_B_Pin|KEY_A_Pin);
	HAL_GPIO_DeInit(LCD_DC_GPIO_Port, LCD_DC_Pin);
	HAL_GPIO_DeInit(GPIOB, LCD_RST_Pin|LCD_CS_Pin);

	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}


static void bootGame(){
	uint32_t address = 0x8010000;
	typedef void (application_t)(void);
	typedef struct
	{
	    uint32_t		stack_addr;     // Stack Pointer
	    application_t*	func_p;        // Program Counter
	} JumpStruct;
	const JumpStruct* vector_p = (JumpStruct*)address;
	//uint32_t game_address = 0x8010000;
	//uint32_t msp_address = *(__IO uint32_t*)game_address;
	//uint32_t reset_address = *(__IO uint32_t*)(game_address+4);
	deInitPeripherals();
	__HAL_RCC_GPIOC_CLK_DISABLE(); // GPIOC not deinitialized inside deInitPeripherals
	//SCB->VTOR = game_address;
	//void (*jumpAddress)(void);
	//jumpAddress = (void (*)())reset_address;
	//jumpAddress();

	//__set_MSP(msp_address);
	asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));


}

static void updateSelectedGame(uint8_t hightlighted_game){
	//uint16_t option_colors[3] = {C_WHITE, C_WHITE, C_WHITE};


	uint16_t color;
	int padding = 0;
	int font_height = 26;
	for(uint8_t i=0; i<nr_of_games; i++){
		color = hightlighted_game == i ? C_PURPLE :  C_WHITE;
		UG_DrawFrame(0, font_height+padding, LCD_WIDTH, 2*font_height+padding, color);
		UG_DrawFrame(0, font_height+padding-1, LCD_WIDTH, 2*font_height+padding+1, color);
		padding+=26;
	}


}

static void updateSelectedOption(ActiveGameBobOption selected_menu_option){
	uint16_t option_colors[3] = {C_WHITE, C_WHITE, C_WHITE};

	for(int i=0; i<3; i++){
		option_colors[i] = selected_menu_option == i ?  C_PURPLE : C_WHITE;
	}
	uint16_t pic_width = 84;
	uint16_t pic_height = 51;
	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2-62, LCD_WIDTH/2-42+pic_width, LCD_HEIGHT/2-62+pic_height, option_colors[0]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2-62-1, LCD_WIDTH/2-42+pic_width+1, LCD_HEIGHT/2-62+1+pic_height, option_colors[0]);

	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2, LCD_WIDTH/2-42+pic_width, LCD_HEIGHT/2+pic_height, option_colors[1]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2-1, LCD_WIDTH/2-42+1+pic_width, LCD_HEIGHT/2+1+pic_height, option_colors[1]);

	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2+62, LCD_WIDTH/2-42+pic_width, LCD_HEIGHT/2+62+pic_height, option_colors[2]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2+62-1, LCD_WIDTH/2-42+1+pic_width, LCD_HEIGHT/2+62+1+pic_height, option_colors[2]);

}

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}

static void powerOFF(){
	//deInitPeripherals();
	HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_SET);
}


static void window_1_callback(UG_MESSAGE *msg)
{

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	ESP8266_UART_TxCpltCallback(huart);
	/*
	tx_completed_flag = 1;
	*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	ESP8266_UART_RxCpltCallback(huart);

}

/* USER CODE END 4 */

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
  if (htim->Instance == TIM2)
  {
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
