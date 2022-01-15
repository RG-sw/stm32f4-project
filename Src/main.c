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
#include <eth_utils.h>
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"
#include "ADXL345.h"
#include "sd_utils.h"
#include "socket_utils.h"
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
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi3;


/* Definitions for AccToEth */
osThreadId_t AccToEthHandle;
const osThreadAttr_t AccToEth_attributes = {
  .name = "AccToEth",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SDToEth */
osThreadId_t SDToEthHandle;
const osThreadAttr_t SDToEth_attributes = {
  .name = "SDToEth",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AccToSD */
osThreadId_t AccToSDHandle;
const osThreadAttr_t AccToSD_attributes = {
  .name = "AccToSD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SD_Mutex */
osMutexId_t SD_MutexHandle;
const osMutexAttr_t SD_Mutex_attributes = {
  .name = "SD_Mutex"
};
/* USER CODE BEGIN PV */
int SD_FULL=0, SD_EMPTY=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);
void StartAccToEth(void *argument);
void StartSDToEth(void *argument);
void StartAccToSD(void *argument);

/* USER CODE BEGIN PFP */
void Next_Active_Task(void);
//void Toggle_Led(uint16_t);
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
  MX_SDIO_SD_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /*ADXL345 INIT*/
  if( ADXL345_Init() != ADXL345_OK) {
	  myErrorHandler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the mutex(es) */
  /* creation of SD_Mutex */
  SD_MutexHandle = osMutexNew(&SD_Mutex_attributes);

  /* Create the thread(s) */
  /* creation of AccToEth */
  AccToEthHandle = osThreadNew(StartAccToEth, NULL, &AccToEth_attributes);

  /* creation of SDToEth */
  SDToEthHandle = osThreadNew(StartSDToEth, NULL, &SDToEth_attributes);

  /* creation of AccToSD */
  AccToSDHandle = osThreadNew(StartAccToSD, NULL, &AccToSD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* The system Starts with ACCtoETH Thread */
  osThreadSuspend(SDToEthHandle);
  osThreadSuspend(AccToSDHandle);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  (USER IMPLEMENTATION) function executed in case of error.
  * @retval None
  */
void myErrorHandler(void)
{
  __disable_irq();
  while (1)
  {
	  Toggle_Led(RED_LED);
	  HAL_Delay(500);
	  Toggle_Led(RED_LED);
	  HAL_Delay(500);
  }
}

/**
  * @brief LED Toggle Function
  * @param LED Pin
  * @retval None
  */
void Toggle_Led(uint16_t LED){
	HAL_GPIO_TogglePin(GPIOD,LED);
}

/**
  * @brief Task Scheduler Function
  * @param None
  * @retval None
  */
void Next_Active_Task(){

	osThreadId_t id; // id of the currently running thread

	/* CRITICAL SECTION */
	__disable_irq();

	id = osThreadGetId();

	if(is_ETH_UP()){
		if(SD_EMPTY && id!=AccToEthHandle) {
			osThreadResume(AccToEthHandle);
			osThreadSuspend(id);
		}
		if(!SD_EMPTY && id!=SDToEthHandle) {
			osThreadResume(SDToEthHandle);
			osThreadSuspend(id);
		}
	}
	else if(!is_ETH_UP()){
		if(!SD_FULL && id!=AccToSDHandle) {
			osThreadResume(AccToSDHandle);
			osThreadSuspend(id);
		}
	}

	__enable_irq();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAccToEth */
/**
* @brief Function implementing the AccToEth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccToEth */
void StartAccToEth(void *argument)
{
	/* USER CODE BEGIN StartAccToEth */
	MX_LWIP_Init();

	ADXL345_t Data;

	int sd;
	struct sockaddr_in addr;

	/* Setting remote Server Address*/
	init_sock_addr(&addr);

	/* Infinite loop */
	for(;;)
	{
		Next_Active_Task();

		/* Starting connection w/ remote Server */
		sd= start_TCP_socket(&addr);

		Toggle_Led(BLUE_LED);

		/* Reading Data from ADXL345 -> Writing on Socket */
		while(is_ETH_UP()){
			if( ADXL345_Read_Axis(&Data) != ADXL345_OK) {
				myErrorHandler();
			}
			socket_write(sd, Data);

			HAL_Delay(500);
		}
		close_socket(sd);

		Toggle_Led(BLUE_LED);
		HAL_Delay(100);

	}
	/* USER CODE END StartAccToEth */
}

/* USER CODE BEGIN Header_StartSDToEth */
/**
* @brief Function implementing the SDToEth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDToEth */
void StartSDToEth(void *argument)
{
  /* USER CODE BEGIN StartSDToEth */

	ADXL345_t Data;

	int sd;
	struct sockaddr_in addr;

	char buf[16];
	FATFS FatFs;
	FIL fil;

	init_sock_addr(&addr);

	  /* Infinite loop */
	  for(;;)
	  {
		  Next_Active_Task();

		  sd=start_TCP_socket(&addr);

		  /* Mount drive */
		  if(f_mount(&FatFs, "", 0) != FR_OK) myErrorHandler();
		  /* Open File */
		  if( f_open(&fil,"Data.txt", FA_OPEN_EXISTING|FA_READ) != FR_OK ) myErrorHandler();

		  Toggle_Led(GREEN_LED);

		  /* Reading Data from SD -> Writing on Socket */
		  while(is_ETH_UP() && !is_SD_EMPTY()) {
			  /* If EOF, stop reading from SD */
			  if (f_eof(&fil)) {
				  break;
			  }
			  memset(buf, 0 ,sizeof(buf));
			  f_gets(buf, sizeof(buf), &fil);

			  /* Conversion char[] -> ADXL345_t type */
			  StrToNum(buf, &Data);

			  socket_write(sd, Data);

			  HAL_Delay(500);
		  }
		  close_socket(sd);

		  /* Close File */
		  f_close(&fil);
		  Toggle_Led(GREEN_LED);
		  /* Deleting File */
		  f_unlink("Data.txt");

		  set_SD_Status('E',1); /*Set SD_EMPTY global variable*/
		  set_SD_Status('F',0); /*Reset SD_FULL global variable*/

		  f_mount(0, "", 0);
		  HAL_Delay(100);

	  }
  /* USER CODE END StartSDToEth */
}

/* USER CODE BEGIN Header_StartAccToSD */
/**
* @brief Function implementing the AccToSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccToSD */
void StartAccToSD(void *argument)
{
  /* USER CODE BEGIN StartAccToSD */
	ADXL345_t Data;

	int tot=0;
	char buf[16];
	FATFS FatFs;
	FIL fil;

	/* Infinite loop */
	for(;;)
	{
	  Next_Active_Task();

	  /* Mount Drive */
	  if (f_mount(&FatFs, "", 0) != FR_OK)  myErrorHandler();
	  /* Open File */
	  if ( f_open(&fil, "Data.txt", FA_CREATE_ALWAYS|FA_WRITE) != FR_OK) myErrorHandler();

	  Toggle_Led(ORANGE_LED);

	  /* Reading Data from ADXL345 -> Writing on SD */
	  while(!is_ETH_UP() && !is_SD_FULL()) {

		  if( ADXL345_Read_Axis(&Data) != ADXL345_OK){
			  myErrorHandler();
		  }
		  sprintf(buf,"%d %d %d\n",Data.X,Data.Y,Data.Z);

		  if ((tot+=f_puts(buf, &fil)) < 0){
			  break;
		  }
		  HAL_Delay(500);
	  }

	  Toggle_Led(ORANGE_LED);

	  /* Close File */
	  f_close(&fil);

	  /* If SD written, Reset SD_EMPTY */
	  if(tot > 0){
		  set_SD_Status('E',0); /*Reset SD_EMPTY global variable*/

		  /*If SD card is Full, Thread waits ACTIVELY ETH connection */
		  if(is_SD_FULL()){
			  while(!is_ETH_UP())
				  HAL_Delay(100);
		  }
	  }

	  f_mount(0, "", 0);
	  HAL_Delay(100);
	}
  /* USER CODE END StartAccToSD */
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
  Toggle_Led(RED_LED);
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
