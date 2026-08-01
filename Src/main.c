/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bme280.h"
#include "rc522.h"
#include "nrf24l01.h"
#include "mpu6050.h"
#include "string.h"
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

/* Definitions for bme280task */
osThreadId_t bme280taskHandle;
const osThreadAttr_t bme280task_attributes = {
  .name = "bme280task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorhubtask */
osThreadId_t sensorhubtaskHandle;
const osThreadAttr_t sensorhubtask_attributes = {
  .name = "sensorhubtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rc522task */
osThreadId_t rc522taskHandle;
const osThreadAttr_t rc522task_attributes = {
  .name = "rc522task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6050task */
osThreadId_t mpu6050taskHandle;
const osThreadAttr_t mpu6050task_attributes = {
  .name = "mpu6050task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spidmatask */
osThreadId_t spidmataskHandle;
const osThreadAttr_t spidmatask_attributes = {
  .name = "spidmatask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for nrf24l01TXtask */
osThreadId_t nrf24l01TXtaskHandle;
const osThreadAttr_t nrf24l01TXtask_attributes = {
  .name = "nrf24l01TXtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nrf24l01RXtask */
osThreadId_t nrf24l01RXtaskHandle;
const osThreadAttr_t nrf24l01RXtask_attributes = {
  .name = "nrf24l01RXtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c_mutex */
osMutexId_t i2c_mutexHandle;
const osMutexAttr_t i2c_mutex_attributes = {
  .name = "i2c_mutex"
};
/* Definitions for spi_mutex */
osMutexId_t spi_mutexHandle;
const osMutexAttr_t spi_mutex_attributes = {
  .name = "spi_mutex"
};
/* Definitions for spi3_mutex */
osMutexId_t spi3_mutexHandle;
const osMutexAttr_t spi3_mutex_attributes = {
  .name = "spi3_mutex"
};
/* Definitions for dma_semaphore */
osSemaphoreId_t dma_semaphoreHandle;
const osSemaphoreAttr_t dma_semaphore_attributes = {
  .name = "dma_semaphore"
};
/* Definitions for spi_semaphore */
osSemaphoreId_t spi_semaphoreHandle;
const osSemaphoreAttr_t spi_semaphore_attributes = {
  .name = "spi_semaphore"
};
/* Definitions for spi3_semaphore */
osSemaphoreId_t spi3_semaphoreHandle;
const osSemaphoreAttr_t spi3_semaphore_attributes = {
  .name = "spi3_semaphore"
};
/* USER CODE BEGIN PV */
bme280_handle_t my_bme280 = NULL;
rc522_handle_t my_rc522 = NULL;
mpu6050_handle_t my_mpu6050 = NULL;
nrf24l01_handle_t my_nrf2401 = NULL;
uint8_t sayac=0;
nrf24l01_handle_t my_nrf_tx = NULL;
nrf24l01_handle_t my_nrf_rx = NULL;
volatile uint32_t nrf_tx_success_count = 0;
volatile uint32_t nrf_tx_error_count = 0;
volatile uint32_t nrf_rx_packet_count = 0;
volatile uint8_t nrf_rx_fifo_count = 0;
volatile bool nrf_rx_fifo_was_full = false;
uint8_t nrf_rx_fifo[NRF24L01_FIFO_DEPTH][NRF24L01_PAYLOAD_SIZE] = {0};

osMessageQueueId_t i2c_job_queueHandle;
const osMessageQueueAttr_t i2c_job_queue_attributes = {
  .name = "i2c_job_queue"
};
osMessageQueueId_t spi_job_queueHandle;
const osMessageQueueAttr_t spi_job_queue_attributes = {
  .name = "spi_job_queue"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
void startbme280task(void *argument);
void startsensorhubtask(void *argument);
void startrc522task(void *argument);
void startmpu6050task(void *argument);
void startspidmatask(void *argument);
void startnrf24l01TXtask(void *argument);
void startnrf24l01RXtask(void *argument);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c_mutex */
  i2c_mutexHandle = osMutexNew(&i2c_mutex_attributes);

  /* creation of spi_mutex */
  spi_mutexHandle = osMutexNew(&spi_mutex_attributes);

  /* creation of spi3_mutex */
  spi3_mutexHandle = osMutexNew(&spi3_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dma_semaphore */
  dma_semaphoreHandle = osSemaphoreNew(1, 1, &dma_semaphore_attributes);

  /* creation of spi_semaphore */
  spi_semaphoreHandle = osSemaphoreNew(1, 1, &spi_semaphore_attributes);

  /* creation of spi3_semaphore */
  spi3_semaphoreHandle = osSemaphoreNew(1, 1, &spi3_semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  i2c_manager_assign_bus(I2C1, i2c_mutexHandle, dma_semaphoreHandle);
  spi_manager_assign_bus(SPI1, spi_mutexHandle, spi_semaphoreHandle);
  spi_manager_assign_bus(SPI3, spi3_mutexHandle, spi3_semaphoreHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  i2c_job_queueHandle = osMessageQueueNew(8, sizeof(i2c_job_t), &i2c_job_queue_attributes);
  spi_job_queueHandle = osMessageQueueNew(8, sizeof(spi_job_t), &spi_job_queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bme280task */
  //bme280taskHandle = osThreadNew(startbme280task, NULL, &bme280task_attributes);

  /* creation of sensorhubtask */
  sensorhubtaskHandle = osThreadNew(startsensorhubtask, NULL, &sensorhubtask_attributes);

  /* creation of rc522task */
  //rc522taskHandle = osThreadNew(startrc522task, NULL, &rc522task_attributes);

  /* creation of mpu6050task */
  //mpu6050taskHandle = osThreadNew(startmpu6050task, NULL, &mpu6050task_attributes);

  /* creation of spidmatask */
  spidmataskHandle = osThreadNew(startspidmatask, NULL, &spidmatask_attributes);

  /* creation of nrf24l01TXtask */
  nrf24l01TXtaskHandle = osThreadNew(startnrf24l01TXtask, NULL, &nrf24l01TXtask_attributes);

  /* creation of nrf24l01RXtask */
  nrf24l01RXtaskHandle = osThreadNew(startnrf24l01RXtask, NULL, &nrf24l01RXtask_attributes);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 DMA Init */

  /* I2C1_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* SPI1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */

  /* SPI3_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* SPI3_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_2, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  /* SPI3 interrupt Init */
  NVIC_SetPriority(SPI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(nrf24_ce_pin_spi1_GPIO_Port, nrf24_ce_pin_spi1_Pin);

  /**/
  LL_GPIO_SetOutputPin(nrf24_csn_pin_spi1_GPIO_Port, nrf24_csn_pin_spi1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(rc522_rst_pin_GPIO_Port, rc522_rst_pin_Pin);

  /**/
  LL_GPIO_SetOutputPin(nrf24_csn_pin_spi3_GPIO_Port, nrf24_csn_pin_spi3_Pin);
  LL_GPIO_ResetOutputPin(nrf24_ce_pin_spi3_GPIO_Port, nrf24_ce_pin_spi3_Pin);

  /**/
  LL_GPIO_SetOutputPin(rc522_cs_pin_GPIO_Port, rc522_cs_pin_Pin);

  /**/
  GPIO_InitStruct.Pin = nrf24_ce_pin_spi1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(nrf24_ce_pin_spi1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = rc522_cs_pin_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(rc522_cs_pin_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = nrf24_csn_pin_spi1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(nrf24_csn_pin_spi1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = rc522_rst_pin_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(rc522_rst_pin_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = nrf24_csn_pin_spi3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(nrf24_csn_pin_spi3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = nrf24_ce_pin_spi3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(nrf24_ce_pin_spi3_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE7);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE9);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE2);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(nrf24_irq_pin_spi1_GPIO_Port, nrf24_irq_pin_spi1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(rc522_irq_pin_GPIO_Port, rc522_irq_pin_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(mpu6050_irq_pin_GPIO_Port, mpu6050_irq_pin_Pin, LL_GPIO_PULL_DOWN);

  /**/
  LL_GPIO_SetPinPull(nrf24_irq_pin_spi3_GPIO_Port, nrf24_irq_pin_spi3_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(nrf24_irq_pin_spi1_GPIO_Port, nrf24_irq_pin_spi1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(rc522_irq_pin_GPIO_Port, rc522_irq_pin_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(mpu6050_irq_pin_GPIO_Port, mpu6050_irq_pin_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(nrf24_irq_pin_spi3_GPIO_Port, nrf24_irq_pin_spi3_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startbme280task */
/**
  * @brief  Function implementing the bme280task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startbme280task */
void startbme280task(void *argument)
{
  /* USER CODE BEGIN 5 */
	bme280_user_configs bme_cfg;

	bme_cfg.i2c_handle = I2C1;
	bme_cfg.dma_handle = DMA1;
	bme_cfg.dma_stream = LL_DMA_STREAM_0;
	bme_cfg.i2c_addr = bme280_i2c_addr0;

	bme_cfg.config_t.bits.t_sb = 0;
	bme_cfg.config_t.bits.filter = 0;
	bme_cfg.config_t.bits.spi_en = 0;
	bme_cfg.ctrlhum_t.bits.osrs_h = 1;
	bme_cfg.ctrlmeas_t.bits.osrs_t = 1;
	bme_cfg.ctrlmeas_t.bits.osrs_p = 1;
	bme_cfg.ctrlmeas_t.bits.mode = 3;

	my_bme280 = bme280_init(&bme_cfg);

	if(my_bme280 == NULL){
	    while(1);
	}

	const uint32_t bme280_period_ms = 100;
	uint32_t bme280_next_wake = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  bme280_next_wake += bme280_period_ms;
	  osDelayUntil(bme280_next_wake);

	  i2c_job_t job;
	  if(bme280_build_read_job(my_bme280, bme280taskHandle, 0x01, &job) == _bme280_ok)
	  {
	      if(osMessageQueuePut(i2c_job_queueHandle, &job, 0, 0) == osOK)
	      {
	          if(osThreadFlagsWait(0x01, osFlagsWaitAny, 1000) > 0)
	          {
	              bme280_get_value(my_bme280);
	          }
	      }
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startsensorhubtask */
/**
* @brief Function implementing the sensorhubtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startsensorhubtask */
void startsensorhubtask(void *argument)
{
  /* USER CODE BEGIN startsensorhubtask */
	// BAŞLANGIÇ TEMİZLİĞİ - olası bayat semaphore jetonlarını yut
	while(osSemaphoreAcquire(dma_semaphoreHandle, 0) == osOK) { }

  /* Infinite loop */
  for(;;)
  {
      i2c_job_t job;
      if(osMessageQueueGet(i2c_job_queueHandle, &job, NULL, osWaitForever) == osOK)
      {
          if(i2c_manager_read_dma(job.dma_handle, job.dma_stream, job.i2c_handle,
                                   job.dev_addr, job.reg_addr, job.rxdata, job.size) == _i2c_manager_ok)
          {
              if(osSemaphoreAcquire(dma_semaphoreHandle, 100) == osOK)
              {
                  if(job.notify_task != NULL){
                      osThreadFlagsSet(job.notify_task, job.notify_flag);
                  }
              }
              osMutexRelease(i2c_mutexHandle);
          }
      }
  }
  /* USER CODE END startsensorhubtask */
}

/* USER CODE BEGIN Header_startrc522task */
/**
* @brief Function implementing the rc522task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startrc522task */
void startrc522task(void *argument)
{
  /* USER CODE BEGIN startrc522task */
	rc522_user_configs rc522_cfg;

	rc522_cfg.spi_handle = SPI1;
	rc522_cfg.cs_port = rc522_cs_pin_GPIO_Port;
	rc522_cfg.cs_pin = rc522_cs_pin_Pin;
	rc522_cfg.rst_port = rc522_rst_pin_GPIO_Port;
	rc522_cfg.rst_pin = rc522_rst_pin_Pin;

	my_rc522 = rc522_init(&rc522_cfg);

	if(my_rc522 == NULL){
		while(1); // RC522 bulunamadıysa sistemi durdur
	}

	rc522_assign_interrupt_task(my_rc522, rc522taskHandle, 0x04);
	rc522_set_active_irq_device(my_rc522);

	uint8_t card_type[2];
	uint8_t card_uid[5];
	uint8_t status;
	uint8_t card_size;

	uint8_t keyA[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t block_to_read = 4;
	uint8_t block_data[16];
	uint8_t block_data2[16];
	int16_t counter = 0;
	uint8_t counter2 = 0;
	uint8_t data_to_write[16]  = "Enes Was Here :)";
	uint8_t data_to_write2[16] = "Sene Was Here :)";

  /* Infinite loop */
  for(;;)
  {
	  memset(card_type, 0, 2);
	  memset(card_uid, 0, 5);
	  card_size = 0;

	  rc522_stop_crypto1(my_rc522);

	  status = rc522_request(my_rc522, PICC_REQIDL, card_type);
	  if(status == MI_OK){
		  status = rc522_anticoll(my_rc522, card_uid);
		  if(status == MI_OK){

			  if((card_uid[0] == 193) && (card_uid[1] == 99) && (card_uid[2] == 247) && (card_uid[3] == 3) && (card_uid[4] == 86)){
				  card_size = rc522_select_tag(my_rc522, card_uid);
				  if(card_size != 0){
					  status = rc522_auth(my_rc522, PICC_AUTHENT1A, block_to_read, keyA, card_uid);
					  if(status == MI_OK){
						  if(counter == 0){
							  counter++;
							  rc522_write(my_rc522, block_to_read, data_to_write);
						  }
						  status = rc522_read(my_rc522, block_to_read, block_data);
						  rc522_halt(my_rc522);
					  }
				  }
			  }

			  if((card_uid[0] == 162) && (card_uid[1] == 98) && (card_uid[2] == 192) && (card_uid[3] == 1) && (card_uid[4] == 1)){
				  card_size = rc522_select_tag(my_rc522, card_uid);
				  if(card_size != 0){
					  status = rc522_auth(my_rc522, PICC_AUTHENT1A, block_to_read, keyA, card_uid);
					  if(status == MI_OK){
						  if(counter2 == 0){
							  counter2++;
							  rc522_write(my_rc522, block_to_read, data_to_write2);
						  }
						  status = rc522_read(my_rc522, block_to_read, block_data2);
						  rc522_halt(my_rc522);
					  }
				  }
			  }
		  }
	  }
	  osDelay(10);
  }
  /* USER CODE END startrc522task */
}

/* USER CODE BEGIN Header_startmpu6050task */
/**
* @brief Function implementing the mpu6050task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startmpu6050task */
void startmpu6050task(void *argument)
{
  /* USER CODE BEGIN startmpu6050task */
	mpu6050_user_configs mpu_cfg = {0};

	mpu_cfg.i2c_handle = I2C1;
	mpu_cfg.i2c_addr = mpu6050_i2c_addr_0;
	mpu_cfg.dma_handle = DMA1;
	mpu_cfg.dma_stream = LL_DMA_STREAM_0;
	mpu_cfg.sample_rate = 9;

	mpu_cfg.config_t.bits.dlpf_cfg = 3;
	mpu_cfg.config_t.bits.ext_sync_set = 0;
	mpu_cfg.gyro_config_t.bits.fs_sel = 0;
	mpu_cfg.accel_config_t.bits.afs_sel = 0;

	// 1. FIFO'YU VE İÇİNE GİRECEK VERİLERİ AKTİF EDİYORUZ
	mpu_cfg.fifo_en_t.bits.accel_fifo_en = 1;
	mpu_cfg.fifo_en_t.bits.temp_fifo_en = 1;
	mpu_cfg.fifo_en_t.bits.xg_fifo_en = 1;
	mpu_cfg.fifo_en_t.bits.yg_fifo_en = 1;
	mpu_cfg.fifo_en_t.bits.zg_fifo_en = 1;

	mpu_cfg.int_pin_cfg_t.bits.int_level = 0;
	mpu_cfg.int_pin_cfg_t.bits.int_open = 0;
	mpu_cfg.int_pin_cfg_t.bits.latch_int_en = 1;
	mpu_cfg.int_pin_cfg_t.bits.int_rd_clear = 1; // don't make it 0 or interrupt will stay high forever, if you make it 0, then read int_status

	// 2. HEM DATA READY HEM DE FIFO OVERFLOW KESMELERİNİ AKTİF EDİYORUZ
	mpu_cfg.int_enable_t.bits.data_rdy_en = 1;
	mpu_cfg.int_enable_t.bits.fifo_oflow_en = 1;

	// 3. FIFO DONANIMINI GENEL OLARAK AÇIYORUZ
	mpu_cfg.user_ctrl_t.bits.fifo_en = 1;

	mpu_cfg.pwr_mgmt_1_t.bits.sleep = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.clksel = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.temp_dis = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.cycle = 0;
	mpu_cfg.pwr_mgmt_2_t.raw = 0;

	my_mpu6050 = mpu6050_init(&mpu_cfg);
	if(my_mpu6050 == NULL){
		while(1);
	}

	mpu6050_assign_interrupt_task(my_mpu6050, mpu6050taskHandle, 0x01); // EXTI -> flag 0x01
	mpu6050_set_active_irq_device(my_mpu6050);

	if(mpu6050_configurate(my_mpu6050) != _mpu6050_ok){
		while(1);
	}

  /* Infinite loop */
  for(;;)
  {
	  if(osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever) > 0)
	  {
		  mpu6050_int_status_t int_status;

		  // Sensöre sor: "Zili çaldın ama neden çaldın?" (Aynı zamanda kesme Latch'ini indirir)
		  if(mpu6050_read_int_status(my_mpu6050, &int_status) == _mpu6050_ok)
		  {
			  i2c_job_t job;
			  bool job_ready = false;
			  bool is_fifo_job = false;
			  uint16_t bytes_to_read = 0;

			  // ÖNCELİK 1: FIFO Taştıysa (İkisi birden gelse bile önce FIFO boşaltılmak ZORUNDADIR)
			  if(int_status.fifo_overflow)
			  {
				  uint16_t fifo_count;
				  if(mpu6050_read_fifo_count(my_mpu6050, &fifo_count) == _mpu6050_ok && fifo_count >= 14)
				  {
					  bytes_to_read = (fifo_count / 14) * 14;
					  if(bytes_to_read > 1022) bytes_to_read = 1022;

					  if(mpu6050_build_fifo_job(my_mpu6050, bytes_to_read, mpu6050taskHandle, 0x02, &job) == _mpu6050_ok)
					  {
						  job_ready = true;
						  is_fifo_job = true;
					  }
				  }
			  }
			  // ÖNCELİK 2: Sadece Data Ready geldiyse
			  else if(int_status.data_rdy)
			  {
				  if(mpu6050_build_data_job(my_mpu6050, mpu6050taskHandle, 0x02, &job) == _mpu6050_ok)
				  {
					  job_ready = true;
					  is_fifo_job = false; // Bu standart bir Data okumasıdır
				  }
			  }

			  // Hazırlanan iş biletini Kuryeye (Sensor Hub) yolla
			  if(job_ready)
			  {
				  if(osMessageQueuePut(i2c_job_queueHandle, &job, 0, 0) == osOK)
				  {
					  if(osThreadFlagsWait(0x02, osFlagsWaitAny, 500) > 0) // Kurye işi bitirdi
					  {
						  // Eğer bu bir FIFO okumasıysa, ek temizlik işlemlerini yap
						  if(is_fifo_job)
						  {
							  mpu6050_fifo_extract_latest(my_mpu6050, bytes_to_read);
							  mpu6050_reset_fifo(my_mpu6050); // Sensörün içindeki Buffer'ı sıfırla
						  }

						  // İkisi için de ORTAK İŞLEM: Çekilen ham veriyi matematiğe dök
						  mpu6050_get_values(my_mpu6050);
					  }
				  }
			  }
		  }
	  }
  }
  /* USER CODE END startmpu6050task */
}

/* USER CODE BEGIN Header_startspidmatask */
/**
* @brief Function implementing the central SPI DMA worker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startspidmatask */
void startspidmatask(void *argument)
{
  /* USER CODE BEGIN startspidmatask */
  // CubeMX binary semaphore'lari baslangicta dolu olabildigi icin temizle.
  while(osSemaphoreAcquire(spi_semaphoreHandle, 0) == osOK) { }
  while(osSemaphoreAcquire(spi3_semaphoreHandle, 0) == osOK) { }

  for(;;)
  {
    spi_job_t job;
    osStatus_t queue_status;
    queue_status = osMessageQueueGet(spi_job_queueHandle, &job, NULL, osWaitForever);
    if(queue_status != osOK) continue;

    uint32_t result_flag = job.error_flag;
    spi_manager_return_status transfer_status;
    transfer_status = spi_manager_transfer_dma(job.spi_handle, job.dma_handle,
                                               job.rx_stream, job.tx_stream,
                                               job.cs_port, job.cs_pin,
                                               job.txdata, job.rxdata, job.size);
    if(transfer_status == _spi_manager_ok)
    {
      osStatus_t semaphore_status;
      semaphore_status = osSemaphoreAcquire(job.completion_semaphore, 500);
      if(semaphore_status == osOK)
      {
        bool transfer_succeeded;
        transfer_succeeded = spi_manager_last_transfer_succeeded(job.spi_handle);
        if(transfer_succeeded){
          result_flag = job.notify_flag;
        }
        spi_manager_unlock_bus(job.spi_handle);
      }
      else
      {
        spi_manager_abort_transfer(job.spi_handle);
      }
    }

    if(job.notify_task != NULL && result_flag != 0){
      osThreadFlagsSet(job.notify_task, result_flag);
    }
  }
  /* USER CODE END startspidmatask */
}

/* USER CODE BEGIN Header_startnrf24l01TXtask */
/**
* @brief Function implementing the nrf24l01TXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startnrf24l01TXtask */
void startnrf24l01TXtask(void *argument)
{
  /* USER CODE BEGIN startnrf24l01TXtask */
  nrf24l01_user_configs tx_cfg = {0};
  tx_cfg.spi_handle = SPI3;
  tx_cfg.dma_handle = DMA1;
  tx_cfg.rx_stream = LL_DMA_STREAM_2;
  tx_cfg.tx_stream = LL_DMA_STREAM_5;
  tx_cfg.csn_port = nrf24_csn_pin_spi3_GPIO_Port;
  tx_cfg.csn_pin = nrf24_csn_pin_spi3_Pin;
  tx_cfg.ce_port = nrf24_ce_pin_spi3_GPIO_Port;
  tx_cfg.ce_pin = nrf24_ce_pin_spi3_Pin;
  tx_cfg.spi_semaphore = spi3_semaphoreHandle;

  my_nrf_tx = nrf24l01_init(&tx_cfg);
  if(my_nrf_tx == NULL) {
    for(;;) { osDelay(1000); }
  }

  nrf24l01_assign_interrupt_task(my_nrf_tx, nrf24l01TXtaskHandle, 0x01);
  osThreadFlagsClear(0x07);

  // RX taskinin init ve listening islemlerini tamamlamasina zaman ver.
  osDelay(100);

  const uint8_t tx_msg[] = "iron man";
  /* Infinite loop */
  for(;;)
  {
    spi_job_t job;
    nrf24l01_return_status nrf_status;
    nrf_status = nrf24l01_build_tx_job(my_nrf_tx, tx_msg, sizeof(tx_msg) - 1,
                                      nrf24l01TXtaskHandle, 0x02, 0x04, &job);
    if(nrf_status != _nrf24l01_ok)
    {
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    osStatus_t queue_status;
    queue_status = osMessageQueuePut(spi_job_queueHandle, &job, 0, 100);
    if(queue_status != osOK)
    {
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    uint32_t dma_flags = osThreadFlagsWait(0x06, osFlagsWaitAny, 500);
    bool dma_wait_error = (dma_flags & osFlagsError) != 0;
    bool dma_finished = (dma_flags & 0x02) != 0;
    if(dma_wait_error || dma_finished == false)
    {
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    osThreadFlagsClear(0x01);
    nrf_status = nrf24l01_trigger_transmission(my_nrf_tx);
    if(nrf_status != _nrf24l01_ok)
    {
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    uint32_t irq_flags = osThreadFlagsWait(0x01, osFlagsWaitAny, 200);
    bool irq_wait_error = (irq_flags & osFlagsError) != 0;
    bool irq_arrived = (irq_flags & 0x01) != 0;
    if(irq_wait_error || irq_arrived == false)
    {
      nrf24l01_clear_interrupts(my_nrf_tx);
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    nrf24l01_irq_status_t irq_status;
    nrf_status = nrf24l01_get_irq_status(my_nrf_tx, &irq_status);
    if(nrf_status != _nrf24l01_ok)
    {
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
      osDelay(1000);
      continue;
    }

    nrf24l01_clear_irq_sources(my_nrf_tx, irq_status.raw);
    if(irq_status.tx_ds){
      nrf_tx_success_count++;
    }
    else if(irq_status.max_rt){
      flush_tx(my_nrf_tx);
      nrf_tx_error_count++;
    }

    osDelay(1000);
  }
  /* USER CODE END startnrf24l01TXtask */
}

/* USER CODE BEGIN Header_startnrf24l01RXtask */
/**
* @brief Function implementing the nrf24l01RXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startnrf24l01RXtask */
void startnrf24l01RXtask(void *argument)
{
  /* USER CODE BEGIN startnrf24l01RXtask */
  nrf24l01_user_configs rx_cfg = {0};
  rx_cfg.spi_handle = SPI1;
  rx_cfg.dma_handle = DMA2;
  rx_cfg.rx_stream = LL_DMA_STREAM_0;
  rx_cfg.tx_stream = LL_DMA_STREAM_3;
  rx_cfg.csn_port = nrf24_csn_pin_spi1_GPIO_Port;
  rx_cfg.csn_pin = nrf24_csn_pin_spi1_Pin;
  rx_cfg.ce_port = nrf24_ce_pin_spi1_GPIO_Port;
  rx_cfg.ce_pin = nrf24_ce_pin_spi1_Pin;
  rx_cfg.spi_semaphore = spi_semaphoreHandle;

  my_nrf_rx = nrf24l01_init(&rx_cfg);
  if(my_nrf_rx == NULL) {
    for(;;) { osDelay(1000); }
  }

  nrf24l01_assign_interrupt_task(my_nrf_rx, nrf24l01RXtaskHandle, 0x01);
  osThreadFlagsClear(0x07);
  nrf24l01_clear_interrupts(my_nrf_rx);
  nrf24l01_start_listening(my_nrf_rx);

  /* Infinite loop */
  for(;;)
  {
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    bool flag_wait_error = (flags & osFlagsError) != 0;
    bool interrupt_arrived = (flags & 0x01) != 0;
    if(flag_wait_error || interrupt_arrived == false) continue;

    nrf24l01_irq_status_t irq_status = {0};
    nrf24l01_return_status nrf_status;
    nrf_status = nrf24l01_get_irq_status(my_nrf_rx, &irq_status);
    if(nrf_status != _nrf24l01_ok) continue;

    if(irq_status.rx_dr == false)
    {
      nrf24l01_clear_irq_sources(my_nrf_rx, irq_status.raw);
      continue;
    }

    nrf_rx_fifo_count = 0;
    nrf24l01_fifo_status_t fifo_status;
    nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);
    if(nrf_status != _nrf24l01_ok) continue;

    // nRF24L01 ayri bir FIFO-full interrupt'i uretmez.
    // RX_DR geldikten sonra RX_FULL biti, FIFO'nun 3 paket dolu oldugunu gosterir.
    nrf_rx_fifo_was_full = fifo_status.rx_full;

    // FIFO doluysa 3 paket; dolu degilse o anda bulunan 1 veya 2 paket okunur.
    while(fifo_status.rx_empty == false &&
          nrf_rx_fifo_count < NRF24L01_FIFO_DEPTH)
    {
      spi_job_t job;
      nrf_status = nrf24l01_build_rx_fifo_job(my_nrf_rx, nrf24l01RXtaskHandle,
                                              0x02, 0x04, &job);
      if(nrf_status != _nrf24l01_ok) break;

      osStatus_t queue_status;
      queue_status = osMessageQueuePut(spi_job_queueHandle, &job, 0, 100);
      if(queue_status != osOK) break;

      uint32_t dma_flags = osThreadFlagsWait(0x06, osFlagsWaitAny, 500);
      bool dma_wait_error = (dma_flags & osFlagsError) != 0;
      bool dma_finished = (dma_flags & 0x02) != 0;
      if(dma_wait_error || dma_finished == false) break;

      nrf_status = nrf24l01_finish_rx_fifo_job(my_nrf_rx,
                                               nrf_rx_fifo[nrf_rx_fifo_count],
                                               NRF24L01_PAYLOAD_SIZE);
      if(nrf_status != _nrf24l01_ok) break;

      nrf_rx_fifo_count++;
      nrf_rx_packet_count++;
      sayac++;

      // Datasheet sirasi: payload oku, RX_DR temizle, FIFO_STATUS kontrol et.
      nrf24l01_clear_irq_sources(my_nrf_rx, NRF24L01_IRQ_RX_DR);
      nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);
      if(nrf_status != _nrf24l01_ok) break;
    }

    // Okuma sirasinda yeni paket geldiyse kalan FIFO'yu sonraki turda bosalt.
    nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);
    if(nrf_status == _nrf24l01_ok && fifo_status.rx_empty == false){
      osThreadFlagsSet(nrf24l01RXtaskHandle, 0x01);
    }
  }
  /* USER CODE END startnrf24l01RXtask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
