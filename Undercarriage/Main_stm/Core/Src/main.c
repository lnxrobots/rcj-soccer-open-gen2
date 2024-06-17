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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
//#include "cmsis_os2.h"

#include "bno055_stm32.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;

/* Definitions for task_IO */
osThreadId_t task_IOHandle;
uint32_t task_IOBuffer[ 1024 ];
osStaticThreadDef_t task_IOControlBlock;
const osThreadAttr_t task_IO_attributes = {
  .name = "task_IO",
  .cb_mem = &task_IOControlBlock,
  .cb_size = sizeof(task_IOControlBlock),
  .stack_mem = &task_IOBuffer[0],
  .stack_size = sizeof(task_IOBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for task_display */
osThreadId_t task_displayHandle;
uint32_t task_displayBuffer[ 1024 ];
osStaticThreadDef_t task_displayControlBlock;
const osThreadAttr_t task_display_attributes = {
  .name = "task_display",
  .cb_mem = &task_displayControlBlock,
  .cb_size = sizeof(task_displayControlBlock),
  .stack_mem = &task_displayBuffer[0],
  .stack_size = sizeof(task_displayBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for task_compass */
osThreadId_t task_compassHandle;
uint32_t task_compassBuffer[ 3072 ];
osStaticThreadDef_t task_compassControlBlock;
const osThreadAttr_t task_compass_attributes = {
  .name = "task_compass",
  .cb_mem = &task_compassControlBlock,
  .cb_size = sizeof(task_compassControlBlock),
  .stack_mem = &task_compassBuffer[0],
  .stack_size = sizeof(task_compassBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_UART7_Rx */
osThreadId_t task_UART7_RxHandle;
uint32_t task_UART7_RxBuffer[ 1024 ];
osStaticThreadDef_t task_UART7_RxControlBlock;
const osThreadAttr_t task_UART7_Rx_attributes = {
  .name = "task_UART7_Rx",
  .cb_mem = &task_UART7_RxControlBlock,
  .cb_size = sizeof(task_UART7_RxControlBlock),
  .stack_mem = &task_UART7_RxBuffer[0],
  .stack_size = sizeof(task_UART7_RxBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_UART7_Tx */
osThreadId_t task_UART7_TxHandle;
uint32_t task_UART7_TxBuffer[ 1024 ];
osStaticThreadDef_t task_UART7_TxControlBlock;
const osThreadAttr_t task_UART7_Tx_attributes = {
  .name = "task_UART7_Tx",
  .cb_mem = &task_UART7_TxControlBlock,
  .cb_size = sizeof(task_UART7_TxControlBlock),
  .stack_mem = &task_UART7_TxBuffer[0],
  .stack_size = sizeof(task_UART7_TxBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_UART4_Rx */
osThreadId_t task_UART4_RxHandle;
uint32_t task_UART4_RxBuffer[ 1024 ];
osStaticThreadDef_t task_UART4_RxControlBlock;
const osThreadAttr_t task_UART4_Rx_attributes = {
  .name = "task_UART4_Rx",
  .cb_mem = &task_UART4_RxControlBlock,
  .cb_size = sizeof(task_UART4_RxControlBlock),
  .stack_mem = &task_UART4_RxBuffer[0],
  .stack_size = sizeof(task_UART4_RxBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_UART4_Tx */
osThreadId_t task_UART4_TxHandle;
uint32_t task_UART4_TxBuffer[ 1024 ];
osStaticThreadDef_t task_UART4_TxControlBlock;
const osThreadAttr_t task_UART4_Tx_attributes = {
  .name = "task_UART4_Tx",
  .cb_mem = &task_UART4_TxControlBlock,
  .cb_size = sizeof(task_UART4_TxControlBlock),
  .stack_mem = &task_UART4_TxBuffer[0],
  .stack_size = sizeof(task_UART4_TxBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
void task_IO_function(void *argument);
void task_display_function(void *argument);
void task_compass_function(void *argument);
void task_UART7_Rx_function(void *argument);
void task_UART7_Tx_function(void *argument);
void task_UART4_Rx_function(void *argument);
void task_UART4_Tx_function(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SENSOR_COUNT 7
#define MOTOR_COUNT 4
#define BUTTON_COUNT 3
#define BATTERY_COUNT 2
#define SEND_BUFFER_SIZE 128
#define READ_BUFFER_SIZE 128
#define MSG_E_SIZE 80
#define MSG_F_SIZE 152 //72
#define MSG_G_SIZE 40
#define MSG_H_SIZE 48
#define MSG_LIDAR_SIZE 47
#define UART7_TIME_BETWEEN_SEND 10
#define UART4_TIME_BETWEEN_SEND 5 // 10
#define UART4_BLOCK_SIZE_SEND 152 //(sending the whole packet) 128, 48, 8
#define UART7_BLOCK_SIZE_SEND 8
#define BUZZER_ON_TIME 500
#define BUZZER_OFF_TIME 5000
#define BATTERY_COEF1 150 //coef for the sum of battery voltage
#define BATTERY_COEF2 310
#define BATTERY_THRESHOLD 1000 // in centi volts
#define LIDAR_DATA_SIZE 32
#define LIDAR_DATA_COUNT 24 // 16

/*
 * LEds from left to right:
 * L1: blinking after receiving couple messages from rpi5
 * L2: blinking after receiving couple messages from bottom stm
 * L3: blinking when heading from bno055 is received successfully
 *
 * Buttons from left to right:
 * B1: used by rpi5 to start/stop robot movement
 * B2: not used for now
 * B3: used for reseting north of bno055
 *
 *  */
int16_t motorValue[MOTOR_COUNT]= {0,0,0,0};
int16_t dribblerValue = 0;
uint8_t kickerValue = 0;
uint16_t lineSensorValue[SENSOR_COUNT] = {0,0,0,0,0,0,0};
uint16_t lineSensorValue_last[SENSOR_COUNT] = {0,0,0,0,0,0,0};
uint16_t lineSensorValue3_last = 0;
uint16_t gateSensorValue = 0; // inverted values from the light gate
uint16_t ledValue = 0;
uint16_t displayValue[4] = {0,0,0,0};
uint16_t ipAddressValue[4] = {0,0,0,0};
uint16_t headingValue = 0;
uint16_t headingNorth = 0;
uint16_t headingRelative = 0;
uint16_t headingLastValue = 0;
uint16_t headingValue_status0 = 0;
int heading_status0_cnt = 0;
uint16_t buttonValue = 0;
uint16_t buttonState[BUTTON_COUNT] = {0,0,0};
uint16_t buttonState_last[BUTTON_COUNT] = {0,0,0};
uint8_t buzzerValue = 0;
uint32_t buzzerClock = 0;
uint16_t batteryValue[BATTERY_COUNT] = {0,0};
uint16_t batteryVoltage[BATTERY_COUNT] = {0,0}; //real voltage of the batteries in centiVolts (Voltage times 100); 0 - right, 1 - left battery
int LowBattCnt = 0;

//huart7: communication with bottom stm
//char huart7_RxBuffer[READ_BUFFER_SIZE];
char huart7_TxBuffer[SEND_BUFFER_SIZE];
int huart7_TxBuffer_toSend = 0;
int huart7_TxBuffer_sending = 0;
int huart7_TxBuffer_sent = 0;

uint32_t huart7_TxStart = 0;
uint32_t huart7_TxDuration = 0;
uint32_t huart7_TxInProgress = 0;
uint32_t huart7_TxLastSent = 0;
int huart7_TxOkCnt = 0;

char huart7_RxBuffer1[READ_BUFFER_SIZE];
char huart7_RxBuffer2[READ_BUFFER_SIZE];
int huart7_RxBuffer1_state = 0; // 0 - empty, 1 - receiving, 2 - ready
int huart7_RxBuffer2_state = 0;
int huart7_RxBuffer1_len = 0;
int huart7_RxBuffer2_len = 0;
int huart7_RxBuffer_idx = 0; // index into which is the interrupt receiving
int huart7_RxErrCnt = 0;
int huart7_RxOkCnt2 = 0;
int huart7_RxErrCnt3 = 0;
int huart7_RxErrCnt4 = 0;

//huart4: communication with rpi5
char huart4_RxBuffer[READ_BUFFER_SIZE];
char huart4_TxBuffer[MSG_F_SIZE]; // SEND_BUFFER_SIZE
int huart4_TxBuffer_toSend = 0;
int huart4_TxBuffer_sending = 0;
int huart4_TxBuffer_sent = 0;

uint32_t huart4_TxStart = 0;
uint32_t huart4_TxDuration = 0;
uint32_t huart4_TxInProgress = 0;
uint32_t huart4_TxLastSent = 0;
int huart4_TxOkCnt = 0;

char huart4_RxBuffer1[READ_BUFFER_SIZE];
char huart4_RxBuffer2[READ_BUFFER_SIZE];
int huart4_RxBuffer1_state = 0; // 0 - empty, 1 - receiving, 2 - ready
int huart4_RxBuffer2_state = 0;
int huart4_RxBuffer1_len = 0;
int huart4_RxBuffer2_len = 0;
int huart4_RxBuffer_idx = 0; // index into which is the interrupt receiving
int huart4_RxErrCnt = 0;
int huart4_RxOkCnt2 = 0;
int huart4_RxErrCnt3 = 0;
int huart4_RxErrCnt4 = 0;

uint8_t huart5_RxBuffer1[READ_BUFFER_SIZE];
int huart5_RxBuffer1_len = 0;
int huart5_RxErrCnt = 0;
int huart5_RxOkCnt2 = 0;
int huart5_RxErrCnt3 = 0;
//int huart5_RxErrCnt4 = 0;

uint8_t lidar_dataBuffer[LIDAR_DATA_COUNT][LIDAR_DATA_SIZE];
int lidar_readIdx = 0;
int lidar_writeIdx = 0;
int lidar_ErrCnt = 0;
int lidar_map[MSG_LIDAR_SIZE] = {-1,-1, 0, 1, 4, 5,
								 8,9,-1,10,11,-1,12,13,-1,14,15,-1,16,17,-1,18,19,-1,20,21,-1,22,23,-1,24,25,-1,26,27,-1,28,29,-1,30,31,-1,
								 6, 7, 2, 3};

uint32_t timestamps[16];
int errcnt = 0;
int errcnt2 = 0;
int errcnt3 = 0;
//uint32_t ts[32];
//HAL_StatusTypeDef ts2[32];
//int tsc = 0;

int bnoErr = 0;

uint32_t led_lastTime = 0;
uint8_t led_lastValue= 0;

//osMutexId_t i2c_mutex_id;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//vyhodnotenie dlzky transmitu
	// nastavenie flagu o ukonceni komunikacie
	if(huart == &huart7){
		//huart7_TxDuration = HAL_GetTick() - huart7_TxStart;
		huart7_TxInProgress = 2;
		//huart7_TxErrCnt++;
	}else
	if(huart == &huart4){
		//huart4_TxDuration = HAL_GetTick() - huart4_TxStart;
		huart4_TxInProgress = 2;
		//huart4_TxErrCnt++;
	}else
	if(huart == _bno055_uart_port){
		bno055_TxInProgress = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart7){
		huart7_RxErrCnt++;

		if(huart7_RxBuffer_idx == 0){
			if(huart7_RxBuffer1_len > 0 || (huart7_RxBuffer1_len == 0 && huart7_RxBuffer1[0] == '{')) {
				huart7_RxBuffer1_len++;
			}
			if(huart7_RxBuffer1_len == MSG_H_SIZE){
				if(huart7_RxBuffer1[0] == '{' && huart7_RxBuffer1[2] == 'h' && huart7_RxBuffer1[MSG_H_SIZE-2] == '}'){
					huart7_RxBuffer1_state = 2;
					huart7_RxBuffer_idx = 1;
					huart7_RxBuffer2_state = 1;
					huart7_RxBuffer2_len = 0;
					huart7_RxOkCnt2++;
				} else {
					huart7_RxBuffer1_len = 0;
					huart7_RxErrCnt3++;
				}
			}
		}else {
			if(huart7_RxBuffer2_len > 0 || (huart7_RxBuffer2_len == 0 && huart7_RxBuffer2[0] == '{')) {
				huart7_RxBuffer2_len++;
			}
			if(huart7_RxBuffer2_len == MSG_H_SIZE){
				if(huart7_RxBuffer2[0] == '{' && huart7_RxBuffer2[2] == 'h' && huart7_RxBuffer2[MSG_H_SIZE-2] == '}'){
					huart7_RxBuffer2_state = 2;
					huart7_RxBuffer_idx = 0;
					huart7_RxBuffer1_state = 1;
					huart7_RxBuffer1_len = 0;
					huart7_RxOkCnt2++;
				} else {
					huart7_RxBuffer2_len = 0;
					huart7_RxErrCnt3++;
				}
			}
		}

		if(huart7_RxBuffer_idx == 0){
		    HAL_UART_Receive_IT(&huart7, (uint8_t *)&huart7_RxBuffer1[huart7_RxBuffer1_len], 1);
		} else {
			HAL_UART_Receive_IT(&huart7, (uint8_t *)&huart7_RxBuffer2[huart7_RxBuffer2_len], 1);
		}

	} else
	if(huart == &huart4){
		huart4_RxErrCnt++;

		if(huart4_RxBuffer_idx == 0){
			if(huart4_RxBuffer1_len > 0 || (huart4_RxBuffer1_len == 0 && huart4_RxBuffer1[0] == '{')) {
				huart4_RxBuffer1_len++;
			}
			if(huart4_RxBuffer1_len == MSG_E_SIZE){
				if(huart4_RxBuffer1[0] == '{' && huart4_RxBuffer1[2] == 'e' && huart4_RxBuffer1[MSG_E_SIZE-1] == '\n'){
					huart4_RxBuffer1_state = 2;
					huart4_RxBuffer_idx = 1;
					huart4_RxBuffer2_state = 1;
					huart4_RxBuffer2_len = 0;
					huart4_RxOkCnt2++;
				} else {
					huart4_RxBuffer1_len = 0;
					huart4_RxErrCnt3++;
				}
			}
		}else {
			if(huart4_RxBuffer2_len > 0 || (huart4_RxBuffer2_len == 0 && huart4_RxBuffer2[0] == '{')) {
				huart4_RxBuffer2_len++;
			}
			if(huart4_RxBuffer2_len == MSG_E_SIZE){
				if(huart4_RxBuffer2[0] == '{' && huart4_RxBuffer2[2] == 'e' && huart4_RxBuffer2[MSG_E_SIZE-1] == '\n'){
					huart4_RxBuffer2_state = 2;
					huart4_RxBuffer_idx = 0;
					huart4_RxBuffer1_state = 1;
					huart4_RxBuffer1_len = 0;
					huart4_RxOkCnt2++;
				} else {
					huart4_RxBuffer2_len = 0;
					huart4_RxErrCnt3++;
				}
			}
		}

		if(huart4_RxBuffer_idx == 0){
			HAL_UART_Receive_IT(&huart4, (uint8_t *)&huart4_RxBuffer1[huart4_RxBuffer1_len], 1);
		} else {
			HAL_UART_Receive_IT(&huart4, (uint8_t *)&huart4_RxBuffer2[huart4_RxBuffer2_len], 1);
		}

	}else
	if(huart == _bno055_uart_port){
		if((bno055_RxBuffer_len > 0) || ((bno055_RxBuffer_len == 0) && (bno055_RxBuffer[0] == bno055_RxBuffer_first))){
			bno055_RxBuffer_len++;
		}
		if(bno055_RxBuffer_len >= bno055_RxBuffer_toRead){
			bno055_RxInProgress = 0;
		} else {
			HAL_UART_Receive_IT(_bno055_uart_port, &bno055_RxBuffer[bno055_RxBuffer_len], 1);
		}
	} else
	if(huart == &huart5){
		huart5_RxErrCnt++;

		if(huart5_RxBuffer1_len > 1 ||
		   (huart5_RxBuffer1_len == 0 && huart5_RxBuffer1[0] == 0x54) ||
		   (huart5_RxBuffer1_len == 1 && huart5_RxBuffer1[1] == 0x2C)) {
			int idx = lidar_map[huart5_RxBuffer1_len];
			if(idx >= 0){
				lidar_dataBuffer[lidar_writeIdx][idx] = huart5_RxBuffer1[huart5_RxBuffer1_len];
				//FIXME add up tcrc8
			}
			huart5_RxBuffer1_len++;
		} else {
			huart5_RxBuffer1_len = 0;
		}
		if(huart5_RxBuffer1_len == MSG_LIDAR_SIZE){
			uint8_t crc8 = huart5_RxBuffer1[MSG_LIDAR_SIZE-1]; // FIXME add CRC check
			if(huart5_RxBuffer1[0] == 0x54 && huart5_RxBuffer1[1] == 0x2C && huart5_RxBuffer1[MSG_LIDAR_SIZE-1] == crc8){
				int writeIdx = lidar_writeIdx + 1;
				if(writeIdx >= LIDAR_DATA_COUNT-1){
					writeIdx = 0;
				}
				if(writeIdx == lidar_readIdx){
					lidar_readIdx++;
					if(lidar_readIdx >= LIDAR_DATA_COUNT-1){
						lidar_readIdx = 0;
					}
					lidar_ErrCnt++;
				}
				lidar_writeIdx = writeIdx;

				huart5_RxBuffer1_len = 0;
				huart5_RxOkCnt2++;
			} else {
				huart5_RxBuffer1_len = 0;
				huart5_RxErrCnt3++;
			}
		}

		HAL_UART_Receive_IT(&huart5, (uint8_t *)&huart5_RxBuffer1[huart5_RxBuffer1_len], 1);

	}
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
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//int button1_state = 0;
	//int button2_state = 0;

	/*i2c_mutex_id = osMutexNew(NULL);
	if (i2c_mutex_id == NULL)  {
		Error_Handler();
	}*/
	/*osStatus_t  status;
	status = osMutexAcquire(mutex_id, 0U);
    if (status != osOK)  {
    	Error_Handler();
    }

    status = osMutexRelease(mutex_id);
    if (status != osOK)  {
    	Error_Handler();
    }*/
    huart5_RxBuffer1_len = 0;
    HAL_StatusTypeDef res;
    res = HAL_UART_Receive_IT(&huart5, (uint8_t *)huart5_RxBuffer1, 1);
    if (res != HAL_OK)
    {
  	    Error_Handler();
    }
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_IO */
  task_IOHandle = osThreadNew(task_IO_function, NULL, &task_IO_attributes);

  /* creation of task_display */
  task_displayHandle = osThreadNew(task_display_function, NULL, &task_display_attributes);

  /* creation of task_compass */
  task_compassHandle = osThreadNew(task_compass_function, NULL, &task_compass_attributes);

  /* creation of task_UART7_Rx */
  task_UART7_RxHandle = osThreadNew(task_UART7_Rx_function, NULL, &task_UART7_Rx_attributes);

  /* creation of task_UART7_Tx */
  task_UART7_TxHandle = osThreadNew(task_UART7_Tx_function, NULL, &task_UART7_Tx_attributes);

  /* creation of task_UART4_Rx */
  task_UART4_RxHandle = osThreadNew(task_UART4_Rx_function, NULL, &task_UART4_Rx_attributes);

  /* creation of task_UART4_Tx */
  task_UART4_TxHandle = osThreadNew(task_UART4_Tx_function, NULL, &task_UART4_Tx_attributes);

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
	//int j = 0;
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L3_Pin|L2_Pin|L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L3_Pin L2_Pin L1_Pin */
  GPIO_InitStruct.Pin = L3_Pin|L2_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B3_Pin */
  GPIO_InitStruct.Pin = B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B1_Pin GAL_M_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B1_Pin|GAL_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_IO_function */
/**
  * @brief  Function implementing the task_IO thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_IO_function */
void task_IO_function(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	//int j = 0;
	int Buzz = 0;
	//int dribbler_state = 0;
	while(1)
	{
		/*osDelay(500);
	}
	{*/

		buttonState_last[0] = buttonState[0];
		buttonState_last[1] = buttonState[1];
		buttonState_last[2] = buttonState[2];

		buttonState[0] = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		buttonState[1] = HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin);
		buttonState[2] = HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin);

		buttonValue = buttonState[0] + buttonState[1]*10 + buttonState[2]*100;

		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);

		HAL_ADC_PollForConversion(&hadc1, 10);
		batteryValue[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		HAL_ADC_PollForConversion(&hadc2, 10);
		batteryValue[1] = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);

		batteryVoltage[1] = 100 * ((int)batteryValue[1]) / BATTERY_COEF2;
		batteryVoltage[0] = 100 * ((int)batteryValue[0]) / BATTERY_COEF1 - batteryVoltage[1];

		if(batteryVoltage[0] <= BATTERY_THRESHOLD || batteryVoltage[1] <= BATTERY_THRESHOLD){
			LowBattCnt++;
		} else {
			LowBattCnt = 0;
		}
		if(LowBattCnt >= 30 && HAL_GetTick() > 1000){
			Buzz = 1;
		}
		//when one of the batteryValues is below 3000 the voltage on the batteries is below 10V each
		if(Buzz){
			if(buzzerClock + BUZZER_OFF_TIME < HAL_GetTick()){
				if(buzzerClock + BUZZER_OFF_TIME + BUZZER_ON_TIME < HAL_GetTick()){
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
					buzzerClock = HAL_GetTick();
				} else {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
				}

			}
		}
		/*if(buttonState[1] == 1){
			kickerValue = 1;
		} else {
			kickerValue = 0;
		}

		if(buttonState[0] != buttonState_last[0]){
			if(dribbler_state == 0){
				dribbler_state = 1;
				dribblerValue = 200; //400
				//HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			}else
			if(dribbler_state == 1){
				dribbler_state = 2;
			}else
			if(dribbler_state == 2){
				dribbler_state = 3;
				dribblerValue = 0;
				//HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
			}else
			if(dribbler_state == 3){
				dribbler_state = 0;
			}
		}*/
		/*if(buttonState[1] == 1){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
		} else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		}*/
		/*if(buttonState[0] == 1){
			motorValue[0] = -100;
			motorValue[1] = -100;
			motorValue[2] = -100;
			motorValue[3] = -100;
		} else {
			motorValue[0] = 0;
			motorValue[1] = 0;
			motorValue[2] = 0;
			motorValue[3] = 0;
		}*/

		/*j++;
		if(buttonState[0] == 1){
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
		}else
		if(j > 100){
			HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
			j = 0;
		}*/
		osDelay(5);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_display_function */
/**
* @brief Function implementing the task_display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_display_function */
void task_display_function(void *argument)
{
  /* USER CODE BEGIN task_display_function */
	ssd1306_Init();
	int res5 = 0;
	/* Infinite loop */
	while(1)
	{
		/*osDelay(500);
	}
	{*/
		res5++;
		//if(motorValue[0] == 0 && motorValue[0] == 0 && motorValue[0] == 0 && motorValue[0] == 0 && (led_lastTime+500 < HAL_GetTick())){
		char buffer[32];
		char buffer2[32];
		char buffer3[32];
		//sprintf(buffer, "H: %d\n", headingValue);
		sprintf(buffer, "IP: %d.%d.%d.%d", (int)ipAddressValue[0],(int)ipAddressValue[1],(int)ipAddressValue[2],(int)ipAddressValue[3]);
		//sprintf(buffer, "B: %d, %d, %d", (int)bno055_readData_okCnt,(int)bno055_readData_errCnt,(int)bno055_readData_errCnt2);
		sprintf(buffer2, "H: %d/%d S: %d", (int)headingRelative, (int)headingNorth, (int)lineSensorValue3_last);
		sprintf(buffer3, "C: %d,%d,%d V:%d,%d", (int)res5%100, (int)motorValue[0], (int)buttonValue, (int)batteryVoltage[0]/10, (int)batteryVoltage[1]/10);

		/*osStatus_t  status;
		status = osMutexAcquire(i2c_mutex_id, osWaitForever);
		if (status == osOK)*/  {
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString(buffer, Font_6x8, White); // Font_11x18
			ssd1306_SetCursor(0, 10);
			ssd1306_WriteString(buffer2, Font_6x8, White); // Font_11x18
			ssd1306_SetCursor(0, 20);
			ssd1306_WriteString(buffer3, Font_6x8, White); // Font_11x18
			ssd1306_UpdateScreen();

			//osDelay(3);
			//HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
			//osMutexRelease(i2c_mutex_id);
		}
		//}
		osDelay(500);
	}
  /* USER CODE END task_display_function */
}

/* USER CODE BEGIN Header_task_compass_function */
/**
* @brief Function implementing the task_compass thread.
* @param argument: Not used
* @retval None
*/
#define BNO055_RESET_TIMEOUT 500
int bno055_heading_okCnt = 0;
/* USER CODE END Header_task_compass_function */
void task_compass_function(void *argument)
{
  /* USER CODE BEGIN task_compass_function */
#ifdef BNO055_I2C_ENABLED
	bno055_assignI2C(&hi2c2);
#endif
#ifdef BNO055_UART_ENABLED
  	bno055_assignUART(&huart3);
#endif
	bno055_setup();
	bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);

	/* Infinite loop */
	int j = 0;
	uint32_t lastOkTm = HAL_GetTick();
	while(1)
	{
		osDelay(3);
		//Nakodit do led/button modulu reset compassu
		if(buttonState[2] == 1){
			headingNorth = headingValue;
		}
		{


			if (bno055_readData_err != 0){
				uint8_t res5 = bno055_getSystemStatus();
				if ((bno055_readData_err == 0) && (res5 == 5)){
					lastOkTm = HAL_GetTick();
				}
			}


			if(lastOkTm + BNO055_RESET_TIMEOUT < HAL_GetTick()){// ((bno055_readData_err == 0) && (res5 != 5)) ||
				lastOkTm = HAL_GetTick();
#ifdef BNO055_I2C_ENABLED
				HAL_I2C_DeInit(&hi2c2);
				MX_I2C2_Init();
				bno055_assignI2C(&hi2c2);
#endif
#ifdef BNO055_UART_ENABLED
				HAL_UART_DeInit(&huart3);
				MX_USART3_UART_Init();
				bno055_assignUART(&huart3);
#endif
				bno055_setup();
				bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);
				bnoErr++;
				int hh = (int)headingNorth -(int)headingLastValue;
				while(hh < 0) {
					hh+=360*100;
				}
				headingNorth = hh;
				headingValue = headingLastValue;

			}

			bno055_vector_t v = bno055_getVectorEuler();
			if(bno055_readData_err == 0){
				lastOkTm = HAL_GetTick();

				headingLastValue = headingValue;
				headingValue = (uint16_t)(v.x*100);
				bno055_heading_okCnt++;

				int hh = (int)headingValue - (int)headingNorth;
				while(hh >= 360*100) {
					hh-=360*100;
				}
				while(hh < 0) {
					hh+=360*100;
				}
				headingRelative = hh;
				j++;
			}

		}

		if(buttonState[2] == 1){
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
		}else
		if(j > 20){
			HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
			j = 0;
		}
		//HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
	}
  /* USER CODE END task_compass_function */
}

/* USER CODE BEGIN Header_task_UART7_Rx_function */
/**
* @brief Function implementing the task_UART7_Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_UART7_Rx_function */
void task_UART7_Rx_function(void *argument)
{
  /* USER CODE BEGIN task_UART7_Rx_function */
	huart7_RxBuffer_idx = 0;
	huart7_RxBuffer1_state = 1;
	huart7_RxBuffer1_len = 0;
	HAL_StatusTypeDef res;
	res = HAL_UART_Receive_IT(&huart7, (uint8_t *)huart7_RxBuffer1, 1); // MSG_H_SIZE
	if (res != HAL_OK)
	{
		Error_Handler();
	}
	int j = 0;
	/* Infinite loop */
	while(1)
	{
		/*osDelay(500);
    }
    {*/
		char* huart7_RxBuffer = NULL;
		int idx = -1;
		if(huart7_RxBuffer1_state == 2){
			idx = 0;
			huart7_RxBuffer1_state = 3;
			huart7_RxBuffer = huart7_RxBuffer1;
		} else
		if(huart7_RxBuffer2_state == 2){
			idx = 1;
			huart7_RxBuffer2_state = 3;
			huart7_RxBuffer = huart7_RxBuffer2;
		}

		if(idx >= 0){
			if(huart7_RxBuffer[0] == '{' && huart7_RxBuffer[2] == 'h' && huart7_RxBuffer[MSG_H_SIZE-2] == '}'){
				int i = 6;
				while((huart7_RxBuffer[i] != '\"') && (huart7_RxBuffer[i] != '\'') && (i< MSG_H_SIZE)){
					i++;
				}
				if(i<MSG_H_SIZE){
					huart7_RxBuffer[i] = 0;
					int res3, d1, d2, d3, d4, d5, d6, d7, d8;
					res3 = sscanf(&huart7_RxBuffer[6], "%d,%d,%d,%d,%d,%d,%d,%d", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8);
					if(res3 == 8){
						if(d1 > lineSensorValue[0]){
							lineSensorValue[0] = d1;
						}
						if(d2 > lineSensorValue[1]){
							lineSensorValue[1] = d2;
						}
						if(d3 > lineSensorValue[2]){
							lineSensorValue[2] = d3;
						}
						lineSensorValue3_last = d3;
						if(d4 > lineSensorValue[3]){
							lineSensorValue[3] = d4;
						}
						if(d5 > lineSensorValue[4]){
							lineSensorValue[4] = d5;
						}
						if(d6 > lineSensorValue[5]){
							lineSensorValue[5] = d6;
						}
						if(d7 > lineSensorValue[6]){
							lineSensorValue[6] = d7;
						}
						if(d8 > gateSensorValue){
							gateSensorValue = d8;
						}
						j++;
					}
				}
			}
			if(idx == 0){
				huart7_RxBuffer1_state = 0;
			} else
			if(idx == 1){
				huart7_RxBuffer2_state = 0;
			}
		}
		if(j > 20){
			HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
			j = 0;
		}
		osDelay(1);
	}
  /* USER CODE END task_UART7_Rx_function */
}

/* USER CODE BEGIN Header_task_UART7_Tx_function */
/**
* @brief Function implementing the task_UART7_Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_UART7_Tx_function */
void task_UART7_Tx_function(void *argument)
{
  /* USER CODE BEGIN task_UART7_Tx_function */
  /* Infinite loop */
	while(1)
	{
		/*osDelay(500);
	}
	{*/
		if((huart7_TxInProgress == 0) && (HAL_GetTick() >= huart7_TxLastSent + UART7_TIME_BETWEEN_SEND)){
			huart7_TxStart = HAL_GetTick();
			huart7_TxInProgress = 1;

			sprintf(huart7_TxBuffer, "{\"g\":\"%d,%d,%d,%d,%d,%d\"", (int)motorValue[0], (int)motorValue[1], (int)motorValue[2], (int)motorValue[3], (int)dribblerValue, (int)kickerValue);
			uint16_t huart7_TxBuffer_len = strlen(huart7_TxBuffer);
			while(huart7_TxBuffer_len< MSG_G_SIZE-2){
				huart7_TxBuffer[huart7_TxBuffer_len++] = ' ';
			}
			huart7_TxBuffer[huart7_TxBuffer_len++] = '}';
			huart7_TxBuffer[huart7_TxBuffer_len++] = '\n';

			HAL_StatusTypeDef res2;
			huart7_TxBuffer_toSend = huart7_TxBuffer_len;
			huart7_TxBuffer_sending = UART7_BLOCK_SIZE_SEND;
			huart7_TxBuffer_sent = 0;
			res2 = HAL_UART_Transmit_IT(&huart7, (uint8_t *)huart7_TxBuffer, huart7_TxBuffer_sending);

			if(res2 == HAL_OK){
				huart7_TxLastSent = huart7_TxStart;
			}else{
				huart7_TxInProgress = 0;
			}
		}else
		if(huart7_TxInProgress == 2){
			huart7_TxBuffer_sent += huart7_TxBuffer_sending;
			if(huart7_TxBuffer_sent < huart7_TxBuffer_toSend){
				huart7_TxInProgress = 1;
				HAL_StatusTypeDef res2;
				huart7_TxBuffer_sending = huart7_TxBuffer_toSend - huart7_TxBuffer_sent;
				if(huart7_TxBuffer_sending > UART7_BLOCK_SIZE_SEND){
					huart7_TxBuffer_sending = UART7_BLOCK_SIZE_SEND;
				}

				res2 = HAL_UART_Transmit_IT(&huart7, (uint8_t *)&huart7_TxBuffer[huart7_TxBuffer_sent], huart7_TxBuffer_sending);
				if(res2 != HAL_OK){
					huart7_TxInProgress = 0;
				}
			} else {
				huart7_TxDuration = HAL_GetTick() - huart7_TxStart;
				huart7_TxInProgress = 0;
				huart7_TxOkCnt++;
			}
		}
		osDelay(1);
	}
  /* USER CODE END task_UART7_Tx_function */
}

/* USER CODE BEGIN Header_task_UART4_Rx_function */
/**
* @brief Function implementing the task_UART4_Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_UART4_Rx_function */
void task_UART4_Rx_function(void *argument)
{
  /* USER CODE BEGIN task_UART4_Rx_function */
	huart4_RxBuffer_idx = 0;
	huart4_RxBuffer1_state = 1;
	huart4_RxBuffer1_len = 0;
	HAL_StatusTypeDef res;
	res = HAL_UART_Receive_IT(&huart4, (uint8_t *)huart4_RxBuffer1, 1);
	if (res != HAL_OK)
	{
		Error_Handler();
	}
	int j = 0;
	/* Infinite loop */
	while(1)
	{
		/*osDelay(500);
	}
	{*/
		char* huart4_RxBuffer = NULL;
		int idx = -1;
		if(huart4_RxBuffer1_state == 2){
			idx = 0;
			huart4_RxBuffer1_state = 3;
			huart4_RxBuffer = huart4_RxBuffer1;
		} else
		if(huart4_RxBuffer2_state == 2){
			idx = 1;
			huart4_RxBuffer2_state = 3;
			huart4_RxBuffer = huart4_RxBuffer2;
		}

		if(idx >= 0){
			if(huart4_RxBuffer[0] == '{' && huart4_RxBuffer[2] == 'e' && huart4_RxBuffer[MSG_E_SIZE-1] == '\n'){
				int i = 6;
				while((huart4_RxBuffer[i] != '\"') && (huart4_RxBuffer[i] != '\'') && (i< MSG_E_SIZE)){
					i++;
				}
				if(i<MSG_E_SIZE){
					huart4_RxBuffer[i] = 0;

					int res3, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16;
					res3 = sscanf(&huart4_RxBuffer[6], "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9, &d10, &d11, &d12, &d13, &d14, &d15, &d16);
					if(res3 == 16){
						motorValue[0] = d1;
						motorValue[1] = d2;
						motorValue[2] = d3;
						motorValue[3] = d4;
						ledValue = d5;
						displayValue[0] = d6;
						displayValue[1] = d7;
						displayValue[2] = d8;
						displayValue[3] = d9;
						ipAddressValue[0] = d10;
						ipAddressValue[1] = d11;
						ipAddressValue[2] = d12;
						ipAddressValue[3] = d13;
						dribblerValue = d14;
						kickerValue = d15;
						buzzerValue = d16;
						//lastMsgRecieved = HAL_GetTick();
						j++;
					}
				}
			}
			if(idx == 0){
				huart4_RxBuffer1_state = 0;
			} else
			if(idx == 1){
				huart4_RxBuffer2_state = 0;
			}
		}


		if(buttonState[0] == 1){
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
		}else
		if(j > 20){
			HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
			j = 0;
		}

		osDelay(1);
	}
  /* USER CODE END task_UART4_Rx_function */
}

/* USER CODE BEGIN Header_task_UART4_Tx_function */
/**
* @brief Function implementing the task_UART4_Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_UART4_Tx_function */
void task_UART4_Tx_function(void *argument)
{
  /* USER CODE BEGIN task_UART4_Tx_function */
  /* Infinite loop */
	int lineSenesorValue_max = 0;
  while(1)
  {
	  /*osDelay(500);
  }
  {*/
	  if((huart4_TxInProgress == 0) && (HAL_GetTick() >= huart4_TxLastSent + UART4_TIME_BETWEEN_SEND)){
		  huart4_TxStart = HAL_GetTick();
		  huart4_TxInProgress = 1;
		  lineSenesorValue_max = 0;
		  for (int i = 0; i < SENSOR_COUNT; i++) {
			  if(lineSensorValue[i] > lineSenesorValue_max){
				  lineSenesorValue_max = lineSensorValue[i];
			  }
		  }
		  uint16_t *value = lineSensorValue;
		  if(lineSenesorValue_max <= 0){
			  value = lineSensorValue_last;
		  }
		  uint16_t huart4_TxBuffer_len = 0;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '{';
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '#';
		  huart4_TxBuffer[huart4_TxBuffer_len++] = 'f';
		  huart4_TxBuffer[huart4_TxBuffer_len++] = 145; // size
		  for (int i = 0; i < SENSOR_COUNT; i++) { // sensors
			  huart4_TxBuffer[huart4_TxBuffer_len++] = value[i] & 0xFF;
			  huart4_TxBuffer[huart4_TxBuffer_len++] = (value[i] >> 8) & 0xFF;
		  }
		  huart4_TxBuffer[huart4_TxBuffer_len++] = headingRelative & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = (headingRelative >> 8) & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = buttonValue & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = gateSensorValue & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = (gateSensorValue >> 8) & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = batteryVoltage[0] & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = (batteryVoltage[0] >> 8) & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = batteryVoltage[1] & 0xFF;
		  huart4_TxBuffer[huart4_TxBuffer_len++] = (batteryVoltage[1] >> 8) & 0xFF;

		  if(lidar_readIdx != lidar_writeIdx){ // speed
			  huart4_TxBuffer[huart4_TxBuffer_len++] = lidar_dataBuffer[lidar_readIdx][0];
			  huart4_TxBuffer[huart4_TxBuffer_len++] = lidar_dataBuffer[lidar_readIdx][1];
		  } else {
			  huart4_TxBuffer[huart4_TxBuffer_len++] = 0;
			  huart4_TxBuffer[huart4_TxBuffer_len++] = 0;
		  }

		  for (int i = 0; i < 4; i++) {
			  if(lidar_readIdx != lidar_writeIdx){ // speed
				  memcpy(&huart4_TxBuffer[huart4_TxBuffer_len], &lidar_dataBuffer[lidar_readIdx][2], LIDAR_DATA_SIZE - 2);
				  huart4_TxBuffer_len += LIDAR_DATA_SIZE - 2;
				  lidar_readIdx++;
				  if(lidar_readIdx >= LIDAR_DATA_COUNT-1){
					  lidar_readIdx = 0;
				  }
			  } else {
				  memset(&huart4_TxBuffer[huart4_TxBuffer_len], 0, LIDAR_DATA_SIZE - 2);
				  huart4_TxBuffer_len += LIDAR_DATA_SIZE - 2;
			  }
		  }

		  huart4_TxBuffer[huart4_TxBuffer_len++] = 0; // crc8
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '}';
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '\n';

		  if(lineSenesorValue_max > 0){
			  for (int i = 0; i < SENSOR_COUNT; i++) {
				  lineSensorValue_last[i] = lineSensorValue[i];
				  lineSensorValue[i] = 0;
			  }
		  }
		  /*if(lineSenesorValue_max > 0){
			  sprintf(huart4_TxBuffer, "{\"f\":\"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\"", (int) lineSensorValue[0], (int)lineSensorValue[1], (int)lineSensorValue[2], (int)lineSensorValue[3],
					  (int)lineSensorValue[4], (int)lineSensorValue[5], (int)lineSensorValue[6], (int)headingRelative, (int)buttonValue, (int)gateSensorValue, (int)batteryVoltage[0], (int)batteryVoltage[1]);
			  for (int i = 0; i < SENSOR_COUNT; i++) {
				  lineSensorValue_last[i] = lineSensorValue[i];
				  lineSensorValue[i] = 0;
			  }
		  } else {
			  sprintf(huart4_TxBuffer, "{\"f\":\"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\"", (int) lineSensorValue_last[0], (int)lineSensorValue_last[1], (int)lineSensorValue_last[2], (int)lineSensorValue_last[3],
					  (int)lineSensorValue_last[4], (int)lineSensorValue_last[5], (int)lineSensorValue_last[6], (int)headingRelative, (int)buttonValue, (int)gateSensorValue, (int)batteryVoltage[0], (int)batteryVoltage[1]);
		  }
		  uint16_t huart4_TxBuffer_len = strlen(huart4_TxBuffer);
		  while(huart4_TxBuffer_len< MSG_F_SIZE-2){
			  huart4_TxBuffer[huart4_TxBuffer_len++] = ' ';
		  }
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '}';
		  huart4_TxBuffer[huart4_TxBuffer_len++] = '\n';*/

		  HAL_StatusTypeDef res2;
		  huart4_TxBuffer_toSend = huart4_TxBuffer_len;
		  huart4_TxBuffer_sending = UART4_BLOCK_SIZE_SEND;
		  huart4_TxBuffer_sent = 0;
		  res2 = HAL_UART_Transmit_IT(&huart4, (uint8_t *)huart4_TxBuffer, huart4_TxBuffer_sending); // huart4_TxBuffer_len
		  //res2 = HAL_UART_Transmit(&huart4, (uint8_t *)huart4_TxBuffer, huart4_TxBuffer_len, 1000);

		  if(res2 == HAL_OK){
			  huart4_TxLastSent = huart4_TxStart;
		  }else{
			  huart4_TxInProgress = 0;
		  }
	  } else
	  if(huart4_TxInProgress == 2){
		  huart4_TxBuffer_sent += huart4_TxBuffer_sending;
		  if(huart4_TxBuffer_sent < huart4_TxBuffer_toSend){
			  huart4_TxInProgress = 1;
			  HAL_StatusTypeDef res2;
			  huart4_TxBuffer_sending = huart4_TxBuffer_toSend - huart4_TxBuffer_sent;
			  if(huart4_TxBuffer_sending > UART4_BLOCK_SIZE_SEND){
				  huart4_TxBuffer_sending = UART4_BLOCK_SIZE_SEND;
			  }

			  res2 = HAL_UART_Transmit_IT(&huart4, (uint8_t *)&huart4_TxBuffer[huart4_TxBuffer_sent], huart4_TxBuffer_sending);
			  if(res2 != HAL_OK){
				  huart4_TxInProgress = 0;
			  }
		  } else {
			  huart4_TxDuration = HAL_GetTick() - huart4_TxStart;
			  huart4_TxInProgress = 0;
			  huart4_TxOkCnt++;
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END task_UART4_Tx_function */
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
  if (htim->Instance == TIM6) {
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
