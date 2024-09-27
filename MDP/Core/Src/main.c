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
#include "oled.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;             // Proportional gain
    float Ki;             // Integral gain
    float Kd;             // Derivative gain
    int32_t previousError;  // Previous error value
    float integral;       // Integral term
    int32_t outputMin;      // Minimum output limit
    int32_t outputMax;      // Maximum output limit
} PIDController;

// Define a struct to hold the encoder data
typedef struct {
    uint16_t position;   // Encoder position
    uint16_t speed;      // Encoder speed (counts per interval)
    uint16_t direction; // Encoder direction (1 = forward, 0 = backward)
} EncoderData;

typedef enum EncoderDat {
	POSITION = 0x0,
	SPEED,
	ALL
} EncoderDat;

typedef enum DIRECTION {
	STRAIGHT = 0x0,
	RIGHT,
	LEFT
}DIRECTION;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId showHandle;
osThreadId EncoderHandle;
osThreadId MoveStraightHandle;
osThreadId motorTaskHandle;
osMessageQId cmdQueueHandle;
osMutexId EncoderLeftMutexHandle;
osStaticMutexDef_t EncoderLeftMutexControlBlock;
osMutexId EncoderRightMutexHandle;
osStaticMutexDef_t EncoderRightMutexControlBlock;
/* USER CODE BEGIN PV */
volatile EncoderData encoderLeft = {0,0,0};
volatile EncoderData encoderRight = {0,0,0};
PIDController leftPID;
PIDController rightPID;
volatile uint32_t last_encoder_read_time = 0;
int16_t counter_prev = 0;
uint32_t speed = 0;

uint8_t aRxBuffer[20];
uint8_t circularBuffer[BUFFER_SIZE];
uint8_t ack = 0xFF;
volatile uint16_t writeIndex = 0;  // ISR writes to this index
volatile uint16_t readIndex = 0;   // Task reads from this index


uint32_t speed_set = 0;
const uint8_t T = 20;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void EncoderTask(void const * argument);
void StartStraight(void const * argument);
void MotorTask(void const * argument);

/* USER CODE BEGIN PFP */
void SetForward(bool right);
void SetDir(bool right, bool forward);
void SetSpeed(bool right, uint32_t pwmVal);
void UART_Transmit(char* message);
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, int32_t outputMin, int32_t outputMax);
uint32_t PID_Compute(PIDController *pid, int32_t setpoint, int32_t measuredValue, int32_t deltaTime);
void EncoderReset(bool right, EncoderDat EncoderData);
void SetFacing(DIRECTION direction);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Initializes the PID controller.
 * @param  pid: Pointer to the PIDController structure.
 * @param  Kp: Proportional gain.
 * @param  Ki: Integral gain.
 * @param  Kd: Derivative gain.
 * @param  outputMin: Minimum output limit.
 * @param  outputMax: Maximum output limit.
 * @retval None
 */
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, int32_t outputMin, int32_t outputMax) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previousError = 0.0;
    pid->integral = 0.0;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
}

/**
 * @brief  Computes the PID output.
 * @param  pid: Pointer to the PIDController structure.
 * @param  setpoint: Desired setpoint value.
 * @param  measuredValue: Current measured value.
 * @param  deltaTime: Time elapsed since the last update (in seconds).
 * @retval PID output value.
 */
uint32_t PID_Compute(PIDController *pid, int32_t setpoint, int32_t measuredValue, int32_t deltaTime) {
    // Calculate the error
    float error = setpoint - measuredValue;

    // Proportional term
    float Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * deltaTime;

    // Anti-windup: limit the integral value to prevent it from growing too large
    if (pid->integral > pid->outputMax) {
        pid->integral = pid->outputMax;
    } else if (pid->integral < pid->outputMin) {
        pid->integral = pid->outputMin;
    }

    float Iout = pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->previousError) / deltaTime;
    float Dout = pid->Kd * derivative;

    // Save the current error for the next derivative calculation
    pid->previousError = error;

    // Compute total output (PWM value)
    float output = Pout + Iout + Dout;

    // Saturate the output to be within limits
    if (output > pid->outputMax) {
        output = pid->outputMax;
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
    }

    return (uint32_t)output;
}


void UART_Transmit(char* message) {
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)message, strlen(message));
}

void SetDir(bool right, bool forward){
	if (right){
		if(!forward){
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		}
	}
	else{
		if(!forward){
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		}
	}
}

void SetSpeed(bool right, uint32_t pwmVal){
	if(right){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmVal);
	}
	else __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmVal);
}

void EncoderReset(bool right, EncoderDat EncoderData){
	if(right){
		switch (EncoderData){
		case POSITION:
			osMutexWait(EncoderRightMutexHandle, osWaitForever);
			encoderRight.position = 0;
			osMutexRelease(EncoderRightMutexHandle);
			break;
		case SPEED:
			osMutexWait(EncoderRightMutexHandle, osWaitForever);
			encoderRight.speed = 0;
			osMutexRelease(EncoderRightMutexHandle);
			break;
		case ALL:
		default:
			osMutexWait(EncoderRightMutexHandle, osWaitForever);
			encoderRight.speed = 0;
			encoderRight.position = 0;
			encoderRight.direction = 0;
			osMutexRelease(EncoderRightMutexHandle);
			break;
		}
	}
	else{
		switch (EncoderData){
			case POSITION:
				osMutexWait(EncoderLeftMutexHandle, osWaitForever);
				encoderLeft.position = 0;
				osMutexRelease(EncoderLeftMutexHandle);
				break;
			case SPEED:
				osMutexWait(EncoderLeftMutexHandle, osWaitForever);
				encoderLeft.speed = 0;
				osMutexRelease(EncoderLeftMutexHandle);
				break;
			case ALL:
			default:
				osMutexWait(EncoderLeftMutexHandle, osWaitForever);
				encoderLeft.speed = 0;
				encoderLeft.position = 0;
				encoderLeft.direction = 0;
				osMutexRelease(EncoderLeftMutexHandle);
				break;
		}
	}
}

void SetFacing(DIRECTION direction) {
	switch(direction){
	case STRAIGHT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,150);
		break;
	case RIGHT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,200);
		break;
	case LEFT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,100);
		break;
	default:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,150);
		break;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

//  HAL_UART_Transmit_IT(&huart3, (uint8_t *)aRxBuffer, 10);
  // Start Encoders
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Left encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Right encoder
  __HAL_TIM_SET_COUNTER(&htim2, 0); // Left encoder
  __HAL_TIM_SET_COUNTER(&htim3, 0); // Right encoder

  // Start PWM for motors
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Left motor PWM
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Right motor PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Servo motor PWM
//  SetServo(150);
  // Initialize the PID controllers for each motor
  PID_Init(&leftPID, 7, 0.01, 0.01, 0, 1500);  // Set your PID constants and output limits
  PID_Init(&rightPID, 4, 0.01, 0.01, 0, 1500); // Set your PID constants and output limits

  SetDir(0,1);
  SetDir(1,1);
//  SetFacing(RIGHT);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of EncoderLeftMutex */
  osMutexStaticDef(EncoderLeftMutex, &EncoderLeftMutexControlBlock);
  EncoderLeftMutexHandle = osMutexCreate(osMutex(EncoderLeftMutex));

  /* definition and creation of EncoderRightMutex */
  osMutexStaticDef(EncoderRightMutex, &EncoderRightMutexControlBlock);
  EncoderRightMutexHandle = osMutexCreate(osMutex(EncoderRightMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of cmdQueue */
  osMessageQDef(cmdQueue, 16, uint8_t);
  cmdQueueHandle = osMessageCreate(osMessageQ(cmdQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of show */
  osThreadDef(show, StartTask02, osPriorityIdle, 0, 128);
  showHandle = osThreadCreate(osThread(show), NULL);

  /* definition and creation of Encoder */
  osThreadDef(Encoder, EncoderTask, osPriorityIdle, 0, 128);
  EncoderHandle = osThreadCreate(osThread(Encoder), NULL);

  /* definition and creation of MoveStraight */
//  osThreadDef(MoveStraight, StartStraight, osPriorityIdle, 0, 128);
//  MoveStraightHandle = osThreadCreate(osThread(MoveStraight), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, MotorTask, osPriorityIdle, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
//  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(USART3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCL_Pin|SDA_Pin|RESET_Pin|DC_Pin
                          |LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCL_Pin SDA_Pin RESET_Pin DC_Pin
                           LED_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin|RESET_Pin|DC_Pin
                          |LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	circularBuffer[writeIndex] = aRxBuffer[0];  // Assuming a single byte command for simplicity

	writeIndex = (writeIndex + 1) % BUFFER_SIZE;

	HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
	// Re-enable UART reception for the next byte
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);

}

typedef enum STATE {
	WAIT,
	STOP,
	FORWARD,
	BACKWARD,
	TURN
} STATE;


STATE UpdateState(uint8_t command, uint8_t degree) {
  switch (command) {
    case 0x00:
    	SetSpeed(0,0);
    	SetSpeed(1,0);
      return STOP;
      break;
    case 0x01:
      SetFacing(STRAIGHT);
      osDelay(500);
      SetDir(0,1);
	    SetDir(1,1);
      SetSpeed(0,1500);
	    SetSpeed(1,1750);
      return FORWARD;
      break;
    case 0x02:
      SetFacing(STRAIGHT);
      osDelay(500);
      SetDir(0,0);
	    SetDir(1,0);
      SetSpeed(0,1500);
	    SetSpeed(1,1750);
      return BACKWARD;
      break;
    case 0x03:
      if ((degree & 0x20) >> 5 == 0x01) SetFacing(RIGHT);
      else SetFacing(LEFT);
      osDelay(500);
      if ((degree & 0x10) >> 4 == 0x01){
        SetDir(0,1);
	      SetDir(1,1);
      }
      else {
        SetDir(0,0);
	      SetDir(1,0);
      }
      SetSpeed(0,1500);
	  SetSpeed(1,1650);
      return TURN;
      break;
    default:
      return WAIT;  // Default to WAIT if an unexpected value is received
      break;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	uint8_t ch = 'A';
  for(;;)
  {
//	  UART_Transmit((uint8_t *)&ch);
//	  if(ch < 'Z') ch++;
	  HAL_GPIO_TogglePin(GPIOE,LED_Pin);
	  osDelay(5000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t hello[20];
  /* Infinite loop */
  for(;;)
  {
//  sprintf(hello, "%02X", aRxBuffer);
//  OLED_ShowString(10,5,hello);
  OLED_Refresh_Gram();
  osDelay(2000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_EncoderTask */
/**
* @brief Function implementing the Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTask */
void EncoderTask(void const * argument)
{
  /* USER CODE BEGIN EncoderTask */
  /* Infinite loop */
  char speedStr[20];
  last_encoder_read_time = HAL_GetTick();
  uint32_t position;
  uint16_t counter_current;
  for(;;)
  {
	  uint32_t current_time = HAL_GetTick();
	      if(__HAL_TIM_IS_TIM_COUNTING_DOWN (&htim2)) counter_current = __HAL_TIM_GET_COUNTER(&htim2) * -1;
	      else counter_current = __HAL_TIM_GET_COUNTER(&htim2);

	      // Calculate the elapsed time in milliseconds
	      uint32_t elapsed_time = current_time - last_encoder_read_time;
	      if (elapsed_time >= T) {  // Every 20ms (T = 0.02s)
	          // Calculate the speed in ticks per second
	          uint16_t delta_ticks = (uint16_t)(counter_current);
	          speed = (uint32_t)(delta_ticks) * 1000 / elapsed_time; // Speed in ticks per second
            osMutexWait(EncoderLeftMutexHandle, osWaitForever);
            encoderLeft.speed = speed;
            encoderLeft.position += counter_current;
            position = encoderLeft.position;
            osMutexRelease(EncoderLeftMutexHandle);
        
            sprintf(speedStr, "%.7d", position);
            OLED_ShowString(10,30,speedStr);
	          // OLED_Refresh_Gram();
	          // Save the current time and encoder count for the next calculation
	          last_encoder_read_time = current_time;
	          // counter_prev = counter_current;

	          // Optionally reset encoder counter
	          __HAL_TIM_SET_COUNTER(&htim2, 0);
	      }
    osDelay(10);
  }
  /* USER CODE END EncoderTask */
}

/* USER CODE BEGIN Header_StartStraight */
/**
* @brief Function implementing the MoveStraight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStraight */
void StartStraight(void const * argument)
{
  /* USER CODE BEGIN StartStraight */

	// Desired setpoint (target speed)
	uint8_t hello[20];
	uint16_t targetSpeed = 900.0; // Example target speed in counts per second

	// Time between PID updates
	uint32_t deltaTime = 0.02; // 10 ms
	SetDir(0,1);

	// Initial PWM values
	uint32_t leftMotorPWM = 0;
	uint32_t rightMotorPWM = 0;
	uint32_t leftSpeed = 0;
	uint32_t rightSpeed = 0;
	uint32_t lastTick;
	uint32_t currentTick;

	// Initial encoder positions
	osMutexWait(EncoderLeftMutexHandle, osWaitForever);
	leftSpeed = encoderLeft.speed;
	osMutexRelease(EncoderLeftMutexHandle);

	osMutexWait(EncoderRightMutexHandle, osWaitForever);
	rightSpeed = encoderRight.speed;
	osMutexRelease(EncoderRightMutexHandle);
	SetSpeed(0,leftMotorPWM);
	SetSpeed(0,leftMotorPWM);

  lastTick = HAL_GetTick();

  /* Infinite loop */
  	  for(;;)
  	  {
	  currentTick = HAL_GetTick();
	  deltaTime = (currentTick - lastTick); // Convert ms to seconds
	  if(deltaTime >= T){
      osMutexWait(EncoderLeftMutexHandle, osWaitForever);
      leftSpeed = encoderLeft.speed;
      osMutexRelease(EncoderLeftMutexHandle);
      
      // Compute the PID output for each motor
      leftMotorPWM = (uint32_t)PID_Compute(&leftPID, targetSpeed, leftSpeed, deltaTime);

      // Set the PWM duty cycle for each motor
      SetSpeed(0,leftMotorPWM);
      lastTick = currentTick;
	  }
	  sprintf(hello, "leftpwm:%5d", leftMotorPWM);
	  OLED_ShowString(10, 40, hello);

	  osDelay(10); // Convert deltaTime to milliseconds
  }
  /* USER CODE END StartStraight */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
void MotorTask(void const * argument)
{
  /* USER CODE BEGIN MotorTask */
  /* Infinite loop */
	SetDir(0,1);
	SetDir(1,1);
	SetSpeed(0,0);
	SetSpeed(1,0);

	uint32_t currdist;
	uint8_t degree;
	uint8_t command;
	uint32_t dist;
	uint8_t showBuf;
	uint8_t buf;
	STATE CurrentState = WAIT;
	bool done;

	EncoderReset(false, POSITION);
	SetFacing(LEFT);
	osDelay(500);
	SetFacing(RIGHT);
	osDelay(500);
	SetFacing(STRAIGHT);
	osDelay(500);

  for(;;)
  {
	  
    switch (CurrentState)
    {
    case WAIT:
    	SetFacing(STRAIGHT);
    	osDelay(500);
    	if (readIndex != writeIndex) {
    		buf = circularBuffer[readIndex];
    		readIndex = (readIndex + 1) % BUFFER_SIZE;
    		command = (buf & 0xC0) >> 6;
    		degree = buf & 0x3F;
    		EncoderReset(false, POSITION);
    		CurrentState = UpdateState(command,degree);
	    }
      break;
    case STOP:
    	SetSpeed(0,0);
	    SetSpeed(1,0);
	    CurrentState = WAIT;
	    UART_Transmit(&buf);
      break;
    case FORWARD:
    	dist = (degree) / 5 * 300;
    	osMutexWait(EncoderLeftMutexHandle, osWaitForever);
    	currdist = encoderLeft.position;
    	osMutexRelease(EncoderLeftMutexHandle);
    	if (currdist >  dist ){
    		SetSpeed(0,0);
    		SetSpeed(1,0);
    		CurrentState = WAIT;
    		HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
    	}
      break;
    case BACKWARD:
      dist = (degree) / 5 * 300;
      osMutexWait(EncoderLeftMutexHandle, osWaitForever);
      currdist = encoderLeft.position;
      osMutexRelease(EncoderLeftMutexHandle);
      if (currdist >  dist ){
        SetSpeed(0,0);
        SetSpeed(1,0);
        CurrentState = WAIT;
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
      }
      break;
    case TURN:
      dist = (degree & 0x0F);
      osMutexWait(EncoderLeftMutexHandle, osWaitForever);
      currdist = encoderLeft.position;
      osMutexRelease(EncoderLeftMutexHandle);
      if ((degree & 0x20) >> 5 == 0x01){
        if (currdist >  4270 * dist ){
		      SetSpeed(0,0);
		      SetSpeed(1,0);
          CurrentState = WAIT;
          HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
	      }
      }
      else{
        if (currdist >  1700 * dist ){
		      SetSpeed(0,0);
		      SetSpeed(1,0);
          CurrentState = WAIT;
          HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
	      }
      }
      break;
    default:
      break;
    }
//	  osMutexWait(EncoderLeftMutexHandle, osWaitForever);
//	  currdist = encoderLeft.position;
//	  osMutexRelease(EncoderLeftMutexHandle);
//	  uint32_t dist = (115 - 80) / 3.5 * 200 + 4150;
//	  if (currdist >  dist ){
//		  SetSpeed(0,0);
//		  SetSpeed(1,0);
//	  }

	  //left
//	  osMutexWait(EncoderLeftMutexHandle, osWaitForever);
//	  currdist = encoderLeft.position;
//	  osMutexRelease(EncoderLeftMutexHandle);
//	  if (currdist >  300 ){
//		  SetSpeed(0,0);
//		  SetSpeed(1,0);
//	  }

	  //left
//	   osMutexWait(EncoderLeftMutexHandle, osWaitForever);
//	   currdist = encoderLeft.position;
//	   osMutexRelease(EncoderLeftMutexHandle);
//	   if (currdist >  1800 ){
//		   SetSpeed(0,0);
//		   SetSpeed(1,0);
//		   SetFacing(STRAIGHT);
//	   }
//	   if (currdist >  4250 ){
//		   SetSpeed(0,0);
//		   SetSpeed(1,0);
//		   SetFacing(STRAIGHT);
//	   }

    osDelay(10);
  }
  /* USER CODE END MotorTask */
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
