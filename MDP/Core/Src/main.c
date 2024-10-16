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
#include "ICM20948.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _PIDController_t{
    float Kp;             // Proportional gain
    float Ki;             // Integral gain
    float Kd;             // Derivative gain
    int32_t previousError;  // Previous error value
    float integral;       // Integral term
    int32_t outputMin;      // Minimum output limit
    int32_t outputMax;      // Maximum output limit
} PIDController;

// Define a struct to hold the encoder data
typedef struct _EncoderData_t{
    uint16_t position;   // Encoder position
    uint16_t speed;      // Encoder speed (counts per interval)
    uint16_t direction; // Encoder direction (1 = forward, 0 = backward)
} EncoderData;

typedef enum _EncoderDat_t {
	POSITION = 0x0,
	SPEED,
	ALL
} EncoderDat;

typedef enum _DIRECTION_t {
	STRAIGHT = 0x0,
	RIGHT,
	LEFT,
  HALF_RIGHT,
  HALF_LEFT
}DIRECTION;

typedef struct _IMURead_t{
	float pitch_CF;
	float roll_CF;
} IMURead_t;

//in mm
typedef struct _IRRead_t{
	float left;
	float right;
}IRRead_t;

typedef struct _UltraRead_t{
	uint16_t frontDist;
} UltraRead_t;

typedef enum STATE {
	WAIT,
	STOP,
	GOTOB1,
	B1LEFT,
	B1RIGHT,
	B2LEFT,
	B2RIGHT,
	FORWARD,
	BACKWARD,
	FWLEFT,
	BWLEFT,
	FWRIGHT,
	BWRIGHT
} STATE;

typedef enum MOVE_STATE_t {
	INIT,
  PROGRESS,
  DONE
} MOVE_STATE;

typedef enum _CMD_STATE_T{
	ERR,
	PROG,
	FIN
} CMD_STATE_T;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId showHandle;
osThreadId EncoderHandle;
osThreadId MoveStraightHandle;
osThreadId motorTaskHandle;
osThreadId icmTaskHandle;
osThreadId infraTaskHandle;
osThreadId ultrasonicTaskHandle;
osMutexId EncoderLeftMutexHandle;
osStaticMutexDef_t EncoderLeftMutexControlBlock;
osMutexId EncoderRightMutexHandle;
osStaticMutexDef_t EncoderRightMutexControlBlock;
osMutexId IMUReadMutexHandle;
osStaticMutexDef_t IMUReadMutexControlBlock;
osMutexId IRMutexHandle;
osStaticMutexDef_t IRMutexControlBlock;
osMutexId UltrasonicMutexHandle;
osStaticMutexDef_t UltrasonicMutexControlBlock;
/* USER CODE BEGIN PV */
volatile EncoderData encoderLeft = {0,0,0};
volatile EncoderData encoderRight = {0,0,0};
volatile IMURead_t imuRead = {0.0, 0.0};
volatile IRRead_t IRRead = {0,0};
volatile UltraRead_t UltraRead = {0};
volatile uint32_t LEFT_IR_RES_BUFFER[1];
volatile uint32_t RIGHT_IR_RES_BUFFER[1];

PIDController leftPID;
PIDController rightPID;
int16_t counter_prev = 0;
uint32_t speed = 0;

uint8_t aRxBuffer[20];
uint8_t circularBuffer[BUFFER_SIZE] = {0};
uint8_t ack = 0xFF;
volatile uint16_t writeIndex = 0;  // ISR writes to this index
volatile uint16_t readIndex = 0;   // Task reads from this index


uint32_t speed_set = 0;
const uint8_t T = 20;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint16_t Distance  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void EncoderTask(void const * argument);
void StartStraight(void const * argument);
void MotorTask(void const * argument);
void ICMUpdate(void const * argument);
void IRTask(void const * argument);
void UltrasonicTask(void const * argument);

/* USER CODE BEGIN PFP */
void SetForward(bool right);
void SetDir(bool right, bool forward);
void SetSpeed(bool right, uint32_t pwmVal);
void UART_Transmit(char* message);
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, int32_t outputMin, int32_t outputMax);
uint32_t PID_Compute(PIDController *pid, int32_t setpoint, int32_t measuredValue, float deltaTime);
void EncoderReset(bool right, EncoderDat EncoderData);
void SetFacing(DIRECTION direction);
CMD_STATE_T Move_Turn(bool forward, DIRECTION direction, float degree);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
    {
        if (Is_First_Captured==0) // if the first value is not captured
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
            Is_First_Captured = 1;  // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
 
        else if (Is_First_Captured==1)   // if the first is already captured
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
 
            if (IC_Val2 > IC_Val1)
            {
                Difference = IC_Val2-IC_Val1;
            }
 
            else if (IC_Val1 > IC_Val2)
            {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }
 
            UltraRead.frontDist = Difference * (0.034/2);
            Is_First_Captured = 0; // set it back to false
 
            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
        }
    }
}
 
void HCSR04_Read (void)
{
    HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    delay_us(10);  // wait for 10 us
    HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
 
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

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
uint32_t PID_Compute(PIDController *pid, int32_t setpoint, int32_t measuredValue, float deltaTime) {
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
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,SERVO_STRAIGHT);
		break;
	case RIGHT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,SERVO_RIGHT);
		break;
	case LEFT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,SERVO_LEFT);
		break;
	case HALF_RIGHT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 180);
		break;
	case HALF_LEFT:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 125);
		break;
	default:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,150);
		break;
	}
}

double compute_polynomial(double x) {
    // Coefficients of the polynomial
    double b5 = -207.6802923;
    double b4 = 1644.417156;
    double b3 = -5142.954895;
    double b2 = 8014.646796;
    double b1 = -6396.634059;
    double b0 = 2328.775415;

    // Using Horner's method for efficient computation
    return ((((b5 * x + b4) * x + b3) * x + b2) * x + b1) * x + b0;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  UART_Transmit("Hello");

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
  HAL_ADC_Start_DMA(&hadc1, LEFT_IR_RES_BUFFER, 1);
  HAL_ADC_Start_DMA(&hadc2, RIGHT_IR_RES_BUFFER, 1);
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
//  SetServo(150);
  // Initialize the PID controllers for each motor
  PID_Init(&leftPID, 1, 0.95, 0.002, 0, 2000);  // Set your PID constants and output limits
  PID_Init(&rightPID, 1, 0.01, 0.01, 0, 1500); // Set your PID constants and output limits

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

  /* definition and creation of IMUReadMutex */
  osMutexStaticDef(IMUReadMutex, &IMUReadMutexControlBlock);
  IMUReadMutexHandle = osMutexCreate(osMutex(IMUReadMutex));

  /* definition and creation of IRMutex */
  osMutexStaticDef(IRMutex, &IRMutexControlBlock);
  IRMutexHandle = osMutexCreate(osMutex(IRMutex));

  /* definition and creation of UltrasonicMutex */
  osMutexStaticDef(UltrasonicMutex, &UltrasonicMutexControlBlock);
  UltrasonicMutexHandle = osMutexCreate(osMutex(UltrasonicMutex));

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of show */
  osThreadDef(show, StartTask02, osPriorityLow, 0, 128);
  showHandle = osThreadCreate(osThread(show), NULL);

  /* definition and creation of Encoder */
  osThreadDef(Encoder, EncoderTask, osPriorityHigh, 0, 128);
  EncoderHandle = osThreadCreate(osThread(Encoder), NULL);

  /* definition and creation of MoveStraight */
//  osThreadDef(MoveStraight, StartStraight, osPriorityIdle, 0, 256);
//  MoveStraightHandle = osThreadCreate(osThread(MoveStraight), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, MotorTask, osPriorityHigh, 0, 2048);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of icmTask */
//  osThreadDef(icmTask, ICMUpdate, osPriorityNormal, 0, 1024);
//  icmTaskHandle = osThreadCreate(osThread(icmTask), NULL);

  /* definition and creation of infraTask */
 osThreadDef(infraTask, IRTask, osPriorityIdle, 0, 1024);
 infraTaskHandle = osThreadCreate(osThread(infraTask), NULL);

  /* definition and creation of ultrasonicTask */
  osThreadDef(ultrasonicTask, UltrasonicTask, osPriorityIdle, 0, 128);
  ultrasonicTaskHandle = osThreadCreate(osThread(ultrasonicTask), NULL);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCL_Pin|SDA_Pin|RESET_Pin|DC_Pin
                          |LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : IR_LEFT_Pin IR_RIGHT_Pin */
  GPIO_InitStruct.Pin = IR_LEFT_Pin|IR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ULTRA_TRIG_Pin */
  GPIO_InitStruct.Pin = ULTRA_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ULTRA_TRIG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	circularBuffer[writeIndex] = aRxBuffer[0];  // Assuming a single byte command for simplicity

	writeIndex = (writeIndex + 1) % BUFFER_SIZE;

//	HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
	// Re-enable UART reception for the next byte
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);

}

// Ramp state structure to hold the state between function calls
typedef struct {
    uint16_t pwmLeft;
    uint16_t pwmRight;
    uint16_t targetLeft;
    uint16_t targetRight;
    uint16_t rampStep;
    uint32_t stepDelay;      // Delay between each step in milliseconds
    uint32_t lastTick;
    bool rampingComplete;
} MotorRampState;

// Assume that the SetSpeed function is already defined
// void SetSpeed(bool right, uint16_t pwm);

// Non-blocking motor ramping function
void RampMotors(MotorRampState* state) {
    // If ramping is already complete, return immediately
    if (state->rampingComplete) {
        return;
    }

    // Check if the delay time between steps has passed
    if (HAL_GetTick() - state->lastTick >= state->stepDelay) {
        // Update the last tick time
        state->lastTick = HAL_GetTick();

        // Increase left motor PWM
        if (state->pwmLeft < state->targetLeft) {
            state->pwmLeft += state->rampStep;
            if (state->pwmLeft > state->targetLeft) {
                state->pwmLeft = state->targetLeft;  // Clamp to the target
            }
        }

        // Increase right motor PWM proportionally
        if (state->pwmRight < state->targetRight) {
            state->pwmRight = state->pwmLeft * state->targetRight / state->targetLeft;
            if (state->pwmRight > state->targetRight) {
                state->pwmRight = state->targetRight;  // Clamp to the target
            }
        }

        // Set the speed for both motors
        SetSpeed(false, state->pwmLeft);  // Left motor
        SetSpeed(true, state->pwmRight);  // Right motor

        // Check if both motors have reached their target speeds
        if (state->pwmLeft >= state->targetLeft && state->pwmRight >= state->targetRight) {
            state->rampingComplete = true;
        }
    }
}

// Function to initialize the MotorRampState with proper delay calculation
void InitializeRampState(MotorRampState* state, uint16_t targetLeftPWM, uint16_t targetRightPWM, uint16_t rampStep, uint32_t maxDelay) {
    // Initialize state
    state->pwmLeft = 0;  // Assuming motors start from 0 PWM
    state->pwmRight = 0; // Same assumption for right motor
    state->targetLeft = targetLeftPWM;
    state->targetRight = targetRightPWM;
    state->rampStep = rampStep;
    state->rampingComplete = false;
    state->lastTick = HAL_GetTick();  // Initialize with current time

    // Calculate the number of steps required to reach the target speed
    uint16_t maxSteps = (targetLeftPWM + rampStep - 1) / rampStep;  // Ceil of targetLeftPWM / rampStep

    // Calculate step delay based on maxDelay and number of steps
    state->stepDelay = maxDelay / maxSteps;  // Delay per step in milliseconds
}

MotorRampState rampState;
uint32_t Block2Far = 0;

CMD_STATE_T GoToB1(){
  uint16_t frontDist;
  RampMotors(&rampState);
//  SetSpeed(0, 1000);
//  SetSpeed(1, 1200);
  osMutexWait(UltrasonicMutexHandle, HAL_MAX_DELAY);
  frontDist = UltraRead.frontDist;
  osMutexRelease(UltrasonicMutexHandle);
  // EncoderReset(0, POSITION);
  // Check if the ramping process is complete
  if (rampState.rampingComplete || frontDist < 40) {
    if (frontDist < 30){
      SetSpeed(1,0);
      SetSpeed(0,0);
      // osMutexWait(EncoderLeftMutexHandle, HAL_MAX_DELAY);
      // Block2Far += encoderLeft.position;
      // osMutexRelease(EncoderLeftMutexHandle);
      return FIN;
    }
    if(frontDist < 37){
      SetSpeed(1, 1800);
      SetSpeed(0, 1500);
    }
    if(frontDist < 34){
      SetSpeed(1, 600);
      SetSpeed(0, 500);
    }
  }
	return PROG;
}

typedef enum _B1LEFT_STATE_t{
  B1LEFT_LEFT,
  B1LEFT_RIGHT,
  B1LEFT_STRAIGHTEN,
  B1LEFT_ADJUST
} B1LEFT_STATE_t;

B1LEFT_STATE_t B1LEFT_STATE = B1LEFT_LEFT;
bool measure = false;
uint32_t toAdjust;
CMD_STATE_T B1Left(){
  uint32_t frontDist;
  switch (B1LEFT_STATE)
  {
  case B1LEFT_LEFT:
    if(Move_Turn(true, HALF_LEFT, 1.05f) == FIN) B1LEFT_STATE = B1LEFT_RIGHT;
    break;
  case B1LEFT_RIGHT:
    if(Move_Turn(true, RIGHT, 0.9f) == FIN) B1LEFT_STATE = B1LEFT_STRAIGHTEN;
    break;
  case B1LEFT_STRAIGHTEN:
    // if(Move_Turn(true, HALF_LEFT, 0.93f) == FIN) {
    if(Move_Turn(true, LEFT, 0.48f) == FIN) {
      B1LEFT_STATE = B1LEFT_ADJUST;
      SetFacing(STRAIGHT);
      EncoderReset(0, POSITION);
      osDelay(100);
    }
    break;
  case B1LEFT_ADJUST:
    osMutexWait(UltrasonicMutexHandle, HAL_MAX_DELAY);
    frontDist = UltraRead.frontDist;
    osMutexRelease(UltrasonicMutexHandle);
    if(frontDist <= 26) {
      SetDir(1,0);
      SetDir(0,0);
      SetFacing(STRAIGHT);
      SetSpeed(1,1200);
      SetSpeed(0,1000);
    }
    if(frontDist >= 30) {
      measure = true;
      SetDir(1,1);
      SetDir(0,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,2400);
      SetSpeed(0,2000);
    }
    if(frontDist < 30 && frontDist > 26){
      SetSpeed(1,0);
      SetSpeed(0,0);
      osMutexWait(EncoderLeftMutexHandle, HAL_MAX_DELAY);
      toAdjust = (uint32_t) ((float) (encoderLeft.position) * 5 / 260);
      osMutexRelease(EncoderLeftMutexHandle);
      Block2Far += 70;
      if(measure) Block2Far += toAdjust;
      else Block2Far -= toAdjust;
      B1LEFT_STATE = B1LEFT_LEFT;
      measure = false;
      return FIN;
    }
    break;
  default:
    return ERR;
    break;
  }
  return PROG;
}

typedef enum _B1RIGHT_STATE_t{
  B1RIGHT_LEFT,
  B1RIGHT_RIGHT,
  B1RIGHT_STRAIGHTEN,
  B1RIGHT_ADJUST
} B1RIGHT_STATE_t;

B1RIGHT_STATE_t B1RIGHT_STATE = B1RIGHT_LEFT;

CMD_STATE_T B1Right(){
  uint32_t frontDist;
  switch (B1RIGHT_STATE)
  {
  case B1RIGHT_LEFT:
    if(Move_Turn(true, HALF_RIGHT, 0.71f) == FIN) B1RIGHT_STATE = B1RIGHT_RIGHT;
    break;
  case B1RIGHT_RIGHT:
    if(Move_Turn(true, LEFT, 1.03f) == FIN) B1RIGHT_STATE = B1RIGHT_STRAIGHTEN;
    break;
  case B1RIGHT_STRAIGHTEN:
    // if(Move_Turn(true, HALF_LEFT, 0.93f) == FIN) {
    if(Move_Turn(true, RIGHT, 0.54f) == FIN) {
    	B1RIGHT_STATE = B1RIGHT_ADJUST;
      osDelay(100);
    }
    break;
  case B1RIGHT_ADJUST:
    osMutexWait(UltrasonicMutexHandle, HAL_MAX_DELAY);
    frontDist = UltraRead.frontDist;
    osMutexRelease(UltrasonicMutexHandle);
    
    if(frontDist <= 25) {
      SetDir(1,0);
      SetDir(0,0);
      SetFacing(STRAIGHT);
      SetSpeed(1,2400);
      SetSpeed(0,2000);
    }
    if(frontDist >= 27) {
      SetDir(1,1);
      SetDir(0,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,2400);
      SetSpeed(0,2000);
    }
    if(frontDist < 27 && frontDist > 25){
      SetSpeed(1,0);
      SetSpeed(0,0);
      B1RIGHT_STATE = B1RIGHT_LEFT;
      return FIN;
    }
    break;
  default:
    return ERR;
    break;
  }
  return PROG;
}

typedef enum _B2RIGHT_STATE_t{
 B2RIGHT_RIGHT,
 B2RIGHT_FIND_BLOCK_RIGHT,
 B2RIGHT_FIND_GAP_RIGHT,
 B2RIGHT_TURN_FACE_BACK,
 B2RIGHT_BACK_FACE_BACK,
 B2RIGHT_TURN_FACE_LEFT,
 B2RIGHT_RESET_TO_BACK,
 B2RIGHT_FIND_BLOCK,
 B2RIGHT_FIND_GAP_LEFT,
 B2RIGHT_TURN_TO_FRONT,
 B2RIGHT_GO_BACK_STRAIGHT,
 B2RIGHT_FIRST_TURN_BACK_CENTER,
 B2RIGHT_GOTO_BLOCK_BACK_CENTER,
 B2RIGHT_BACKWARDS_AFTER_GOTO,
 B2RIGHT_SECOND_TURN_BACK_CENTER,
 B2RIGHT_HUIJIALO
} B2RIGHT_STATE_t;

B2RIGHT_STATE_t B2RIGHT_STATE = B2RIGHT_RIGHT;
uint32_t Block2Dist = 0;


CMD_STATE_T B2Right(){
  uint32_t leftDist;
  uint32_t fwDist;
  uint32_t dist;
  uint32_t frontDist;
  switch (B2RIGHT_STATE)
  {
  case B2RIGHT_RIGHT:
    if(Move_Turn(true, RIGHT, 1) == FIN) {
      B2RIGHT_STATE = B2RIGHT_FIND_BLOCK_RIGHT;
      SetDir(0,0);
      SetDir(1,0);
      SetFacing(STRAIGHT);
      osDelay(100);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
    }
    break;
  case B2RIGHT_FIND_BLOCK_RIGHT:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist < 200) {
      B2RIGHT_STATE = B2RIGHT_FIND_GAP_RIGHT;
      SetDir(0,1);
      SetDir(1,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
    }
    break;
  case B2RIGHT_FIND_GAP_RIGHT:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist > 250) {
      B2RIGHT_STATE = B2RIGHT_TURN_FACE_BACK;
      osDelay(150);
      SetSpeed(1,0);
      SetSpeed(0,0);
    }
    break;
  case B2RIGHT_TURN_FACE_BACK:
   // if(Move_Turn(true, HALF_LEFT, 0.93f) == FIN) {
   if(Move_Turn(true, LEFT, 1) == FIN) {
   	B2RIGHT_STATE = B2RIGHT_BACK_FACE_BACK;
    EncoderReset(0, POSITION);
    SetDir(0,0);
    SetDir(1,0);
    SetFacing(STRAIGHT);
    osDelay(100);
    SetSpeed(1,2400);
    SetSpeed(0,2000);
   }
   break;
  case B2RIGHT_BACK_FACE_BACK:
    dist = (uint32_t)( 7 * 260 / 4.8);
    osMutexWait(EncoderLeftMutexHandle, HAL_MAX_DELAY);
    fwDist = encoderLeft.position;
    osMutexRelease(EncoderLeftMutexHandle);
    if(fwDist > dist){
      B2RIGHT_STATE = B2RIGHT_TURN_FACE_LEFT;
      SetSpeed(1,0);
      SetSpeed(0,0);
    }
    break;
  case B2RIGHT_TURN_FACE_LEFT:
   // if(Move_Turn(true, HALF_LEFT, 0.93f) == FIN) {
   if(Move_Turn(true, LEFT, 1) == FIN) {
   	B2RIGHT_STATE = B2RIGHT_RESET_TO_BACK;
    EncoderReset(0, POSITION);
    SetDir(0,0);
    SetDir(1,0);
    SetFacing(STRAIGHT);
    osDelay(100);
    SetSpeed(1,1800);
    SetSpeed(0,1500);
   }
   break;
  case B2RIGHT_RESET_TO_BACK:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist > 300) {
   	  B2RIGHT_STATE = B2RIGHT_FIND_BLOCK;
      SetDir(0,1);
      SetDir(1,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
    }
   break;
  case B2RIGHT_FIND_BLOCK:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist < 200) {
      B2RIGHT_STATE = B2RIGHT_FIND_GAP_LEFT;
      SetDir(0,1);
      SetDir(1,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
    }
   break;
 case B2RIGHT_FIND_GAP_LEFT:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist > 300) {
      B2RIGHT_STATE = B2RIGHT_TURN_TO_FRONT;
      SetSpeed(1,0);
      SetSpeed(0,0);
    }
   break;
  case B2RIGHT_TURN_TO_FRONT:
    if(Move_Turn(true, LEFT, 1.0f) == FIN) {
   	B2RIGHT_STATE = B2RIGHT_GO_BACK_STRAIGHT;
    EncoderReset(0, POSITION);
    SetDir(0,1);
    SetDir(1,1);
    SetFacing(STRAIGHT);
    SetSpeed(1,1800);
    SetSpeed(0,1500);
   }
   break;
  case B2RIGHT_GO_BACK_STRAIGHT:
    dist = (uint32_t)((float)(Block2Far)* 260 / 4.8);
    osMutexWait(EncoderLeftMutexHandle, HAL_MAX_DELAY);
    fwDist = encoderLeft.position;
    osMutexRelease(EncoderLeftMutexHandle);
    if(fwDist > dist){
      SetSpeed(0,0);
      SetSpeed(1,0);
      B2RIGHT_STATE = B2RIGHT_FIRST_TURN_BACK_CENTER;
      osDelay(10000);
    }
   break;
  case B2RIGHT_FIRST_TURN_BACK_CENTER:
    if(Move_Turn(1, LEFT, 1.0f) == FIN){
      osDelay(10000);
      SetSpeed(0,0);
      SetSpeed(1,0);
      B2RIGHT_STATE = B2RIGHT_GOTO_BLOCK_BACK_CENTER;
      SetDir(0,1);
      SetDir(1,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
    }
  break;
  case B2RIGHT_GOTO_BLOCK_BACK_CENTER:
    osMutexWait(IRMutexHandle, HAL_MAX_DELAY);
    leftDist = IRRead.left;
    osMutexRelease(IRMutexHandle);
    if(leftDist < 300) {
      B2RIGHT_STATE = B2RIGHT_BACKWARDS_AFTER_GOTO;
      SetSpeed(0,0);
      SetSpeed(1,0);
      SetDir(0,0);
      SetDir(1,0);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
      EncoderReset(0, POSITION);
    }
  break;
  case B2RIGHT_BACKWARDS_AFTER_GOTO:
    dist = (uint32_t)((float)(10)* 260 / 4.8);
    osMutexWait(EncoderLeftMutexHandle, HAL_MAX_DELAY);
    fwDist = encoderLeft.position;
    osMutexRelease(EncoderLeftMutexHandle);
    if(fwDist > dist){
      SetSpeed(0,0);
      SetSpeed(1,0);
      B2RIGHT_STATE = B2RIGHT_SECOND_TURN_BACK_CENTER;
    }
  break;
  case B2RIGHT_SECOND_TURN_BACK_CENTER:
    if(Move_Turn(1, RIGHT, 1.0f) == FIN){
      SetSpeed(0,0);
      SetSpeed(1,0);
      SetDir(0,1);
      SetDir(1,1);
      SetFacing(STRAIGHT);
      SetSpeed(1,1800);
      SetSpeed(0,1500);
      B2RIGHT_STATE = B2RIGHT_HUIJIALO;
    }
  break;
  case B2RIGHT_HUIJIALO:
    osMutexWait(UltrasonicMutexHandle, HAL_MAX_DELAY);
    frontDist = UltraRead.frontDist;
    osMutexRelease(UltrasonicMutexHandle);
    if (frontDist < 10) {
      SetSpeed(1,0);
      SetSpeed(0,0);
      B2RIGHT_STATE = B2RIGHT_RIGHT;
      return FIN;
    }
    break;
  default:
    return ERR;
    break;
  }
 return PROG;
}

CMD_STATE_T Task_Two(STATE task){
  uint16_t leftDist;
  uint16_t rightDist;
	switch (task)
	{
	  case GOTOB1:
	    SetDir(1,1);
      SetDir(0,1);
      SetFacing(STRAIGHT);
      if(GoToB1() == FIN) return FIN;
      break;
	  case B1LEFT:
      if(B1Left() == FIN) return FIN;
	    return PROG;
	    break;
	  case B1RIGHT:
      if(B1Right() == FIN ) return FIN;
	    return PROG;
	    break;
	  case B2LEFT:
	    return PROG;
	    break;
	  case B2RIGHT:
	    if(B2Right() == FIN ) return FIN;
	    return PROG;
	    break;
	  default:
	    return ERR;
	    break;
	}
  return PROG;
}

MOVE_STATE move_straight_state = INIT;
MOVE_STATE Move_Straight(bool forward, uint8_t degree){
  uint32_t currdist;
  uint32_t dist;

  switch(move_straight_state) {
    case INIT:
      SetDir(1, forward);
      SetDir(0, forward);
      SetFacing(STRAIGHT);
      osDelay(200);
      SetSpeed(0,1000);
	    SetSpeed(1,1200);
      EncoderReset(false, POSITION);
      move_straight_state = PROGRESS;
      return INIT;
      break;
    case PROGRESS:
      dist = (uint32_t)((float)(degree)* 260 / 4.8);
    	osMutexWait(EncoderLeftMutexHandle, osWaitForever);
    	currdist = encoderLeft.position;
    	osMutexRelease(EncoderLeftMutexHandle);
    	if (currdist >  dist ){
    		SetSpeed(0,0);
    		    	SetSpeed(1,0);
    		move_straight_state = DONE;
    	}
      return PROGRESS;
      break;
    case DONE:
//      move_straight_state = INIT;

      return DONE;
      break;
    default:
      move_straight_state = INIT;
      return INIT;
      break;
  }
}
MOVE_STATE move_turn_state = INIT;
CMD_STATE_T Move_Turn(bool forward, DIRECTION direction, float degree){
  uint32_t currdist;
  float dist;

  switch(move_turn_state) {
    case INIT:
      SetDir(1, forward);
      SetDir(0, forward);
      SetFacing(direction);
      osDelay(200);
      SetSpeed(0,1300);
	    SetSpeed(1,1300);
      EncoderReset(false, POSITION);
      move_turn_state = PROGRESS;
      break;
    case PROGRESS:
      dist = degree;
      osMutexWait(EncoderLeftMutexHandle, osWaitForever);
      currdist = encoderLeft.position;
      osMutexRelease(EncoderLeftMutexHandle);
    	if (direction == RIGHT){
        if ((float)currdist >  RIGHT_TURN * dist ){
		      move_turn_state = DONE;
		      SetSpeed(0,0);
		          	SetSpeed(1,0);
	      }
      }
      else if (direction == LEFT){
        if(forward){
        if ((float)currdist >  LEFT_TURN * dist ){
		      move_turn_state = DONE;
		      SetSpeed(0,0);
		      SetSpeed(1,0);
	      }
        }
        else{
        if ((float)currdist >  (LEFT_TURN + 5) * dist ){
		      move_turn_state = DONE;
		      SetSpeed(0,0);
		      SetSpeed(1,0);
	      }
        }
      }
      if (direction == HALF_LEFT){
        if ((float)currdist >  LEFT_TURN * dist ){
		      move_turn_state = DONE;
		      SetSpeed(0,0);
		          	SetSpeed(1,0);
	      }
      }
      if (direction == HALF_RIGHT){
        if ((float)currdist >  RIGHT_TURN * dist ){
		      move_turn_state = DONE;
		      SetSpeed(0,0);
		          	SetSpeed(1,0);
	      }
      }
      break;
    case DONE:
      move_turn_state = INIT;
      return FIN;
      break;
    default:
      move_turn_state = INIT;
      return ERR;
      break;
  }
  return PROG;
}

STATE UpdateState(uint8_t command, uint8_t degree) {
  switch (command) {
    case 0x00:
      uint8_t task2State = (degree & 0x07) ;
      switch (task2State)
      {
        case 0x00:
          return STOP;
          break;
        case 0x01:
          return GOTOB1;
          break;
        case 0x02:
          return B1LEFT;
          break;
        case 0x03:
          return B1RIGHT;
          break;
        case 0x04:
          return B2LEFT;
          break;
        case 0x05:
          return B2RIGHT;
          break;
        default:
          break;
      }
      break;
    case 0x01:
      return FORWARD;
      break;
    case 0x02:
      return BACKWARD;
      break;
    case 0x03:
      if ((degree & 0x20) >> 5 == 0x01) {
        if ((degree & 0x10) >> 4 == 0x01) return FWRIGHT;
        else return BWRIGHT;
      }
      else{
        if ((degree & 0x10) >> 4 == 0x01) return FWLEFT;
        else return BWLEFT;
      }
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
	  osDelay(1000);
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
	uint8_t hello[20] = "hello";
  /* Infinite loop */
  for(;;)
  {

 OLED_ShowString(10,5,hello);
  OLED_Refresh_Gram();
  osDelay(100);
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
  uint32_t last_encoder_read_time = HAL_GetTick();
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
	          speed = (uint32_t)(delta_ticks * 1000 / elapsed_time); // Speed in ticks per second
            osMutexWait(EncoderLeftMutexHandle, osWaitForever);
            encoderLeft.speed = speed;
            encoderLeft.position += counter_current;
            position = encoderLeft.position;
            osMutexRelease(EncoderLeftMutexHandle);
        
            sprintf(speedStr, "%.7d", speed);
            OLED_ShowString(10,30,speedStr);
//	           OLED_Refresh_Gram();
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
	uint16_t targetSpeed = 600.0; // Example target speed in counts per second

	// Time between PID updates
	TickType_t deltaTime;  // Time difference between ticks
	const TickType_t T_in_ticks = pdMS_TO_TICKS(T);  // Convert time threshold `T` from ms to ticks

//	uint32_t deltaTime = 0.02; // 10 ms
	SetDir(0,1);

	// Initial PWM values
	uint32_t leftMotorPWM = 0;
	uint32_t rightMotorPWM = 0;
	uint32_t leftSpeed = 0;
	uint32_t rightSpeed = 0;
	TickType_t lastTick;
	TickType_t currentTick;

	// Initial encoder positions
	osMutexWait(EncoderLeftMutexHandle, osWaitForever);
	leftSpeed = encoderLeft.speed;
	osMutexRelease(EncoderLeftMutexHandle);

	osMutexWait(EncoderRightMutexHandle, osWaitForever);
	rightSpeed = encoderRight.speed;
	osMutexRelease(EncoderRightMutexHandle);
	SetSpeed(0,leftMotorPWM);
	SetSpeed(0,leftMotorPWM);

	lastTick = xTaskGetTickCount();

  /* Infinite loop */
  	  for(;;)
  	  {
	  currentTick = xTaskGetTickCount();
	  deltaTime = (currentTick - lastTick); // Convert ms to seconds
	  if(deltaTime >= T_in_ticks){
      osMutexWait(EncoderLeftMutexHandle, osWaitForever);
      leftSpeed = encoderLeft.speed;
      osMutexRelease(EncoderLeftMutexHandle);
      
      // Compute the PID output for each motor
      leftMotorPWM = (uint32_t)PID_Compute(&leftPID, targetSpeed, leftSpeed, T / 1000.0f);

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
//  SetSpeed(1, 2000);
//  SetSpeed(0, 2000);
	uint16_t targetLeftPWM = 2000;
    uint16_t targetRightPWM = 2400;
    uint16_t rampStep = 50;
    uint32_t maxDelay = 100;
  for(;;)
  {
	  
    switch (CurrentState)
    {
    case WAIT:
    	SetFacing(STRAIGHT);
    	osDelay(200);
    	if (readIndex != writeIndex) {
    		buf = circularBuffer[readIndex];
    		readIndex = (readIndex + 1) % BUFFER_SIZE;
    		command = (buf & 0xC0) >> 6;
    		degree = buf & 0x3F;
    		EncoderReset(false, POSITION);
        move_straight_state = INIT;
        move_turn_state = INIT;
        InitializeRampState(&rampState, targetLeftPWM, targetRightPWM, rampStep, maxDelay);
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
      if (Move_Straight(true, degree) == DONE) {
    	  HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
      }
      break;
    case BACKWARD:
      if (Move_Straight(false, degree) == DONE) {
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
      }
      break;
    case FWLEFT:
//    	if (Move_Straight(true, 0x09) == DONE)
    	degree &= 0x0F;
      if (Move_Turn(true, LEFT, degree) == FIN) {
//    	  degree = 0x08;
    	  CurrentState = WAIT;
    	  HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//    	  move_straight_state = INIT;
      }
//        if (Move_Straight(true, 0x0A) == DONE) {
//          HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1,                                            );
//          CurrentState = WAIT;
//        }
      break;
    case FWRIGHT:
    	degree &= 0x0F;
//      if (Move_Straight(true, 0x08) == DONE)
        if (Move_Turn(true, RIGHT, degree) == FIN) {
//        	degree = 0x05;
        	    	  CurrentState = WAIT;
        	    	  HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//        	    	  move_straight_state = INIT;
        }
      break;
    case BWLEFT:
    	degree &= 0x0F;
//    	if (Move_Straight(false, 0x09) == DONE)
      if (Move_Turn(false, LEFT, degree) == FIN){
//    	  degree = 0x0C;
    	  CurrentState = WAIT;
    	  HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//    	  move_straight_state = INIT;
      }
//        if (Move_Straight(false, 0x0A) == DONE){
//          HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//          CurrentState = WAIT;
//        }
      break;
    case BWRIGHT:
    	degree &= 0x0F;
//      if (Move_Straight(false, 0x05) == DONE)
        if (Move_Turn(false, RIGHT, degree) == FIN){
//        	degree = 0x02;
        	CurrentState = WAIT;
        	HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//        	move_straight_state = INIT;
        }
//        	if (Move_Straight(false, 0x0F) == DONE) {
//          HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
//          CurrentState = WAIT;
//        }
      break;
    case GOTOB1:
      if(Task_Two(GOTOB1) == FIN) {
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
      }
      break;
    case B1LEFT:
      if(Task_Two(B1LEFT) == FIN){
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
      }
      break;
    case B1RIGHT:
      if(Task_Two(B1RIGHT) == FIN){
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
      }
      break;
    case B2RIGHT:
      if(Task_Two(B2RIGHT) == FIN){
        HAL_UART_Transmit(&huart3, (uint8_t *)&ack, 1, HAL_MAX_DELAY);
        CurrentState = WAIT;
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

    osDelay(5);
  }
  /* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_ICMUpdate */
/**
* @brief Function implementing the icmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ICMUpdate */
void ICMUpdate(void const * argument)
{
  /* USER CODE BEGIN ICMUpdate */
  /* Infinite loop */
	ICM20948 imu;
	bool init = false;
	uint8_t* status;


	int i;
	float ax,ay, az, gx,gy,gz;
	float pitch, roll, pitch_gyro = 0, roll_gyro = 0, pitch_CF = 0, roll_CF = 0;

	uint32_t millisOld = HAL_GetTick();
	uint32_t millisNow, dt;
  for(;;)
  {
	  if(!init){
		  status = IMU_Initialise( &imu, &hi2c1, &huart3);
		  if (status == 0)init = true;
	  }

	  uint8_t sbuf[10][10] = {0};

	  	  IMU_AccelRead(&imu);
	  	  IMU_GyroRead(&imu);

	  	  millisNow = HAL_GetTick();
	  	  dt = millisNow - millisOld;
	  	  millisOld = millisNow;

	  	  ax = imu.acc[0];
	  	  ay = imu.acc[1];
	  	  az = imu.acc[2];
	  	  roll = atan(ay/az) / M_PI * 180;
	  	  pitch = atan(ax/az) / M_PI * 180;
	  	  pitch = -pitch;

	  	  gx = imu.gyro[0];
	  	  gy = imu.gyro[1];
	  	  gz = imu.gyro[2];

	  	  roll_gyro = roll_gyro + (imu.gyro[0]*dt*0.001);
	  	  pitch_gyro = pitch_gyro + (imu.gyro[1]*dt*0.001);

	  	  roll_CF = 0.05*roll +0.95*(roll_CF + imu.gyro[0]*dt*0.001);
	  	  pitch_CF = 0.05*pitch + 0.95*(pitch_CF + imu.gyro[1]*dt*0.001);

	  	  osMutexWait(IMUReadMutexHandle, osWaitForever);
	  	  imuRead.pitch_CF = pitch_CF;
	  	  imuRead.roll_CF = roll_CF;
	  	  osMutexRelease(IMUReadMutexHandle);

	  	snprintf(sbuf[0], sizeof(sbuf[0]), "%5.2f , ", roll);
	  	snprintf(sbuf[1], sizeof(sbuf[1]), "%5.2f , ", pitch);


	  	snprintf(sbuf[2], sizeof(sbuf[2]), "%7.2f , ", roll_gyro);
	  	snprintf(sbuf[3], sizeof(sbuf[3]), "%7.2f , ", pitch_gyro);

	  	snprintf(sbuf[4], sizeof(sbuf[4]), "%7.2f , ", roll_CF);
	  	snprintf(sbuf[5], sizeof(sbuf[5]), "%7.2f , ", pitch_CF);
        // OLED_ShowString(10,20, (uint8_t *)&(sbuf[4]));

//	  	  HAL_UART_Transmit(&huart3, sbuf[0], 8, HAL_MAX_DELAY);
//	  	  HAL_UART_Transmit(&huart3, sbuf[1], 8, HAL_MAX_DELAY);
//	  	  HAL_UART_Transmit(&huart3, sbuf[2], 10, HAL_MAX_DELAY);
//	  	  HAL_UART_Transmit(&huart3, sbuf[3], 10, HAL_MAX_DELAY);
//	  	  HAL_UART_Transmit(&huart3, sbuf[4], 10, HAL_MAX_DELAY);
//	  	  HAL_UART_Transmit(&huart3, sbuf[5], 10, HAL_MAX_DELAY);
//
//	  	  HAL_UART_Transmit(&huart3, "\r\n", 10, HAL_MAX_DELAY);
	  	char txBuffer[64];  // Large enough to hold all formatted data
	  	snprintf(txBuffer, sizeof(txBuffer),
	  	         "%5.2f , %5.2f , %7.2f , %7.2f , %7.2f , %7.2f\r\n",
	  	         roll, pitch, roll_gyro, pitch_gyro, roll_CF, pitch_CF);

	  	// Transmit the entire buffer in one call
//	  	HAL_UART_Transmit(&huart3, (uint8_t*)txBuffer, strlen(txBuffer), 100);

    osDelay(10);
  }
  /* USER CODE END ICMUpdate */
}

/* USER CODE BEGIN Header_IRTask */
/**
* @brief Function implementing the infraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRTask */
void IRTask(void const * argument)
{
  /* USER CODE BEGIN IRTask */
  /* Infinite loop */
  osDelay(500);
  uint16_t left, right;
  double bufLeft, bufRight;
  double movingAverageLeft = 0.0, movingAverageRight = 0.0;
  double movingBufferLeft[10] = {0}, movingBufferRight[10] = {0};
  int indexLeft = 0, indexRight = 0;
  double sumLeft = 0.0, sumRight = 0.0;
  int countLeft = 0, countRight = 0;
  int transmitCounter = 0;

  for(;;)
  {
    // Check DMA flags and retrieve left and right sensor data
    if (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4))
    {
        left = LEFT_IR_RES_BUFFER[0];
        __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4);
    }
    if (__HAL_DMA_GET_FLAG(&hdma_adc2, DMA_FLAG_TCIF2_6))
    {
        right = RIGHT_IR_RES_BUFFER[0];
        __HAL_DMA_CLEAR_FLAG(&hdma_adc2, DMA_FLAG_TCIF2_6);
    }

    // Mask lower 12 bits (ADC resolution)
    left &= 0x0FFF;
    right &= 0x0FFF;

    // Convert to voltage (assuming 3.3V reference)
    bufLeft = (left * 3.3f / 4095.0f);
    bufRight = (right * 3.3f / 4095.0f);

    // Update moving average for left sensor
    sumLeft -= movingBufferLeft[indexLeft];  // Subtract the oldest value
    movingBufferLeft[indexLeft] = bufLeft;   // Store the new value
    sumLeft += bufLeft;  // Add the new value to the sum

    // Update the index and wrap around if necessary
    indexLeft = (indexLeft + 1) % 10;

    // Calculate the average for left
    countLeft = (countLeft < 10) ? countLeft + 1 : 10;
    movingAverageLeft = sumLeft / countLeft;

    // Update moving average for right sensor
    sumRight -= movingBufferRight[indexRight];  // Subtract the oldest value
    movingBufferRight[indexRight] = bufRight;   // Store the new value
    sumRight += bufRight;  // Add the new value to the sum

    // Update the index and wrap around if necessary
    indexRight = (indexRight + 1) % 10;

    // Calculate the average for right
    countRight = (countRight < 10) ? countRight + 1 : 10;
    movingAverageRight = sumRight / countRight;

    // Process values using your polynomial computation (if needed)
    double processedLeft = compute_polynomial(movingAverageLeft);
    double processedRight = compute_polynomial(movingAverageRight);

    // Store the processed values in a shared structure
    osMutexWait(IRMutexHandle, osWaitForever);
    IRRead.left = processedLeft;
    IRRead.right = processedRight;
    osMutexRelease(IRMutexHandle);

//     // Transmit the right sensor data via UART
//     char txBuffer[64];  // Large enough to hold all formatted data
//     snprintf(txBuffer, sizeof(txBuffer),
//              "%8.2f\r\n",
//              bufRight);
// //    HAL_UART_Transmit(&huart3, (uint8_t*)txBuffer, strlen(txBuffer), 100);
    osDelay(2);
  }
  /* USER CODE END IRTask */
}

/* USER CODE BEGIN Header_UltrasonicTask */
/**
* @brief Function implementing the ultrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicTask */
void UltrasonicTask(void const * argument)
{
  /* USER CODE BEGIN UltrasonicTask */
  /* Infinite loop */
  for(;;)
  {

	HCSR04_Read();
    osDelay(50);
  }
  /* USER CODE END UltrasonicTask */
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
