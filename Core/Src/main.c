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
#include <stdio.h>
#include <string.h>

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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define NUM_SENSORS 16
#define CALIB_SAMPLES 50
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define BASE_SPEED 500         // Base PWM value
#define MAX_SPEED 900          // Maximum PWM value
#define MIN_SPEED 300          // Minimum PWM value

#define KP 20.0f
#define KI 0.0f
#define KD 0.0f

volatile float integral = 0.0f;
volatile float last_error = 0.0f;
volatile int16_t current_position = 0;
volatile uint8_t line_detected = 0;

volatile uint16_t adc_buffers[2][NUM_SENSORS] = {0};
volatile uint16_t *adc_buffer_ptrs[2] = {
    adc_buffers[0],
    adc_buffers[1]
};
volatile uint8_t adc_buffer_write_ptr_index = 0;
volatile uint8_t adc_buffer_read_ptr_index = 1;
volatile uint8_t current_sensor_index = 0;
volatile uint16_t adc_dma_single_value;
const uint16_t sensor_weights[16] = {-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7,0};
volatile uint16_t sensor_thresholds[16]= {0};
const float dt = 0.002f;

volatile uint8_t is_running=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void setMuxChannel(uint8_t ch)
{
    if(ch >= 16) return;

    switch(ch)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 4:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 5:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 6:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 7:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 8:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 9:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 10:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 11:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 12:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 13:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 14:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 15:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

int16_t calculate_line_position(void) {
    uint32_t weighted_sum = 0;
    uint8_t sensors_on_line = 0;

    volatile uint16_t *sensor_data = adc_buffer_ptrs[adc_buffer_read_ptr_index];

    for(int i = 0; i < NUM_SENSORS; i++) {
        if(sensor_data[i] > sensor_thresholds[i]) {
        	weighted_sum+=sensor_weights[i];
        	sensors_on_line++;
        }
    }

    if (!sensors_on_line){
    	line_detected=0;
    }
    else{
    	line_detected=1;
    }

    return weighted_sum/sensors_on_line;
}


void set_motor_speeds(int16_t speed1, int16_t speed2) {
    uint16_t pwm1, pwm2;

    if(speed1 >= 0) {
        pwm1 = (uint16_t)MIN(speed1, 999);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else {
        pwm1 = (uint16_t)MIN(-speed1, 999);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm1);
    }

    if(speed2 >= 0) {
        pwm2 = (uint16_t)MIN(speed2, 999);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        pwm2 = (uint16_t)MIN(-speed2, 999);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm2);
    }
}

void main_pid_loop(void) {
	if (!is_running){
		return;
	}

	int16_t left_speed, right_speed;

    current_position = calculate_line_position();

    float error = (float)current_position;

    float proportional = KP * error;

    integral += error*dt;

    if(integral > 1000.0f) integral = 1000.0f;
    if(integral < -1000.0f) integral = -1000.0f;
    float integral_term = KI * integral;

    float derivative = KD * (error - last_error) / dt;
    last_error = error;

    float pid_output = proportional + integral_term + derivative;


    if(!line_detected) {
    	integral = 0.0f;
        if(current_position > 0) {
            left_speed = BASE_SPEED / 2;
            right_speed = -BASE_SPEED / 2;
        } else {
            left_speed = -BASE_SPEED / 2;
            right_speed = BASE_SPEED / 2;
        }
    } else {
        left_speed = BASE_SPEED - (int16_t)pid_output;
        right_speed = BASE_SPEED + (int16_t)pid_output;
        left_speed = MAX(MIN_SPEED, MIN(left_speed, MAX_SPEED));
        right_speed = MAX(MIN_SPEED, MIN(right_speed, MAX_SPEED));
    }

    set_motor_speeds(left_speed, right_speed);

}

void send_ir_data(void){
    char buffer[256];

    snprintf(buffer, sizeof(buffer),
             "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
			 sensor_thresholds[0], sensor_thresholds[1], sensor_thresholds[2], sensor_thresholds[3],
			 sensor_thresholds[4], sensor_thresholds[5], sensor_thresholds[6], sensor_thresholds[7],
			 sensor_thresholds[8], sensor_thresholds[9], sensor_thresholds[10], sensor_thresholds[11],
			 sensor_thresholds[12], sensor_thresholds[13], sensor_thresholds[14], sensor_thresholds[15]);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void calibrate(void){
	    int calib_data[CALIB_SAMPLES][NUM_SENSORS];
	    int i, j;

	    set_motor_speeds(700,-700);
	    for (i = 0; i < CALIB_SAMPLES; i++) {
	        for (j = 0; j < NUM_SENSORS; j++) {
	            calib_data[i][j] = adc_buffer_ptrs[adc_buffer_read_ptr_index][j];
	            HAL_Delay(10);
	        }
	        HAL_Delay(10);
	    }
	    set_motor_speeds(0,0);

	    for (j = 0; j < NUM_SENSORS; j++) {
	        int min_val = 4095;
	        int max_val = 0;

	        for (i = 0; i < CALIB_SAMPLES; i++) {
	            int value = calib_data[i][j];

	            if (value < min_val) min_val = value;
	            if (value > max_val) max_val = value;
	        }

	        sensor_thresholds[j] = (min_val + max_val) / 2;
	    }
	}

void reset_pid_variables(void) {
    integral = 0.0f;
    last_error = 0.0f;
    current_position = 0;
    line_detected = 0;
}

void init_all_variables(void) {
    integral = 0.0f;
    last_error = 0.0f;
    current_position = 0;
    line_detected = 0;

    is_running = 0;

    adc_buffer_write_ptr_index = 0;
    adc_buffer_read_ptr_index = 1;
    current_sensor_index = 0;
    adc_dma_single_value = 0;

    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < NUM_SENSORS; j++) {
            adc_buffers[i][j] = 0;
        }
    }

    for(int i = 0; i < NUM_SENSORS; i++) {
        sensor_thresholds[i] = 2047;
    }

    adc_buffer_ptrs[0] = adc_buffers[0];
    adc_buffer_ptrs[1] = adc_buffers[1];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM3)
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_dma_single_value, 1);
    }
    else if(htim->Instance == TIM4)
    {
    	main_pid_loop();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        adc_buffer_ptrs[adc_buffer_write_ptr_index][current_sensor_index] = adc_dma_single_value;

        current_sensor_index++;

        if(current_sensor_index >= NUM_SENSORS)
        {

            current_sensor_index = 0;

            adc_buffer_write_ptr_index ^= 1;
            adc_buffer_read_ptr_index ^= 1;
        }

        setMuxChannel(current_sensor_index);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	init_all_variables();
	setMuxChannel(0);

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_Delay(5000);
  HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
  calibrate();

  HAL_Delay(10000);
  send_ir_data();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 100-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S3_Pin|S2_Pin|S1_Pin|S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_Pin S2_Pin S1_Pin S0_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S2_Pin|S1_Pin|S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STBY_Pin */
  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
