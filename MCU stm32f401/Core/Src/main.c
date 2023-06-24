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
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CURRENT_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 2
#define SPEED_CONTROL_MODE 3
#define IMPEDANCE_CONTROL_MODE 4
#define FORCE_CONTROL_MODE 5
#define MICRO_TEST_MODE 10


#define TOTAL_PACKET_SIZE 8

#define LOADCELL_DATA_ID 1

#define SET_MOTOR_POSITION_SP_ID 21
#define SET_MOTOR_SPEED_SP_ID 22
#define SET_MOTOR_CURRENT_SP_ID 23
#define SET_MOTOR_EMERGENCY_STOP_ID 24
#define EMERGENCY_STOP_MOTOR_ID 27

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
uint8_t UART1_rxBuffer[TOTAL_PACKET_SIZE] , c , buff_1[TOTAL_PACKET_SIZE];
uint8_t UART2_rxBuffer[TOTAL_PACKET_SIZE], buff_2[TOTAL_PACKET_SIZE];
uint8_t UART6_rxBuffer[TOTAL_PACKET_SIZE], buff_6[TOTAL_PACKET_SIZE];

int is_uart1_single_byte_recieved = 1 , is_uart2_single_byte_recieved = 1 , is_uart3_single_byte_recieved = 1;
uint32_t adc_buff[4];
double fsr1,fsr2,fsr3,theta_ankle;

uint8_t MOTOR_ACTIVE_FLAG = 1;
uint8_t control_mode = IMPEDANCE_CONTROL_MODE; //IMPEDANCE_CONTROL_MODE;

double Kp_impedance = 10 , Kd_impedance = 0.0 , Ki_impedance = 0 ;
double alpha_impedance = 0.8 , alpha_force_control = 0.1;
double sp_force_filtered = 0 , control_signal_filtered = 0 ;
double sp_theta_ankle = 0;

uint32_t dt , test_num;
double theta_ankle , theta_ankle_last , theta_dot , theta_dot_filtered , alpha_speed = 0.5;

double err_pos,pid_out,err_pos_dot, sum_err_pos, ff, control_signal , last_err_pos;
double Ki_pos = 10.0,Kd_pos = 0,Kp_pos = 100.0;
double sp_force , err_force,loadcell_force , last_err_force, sum_err_force,Kp_force = 3.0 ,Kd_force=0 , Ki_force =0.1;
double force , alpha_force = 0.85 , ff_impedance = 0;

double freq = 3.14;
int32_t load_int;
int32_t i, ii , i2 , n_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void send_packet_to_motor(uint8_t command , int32_t val)
{
	uint8_t buf[TOTAL_PACKET_SIZE] = {0xAA , 0xAA , command , (val>>24 & 0xFF) , (val>>16 & 0xFF) , (val>>8 & 0xFF) , (val & 0xFF) , 0xBB};
	//HAL_UART_Transmit_DMA(&huart6 , buf , TOTAL_PACKET_SIZE);
	HAL_UART_Transmit(&huart2 , buf , TOTAL_PACKET_SIZE , 10);
}

void send_packet_to_PC(uint8_t command , int32_t val)
{
	uint8_t buf[TOTAL_PACKET_SIZE] = {command , (val>>24 & 0xFF) , (val>>16 & 0xFF) , (val>>8 & 0xFF) , (val & 0xFF)};
	HAL_UART_Transmit_DMA(&huart6 , buf , TOTAL_PACKET_SIZE);
}

void send_packet_to_Arduino(uint8_t command , int32_t val)
{
	uint8_t buf[TOTAL_PACKET_SIZE] = {command , (val>>24 & 0xFF) , (val>>16 & 0xFF) , (val>>8 & 0xFF) , (val & 0xFF)};
	HAL_UART_Transmit_DMA(&huart1 , buf , TOTAL_PACKET_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	fsr1 = adc_buff[0];
	fsr2 = adc_buff[1];
	fsr3 = adc_buff[2];
	
	double theta = 0.0008725*(adc_buff[3] - 3000.0);
	theta_ankle = 0.8 * theta_ankle + 0.2 * theta;		// -0.42 rad < theta_ankle < 0.48 rad
}

void push_to_arr(uint8_t* arr , uint8_t  len , uint8_t c)
{
		/*
			arr[0] = arr[1];
			arr[1] = arr[2];
			arr[2] = arr[3];
			arr[3] = arr[4];
			arr[4] = arr[5];
			arr[5] = arr[6];
			arr[6] = arr[7];
			arr[7] = c;
		*/
	for(int i = 0; i<len-1 ;i++)
	{
		arr[i] = arr[i+1];
	}
	arr[len-1] = c;
}
uint8_t check_packet(uint8_t* buf)
{
	if (buf[0]==0xAA && buf[1]==0xAA && buf[7]==0xBB)
		return 1;
	else
		return 0;
}
void read_packet_data(uint8_t* buf)
{
	uint8_t command = buf[2] ;
	int32_t value = (int32_t)((buf[3]<<24) | (buf[4]<<16) | (buf[5]<<8) | (buf[6]));
	if (command == LOADCELL_DATA_ID )
	{
		load_int = value;
		force = ((double)load_int)/1000.0;
		loadcell_force = alpha_force*loadcell_force + (1.0 - alpha_force)*force;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) { 		// Load cell
		if (is_uart1_single_byte_recieved)
		{
			push_to_arr(buff_1 , TOTAL_PACKET_SIZE , UART1_rxBuffer[0]);

			if (check_packet(buff_1))	// correct packet recieved
			{	
				read_packet_data(buff_1);
				is_uart1_single_byte_recieved = 0;
				HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart1_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
		} 
		else {
			if (check_packet(UART1_rxBuffer))	// correct packet recieved
			{	
				i++;
				read_packet_data(UART1_rxBuffer);
				HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart1_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
		}
		
	}else if (huart->Instance == USART6) {		// PC
		
		if (is_uart2_single_byte_recieved)
		{
			push_to_arr(buff_2 , TOTAL_PACKET_SIZE , UART2_rxBuffer[0]);

			if (check_packet(buff_2))	// correct packet recieved
			{	
				read_packet_data(buff_2);
				is_uart2_single_byte_recieved = 0;
				HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart2_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 1);
		} 
		else {
			if (check_packet(UART2_rxBuffer))	// correct packet recieved
			{	
				i2++;
				read_packet_data(UART2_rxBuffer);
				HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart2_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 1);
		}
		
	}else if (huart->Instance == USART2) {		// Motor
		
	}
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MicroSwitch_Down_Pin) 
    {
			// send_packet_to_motor(EMERGENCY_STOP_MOTOR_ID , 0);
    } else if(GPIO_Pin == MicroSwitch_Up_Pin) 
    {
			// send_packet_to_motor(EMERGENCY_STOP_MOTOR_ID , 0);
    }
}

double saturation_f(double a , double min_ , double max_)
{
	if (a > max_)
		return max_;
	else if (a < min_)
		return  min_;
	else 
		return a;
}
int32_t saturation_i(int32_t a , int32_t min_ , int32_t max_)
{
	if (a > max_)
		return max_;
	else if (a < min_)
		return  min_;
	else 
		return a;
}
double abs_f(double a)
{
		if (a<0)
			return -a;
		return a;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim4);
	HAL_ADC_Start_DMA(&hadc1 , adc_buff , 4);
	HAL_UART_Receive_DMA(&huart1 , UART1_rxBuffer , 1);
	HAL_UART_Receive_DMA(&huart6 , UART2_rxBuffer , 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
		double t = ((double)HAL_GetTick())/1000.0; 
		//double p = 6.2*sin(0.1*t);
		//send_packet_to_motor(SET_MOTOR_POSITION_SP_ID , (int32_t)(p*1000.0) ); 
		
		
		// loop time
		dt = __HAL_TIM_GET_COUNTER(&htim4) ;
		__HAL_TIM_SetCounter(&htim4 , 0);
		
		// update variable
		
		theta_dot = 1000000.0*(theta_ankle - theta_ankle_last)/((double)dt);
		theta_ankle_last = theta_ankle;
		theta_dot_filtered = alpha_speed*theta_dot_filtered + (1.0 - alpha_speed)*theta_dot;
		
		if (HAL_GPIO_ReadPin(MicroSwitch_Down_GPIO_Port , MicroSwitch_Down_Pin) && HAL_GPIO_ReadPin(MicroSwitch_Up_GPIO_Port , MicroSwitch_Up_Pin))
		  MOTOR_ACTIVE_FLAG = 1;
		else 
			MOTOR_ACTIVE_FLAG = 0;

		if (control_mode == FORCE_CONTROL_MODE || control_mode == IMPEDANCE_CONTROL_MODE)
		{
			
			if (control_mode == IMPEDANCE_CONTROL_MODE)
			{
				err_pos = sp_theta_ankle - theta_ankle;
				err_pos_dot =  1000000.0*(err_pos - last_err_pos)/((double)dt);
				last_err_pos = err_pos;
				sum_err_pos = saturation_f(sum_err_pos + err_pos*(dt/1000000.0) , -1 , 1);
				pid_out = Kp_impedance*err_pos + Kd_impedance*err_pos_dot + Ki_impedance*sum_err_pos ;
				ff = ff_impedance;
				control_signal = pid_out + ff;
				sp_force_filtered = alpha_impedance*sp_force_filtered  +  (1.0 - alpha_impedance)*control_signal;
				sp_force = sp_force_filtered;
			}
			
				// Control force
				{
					err_force = sp_force - loadcell_force;
					err_pos_dot =  1000000.0*(err_force - last_err_force)/((double)dt);
					last_err_force = err_force;
					sum_err_force = saturation_f(sum_err_force + err_force*(dt/1000000.0) , -1 , 1);
					
					if (abs_f(err_force) < 0.08)
						err_force = 0.0;
										
					pid_out = Kp_force*err_force + Kd_force*err_pos_dot + Ki_force*sum_err_force ;
					ff = 0;
					control_signal = pid_out + ff;
					control_signal_filtered = alpha_force_control*control_signal_filtered + (1.0 - alpha_force_control)*control_signal;
					
					if (MOTOR_ACTIVE_FLAG)
						send_packet_to_motor(SET_MOTOR_CURRENT_SP_ID , -1000.0*control_signal_filtered);
					else 
						send_packet_to_motor(SET_MOTOR_CURRENT_SP_ID , 1000.0*0);
				}
			
		}
		else if (control_mode == POSITION_CONTROL_MODE)
		{
			sp_theta_ankle = 0.15*sin(freq*t);
			err_pos = sp_theta_ankle - theta_ankle;
			err_pos_dot =  1000000.0*(err_pos - last_err_pos)/((double)dt);
			last_err_pos = err_pos;
			sum_err_pos = 0;
			pid_out = Kp_pos*err_pos + Kd_pos*err_pos_dot + Ki_pos*sum_err_pos ;
			ff = 0;
			control_signal = pid_out + ff;
			if (MOTOR_ACTIVE_FLAG)
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 1000.0*control_signal);
			else 
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 1000.0*0);
		}
		else if (control_mode == MICRO_TEST_MODE)
		{
			char str_buffer[15];
			
			test_num++;
			n_byte = sprintf(str_buffer, "%.2f \n\r",(float)(test_num%2000)+0.1);
      HAL_UART_Transmit(&huart6, (uint8_t *)str_buffer, n_byte, 2);
			
			//test
			if (!HAL_GPIO_ReadPin(MicroSwitch_Down_GPIO_Port , MicroSwitch_Down_Pin))
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , -4*1000.0);
			else if (!HAL_GPIO_ReadPin(MicroSwitch_Up_GPIO_Port , MicroSwitch_Up_Pin))
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 4*1000.0);
			else 
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 0);
				
		}
		else {
			if (!HAL_GPIO_ReadPin(MicroSwitch_Down_GPIO_Port , MicroSwitch_Down_Pin))
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , -4*1000.0);
			else if (!HAL_GPIO_ReadPin(MicroSwitch_Up_GPIO_Port , MicroSwitch_Up_Pin))
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 4*1000.0);
			else 
				send_packet_to_motor(SET_MOTOR_SPEED_SP_ID , 0);
		}
		
		
		// send data
		{
			char str_buffer[40];
			// theta_ankle , sp_theta_ankle , loadcell_force , sp_force
			n_byte = sprintf(str_buffer, "%.4f , %.4f ,%.3f , %.3f \n\r",theta_ankle , sp_theta_ankle , loadcell_force , sp_force);
      HAL_UART_Transmit(&huart6, (uint8_t *)str_buffer, n_byte, 5);
		}
			
		
				
	 HAL_Delay(8);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MicroSwitch_Up_Pin */
  GPIO_InitStruct.Pin = MicroSwitch_Up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MicroSwitch_Up_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MicroSwitch_Down_Pin */
  GPIO_InitStruct.Pin = MicroSwitch_Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MicroSwitch_Down_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
