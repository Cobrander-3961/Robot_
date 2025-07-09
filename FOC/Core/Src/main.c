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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_pca9685.h"
#include <stdio.h>
#include <foc.h>
#include <stdio.h>
#include "motor_runtime_param.h"
#include "foc.h"

#include "arm_math.h"
#include "MagneticSensor.h" 
#include "filter.h"
int fputc(int c, FILE *stream)
{
  uint8_t ch[] = {(uint8_t)c};
  HAL_UART_Transmit(&huart2, ch, 1, HAL_MAX_DELAY);
  return c;
}
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define DLY_TIM_Handle  (&htim6)
void delay_us(uint32_t nus)
{
	__HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
	__HAL_TIM_ENABLE(DLY_TIM_Handle);
	while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
	{
	}
	__HAL_TIM_DISABLE(DLY_TIM_Handle);
}


// // ���ã�Delay_us_Asm(170); // 170 cycles �� 1us��170MHz�£�
void delay_1ms(uint32_t Delay)
{
    delay_us(Delay * 1000); // 170MHz�£�1us��Լ��Ҫ170������
}
uint8_t receivedata[8];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// printf("suc\n");
	if(huart == &huart3){
		if(receivedata[0] == 0xAA &&receivedata[1] == 0xFF && receivedata[2] == 0x00&& receivedata[3] == 0x20)
			{
                  motor_control_context.speed = receivedata[4]; 
		}
	
	}
//	receivedata_1 = {0,0,0,0,0,0,0,0};
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receivedata,sizeof(receivedata));	

	// __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
  {

  }
}
extern motor_control_context_t motor_control_context;

float _motor_i_d = 0;
float _motor_i_q = 0.9;
  extern float B_Gain_c;

//速度环PID
extern float P_Gain_s;      // 比例增益
 extern   float I_Gain_s;      // 积分增益
  extern  float B_Gain_s;      // 抗饱和增益

extern  arm_pid_instance_f32 pid_speed;
extern float encoder_angle; 
extern float motor_speed; 
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
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//      HAL_ADCEx_Calibration_Start(&hadc1);
//   HAL_ADCEx_Calibration_Start(&hadc2);
  MagneticSensor_Init();
  set_motor_pid(
      2, 0, 7,
      0.3, 0.001, 0,
      1.0, 0.001, 0,
      1.5, 0.001, 0);

  extern uint8_t mt6701_rx_data[3];
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  HAL_SPI_TransmitReceive_DMA(&hspi1, mt6701_rx_data, mt6701_rx_data, 3);
  HAL_Delay(10); // 延时一会，让角度变量被赋值，不然默认角度是0
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim4);
//  HAL_ADCEx_Calibration_Start(&hadc1);
//  HAL_ADCEx_Calibration_Start(&hadc2);
//  HAL_ADCEx_InjectedStart_IT(&hadc1);
//  HAL_ADCEx_InjectedStart(&hadc2);
  set_pwm_duty(0.5, 0, 0);              // d轴强拖，形成SVPWM模型中的基础矢量1，即对应转子零度位置
  HAL_Delay(400);                       // 保持一会
  rotor_zero_angle = encoder_angle;
  set_pwm_duty(0, 0, 0);                // 松开电机
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//   motor_control_context.torque_norm_d = 0;
//   motor_control_context.torque_norm_q = 0.9;
//  motor_control_context.type = control_type_torque;
  HAL_Delay(1000);
//  motor_control_context.position = deg2rad(90);
//  motor_control_context.type = control_type_position;
  // 速度模式     
  motor_control_context.speed = -1000;       //每秒转30弧度
  motor_control_context.type = control_type_speed;
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // MT6701上电
  /* USER CODE END 2 */

   /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
 
    printf("Encoder Angle: %.2f, Motor Logic Angle: %.2f, Motor Speed: %.2f\n", 
           encoder_angle, motor_logic_angle, motor_speed);
           float u_1 = ADC_REFERENCE_VOLT * ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) / ((1 << ADC_BITS) - 1) - 0.5);
    float u_2 = ADC_REFERENCE_VOLT * ((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) / ((1 << ADC_BITS) - 1) - 0.5);
    float i_1 = u_1 / R_SHUNT / OP_GAIN;
    float i_2 = u_2 / R_SHUNT / OP_GAIN;
    motor_i_u = i_1;
    motor_i_v = i_2;

    float i_alpha = 0;
    float i_beta = 0;
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
    float sin_value = arm_sin_f32(rotor_logic_angle);
    float cos_value = arm_cos_f32(rotor_logic_angle);
    float _motor_i_d = 0;
    float _motor_i_q = 0;
    arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);
    float filter_alpha_i_d = 0.1;
    float filter_alpha_i_q = 0.9;
    motor_i_d = low_pass_filter(_motor_i_d, motor_i_d, filter_alpha_i_d);
    motor_i_q = low_pass_filter(_motor_i_q, motor_i_q, filter_alpha_i_q);

    switch (motor_control_context.type)
    {
    case control_type_position:
      lib_position_control(motor_control_context.position);
      break;
    case control_type_speed:
      lib_speed_control(motor_control_context.speed);
      break;
    case control_type_torque:
      lib_torque_control(motor_control_context.torque_norm_d, motor_control_context.torque_norm_q);
      break;
    case control_type_speed_torque:
      lib_speed_torque_control(motor_control_context.speed, motor_control_context.max_torque_norm);
      break;
    case control_type_position_speed_torque:
      lib_position_speed_torque_control(motor_control_context.position, motor_control_context.max_speed, motor_control_context.max_torque_norm);
      break;
    default:
      break;
    }
   for (size_t i = 0; i < 360; i += 20)
   {
     HAL_Delay(2);
     foc_forward(0, 0.9, deg2rad(i));

   }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
