/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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

/* USER CODE BEGIN PV */
#define pulse_per_revolution 3600
#define num_muestras 7000
int32_t count = 0;
int cuenta = 0;
int16_t count_pul = 0;
int medidas[num_muestras];
uint32_t i = 0;
char str_name[10000];
uint32_t pos_i = 0;
int cte_prop = 5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void move(double_t v1,double_t v2);
void obtenerdatos(double_t V);
void selec_voltage(double_t V);
void stop();
void enviarcuenta();
double controlador(double pos);
void reductora();
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Init(&htim6);
  reductora();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}
/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM6) {
		medidas[i] = __HAL_TIM_GET_COUNTER(&htim2);
		i += 1;
		if(i == 600){
			selec_voltage(0);
			medidas[i] = __HAL_TIM_GET_COUNTER(&htim2);
			i += 1;
		}
		else if(i == 1200){
			selec_voltage(0);
			medidas[i] = __HAL_TIM_GET_COUNTER(&htim2);
			HAL_TIM_Base_Stop_IT(&htim6);
			enviarcuenta();
		}
	}
	else {
	}
}

/**
  * @brief  Start PWM pins with specific duties cycles
  * @param  v1: Duty cycle of first PWM .
  * @param  v2:  Duty cycle of second PWM.
  * @retval None
  */
void move(double_t v1,double_t v2){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	v1 = round(v1);
	v2 = round(v2);

	if(v1>999){
		v1 = 999;
	}
	else if(v1<0){
		v1 = 0;
	}

	if(v2<0){
		v2 = 0;
	}
	else if(v2>999){
		v2 = 999;
	}

	if(v1 != 0 && v2 !=0){
		v1 = 0;
		v2 = 0;
	}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,v1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,v2);
}
/**
  * @brief  Set both PWM to 0 and lock the enable A
  * @retval None
  */
void stop(){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

/**
  * @brief  Estimate the duty cycle of the PWM, according to the mean voltage selected and call move function with this values.
  * @param  V: the average voltage of the PWM.
  * @retval None
  */
void selec_voltage (double_t V){
	double tension_pwm;
	tension_pwm = (V/12)*999;
	if (V>0) {
		move(tension_pwm,0);
	}else {
		tension_pwm = abs(tension_pwm);
		move(0,tension_pwm);
	}

}
/**
  * @brief  Send measures via ST link USB for UART
  * @retval None
  */
void enviarcuenta(){
	for(int i = 0; i<1200; i++){
		sprintf(str_name, "%s%d\t%d\n",str_name , i, medidas[i]);
	}
	sprintf(str_name, "%s#",str_name);
	HAL_UART_Transmit(&huart2,(uint8_t*) str_name, strlen(str_name), HAL_MAX_DELAY);
}
/**
  * @brief  Test start with certain voltage and sampling counter
  * @param  V: the average voltage of the PWM.
  * @retval None
  */
void obtenerdatos(double_t V){
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start_IT(&htim6);
	count_pul = 0;
	selec_voltage(V);
}
/**
  * @brief Reducer measurement experimentally
  * @retval None
  */
void reductora(){
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start_IT(&htim6);
}
/**
  * @brief	Linear Controller
  * @param  pos: The position we want to move the engine in radians
  * @retval None
  */
void controlador(uint32_t pos){
	//MAL TENEMOS QUE COMENTARLO
	pos_i = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t pos_pulsos = pos*pulse_per_revolution/2*M_PI;
	uint32_t pos_ideal;
	pos_ideal = pos_i + pos_pulsos;
	if(pos_ideal > 65535){
		pos_ideal = pos_ideal - 65535;
		pos_i = pos_i - 65535;
	}
	else if(pos_ideal < 0){
		pos_ideal = pos_ideal + 65535;
		pos_i = pos_i + 65535;
	}
	if (pos_i != pos_ideal){
		//porque 5???
		selec_voltage(cte_prop*(pos_ideal - pos_i));
		controlador(pos);
	}
	else{
		selec_voltage(0);
	}

	/*


	 */
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
