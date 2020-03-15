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
#include <stdbool.h>
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
#define pulse_per_revolution 3536
#define max_enconder_count 65535
#define num_muestras 1200
int32_t count = 0;
double cuenta2 = 0;
double cuenta = 0;
int16_t count_pul = 0;
double medidas[num_muestras];
uint32_t i = 0;
char str_name_reducer[100];
char str_name[20001];
double kp = 10;
double kd = 10;
double ki = 10;
double referencia = 0;
double current_value = 0;
double diff = 0;
double last_value = 0;
double pos_i = 0.0;
double e = 0;
double e_last = 0;
double e_sum =  0;
//Flags
bool FLAG_REDUCER = false;
bool FLAG_TRANSFER = false;
bool FLAG_PROPORTIONAL_CONTROLER = false;
bool FLAG_DERIVATIVE_CONTROLER = false;
bool FLAG_INTEGRATOR_CONTROLER = false;
//enum
enum Controlador{Lineal, Derivativo, Integrador};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void move(double_t v1,double_t v2);
void obtenerdatos(double_t V);
void selec_voltage(double_t V);
void stop();
void enviarcuenta();
void controlador_lineal(double pos_i);
void controlador_derivativo(double pos_i);
void controlador_integrador(double pos_i);
void reductora();
void setref(double ref,enum Controlador controlador);
void funtion_trasfer(double_t V);
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
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//reductora();// Uncommenting this line to calculate the reducer value
	 //funtion_trasfer(12);// Uncommenting this line to calculate the function transfer

	 setref(M_PI,0); // set a first  ref to linear controler

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		controlador_lineal(pos_i);



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

		if (FLAG_PROPORTIONAL_CONTROLER == true) {

			current_value =__HAL_TIM_GET_COUNTER(&htim2);
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){


				if(current_value <= last_value){
					   diff = last_value - current_value;
				}
				else{
					diff = (max_enconder_count-current_value) + last_value;
				}

			}
			else {

				if (current_value < last_value) {
					diff = (max_enconder_count - last_value) + current_value;
				} else {
					diff = current_value - last_value;

				}

			}

			last_value = current_value;
			pos_i = pos_i + diff;
			if (FLAG_PROPORTIONAL_CONTROLER == true) {
				controlador_lineal(pos_i);
			}
			else if (FLAG_DERIVATIVE_CONTROLER == true) {
				controlador_derivativo(pos_i);

			} else if (FLAG_INTEGRATOR_CONTROLER == true) {
				controlador_integrador(pos_i);
			}


		}
		if (FLAG_REDUCER == true) {
			i += 1;
			if(i == 4000){
				cuenta = __HAL_TIM_GET_COUNTER(&htim2);
				cuenta2 = TIM2->CNT;
				HAL_TIM_Base_Stop_IT(&htim6);
				enviarcuenta();
				FLAG_REDUCER = false;
			}
		}
		if (FLAG_TRANSFER == true) {
			medidas[i] = __HAL_TIM_GET_COUNTER(&htim2);
			i += 1;
			if(i == 600){
				selec_voltage(0);

				i += 1;
			}
			else if(i == 1200){
				selec_voltage(0);

				HAL_TIM_Base_Stop_IT(&htim6);
				enviarcuenta();
				FLAG_TRANSFER = false;
			}
		}

	}
		else if(htim->Instance==TIM2){

				cuenta = 1;
				sprintf(str_name_reducer, "Cuenta = %f", cuenta);
				HAL_UART_Transmit(&huart2,(uint8_t*) str_name_reducer, strlen(str_name_reducer), HAL_MAX_DELAY);





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
	if (FLAG_TRANSFER == true) {
		for(int i = 0; i<1200; i++){
			sprintf(str_name, "%s%d\t%f\n",str_name , i,medidas[i]);
		}
		sprintf(str_name, "%s#",str_name);
		HAL_UART_Transmit(&huart2,(uint8_t*) str_name, strlen(str_name), HAL_MAX_DELAY);
	}
	if (FLAG_REDUCER == true) {
		sprintf(str_name_reducer, "Cuenta = %f\t %f", cuenta,cuenta2);
		HAL_UART_Transmit(&huart2,(uint8_t*) str_name_reducer, strlen(str_name_reducer), HAL_MAX_DELAY);
	}

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
	FLAG_REDUCER = true;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Set reference for controller
 * @param ref: reference
 * @retval None
 */
void setref(double ref, enum Controlador controlador){
	referencia = ref;
	if (controlador == Lineal) {
		FLAG_PROPORTIONAL_CONTROLER = true;
	}
	else if (controlador == Derivativo) {
		FLAG_DERIVATIVE_CONTROLER = true;

	} else if (controlador == Integrador) {
		FLAG_INTEGRATOR_CONTROLER = true;
	}

	HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief	Linear Controller
 * @retval None
 */
void controlador_lineal(double pos_i){
if (referencia <0) {
	e = referencia + (pos_i * 2 * M_PI / pulse_per_revolution);
}else {
	e = referencia - (pos_i * 2 * M_PI / pulse_per_revolution);
}

	selec_voltage((double)kp * e);

}
void controlador_derivativo(double pos_i){
if (referencia <0) {
	e_last = e;
	e = referencia + (pos_i * 2 * M_PI / pulse_per_revolution);
}else {
	e_last = e;
	e = referencia - (pos_i * 2 * M_PI / pulse_per_revolution);
}

	selec_voltage((double)(kp * e +kd * (e-e_last)));

}
void controlador_integrador(double pos_i){
if (referencia <0) {

	e = referencia + (pos_i * 2 * M_PI / pulse_per_revolution);
	e_sum= e_sum + e;
}else {

	e = referencia - (pos_i * 2 * M_PI / pulse_per_revolution);
	e_sum = e_sum + e;
}

	selec_voltage((double)(kp * e +ki * e_sum));

}
void funtion_trasfer(double_t V){
	FLAG_TRANSFER = true;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start_IT(&htim6);
	selec_voltage(V);

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
