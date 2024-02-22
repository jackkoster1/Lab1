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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int16_t red = 6; 
int16_t blue = 7; 
int16_t orange = 8;
int16_t green = 9;
void TIM2_IRQHandler(void)
{
	GPIOC->ODR ^= (1<<green) ;
	GPIOC->ODR ^= (1<<orange) ;
	TIM2->SR &=~(1);
}


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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER &= ~((1<<(green*2))|(1<<(red*2)) | (1<<(orange*2))|(1<<(blue*2)));
	GPIOC->MODER |= (1<<(green*2))|(2<<(red*2)) | (1<<(orange*2))|(2<<(blue*2));
	GPIOC->OTYPER &= ~((1<<green) | (1<<red) | (1<<orange) | (1<<blue));
	GPIOC->OSPEEDR &= ~((1<<green*2) | (1<<red*2) | (1<<orange*2) | (1<<blue*2));
	GPIOC->PUPDR &= ~((1<<red*2) | (1<<(green*2+1)) |(1<<green*2)|(1<<(red*2+1)) |( 1<<orange*2) | (1<<(blue*2+1)) |(1<<blue*2)|(1<<(orange*2+1)) );
  /* Configure the system clock */
  SystemClock_Config();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2->PSC = 999;
	TIM2->ARR = 2000;
	TIM2->DIER |= 1;
	TIM2->CR1 |= 1;
	

	//3.2 *******************************************
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->PSC = 99;
	TIM3->ARR = 100;
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1);
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC2S_1);
	TIM3->CCMR1 |= ((TIM_CCMR1_OC1M_0) | (TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1M_2 );
	TIM3->CCMR1 |= ((TIM_CCMR1_OC2M_1) | TIM_CCMR1_OC2M_2 );
	TIM3->CCMR1 |= (TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE );
	TIM3 ->CR1 |= TIM_CR1_CEN;
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	
	TIM3->CCR1 = 50;
	TIM3->CCR2 = 50;
	
	//3.2 end ****************************************
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	GPIOC->ODR |= (1<<green) ;
	//GPIOC->ODR |= (1<<red) ;
	//GPIOC->ODR |= (1<<blue) ;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
