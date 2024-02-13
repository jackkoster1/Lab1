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
int16_t red = 6; 
int16_t blue = 7; 
int16_t orange = 8;
int16_t green = 9;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int c1 = 0;
void EXTI0_1_IRQHandler()
{
	
	EXTI->PR |= 1;//resets flag for interupt
	GPIOC->ODR ^= (1<<green) ;
	GPIOC->ODR ^= (1<<orange) ;
	
	while(c1 <= 1500000)
	{c1++;}
	
	c1=0;
	GPIOC->ODR ^= (1<<green) ;
	GPIOC->ODR ^= (1<<orange) ;
	
	
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	
	
	int16_t c1 = green;
	int16_t c2 = red;
	int16_t c3 = blue;
	int16_t c4 = orange;
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	GPIOC->MODER &= ~((1<<(c2*2+1))|(1<<(c1*2+1)) | (1<<(c3*2+1))|(1<<(c4*2+1)));
	GPIOC->MODER |= (1<<c2*2)|(1<<c1*2) | (1<<c3*2)|(1<<c4*2);
	GPIOC->OTYPER &= ~((1<<c2) | (1<<c1) | (1<<c3) | (1<<c4));
	GPIOC->OSPEEDR &= ~((1<<c2*2) | (1<<c1*2) | (1<<c3*2) | (1<<c4*2));
	GPIOC->PUPDR &= ~((1<<c1*2) | (1<<(c2*2+1)) |(1<<c2*2)|(1<<(c1*2+1)) |( 1<<c3*2) | (1<<(c4*2+1)) |(1<<c4*2)|(1<<(c3*2+1)) );
  /* USER CODE BEGIN 1 */

	
	GPIOA->MODER &= ~(1<<1 | 1);
	GPIOA->OSPEEDR &= ~(1);
	GPIOA->PUPDR &= ~(1);
	GPIOA->PUPDR |= (1<<1);
	
	SYSCFG->CFGR1 &= ~7; //111
	EXTI->IMR |= 1;
	EXTI->RTSR |= 1;
	
	SYSCFG->EXTICR[0] &= ~7;
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn,1);
	NVIC_SetPriority(SysTick_IRQn,2);
	NVIC_SetPriority(EXTI0_1_IRQn,3);
	
  GPIOC->ODR |= (1<<c1) ;
	while (1) {
		HAL_Delay(600); // Delay 200ms
		// Toggle the output state of both colors
		
		GPIOC->ODR ^= (1<<c2) ;
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
