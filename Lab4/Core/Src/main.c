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

void init_led()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER &= ~((1<<(red*2+1))|(1<<(green*2+1)) | (1<<(blue*2+1))|(1<<(orange*2+1)));
	GPIOC->MODER |= (1<<red*2)|(1<<green*2) | (1<<blue*2)|(1<<orange*2);
	GPIOC->OTYPER &= ~((1<<red) | (1<<green) | (1<<blue) | (1<<orange));
	GPIOC->OSPEEDR &= ~((1<<red*2) | (1<<green*2) | (1<<blue*2) | (1<<orange*2));
	GPIOC->PUPDR &= ~((1<<green*2) | (1<<(red*2+1)) |(1<<red*2)|(1<<(green*2+1)) |( 1<<blue*2) | (1<<(orange*2+1)) |(1<<orange*2)|(1<<(blue*2+1)) );
}
void transmit_string(char arr[])
{
	for(int i = 0; arr[i] != '\0'; i++)
	{
		while(!((USART3->ISR >> 7 )& 1));
		USART3->TDR = arr[i];
	}
}
int16_t mode = 6;
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
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	init_led();
	GPIOC->MODER &= ~((1<<(5*2))|(1<<(4*2)));
	GPIOC->MODER |= ((2<<(4*2))|(2<<(5*2)));
	GPIOC->AFR[0] |= 1 << GPIO_AFRL_AFRL4_Pos;
	GPIOC->AFR[0] |= 1 << GPIO_AFRL_AFRL5_Pos;
	
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	USART3->CR1 |= USART_CR1_UE;
	USART3 ->CR1 |= USART_CR1_TE;
	USART3 ->CR1 |= USART_CR1_RE;
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	GPIOC->ODR ^= 1 << red;
	char error[] = {'E','r','r','o','r','\n', '\0'};
	char cmd[] = {'C','M','D', ' ', '\0'};
	char on[] = {'O','N', ' ', '\0'};
	char off[] = {'O','F','F', ' ', '\0'};
	char toggle[] = {'T','O','G','G','L','E', ' ', '\0'};
	char grn[] = {'G','R','E','E','N', ' ', '\0'};
	char orng[] = {'O','R','A','N','G','E', ' ', '\0'};
	char rd[] = {'R','E','D', ' ', '\0'};
	char blu[] = {'B','L','U','E', ' ', '\0'};
  while (1)
  {
		transmit_string(cmd);
    while(!((USART3->ISR >> 5) & 1));
		char rec = USART3->RDR;
		if(0)// part 1
		{
			switch (rec)
			{
				case 'r':
					GPIOC->ODR ^= 1 << red;
					break;
				case 'g':
					GPIOC->ODR ^= 1 << green;
					break;
				case 'o':
					GPIOC->ODR ^= 1 << orange;
					break;
				case 'b':
					GPIOC->ODR ^= 1 << blue;
					break;
				default:
					
					transmit_string(error);
					break;
			}
		}
		else{ //part 2
			switch (rec)
			{
				case 'r':
					mode = red;
					transmit_string(rd);
					break;
				case 'g':
					mode = green;
					transmit_string(grn);
					break;
				case 'o':
					mode =  orange;
					transmit_string(orng);
					break;
				case 'b':
					mode =  blue;
					transmit_string(blu);
					break;
				case '0':
					GPIOC->ODR &= ~(1 << mode);
					transmit_string(off);
					break;
				case '1':
					GPIOC->ODR |= 1 << mode;
					transmit_string(on);
					break;
				case '2':
					GPIOC->ODR ^= 1 << mode;
					transmit_string(toggle);
					break;
				default:
					GPIOC->ODR &= ~(1 << blue);
					GPIOC->ODR &= ~(1 << orange);
					GPIOC->ODR &= ~(1 << green);
					GPIOC->ODR |= (1 << red);
					transmit_string(error);
					break;
			}
		}
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
