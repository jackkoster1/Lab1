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


void write(char mess)
{
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 ); //sets slave address, sets bytes to transmit and restart, inplicit write
	I2C2 -> CR2 |= 1<< 13 ;
	while(!((I2C2->ISR >> 1) & 1)) //wait till clear
			if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
			
	I2C2->TXDR = mess;
		
	while(!((I2C2->ISR >> 6) & 1)) //transfer complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field

	
}
int read(char mess)
{
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 | 1<<10 );//sets slave address, sets bytes to transmit and set read and restart
	I2C2 -> CR2 |= 1<< 13 ;
	while(!((I2C2->ISR >> 2) & 1))//Receive Register Not Empty
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	while(!((I2C2->ISR >> 6) & 1))//Transfer Complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	
	if((I2C2->RXDR != mess))
	{
		GPIOC -> ODR |= 1<<orange;
		return 0;
	}
	return 1;
	
}

int16_t readx()
{
	write(0x28);
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 | 1<<10 );//sets slave address, sets 2 bytes to transmit and set read and restart
	I2C2 -> CR2 |= 1<< 13 ;
	while(!((I2C2->ISR >> 2) & 1))//Receive Register Not Empty
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	while(!((I2C2->ISR >> 6) & 1))//Transfer Complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	
	int16_t ret = I2C2->RXDR;
		
		write(0x29);
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 | 1<<10 );//sets slave address, sets 2 bytes to transmit and set read and restart
	I2C2 -> CR2 |= 1<< 13 ;
	while(!((I2C2->ISR >> 2) & 1))//Receive Register Not Empty
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	while(!((I2C2->ISR >> 6) & 1))//Transfer Complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	
	 ret |= I2C2->RXDR<<8;
		return ret;
	
	
}

int16_t ready()
{
	
	write(0x2a);
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 | 1<<10 );//sets slave address, sets 2 bytes to transmit and set read and 
	I2C2 -> CR2 |= 1<< 13 ;//restart
	while(!((I2C2->ISR >> 2) & 1))//Receive Register Not Empty
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	while(!((I2C2->ISR >> 6) & 1))//Transfer Complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	
	int16_t ret = I2C2->RXDR;
		
		write(0x2b);
	I2C2 -> CR2 = (0X69 << 1 | 1<<16 | 1<<10 );//sets slave address, sets 2 bytes to transmit and set read and restart
	I2C2 -> CR2 |= 1<< 13 ;
	while(!((I2C2->ISR >> 2) & 1))//Receive Register Not Empty
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	while(!((I2C2->ISR >> 6) & 1))//Transfer Complete
		if(((I2C2->ISR >> 4) & 1))
				GPIOC -> ODR |= 1<<red; //bad nack field
	
	 ret |= I2C2->RXDR<<8;
		return ret;
	
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	init_led();
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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= 1<<12;
	
	GPIOB->MODER |= GPIO_MODER_MODER13_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	GPIOB->AFR[1] |= 5<<20;
	
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;
	GPIOB->ODR |= GPIO_ODR_14;
	
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;
	GPIOC->ODR |= GPIO_ODR_0;
	
	I2C2->TIMINGR |= (1<<28 | 0X13 <<20 | 0XF << 16 | 0X2 <<8 | 0X4);
	
	I2C2 -> CR1 |= 1; //enable
	
	

	if(0) //part 1
	{
		write(0X0F);
		read(0XD3);
		GPIOC->ODR |= 1<<green;
		I2C2->CR2 |= 1 << 14;
	}
	else //part 2
	{
		
		I2C2 -> CR2 = (0X69 << 1 | 2<<16 ); //sets slave address, sets 2 bytes to transmit and restart, inplicit write
		I2C2 -> CR2 |= 1<< 13 ;
		while(!((I2C2->ISR >> 1) & 1)) //wait till clear
				if(((I2C2->ISR >> 4) & 1))
					GPIOC -> ODR |= 1<<red; //bad nack field
				
		I2C2->TXDR = 0x20; //get gyoscope
		
		while(!((I2C2->ISR >> 1) & 1)) //wait till clear
				if(((I2C2->ISR >> 4) & 1))
					GPIOC -> ODR |= 1<<red; //bad nack field
				
		I2C2->TXDR = 0x0b; //set pd to sleep/normal and y/x enable
			
		while(!((I2C2->ISR >> 6) & 1)) //transfer complete
			if(((I2C2->ISR >> 4) & 1))
					GPIOC -> ODR |= 1<<red; //bad nack field
		
			//confirm
			write(0x20);
			read(0x0b);
	
		int16_t x,y, thres = 0x01ff;
	
		while (1)
		{
			HAL_Delay(100);
				x = readx();
				y = ready();
				if(y < thres && y > -thres)
					GPIOC->ODR &= ~(1<<blue | 1 << red);
				else if (y >= thres)
				{
					GPIOC->ODR &= ~(1<<blue);
					GPIOC->ODR |= 1<<red;
				}
				else
					{
					GPIOC->ODR &= ~(1<<red);
					GPIOC->ODR |= 1<<blue;
				}
				
				if(x < thres && x > -thres)
					GPIOC->ODR &= ~(1<<orange | 1 << green);
				else if (x <= thres)
				{
					GPIOC->ODR &= ~(1<<green);
					GPIOC->ODR |= 1<<orange;
				}
				else
					{
					GPIOC->ODR &= ~(1<<orange);
					GPIOC->ODR |= 1<<green;
				}
				
			
		}
		
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1);
	
	
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
