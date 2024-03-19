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

void init_leds(void)
{
	/* Setup Red LED (PC6) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<12);
	GPIOC->MODER &= ~(1<<13);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<6);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<13);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	// Initialize to low
	GPIOC->ODR &= ~(1<<6);

	/* Setup Blue LED (PC7) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<14);
	GPIOC->MODER &= ~(1<<15);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<7);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<14);
	GPIOC->OSPEEDR &= ~(1<<15);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	// Initialize to low
	GPIOC->ODR &= ~(1<<7);
	
	/* Setup Orange LED (PC8) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<16);
	GPIOC->MODER &= ~(1<<17);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<8);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<16);
	GPIOC->OSPEEDR &= ~(1<<17);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	// Initialize to low
	GPIOC->ODR &= ~(1<<8);
	
	/* Setup Green LED (PC9) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<18);
	GPIOC->MODER &= ~(1<<19);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<9);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->OSPEEDR &= ~(1<<19);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
	// Initialize to low
	GPIOC->ODR &= ~(1<<9);
	
	/* Enable LEDs in RCC */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
}

int init_adc()
{
	int cal_factor = 0;
	/* Setup GPIO for ADC (PC0) */
	// Set to analog mode
	GPIOC->MODER |= (1<<0);
	GPIOC->MODER |= (1<<1);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<0);
	GPIOC->PUPDR &= ~(1<<1);
	
	/* Enable ADC1 in RCC */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	/* Configure ADC */
	// 8-bit resolution
	ADC1->CFGR1 &= ~(1<<3);
	ADC1->CFGR1 |= (1<<4);
	// Continuous conversion mode
	ADC1->CFGR1 |= (1<<13);
	// Disable hardware triggers (software trigger only)
	ADC1->CFGR1 &= ~(1<<10);
	ADC1->CFGR1 &= ~(1<<11);
	
	/* Select input pin/channel for ADC */
	ADC1->CHSELR |= (1<<10);
	
	/* Perform self-calibration */
	// Set ADEN = 0
	ADC1->CR &= ~(1<<0);
	// Set DMAEN = 0
	ADC1->CFGR1 &= ~(1<<0);
	// Set ADCAL = 1
	ADC1->CR |= (1<<31);
	// Wait until ADCAL = 0
	while (ADC1->CR & (1<<31)) {}
	// Read calibration factor
	cal_factor = (ADC1->DR & (0x3F));
		
	/* Enable ADC */
	
	
	/* Start ADC */
		
	return cal_factor;
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	init_leds();
	init_adc();
  
  while (1)
  {
    
  }
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
