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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "CCS811_Basic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AHT10_ADRESS (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

uint8_t AHT10_RX_Data[6];
uint32_t AHT10_ADC_Raw;
float AHT10_Temperature;
float AHT10_Humidity;
uint8_t AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};
uint8_t* BUFFER2 = 0;
uint16_t conter2 = 0;

uint8_t dacReset[3] = {0b00101000 ,0b00000000 ,0b00000001};
uint8_t dacRegister[3] = {0b00000111 ,0b11111111 ,0b11111111};
uint8_t dacUpdate[3] = {0b00001111 ,0b00000000 ,0b00000000};

bool T_100ms = true;
bool AHT10_Switcher = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1 ,&dacReset[0] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacReset[1] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacReset[2] ,1 ,300);
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_SET);
	HAL_Delay(1000);
	
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1 ,&dacRegister[0] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacRegister[1] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacRegister[2] ,1 ,300);
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_SET);
	HAL_Delay(1000);
	
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1 ,&dacUpdate[0] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacUpdate[1] ,1 ,300);
	HAL_SPI_Transmit(&hspi1 ,&dacUpdate[2] ,1 ,300);
	
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 ,GPIO_PIN_SET);
	
	/* I2C Scanner */
	HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
	uint8_t i = 0, ret;
  for(i=1; i<128; i++)
  {
		ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    if (ret != HAL_OK) /* No ACK Received At That Address */
    {
			HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
    }
    else if(ret == HAL_OK)
    {
      sprintf(Buffer, "0x%X", i);
      HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), 10000);
		}
  }
  HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
	/* END I2C Scanner */
	
	/* CCS811 Confing */
	configureCCS811();
	readAlgorithmResults();
	/* END CCS811 Confing */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Get ATH20 Data */
		if(AHT10_Switcher)
		{
			HAL_I2C_Master_Transmit_IT(&hi2c1, AHT10_ADRESS, (uint8_t*)AHT10_TmpHum_Cmd, 3); /* Send command (trigger measuremetns) + parameters */
		} 
		else
		{
			HAL_I2C_Master_Receive_IT(&hi2c1, AHT10_ADRESS, (uint8_t*)AHT10_RX_Data, 6); /* Receive data: STATUS[1]:HIMIDITY[2.5]:TEMPERATURE[2.5] */
		}
		
		if(~AHT10_RX_Data[0] & 0x80)
		{
			/* Convert to Temperature in °C */
			AHT10_ADC_Raw = (((uint32_t)AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
			AHT10_Temperature = (float)(AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

			/* Convert to Relative Humidity in % */
			AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
			AHT10_Humidity = (float)(AHT10_ADC_Raw*100.00/1048576.00);
		}

		if(AHT10_Switcher)
		{
			AHT10_Switcher = false;
		}
		else
		{
			AHT10_Switcher = true;
		}
		HAL_Delay(100);
		/* END Get ATH20 Data */
		
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
