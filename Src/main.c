/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
#include "stm32f0xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define REG_INTERRUPT_STATUS_1 0x00
#define REG_INTERRUPT_STATUS_2 0x01
#define REG_INTERRUPT_ENABLE_1 0x02
#define REG_INTERRUPT_ENABLE_2 0x03
#define MAX30102_ADDR_WRITE 0xAE
#define MAX30102_ADDR_READ 0xAF
#define PART_ID 0xFF
#define REG_MODE_CONFIGURATION 0x09
#define REG_LED_PLUSE_AMPLITUDE_1 0x0c
#define REG_LED_PLUSE_AMPLITUDE_2 0x0d
#define REG_PROXIMITY_MODE_LED_PLUSE_AMPLITUDE 0x10
#define REG_SPO2_CONFIGURATION 0x0a
#define REG_FIFO_DATA_REGISTER 0x07
#define REG_FIFO_WRITE_POINTER 0x04
#define REG_OVERFLOW_COUNTER 0x05
#define REG_FIFO_READ_POINTER 0x06
#define REG_FIFO_CONFIGURATION 0x08

#define BUFF_SIZE 200
#define BUFF_SIZE_half BUFF_SIZE/2

#define FILTER_LEVEL 8

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int __io_putchar(int ch){
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  printf("Hello, dear user\r\n");

  max30102_init();

  uint8_t flag = 0;
  uint8_t tstflag = 0;
  uint8_t spo2s = 0;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  uint8_t part_id = 0;
  uint8_t disflag = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WLE */

	  while(HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, PART_ID, I2C_MEMADD_SIZE_8BIT, &part_id, 1, 10)){
			if(disflag == 0){
				printf("MAX30102 is disconnected\r\n");
			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			disflag = 1;
	  }
	  disflag = 0;


		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {


			max30102_cal();
			uint8_t spo2 = max30102_getSpO2();

			if(spo2 != 0){

				if(spo2 > 89){
					if(spo2s != spo2){
						printf("Oxygen level: %d percent\r\n", spo2);

						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
						spo2s = spo2;
						HAL_Delay(10);
						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
					}
					tstflag = 0;
				}else if(tstflag == 0){
					tstflag = 1;
				}
				flag = 0;
			}else if(spo2 == 0 && flag == 0){
				flag = 1;
			}
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}




void max30102_init()
{
	  uint8_t part_id = 0;
	  uint8_t data;
	  uint8_t onflag = 0;

	  while(part_id != 0x15){
		  HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, PART_ID, I2C_MEMADD_SIZE_8BIT, &part_id, 1, 10);
		  if(onflag == 0 && part_id != 0x15){
			   printf("MAX30102 is not available\r\n");
		  }
		  onflag = 1;
	  }

     printf("MAX30102 is connected\r\n");

	 uint8_t mode = 0;

	 //MAX check

	 HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
	 if(mode == 0x00){
		 printf("MAX30102 is OFF\r\n");
		 while (mode == 0x00){
			 printf("proceccing...\r\n");
			 mode = 0x03;
			 HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
			 HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
		 }
	 }

	 //LED CONF
	data = 0x47;
	HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_LED_PLUSE_AMPLITUDE_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_LED_PLUSE_AMPLITUDE_2, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	//data = 0x7f;
	HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_PROXIMITY_MODE_LED_PLUSE_AMPLITUDE, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

	//Spo2 CONF
	data = 0x63;
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_SPO2_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

    //FIFO conf
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_FIFO_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

    //INTERRUPT
    data= 0xc0;
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_INTERRUPT_ENABLE_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    //INTERRUPT2
    data= 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_INTERRUPT_ENABLE_2, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

	// FIFO CLEAR
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_FIFO_WRITE_POINTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_OVERFLOW_COUNTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR_WRITE, REG_FIFO_READ_POINTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

    // STATUS CLEAR
    uint8_t dataTemp = 0;
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_INTERRUPT_STATUS_1, I2C_MEMADD_SIZE_8BIT, &dataTemp, 1, 10);
    data = dataTemp;
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_INTERRUPT_STATUS_2, I2C_MEMADD_SIZE_8BIT, &dataTemp, 1, 10);

	printf("MAX30102 is ON\r\n");
}

uint8_t max30102_getUnreadSampleCount()
{
	uint8_t wr = 0, rd = 0;
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_FIFO_WRITE_POINTER, I2C_MEMADD_SIZE_8BIT, &wr, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_FIFO_READ_POINTER, I2C_MEMADD_SIZE_8BIT, &rd, 1, 10);

    if ((wr - rd) < 0)
        return wr - rd + 32;
    else
        return wr - rd;
}


uint8_t flagput = 0;
uint8_t flagproc = 0;

void max30102_getFIFO(SAMPLE *data, uint8_t sampleCount){

    uint8_t dataTemp[sampleCount * 6];

    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR_READ, REG_FIFO_DATA_REGISTER, I2C_MEMADD_SIZE_8BIT, dataTemp, 6 * sampleCount, 25);

    if(dataTemp[0] != 0){
		flagput = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		if(flagproc == 0){
			printf("Processing...\r\n");
			flagproc = 1;
		}
    }else{
    	flagproc = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		if(flagput == 0){
			printf("Put your finger please\r\n");
			flagput = 1;
		}
    }

    uint8_t i;
    for (i = 0; i < sampleCount; i++)
    {
        data[i].red = (((uint32_t)dataTemp[i * 6]) << 16 | ((uint32_t)dataTemp[i * 6 + 1]) << 8 | dataTemp[i * 6 + 2]) & 0x3ffff;
        data[i].iRed = (((uint32_t)dataTemp[i * 6 + 3]) << 16 | ((uint32_t)dataTemp[i * 6 + 4]) << 8 | dataTemp[i * 6 + 5]) & 0x3ffff;
    }

}

SAMPLE sampleBuff[BUFF_SIZE];
uint8_t spo2 = 0;

uint16_t redAC = 0;
uint32_t redDC = 0;
uint16_t iRedAC = 0;
uint32_t iRedDC = 0;

uint16_t redAC_prev = 0;
uint32_t redDC_prev = 0;
uint16_t iRedAC_prev = 0;
uint32_t iRedDC_prev = 0;

#define FILTER_LEVEL 8

void filter(SAMPLE *s)
{
    uint8_t i;
    uint32_t red = 0;
    uint32_t ired = 0;
    for (i = 0; i < FILTER_LEVEL - 1; i++)
    {
        red += sampleBuff[i].red;
        ired += sampleBuff[i].iRed;
    }
    s->red = (red + s->red) / FILTER_LEVEL;
    s->iRed = (ired + s->iRed) / FILTER_LEVEL;
}


uint32_t rMax = 2;
uint32_t rMin = 1;
uint32_t iMax = 2;
uint32_t iMin = 1;

uint32_t maxFindSteep=0;
uint32_t minFindSteep=0;

int16_t eachSampleDiff = 0;


uint32_t	lenghtWave[2] = {0,0};

#define MIDDLEBUUF BUFF_SIZE
uint32_t middle_ir[1 + 1 + 1 + MIDDLEBUUF];

void calcAcDcFIFO(SAMPLE s, uint16_t *rac, uint32_t *rdc, uint16_t *iac, uint32_t *idc) {
	// Уровень с ик диода выше чем с красного
    uint8_t i;
	for (i = BUFF_SIZE - 1; i > 0; i--) {
		sampleBuff[i].red = sampleBuff[i - 1].red;
		sampleBuff[i].iRed = sampleBuff[i - 1].iRed;
		middle_ir[3 + i] = middle_ir[3 + i - 1];
	}
	if(s.red > s.iRed) {	// Уровень с ик диода выше чем с красного
		sampleBuff[0].red = s.iRed;
		sampleBuff[0].iRed = s.red;
		middle_ir[3] = s.red;
	} else {
		sampleBuff[0].red = s.red;
		sampleBuff[0].iRed = s.iRed;
		middle_ir[3] = s.iRed;
	}

	{	// фильтрация
		uint32_t acc32 = (middle_ir[3] + middle_ir[4] + middle_ir[5]) / 3;// + middle_ir[6] + middle_ir[7]) / 5;
		middle_ir[3] = acc32;

		// среднеарифметическое значение в middle_ir[0]
		uint64_t acc64=0;
		for(uint32_t k=3; k<(MIDDLEBUUF+3); k++)
			acc64 += ((uint64_t)middle_ir[k]*(uint64_t)middle_ir[k]);
		acc64 = acc64/MIDDLEBUUF;
		middle_ir[0] = ( ((uint32_t)sqrt(acc64)));
	}

	if( (middle_ir[0] < middle_ir[3+BUFF_SIZE_half]) && (middle_ir[0] > middle_ir[3+BUFF_SIZE_half+2]) ) { // рост максимума
		maxFindSteep = 0;
		minFindSteep = 0;

    	iMax = 0;
    	iMin = 0xffffffff;
    	rMax = 0;
    	rMin = 0xffffffff;

		uint32_t max_num = 0;
		for(uint32_t k=0; k<BUFF_SIZE_half; k++) {// поиск максимума
			if( (middle_ir[0] > middle_ir[3+BUFF_SIZE_half-k]) && (middle_ir[0] < middle_ir[3+BUFF_SIZE_half+2-k]) )
				break; // уход в минус относительно средней
			if (middle_ir[3+BUFF_SIZE_half-k] > iMax){
				iMax = middle_ir[3+BUFF_SIZE_half-k];
				max_num = BUFF_SIZE_half-k;
			}
			maxFindSteep++;
		}
		for(uint32_t k=(max_num-10); k < (max_num+10); k++) {// уточнение максимума
			if (sampleBuff[k].iRed > iMax){
				iMax = sampleBuff[k].iRed;
				rMax = sampleBuff[k].red;
			}
		}
		uint32_t min_num=0;
		for(uint32_t k=0; k<(BUFF_SIZE_half-2); k++) {// поиск минимума
			if( (middle_ir[0] > middle_ir[3+BUFF_SIZE_half+k]) && (middle_ir[0] < middle_ir[3+BUFF_SIZE_half+2+k]) )
				break;
			if (middle_ir[3+BUFF_SIZE_half+k] < iMin){
				iMin = middle_ir[3+BUFF_SIZE_half+k];
				min_num = BUFF_SIZE_half+k;
			}
			minFindSteep++;
		}
		for(uint32_t k=(min_num-10); k < (min_num+10); k++) {// уточнение минимума
			if (sampleBuff[k].iRed < iMin){
				iMin = sampleBuff[k].iRed;
				rMin = sampleBuff[k].red;
			}
		}

		lenghtWave[0] = (maxFindSteep);
		lenghtWave[1] = (minFindSteep);
	}

    *rac = rMax - rMin;
    *rdc = (rMax + rMin) / 2;
    *iac = iMax - iMin;
    *idc = (iMax + iMin) / 2;
}

void max30102_cal()
{
    uint8_t unreadSampleCount = max30102_getUnreadSampleCount();

    SAMPLE sampleBuffTemp[unreadSampleCount];
    max30102_getFIFO(sampleBuffTemp, unreadSampleCount);

    uint8_t i = 0;

    for (i = 0; i < unreadSampleCount; i++)
    {
        if (sampleBuffTemp[i].iRed < 40000) //Нет пальцев, нет расчета, пропустить
        {
            spo2 = 0;
            eachSampleDiff = 0;
            continue;
        }

        calcAcDcFIFO(sampleBuffTemp[i], &redAC, &redDC, &iRedAC, &iRedDC);
        filter(&sampleBuffTemp[i]);

			float R = (((float)(redAC)) / ((float)(redDC))) / (((float)(iRedAC)) / ((float)(iRedDC)));
			if (R >= 0.36 && R < 0.66)
				spo2 = (uint8_t)(107 - 20 * R);
			else if (R >= 0.66 && R < 1)
				spo2 = (uint8_t)(129.64 - 54 * R);

			redAC_prev = redAC;
			redDC_prev = redDC;
			iRedAC_prev = iRedAC;
			iRedDC_prev = iRedDC;
        eachSampleDiff = middle_ir[3 + BUFF_SIZE_half] - iMin;

    }

}


uint8_t max30102_getSpO2() { return spo2; }
uint16_t max30102_getMaxMin() { return (uint16_t)(iMax - iMin); }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
