/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "AD7190.h"
#include "adc.h"
#include "buttons.h"
#include "eeprom.h"
#include "esp8266.h"
#include "ra6963.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const char* buttonStrings[24] = {"ZERO", "UNITS", "MENU", "UP", "DISPLAY TARE", "TARE", "ENTER", "RIGHT", "LEFT", "CALIBRATE", "DOWN", "GROSS/NET", "3", "2", "1", "5", "6", "4", "8", "9", "7", ".", "0", "DELETE"};
int count = 0;
FATFS mynewdiskFatFs; /* File system object for User logical drive */
FIL MyFile; /* File object */
char mynewdiskPath[4]; /* User logical drive path */
float calibrationSlope0 = 0;
float calibrationIntercept0 = 0;
float calibrationSlope1 = 0;
float calibrationIntercept1 = 0;
float calibrationSlope2 = 0;
float calibrationIntercept2 = 0;
float calibrationSlope3 = 0;
float calibrationIntercept3 = 0;
const unsigned char kChar [] = {
0xFC, 0x00, 0xFC, 0xFC, 0x00, 0xF8, 0xFC, 0x01, 0xF0, 0xFC, 0x03, 0xE0, 0xFC, 0x07, 0xE0, 0xFC,
0x0F, 0xC0, 0xFC, 0x0F, 0x80, 0xFC, 0x1F, 0x00, 0xFC, 0x3E, 0x00, 0xFC, 0x7E, 0x00, 0xFC, 0xFC,
0x00, 0xFC, 0xF8, 0x00, 0xFD, 0xF0, 0x00, 0xFF, 0xE0, 0x00, 0xFF, 0xE0, 0x00, 0xFF, 0xC0, 0x00,
0xFF, 0xC0, 0x00, 0xFF, 0xE0, 0x00, 0xFF, 0xE0, 0x00, 0xFD, 0xF0, 0x00, 0xFC, 0xF8, 0x00, 0xFC,
0xFC, 0x00, 0xFC, 0x7E, 0x00, 0xFC, 0x3E, 0x00, 0xFC, 0x1F, 0x00, 0xFC, 0x0F, 0x80, 0xFC, 0x0F,
0xC0, 0xFC, 0x07, 0xE0, 0xFC, 0x03, 0xE0, 0xFC, 0x01, 0xF0, 0xFC, 0x00, 0xF8, 0xFC, 0x00, 0xFC
};
const unsigned char gChar [] = {
0x00, 0x3C, 0x00, 0x07, 0xFF, 0xE0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xFC, 0x3F, 0xFF, 0xFC, 0x7E,
0x00, 0x7E, 0x7C, 0x00, 0x3E, 0x7C, 0x00, 0x3E, 0x7C, 0x00, 0x3E, 0xFC, 0x00, 0x3F, 0xFC, 0x00,
0x00, 0xFC, 0x00, 0x00, 0xFC, 0x00, 0x00, 0xFC, 0x00, 0x00, 0xFC, 0x00, 0x00, 0xFC, 0x00, 0x00,
0xFC, 0x07, 0xFF, 0xFC, 0x07, 0xFF, 0xFC, 0x07, 0xFF, 0xFC, 0x07, 0xFF, 0xFC, 0x07, 0xFF, 0xFC,
0x00, 0x3F, 0xFC, 0x00, 0x3F, 0x7C, 0x00, 0x3E, 0x7C, 0x00, 0x3E, 0x7C, 0x00, 0x3E, 0x7E, 0x00,
0x7E, 0x3F, 0xFF, 0xFC, 0x3F, 0xFF, 0xFC, 0x1F, 0xFF, 0xF8, 0x07, 0xFF, 0xE0, 0x00, 0x3C, 0x00
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
void sdInit();
void sd_test();
void setSdSpi();
void setAdcSpi();
void calibrate();
float getLoadCellVoltage(uint8_t loadCell);
void initCalibrationArray(Calibration_Array *arr, int initialSize);
void insertArray(Calibration_Array *arr, Calibration_Pair pair);
void freeArray(Calibration_Array *arr);
void linearReg(Calibration_Array *arr, float* calibrationSlope, float* calibrationIntercept);
float getAverageLoadCellVoltage(uint8_t loadCell);
void displayWeight(float weight);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  float voltage;
  float totalWeight;
  float weight[4] = {0};
  char weightString[10] = {0};
  char weightZeroString[10] = {0};
  char weightOneString[10] = {0};
  char weightTwoString[10] = {0};
  char weightThreeString[10] = {0};
  char display[30] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_UART4_Init();
  //MX_FATFS_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  buttonsInit();
  ra6963Init();
  setSdSpi();
  sdInit();
  setAdcSpi();

  /*ra6963TextGoTo(0, 0);
  ra6963WriteString("RemoDi Scale");

  ra6963TextGoTo(21, 0);
  ra6963WriteString("12:36 PM");

  ra6963Rectangle(0, 14, 240, 100);
  ra6963Rectangle(1, 15, 238, 98);

  ra6963WriteBigChar('0', 9, 24);
  ra6963WriteBigChar('0', 57, 24);
  ra6963WriteBigChar('0', 113, 24);
  ra6963WriteBigChar('0', 161, 24);

  ra6963Rectangle(99, 96, 8, 8);
  ra6963Rectangle(100, 97, 6, 6);
  ra6963Rectangle(101, 98, 4, 4);
  ra6963Rectangle(102, 99, 2, 2);
  drawImage(211, 27, 32, 24, kChar);
  drawImage(211, 69, 32, 24, gChar);

  ra6963TextGoTo(0, 15);
  ra6963WriteString("ALERT:");*/
  /*ra6963ClearGraphic();
  ra6963ClearText();
  ra6963ClearCG();*/

  /*wifiConnect();
  serverConnect();*/

  //sd_test();



  calibrationSlope0 = readEepromFloat(CAL_SLOPE_ADD_0);
  calibrationIntercept0 = readEepromFloat(CAL_INT_ADD_0);
  calibrationSlope1 = readEepromFloat(CAL_SLOPE_ADD_1);
  calibrationIntercept1 = readEepromFloat(CAL_INT_ADD_1);
  calibrationSlope2 = readEepromFloat(CAL_SLOPE_ADD_2);
  calibrationIntercept2 = readEepromFloat(CAL_INT_ADD_2);
  calibrationSlope3 = readEepromFloat(CAL_SLOPE_ADD_3);
  calibrationIntercept3 = readEepromFloat(CAL_INT_ADD_3);

  if(adcInit(1) && adcInit(2))
  {
	  ra6963TextGoTo(0,0);
 	  ra6963WriteString("Part Present");
  }
  else
  {
 	  ra6963TextGoTo(0,0);
 	  ra6963WriteString("Part(s) Not Present");
  }
  adcRangeSetup(0, AD7190_CONF_GAIN_128, 1);
  adcRangeSetup(0, AD7190_CONF_GAIN_128, 2);

  adcCalibrateAll();
/*  adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M, 1);
  adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M, 1);
  adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AIN4M, 1);
  adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AIN4M, 1);
  adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M, 2);
  adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M, 2);
  adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AIN4M, 2);
  adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AIN4M, 2);*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN 3 */
  while (1)
  {
	  /*voltage = getAverageLoadCellVoltage(0);
	  floatToString(voltage, display, 4);
	  ra6963ClearText();
	  ra6963TextGoTo(0, 0);
	  ra6963WriteString(display);

	  voltage = getAverageLoadCellVoltage(1);
	  floatToString(voltage, display, 4);
	  ra6963TextGoTo(0, 1);
	  ra6963WriteString(display);

	  voltage = getAverageLoadCellVoltage(2);
	  floatToString(voltage, display, 4);
	  ra6963TextGoTo(0, 2);
	  ra6963WriteString(display);

	  voltage = getAverageLoadCellVoltage(3);
	  floatToString(voltage, display, 4);
	  ra6963TextGoTo(0, 3);
	  ra6963WriteString(display);

	  HAL_Delay(500);*/
	  voltage = getLoadCellVoltage(0);
	  weight[0] = voltage * calibrationSlope0 + calibrationIntercept0;
	  floatToString(weight[0], weightZeroString, 2);
	  voltage = getLoadCellVoltage(1);
	  weight [1] = voltage * calibrationSlope1 + calibrationIntercept1;
	  floatToString(weight[1], weightOneString, 2);
	  voltage = getLoadCellVoltage(2);
	  weight[2] = voltage * calibrationSlope2 + calibrationIntercept2;
	  floatToString(weight[2], weightTwoString, 2);
	  voltage = getLoadCellVoltage(3);
	  weight[3] = voltage * calibrationSlope3 + calibrationIntercept3;
	  floatToString(weight[3], weightThreeString, 2);
	  totalWeight = (weight[0] + weight[1] + weight[2] + weight[3]) / 4;

	  /*displayWeight(totalWeight);*/
	  /*floatToString(totalWeight, weightString, 2);
	  sprintf(display,"%s kg", weightString);
	  ra6963ClearText();
	  ra6963TextGoTo(0,0);
	  ra6963WriteString(display);*/

	  sprintf(display, "L,%s,%s,%s,%s", weightZeroString, weightOneString, weightTwoString, weightThreeString);
	  serverWrite(display, 10);
	  setSdSpi();
	  if(f_open(&MyFile, "LOG.CSV", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
	  {
		  f_lseek(&MyFile, f_size(&MyFile));
		  f_puts(display, &MyFile);
		  f_putc('\n', &MyFile);
		  f_close(&MyFile);
	  }
	  setAdcSpi();

	  /*floatToString(calibrationSlope0, weightString, 2);
	  ra6963TextGoTo(0,1);
	  ra6963WriteString(weightString);
	  floatToString(calibrationIntercept0, weightString, 2);
	  ra6963TextGoTo(10,1);
	  ra6963WriteString(weightString);

	  floatToString(calibrationSlope1, weightString, 2);
	  ra6963TextGoTo(0,2);
	  ra6963WriteString(weightString);
	  floatToString(calibrationIntercept1, weightString, 2);
	  ra6963TextGoTo(10,2);
	  ra6963WriteString(weightString);

	  floatToString(calibrationSlope2, weightString, 2);
	  ra6963TextGoTo(0,3);
	  ra6963WriteString(weightString);
	  floatToString(calibrationIntercept2, weightString, 2);
	  ra6963TextGoTo(10,3);
	  ra6963WriteString(weightString);

	  floatToString(calibrationSlope3, weightString, 2);
	  ra6963TextGoTo(0,4);
	  ra6963WriteString(weightString);
	  floatToString(calibrationIntercept3, weightString, 2);
	  ra6963TextGoTo(10,4);
	  ra6963WriteString(weightString);*/
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 313;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin|COL_7_Pin|COL_4_Pin 
                          |COL_3_Pin|COL_2_Pin|COL_1_Pin|ADC2_CS_Pin 
                          |SD_CS_Pin|WIFI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, COL_6_Pin|COL_5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC1_CS_Pin|ADC_SYNC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_WR_Pin|LCD_RD_Pin|LCD_CE_Pin|LCD_RESET_Pin 
                          |COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CD_Pin|LCD_FS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RV_GPIO_Port, LCD_RV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port, WIFI_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : COL_9_Pin COL_8_Pin COL_7_Pin COL_4_Pin 
                           COL_3_Pin COL_2_Pin COL_1_Pin LCD_RV_Pin 
                           WIFI_CS_Pin */
  GPIO_InitStruct.Pin = COL_9_Pin|COL_8_Pin|COL_7_Pin|COL_4_Pin 
                          |COL_3_Pin|COL_2_Pin|COL_1_Pin|LCD_RV_Pin 
                          |WIFI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : COL_6_Pin COL_5_Pin */
  GPIO_InitStruct.Pin = COL_6_Pin|COL_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC1_CS_Pin ADC_SYNC_Pin */
  GPIO_InitStruct.Pin = ADC1_CS_Pin|ADC_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC2_CS_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = ADC2_CS_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_WR_Pin LCD_RD_Pin LCD_CE_Pin LCD_CD_Pin 
                           LCD_RESET_Pin LCD_FS_Pin COL_12_Pin COL_11_Pin 
                           COL_10_Pin */
  GPIO_InitStruct.Pin = LCD_WR_Pin|LCD_RD_Pin|LCD_CE_Pin|LCD_CD_Pin 
                          |LCD_RESET_Pin|LCD_FS_Pin|COL_12_Pin|COL_11_Pin 
                          |COL_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D3_Pin LCD_D2_Pin LCD_D1_Pin 
                           LCD_D0_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D3_Pin|LCD_D2_Pin|LCD_D1_Pin 
                          |LCD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RESET_Pin */
  GPIO_InitStruct.Pin = WIFI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(WIFI_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_1_Pin ROW_2_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin|ROW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//uint8_t key;
	uint8_t col;
	//char display[30];
	delay(50);
	if(GPIO_Pin == GPIO_PIN_5)
	{
		col = getCol(1);
		if(col >= 0 && col <= 11)
		{
			switch(col)
			{
				case CALIBRATE:
					calibrate();
					break;
				default:
					ra6963ClearText();
					ra6963TextGoTo(0, 0);
					ra6963WriteString("NOT CALIBRATION MODE");
			}
		}

/*		if(col >= 1 && col <= 12)
		{
			//ra6963ClearText();
			//count++;
			//ra6963TextGoTo(count, 0);
			//sprintf(display, "%d", count);
			//ra6963WriteString(rowOneStrings[col - 1]);
			//ra6963WriteString(display);
			//key = getKey();
			//ra6963TextGoTo(count, 1);
			//sprintf(display, "%d", key);
			//ra6963WriteString(buttonStrings[key]);
		}*/
		setAllCols();
		while(readRow(1));
	}
	else if(GPIO_Pin == GPIO_PIN_6)
	{
		col = getCol(2);
		if(col >= 0 && col <= 11)
		{
			ra6963ClearText();
			ra6963TextGoTo(0, 0);
			ra6963WriteString("NOT CALIBRATION MODE");
			//count++;
			//ra6963ClearText();
			//ra6963TextGoTo(count, 0);
			//sprintf(display, "%d", count);
			//ra6963WriteString(rowTwoStrings[col - 1]);
			//ra6963WriteString(display);
			//key = getKey();
			//ra6963TextGoTo(count, 1);
			//ra6963WriteString(buttonStrings[key]);
		}
		setAllCols();
		while(readRow(2));
	}
}

void setAdcSpi()
{
	MX_SPI1_Init();
}


void setSdSpi()
{
	 //SPI1 parameter configuration
	  hspi1.Instance = SPI1;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi1.Init.NSS = SPI_NSS_SOFT;
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi1.Init.CRCPolynomial = 10;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }
}

void calibrate()
{
	static Calibration_Array LC0;
	static Calibration_Array LC1;
	static Calibration_Array LC2;
	static Calibration_Array LC3;
	static uint8_t init = 0;
	Calibration_Pair tmp = {0, 0};
	uint8_t key;
	float cWeight;
	char cWeightStr[10] = {0};
	char display [30];

	// Initialize arrays if not already initialized
	if(!init)
	{
		initCalibrationArray(&LC0, 7);
		initCalibrationArray(&LC1, 7);
		initCalibrationArray(&LC2, 7);
		initCalibrationArray(&LC3, 7);
		init = 1;
	}

	ra6963ClearText();
	ra6963TextGoTo(0, 0);
	ra6963WriteString("CALIBRATION MODE");

	// Wait for weight to be placed on scale
	ra6963TextGoTo(0, 2);
	ra6963WriteString("Press Enter After Calibrated");
	ra6963TextGoTo(0, 3);
	ra6963WriteString("Weight is Placed on Scale!");
	do
	{
		key = getKey();
	}while(key != ENTER);

	//Read ADC values
	adcCalibrateAll();
	tmp.voltage = getAverageLoadCellVoltage(0);
	insertArray(&LC0, tmp);
	tmp.voltage = getAverageLoadCellVoltage(1);
	insertArray(&LC1, tmp);
	tmp.voltage = getAverageLoadCellVoltage(2);
	insertArray(&LC2, tmp);
	tmp.voltage = getAverageLoadCellVoltage(3);
	insertArray(&LC3, tmp);
	/*ra6963ClearText();

	int length = LC0.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC0.array[i].voltage;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		delay(500);
	}

	ra6963ClearText();

	length = LC1.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC1.array[i].voltage;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		delay(500);
	}

	ra6963ClearText();

	length = LC2.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC2.array[i].voltage;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		delay(500);
	}

	ra6963ClearText();

	length = LC3.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC3.array[i].voltage;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		delay(500);
	}*/
	//Get Weight
	ra6963TextGoTo(0, 5);
	ra6963WriteString("Enter Weight: ");
	//cWeight  = atof(str);
	//floatToString(cWeight, display, 2);
	//ra6963TextGoTo(14, 5);
	//ra6963WriteString("1");
	do
	{
		key = getKey();
		switch(key)
		{
			case ZERO:
				strcat(cWeightStr, "0");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("0");
				break;
			case ONE:
				strcat(cWeightStr, "1");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("1");
				break;
			case TWO:
				strcat(cWeightStr, "2");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("2");
				break;
			case THREE:
				strcat(cWeightStr, "3");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("3");
				break;
			case FOUR:
				strcat(cWeightStr, "4");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("4");
				break;
			case FIVE:
				strcat(cWeightStr, "5");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("5");
				break;
			case SIX:
				strcat(cWeightStr, "6");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("6");
				break;
			case SEVEN:
				strcat(cWeightStr, "7");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("7");
				break;
			case EIGHT:
				strcat(cWeightStr, "8");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("8");
				break;
			case NINE:
				strcat(cWeightStr, "9");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString("9");
				break;
			case PERIOD:
				strcat(cWeightStr, ".");
				ra6963TextGoTo(13 + strlen(cWeightStr), 5);
				ra6963WriteString(".");
				break;
		}
	}while(key != ENTER);
	cWeight  = atof(cWeightStr);
	LC0.array[LC0.used - 1].weight = cWeight;
	LC1.array[LC1.used - 1].weight = cWeight;
	LC2.array[LC2.used - 1].weight = cWeight;
	LC3.array[LC3.used - 1].weight = cWeight;

/*	ra6963ClearText();

	int length = LC0.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC0.array[i].voltage;
		float tmpWeight = LC0.array[i].weight;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		floatToString(tmpWeight, cWeightStr, 2);
		ra6963TextGoTo(10, i);
		ra6963WriteString(cWeightStr);
		delay(1000);
	}

	ra6963ClearText();

	length = LC1.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC1.array[i].voltage;
		float tmpWeight = LC1.array[i].weight;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		floatToString(tmpWeight, cWeightStr, 2);
		ra6963TextGoTo(10, i);
		ra6963WriteString(cWeightStr);
		delay(1000);
	}

	ra6963ClearText();

	length = LC2.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC2.array[i].voltage;
		float tmpWeight = LC2.array[i].weight;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		floatToString(tmpWeight, cWeightStr, 2);
		ra6963TextGoTo(10, i);
		ra6963WriteString(cWeightStr);
		delay(1000);
	}

	ra6963ClearText();

	length = LC3.used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = LC3.array[i].voltage;
		float tmpWeight = LC3.array[i].weight;
		floatToString(tmpVoltage, cWeightStr, 2);
		ra6963TextGoTo(0, i);
		ra6963WriteString(cWeightStr);
		floatToString(tmpWeight, cWeightStr, 2);
		ra6963TextGoTo(10, i);
		ra6963WriteString(cWeightStr);
		delay(1000);
	}*/
	floatToString(cWeight, display, 2);
	ra6963TextGoTo(0, 6);
	ra6963WriteString(display);
	linearReg(&LC0, &calibrationSlope0, &calibrationIntercept0);
	writeEepromFloat(CAL_SLOPE_ADD_0, calibrationSlope0);
	writeEepromFloat(CAL_INT_ADD_0, calibrationIntercept0);
	linearReg(&LC1, &calibrationSlope1, &calibrationIntercept1);
	writeEepromFloat(CAL_SLOPE_ADD_1, calibrationSlope1);
	writeEepromFloat(CAL_INT_ADD_1, calibrationIntercept1);
	linearReg(&LC2, &calibrationSlope2, &calibrationIntercept2);
	writeEepromFloat(CAL_SLOPE_ADD_2, calibrationSlope2);
	writeEepromFloat(CAL_INT_ADD_2, calibrationIntercept2);
	linearReg(&LC3, &calibrationSlope3, &calibrationIntercept3);
	writeEepromFloat(CAL_SLOPE_ADD_3, calibrationSlope3);
	writeEepromFloat(CAL_INT_ADD_3, calibrationIntercept3);
}

void linearReg(Calibration_Array *arr, float* calibrationSlope, float* calibrationIntercept)
{
	float voltageSum = 0;
	float voltageSquaredSum = 0;
	float weightSum = 0;
	float voltageWeightSum = 0;
	int length = arr->used;

	for(int i = 0; i < length; i++)
	{
		float tmpVoltage = arr->array[i].voltage;
		float tmpWeight = arr->array[i].weight;
		voltageSum += tmpVoltage;
		voltageSquaredSum += tmpVoltage * tmpVoltage;
		weightSum += tmpWeight;
		voltageWeightSum += tmpVoltage * tmpWeight;
	}

	*calibrationSlope = (length * voltageWeightSum - voltageSum * weightSum) / (length * voltageSquaredSum - voltageSum * voltageSum);
	*calibrationIntercept = (weightSum * voltageSquaredSum - voltageSum * voltageWeightSum) / (length * voltageSquaredSum - voltageSum * voltageSum);
}

float getLoadCellVoltage(uint8_t loadCell)
{
	unsigned long buffer = 0;
	float voltage;

	switch(loadCell)
	{
		case 0:
			adcChannelSelect(AD7190_CH_AIN1P_AIN2M, 1);
			buffer = adcSingleConversion(1);
			break;
		case 1:
			adcChannelSelect(AD7190_CH_AIN3P_AIN4M, 1);
			buffer = adcSingleConversion(1);
			break;
		case 2:
			adcChannelSelect(AD7190_CH_AIN1P_AIN2M, 2);
			buffer = adcSingleConversion(2);
			break;
		case 3:
			adcChannelSelect(AD7190_CH_AIN3P_AIN4M, 2);
			buffer = adcSingleConversion(2);
			break;
	}

	voltage = ((float)buffer / 16777215ul) * 78.1 - 39.05;
	return voltage;
}

void displayWeight(float weight)
{
	char weightStr[7];

	floatToString(weight, weightStr, 2);
	//sprintf(display,"%s kg", weightString);
	//ra6963ClearText();
	//ra6963TextGoTo(0,0);
	//ra6963WriteString(display);
	ra6963WriteBigChar(weightStr[0], 9, 24);
	ra6963WriteBigChar(weightStr[1], 57, 24);
	ra6963WriteBigChar(weightStr[3], 113, 24);
	ra6963WriteBigChar(weightStr[4], 161, 24);
}


float getAverageLoadCellVoltage(uint8_t loadCell)
{
	unsigned long buffer = 0;
	float voltage;

	switch(loadCell)
	{
		case 0:
			adcChannelSelect(AD7190_CH_AIN1P_AIN2M, 1);
			buffer = adcContinuousReadAvg(50, 1);
			break;
		case 1:
			adcChannelSelect(AD7190_CH_AIN3P_AIN4M, 1);
			buffer = adcContinuousReadAvg(50, 1);
			break;
		case 2:
			adcChannelSelect(AD7190_CH_AIN1P_AIN2M, 2);
			buffer = adcContinuousReadAvg(50, 2);
			break;
		case 3:
			adcChannelSelect(AD7190_CH_AIN3P_AIN4M, 2);
			buffer = adcContinuousReadAvg(50, 2);
			break;
	}

	voltage = ((float)buffer / 16777215ul) * 78.1 - 39.05;
	return voltage;
}

void initCalibrationArray(Calibration_Array *arr, int initialSize)
{
	arr->array = (Calibration_Pair *)malloc(initialSize * sizeof(Calibration_Pair));
	arr->used = 0;
	arr->size = initialSize;
}

void insertArray(Calibration_Array *arr, Calibration_Pair pair)
{
	if(arr->used == arr->size)
	{
		arr->size *= 2;
		arr->array = (Calibration_Pair *)realloc(arr->array, arr->size * sizeof(Calibration_Pair));
	}
	arr->array[arr->used++] = pair;
}

void freeArray(Calibration_Array *arr)
{
	free(arr->array);
	arr->array = NULL;
	arr->used = arr->size = 0;
}

void sdInit()
{
	if(FATFS_LinkDriver(&SD_Driver, mynewdiskPath) == 0)
	{
		f_mount(&mynewdiskFatFs, (TCHAR const*)mynewdiskPath, 0) == FR_OK;
	}
}

/*FATFS_UnLinkDriver(mynewdiskPath);
clearScreen();
ssd1306_WriteString("Unlinked", 1);
updateScreen();
HAL_Delay(1000);*/

/*void sd_test()
{
	//uint32_t wbytes; /* File write counts */
	//uint8_t wtext[] = "text to write logical disk"; /* File write buffer */
	/*char line[15];

	setSdSpi();

	if(FATFS_LinkDriver(&SD_Driver, mynewdiskPath) == 0)
	{
		ra6963ClearText();
		ra6963TextGoTo(0, 0);
		ra6963WriteString("Linked");
		HAL_Delay(1000);
		if(f_mount(&mynewdiskFatFs, (TCHAR const*)mynewdiskPath, 0) == FR_OK)
		{
			ra6963ClearText();
			ra6963TextGoTo(0, 0);
			ra6963WriteString("Mounted");
			HAL_Delay(1000);
//			if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
			if(f_open(&MyFile, "STM32.TXT", FA_READ) == FR_OK)
			{
				ra6963ClearText();
				ra6963TextGoTo(0, 0);
				ra6963WriteString("Opened");
				HAL_Delay(1000);
//				if(f_write(&MyFile, wtext, sizeof(wtext), (void *)&wbytes) == FR_OK)
				f_gets(line, sizeof(line), &MyFile);
//				{
					ra6963ClearText();
					ra6963TextGoTo(0, 0);
//					ra6963WriteString("Written");
					ra6963WriteString("Read");
					ra6963WriteString(line);
					HAL_Delay(1000);
					f_close(&MyFile);
					ra6963ClearText();
					ra6963TextGoTo(0, 0);
					ra6963WriteString("Closed");
					HAL_Delay(1000);
//				}
			}
		}
	}
	FATFS_UnLinkDriver(mynewdiskPath);
	ra6963ClearText();
	ra6963TextGoTo(0, 0);
	ra6963WriteString("Unlinked");
	HAL_Delay(1000);
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
