/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define COL_9_Pin GPIO_PIN_13
#define COL_9_GPIO_Port GPIOC
#define COL_8_Pin GPIO_PIN_14
#define COL_8_GPIO_Port GPIOC
#define COL_7_Pin GPIO_PIN_15
#define COL_7_GPIO_Port GPIOC
#define COL_6_Pin GPIO_PIN_0
#define COL_6_GPIO_Port GPIOH
#define COL_5_Pin GPIO_PIN_1
#define COL_5_GPIO_Port GPIOH
#define COL_4_Pin GPIO_PIN_0
#define COL_4_GPIO_Port GPIOC
#define COL_3_Pin GPIO_PIN_1
#define COL_3_GPIO_Port GPIOC
#define COL_2_Pin GPIO_PIN_2
#define COL_2_GPIO_Port GPIOC
#define COL_1_Pin GPIO_PIN_3
#define COL_1_GPIO_Port GPIOC
#define ADC1_CS_Pin GPIO_PIN_3
#define ADC1_CS_GPIO_Port GPIOA
#define ADC_SYNC_Pin GPIO_PIN_4
#define ADC_SYNC_GPIO_Port GPIOA
#define ADC2_CS_Pin GPIO_PIN_4
#define ADC2_CS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_5
#define SD_CS_GPIO_Port GPIOC
#define LCD_WR_Pin GPIO_PIN_10
#define LCD_WR_GPIO_Port GPIOB
#define LCD_RD_Pin GPIO_PIN_11
#define LCD_RD_GPIO_Port GPIOB
#define LCD_CE_Pin GPIO_PIN_12
#define LCD_CE_GPIO_Port GPIOB
#define LCD_CD_Pin GPIO_PIN_13
#define LCD_CD_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_14
#define LCD_RESET_GPIO_Port GPIOB
#define LCD_FS_Pin GPIO_PIN_15
#define LCD_FS_GPIO_Port GPIOB
#define LCD_RV_Pin GPIO_PIN_6
#define LCD_RV_GPIO_Port GPIOC
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_9
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_8
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D3_Pin GPIO_PIN_9
#define LCD_D3_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_10
#define LCD_D2_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_11
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_12
#define LCD_D0_GPIO_Port GPIOA
#define WIFI_TX_Pin GPIO_PIN_10
#define WIFI_TX_GPIO_Port GPIOC
#define WIFI_RX_Pin GPIO_PIN_11
#define WIFI_RX_GPIO_Port GPIOC
#define WIFI_CS_Pin GPIO_PIN_12
#define WIFI_CS_GPIO_Port GPIOC
#define WIFI_RESET_Pin GPIO_PIN_2
#define WIFI_RESET_GPIO_Port GPIOD
#define ROW_1_Pin GPIO_PIN_5
#define ROW_1_GPIO_Port GPIOB
#define ROW_1_EXTI_IRQn EXTI9_5_IRQn
#define ROW_2_Pin GPIO_PIN_6
#define ROW_2_GPIO_Port GPIOB
#define ROW_2_EXTI_IRQn EXTI9_5_IRQn
#define COL_12_Pin GPIO_PIN_7
#define COL_12_GPIO_Port GPIOB
#define COL_11_Pin GPIO_PIN_8
#define COL_11_GPIO_Port GPIOB
#define COL_10_Pin GPIO_PIN_9
#define COL_10_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
