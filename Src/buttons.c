/*
 * buttons.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#include "stm32l1xx_hal.h"
#include "buttons.h"

/* Private Variables */
void (*setCol[12])(void);
/*********************/

/* Private function prototypes */
void setColOne();
void setColTwo();
void setColThree();
void setColFour();
void setColFive();
void setColSix();
void setColSeven();
void setColEight();
void setColNine();
void setColTen();
void setColEleven();
void setColTwelve();
/******************************/

void buttonsInit()
{
	  setCol[0] = setColOne;
	  setCol[1] = setColTwo;
	  setCol[2] = setColThree;
	  setCol[3] = setColFour;
	  setCol[4] = setColFive;
	  setCol[5] = setColSix;
	  setCol[6] = setColSeven;
	  setCol[7] = setColEight;
	  setCol[8] = setColNine;
	  setCol[9] = setColTen;
	  setCol[10] = setColEleven;
	  setCol[11] = setColTwelve;
}

uint8_t getKey()
{
	uint8_t key;

	// Wait till all keys are released
	setAllCols();
	while(readRow(1) || readRow(2));

	// Wait for key to be pressed
	do
	{
		while(!readRow(1) && !readRow(2));
		delay(20);
	}while(!readRow(1) && !readRow(2));

	// Get Key
	if(readRow(1))
	{
		key = getCol(1);
	}
	else if(readRow(2))
	{
		key = 12 + getCol(2);
	}
	else
	{
		key = -1;
	}

	// Wait till all keys are released
	setAllCols();
	while(readRow(1) || readRow(2));

	return key;
}

void delay(uint32_t milliseconds)
{
   /* Initially clear flag */
   (void) SysTick->CTRL;
   while (milliseconds != 0) {
      /* COUNTFLAG returns 1 if timer counted to 0 since the last flag read */
      milliseconds -= (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> SysTick_CTRL_COUNTFLAG_Pos;
   }
}

uint8_t getCol(uint8_t row)
{
	for(int i = 0; i < 12; i++)
	{
		(*setCol[i])();
		if(readRow(row))
		{
			return i;
		}
	}
	return -1;
}

void setColOne()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColTwo()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColThree()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_1_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColFour()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_1_Pin|COL_3_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColFive()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColSix()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, COL_6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColSeven()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_7_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_1_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColEight()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_1_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColNine()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_1_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColTen()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, COL_10_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin, GPIO_PIN_RESET);
}

void setColEleven()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, COL_11_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_10_Pin, GPIO_PIN_RESET);
}

void setColTwelve()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COL_10_Pin|COL_11_Pin, GPIO_PIN_RESET);
}

void setAllCols()
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, COL_9_Pin|COL_8_Pin |COL_7_Pin|COL_4_Pin|COL_3_Pin|COL_2_Pin|COL_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, COL_5_Pin|COL_6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, COL_12_Pin|COL_11_Pin|COL_10_Pin, GPIO_PIN_SET);
}

uint8_t readRow(uint8_t row)
{
	uint8_t reading;
	switch(row)
	{
		case 1:
			reading = HAL_GPIO_ReadPin(GPIOB, ROW_1_Pin);
			break;
		case 2:
			reading = HAL_GPIO_ReadPin(GPIOB, ROW_2_Pin);
			break;
		default:
			reading = 0;
	}
	return reading;
}
