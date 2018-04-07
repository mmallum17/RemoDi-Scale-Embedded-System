/*
 * ra6963.c
 *
 *  Created on: Mar 14, 2018
 *      Author: Marcus Mallum
 */

#include "stm32l1xx_hal.h"
#include "ra6963.h"

/*Private Function Prototypes*/
unsigned char ra6963In();
void ra6963Out(unsigned char outByte);

const char color = 1;

/*Functions*/
void ra6963Init()
{
	HAL_GPIO_WritePin(GPIOB, LCD_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB, LCD_RESET_Pin, GPIO_PIN_SET);

	ra6963WriteData(GLCD_GRAPHIC_HOME & 0xFF);
	ra6963WriteData(GLCD_GRAPHIC_HOME >> 8);
	ra6963WriteCommand(T6963_SET_GRAPHIC_HOME_ADDRESS);

	ra6963WriteData(GLCD_GRAPHIC_AREA);
	ra6963WriteData(0);
	ra6963WriteCommand(T6963_SET_GRAPHIC_AREA);

	ra6963WriteData(GLCD_TEXT_HOME);
	ra6963WriteData(GLCD_TEXT_HOME >> 8);
	ra6963WriteCommand(T6963_SET_TEXT_HOME_ADDRESS);

	ra6963WriteData(GLCD_TEXT_AREA);
	ra6963WriteData(0);
	ra6963WriteCommand(T6963_SET_TEXT_AREA);

	ra6963WriteData(GLCD_OFFSET_REGISTER);
	ra6963WriteData(0);
	ra6963WriteCommand(T6963_SET_OFFSET_REGISTER);

	ra6963WriteData(0);
	ra6963WriteData(0);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);

	ra6963WriteCommand(T6963_DISPLAY_MODE | T6963_TEXT_DISPLAY_ON | T6963_GRAPHIC_DISPLAY_ON);

	ra6963WriteCommand(T6963_MODE_SET | 0);
}

void ra6963ClearText()
{
	int i;

	ra6963WriteData(GLCD_TEXT_HOME);
	ra6963WriteData(GLCD_TEXT_HOME >> 8);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);

	for(i = 0; i < GLCD_TEXT_SIZE; i++)
	{
		ra6963WriteData(0);
		ra6963WriteCommand(T6963_DATA_WRITE_AND_INCREMENT);
	}
}

void ra6963ClearCG()
{
	int i;

	ra6963WriteData(GLCD_EXTERNAL_CG_HOME & 0xFF);
	ra6963WriteData(GLCD_EXTERNAL_CG_HOME >> 8);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);

	for(i = 0; i < 256 * 8; i++)
	{
		ra6963WriteData(0);
		ra6963WriteCommand(T6963_DATA_WRITE_AND_INCREMENT);
	}
}

void ra6963ClearGraphic()
{
	int i;

	ra6963WriteData(GLCD_GRAPHIC_HOME & 0xFF);
	ra6963WriteData(GLCD_GRAPHIC_HOME >> 8);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);

	for(i = 0; i < GLCD_GRAPHIC_SIZE; i++)
	{
		ra6963WriteData(0);
		ra6963WriteCommand(T6963_DATA_WRITE_AND_INCREMENT);
	}
}

unsigned char ra6963ReadData()
{
	unsigned char readData;

	while(!(ra6963CheckStatus() & 0x03));

	HAL_GPIO_WritePin(GPIOB, LCD_CD_Pin, GPIO_PIN_RESET); // Clear C/D for data mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_RESET); // Enable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_RD_Pin, GPIO_PIN_RESET); // Enable read mode

	readData = ra6963In();

	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_SET); // Disable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_RD_Pin, GPIO_PIN_SET); // Disable read mode

	return readData;
}

void ra6963WriteData(unsigned char writeData)
{
	// Wait for valid status
	while(!(ra6963CheckStatus() & 0x03));

	// Send data to data bus
	ra6963Out(writeData);

	// Write data to LCD
	HAL_GPIO_WritePin(GPIOB, LCD_CD_Pin, GPIO_PIN_RESET); // Clear C/D for data mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_RESET); // Enable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_WR_Pin, GPIO_PIN_RESET); // Enable write mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_SET); // Disable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_WR_Pin, GPIO_PIN_SET); // Disable write mode
}

void ra6963WriteCommand(unsigned char writeCommand)
{
	// Wait for valid status
	while(!(ra6963CheckStatus() & 0x03));

	// Send data to data bus
	ra6963Out(writeCommand);

	// Write data to LCD
	HAL_GPIO_WritePin(GPIOB, LCD_CD_Pin, GPIO_PIN_SET); // Set C/D for command mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_RESET); // Enable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_WR_Pin, GPIO_PIN_RESET); // Enable write mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_SET); // Disable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_WR_Pin, GPIO_PIN_SET); // Disable write mode
}

unsigned char ra6963CheckStatus()
{
	unsigned char statusData;
	//char display[30];

	HAL_GPIO_WritePin(GPIOB, LCD_CD_Pin, GPIO_PIN_SET); // Set C/D for command mode
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_RESET); // Enable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_RD_Pin, GPIO_PIN_RESET); // Enable read mode

	statusData = ra6963In();

	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin, GPIO_PIN_SET); // Disable LCD controller
	HAL_GPIO_WritePin(GPIOB, LCD_RD_Pin, GPIO_PIN_SET); // Disable read mode

	/*sprintf(display, "%X", statusData);
	clearScreen();
	ssd1306_WriteString(display, 1);
	updateScreen();*/

	return statusData;
}

void ra6963WriteString(char* str)
{
	while(*str)
	{
		ra6963WriteChar(*str++);
	}
}

void ra6963WriteChar(char ch)
{
	ra6963WriteData(ch - 32);
	ra6963WriteCommand(T6963_DATA_WRITE_AND_INCREMENT);
}

void ra6963TextGoTo(int x, int y)
{
	int address = GLCD_TEXT_HOME + x + (GLCD_TEXT_AREA * y);

	ra6963WriteData(address);
	ra6963WriteData(address >> 8);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);
}


void ra6963SetPixel(int x, int y, int color)
{
	unsigned char tmp;
	int address;

	address = GLCD_GRAPHIC_HOME + (x / GLCD_FONT_WIDTH) + (GLCD_GRAPHIC_AREA * y);

	ra6963WriteData(address & 0xFF);
	ra6963WriteData(address >> 8);
	ra6963WriteCommand(T6963_SET_ADDRESS_POINTER);

	ra6963WriteCommand(T6963_DATA_READ_AND_NONVARIABLE);
	tmp = ra6963ReadData();

	if(color)
	{
		tmp |= (1 << (GLCD_FONT_WIDTH - 1 - (x % GLCD_FONT_WIDTH)));
	}
	else
	{
		tmp &= (1 << (GLCD_FONT_WIDTH - 1 - (x % GLCD_FONT_WIDTH)));
	}

	ra6963WriteData(tmp);
	ra6963WriteCommand(T6963_DATA_WRITE_AND_INCREMENT);

}

void ra6963Out(unsigned char outByte)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure Data Bus (D5-D7) as Output
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Configure Data Bus (D0-D4) as Output
	GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D3_Pin|LCD_D2_Pin|LCD_D1_Pin|LCD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Write to Data Bus
	HAL_GPIO_WritePin(GPIOA, LCD_D0_Pin, outByte & 0x01); // Write to D0
	HAL_GPIO_WritePin(GPIOA, LCD_D1_Pin, outByte & 0x02); // Write to D1
	HAL_GPIO_WritePin(GPIOA, LCD_D2_Pin, outByte & 0x04); // Write to D2
	HAL_GPIO_WritePin(GPIOA, LCD_D3_Pin, outByte & 0x08); // Write to D3
	HAL_GPIO_WritePin(GPIOA, LCD_D4_Pin, outByte & 0x10); // Write to D4
	HAL_GPIO_WritePin(GPIOC, LCD_D5_Pin, outByte & 0x20); // Write to D5
	HAL_GPIO_WritePin(GPIOC, LCD_D6_Pin, outByte & 0x40); // Write to D6
	HAL_GPIO_WritePin(GPIOC, LCD_D7_Pin, outByte & 0x80); // Write to D7
}


unsigned char ra6963In()
{
	unsigned char inByte;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure Data Bus (D5-D7) as Input
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Configure Data Bus (D0-D4) as Input
	GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D3_Pin|LCD_D2_Pin|LCD_D1_Pin
	                          |LCD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Read Data Bus
	inByte = HAL_GPIO_ReadPin(GPIOA, LCD_D0_Pin);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOA, LCD_D1_Pin) << 1);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOA, LCD_D2_Pin) << 2);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOA, LCD_D3_Pin) << 3);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOA, LCD_D4_Pin) << 4);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOC, LCD_D5_Pin) << 5);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOC, LCD_D6_Pin) << 6);
	inByte = inByte | ((char)HAL_GPIO_ReadPin(GPIOC, LCD_D7_Pin) << 7);

	return inByte;
}

/****************GRAPHICS***********************/
void ra6963Rectangle(unsigned char x, unsigned char y, unsigned char b, unsigned char a)
{
	unsigned char j;
	for (j = 0; j < a; j++)
	{
		ra6963SetPixel(x, y + j, color);
		ra6963SetPixel(x + b - 1, y + j, color);
	}
	for (j = 0; j < b; j++)
	{
		ra6963SetPixel(x + j, y, color);
		ra6963SetPixel(x + j, y + a - 1, color);
	}
}

void ra6963Circle(unsigned char cx, unsigned char cy ,unsigned char radius)
{
	int x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while(x >= y)
	{
		ra6963SetPixel(cx+x, cy+y, color);
		ra6963SetPixel(cx-x, cy+y, color);
		ra6963SetPixel(cx-x, cy-y, color);
		ra6963SetPixel(cx+x, cy-y, color);
		ra6963SetPixel(cx+y, cy+x, color);
		ra6963SetPixel(cx-y, cy+x, color);
		ra6963SetPixel(cx-y, cy-x, color);
		ra6963SetPixel(cx+y, cy-x, color);
		y++;
		radiusError += ychange;
		ychange += 2;
		if ( 2*radiusError + xchange > 0 )
		{
			x--;
			radiusError += xchange;
			xchange += 2;
		}
	}
}

void ra6963Line(int X1, int Y1,int X2,int Y2)
{
	int CurrentX, CurrentY, Xinc, Yinc,
    Dx, Dy, TwoDx, TwoDy,
	TwoDxAccumulatedError, TwoDyAccumulatedError;

	Dx = (X2-X1);
	Dy = (Y2-Y1);

	TwoDx = Dx + Dx;
	TwoDy = Dy + Dy;

	CurrentX = X1;
	CurrentY = Y1;

	Xinc = 1;
	Yinc = 1;

	if(Dx < 0)
	{
		Xinc = -1;
		Dx = -Dx;
		TwoDx = -TwoDx;
	}

	if (Dy < 0)
	{
		Yinc = -1;
		Dy = -Dy;
		TwoDy = -TwoDy;
	}

	ra6963SetPixel(X1,Y1, color);

	if ((Dx != 0) || (Dy != 0))
	{
		if (Dy <= Dx)
		{
			TwoDxAccumulatedError = 0;
			do
			{
				CurrentX += Xinc;
				TwoDxAccumulatedError += TwoDy;
				if(TwoDxAccumulatedError > Dx)
				{
					CurrentY += Yinc;
					TwoDxAccumulatedError -= TwoDx;
				}
				ra6963SetPixel(CurrentX,CurrentY, color);
			}while (CurrentX != X2);
		}
		else
		{
			TwoDyAccumulatedError = 0;
			do
			{
				CurrentY += Yinc;
				TwoDyAccumulatedError += TwoDx;
				if(TwoDyAccumulatedError>Dy)
				{
					CurrentX += Xinc;
					TwoDyAccumulatedError -= TwoDy;
				}
				ra6963SetPixel(CurrentX,CurrentY, color);
			}while (CurrentY != Y2);
		}
	}
}
