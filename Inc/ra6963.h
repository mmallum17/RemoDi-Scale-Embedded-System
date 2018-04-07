/*
 * ra6963.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Marcus
 */

#ifndef RA6963_H_
#define RA6963_H_

/* Constant Definitions */
#define GLCD_NUMBER_OF_LINES 				128
#define GLCD_PIXELS_PER_LINE 				240
#define GLCD_FONT_WIDTH						8

#define GLCD_GRAPHIC_AREA 					(GLCD_PIXELS_PER_LINE / GLCD_FONT_WIDTH)
#define GLCD_TEXT_AREA 						(GLCD_PIXELS_PER_LINE / GLCD_FONT_WIDTH)
#define GLCD_GRAPHIC_SIZE 					(GLCD_GRAPHIC_AREA * GLCD_NUMBER_OF_LINES)
#define GLCD_TEXT_SIZE 						(GLCD_TEXT_AREA * (GLCD_NUMBER_OF_LINES / 8))

#define GLCD_TEXT_HOME 						0
#define GLCD_GRAPHIC_HOME 					(GLCD_TEXT_HOME + GLCD_TEXT_SIZE)
#define GLCD_OFFSET_REGISTER 				2

#define GLCD_EXTERNAL_CG_HOME				(GLCD_OFFSET_REGISTER << 11)


#define T6963_SET_CURSOR_POINTER			0x21
#define T6963_SET_OFFSET_REGISTER			0x22
#define T6963_SET_ADDRESS_POINTER			0x24

#define T6963_SET_TEXT_HOME_ADDRESS			0x40
#define T6963_SET_TEXT_AREA					0x41
#define T6963_SET_GRAPHIC_HOME_ADDRESS		0x42
#define T6963_SET_GRAPHIC_AREA				0x43

#define T6963_MODE_SET						0x80


#define T6963_DISPLAY_MODE					0x90
#define T6963_CURSOR_BLINK_ON				0x01
#define T6963_CURSOR_DISPLAY_ON				0x02
#define T6963_TEXT_DISPLAY_ON				0x04
#define T6963_GRAPHIC_DISPLAY_ON			0x08

//#define T6963_
//#define T6963_

#define T6963_CURSOR_PATTERN_SELECT			0xA0
#define T6963_CURSOR_1_LINE					0x00
#define T6963_CURSOR_2_LINE					0x01
#define T6963_CURSOR_3_LINE					0x02
#define T6963_CURSOR_4_LINE					0x03
#define T6963_CURSOR_5_LINE					0x04
#define T6963_CURSOR_6_LINE					0x05
#define T6963_CURSOR_7_LINE					0x06
#define T6963_CURSOR_8_LINE					0x07

#define T6963_SET_DATA_AUTO_WRITE			0xB0
#define T6963_SET_DATA_AUTO_READ			0xB1
#define T6963_AUTO_RESET					0xB2

#define T6963_DATA_WRITE_AND_INCREMENT		0xC0
#define T6963_DATA_READ_AND_INCREMENT		0xC1
#define T6963_DATA_WRITE_AND_DECREMENT		0xC2
#define T6963_DATA_READ_AND_DECREMENT		0xC3
#define T6963_DATA_WRITE_AND_NONVARIALBE	0xC4
#define T6963_DATA_READ_AND_NONVARIABLE		0xC5

#define T6963_SCREEN_PEEK					0xE0
#define T6963_SCREEN_COPY					0xE8

/*Public Function Prototypes*/
void ra6963Init();
void ra6963ClearText();
void ra6963ClearCG();
void ra6963ClearGraphic();
unsigned char ra6963CheckStatus();
void ra6963WriteString(char* str);
void ra6963WriteChar(char ch);
void ra6963WriteData(unsigned char writeData);
void ra6963WriteCommand(unsigned char writeCommand);
void ra6963TextGoTo(int x, int y);
void ra6963Rectangle(unsigned char x, unsigned char y, unsigned char b, unsigned char a);
void ra6963Circle(unsigned char cx, unsigned char cy ,unsigned char radius);
void ra6963Line(int X1, int Y1,int X2,int Y2);



#endif /* RA6963_H_ */
