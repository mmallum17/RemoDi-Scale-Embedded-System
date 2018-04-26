/*
 * buttons.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#define	ZERO_FN			23
#define UNITS 			22
#define MENU			21
#define UP				20
#define DISPLAY_TARE	19
#define TARE			18
#define ENTER			17
#define RIGHT			16
#define LEFT			15
#define CALIBRATE		14
#define	DOWN			13
#define GROSS_NET		12
#define THREE			2
#define TWO				1
#define ONE				0
#define FIVE			5
#define SIX				3
#define FOUR			4
#define EIGHT			8
#define NINE			6
#define SEVEN			7
#define PERIOD			11
#define ZERO			10
#define DELETE			9
/*#define	ZERO_FN			11
#define UNITS 			10
#define MENU			9
#define UP				8
#define DISPLAY_TARE	7
#define TARE			6
#define ENTER			5
#define RIGHT			4
#define LEFT			3
#define CALIBRATE		2
#define	DOWN			1
#define GROSS_NET		0
#define THREE			14
#define TWO				13
#define ONE				12
#define FIVE			17
#define SIX				15
#define FOUR			16
#define EIGHT			20
#define NINE			18
#define SEVEN			19
#define PERIOD			23
#define ZERO			22
#define DELETE			21*/

void buttonsInit();
uint8_t getKey();
void delay(uint32_t milliseconds);
uint8_t getCol(uint8_t row);
uint8_t readRow(uint8_t row);
void setAllCols();

#endif /* BUTTONS_H_ */
