/*
 * buttons.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#define	ZERO_FN			0
#define UNITS 			1
#define MENU			2
#define UP				3
#define DISPLAY_TARE	4
#define TARE			5
#define ENTER			6
#define RIGHT			7
#define LEFT			8
#define CALIBRATE		9
#define	DOWN			10
#define GROSS_NET		11
#define THREE			12
#define TWO				13
#define ONE				14
#define FIVE			15
#define SIX				16
#define FOUR			17
#define EIGHT			18
#define NINE			19
#define SEVEN			20
#define PERIOD			21
#define ZERO			22
#define DELETE			23

void buttonsInit();
uint8_t getKey();
void delay(uint32_t milliseconds);
uint8_t getCol(uint8_t row);
uint8_t readRow(uint8_t row);
void setAllCols();

#endif /* BUTTONS_H_ */
