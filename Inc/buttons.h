/*
 * buttons.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#define	ZERO_FN			11
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
#define THREE			23
#define TWO				22
#define ONE				21
#define FIVE			20
#define SIX				19
#define FOUR			18
#define EIGHT			17
#define NINE			16
#define SEVEN			15
#define PERIOD			14
#define ZERO			13
#define DELETE			12

void buttonsInit();
uint8_t getKey();
void delay(uint32_t milliseconds);
uint8_t getCol(uint8_t row);
uint8_t readRow(uint8_t row);
void setAllCols();

#endif /* BUTTONS_H_ */
