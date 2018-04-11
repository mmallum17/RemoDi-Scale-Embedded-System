/*
 * esp8266.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#include "stm32l1xx_hal.h"
#include "ra6963.h"
#include "esp8266.h"
#include <string.h>

extern UART_HandleTypeDef huart4;

void wifiConnect()
{
    esp8266Write("AT+CWQAP", 80, 500);
	//esp8266Write("AT+CWJAP_CUR=\"SCOTTCAMPUS\",\"mavericks\"", 76, 10000);
	//esp8266Write("AT+CWJAP_CUR=\"Hold The Door\",\"sset957578peekaboo64\"", 87 , 10000);
	//esp8266Write("AT+CWJAP_CUR=\"Connectify-Scale\",\"vezopyc4\"", 80, 10000);
	esp8266Write("AT+CWJAP_CUR=\"Collin\",\"CantTouchThis\"", 75 , 10000);
}

void serverConnect(/*char* serverName*/)
{
	esp8266Write("AT+CIPSTART=\"TCP\",\"18.221.30.192\",4000", 80, 500);
	esp8266Write("AT+CIPMODE=1", 80, 500);
	esp8266Write("AT+CIPSEND", 80, 500);
}

void serverClose()
{
	HAL_Delay(250);
	serverWrite("+++", 500);
	HAL_Delay(1000);

	esp8266Write("AT+CIPMODE=0", 80, 500);
	esp8266Write("AT+CIPCLOSE", 80, 500);
}

void esp8266Write(char* command, uint16_t readBytes, uint32_t timeout)
{
	char rcvBuffer[80] = "";
	uint16_t length = strlen(command);
	HAL_UART_Transmit(&huart4, (uint8_t*)command, length, 50);
	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 50);
	HAL_UART_Receive(&huart4, (uint8_t*)rcvBuffer, readBytes, timeout);
	// ra6963ClearText();
	// ra6963WriteString(rcvBuffer);
}

void serverWrite(char* command, uint32_t timeout)
{
	uint16_t length = strlen(command);
	HAL_UART_Transmit(&huart4, (uint8_t*)command, length, 50);
}
