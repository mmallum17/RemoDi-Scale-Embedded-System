/*
 * esp8266.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#ifndef ESP8266_H_
#define ESP8266_H_

void serverConnect(/*char* serverName*/);
void serverClose();
void wifiConnect();
void esp8266Write(char* command, uint16_t readBytes, uint32_t timeout);
void serverWrite(char* command, uint32_t timeout);

#endif /* ESP8266_H_ */
