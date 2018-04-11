/*
 * adc.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#ifndef ADC_H_
#define ADC_H_

unsigned char adcInit(uint8_t chip);
void adcChipSelect(uint8_t chip);
void adcChipDeselect(uint8_t chip);
void floatToString(float value, char* floatString, int afterpoint);
void adcRangeSetup(unsigned char polarity, unsigned char range, uint8_t chip);
void adcChannelSelect(unsigned short channel, uint8_t chip);
void adcCalibrate(unsigned char mode, unsigned char channel, uint8_t chip);
void adcCalibrateAll();
unsigned long adcSingleConversion(uint8_t chip);
unsigned long adcContinuousReadAvg(unsigned char sampleNumber, uint8_t chip);

#endif /* ADC_H_ */
