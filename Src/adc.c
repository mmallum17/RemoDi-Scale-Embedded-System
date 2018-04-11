/*
 * adc.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Marcus
 */

#include "AD7190.h"
#include <math.h>
#include "adc.h"

unsigned char adcInit(uint8_t chip)
{
	unsigned char result;
	adcChipSelect(chip);
	result = AD7190_Init();
	adcChipDeselect(chip);
	return result;
}

void adcRangeSetup(unsigned char polarity, unsigned char range, uint8_t chip)
{
	adcChipSelect(chip);
	AD7190_RangeSetup(polarity, range);
	adcChipDeselect(chip);
}

void adcCalibrate(unsigned char mode, unsigned char channel, uint8_t chip)
{
	adcChipSelect(chip);
	AD7190_Calibrate(mode, channel);
	adcChipDeselect(chip);
}

void adcCalibrateAll()
{
	adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M, 1);
	adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M, 1);
	adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AIN4M, 1);
	adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AIN4M, 1);
	adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M, 2);
	adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M, 2);
	adcCalibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AIN4M, 2);
	adcCalibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AIN4M, 2);
}

void adcChannelSelect(unsigned short channel, uint8_t chip)
{
	adcChipSelect(chip);
	AD7190_ChannelSelect(channel);
	adcChipDeselect(chip);
}

unsigned long adcSingleConversion(uint8_t chip)
{
	unsigned long buffer;
	adcChipSelect(chip);
	buffer = AD7190_SingleConversion();
	adcChipDeselect(chip);
	return buffer;
}

unsigned long adcContinuousReadAvg(unsigned char sampleNumber, uint8_t chip)
{
	unsigned long buffer;
	adcChipSelect(chip);
	buffer = AD7190_ContinuousReadAvg(50);
	adcChipDeselect(chip);
	return buffer;
}

void adcChipSelect(uint8_t chip)
{
	switch(chip)
	{
	case 1:
		ADC_CS1_LOW();
		break;
	case 2:
		ADC_CS2_LOW();
		break;
	}
}

void adcChipDeselect(uint8_t chip)
{
	switch(chip)
	{
	case 1:
		ADC_CS1_HIGH();
		break;
	case 2:
		ADC_CS2_HIGH();
		break;
	}
}

void floatToString(float value, char* floatString, int afterpoint)
{
	uint8_t negative = 0;
	uint32_t intValue;
	float tmpFrac;
	uint32_t intFrac;
	if(value < 0)
	{
		value *= -1;
		negative = 1;
	}
	intValue = value;
	tmpFrac = value - intValue;
	intFrac = trunc(tmpFrac * pow(10, afterpoint));

	if(negative)
	{
		sprintf(floatString, "-%lu.%.*lu", intValue, afterpoint, intFrac);
	}
	else
	{
		sprintf(floatString, "%02lu.%.*lu", intValue, afterpoint, intFrac);
	}
}
