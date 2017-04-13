/*
 * initAPins.h
 *
 *  Created on: Mar 19, 2017
 *      Author: Olaf
 */

#ifndef TEMPSENSOR_H_
#define TEMPSENSOR_H_

#include "stm32f4xx_adc.h"
#include "math.h"

void initTempSensor(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit(); //reset previous settings to default.

//Initialize ADC
ADC_CommonInitTypeDef common_adc;
	common_adc.ADC_Prescaler = ADC_Prescaler_Div2;
	common_adc.ADC_Mode = ADC_Mode_Independent;
	common_adc.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	common_adc.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&common_adc);

ADC_InitTypeDef adc1_init;
	adc1_init.ADC_ContinuousConvMode = DISABLE;
	adc1_init.ADC_DataAlign = ADC_DataAlign_Right;
	adc1_init.ADC_Resolution = ADC_Resolution_12b;
	adc1_init.ADC_NbrOfConversion = 1;
	adc1_init.ADC_ScanConvMode = DISABLE;
	adc1_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc1_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_Init(ADC1, &adc1_init);


//Initialize internal temp sensor to ADC1
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC1,ENABLE);

	ADC_SoftwareStartConv(ADC1);
}



//Get temp voltage
uint16_t temp(){
	uint16_t reading =  ADC_GetConversionValue(ADC1);
	//3300mV, 0xfff=4095 for 2astmes12 bit resolution
	float tempC = ((reading*3300/0xfff-760)/2.5+25.0);
	//float nearest = (roundf(tempC * 10) / 10);
	return tempC; //casting to int..
}

#endif /* TEMPSENSOR_H_ */
