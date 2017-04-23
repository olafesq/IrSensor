/*
 * i2cMelexis.h
 *
 *  Created on: Mar 23, 2017
 *      Author: Olaf
 */

#ifndef I2CMELEXIS_H_
#define I2CMELEXIS_H_

#include "stm32f4xx_i2c.h"

void init_i2c1IR(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //clock for i2c
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//clock for i2c pins

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); //SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); //SDA

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);


	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable ;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 10000; //in Hz, min 10kHz, max 100kHz
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; //should not be used this case
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 1;//should not be used this case
	I2C_Init(I2C1, &I2C_InitStruct);

	//I2C_CalculatePEC(I2C1, ENABLE); //CTC-8? Wrong type, stm32f4 is fixed CRC32

	I2C_Cmd(I2C1, ENABLE);

	//init_stm32CRC8(); //start built-in PEC calculator, BUT has fixed polynom of CRC32 only..
}




#endif /* I2CMELEXIS_H_ */
