/*
 * sensors.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Olaf
 */

#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include "bitset"
#include "usart.h" //serial send jaoks
#include "calcCRC.h"
#include "timer.h"


class IrSensor{
	// tire light emissivity value might be around 0.9
public:
	IrSensor(uint8_t address) {
		//constructor to add new sensor, initialize sensor with new address
		this->address = address;
		uint8_t LSB = address;
		uint8_t MSB = 0x00;

		calcCRCAdr(); //generates PEC values for address change
		//uint8_t kontroll = checkCRCTemp(0xb9, 0x8b);

		//Kommitud, et iga restart ei kirjutaks EEPROM'i üle..

/*		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
		I2C_Send7bitAddress(I2C1, 0x00, I2C_Direction_Transmitter); //default adr is zeros
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, 0x2e); //eeprom adr
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, 0x00); //LSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, MSB); //MSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, 0x6f);  //CRC8 control value
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTOP(I2C1, ENABLE);

		delay_ms(10);

		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
		I2C_Send7bitAddress(I2C1, 0x00, I2C_Direction_Transmitter); //default adr is zeros
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, 0x2e); //eeprom adr
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, LSB); //LSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, MSB); //MSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, this->adrPEC);  //CRC8 control value
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTOP(I2C1, ENABLE);

		delay_ms(10);
		setEmissivity(0.95);*/
	}

	uint16_t* readSensor(uint8_t command){
		//Send I2C message to IR
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR); //event checking stuff defined in stm32f4xx_i2c.h
		I2C_Send7bitAddress(I2C1, this->address<<1, I2C_Direction_Transmitter);//transmitter = LSB 0
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, command); //actual command to read Tobj1
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTART(I2C1, ENABLE); //Repeated start
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
		I2C_Send7bitAddress(I2C1, this->address<<1, I2C_Direction_Receiver); //receiver = LSB 1
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR){
				if(I2C_GetFlagStatus(I2C1,I2C_FLAG_TIMEOUT)) serialSend("timed out!");
			}
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR){
				if(I2C_GetFlagStatus(I2C1,I2C_FLAG_TIMEOUT)) serialSend("timed out!");
			}
		uint8_t LSB = I2C_ReceiveData(I2C1);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
		uint8_t MSB = I2C_ReceiveData(I2C1);
		I2C_AcknowledgeConfig(I2C1, DISABLE); //NACK before STOP
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
		uint8_t PEC = I2C_ReceiveData(I2C1); //recieved CRC 8 control value
		I2C_GenerateSTOP(I2C1, ENABLE);

		uint8_t calcPEC = checkCRC(command, LSB, MSB);

		if(PEC==calcPEC){ //if no data transmission errors
			data16b = (MSB<<8)|LSB;
			return &data16b;
		} else {
			serialSend("PEC ei klapi");
			return nullptr;
		}
	}

	float readTemp(){
		float tempC = 0;
		uint8_t command = 0x07;

		uint16_t data = *readSensor(command);
		float tempK = data/50;
		tempC = tempK - 273.15;
	return tempC;
	}

	float readAmbient(){
		float tempC = 0;
		uint8_t command = 0x06;

		uint16_t data = *readSensor(command);
		float tempK = data/50;
		tempC = tempK - 273.15;
	return tempC;
	}

	float readEmiss(){
		float emissf = 0.0;
		uint8_t command = 0x24;

		uint16_t data = *readSensor(command);
		emissf = data/65535.0;
	return emissf;
	}

	uint16_t readSMBaddr(){
		uint16_t adr = 0;

		//Send I2C message to IR
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR); //event checking stuff defined in stm32f4xx_i2c.h
		I2C_Send7bitAddress(I2C1, 0x00, I2C_Direction_Transmitter);//transmitter = LSB 0
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, 0x2e);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTART(I2C1, ENABLE); //Repeated start
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
		I2C_Send7bitAddress(I2C1, 0x00, I2C_Direction_Receiver); //receiver = LSB 1
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR){
				if(I2C_GetFlagStatus(I2C1,I2C_FLAG_TIMEOUT)) serialSend("timed out!");
			}
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR){
				if(I2C_GetFlagStatus(I2C1,I2C_FLAG_TIMEOUT)) serialSend("timed out!");
			}
		uint8_t LSB = I2C_ReceiveData(I2C1);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
		uint8_t MSB = I2C_ReceiveData(I2C1);
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Next); //NACK before STOP
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
		uint8_t PEC = I2C_ReceiveData(I2C1); //recieved CRC 8 control value
		//I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current); //NACK before STOP
		I2C_GenerateSTOP(I2C1, ENABLE);

//		uint8_t calcPEC = checkCRC(0x2e, LSB, MSB);
//
//		if(PEC==calcPEC){ //if no data transmission errors
//
//		} else {
//			serialSend("PEC ei klapi");
//		}
		adr= (MSB<<8)|LSB;

	return adr;
	}

	void setEmissivity(float epsilon){//Melexis dedault emissivity is 1. Value can be between 0,1 to 1,0.
		uint16_t emissivity = round(65535*epsilon); //from Melexis datasheet

		//Write 0x0000 first
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
		I2C_Send7bitAddress(I2C1, this->address<<1, I2C_Direction_Transmitter);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, 0x24); //eeprom adr
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, 0x0);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, 0x0);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, calcCRCEmm(0x00,0x00)); //LSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTOP(I2C1, ENABLE);

		delay_ms(10);
		//Write new emessivity value
		uint8_t MSB = ((emissivity >> 8) & 0xff);
		uint8_t LSB = ((emissivity >> 0) & 0xff);
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
		I2C_Send7bitAddress(I2C1, this->address<<1, I2C_Direction_Transmitter);
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
		I2C_SendData(I2C1, 0x24); //eeprom adr
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, LSB); //LSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, MSB); //MSB
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_SendData(I2C1, calcCRCEmm(LSB,MSB));  //CRC8 control value
			while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);
		I2C_GenerateSTOP(I2C1, ENABLE);
	}

	uint8_t testCRC82(){ //X8+X2+X1+1 //inline CRC calculator
			bitset<32> poly((0x83<<24) | (0x01<<23));//X8+X2+X1+1

			bitset<32> crcBits((0x00<<24) | (0x22<<16) | (0x07<<8) | 0xc8); //initialize all bits to 1

			uint8_t bindex = 0;

			while (bindex<32){
				if (!crcBits.test(31)) crcBits <<= 1; // to make first bit equal 1
				else {
					crcBits ^= poly;
					crcBits <<= 1; //bcs after XOR, first will always be 0
				}
				bindex++;
			}

			crcBits>>=24;
			return crcBits.to_ulong();
	}


private:

	uint8_t address = 0x00; //default address is zero
	uint8_t adrPEC;
	uint16_t data16b = 0x00;

	void calcCRCAdr(){//calc address PEC
		bitset<32> inData((0x00<<24) | (0x2e<<16) | (this->address<<8) | 0x00);
		//bitset<32> inData((0x00<<24) | (0x2e<<16) | 0x00<<8 | (this->address));
		this->adrPEC = calcCRC(inData);
	}

	uint8_t calcCRCEmm(uint8_t LSB, uint8_t MSB ){
		bitset<32> inData(((this->address<<1)<<24) | (0x24<<16) | (LSB<<8) | MSB);
	return calcCRC(inData);
	}

	uint8_t checkCRC(uint8_t command, uint8_t LSB, uint8_t MSB ){
		bitset<40> inData(this->address<<1);
		bitset<40> bitStringBS1((command<<24) | (((this->address<<1) +1)<<16) | (LSB<<8) | MSB);
		inData<<=32;
		inData |= bitStringBS1;
	return calcCRC(inData);
	}

};
#endif /* IRSENSOR_H_ */

