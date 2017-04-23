/*
 * sensors.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Olaf
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "bitset"
#include "usart.h" //serial send jaoks
#include "calcCRC.h"


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

		I2C_GenerateSTART(I2C1, ENABLE); //start condition
		I2C_Send7bitAddress(I2C1, 0x00, I2C_Direction_Transmitter); //default adr is zeros
		I2C_SendData(I2C1, 0x2e); //eeprom adr
		I2C_SendData(I2C1, LSB); //LSB
		I2C_SendData(I2C1, MSB); //MSB
		I2C_SendData(I2C1, this->adrPEC);  //CRC8 control value
		I2C_GenerateSTOP(I2C1, ENABLE);

		setEmissivity(0.95);
	}

	uint16_t readSensor(){
		//uint16_t ambientT; //read address 006h
		uint16_t objectT = 0; //read address 007h or 008h

		//Send I2C message to IR
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
		I2C_Send7bitAddress(I2C1, this->address, I2C_Direction_Transmitter);//transmitter = LSB 0
		I2C_SendData(I2C1, 0x07); //actual command to read Tobj1
		I2C_GenerateSTART(I2C1, ENABLE);
		I2C_Send7bitAddress(I2C1, this->address, I2C_Direction_Receiver); //receiver = LSB 1
		uint8_t LSB = I2C_ReceiveData(I2C1);
		uint8_t MSB = I2C_ReceiveData(I2C1);
		uint8_t PEC = I2C_ReceiveData(I2C1); //recieved CRC 8 control value
		I2C_GenerateSTOP(I2C1, ENABLE);

		uint8_t calcPEC = checkCRCTemp(LSB, MSB);

		if(PEC==calcPEC){ //if no data transmission errors
		//Convert temp data to C
		uint16_t data16b= (MSB<<8)|LSB;
		float tempK = data16b/50;
		float tempC = tempK - 273.15;

		objectT = tempC;
		} else {
			serialSend("PEC ei klapi");
		}

	return objectT;
	}

	void setEmissivity(float epsilon){//Melexis dedault emissivity is 1. Value can be between 0,1 to 1,0.
		uint16_t emissivity = round(65535*epsilon); //from Melexis datasheet

		//Write 0x0000 first
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
		I2C_Send7bitAddress(I2C1, this->address, I2C_Direction_Transmitter);
		I2C_SendData(I2C1, 0x24); //eeprom adr
		I2C_SendData(I2C1, 0x0);
		I2C_SendData(I2C1, 0x0);
		I2C_SendData(I2C1, calcCRCEmm(0x00,0x00)); //LSB
		I2C_GenerateSTOP(I2C1, ENABLE);

		//Write new emessivity value
		uint8_t MSB = ((emissivity >> 8) & 0xff);
		uint8_t LSB = ((emissivity >> 0) & 0xff);
		I2C_GenerateSTART(I2C1, ENABLE); //start condition
		I2C_Send7bitAddress(I2C1, this->address, I2C_Direction_Transmitter);
		I2C_SendData(I2C1, 0x24); //eeprom adr
		I2C_SendData(I2C1, LSB); //LSB
		I2C_SendData(I2C1, MSB); //MSB
		I2C_SendData(I2C1, calcCRCEmm(LSB,MSB));  //CRC8 control value
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

	uint8_t testCRC83(){
		bitset<32> inData((0x00<<24) | (0x22<<16) | (0x07<<8) | 0xc8); //initialize all bits to 1
	return calcCRC(inData);
	}

private:

	uint8_t address;
	uint8_t adrPEC;

	void calcCRCAdr(){//calc address PEC
		bitset<32> inData((0x00<<24) | (0x2e<<16) | (this->address<<8) | 0x00);
		this->adrPEC = calcCRC(inData);
	}

	uint8_t calcCRCEmm(uint8_t LSB, uint8_t MSB ){
		bitset<32> inData(((this->address)<<24) | (0x24<<16) | (LSB<<8) | MSB);
	return calcCRC(inData);
	}

	uint8_t checkCRCTemp(uint8_t LSB, uint8_t MSB ){
		bitset<40> inData(this->address);
		bitset<40> bitStringBS1((0x07<<24) | ((this->address+1)<<16) | (LSB<<8) | MSB);
		inData<<=32;
		inData |= bitStringBS1;
	return calcCRC(inData);
	}

};
#endif /* SENSORS_H_ */

