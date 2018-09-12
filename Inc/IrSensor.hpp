/*
 * IrSensor.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Olaf
 */

#ifndef IRSENSOR_HPP_
#define IRSENSOR_HPP_

/*
 * sensors.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Olaf
 */

#include "bitset"
#include "usart.h" //serial send jaoks
#include "math.h"
using namespace std;

static uint8_t mem = 0x2e;



class IrSensor{
	// tire light emissivity value might be around 0.9
public:

static void setSMBaddr(uint8_t address){
		uint8_t LSB = address;
		uint8_t MSB = 0x00;

		uint8_t buffer[] = {mem, 0, 0, 0x6f};
		HAL_I2C_Master_Transmit(&hi2c1, 0x00, buffer, 4, 500);
		HAL_Delay(10);
		HAL_WWDG_Refresh(&hwwdg);
		uint8_t buffer2[] = {mem, LSB, MSB, calcCRCAdr2(address)};
		HAL_I2C_Master_Transmit(&hi2c1, 0x00, buffer2, 4, 500);
		HAL_Delay(10);

		HAL_WWDG_Refresh(&hwwdg); //watchdog kicker
}

	IrSensor(uint8_t address) {
		//constructor to add new sensor, initialize sensor with new address
		this->address = address;
		uint8_t LSB = address;
		uint8_t MSB = 0x00;

		calcCRCAdr(); //generates PEC values for address change
		//uint8_t kontroll = checkCRCTemp(0xb9, 0x8b);
		/*
		uint8_t buffer[] = {mem, 0, 0, 0x6f};
		HAL_I2C_Master_Transmit(&hi2c1, 0x00, buffer, 4, 500);
		HAL_Delay(100);
		uint8_t buffer2[] = {mem, LSB, MSB, this->adrPEC};
		HAL_I2C_Master_Transmit(&hi2c1, 0x00, buffer2, 4, 500);
		HAL_Delay(100);
		*/
		setEmissivity(0.95);
		//HAL_Delay(1);
	}

	uint16_t* readSensor(uint8_t command){
		//Send I2C message to IR
		HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, this->address<<1, &command, 1, I2C_FIRST_FRAME);
		HAL_Delay(1);

		uint8_t buffer[] = {0, 0, 0};

		HAL_I2C_Master_Sequential_Receive_IT(&hi2c1, this->address<<1, buffer, 3, I2C_LAST_FRAME);
		HAL_Delay(1);
		uint8_t LSB = buffer[0];
		uint8_t MSB = buffer[1];
		uint8_t PEC = buffer[2];

		uint8_t calcPEC = checkCRC(command, LSB, MSB);

		HAL_WWDG_Refresh(&hwwdg); //watchdog kicker

		if(MSB>>7==1){
			Serial_PutString("\n Tempis oli error. ");
			return nullptr; //st errorit
		}

		if(PEC==calcPEC){ //if no data transmission errors
			data16b = (MSB<<8)|LSB;
			return &data16b;
		} else {
			Serial_PutString("PEC ei klapi");
			return nullptr;
		}

	}

	float readTemp(){
		float tempC = 0.0;
		uint8_t command = 0x07;

		uint16_t* dataP = readSensor(command);
//		if(readSensor(command)==nullptr) return 0;
		if(dataP==nullptr) return 0;
//		uint16_t data = *readSensor(command);
		uint16_t data = *dataP;
		float tempK = data/50.0;
		tempC = tempK - 273.15;

	return tempC;
	}

	float readAmbient(){
		float tempC = 0.0;
		uint8_t command = 0x06;

		uint16_t* dataP = readSensor(command);
//		if(readSensor(command)==nullptr) return 0;
		if(dataP==nullptr) return 0;
//		uint16_t data = *readSensor(command);
		uint16_t data = *dataP;
				float tempK = data/50.0;
		tempC = tempK - 273.15;
	return tempC;
	}

	float readEmiss(){
		float emissf = 0.0;
		uint8_t command = 0x24;

		if(readSensor(command)==nullptr) return 0;

		uint16_t data = *readSensor(command);
		emissf = data/65535.0;
	return emissf;
	}

	uint16_t readSMBaddr(){
		uint16_t adr = 0;

		HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, 0x00, &mem, 1, I2C_FIRST_FRAME);
		uint8_t buffer[] = {0, 0, 0};
		HAL_Delay(5);
		HAL_I2C_Master_Sequential_Receive_IT(&hi2c1, 0x00, buffer, 3, I2C_LAST_FRAME);
		HAL_Delay(5);

		HAL_WWDG_Refresh(&hwwdg); //watchdog kicker

		uint8_t LSB = buffer[0];
		uint8_t MSB = buffer[1];
		uint8_t PEC = buffer[2];

		uint8_t calcPEC = checkCRCadr(mem, LSB, MSB);
//
		if(PEC!=calcPEC) Serial_PutString("PEC ei klapi");

		adr = (MSB<<8)|LSB;

	return adr;
	}

	void setEmissivity(float epsilon){//Melexis dedault emissivity is 1. Value can be between 0,1 to 1,0.
		uint16_t emissivity = round(65535*epsilon); //from Melexis datasheet
		uint8_t memE = 0x24;
		uint8_t buffer[] = {memE, 0, 0, calcCRCEmm(0x00,0x00)};
		HAL_I2C_Master_Transmit(&hi2c1, this->address<<1, buffer, 4, 500);
		HAL_Delay(5);
		uint8_t MSB = ((emissivity >> 8) & 0xff);
		uint8_t LSB = ((emissivity >> 0) & 0xff);
		uint8_t buffer2[] = {mem, LSB, MSB, calcCRCEmm(LSB,MSB)};
		HAL_I2C_Master_Transmit(&hi2c1, this->address<<1, buffer2, 4, 500);
		HAL_Delay(5);

		HAL_WWDG_Refresh(&hwwdg); //watchdog kicker

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

	uint8_t checkCRCadr(uint8_t command, uint8_t LSB, uint8_t MSB ){
		bitset<40> inData(0x00<<1);
		bitset<40> bitStringBS1((command<<24) | (((0x00<<1) +1)<<16) | (LSB<<8) | MSB);

		inData<<=32;
		inData |= bitStringBS1;
	return calcCRC(inData);
	}


	static uint8_t calcCRCAdr2(uint8_t address){//calc address PEC
		bitset<32> inData((0x00<<24) | (0x2e<<16) | address<<8 | 0x00);
	return(calcCRC(inData));
	}



	static uint8_t calcCRC(bitset<32> inData){ //X8+X2+X1+1
		bitset<32> poly((0x83<<24) | (0x01<<23));//X8+X2+X1+1
		bitset<32> crcBits = inData;

		///
		uint8_t bindex = 0;
		uint8_t dS = inData.size();

		while (bindex<dS){
			if (!crcBits.test(dS-1)) crcBits <<= 1; // to make first bit equal 1
			else {
				crcBits ^= poly;
				crcBits <<= 1; //bcs after XOR, first will always be 0
			}
			bindex++;
		}
		crcBits>>=(dS-8);
	return crcBits.to_ulong();
	}

	static uint8_t calcCRC(bitset<40> inData){ //X8+X2+X1+1 //overloading for longer data
		bitset<40> poly((0x83<<24) | (0x01<<23));//X8+X2+X1+1 //int doesnt want to go over 32bit
		poly <<= 8; //shift poly from pos 32 to 40bi
		bitset<40> crcBits = inData;

		///
		uint8_t bindex = 0;
		uint8_t dS = inData.size();

		while (bindex<dS){
			if (!crcBits.test(dS-1)) crcBits <<= 1; // to make first bit equal 1
			else {
				crcBits ^= poly;
				crcBits <<= 1; //bcs after XOR, first will always be 0
			}
			bindex++;
		}
		crcBits>>=(dS-8);
	return crcBits.to_ulong();
	}
};




#endif /* IRSENSOR_HPP_ */
