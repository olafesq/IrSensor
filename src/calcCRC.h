/*
 * calcCRC.h
 * Calculates PEC vaulues. Set your poly all the way to the left. And ensure that inData fits to bitset size, if not, increase it.
 *  Created on: Apr 4, 2017
 *      Author: Olavi
 */

#ifndef CALCCRC_H_
#define CALCCRC_H_


uint8_t calcCRC(bitset<32> inData){ //X8+X2+X1+1
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

uint8_t calcCRC(bitset<40> inData){ //X8+X2+X1+1 //overloading for longer data
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

#endif /* CALCCRC_H_ */
