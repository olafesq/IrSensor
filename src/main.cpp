//Blinky MCU'le

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <string>
#include "math.h"
//#include "stdio.h"
using namespace std;

//My classes
#include <IrSensor.h>
#include <usart.h>
#include "timer.h"
#include <tempSensor.h>
#include "i2cMelexis.h"

//Before anything SystemInit from startup assembly source file is called to set up clock
// Float doesn't work with reduced runtime lib!
void initDPins(){
	GPIO_InitTypeDef GPIO_InitDef;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //Activates clock D
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT; //GPIO_Mode_IN,	GPIO_Mode_OUT, GPIO_Mode_AF GPIO Alternate function Mode, GPIO_Mode_AN GPIO Analog Mode
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP; //push-pull,  GPIO_OType_OD open drain
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP , GPIO_PuPd_DOWN
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz; // GPIO_Speed_2MHz = 0x00, GPIO_Speed_25MHz = 0x01, GPIO_Speed_50MHz = 0x02, GPIO_Speed_100MHz = 0x03
    //Initialize pins
    GPIO_Init(GPIOD, &GPIO_InitDef);

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    //Initialize pins
    GPIO_Init(GPIOA, &GPIO_InitDef);
}

int main(void) {
	systickInit(1000); //initialize system tick interrupt at every 1ms

	initDPins();
	initTempSensor();
	init_i2c1IR();

	init_USART2(9600); // initialize USART1 @ 9600 baud
	serialSend("Init complete! Hello World!\r\n"); // just send a message to indicate that it works

	IrSensor sensor1(0x10); //create new sensor object, initialize with address


    while (1) {

        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
            GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14);
            USART_puts(USART2,to_string(minutes())+ "button\n"); //naljakas, aga c++ compiler command line to see juurde lisada  -D_GLIBCXX_USE_C99
            serialSend("Toa temperatuur on: "+ to_string(temp()));
            serialSend(" C.\n");

            //string s = sensor1.testCRC8();
			//serialSend("CRC8 kontroll: " + s);
			//serialSend("CRC8 kontroll: " + to_string(sensor1.testCRC83()));
            serialSend("Aadress on " + to_string(sensor1.readSMBaddr()));

            serialSend("Temp on " + to_string(sensor1.readSensor()));

			delay_ms(500);
        } else {
            GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14);
        }

    	GPIO_SetBits(GPIOD, GPIO_Pin_13);
    	delay_ms(500);
    	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    	delay_ms(500);
    }
}

