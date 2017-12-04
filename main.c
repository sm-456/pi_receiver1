#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "SPIRIT_Config.h"
#include "globals.h"

//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

//static const int CHANNEL = 0;   // wPi chip select

int main()
{
/* 	
	// wiringPi Setup
    if (wiringPiSetup() == -1)
        printf("wiringPi setup OK\n");
    else
		printf("setup failed\n");
*/	
	uint8_t fifo_adress[CIRCULARBUFFER_SIZE*20] = {0};

    int fd,i;
    int counter = 0;
    int level = HIGH;
    uint16_t tmp_ui16;
    uint16_t* pointer_ui16;
    unsigned char buffer[26]={"abcdefghijklmnopqrstuvwxyz"};
	
    printf("Hello world!\n");
	
    fd = wiringPiSPISetup(CHANNEL, SPEED);
	wiringPiSetup();
	printf("SPI Setup: %d\n", fd);
	
	SGpioInit gpioIRQ={
		SPIRIT_GPIO_3,
		SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
		SPIRIT_GPIO_DIG_OUT_IRQ
	};
	SpiritGpioInit(&gpioIRQ);
	
	wiringPiSPIDataRW(CHANNEL, buffer, 26);
	
	printf("Buffer: %s\n", buffer);
	
	for (i=0;i<26;i++)
	{
		printf("%x  ", buffer[i]);
	}
    
	printf("\nFinish\n");
	uint32_t adress = (uint32_t) &fifo_adress[0];
	//printf("FIFO: %x\n", adress);
/*	
	uint8_t test[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
	uint8_t* test_p = &test[0];
	
	pointer_ui16 = (uint16_t)* test;
	tmp_ui16 = *(pointer_ui16);
	tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (10<<1) + 1;
	*pointer_ui16 = tmp_ui16;
*/	
	uint16_t test[10] = {474,869,9765,543,765,890,532,434,642,8643};
	uint16_t* test_p = &test[0];
	
	pointer_ui16 = test_p;
	tmp_ui16 = *(pointer_ui16);
	tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (10<<1) + 1;
	*pointer_ui16 = tmp_ui16;
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();
	SpiritPktBasicSetPayloadLength(20+4);
	SpiritCmdStrobeFlushTxFifo();
	SpiritRefreshStatus();
	//printf("1\n");
	SpiritGpioSetLevel(SPIRIT_GPIO_3, level);
	SpiritGpioSetLevel(SPIRIT_GPIO_3,HIGH);
	printf("GPIO: %x\n", digitalRead(5));
	
	int tst = 1; // 1 = GPIO, 2 = transmission
	while(1)
	{
		if(tst==1)
		{
//------------------GPIO test---------------------------------		
		if(1)
		{
			if(level==HIGH)
				level = LOW;
			else
				level = HIGH;	
			SpiritGpioSetLevel(SPIRIT_GPIO_3,level);
			counter = 0;
		}
		printf("GPIO: %x\n", digitalRead(5));
		delay(500);
		}
		
		if(tst==2)
		{
//-------------------Transmission test-------------------------
		if(g_xStatus.MC_STATE != MC_STATE_READY)
		{
			//set the ready state 
			SpiritCmdStrobeSabort();
			do
			{ 
				SpiritRefreshStatus();
				printf("State: %x\n", g_xStatus.MC_STATE);
				delay(500);
			}while(g_xStatus.MC_STATE!=MC_STATE_READY);	
		}

		//printf("2\n");
			/* clear the Irq */
		SpiritIrqClearStatus();

			/* fit the TX FIFO */
		SpiritCmdStrobeFlushTxFifo();
		SpiritSpiWriteLinearFifo(10+4, test_p);
		SpiritCmdStrobeTx();
		printf("send data...\n");
		delay(2000);
		SpiritCmdStrobeSabort();
		SpiritRefreshStatus();
		//printf("3\n");
		//printf("GPIO: %x\n", digitalRead(5));
		delay(500);
		}
		
		counter++;
	}
	
	while(0)
	{
		printf("...\n");
		delay(1000);
	}
    return 0;
    
// 6C CA AB 24 28 81 BF DD FF AB F5 4C C4 04 0E 11 71 69 89 2F F4 28 F3 1B C5 B9 FD 84 61 19 E0 CB C3 95 8F 54 D7 2E CE A0 D6 F4 0F 6A A5 A4 A9 45 3B 79 B7 99 D9 0E E0 4C 70 7A 61 07 44 27 B7 5C 39 52 73 E1 60 5E B6    
}
