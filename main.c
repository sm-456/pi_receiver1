#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include "SPIRIT_Config.h"
#include "globals.h"
#include "bcm2835.h"
#include "SPI_interface.h"

#define PIN RPI_GPIO_P1_18
//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

//static const int CHANNEL = 0;   // wPi chip select

int main()
{
	printf("Hello world!\n");

    int fd,i;
    int counter = 0;
    int level = LOW;
    uint8_t tmp_ui8;
    uint16_t tmp_ui16;
    uint16_t* pointer_ui16;
    unsigned char buffer[26]={"abcdefghijklmnopqrstuvwxyz"};

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
	delay(10);
	
	bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    //  with a pullup
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_UP);
    // And a low detect enable
    bcm2835_gpio_len(PIN);
	
	SpiritCmdStrobeSres();
/*
	uint8_t test[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
	uint8_t* pointer_ui8 = &test[0];
	tmp_ui8 = *(pointer_ui8);
	tmp_ui8 = ((tmp_ui8 & 0xC0) + 0x40) + (10<<1) + 1;
	*pointer_ui8 = tmp_ui8;
	uint8_t* test_p = pointer_ui8;
*/	
	uint8_t test2[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
	uint8_t* test2_p = &test2[0];
	
/*
	uint16_t test[10] = {474,869,9765,543,765,890,532,434,642,8643};
	uint16_t* test_p = &test[0];
	pointer_ui16 = test_p;
	tmp_ui16 = *(pointer_ui16);
	tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (10<<1) + 1;
	*pointer_ui16 = tmp_ui16;
*/
	printf("initialize RF module...\n");
	wPiSPI_init_RF();
	printf("success!\n");
	delay(2000);
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();
	SpiritPktBasicSetPayloadLength(20+4);
	SpiritCmdStrobeFlushTxFifo();
	SpiritRefreshStatus();

	int tst = 2; // 0 = SPI, 1 = GPIO, 2 = transmission, 3 = receive
	
	while(counter<=15)
	{
		 
		if(tst==0)
		{
//-----------------SPI test---------------------------------
			// MISO and MOSI pins need to be shorted
			//do{
			//wiringPiSPIDataRW(CHANNEL, buffer, 26);
			//}while(buffer[0]==0);
			printf("Buffer: %s\n", buffer);
			for (i=0;i<26;i++)
			{+
				printf("%x  ", buffer[i]);
			}
			printf("\n");
		}
		
		if(tst==1)
		{
//------------------GPIO test---------------------------------		
		if(1)
		{
			level = !level;
			SpiritRefreshStatus();	
			SpiritGpioSetLevel(SPIRIT_GPIO_3,level);
			counter = 0;
		}
		SpiritRefreshStatus();
		printf("GPIO: %x\tState: %x\n", bcm2835_gpio_lev(PIN), g_xStatus.MC_STATE);
		delay(500);

		}
		
		if(tst==2)
		{
//-------------------Transmission test-------------------------
		
		
		//SpiritRefreshStatus();
		
		if(g_xStatus.MC_STATE != MC_STATE_READY)
		{
			//set the ready state 
			//SpiritCmdStrobeSabort();
			do
			{ 
				SpiritCmdStrobeSabort();
				SpiritRefreshStatus();
				printf("State: %x\n", g_xStatus.MC_STATE);
				if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
					SpiritCmdStrobeSabort();
					//SpiritCmdStrobeSres();
					//delay(1);
				delay(100);
			}while(g_xStatus.MC_STATE!=MC_STATE_READY);	
		}

		//printf("2\n");
			/* clear the Irq */
		SpiritIrqClearStatus();

			/* fit the TX FIFO */
		SpiritCmdStrobeFlushTxFifo();
		SpiritSpiWriteLinearFifo(20+4, test2_p);
		SpiritCmdStrobeTx();
		printf("send data...\n");
		//delay(50);
		SpiritCmdStrobeSabort();
		SpiritRefreshStatus();
		//printf("3\n");
		//printf("GPIO: %x\n", digitalRead(5));
		//delay(50);
		counter = 0;
		}
		
		if(tst==3)
		{
			SpiritRefreshStatus();
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
				/* set the ready state */				
				//SpiritCmdStrobeSabort();
				do
				{
					SpiritCmdStrobeSabort();
					SpiritRefreshStatus();
					printf("\tState: %x\n", g_xStatus.MC_STATE);
					if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
						SpiritCmdStrobeSabort();
					//SpiritCmdStrobeSres();
					//delay(1);
					delay(50);
				}while(g_xStatus.MC_STATE!=MC_STATE_READY);

			}

			uint8_t tmp = (uint8_t) SpiritDirectRfGetRxMode();
			printf("\nrx: %d\n", tmp);
			delay(500);
			//put the RF in Rx Mode
			SpiritCmdStrobeRx();
			counter = 0;
		}
		counter = counter + 1;
	}
	
	while(0)
	{
		printf("...\n");
		delay(1000);
	}
	//digitalWrite(1,0);	// RF module power off
	printf("\nfinish\n");
    return 0;
    
// 6C CA AB 24 28 81 BF DD FF AB F5 4C C4 04 0E 11 71 69 89 2F F4 28 F3 1B C5 B9 FD 84 61 19 E0 CB C3 95 8F 54 D7 2E CE A0 D6 F4 0F 6A A5 A4 A9 45 3B 79 B7 99 D9 0E E0 4C 70 7A 61 07 44 27 B7 5C 39 52 73 E1 60 5E B6    
}
