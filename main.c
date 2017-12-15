#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include "SPIRIT_Config.h"
#include "globals.h"
#include "bcm2835.h"
#include "SPI_interface.h"
//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

#define PIN RPI_GPIO_P1_18

uint8_t vectcTxBuff[FIFO_BUFF]={};

int main()
{
	printf("Hello world!\n");

	uint8_t ready = 1;
    int counter = 0;
    int level = LOW;
	uint8_t tmp = 0;
	
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
	
	// HW pin 18 rising edge detect (RX ready)
	bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_ren(PIN);
	bcm2835_gpio_set_eds(PIN);
	
	
	//SpiritCmdStrobeSres();

	uint8_t test2[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
	uint8_t* test2_p = &test2[0];
	
	printf("initialize RF module...\n");
	wPiSPI_init_RF();
	printf("success!\n");
	delay(2000);
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();
	SpiritPktBasicSetPayloadLength(PAYLOAD);
	SpiritCmdStrobeFlushTxFifo();
	SpiritRefreshStatus();
	
	bcm2835_gpio_set_eds(PIN);
	
	while(1)
	{
		SpiritRefreshStatus();
		if(ready==1)
		{
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
				// set the ready state 
				//SpiritCmdStrobeSabort();
				do
				{ 
					SpiritCmdStrobeSabort();
					SpiritRefreshStatus();
					printf("State: %x\n", g_xStatus.MC_STATE);
					if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
					{
						delay(1);
						SpiritCmdStrobeSres();
					}
					delay(100);
				}while(g_xStatus.MC_STATE!=MC_STATE_READY);	

			}

			ready = 0;
		}
		
		if (bcm2835_gpio_eds(PIN))
        {
            CircularBuffer_In(0xAA, &FIFO_IRQ_RF);
            ready = 1;
            bcm2835_gpio_set_eds(PIN);
            printf("event!\n");
        }

		tmp = (uint8_t) SpiritDirectRfGetRxMode();
		SpiritCmdStrobeRx();
		spi_checkFIFO_IRQ_RF();

        delay(10);
	}
	
	while(0)
	{
		printf("...\n");
		delay(1000);
	}

	printf("\nfinish\n");
    return 0;
    
   
}
