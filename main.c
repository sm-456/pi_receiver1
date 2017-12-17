#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include "SPIRIT_Config.h"
#include "globals.h"
//#include "bcm2835.h"
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
	
	static const char *device = "/dev/spidev0.0";
	
	static uint8_t mode = 0;
	static uint8_t bits = 8;
	static uint32_t speed = 500000;
	static uint16_t delay;
	int ret;

	/* Device oeffen */
	if ((fd = open(device, O_RDWR)) < 0)
	  {
	  perror("Fehler Open Device");
	  exit(1);
	  }
	/* Mode setzen */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret < 0)
	  {
	  perror("Fehler Set SPI-Modus");
	  exit(1);
	  }

	/* Mode abfragen */
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret < 0)
	  {
	  perror("Fehler Get SPI-Modus");
	  exit(1);
	  }

	/* Wortlaenge setzen */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret < 0)
	  {
	  perror("Fehler Set Wortlaenge");
	  exit(1);
	  }

	/* Wortlaenge abfragen */
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret < 0)
	  {
	  perror("Fehler Get Wortlaenge");
	  exit(1);
	  }

	/* Datenrate setzen */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret < 0)
	  {
	  perror("Fehler Set Speed");
	  exit(1);
	  }
	   
	/* Datenrate abfragen */
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret < 0)
	  {
	  perror("Fehler Get Speed");
	  exit(1);
	  }

	/* Kontrollausgabe */
	printf("SPI-Device.....: %s\n", device);
	printf("SPI-Mode.......: %d\n", mode);
	printf("Wortlaenge.....: %d\n", bits);
	printf("Geschwindigkeit: %d Hz (%d kHz)\n", speed, speed/1000);
	
	delay(2000);
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
