#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "SPIRIT_Config.h"
#include "globals.h"
#include "bcm2835.h"
#include "SPI_interface.h"
//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

#define PIN18_IRQ RPI_GPIO_P1_18
#define PIN16_SDN RPI_GPIO_P1_16

#define PLOAD 14
#define FIFO 14

uint8_t vectcTxBuff[FIFO_BUFF]={};

int main()
{
	printf("Hello world!\n");
	time_t t;
	struct tm * ts;
	uint8_t ready = 0;
    uint64_t counter = 0;
    int level = LOW;
	uint8_t tmp[4];
	uint8_t rx = 0;
	uint8_t tx = 0;
	uint8_t data_received = 0;
	uint8_t vectcRxBuff[FIFO_BUFF];
	int i;
	uint8_t t_sec;
	uint8_t t_min;
	uint8_t t_hour;
	uint8_t tmp_ui8;
	
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
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
	//delay(10);
	
	// HW pin 18 rising edge detect (RX ready)
	bcm2835_gpio_fsel(PIN18_IRQ, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN18_IRQ, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_ren(PIN18_IRQ);
	bcm2835_gpio_set_eds(PIN18_IRQ);
	
	// HW pin 16 SPIRIT1 shutdown input toggle
	bcm2835_gpio_fsel(PIN16_SDN, BCM2835_GPIO_FSEL_OUTP);
	
	//reset transceiver via SDN
	bcm2835_gpio_write(PIN16_SDN, HIGH);
	delay(1000);
	bcm2835_gpio_write(PIN16_SDN, LOW);
	//SpiritCmdStrobeSres();

	uint8_t test2[18] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
	uint8_t* test2_p = &test2[0];
	
	//tmp_ui8 = *(test2_p);
	//tmp_ui8 = ((tmp_ui8 & 0xC0) + 0x40) + (10<<1) + 1;
	//*test2_p = tmp_ui8;
	
	printf("initialize RF module...\n");
	wPiSPI_init_RF();
	printf("success!\n");
	delay(1000);
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();
	SpiritPktBasicSetPayloadLength(PLOAD);
	SpiritPktStackSetPayloadLength(PLOAD);
	SpiritCmdStrobeFlushTxFifo();
	//SpiritCmdStrobeFlushRxFifo();
	//SpiritRefreshStatus();
	//SET_INFINITE_RX_TIMEOUT();
	/*
	SpiritQiSetSqiThreshold(SQI_TH_0);
	SpiritQiSqiCheck(S_ENABLE);
	SpiritTimerSetRxTimeoutMs(1000);
	SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);
	*/
	//SpiritPktCommonSetCrcMode(PKT_CRC_MODE_16BITS_1);
	
	/*
	bcm2835_gpio_set_eds(PIN18_IRQ);
	printf("set RX mode...\n");
	SpiritRefreshStatus();
	printf("State: %x\n", g_xStatus.MC_STATE);
	SpiritSpiReadRegisters(0x52,1,tmp);
	printf("protocol: %X\n", tmp[0]);
	SpiritSpiReadRegisters(0x50,1,tmp);
	printf("calibration: %X\n", tmp[0]);
	*/
	
	t = time(NULL);
	ts = localtime(&t);
	
	/*
	printf("%s", asctime(ts));
	printf("year = %d\n",ts->tm_year+1900);
	printf("month = %d\n",ts->tm_mon+1);
	printf("day of the month = %d\n",ts->tm_mday);
	printf("day since january = %d\n",ts->tm_yday);
	printf("hour = %d\n",ts->tm_hour);
	printf("min = %d\n",ts->tm_min);
	printf("sec = %d\n",ts->tm_sec);
	printf("wday = %d\n",ts->tm_wday);
	*/
/*	
	do
	{ 
		SpiritSpiCommandStrobes(COMMAND_LOCKRX);
		SpiritRefreshStatus();
		//printf("State: %x\n", g_xStatus.MC_STATE);
		if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
		{
			//delay(1);
			//SpiritCmdStrobeSres();
		}
		//delay(100);
	}while(g_xStatus.MC_STATE!=MC_STATE_LOCK);	
	printf("State(0x0F): %x, LOCK\n", g_xStatus.MC_STATE);
	
	
	do
	{ 
		SpiritSpiCommandStrobes(COMMAND_RX);
		SpiritRefreshStatus();
		//printf("State: %x\n", g_xStatus.MC_STATE);
		if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
		{
			//delay(1);
			//SpiritCmdStrobeSres();
		}
		//delay(100);
	}while(g_xStatus.MC_STATE!=MC_STATE_RX);	
	printf("State(0x33): %x, RX\n", g_xStatus.MC_STATE);
	
*/
	
	while(1)
	{
		if(rx == 1)
		{
			SpiritRefreshStatus();
			if(g_xStatus.MC_STATE != MC_STATE_RX)
			{
				do
				{ 
					SpiritSpiCommandStrobes(COMMAND_RX);
					SpiritRefreshStatus();
					//printf("State: %x\n", g_xStatus.MC_STATE);
					if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
					{
						//SpiritCmdStrobeSres();
						//wPiSPI_init_RF();
						
						SpiritCmdStrobeRx();
						//SpiritBaseConfiguration();
						//SpiritVcoCalibration();
						//delay(1);
						//SpiritCmdStrobeSres();
						//SpiritSpiCommandStrobes(COMMAND_READY);
					}
					
				}while(g_xStatus.MC_STATE!=MC_STATE_RX);	

			}
			//printf("State(0x33): %x, RX\n", g_xStatus.MC_STATE);
/*
			do
			{
				//SpiritCmdStrobeRx();
				//SpiritRefreshStatus();
				data_received = spi_checkFIFO_IRQ_RF();
				//printf("State(0x33): %x\n", g_xStatus.MC_STATE);
				/*
				if (bcm2835_gpio_eds(PIN18_IRQ))
				{
					CircularBuffer_In(0xAA, &FIFO_IRQ_RF);
					//data_received = spi_checkFIFO_IRQ_RF();
					ready = 1;
					bcm2835_gpio_set_eds(PIN18_IRQ);
					printf("event!\n");
				}		
				data_received = spi_checkFIFO_IRQ_RF();
				*/
				//delay(1);
					//printf("receiving...\n");
				
			//while(data_received == 0);
			
			if(data_received == 1)
			{
				rx = 0;
				ready = 1;
				data_received = 0;
			}
		}
		
		if(tx == 1)
		{
			SpiritCmdStrobeFlushTxFifo();
			SpiritRefreshStatus();

			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
 				SpiritCmdStrobeSabort();
 			do
 			{ 

 				SpiritRefreshStatus();
 				//printf("State: %x\n", g_xStatus.MC_STATE);
				if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
				{
					SpiritCmdStrobeSabort();
				}

 				delay(300);
 			}while(g_xStatus.MC_STATE!=MC_STATE_READY);	
			}
			
			SpiritIrqClearStatus();
			SpiritSpiWriteLinearFifo(FIFO, test2_p);
			SpiritCmdStrobeTx();
			printf("send data...\n");
			delay(10);
			
			SpiritRefreshStatus();
			if(g_xStatus.MC_STATE != MC_STATE_TX)
			{
				do
				{ 
					SpiritSpiCommandStrobes(COMMAND_TX);
					SpiritRefreshStatus();
					//printf("State: %x\n", g_xStatus.MC_STATE);	
				}while(g_xStatus.MC_STATE!=MC_STATE_TX);
				SpiritRefreshStatus();
				printf("\tState (TX): %X\n", g_xStatus.MC_STATE);
				
			}	
			delay(10);
			SpiritCmdStrobeSabort();
			SpiritRefreshStatus();
					if(g_xStatus.MC_STATE != MC_STATE_READY)
				{
			/* set the ready state */
					SpiritCmdStrobeSabort();
					do
					{
						SpiritRefreshStatus();
					}while(g_xStatus.MC_STATE!=MC_STATE_READY);

				}

		/* clear the Irq */
				SpiritIrqClearStatus();
			delay(10);
			//delay(500);
			SpiritRefreshStatus();
			printf("State (TX): %X\n", g_xStatus.MC_STATE);
			
		}
		
		
		
		if(ready == 1)
		{
			SpiritRefreshStatus();
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
				// set the ready state 
				//SpiritCmdStrobeSabort();
				do
				{ 
					//SpiritCmdStrobeSabort();
					SpiritSpiCommandStrobes(COMMAND_READY);
					SpiritRefreshStatus();
					//printf("State: %x\n", g_xStatus.MC_STATE);
					if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
					{
						//delay(1);
						//SpiritCmdStrobeSres();
					}
					//delay(100);
				}while(g_xStatus.MC_STATE!=MC_STATE_READY);	

			}
			printf("State: ready\n");

			ready = 0;
		}

		//tmp = (uint8_t) SpiritDirectRfGetRxMode();
		
		//SpiritCmdStrobeRx();
		//spi_checkFIFO_IRQ_RF();

        //delay(10);
        /*
        counter++;
        if((counter % 100000) == 0)
        {
			SpiritRefreshStatus();
			printf("State: %x\n", g_xStatus.MC_STATE);
			counter = 0;
		}
		*/
		if(rx == 0 && tx == 0)
		{
			SpiritRefreshStatus();
			printf("Time: %d State: %X\n", ts->tm_sec, g_xStatus.MC_STATE);
			if(0) //if(g_xStatus.MC_STATE == 0x0)
			{
				SpiritSpiReadRegisters(0x52,1,tmp);
				printf("protocol: %X\n", tmp[0]);
			}
			t = time(NULL);
			ts = localtime(&t);
			delay(1000);
			if(ts->tm_sec >= 70)
			{
				tx = 0;
				rx = 1;
				printf("Time: %d seconds. Start RX mode\n",ts->tm_sec);
			}
			if(ts->tm_sec >= 10)
			{
				rx = 0;
				tx = 1;
				printf("Time: %d seconds. Start TX mode\n",ts->tm_sec);
			}

		}
	//data_received = spi_checkFIFO_IRQ_RF();
	SpiritIrqClearStatus();
	SpiritRefreshStatus();
	printf("State: %x\n", g_xStatus.MC_STATE);
	/*
	//cRxData = SpiritLinearFifoReadNumElementsRxFifo();
				//cRxData = 96;
				//Read the RX FIFO 
				SpiritSpiReadLinearFifo(96, vectcRxBuff);
				
				for(i=0;i<FIFO_BUFF;i++)
				{
					printf("%X ", vectcRxBuff[i]);
					//vectcRxBuff[i] = 0;
				}
				printf("\n");
	//delay(500);
	*/
	}

	printf("\nfinish\n");
    return 0;
    
   
}
