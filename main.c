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

#define PLOAD 18
#define FIFO 18
#define RA 0

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
	uint8_t* p_test;
	uint8_t rand_payload;
	uint8_t rand_fifo;
	srand(time(NULL));
	
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

	uint8_t test2[18] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12};
	uint8_t* test2_p = &test2[0];
	
	uint8_t test10[10] = {0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A};
	uint8_t test11[11] = {0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B};
	uint8_t test12[12] = {0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C};
	uint8_t test13[13] = {0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D};
	uint8_t test14[14] = {0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E};
	uint8_t test15[15] = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
	uint8_t test16[16] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
	uint8_t test17[17] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	uint8_t test18[18] = {0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12};
	uint8_t test19[19] = {0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13};
	uint8_t test20[20] = {0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14};
	uint8_t test21[21] = {0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15};
	uint8_t test22[22] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16};
	uint8_t test23[23] = {0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17};
	uint8_t test24[24] = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
	//tmp_ui8 = *(test2_p);
	//tmp_ui8 = ((tmp_ui8 & 0xC0) + 0x40) + (10<<1) + 1;
	//*test2_p = tmp_ui8;
	
	printf("initialize RF module...\n");
	wPiSPI_init_RF();
	printf("success!\n");
	delay(1000);
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();

	//SpiritPktStackSetPayloadLength(PLOAD);
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
			
			counter++;
			if (counter >= 16)
				counter = 1;
				
			rand_payload = (rand() % (28 + 1 - 8)) + 8;
			rand_fifo = (rand() % (28 + 1 - 8)) + 8;
				
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
				
			
			switch(counter){
				case 1: p_test = test10; break;
				case 2: p_test = test11; break;
				case 3: p_test = test12; break;
				case 4: p_test = test13; break;
				case 5: p_test = test14; break;
				case 6: p_test = test15; break;
				case 7: p_test = test16; break;
				case 8: p_test = test17; break;
				case 9: p_test = test18; break;
				case 10: p_test = test19; break;
				case 11: p_test = test20; break;
				case 12: p_test = test21; break;
				case 13: p_test = test22; break;
				case 14: p_test = test23; break;
				case 15: p_test = test24; break;
				default: p_test = test2; break;				
			}
			
			if(RA)
			{
				SpiritPktBasicSetPayloadLength(PLOAD);
				SpiritSpiWriteLinearFifo(FIFO, p_test);
			}
			else
			{
				SpiritPktBasicSetPayloadLength(PLOAD);
				SpiritSpiWriteLinearFifo(FIFO, test2);
			}
			
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
