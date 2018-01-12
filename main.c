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
	uint8_t state = 0;
	uint8_t spirit_on = 0;
	uint8_t data_received = 0;
	uint8_t irq_rx_data_ready = 0;
	SpiritIrqs irqStatus;
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
    bcm2835_gpio_hen(PIN18_IRQ);
	//bcm2835_gpio_set_eds(PIN18_IRQ);
	
	// HW pin 16 SPIRIT1 shutdown input toggle
	bcm2835_gpio_fsel(PIN16_SDN, BCM2835_GPIO_FSEL_OUTP);
	
	//reset transceiver via SDN
	bcm2835_gpio_write(PIN16_SDN, HIGH);
	delay(500);
	//bcm2835_gpio_write(PIN16_SDN, LOW);
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
	//wPiSPI_init_RF();
	printf("success!\n");
	delay(1000);
	
	//SpiritPktStackRequireAck(S_DISABLE);
	//SpiritCmdStrobeReady();

	//SpiritPktStackSetPayloadLength(PLOAD);
	//SpiritCmdStrobeFlushTxFifo();

	
	t = time(NULL);
	ts = localtime(&t);
	


	
	while(1)
	{
		if(state == 1) //rx
		{
			if(spirit_on == 0)
			{
				spirit_on = 1;
				bcm2835_gpio_write(PIN16_SDN, LOW);
				delay(1);	// SPIRIT MC startup		
				wPiSPI_init_RF();
				SpiritPktStackRequireAck(S_DISABLE);
				SpiritPktCommonRequireAck(S_DISABLE);
				SpiritCmdStrobeReady();
				SpiritPktBasicSetPayloadLength(PLOAD);
				SpiritIrqClearStatus();
				//SpiritTimerSetRxTimeoutMs(3000);
				SET_INFINITE_RX_TIMEOUT();
			}
			
			SpiritCmdStrobeFlushRxFifo();
			printf("receiving...\n");
			SpiritCmdStrobeRx();
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
						SpiritCmdStrobeRx();
					}
					
				}while(g_xStatus.MC_STATE!=MC_STATE_RX);	
			}	
			printf("Status (RX): %X\n", g_xStatus.MC_STATE);	
			/*
			do
			{
				data_received = spi_checkFIFO_IRQ_RF();
			}while(data_received == 0);
			*/

			do
			{
				irq_rx_data_ready = SpiritIrqCheckFlag(RX_DATA_READY);
				SpiritRefreshStatus();
				if(g_xStatus.MC_STATE != MC_STATE_RX)
				{
					do
					{ 
						SpiritCmdStrobeRx();
						SpiritRefreshStatus();		
					}while(g_xStatus.MC_STATE!=MC_STATE_RX);	
				}	
				//printf("Status: %X IRQ: %X\n", g_xStatus.MC_STATE, irq_rx_data_ready);
				bcm2835_delay(10);
			}while(irq_rx_data_ready == 0);
			
			if(irq_rx_data_ready == 1)
			{
				SpiritIrqClearStatus();
				irq_rx_data_ready = 0;
				//bcm2835_gpio_set_eds(PIN18_IRQ);
				printf("data received!\n");
				tmp_ui8 = SpiritLinearFifoReadNumElementsRxFifo();
				printf("No of elements: %d\n", tmp_ui8);
				SpiritSpiReadLinearFifo(tmp_ui8, vectcRxBuff);
				for(i=0;i<tmp_ui8;i++)
				{
					printf("%X ", vectcRxBuff[i]);
				}
				printf("\n");				
				// Flush the RX FIFO 
				SpiritCmdStrobeFlushRxFifo();
				data_received = 1;
				SpiritIrqClearStatus();
			}
			
			if(data_received == 1)
			{
				printf("data received!\n");
				//ready = 1;
				data_received = 0;
				state = 0;
				
				// turn off spirit
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				delay(1000);
				// go back to idle
				state = 0;
			}
		}
		
		if(state == 2) //tx
		{	
			if(spirit_on == 0)
			{
				spirit_on = 1;
				bcm2835_gpio_write(PIN16_SDN, LOW);
				delay(1);	// SPIRIT MC startup		
				wPiSPI_init_RF();
				SpiritPktStackRequireAck(S_DISABLE);
				SpiritPktCommonRequireAck(S_DISABLE);
				SpiritCmdStrobeReady();
				SpiritPktBasicSetPayloadLength(PLOAD);
				SpiritIrqClearStatus();
			}	

			SpiritCmdStrobeFlushTxFifo();
			SpiritCmdStrobeReady();
			SpiritRefreshStatus();
			
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
			// set the ready state 
				SpiritCmdStrobeSabort();
				do
				{
					SpiritRefreshStatus();
				}while(g_xStatus.MC_STATE!=MC_STATE_READY);
			}
			SpiritIrqClearStatus();
			SpiritCmdStrobeFlushTxFifo();
		
			if(RA)
			{
				counter++;
				if (counter >= 16){
					counter = 1;}
				
			rand_payload = (rand() % (28 + 1 - 8)) + 8;
			rand_fifo = (rand() % (28 + 1 - 8)) + 8;
			
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
			
				SpiritPktBasicSetPayloadLength(rand_payload);
				SpiritSpiWriteLinearFifo(rand_fifo, p_test);
			}
			else
			{
				SpiritSpiWriteLinearFifo(FIFO, test2);
			}
			
			SpiritCmdStrobeTx();
			delay(20);
			printf("send data...\n");
			
			
			SpiritCmdStrobeSabort();
			SpiritRefreshStatus();
			
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
				// set the ready state
				SpiritCmdStrobeSabort();
				do
				{
					SpiritRefreshStatus();
				}while(g_xStatus.MC_STATE!=MC_STATE_READY);
			}
			SpiritIrqClearStatus();
			delay(500);
			
			t = time(NULL);
			ts = localtime(&t);
			
			if((ts->tm_sec % 10) == 8)
			{
				// turn off spirit
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				delay(2000);
				// go back to idle
				state = 0;
			}
			
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

		if(state == 0)
		{
			t = time(NULL);
			ts = localtime(&t);
			printf("Time: %d\n", ts->tm_sec);
			//delay(1000);
			if((ts->tm_sec % 10) == 9)
			{
				state = 1;	// rx
				printf("Time: %d seconds. Start RX mode\n",ts->tm_sec);
			}
			else
			{
				if((ts->tm_sec % 10) == 11)
				{
					state = 2;	// tx
					printf("Time: %d seconds. Start TX mode\n",ts->tm_sec);
				}
				else
				{
					delay(1000);	// 1s delay when no RX or TX
				}
			}

		}
	} // while(1) closed

	printf("\nfinish\n");
    return 0;
}

