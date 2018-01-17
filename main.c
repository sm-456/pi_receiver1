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

#define STATE_IDLE 0
#define STATE_RX 1
#define STATE_TX 2
#define STATE_FILE 3
#define STATE_REGISTRATION 4

#define PLOAD 18
#define FIFO 18
#define RA 0
#define SEND_INTERVAL 5			// time between transmissions, minutes
#define MEASURE_INTERVAL 30		// time between measurements, seconds
#define MEASURE_VALUES 10		// number of values per transmission
#define RX_DATA_BUFFER 5		// number of datasets saved between file operations

struct device_ID
{
	uint16_t device; 		// ID of sensor device
	uint8_t measurement;	// ID of measured data
};

struct dataframe_ID
{
	uint16_t data_ID;		// ID of data package, continuous numbering
	uint8_t quantity;		// number of transmitted values (2 Bytes each)
	uint8_t cont;			// 1 = not last package, transmission continued
};

uint8_t vectcTxBuff[FIFO_BUFF]={};

int main()
{
/*==============================================================================
                                VARIABLES
 =============================================================================*/
	int i,j;
	uint8_t tmp_ui8;
	uint16_t received_bytes = 0;
	uint16_t tmp_sensor_id = 0;
	uint16_t tmp_dataframe_id = 0;
	uint16_t parameter = 0;
	uint32_t time_temp = 0;
	FILE * fp;
	
	// time variables
	time_t t;
	time_t t_rx;
	struct tm * ts;
	struct tm * rx_time;
	uint8_t t_sec;
	uint8_t t_min;
	uint8_t t_hour;	
	uint8_t state = 0;
	
/*==============================================================================
                                ARRAYS
 =============================================================================*/	
	uint8_t rx_buffer[FIFO_BUFF];
	
	// oldest dataset at 0
	uint16_t temperature[RX_DATA_BUFFER][MEASURE_VALUES] = {0};
	uint16_t pressure[RX_DATA_BUFFER][MEASURE_VALUES] = {0};
	uint16_t humidity[RX_DATA_BUFFER][MEASURE_VALUES] = {0};
	uint16_t moisture[RX_DATA_BUFFER][MEASURE_VALUES] = {0};
	
	// oldest dataset at 0
	uint32_t rx_time_array[RX_DATA_BUFFER] = {0};	// 32 bit UNIX timestamp
	
	uint8_t received_packets_buffer[2][MEASURE_VALUES*2+4] = {0};  // save first packages until transmission complete
	uint8_t time_table[MEASURE_VALUES][6] = {0};		  // array for time in csv data table
	char string[100];

/*==============================================================================
                                FLAGS
 =============================================================================*/	
	uint8_t go_ready_state = 0;
	uint8_t spirit_on = 0;
	uint8_t irq_rx_data_ready = 0;
	uint8_t data_received = 0;
	uint8_t more_data = 0;
	uint8_t data_ok = 0;
	uint8_t data_write = 0;
		
/*==============================================================================
                                COUNTER
 =============================================================================*/
 
     uint64_t counter = 0;				// generic counter
	 uint8_t stored_datasets_counter = 0;		// number of received complete datasets	
	 uint8_t received_packets_counter = 0;		// number of received packages of single set (3 or 4)
	
	 //uint16_t *p_value_array;

//==============================================================================	
	
	srand(time(NULL));		// RNG init
	
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
	
	// HW pin 18 rising edge detect (RX go_ready_state)
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

	
	t = time(NULL);
	ts = localtime(&t);
	
	char filename[25];
	char* filep = &filename[0];
	//sprintf(filename, "%s%d%d%d%s", 'data', ts->tm_year, (ts->tm_mon)+1, ts->tm_mday, '.csv');
	filep = strcat(filep, "./data/data00.csv");

	for (i = 0; i < 100; i++) {
		filename[11] = i/10 + '0';
		filename[12] = i%10 + '0';
		if( access( filename, F_OK ) != -1 ) {
			// file exists
		} else {
			fp = fopen(filep, "w+");
			break;
		}
	}
	
	fprintf(fp, "hour,minute,second,temperature,pressure,humidity\n");
	printf("file created!\n");
	fclose(fp);
	
	
	while(1)
	{
		if(state == STATE_RX) //rx
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

			// wait for data
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
				//bcm2835_delay(30);
				// save time
				t_rx = time(NULL);
				//rx_time = localtime(&t);
				
				SpiritIrqClearStatus();
				irq_rx_data_ready = 0;
				
				printf("data received!\n");
				received_bytes = SpiritLinearFifoReadNumElementsRxFifo();
				printf("No of elements: %d\n", received_bytes);
				SpiritSpiReadLinearFifo(received_bytes, rx_buffer);
				
				SpiritCmdStrobeFlushRxFifo();
						
				more_data = rx_buffer[2]&0x01;	// extract last bit
				
				for(i=0;i<received_bytes;i++)
				{
					printf("%X ", rx_buffer[i]);
				}
				printf("\n");
				
				if(received_bytes == (MEASURE_VALUES*2 + 4))
				{
					data_ok = 1;
					/*
					stored_datasets_counter++;
					if(stored_datasets_counter == RX_DATA_BUFFER)
					{
						data_write = 1;
						stored_datasets_counter = 0;
					}
					*/
				} 
				
				if(more_data == 0)	// all packages received
				{
					
					if(data_ok == 1)
					{
						data_ok = 0;
						for(j=0;j<2;j++)
						{			
							// sort data
							for(i=0;i<received_bytes;i=i+2)
							{
								tmp_ui8 = rx_buffer[i];
								rx_buffer[i] = rx_buffer[i+1];
								rx_buffer[i+1] = tmp_ui8;
							}
							
							// extract preamble data (temporarily)
							tmp_sensor_id = (rx_buffer[0]<<8)|rx_buffer[1];
							tmp_dataframe_id = (rx_buffer[2]<<8)|rx_buffer[3];
							parameter = tmp_sensor_id & 0x001F;
							
							// write data to value array
							for(i=4;i<received_bytes;i=i+2)
							{
								switch(parameter)
								{
									case 1: 
										temperature[stored_datasets_counter][(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
									case 2: 
										pressure[stored_datasets_counter][(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
									case 3: 
										humidity[stored_datasets_counter][(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
									case 4:
										moisture[stored_datasets_counter][(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
									default: break;
								}
							}
							
							// prepare new array from buffer
							if(j>0)
								received_packets_counter--;
							memcpy(&(received_packets_buffer[j][0]),rx_buffer,MEASURE_VALUES*2+4);
							
						}
					}
					
					//====================================================
					
					/*
					fprintf(fp, "%d,%d,%d,%s\n", ts->tm_hour, ts->tm_min, ts->tm_sec, rx_buffer);	
							
					SpiritCmdStrobeFlushRxFifo();
					data_received = 1;
					SpiritIrqClearStatus();
					
					counter++;
					if(counter == 5)
					{
						counter = 0;
						fclose(fp);
						bcm2835_delay(10);
						printf("write to file...\n");
						fopen(filep, "a+");
					}	
					*/	
					
					//====================================================
					
					rx_time_array[stored_datasets_counter] = (uint32_t)t_rx;	// save UNIX timestamp
								
					stored_datasets_counter++;
					if(stored_datasets_counter == RX_DATA_BUFFER)
					{
						data_write = 1;
						stored_datasets_counter = 0;
					}
					
					// all data stored?	
					// turn off spirit
					spirit_on = 0;
					bcm2835_gpio_write(PIN16_SDN, HIGH);
					delay(500);
					// go back to idle
					
					if(data_write == 0)
						state = STATE_IDLE;
					else
						state = STATE_FILE;
					
				}
				else  // more packages to be received
				{
					received_packets_counter++;
					// copy data to buffer
					memcpy(rx_buffer,&(received_packets_buffer[received_packets_counter-1][0]),2*MEASURE_VALUES+4);		
					// return to rx mode
				}
			}
			
		}
		
		if(state == STATE_FILE)	// write to file
		{
			fp = fopen(filep, "a+");
			// dataset loop (buffered messages, each contains all 4 measurement values)
			for(i=0;i<RX_DATA_BUFFER;i++)
			{
				t = rx_time_array[i];	
				
				// value loop (X values per dataset)
				for(j=MEASURE_VALUES-1;j<=0;j--)
				{
					ts = localtime(&t);
					
					time_table[j][0] = ts->tm_hour; // hour
					time_table[j][1] = ts->tm_min; // min
					time_table[j][2] = ts->tm_sec; // sec
					time_table[j][3] = ts->tm_mday; // day
					time_table[j][4] = ts->tm_mon+1; // month
					time_table[j][5] = ts->tm_year+1900; // year
					
					t = t - MEASURE_INTERVAL; // go back x seconds to get time of previous value
				}
				
				// write data to file
				for(j=0;j<MEASURE_VALUES;j++)
				{
					fprintf(fp, "%d,%d,%d,&d,&d,&d\n", time_table[j][0], time_table[j][1], time_table[j][2], temperature[i][j], pressure[i][j], humidity[i][j]);
				}
			}
			
			fclose(fp);
			stored_datasets_counter = 0;
			state = STATE_IDLE;
			
			
			
			
		}
		
		if(state == STATE_TX) //tx
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
		
			SpiritSpiWriteLinearFifo(FIFO, test2);
			
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
				state = STATE_IDLE;
			}
			
		}
		
		
		
		if(go_ready_state == 1)
		{
			SpiritRefreshStatus();
			if(g_xStatus.MC_STATE != MC_STATE_READY)
			{
				// set the go_ready_state state 
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
			printf("State: go_ready_state\n");

			go_ready_state = 0;
		}

		if(state == STATE_IDLE)
		{
			t = time(NULL);
			ts = localtime(&t);
			printf("Time: %d\n", ts->tm_sec);
			//fprintf(stdout, "%u\n", (unsigned)time(NULL));
			//delay(1000);
			if((ts->tm_sec % 10) == 9)
			{
				state = STATE_RX;	// rx
				printf("Time: %d seconds. Start RX mode\n",ts->tm_sec);
			}
			else
			{
				if((ts->tm_sec % 10) == 11)
				{
					state = STATE_TX;	// tx
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

