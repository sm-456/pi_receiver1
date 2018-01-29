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

//#define OUTPUT

#define PIN18_IRQ RPI_GPIO_P1_18
#define PIN16_SDN RPI_GPIO_P1_16
#define PIN15_BUTTON RPI_GPIO_P1_15

#define STATE_IDLE 0
#define STATE_RX 1
#define STATE_TX 2
#define STATE_FILE 3
#define STATE_REGISTRATION 4

#define PLOAD 18
#define FIFO 18
#define RA 0
#define SEND_INTERVAL 30		// time between transmissions, seconds
#define MEASURE_INTERVAL 3		// time between measurements, seconds
#define MEASURE_VALUES 10		// number of values per transmission
#define MOISTURE_VALUES 1		// values in moisture package
#define RX_DATA_BUFFER 2		// number of datasets saved between file operations
#define TIME_SLOT_DIFF 60		// offset between time slots

#define MAX_DEVICES 16
#define OFFSET_MINUTES 1
#define OFFSET_SECONDS 10

void create_file(uint8_t device_pointer, char* string, char* filenames);
int save_to_file(uint8_t sensor_to_save, time_t t, char* filenames, uint16_t* temperature, uint16_t* pressure, uint16_t* humidity, uint16_t* moisture);
int send_data(uint8_t* data_pointer, uint8_t bytes);
uint8_t receive_data(void);


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

uint8_t tmp_array[16] = {0};
uint8_t spirit_on = 0;
uint8_t rx_buffer[FIFO_BUFF];
uint8_t moisture_received = 0;

int main()
{
/*==============================================================================
                                VARIABLES
 =============================================================================*/
	int i,j,k;
	uint8_t tmp_ui8;
	uint16_t tmp_ui16;
	uint16_t tmp2_ui16;
	uint32_t tmp_ui32;
	uint16_t received_bytes = 0;
	uint16_t tmp_sensor_id = 0;
	uint16_t tmp_dataframe_id = 0;
	uint16_t parameter = 0;
	uint32_t time_temp = 0;
	FILE * fp;
	uint8_t time_slot_offset_minutes = 0;
	uint8_t time_slot_offest_seconds = 0;
	uint8_t day = 0;
	uint8_t sensor_to_save = 0;
	
	// time variables
	time_t t;
	time_t t_rx;
	struct tm * ts;
	struct tm * rx_time;
	uint8_t t_sec;
	uint8_t t_min;
	uint8_t t_hour;	
	uint8_t state = 0;
	uint32_t t_int = 0;
/*==============================================================================
                                ARRAYS
 =============================================================================*/	

	
	// oldest dataset at 0
	uint16_t temperature[MEASURE_VALUES] = {0};
	uint16_t pressure[MEASURE_VALUES] = {0};
	uint16_t humidity[MEASURE_VALUES] = {0};
	//uint16_t moisture[RX_DATA_BUFFER][MOISTURE_VALUES] = {0};
	uint16_t moisture = 0;
	
	// oldest dataset at 0
	//uint32_t rx_time;
	uint32_t rx_time_array[RX_DATA_BUFFER] = {0};	// 32 bit UNIX timestamp
	
	uint8_t received_packets_buffer[3][MEASURE_VALUES*2+4] = {0};  // save first packages until transmission complete
	uint16_t time_table[MEASURE_VALUES][6] = {0};		  // array for time in csv data table
	uint16_t device_storage[MAX_DEVICES];
	uint32_t send_times[MAX_DEVICES];
	char filenames[MAX_DEVICES][30];
	char date[8] = {0};
	uint8_t test[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

/*==============================================================================
                                FLAGS
 =============================================================================*/	
	uint8_t go_ready_state = 0;

	uint8_t irq_rx_data_ready = 0;
	uint8_t data_received = 0;
	uint8_t more_data = 0;
	uint8_t data_ok = 0;
	uint8_t data_write = 0;
	uint8_t moisture_data = 0;
	uint8_t first_message_received = 0;

	uint8_t send_time = 0;
	uint8_t exit_loop = 0;
/*==============================================================================
                                COUNTER
 =============================================================================*/
 
     uint16_t sec_counter = 0;				// generic counter
	 uint8_t stored_datasets_counter = 0;		// number of received complete datasets	
	 uint8_t received_packets_counter = 0;		// number of received packages of single set (3 or 4)
	
	 uint8_t device_pointer = 0;	// point to empty field in device storage 
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
	
	// HW pin 15 rising edge detect for button press
	bcm2835_gpio_fsel(PIN15_BUTTON, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN15_BUTTON, BCM2835_GPIO_PUD_DOWN);
    //bcm2835_gpio_ren(PIN15_BUTTON);
    //bcm2835_gpio_set_eds(PIN15_BUTTON);
	
	
	// HW pin 16 SPIRIT1 shutdown input toggle
	bcm2835_gpio_fsel(PIN16_SDN, BCM2835_GPIO_FSEL_OUTP);
	
	//reset transceiver via SDN
	bcm2835_gpio_write(PIN16_SDN, HIGH);
	delay(500);
	//bcm2835_gpio_write(PIN16_SDN, LOW);
	//SpiritCmdStrobeSres();
	
	t = time(NULL);
	ts = localtime(&t);
	
	/*
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
	
	fprintf(fp, "time,temperature,pressure,humidity,moisture\n");
	printf("file created!\n");
	fclose(fp);
	*/
	
	while(1)
	{
		
/*==============================================================================
                                RX
 =============================================================================*/
 
		if(state == STATE_RX) //rx
		{
			
			do
			{
				received_bytes = receive_data();
				more_data = rx_buffer[2]&0x01;
		
				if (more_data == 1)
				{
					for(k=0;k<(MEASURE_VALUES*2+4);k++)
					{
						received_packets_buffer[received_packets_counter][k] = rx_buffer[k];
						rx_buffer[k] = 0;
					}	
					received_packets_counter++;
					
					if(received_packets_counter == 3)
						moisture_data = 1;
				}
			}while(more_data==1);
	
			if(received_packets_counter==2 || received_packets_counter==3)
			{	
				// data ok
				t_rx = time(NULL);
				for(j=0;j<(3+moisture_data);j++)
				{
#ifdef OUTPUT										
					for(k=0;k<(MEASURE_VALUES*2+4);k++)
					{
						printf("%X ", rx_buffer[k]);
					}
					printf("\n");
					for(k=0;k<(MEASURE_VALUES*2+4);k++)
					{
						printf("%X ", received_packets_buffer[0][k]);
					}
					printf("\n");
					for(k=0;k<(MEASURE_VALUES*2+4);k++)
					{
						printf("%X ", received_packets_buffer[1][k]);
					}
					printf("\n");
#endif
					
					if(j==0 && moisture_data==1)
					{
						received_bytes = 2*MOISTURE_VALUES+4;
					}
					else
					{
						received_bytes = 2*MEASURE_VALUES+4;
					}	
					
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
					parameter = (rx_buffer[1]&0x07);
					//printf("parameter: %X\n", parameter);
					
					// write data to value array
					for(i=4;i<received_bytes;i=i+2)
					{
						// combine 2 bytes to ui16 and save to correct array
						switch(parameter)
						{								
							case 1: 
								temperature[(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
							case 2: 
								pressure[(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
							case 3: 
								humidity[(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1]; break;
							case 4:
								//moisture[stored_datasets_counter][(i/2)-2] = (rx_buffer[i]<<8)|rx_buffer[i+1];
								moisture = (rx_buffer[i]<<8)|rx_buffer[i+1];
								moisture_received = 1;
								for(k=0;k<received_bytes;k++)
								{
										printf("%d ", rx_buffer[k]);
								}
								printf("\n"); break;
							default: break;
						}
					}						
					
					// prepare new array from buffer						
					if(j<(2+moisture_data) && received_packets_buffer[j][0] != 0x0)
					{						
						//memcpy(&(received_packets_buffer[j][0]),rx_buffer,MEASURE_VALUES*2+4);
						//printf("j = %d\tcopy array...\n",j);
						for(k=0;k<(MEASURE_VALUES*2+4);k++)
						{
							rx_buffer[k] = received_packets_buffer[j][k];
						}
						//received_packets_counter--;	
					}

				}	
				received_packets_counter = 0;
				moisture_data = 0;
				//rx_time = (uint32_t) t_rx;
				
				for(j=0;j<MAX_DEVICES;j++)
				{
					if(tmp_sensor_id == device_storage[j])
					{
						sensor_to_save = j;
						break;
					}
				}
				
				save_to_file(sensor_to_save, t_rx, &(filenames[0][0]), temperature, pressure, humidity, &moisture);
				
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				bcm2835_delay(200);
				state = STATE_IDLE;
			}
			else
			{
				//data not ok
				//send request for new send attempt
				state = STATE_RX;
			}
			
		}

/*==============================================================================
                                FILE
 =============================================================================*/

		if(state == STATE_FILE)	// write to file
		{
			// moved to user function
		}
		
/*==============================================================================
                                TX
 =============================================================================*/
 
		if(state == STATE_TX) //tx
		{	
			send_data(test,10);
			
			spirit_on = 0;
			bcm2835_gpio_write(PIN16_SDN, HIGH);
			delay(1000);
			state = STATE_IDLE;
			
		}
		
/*==============================================================================
                                REGISTRATION
 =============================================================================*/		

		if(state == STATE_REGISTRATION)
		{
			printf("Waiting for device...\n");
			bcm2835_delay(1000);

			received_bytes = receive_data();

			t_int = (int) time(NULL);
			//rx_time = localtime(&t);
			
			tmp_sensor_id = (rx_buffer[1]<<8)|rx_buffer[0]; 
			tmp_ui8 = 0;
			// check if device is already registered
			for(i=0;i<device_pointer;i++)
			{
				if(device_storage[i] == tmp_sensor_id)
					tmp_ui8 = 1;
			}
			if(tmp_ui8 == 0)
			{
				// device not yet registered
				device_storage[device_pointer] = tmp_sensor_id;	// save id
				send_times[device_pointer] = t_int;	// save send time
				
				create_file(device_pointer,date, &(filenames[0][0]));  // sensor00_20180101_00.csv
				
				device_pointer++;
				send_time = 1;		
			}
			if (send_time == 1)
			{
				send_time = 0;
				//time_message(t_rx, time_array);	//create 32 bit time value
				tmp_ui32 = t_int&0xFF000000;
				tmp_array[0] = (uint8_t) tmp_ui32;
				tmp_ui32 = t_int&0x00FF0000;
				tmp_array[1] = (uint8_t) tmp_ui32;
				tmp_ui32 = t_int&0x0000FF00;
				tmp_array[2] = (uint8_t) tmp_ui32;
				tmp_ui32 = t_int&0x000000FF;
				tmp_array[3] = (uint8_t) tmp_ui32;
				
				if(device_pointer == 1)
				{
					tmp_ui16 = 60;	// first slave, default wait time 60 sec
				}
				else
				{
					tmp_ui32 = t_int - send_times[0];
					tmp_ui32 = tmp_ui32 % SEND_INTERVAL;
					tmp_ui16 = SEND_INTERVAL - (uint16_t) tmp_ui32;  // sec until 1st slave sends
					tmp_ui16 = tmp_ui16 + device_pointer*TIME_SLOT_DIFF;
					// calc time until 1st slave sends, then offset timeslot according to
					// how many slaves are already registered
				}
				
				tmp2_ui16 = tmp_ui16&0xFF00;
				tmp_array[4] = (uint8_t) tmp2_ui16;
				tmp2_ui16 = tmp_ui16&0x00FF;
				tmp_array[5] = (uint8_t) tmp2_ui16;
				
				send_data(tmp_array, 6);
				
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				
				state = STATE_IDLE;
			}			
		}

/*==============================================================================
                                IDLE
 =============================================================================*/

		if(state == STATE_IDLE)
		{
			bcm2835_delay(1000);
			t = time(NULL);
			ts = localtime(&t);
			
			if(day == 0)
			{
				day = ts->tm_mday;
				sprintf(date, "%d%02d%02d",ts->tm_year+1900,ts->tm_mon+1,ts->tm_mday);
			}
			else
			{
				if(ts->tm_mday != day) // next day!
				{
					day = ts->tm_mday;
					sprintf(date, "%d%02d%02d",ts->tm_year+1900,ts->tm_mon+1,ts->tm_mday);
					// create new files?
				}
			}
			
			first_message_received = 1;	// for debug
			if(first_message_received == 0)
			{
				state = STATE_RX;
			}
			else
			{
				sec_counter++;
				printf("counter: %d\n",sec_counter);
				tmp_ui8 = bcm2835_gpio_lev(PIN15_BUTTON);
				if(tmp_ui8 == 1)
				{
					//state = STATE_REGISTRATION;
					state = STATE_TX; // for debug
					bcm2835_gpio_set_eds(PIN15_BUTTON);
					printf("button pressed!\n");
				}
				else
				{		
					//if((ts->tm_sec % 10) < 9)
					if(sec_counter == (SEND_INTERVAL - 5))
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
							//delay(1000);	// 1s delay when no RX or TX
						}
					}
				}
			}

		}
	} // while(1) closed

	printf("\nfinish\n");
    return 0;
}	// main closed

/*==============================================================================
                                USER FUNCTIONS
 =============================================================================*/

//obsolete?
void time_message(time_t t, uint8_t* p_array)
{
	uint32_t tmp_ui32;
	uint8_t sec, min, h, day, mo, yr;
	struct tm * ts = localtime(&t);
	sec = ts->tm_sec;
	min = ts->tm_min;
	h = ts->tm_hour;
	day = ts->tm_mday;
	mo = ts->tm_mon+1;
	yr = ts->tm_year-100;	// 2000 + year
	tmp_ui32 = (sec&0x3F) | (min&0x3F)<<6 | (h&0x1F)<<6 | (day&0x1F)<<5 | (mo&0x0F)<<5 | (yr&0x3F)<<4;
	*p_array = 0xAA; p_array++;		// first byte to make sure message is recognized
	*p_array = tmp_ui32&0xFF000000; p_array++;
	*p_array = tmp_ui32&0x00FF0000;	p_array++;
	*p_array = tmp_ui32&0x0000FF00; p_array++;
	*p_array = tmp_ui32&0x000000FF; p_array++;
	// offset time for slot
	*p_array = OFFSET_MINUTES; p_array++;
	*p_array = OFFSET_SECONDS;
	//return tmp_array;
}

int send_data(uint8_t* data_pointer, uint8_t bytes)
{
	if(bcm2835_gpio_lev(PIN16_SDN) == 1)
	{
		bcm2835_gpio_write(PIN16_SDN, LOW);
		delay(1);	// SPIRIT MC startup		
		wPiSPI_init_RF();
		SpiritPktStackRequireAck(S_DISABLE);
		SpiritPktCommonRequireAck(S_DISABLE);
		SpiritCmdStrobeReady();
	}
	
	SpiritPktBasicSetPayloadLength(bytes);
	SpiritIrqClearStatus();
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

	SpiritSpiWriteLinearFifo(bytes, data_pointer);
	
	SpiritCmdStrobeTx();
	delay(25);
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
	//delay(500);
	return 1;
}

/* put spirit in rx mode
 * wait until data package received
 * write to buffer array
 */
uint8_t receive_data(void)
{
	uint8_t irq_rx_data_ready = 0;
	uint8_t received_bytes = 0;
	if(spirit_on == 0)
	{
		spirit_on = 1;
		bcm2835_gpio_write(PIN16_SDN, LOW);
		delay(1);	// SPIRIT MC startup		
		wPiSPI_init_RF();
		SpiritPktStackRequireAck(S_DISABLE);
		SpiritPktCommonRequireAck(S_DISABLE);
		SpiritCmdStrobeReady();
		SpiritPktBasicSetPayloadLength(4+MEASURE_VALUES*2);
		SpiritIrqClearStatus();
		//SpiritTimerSetRxTimeoutMs(3000);
		SET_INFINITE_RX_TIMEOUT();
	}
	
	SpiritCmdStrobeFlushRxFifo();
	//printf("receiving...\n");
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
	//printf("Status (RX): %X\n", g_xStatus.MC_STATE);	

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
		bcm2835_delay(1);
		
	}while(irq_rx_data_ready == 0);
	
	if(irq_rx_data_ready == 1)
	{
		SpiritIrqClearStatus();			
		received_bytes = SpiritLinearFifoReadNumElementsRxFifo();
		SpiritSpiReadLinearFifo(received_bytes, rx_buffer);		
		SpiritCmdStrobeFlushRxFifo();
	}
	return received_bytes;
}

void create_file(uint8_t device_pointer, char* string, char* filenames)
{
	FILE * fp;
	int i;
	char* filep = filenames + (device_pointer*30);
	char sensor[3];
	sprintf(sensor, "%02d", device_pointer);
	strcat(filep, "./data/sensor");
	strcat(filep, sensor);
	strcat(filep, "_");
	strcat(filep, string);
	strcat(filep, "_00.csv");
	
	// for debug: additional numbering
	// remove for release
	for (i = 0; i < 100; i++) {
		filep[25] = i/10 + '0';
		filep[26] = i%10 + '0';
		if( access( filep, F_OK ) != -1 ) {
			// file exists
		} else {
			fp = fopen(filep, "w+");
			break;
		}
	}
	
	fprintf(fp, "time,temperature,pressure,humidity,moisture\n");
	printf("file created!\n");
	fclose(fp);
}

int save_to_file(uint8_t sensor_to_save, time_t t, char* filenames, uint16_t* temperature, uint16_t* pressure, uint16_t* humidity, uint16_t* moisture)
{
	FILE * fp;
	struct tm * ts;
	uint16_t tmp_ui16;
	uint16_t time_table[MEASURE_VALUES][6] = {0};
	int j;
#ifdef OUTPUT			
	printf("temperature\n");
	for(j=0;j<MEASURE_VALUES;j++)
	{
		printf("%d\t", *(temperature+j);
	}
	printf("\n");
	printf("pressure\n");	
	for(j=0;j<MEASURE_VALUES;j++)
	{
		printf("%d\t", *(pressure+j));
	}
	printf("\n");
	printf("humidity\n");
	for(j=0;j<MEASURE_VALUES;j++)
	{
		printf("%d\t", *(humidity+j));
	}
	printf("\n");
	printf("moisture\n");
	for(j=0;j<MOISTURE_VALUES;j++)
	{
		printf("%d\t", *moisture);
	}
	printf("\n");	
#endif			
	
	fp = fopen((filenames+sensor_to_save*30), "a+");
	// dataset loop (buffered messages, each contains all 4 measurement values)
	
		// create time table for dataset
	for(j=(MEASURE_VALUES-1);j>=0;j=j-1)
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
		if(moisture_received == 1)
		{
			tmp_ui16 = moisture;
			moisture = 0;
			moisture_received = 0;
		}
		else
		{
			tmp_ui16 = 0;
		}
		
		//fprintf(fp, "%d,%d,%d,%d,%d,%d,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
		fprintf(fp, "%d:%d:%d,%d,%d,%d,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], *(temperature+j), *(pressure+j), *(humidity+j), tmp_ui16);
		//fprintf(fp, "%d,%d,%d,%d,%d\n", (int)rx_time_array[i], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
	}
	
	fclose(fp);
	return 1;
}
