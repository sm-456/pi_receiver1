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
#define SEND_INTERVAL 		55		// time between transmissions, seconds
#define MEASURE_INTERVAL 	5		// time between measurements, seconds
#define MEASURE_VALUES 		10		// number of values per transmission
#define MOISTURE_VALUES 	1		// values in moisture package
#define RX_DATA_BUFFER 		2		// number of datasets saved between file operations
#define TIME_SLOT_DIFF 		20		// offset between slave time slots
#define FILENAME_LENGTH 	40
#define FIRST_SLAVE_OFFSET	30		// first slave has to wait before starting data
#define RX_OFFSET			5		// seconds to go RX state before expected data
#define RX_OFFSET_FIRST		15

#define MAX_DEVICES 		16
#define OFFSET_MINUTES 		1
#define OFFSET_SECONDS 		10

char* create_file(uint8_t device_pointer, char* string);
int save_to_file(uint8_t sensor_to_save, time_t t, char* filenames, uint16_t* temperature, uint16_t* pressure, uint16_t* humidity, uint16_t* moisture);
int send_data(uint8_t* data_pointer, uint8_t bytes);
uint8_t receive_data(uint8_t* rx_buffer);


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
	int64_t tmp64 = 0;
	uint8_t next_sensor = 0;
	
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
	uint32_t t_int2 = 0;
/*==============================================================================
                                ARRAYS
 =============================================================================*/	

	uint8_t rx_buffer[FIFO_BUFF];
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
	uint16_t device_storage[MAX_DEVICES] = {0};
	uint32_t send_times[MAX_DEVICES];
	uint16_t send_counter[MAX_DEVICES] = {0};
	uint32_t last_transmission_time[MAX_DEVICES] = {0};
	uint16_t time_difference[MAX_DEVICES] = {0};
	char* filenames[MAX_DEVICES];
	char date[9];
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
	
	// HW pin 15 rising edge detect for button press
	bcm2835_gpio_fsel(PIN15_BUTTON, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN15_BUTTON, BCM2835_GPIO_PUD_DOWN);
	
	// HW pin 16 SPIRIT1 shutdown input toggle
	bcm2835_gpio_fsel(PIN16_SDN, BCM2835_GPIO_FSEL_OUTP);
	
	//reset transceiver via SDN
	bcm2835_gpio_write(PIN16_SDN, HIGH);
	delay(500);
	
	t = time(NULL);
	ts = gmtime(&t);
	
	//printf("\n");
	
	while(1)
	{
		
/*==============================================================================
                                RX
 =============================================================================*/
		
		if(state == STATE_RX) //rx
		{
			// receive packets and save in buffer
			do
			{
				received_bytes = receive_data(&(rx_buffer[0]));
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
	
			if(received_packets_counter==2 || (received_packets_counter==3 && moisture_data==1))
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
					tmp_sensor_id = ((rx_buffer[0]<<8)|rx_buffer[1])>>5;
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
				
				// find sensor in device array
				for(j=0;j<device_pointer;j++)
				{
					if(tmp_sensor_id == device_storage[j])
					{
						sensor_to_save = j;
						break;
					}
				}
				printf("ID: %d\n", tmp_sensor_id);
				printf("Array: %d %d\n", device_storage[0], device_storage[1]);
				printf("sensor to save: %d\n", sensor_to_save);
				
				if(last_transmission_time[sensor_to_save] == 0)
				{
					// first transmission
					last_transmission_time[sensor_to_save] = (uint32_t) t_rx;
					time_difference[sensor_to_save] = SEND_INTERVAL;
				}
				else
				{
					time_difference[sensor_to_save] = ((uint32_t)t_rx) - last_transmission_time[sensor_to_save];
					last_transmission_time[sensor_to_save] = (uint32_t) t_rx;
				}
				
				printf("save to file: sensor %d\n", sensor_to_save);
				save_to_file(sensor_to_save, t_rx, filenames[sensor_to_save], temperature, pressure, humidity, &moisture);
				
				next_sensor = sensor_to_save+1;
				if(next_sensor == device_pointer)
				{
					next_sensor = 0;
				}
				
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				bcm2835_delay(100);
				state = STATE_IDLE;
			}
			else
			{
				//data not ok
				//send request for new send attempt
				//state = STATE_RX;
				state = STATE_IDLE;
			}
			state = STATE_IDLE;
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
			//bcm2835_delay(100);

			received_bytes = receive_data(&(rx_buffer[0]));
			for(i=0;i<received_bytes;i++)
			{
				printf("%X ",rx_buffer[i]);
			}
			printf("\n");

			t_int = (uint32_t) time(NULL);
			//rx_time = gmtime(&t);
			tmp_ui16 = rx_buffer[0]<<8;
			tmp_sensor_id = (uint16_t) ((tmp_ui16|rx_buffer[1])>>5); 
			printf("sensor id: %d\n", tmp_sensor_id);
			tmp_ui8 = 0;
			// check if device is already registered
			//printf("device pointer: %d\n", device_pointer);
			if(device_pointer > 0)
			{
				for(i=0;i<device_pointer;i++)
				{
					if(device_storage[i] == tmp_sensor_id)
						tmp_ui8 = 1;
				}
			}
			else
			{
				tmp_ui8 = 0;
			}
			
			if(tmp_ui8 == 0)
			{
				// device not yet registered
				device_storage[device_pointer] = tmp_sensor_id;	// save id	
				//create_file(device_pointer,&(date[0]), &(filenames[0][0]));  // sensor00_20180101_00.csv	
				filenames[device_pointer] = create_file(device_pointer,&(date[0]));  // sensor00_20180101_00.csv		
				device_pointer++;
				send_time = 1;		
			}
			if (send_time == 1)
			{
				send_time = 0;
				//time_message(t_rx, time_array);	//create 32 bit time value
				
				t_int2 = t_int;
				tmp_array[0] = 0xAA;			
				tmp_ui32 = t_int2&0x000000FF; 
				tmp_array[4] = (uint8_t) tmp_ui32;
				t_int2 = t_int2 >> 8;
				tmp_ui32 = t_int2&0x000000FF; 
				tmp_array[3] = (uint8_t) tmp_ui32;
				t_int2 = t_int2 >> 8;
				tmp_ui32 = t_int2&0x000000FF; 
				tmp_array[2] = (uint8_t) tmp_ui32;
				t_int2 = t_int2 >> 8;
				tmp_ui32 = t_int2&0x000000FF; 
				tmp_array[1] = (uint8_t) tmp_ui32;
				
				if(device_pointer == 1)
				{
					tmp_ui16 = FIRST_SLAVE_OFFSET;	// first slave, default wait time 60 sec
				}
				else
				{
					tmp_ui32 = send_times[0] - t_int;
					tmp_ui16 = ((uint16_t) tmp_ui32) + device_pointer*TIME_SLOT_DIFF;
					
					//tmp_ui32 = t_int - send_times[0];
					//tmp_ui32 = tmp_ui32 % SEND_INTERVAL;
					//tmp_ui16 = SEND_INTERVAL - (uint16_t) tmp_ui32;  // sec until first slave sends
					//tmp_ui16 = tmp_ui16 + device_pointer*TIME_SLOT_DIFF;
					// calc time until 1st slave sends, then offset timeslot according to
					// how many slaves are already registered
				}
				send_times[device_pointer-1] = t_int + ((uint32_t)tmp_ui16);	// save send time
			
				tmp_ui32 = send_times[device_pointer-1] + SEND_INTERVAL;
				t = (time_t) tmp_ui32;
				ts = gmtime(&t);
				printf("first transmission: %02d:%02d:%02d\n", ts->tm_hour,ts->tm_min,ts->tm_sec);
				//printf("%s\n", asctime(ts));
				
				tmp2_ui16 = tmp_ui16&0xFF00;
				tmp_array[5] = (uint8_t) tmp2_ui16;
				tmp2_ui16 = tmp_ui16&0x00FF;
				tmp_array[6] = (uint8_t) tmp2_ui16;
				
				bcm2835_delay(70);
				
				send_data(tmp_array, 7);
				time_difference[device_pointer-1] = SEND_INTERVAL;
				tmp_ui32 = send_times[device_pointer-1];
				t = (time_t) tmp_ui32;
				ts = gmtime(&t);
				printf("sleep until: %02d:%02d:%02d\n", ts->tm_hour,ts->tm_min,ts->tm_sec);
				
				printf("time_int: %d\n", t_int);
				printf("Data sent: ");
				for(i=0;i<7;i++)
				{
					printf("%X ", tmp_array[i]);
				}
				printf("\n");
				
				spirit_on = 0;
				bcm2835_gpio_write(PIN16_SDN, HIGH);
				printf("new device: slot %d\n", device_pointer-1);
				state = STATE_IDLE;
			}
			else
			{
				printf("device already registered...\n");
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
			ts = gmtime(&t);
			t_int = (uint32_t) t;
			
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

			printf("time: %02d:%02d:%02d\n",ts->tm_hour,ts->tm_min,ts->tm_sec);
			tmp_ui8 = 255;
			// increase send counter for registered slaves
			tmp_ui32 = 0xFFFFFFFF;
			
			if((last_transmission_time[next_sensor] == 0) && device_pointer > 0)
			{
				tmp_ui32 = send_times[next_sensor] + SEND_INTERVAL - RX_OFFSET_FIRST;
			}
			else
			{
				tmp_ui32 = (last_transmission_time[next_sensor] + time_difference[next_sensor] - RX_OFFSET);
			}			
			
			if(t_int > tmp_ui32)
			{
				printf("slave %d sending in %d seconds...\n", next_sensor, RX_OFFSET);
				printf("time difference: %d\n", time_difference[next_sensor]);
				state = STATE_RX;
				/*
				next_sensor++;
				if(next_sensor == device_pointer)
				{
					next_sensor = 0;
				}
				*/

			}
			else
			{
			
			/*
			for(i=0;i<device_pointer;i++)
			{
				if(t_int > send_times[i])
				{	
					tmp_ui32 = (t_int - send_times[i]) % SEND_INTERVAL;
					//printf("%d\n",tmp_ui32);
					if(tmp_ui32 > (SEND_INTERVAL - RX_OFFSET))
					{
						tmp_ui8 = i;
						//printf("transmission: %d\n",tmp_ui8);
					}
				}
				else
				{
					// wait for first transmission
				}
			}

			if(tmp_ui8 != 255)
			{
				//registered sensor about to send data
				state = STATE_RX;
				printf("slave %d sending in 5 seconds...\n", tmp_ui8);
			}
			* */
			//else{
				tmp_ui8 = bcm2835_gpio_lev(PIN15_BUTTON);
				if(tmp_ui8 == 1)
				{
					state = STATE_REGISTRATION;
					//state = STATE_TX; // for debug
					bcm2835_gpio_set_eds(PIN15_BUTTON);
					printf("button pressed!\n");
				}
			}
		} // state_IDLE closed
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
	struct tm * ts = gmtime(&t);
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
uint8_t receive_data(uint8_t* rx_buffer)
{
	uint8_t irq_rx_data_ready = 0;
	uint8_t received_bytes = 0;
	if(bcm2835_gpio_lev(PIN16_SDN)==HIGH)
	{
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

char* create_file(uint8_t device_pointer, char* string)
{
	FILE * fp;
	int i;
	uint16_t length;
	char tmp[FILENAME_LENGTH];
	//char* filep = filenames + (device_pointer*FILENAME_LENGTH);
	char* ret;
	char sensor[3];
	//printf("String: %s\n", tmp);
	sprintf(sensor, "%02d", device_pointer);
	sprintf(tmp, "./data/sensor");
	strcat(tmp, sensor);
	strcat(tmp, "_");	
	strcat(tmp, string);
	strcat(tmp, "_00.csv");
	//printf("String: %s\n", tmp);
	// for debug: additional numbering
	// remove for release
	for (i = 0; i < 100; i++) {
		tmp[25] = i/10 + '0';
		tmp[26] = i%10 + '0';
		if( access( tmp, F_OK ) != -1 ) {
			// file exists
		} else {
			//open file
			break;
		}
	}
	length = strlen(tmp);
	//printf("String: %s\n", tmp);
	//printf("length: %d\n", length);
	ret = (char*) malloc(length * sizeof(char));
	strcpy(ret, tmp);
	
	fp = fopen(ret, "w+");
	fprintf(fp, "time,temperature,pressure,humidity,moisture\n");
	//printf("file created!\n");
	fclose(fp);
	return ret;
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
	
	fp = fopen(filenames, "a+");
	// dataset loop (buffered messages, each contains all 4 measurement values)
	
		// create time table for dataset
	for(j=(MEASURE_VALUES-1);j>=0;j=j-1)
	{
		ts = gmtime(&t);

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
			tmp_ui16 = *moisture;
			moisture = 0;
			moisture_received = 0;
		}
		else
		{
			tmp_ui16 = 0;
		}
		
		//fprintf(fp, "%d,%d,%d,%d,%d,%d,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
		fprintf(fp, "%02d:%02d:%02d,%d,%d,%d,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], *(temperature+j), *(pressure+j), *(humidity+j), tmp_ui16);
		//fprintf(fp, "%d,%d,%d,%d,%d\n", (int)rx_time_array[i], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
	}
	
	fclose(fp);
	return 1;
}
