#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "SPIRIT_Config.h"
#include "globals.h"
#include "bcm2835.h"
#include "SPI_interface.h"
#include <dirent.h>
//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

//#define OUTPUT

/*==============================================================================
                                SETTINGS
 =============================================================================*/

#define TWITTER 	0				// enable tweet feature
#define SEND_REPEAT 0				// enable send repeat after transmission fault
#define USE_UTC 	0				// 1: use UTC, 0: use local time
#define FIFO 		18
#define SEND_INTERVAL 		300 	// time between transmissions, seconds
#define MEASURE_INTERVAL 	30		// time between measurements, seconds
#define SENSOR_WAKEUP_TIME	30
#define MEASURE_VALUES 		10		// number of values per transmission
#define MOISTURE_VALUES 	1		// values in moisture package
#define OUTPUT_POWER		1		// dBm for transmission

#define TIME_SLOT_DIFF 		300		// offset between slave time slots
#define FILENAME_LENGTH 	50
#define FIRST_SLAVE_OFFSET	2		// first slave has to wait before starting data collection
#define RX_OFFSET			40		// seconds to go RX state before expected data
#define RX_OFFSET_FIRST		90		// offset for first transmission
#define RX_TIMEOUT			80		// timeout of RX mode


#define MAX_DEVICES 		16
#define OFFSET_MINUTES 		1
#define OFFSET_SECONDS 		10

/*==============================================================================
                                PARAMETERS
 =============================================================================*/
#define GPIO_RF1 		RPI_V2_GPIO_P1_13			// RF module GPIO 1
#define GPIO_RF3 		RPI_V2_GPIO_P1_11			// RF module GPIO 3
//#define GPIO_SDN 		RPI_V2_GPIO_P1_03
#define GPIO_SDN		RPI_V2_GPIO_P1_32
#define GPIO_BUTTON1 	RPI_V2_GPIO_P1_35
#define GPIO_BUTTON2 	RPI_V2_GPIO_P1_37
#define GPIO_LED_GRUEN 	RPI_V2_GPIO_P1_33
#define GPIO_LED_ROT 	RPI_V2_GPIO_P1_31
#define GPIO_LED_GELB 	RPI_V2_GPIO_P1_29
#define GPIO_LED_BLAU 	RPI_V2_GPIO_P1_07
#define GPIO_V_BUTTON	RPI_V2_GPIO_P1_40


#define STATE_IDLE 	0
#define STATE_RX 	1
#define STATE_TX 	2
#define STATE_FILE 	3
#define STATE_REGISTRATION 4
#define STATE_BT	5

/*==============================================================================
                                PROTOTYPES
 =============================================================================*/
char* create_file(uint8_t device_pointer, char* string);
int save_to_file(uint8_t sensor_to_save, time_t t, char* filenames, uint16_t* temperature, uint16_t* pressure, uint16_t* humidity, float* moisture, uint8_t moisture_received, uint16_t time_difference);
int send_data(uint8_t* data_pointer, uint8_t bytes);
uint8_t receive_data(uint8_t* rx_buffer, uint8_t timeout);
float calc_moisture(uint16_t frequency);
char * stringReplace(char* search, char* replace, char* string);
struct tm * get_time(time_t * tp);
void init_bcm(void);


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
char directory[20];
uint8_t moisture_received = 0;
float last_moisture = 0;
int16_t rssi = 0;

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
	uint8_t day = 0;
	uint8_t sensor_to_save = 0;
	uint8_t next_sensor = 0;
	uint32_t start_time = 0;
	uint8_t rx_sensor = 0;
	int16_t tmp_s16 = 0;
	uint8_t state = 0;
	struct stat st = {0};
	uint16_t rx_offset_time = RX_OFFSET;
	
	// time variables
	time_t t, t2;
	time_t t_rx;
	struct tm * ts, ts2;
	struct tm * rx_time;	
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
	float moisture = 0;
	uint16_t frequency = 0;
	uint8_t tx_buffer[10];
	uint8_t packet_type[4] = {0};
	uint32_t date_list[21] = {0};
	uint32_t epoch_time[MEASURE_VALUES] = {0};
	uint8_t first_transmissions[MAX_DEVICES] = {0};
	
	// oldest dataset at 0
	//uint32_t rx_time;
	
	uint8_t received_packets_buffer[4][MEASURE_VALUES*2+4] = {0};  // save first packages until transmission complete
	uint16_t time_table[MEASURE_VALUES][6] = {0};		  // array for time in csv data table
	uint16_t device_storage[MAX_DEVICES] = {0};
	uint32_t send_times[MAX_DEVICES];
	uint32_t data_collection_start[MAX_DEVICES];
	int16_t send_counter[MAX_DEVICES] = {0};
	uint32_t last_transmission_time[MAX_DEVICES] = {0};
	uint32_t time_difference[MAX_DEVICES] = {0};
	uint32_t next_transmission[MAX_DEVICES] = {0};
	char* filenames[MAX_DEVICES];
	char date[9];
	char * tmp_string = (char*) malloc(50 * sizeof(char));
	char * tmp_string2 = (char*) malloc(50 * sizeof(char));
	char delimiter[2] = "/";
	char * tmp_char_p = (char*) malloc(50 * sizeof(char));
	char * root_path = (char*) malloc(40*sizeof(char));
	char * tmp_string3 = (char*) malloc(20*sizeof(char));
	char * bt_command = (char*) malloc (50*sizeof(char));
	char * absolute_path = (char*) malloc(100*sizeof(char));
	char * command = (char*) malloc(150*sizeof(char));
	uint8_t test[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	char date_search[15];
	char date_replace[15];
	
	char temp_string[20];
	char temp_string2[20];

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
	uint8_t first_transmission = 0;
	uint8_t check_sensor = 0;
	uint8_t go_rx_mode = 0;

	uint8_t send_time = 0;
	uint8_t exit_loop = 0;
	uint8_t timeout_exit = 0;
/*==============================================================================
                                COUNTER
 =============================================================================*/
 
     uint16_t sec_counter = 0;				// generic counter
	 uint8_t stored_datasets_counter = 0;		// number of received complete datasets	
	 uint8_t received_packets_counter = 0;		// number of received packages of single set (3 or 4)
	 uint8_t send_attempts = 0;
	
	 uint8_t device_pointer = 0;	// point to empty field in device storage 
	 uint8_t date_list_pointer = 0;
	 //uint16_t *p_value_array;

//==============================================================================	
	
	init_bcm();

	t = time(NULL);
	ts = get_time(&t);
	
	//root_path = "/home/pi/Projekte/pi_receiver1/";
	root_path = "/home/pi/";
	bt_command = "obexftp -b 40:40:a7:c2:de:0e -c sensordata -p ";
	
	while(1)
	{
		
/*==============================================================================
                                RX
 =============================================================================*/
		
		if(state == STATE_RX) //rx
		{
			bcm2835_gpio_write(GPIO_LED_ROT, HIGH);
			// receive packets and save in buffer
			received_packets_counter = 0;
			moisture_data = 0;
			
			do
			{
				received_bytes = receive_data(&(rx_buffer[0]),rx_offset_time+10);
				if(received_bytes == 255)
				{
					timeout_exit = 1;
				}
				
				more_data = rx_buffer[2]&0x01;
				packet_type[received_packets_counter] = rx_buffer[0]&0x07;
				
				if(packet_type[received_packets_counter] == 4)
					moisture_data = 1;
				
				if (more_data == 1)
				{
					// copy received data into buffer
					for(k=0;k<(MEASURE_VALUES*2+4);k++)
					{
						received_packets_buffer[received_packets_counter][k] = rx_buffer[k];
						rx_buffer[k] = 0;
					}		
				}
				
				received_packets_counter++;
			}while(more_data==1 && timeout_exit != 1);

			if(timeout_exit == 0)
			{			
				printf("packets: %d, moisture data: %d\n", received_packets_counter, moisture_data);
				
				// copy last received data into buffer for easier handling
				for(k=0;k<(MEASURE_VALUES*2+4);k++)
				{
					received_packets_buffer[received_packets_counter-1][k] = rx_buffer[k];
					rx_buffer[k] = 0;
				}	

				if(received_packets_counter==3 || received_packets_counter==4)
				{	
					// data ok: correct number of received packets
					t_rx = time(NULL);
					ts = get_time(&t_rx);
					printf("time_rx: %02d:%02d:%02d\n",ts->tm_hour,ts->tm_min,ts->tm_sec);
					if(SEND_REPEAT == 1)
					{
						// send OK message with slave ID and 0xAA token
						tx_buffer[0] = rx_buffer[1];
						tx_buffer[1] = rx_buffer[0]&0xE0;
						tx_buffer[2] = 0xAA;
						tx_buffer[3] = 0xAA;
						bcm2835_delay(100);
						send_data(&(tx_buffer[0]), 4);
						printf("Data OK sent\n");
					}
					
					for(j=0;j<received_packets_counter;j++)
					{
						
						if(packet_type[j] == 4)
						{
							received_bytes = 2*MOISTURE_VALUES+4;
						}
						else
						{
							received_bytes = 2*MEASURE_VALUES+4;
						}
						
						for(k=0;k<received_bytes;k++)
						{
							printf("%X ", received_packets_buffer[j][k]);
						}
						printf("\n");
											
						// sort data
						for(i=0;i<received_bytes;i=i+2)
						{
							tmp_ui8 = received_packets_buffer[j][i];
							received_packets_buffer[j][i] = received_packets_buffer[j][i+1];
							received_packets_buffer[j][i+1] = tmp_ui8;
						}		
						// extract preamble data (temporarily)
						tmp_sensor_id = ((received_packets_buffer[j][0]<<8)|received_packets_buffer[j][1])>>5;
						tmp_dataframe_id = (received_packets_buffer[j][2]<<8)|received_packets_buffer[j][3];
						//parameter = (received_packets_buffer[j][1]&0x07);
						parameter = packet_type[j];
						//printf("parameter: %X\n", parameter);
						
						// write data to value array
						if(parameter == 4)
						{
							frequency = (uint16_t) (received_packets_buffer[j][4]<<8)|received_packets_buffer[j][5];
							//moisture = frequency;					
							//last_moisture = moisture;								
							moisture_received = 1;
							moisture = calc_moisture(frequency);
							printf("frequency: %d Hz\n", frequency);
							printf("frequency buffer data:\n");
							for(k=0;k<received_bytes;k++)
							{
									printf("%X ", received_packets_buffer[j][k]);
							}
							printf("\n"); 
							//moisture = calc_moisture(frequency);
						}
						else
						{
							for(i=4;i<received_bytes;i=i+2)
							{
								// combine 2 bytes to ui16 and save to correct array
								switch(parameter)
								{								
									case 1: 
										temperature[(i/2)-2] = (received_packets_buffer[j][i]<<8)|received_packets_buffer[j][i+1]; 
										//temperature[(i/2)-2] = (temperature[(i/2)-2] - 4000)/100; 
										break;
									case 2: 
										pressure[(i/2)-2] = (received_packets_buffer[j][i]<<8)|received_packets_buffer[j][i+1];
										//pressure[(i/2)-2] = pressure[(i/2)-2]/10; 
										break;
									case 3: 
										humidity[(i/2)-2] = (received_packets_buffer[j][i]<<8)|received_packets_buffer[j][i+1];
										//humidity[(i/2)-2] = humidity[(i/2)-2]/100; 
										break;

									default: break;
								}
							}
						}						
						
						/*
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
						*/
						packet_type[j] = 0;
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
					printf("ID: %d, sensor no. %d\n", tmp_sensor_id, sensor_to_save);
					//printf("Array: %d %d\n", device_storage[0], device_storage[1]);
					
					if(last_transmission_time[sensor_to_save] == 0)
					{
						// first transmission
						first_transmission = 1;
						last_transmission_time[sensor_to_save] = (uint32_t) t_rx;
						time_difference[sensor_to_save] = SEND_INTERVAL;
						next_transmission[sensor_to_save] = (uint32_t) t_rx + SEND_INTERVAL;
					}
					else
					{
						time_difference[sensor_to_save] = ((uint32_t)t_rx) - last_transmission_time[sensor_to_save];
						if(time_difference[sensor_to_save] >= 1.4*SEND_INTERVAL)
							time_difference[sensor_to_save] = SEND_INTERVAL;
						last_transmission_time[sensor_to_save] = (uint32_t) t_rx;
						/*
						if(time_difference[sensor_to_save] > (SEND_INTERVAL+10))
						{
							time_difference[sensor_to_save] = SEND_INTERVAL;
						}
						*/
						next_transmission[sensor_to_save] = (uint32_t) t_rx + time_difference[sensor_to_save];
					}
					
					//printf("save to file: sensor %2d\n", sensor_to_save);
					save_to_file(sensor_to_save, t_rx, filenames[sensor_to_save], temperature, pressure, humidity, &moisture, moisture_received, time_difference[sensor_to_save]);
					if(TWITTER == 1)
					{
						sprintf(tmp_string, "sudo python3 /home/pi/python/twitter/helloworld.py ");
						printf("%s\n", tmp_string);
						strcpy(tmp_string2, filenames[sensor_to_save]);
						printf("filenames-array: %s\n", tmp_string2);
						//tmp_string2 = filenames[sensor_to_save];
						tmp_char_p = strtok(tmp_string2, delimiter);
						tmp_char_p = strtok(NULL, delimiter);
						tmp_char_p = strtok(NULL, delimiter);
						printf("directory: %s\n",directory);
						
						strcat(tmp_string, tmp_char_p);
						strcat(tmp_string, "/");
						tmp_char_p = strtok(NULL, delimiter);
						strcat(tmp_string, tmp_char_p);
						printf("%s\n", tmp_string);
						
						//tmp_ui8 = system("sudo python3 /home/pi/python/twitter/helloworld.py");
						tmp_ui8 = system(tmp_string);
						
						strcpy(tmp_char_p, "");
						strcpy(tmp_string2, "");
						strcpy(tmp_string, "");
						
						if( tmp_ui8 == -1 )
							printf( "Fehler beim Initialisieren der Shell.\n");
						else if( tmp_ui8 > 0 )
							printf( "Tweet erfolgreich, Code %d\n", tmp_ui8 );
						else 
							printf( "Tweet erfolgreich\n" );
					}
						
					if(moisture_received == 1)
					{
						moisture_received = 0;
					}
					
					send_counter[next_sensor] = 0;
					
					next_sensor = sensor_to_save+1;
					if(next_sensor == device_pointer)
					{
						next_sensor = 0;
					}
					
					
					spirit_on = 0;
					bcm2835_gpio_write(GPIO_SDN, HIGH);
					bcm2835_delay(20);
					state = STATE_IDLE;
				}
				else
				{
					printf("data not ok...\n");
					//send request for new send attempt
					if(SEND_REPEAT == 1)
					{
						send_attempts++;
						if(send_attempts < 4)
						{
							// try up to 3 times
							state = STATE_RX;
							tx_buffer[0] = 0xBB;
							tx_buffer[1] = 0xBB;
							tx_buffer[2] = 0xBB;
							tx_buffer[3] = 0xBB;
							bcm2835_delay(90);
							send_data(&(tx_buffer[0]), 4);
							printf("new attempt request sent\n");
						}
						if(send_attempts >= 4)
						{
							// quit trying to send
							tx_buffer[0] = 0x99;
							tx_buffer[1] = 0x99;
							tx_buffer[2] = 0x99;
							tx_buffer[3] = 0x99;
							bcm2835_delay(90);
							send_data(&(tx_buffer[0]), 4);
							
							send_counter[next_sensor] = 0;
							next_transmission[next_sensor] = (uint32_t) t_rx + time_difference[next_sensor];
							last_transmission_time[next_sensor] = (uint32_t) t_rx;
							next_sensor = sensor_to_save+1;
							if(next_sensor == device_pointer)
							{
								next_sensor = 0;
							}
							state = STATE_IDLE;
							
						}
					}
					else
					{
						t_rx = time(NULL);
						// do not repeat transmission
						printf("transmission fault, skipping dataset...\n");
						send_counter[next_sensor] = 0;
						next_transmission[next_sensor] = (uint32_t) t_rx + time_difference[next_sensor];
						last_transmission_time[next_sensor] = (uint32_t) t_rx;
						next_sensor = sensor_to_save+1;
						if(next_sensor == device_pointer)
						{
							next_sensor = 0;
						}
						state = STATE_IDLE;
					}

				}
				//state = STATE_IDLE;
				bcm2835_gpio_write(GPIO_LED_ROT, LOW);
			}
			else
			{
				// no message received, abort RX so subsequent messages are not missed
				timeout_exit = 0;
				t = time(NULL);
				ts = get_time(&t);
				printf("RX timeout at: %02d:%02d:%02d\n",ts->tm_hour,ts->tm_min,ts->tm_sec);
				send_counter[next_sensor] = 0;
				last_transmission_time[next_sensor] = next_transmission_time[next_sensor] // set last transmission to recent expected transmission
				next_transmission[next_sensor] = next_transmission[next_sensor] + time_difference[next_sensor];  // update to next expected transmission
				//last_transmission_time[next_sensor] = (uint32_t) t_rx;
				next_sensor = sensor_to_save+1;
				if(next_sensor == device_pointer)
				{
					next_sensor = 0;
				}
				state = STATE_IDLE;
			}
		}

/*==============================================================================
                                FILE
 =============================================================================*/

		if(state == STATE_FILE)	// write to file
		{
			// moved to user function in RX state
		}
		
/*==============================================================================
                                TX
 =============================================================================*/
 
		if(state == STATE_TX) //tx
		{	
			send_data(test,10);
			
			spirit_on = 0;
			bcm2835_gpio_write(GPIO_SDN, HIGH);
			delay(1000);
			state = STATE_IDLE;
			
		}
		
/*==============================================================================
                                REGISTRATION
 =============================================================================*/		

		if(state == STATE_REGISTRATION)
		{
			bcm2835_gpio_write(GPIO_LED_GELB, HIGH);
			printf("Waiting for device...\n");
			//bcm2835_delay(100);

			received_bytes = receive_data(&(rx_buffer[0]),10);
			if(received_bytes !=0)
			{
				printf("received data: ");
				for(i=0;i<received_bytes;i++)
				{
					printf("%2X ",rx_buffer[i]);
				}
				printf("\n");

				printf("RSSI: %d\n", rssi);
				
				t = time(NULL);
				t_int = (uint32_t) t;
				ts = get_time(&t);

				tmp_ui16 = rx_buffer[0]<<8;
				tmp_sensor_id = (uint16_t) ((tmp_ui16|rx_buffer[1])>>5); 
				printf("Time: %02d:%02d:%02d\n", ts->tm_hour,ts->tm_min,ts->tm_sec);
				printf("Sensor-ID: %d\n", tmp_sensor_id);
				check_sensor = 0;
				// check if device is already registered
				//printf("device pointer: %d\n", device_pointer);
				if(device_pointer > 0)
				{
					for(i=0;i<device_pointer;i++)
					{
						if(device_storage[i] == tmp_sensor_id)
							check_sensor = 1;
					}
				}
				else
				{
					check_sensor = 0;
				}
				
				if(check_sensor == 0)
				{
					// device not yet registered
					device_storage[device_pointer] = tmp_sensor_id;	// save id	
					//create_file(device_pointer,&(date[0]), &(filenames[0][0]));  // sensor00_20180101_00.csv	
					
					if(device_pointer==0)
					{
						i = 0;
						j = 0;
						do
						{
							sprintf(directory, "./data/run");
							sprintf(temp_string, "%02d_", i);
							strcat(directory, temp_string);
							strcat(directory, date);
							// directory name: runXX_yyyymmdd
							
							if (stat(directory, &st) == -1) 
							{
								mode_t process_mask = umask(0);
								mkdir(directory, ACCESSPERMS);
								umask(process_mask);
								j = 1;
								i = 0;
							
							}
							else
							{
								i++;
							}								
						}while(j==0);
						j = 0;
						
					}
					
					filenames[device_pointer] = create_file(device_pointer,&(date[0]));  // sensor00_20180101_00.csv		
					device_pointer++;
					
					send_time = 1;		
				}
				if (send_time == 1)
				{
					send_time = 0;
			
					// date data for RTC
					tmp_array[0] = 0xAA;
					tmp_array[1] = ts->tm_year - 100;
					tmp_array[2] = ts->tm_mon + 1;
					tmp_array[3] = ts->tm_mday;
					tmp_array[4] = ts->tm_hour;
					tmp_array[5] = ts->tm_min;
					tmp_array[6] = ts->tm_sec;
						
					if(device_pointer == 1) // first slave, default wait time
					{
						tmp_ui16 = FIRST_SLAVE_OFFSET;	
						start_time = t_int + tmp_ui16;				
						
					}
					else
					{
						// offset time until first slave begins
						tmp_ui32 = data_collection_start[0] - t_int;
						// offset between slaves
						tmp_ui16 = ((uint16_t) tmp_ui32) + (device_pointer-1)*TIME_SLOT_DIFF;  
						// final offset
						tmp_ui16 = tmp_ui32 + tmp_ui16; 				
						start_time = t_int + tmp_ui16;
					}
					//send_counter[device_pointer-1] = tmp_ui16 * (-1);
					//send_times[device_pointer-1] = t_int + ((uint32_t)tmp_ui16);	// save send time
					data_collection_start[device_pointer-1] = start_time;
					next_transmission[device_pointer-1] = start_time + SEND_INTERVAL;
					
					tmp_ui32 = start_time + SEND_INTERVAL;
					t = (time_t) tmp_ui32;
					ts = get_time(&t);
					printf("first transmission: approx. %02d:%02d:%02d\n", ts->tm_hour,ts->tm_min,ts->tm_sec);
					
					// encode offset time for transmission
					tmp_array[7] = (uint8_t) (tmp_ui16>>8);
					tmp_array[8] = (uint8_t) (tmp_ui16&0x00FF);
					
					// byte for sensorboard settings
					tmp_ui8 = 0;
					tmp_ui8 = tmp_ui8|SEND_REPEAT;	// transmission fail: repeat
					tmp_array[9] = tmp_ui8;
					
					tmp_ui16 = SENSOR_WAKEUP_TIME;	// dictate wakeup-time for slave
					tmp_array[10] = (uint8_t) (tmp_ui16>>8);
					tmp_array[11] = (uint8_t) (tmp_ui16&0x00FF);
					tmp_array[12] = (uint8_t) (OUTPUT_POWER + 34);  // offset +34 because of unsigned int
					
					printf("Offset time: %d sec\n", tmp_ui16);
						
					bcm2835_delay(70);  // wait for sensorboard to get ready for reception
					
					send_data(tmp_array, 13);
					
					printf("sent data: ");
					for(i=0;i<13;i++)
					{
						printf("%2X ", tmp_array[i]);
					}
					printf("\n");
					
					time_difference[device_pointer-1] = SEND_INTERVAL;

					//printf("sleep until: %02d:%02d:%02d\n", ts->tm_hour,ts->tm_min,ts->tm_sec);
					//printf("time_int: %d\n", t_int);

					printf("new device: slot %d\n", device_pointer-1);
					state = STATE_IDLE;
				}
				else
				{
					printf("device already registered...\n");
					state = STATE_IDLE;
				}			
			}
			else
			{
				// rx timeout, no registration
				printf("no device registered\n");
				state = STATE_IDLE;
			}
			bcm2835_gpio_write(GPIO_LED_GELB, LOW);
		}
		
/*==============================================================================
                                BLUETOOTH
 =============================================================================*/		
		if(state == STATE_BT)
		{
			bcm2835_gpio_write(GPIO_LED_BLAU, HIGH);
			/*
			DIR *d;
			struct dirent *dir;
			d = opendir("./data");
			if (d) {
				while ((dir = readdir(d)) != NULL) {
					printf("%s\n", dir->d_name);
				}
				closedir(d);
			}
			*/
			state = STATE_IDLE;
			sprintf(command,"obexftp -b 40:40:a7:c2:de:0e -c sensordata -p ");
			
			for(i=0;i<device_pointer;i++)
			{
				
				//printf("%s\n", filenames[i]);
				sprintf(tmp_string,"%s",filenames[i]);
				//printf("%s\n",tmp_string);
				tmp_string++;
				tmp_string++;
				printf("%s\n",tmp_string);

				//sprintf(absolute_path,"/home/pi/Projekte/pi_receiver1/%s",tmp_string);
				sprintf(absolute_path, "%s%s",root_path, tmp_string);
				//printf("Pfad: %s\n", absolute_path);
				sprintf(command, "%s%s", bt_command,absolute_path);
				tmp_ui8 = system(command);
				bcm2835_delay(40);
				if(date_list_pointer > 1)
				{
					sprintf(date_search, "%s",date);
					for(j=0;j<date_list_pointer-1;j++)
					{
						sprintf(date_replace,"%d", date_list[j]);  // new (previous) date
						stringReplace(date_search,date_replace,absolute_path);
						sprintf(command, "%s%s", bt_command,absolute_path);
						tmp_ui8 = system(command);
						bcm2835_delay(40);
						sprintf(date_search,"%s",date_replace);	
					}
					sprintf(date_search,"");
					sprintf(date_replace,"");
				}
				strcpy(tmp_string,"");
				strcpy(absolute_path,"");
				strcpy(command,"");
				
			}
			bcm2835_gpio_write(GPIO_LED_BLAU, LOW);
		}
/*==============================================================================
                                IDLE
 =============================================================================*/

		if(state == STATE_IDLE)
		{
	
			bcm2835_delay(1000);
			t = time(NULL);
			ts = get_time(&t);
			t_int = (uint32_t) t;
			//printf("time: %d\n", t_int);
	
			if(day == 0)
			{
				// date initialization
				day = ts->tm_mday;
				sprintf(date, "%d%02d%02d",ts->tm_year+1900,ts->tm_mon+1,ts->tm_mday);
				date_list[date_list_pointer] = atoi(date);
				date_list_pointer++;
			}
			else
			{
				if(ts->tm_mday != day) // next day!
				{
					day = ts->tm_mday;
					sprintf(date, "%d%02d%02d",ts->tm_year+1900,ts->tm_mon+1,ts->tm_mday);
					// create new files?
					date_list[date_list_pointer] = atoi(date);
					date_list_pointer++;
					for(i=0; i<device_pointer; i++)
					{
						// rewrite filenames array
						// create new files
						filenames[i] = create_file(i, date);
					}
				}
			}		
			
			tmp_ui8 = 255;

			i = 0;
			// check if any slave is about to send
			if(device_pointer != 0)
			{
				do
				{
					tmp_ui16 = next_transmission[i] - t_int;
					if(tmp_ui16 > 65000)
					{
						if(i>0)
							next_transmission[i] = next_transmission[i-1] + TIME_SLOT_DIFF;
						else
							next_transmission[i] = t_int + (SEND_INTERVAL/2);
						tmp_ui16 = next_transmission[i] - t_int;
						
					}
					if (tmp_ui16 % 10 == 0)
						//printf("seconds until transmission: %d\t(slave %d)\n", tmp_ui16,i);
						
					switch(first_transmissions[i])
					{
						case 0:
							rx_offset_time = RX_OFFSET_FIRST;
							break;
						case 1:
							rx_offset_time = RX_OFFSET_FIRST;
							break;
						case 2:
							rx_offset_time = RX_OFFSET;
							break;
						default:
							rx_offset_time = RX_OFFSET;
							break;
					}
					//printf("transmission counter: %d, offset time: %d\n", first_transmissions[i], rx_offset_time);
						
					if((next_transmission[i] - t_int) < rx_offset_time && next_transmission[i] > t_int)
					{
						go_rx_mode = 1;
						rx_sensor = i;
						next_sensor = i;
						switch(first_transmissions[i])
						{
							case 0:
								first_transmissions[i]++;
								break;
							case 1:
								first_transmissions[i]++;
								break;
							case 2:
								break;
							default:
								break;
						}
					} 

					if((time_difference[i] <= 0.8*SEND_INTERVAL || time_difference[i] >= 1.4*SEND_INTERVAL) && time_difference[i] != 0)
					{
						printf("time difference: %d, resetting value...\n", time_difference[i]);
						time_difference[i] = SEND_INTERVAL;
					}
					
					i++;
					// break if all devices are checked or if one is about to send
				}while(i<device_pointer && go_rx_mode==0);
			}
			
			if(go_rx_mode == 1)
			{
				printf("time: %02d:%02d:%02d\n",ts->tm_hour,ts->tm_min,ts->tm_sec);
				go_rx_mode = 0;
				// determine time to go RX-mode before actual transmission (for safety)

				printf("slave %d sending in %d seconds...\n", next_sensor, rx_offset_time);
				printf("time difference: %d\n", time_difference[next_sensor]);
				state = STATE_RX;
			}
			else
			{
				// check buttons
				tmp_ui8 = bcm2835_gpio_lev(GPIO_BUTTON1);
				if(tmp_ui8 == 1)
				{
					state = STATE_REGISTRATION;
					bcm2835_gpio_set_eds(GPIO_BUTTON1);
					printf("button A pressed!\n");
				}
				else
				{
					tmp_ui8 = bcm2835_gpio_lev(GPIO_BUTTON2);
					if(tmp_ui8 == 1)
					{
						state = STATE_BT;
						bcm2835_gpio_set_eds(GPIO_BUTTON2);
						printf("button B pressed!\n");
					}
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
	struct tm * ts;
	ts = get_time(&t);
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
	if(bcm2835_gpio_lev(GPIO_SDN) == 1)
	{
		bcm2835_gpio_write(GPIO_SDN, LOW);
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
	delay(50);
	//printf("send data...\n");
	
	/*
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
	*/
	bcm2835_gpio_write(GPIO_SDN, HIGH);
	bcm2835_delay(2);
	return 1;
}

/* put spirit in rx mode
 * wait until data package received
 * write to buffer array
 */
uint8_t receive_data(uint8_t* rx_buffer, uint8_t timeout)
{
	uint32_t timeout_ms = timeout*1000;
	uint32_t time_counter = 0;
	uint8_t irq_rx_data_ready = 0;
	uint8_t received_bytes = 0;
	uint8_t temp_rssi;
	if(bcm2835_gpio_lev(GPIO_SDN)==HIGH)
	{
		bcm2835_gpio_write(GPIO_SDN, LOW);
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
		bcm2835_delayMicroseconds(950);
		time_counter++;	
		if((time_counter > timeout_ms) && (irq_rx_data_ready == 0))
		{
			irq_rx_data_ready = 2;
		}
			
	}while(irq_rx_data_ready == 0);
	
	if(irq_rx_data_ready == 1)
	{
		SpiritIrqClearStatus();			
		received_bytes = SpiritLinearFifoReadNumElementsRxFifo();
		SpiritSpiReadLinearFifo(received_bytes, rx_buffer);	
		//SpiritCmdStrobeSabort();
		SpiritSpiReadRegisters(0xC8, 1, &temp_rssi);	
		SpiritCmdStrobeFlushRxFifo();
	}
	if(irq_rx_data_ready == 2)
	{
		// rx mode timed out
		// return error value 0
		printf("rx mode timeout\n");
		received_bytes = 255;
	}
	rssi = (int16_t) (temp_rssi/2 - 130);
	return received_bytes;
}

char* create_file(uint8_t device_pointer, char* string)
{
	FILE * fp;
	int i;
	uint16_t length;
	char tmp[FILENAME_LENGTH];
	tmp[0] = '\0';
	//char* tmp = (char*) malloc(FILENAME_LENGTH * sizeof(char));
	//printf("tmp: %s\n", tmp);
	//char* filep = filenames + (device_pointer*FILENAME_LENGTH);
	char* ret;
	char sensor[3];
	//printf("directory: %s\n", directory);
	sprintf(sensor, "%02d", device_pointer);
	strcat(tmp, directory);
	strcat(tmp, "/sensor");
	strcat(tmp, sensor);
	strcat(tmp, "_");	
	strcat(tmp, string);
	strcat(tmp, "_00.csv");
	//printf("String: %s\n", tmp);
	// for debug: additional numbering
	// remove for release
	for (i = 0; i < 100; i++) {
		tmp[40] = i/10 + '0';
		tmp[41] = i%10 + '0';
		if( access( tmp, F_OK ) != -1 ) {
			// file exists
		} else {
			//open file
			break;
		}
	}
	length = strlen(tmp)+1;
	//printf("String: %s\n", tmp);
	//printf("length: %d\n", length);
	ret = (char*) malloc(length * sizeof(char));
	strcpy(ret, tmp);
	//printf("String: %s\n", tmp);
	fp = fopen(ret, "w+");
	fprintf(fp, "time,temperature,pressure,humidity,moisture,time2\n");
	//printf("file created!\n");
	fclose(fp);

	return ret;
}

int save_to_file(uint8_t sensor_to_save, time_t t, char* filenames, uint16_t* temperature, uint16_t* pressure, uint16_t* humidity, float* moisture, uint8_t moisture_received, uint16_t time_difference)
{
	FILE * fp;
	struct tm * ts;
	uint16_t tmp_ui16;
	float tmp_f;
	time_t t_tmp = t;
	uint16_t time_table[MEASURE_VALUES][6] = {0};
	uint8_t moisture_flag = moisture_received;
	uint32_t epoch_time[MEASURE_VALUES] = {0};
	int j;
	float tmp_t, tmp_p, tmp_h, tmp_m;
#ifdef OUTPUT			
	printf("temperature\n");
	for(j=0;j<MEASURE_VALUES;j++)
	{
		printf("%d\t", *(temperature+j));
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
		ts = get_time(&t_tmp);
		epoch_time[j] = (uint32_t) (t_tmp);
		time_table[j][0] = ts->tm_hour; // hour
		time_table[j][1] = ts->tm_min; // min
		time_table[j][2] = ts->tm_sec; // sec
		time_table[j][3] = ts->tm_mday; // day
		time_table[j][4] = ts->tm_mon+1; // month
		time_table[j][5] = ts->tm_year+1900; // year

		//t_tmp = t_tmp - MEASURE_INTERVAL; // go back x seconds to get time of previous value
		tmp_f = floor(time_difference/MEASURE_VALUES);
		t_tmp = t_tmp - (uint32_t)(tmp_f);
	}
	printf("last moisture: %.2f\n", last_moisture);
	// write data to file
	for(j=0;j<MEASURE_VALUES;j++)
	{
		tmp_t = (float) (*(temperature+j)-4000)/100;
		tmp_p = (float) (*(pressure+j))/10;
		tmp_h = (float) (*(humidity+j))/100;
		tmp_m = last_moisture;	// always use newest moisture value for continuous graph
		
		//fprintf(fp, "%d,%d,%d,%d,%d,%d,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
		//fprintf(fp, "%02d:%02d:%02d,%f,%f,%f,%f\n", time_table[j][0], time_table[j][1], time_table[j][2], *(temperature+j), *(pressure+j), *(humidity+j), tmp_f);
		fprintf(fp, "%02d:%02d:%02d,%.2f,%.1f,%.2f,%.2f,%d\n", time_table[j][0], time_table[j][1], time_table[j][2], tmp_t, tmp_p, tmp_h, tmp_m,epoch_time[j]);
		//fprintf(fp, "%d,%d,%d,%d,%d\n", (int)rx_time_array[i], temperature[i][j], pressure[i][j], humidity[i][j], moisture[i][j]);
	}
	
	fclose(fp);
	return 1;
}

float calc_moisture(uint16_t frequency){
	float ret;
	ret = (float) ((7666.2-frequency)/54.89);
	//if(ret<0)
		//ret = 0;
	last_moisture = ret;
	printf("calculated moisture: %.2f\n", ret);
	return ret;
}

char * stringReplace(char *search, char *replace, char *string) {
	char *tempString, *searchStart;
	int len=0;


	// preuefe ob Such-String vorhanden ist
	searchStart = strstr(string, search);
	if(searchStart == NULL) {
		return string;
	}

	// Speicher reservieren
	tempString = (char*) malloc(strlen(string) * sizeof(char));
	if(tempString == NULL) {
		return NULL;
	}

	// temporaere Kopie anlegen
	strcpy(tempString, string);

	// ersten Abschnitt in String setzen
	len = searchStart - string;
	string[len] = '\0';

	// zweiten Abschnitt anhaengen
	strcat(string, replace);

	// dritten Abschnitt anhaengen
	len += strlen(search);
	strcat(string, (char*)tempString+len);

	// Speicher freigeben
	free(tempString);
	
	return string;
}	

struct tm* get_time(time_t * tp)
{
	struct tm * ts;
#if (USE_UTC == 1)
	ts = gmtime(tp);
#else
	ts = localtime(tp);
#endif
	return ts;
}

void init_bcm(void)
{
	srand(time(NULL));		// RNG init
	
    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      //return 1;
    }

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      //return 1;
    }
    
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
	//delay(10);
	
	// RF GPIO 3 (IRQ) rising edge detect
	bcm2835_gpio_fsel(GPIO_RF3, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(GPIO_RF3, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_hen(GPIO_RF3);
	
	// HW pin 15 rising edge detect for button press
	bcm2835_gpio_fsel(GPIO_BUTTON1, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(GPIO_BUTTON1, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(GPIO_BUTTON2, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(GPIO_BUTTON2, BCM2835_GPIO_PUD_DOWN);
	
	// HW pin 16 SPIRIT1 shutdown input toggle
	bcm2835_gpio_fsel(GPIO_SDN, BCM2835_GPIO_FSEL_OUTP);
	
	//reset transceiver via SDN
	bcm2835_gpio_write(GPIO_SDN, HIGH);
	delay(500);
	
	// initialize LED and button voltage
	bcm2835_gpio_fsel(GPIO_LED_GRUEN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_LED_ROT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_LED_GELB, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_LED_BLAU, BCM2835_GPIO_FSEL_OUTP);
	//bcm2835_gpio_fsel(GPIO_V_BUTTON, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(GPIO_LED_GRUEN, HIGH);
	bcm2835_gpio_write(GPIO_LED_ROT, LOW);
	bcm2835_gpio_write(GPIO_LED_GELB, LOW);
	bcm2835_gpio_write(GPIO_LED_BLAU, LOW);
	bcm2835_gpio_write(GPIO_LED_GRUEN, HIGH);
}

