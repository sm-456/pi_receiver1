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

#define TWITTER 	1				// enable tweet feature
#define SEND_REPEAT 0				// enable send repeat after transmission fault
#define USE_UTC 	0				// 1: use UTC, 0: use local time
#define FIFO 		18
#define SEND_INTERVAL 		600 	// time between transmissions, seconds
#define MEASURE_INTERVAL 	60		// time between measurements, seconds
#define SENSOR_WAKEUP_TIME	60
#define MEASURE_VALUES 		10		// number of values per transmission
#define MOISTURE_VALUES 	1		// values in moisture package

#define TIME_SLOT_DIFF 		300		// offset between slave time slots
#define FILENAME_LENGTH 	50
#define FIRST_SLAVE_OFFSET	30		// first slave has to wait before starting data collection
#define RX_OFFSET			40		// seconds to go RX state before expected data
#define RX_OFFSET_FIRST		120		// offset for first transmission
#define RX_TIMEOUT			90		// timeout of RX mode


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
	
	uint32_t fault_counter = 0;
	uint16_t fault_bytes = 0;
	uint8_t byte = 0;
	uint8_t fault_flag = 0;
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
	char * buffer_string = (char*) malloc(300*sizeof(char));
	char * message = (char*) malloc(300*sizeof(char));
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
			received_bytes = 0;	
			fault_bytes = 0;
			fault_flag = 0;
			
			do{
				received_bytes = receive_data(&(rx_buffer[0]),RX_TIMEOUT);
				bcm2835_delay(10);
			}while(received_bytes == 0);
			
			
			fault_counter = 0;
			byte = 0;
			for(i=0;i<received_bytes;i++)
			{
				sprintf(buffer_string, "%X ", rx_buffer[i]);
				byte = rx_buffer[i]^0xAA;
				if(i==-1)
				{
					printf("%X\n",byte);
				}
				
				for(j=0;j<8;j++)
				{
					tmp_ui8 = byte&0x01;
					if(tmp_ui8 == 1)
					{
						fault_counter++;
						fault_flag = 1;
					}
					byte = byte>>1;
					if(i==-1)
					{
						printf("%d ", tmp_ui8);
					}
				}
				if(fault_flag == 1)
				{
					fault_flag = 0;
					fault_bytes++;
				}
			}
			
			t = time(NULL);
			ts = get_time(&t);
			
			//sprintf(message, "Bytes: %d, RSSI: %d, Fehler: %d", received_bytes, rssi, fault_counter);
			sprintf(message, "%d %d %d %d %02d%02d%02d", received_bytes, rssi, fault_counter, fault_bytes, ts->tm_hour, ts->tm_min, ts->tm_sec);
			printf("%s\n",message);
			fault_counter = 0;
			
			if(TWITTER == 1)
			{
				sprintf(tmp_string, "sudo python3 /home/pi/python/twitter/test.py ");

				strcat(tmp_string, message);
				//printf("%s\n", tmp_string);
				tmp_ui8 = system(tmp_string);
				
				strcpy(tmp_string, "");
				strcpy(buffer_string, "");
				strcpy(message, "");
				
				/*
				if( tmp_ui8 == -1 )
					printf( "Fehler beim Initialisieren der Shell.\n");
				else if( tmp_ui8 > 0 )
					printf( "Tweet erfolgreich, Code %d\n", tmp_ui8 );
				else 
					printf( "Tweet erfolgreich\n" );
				*/
			}
					
				
			bcm2835_gpio_write(GPIO_LED_ROT, LOW);
			
			state = STATE_IDLE;
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

			
		}
		
/*==============================================================================
                                REGISTRATION
 =============================================================================*/		

		if(state == STATE_REGISTRATION)
		{

		}
		
/*==============================================================================
                                BLUETOOTH
 =============================================================================*/		
		if(state == STATE_BT)
		{

		}
/*==============================================================================
                                IDLE
 =============================================================================*/

		if(state == STATE_IDLE)
		{
	
			bcm2835_delay(500);
			t = time(NULL);
			ts = get_time(&t);
			t_int = (uint32_t) t;
			//printf("time: %d\n", t_int);

			// check buttons
			tmp_ui8 = bcm2835_gpio_lev(GPIO_BUTTON1);
			if(tmp_ui8 == 1)
			{
				state = STATE_RX;
				bcm2835_gpio_set_eds(GPIO_BUTTON1);
				//printf("button A pressed!\n");
			}
			else
			{
				tmp_ui8 = bcm2835_gpio_lev(GPIO_BUTTON2);
				if(tmp_ui8 == 1)
				{
					//state = STATE_BT;
					bcm2835_delay(1);
					bcm2835_gpio_set_eds(GPIO_BUTTON2);
					//printf("button B pressed!\n");
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
	uint16_t time_counter = 0;
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
		bcm2835_delay(1);
		
		time_counter++;	
		if(time_counter > timeout_ms)
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
		received_bytes = 0;
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

   
