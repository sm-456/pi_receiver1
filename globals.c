#include "globals.h"
//#include "driverlib.h"
//#include "bme280_defs.h"


/*Definitions of Buffers*/
circularBuffer FIFO_IRQ_RF;
circularBuffer FIFO_TX_RTC, FIFO_RX_RTC;
buffer128 Buffer_RF;
buffer128 Buffer_RTC;
buffer128 Buffer_Sensor;
//uint8_t fifo_adress[CIRCULARBUFFER_SIZE*20];

uint8_t x_data_sent_flag = 1;	//says, that the data can be send!

uint16_t cyclic_wakeUpTime = 2;	// default: one time per 1 second

//uint8_t fifo_adress[CIRCULARBUFFER_SIZE*20] = {0};

//int LINEAR_FIFO_ADDRESS = &fifo_adress[0];

//struct bme280_dev dev;
//struct bme280_data data;
