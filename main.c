#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
//#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "SPIRIT_Config.h"
#include "globals.h"

//#include "globals.h"

//static const int CHANNEL = 0;   // wPi chip select

int main()
{
/* 	
	// wiringPi Setup
    if (wiringPiSetup() == -1)
        printf("wiringPi setup OK\n");
    else
		printf("setup failed\n");
*/	
	uint8_t fifo_adress[CIRCULARBUFFER_SIZE*20] = {0};

    int fd,i;
    unsigned char buffer[100]={"abcdefghijklmnopqrstuvwxyz"};
	
    printf("Hello world!\n");

    fd = wiringPiSPISetup(CHANNEL, SPEED);
    
	printf("SPI Setup: %d\n", fd);
	

	wiringPiSPIDataRW(CHANNEL, buffer, 26);
	
	printf("Buffer: %s\n", buffer);
	
	for (i=0;i<26;i++)
	{
		printf("%x  ", buffer[i]);
	}
    
	printf("\nFinish\n");
	uint32_t adress = (uint32_t) &fifo_adress[0];
	printf("FIFO: %x", adress);
    return 0;
    
   
}
