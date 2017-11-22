#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
//#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "SPIRIT_Config.h"

//#include "globals.h"

static const int CHANNEL = 0;   // wPi chip select

int main()
{
/* 	
	// wiringPi Setup
    if (wiringPiSetup() == -1)
        printf("wiringPi setup OK\n");
    else
		printf("setup failed\n");
*/	
	
    int fd,i;
    unsigned char buffer[100]={"test1234"};
	
    printf("Hello world!\n");

    fd = wiringPiSPISetup(CHANNEL, 500000);
    
	printf("SPI Setup: %d\n", fd);
	
	wiringPiSPIDataRW(CHANNEL, buffer, 15);
	
	printf("Buffer: %s\n", buffer);
	
	for (i=0;i<15;i++)
	{
		printf("%x  ", buffer[i]);
	}
	
	printf("\nFinish\n");
	
    return 0;
}
