#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
//#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "SPIRIT_Config.h"
#include "globals.h"
//#include "SPIRIT_PktStack.h"
//#include "MCU_Interface.h"
//#include "SPIRIT_Commands.h"

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
	printf("FIFO: %x\n", adress);
	
	uint8_t test[10] = {0,1,2,3,4,5,6,7,8,9};
	uint8_t* test_p = &test[0];
	
	SpiritPktStackRequireAck(S_DISABLE);
	SpiritCmdStrobeReady();
	SpiritPktBasicSetPayloadLength(30);
	SpiritCmdStrobeFlushTxFifo();
	SpiritRefreshStatus();
	printf("1\n");
	if(g_xStatus.MC_STATE != MC_STATE_READY)
	{
		/* set the ready state */
		SpiritCmdStrobeSabort();
		do
		{
			SpiritRefreshStatus();
			printf("State: %x\n", g_xStatus.MC_STATE);
			delay(500);
		}while(g_xStatus.MC_STATE!=MC_STATE_READY);

	}
	printf("2\n");
		/* clear the Irq */
	SpiritIrqClearStatus();

		/* fit the TX FIFO */
	SpiritCmdStrobeFlushTxFifo();
	SpiritSpiWriteLinearFifo(10, test_p);
	SpiritCmdStrobeTx();
	delay(100);
	SpiritCmdStrobeSabort();
	SpiritRefreshStatus();
	printf("3\n");
	while(0)
	{
		printf("...\n");
		delay(1000);
	}
    return 0;
    
   
}
