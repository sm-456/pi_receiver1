// SPI interface functions

#define __DECL_SPI_INTERFACE_H__
#include "SPI_interface.h"
#include "globals.h"
#include "buffer.h"
#include "bcm2835.h"

#define true 1
#define false 0

//********************************************************************************
//			SPIRIT1 Interface functions for definition in MCU_Interface
//			added by smoehrin	Nov17
//********************************************************************************		

/*
 * wPiSPI_Deinit:
 *	de-initialize RPi SPI (functionality?)
 *********************************************************************************
 */	

void wPiSPI_Deinit (void)
{
  // nothing
}

/*
 * wPiSPI_Init:
 *	Initialize RPi wiringPi SPI
 * return file-descriptor for channel
 *********************************************************************************
 */

void wPiSPI_Init (void)
{
	//int fd;
	//fd = wiringPiSPISetup(CHANNEL, SPEED);
	//wiringPiSPISetup(CHANNEL, SPEED);
	//return fd;
}

/*
 * wPiSPI_setRF_Data:
 *	handle and set data for RF module
 *********************************************************************************
 */

StatusBytesRF wPiSPI_setRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
	StatusBytesRF status;
	Buffer128_clean(&Buffer_RF);
	
	Buffer_RF.data[0] = WRITE_HEADER;		// set WRITE_HEADER
	Buffer_RF.data[1] = address;			// set address
	Buffer_RF.dataLength = nBytes + 2;
	
	memcpy(&(Buffer_RF.data[2]), &(tmp[0]), nBytes);	// data in buffer
	
	//wPiSPI_startRF_communication();
	//wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
	//bcm2835_spi_transfern(Buffer_RF.data, (uint32_t) Buffer_RF.dataLength);
	bcm2835_spi_writenb(Buffer_RF.data,(uint32_t) Buffer_RF.dataLength);
	
	((uint8_t*)&status)[1]=Buffer_RF.data[0];
	((uint8_t*)&status)[0]=Buffer_RF.data[1];
	
	return status;
}

/*
 * wPiSPI_getRF_Data:
 *	get data from RF module
 *********************************************************************************
 */

StatusBytesRF wPiSPI_getRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
	StatusBytesRF status;
	Buffer128_clean(&Buffer_RF);

	Buffer_RF.data[0] = READ_HEADER;		// set READ_HEADER
	Buffer_RF.data[1] = address;			// set address
	Buffer_RF.dataLength = nBytes + 2;
	
	//wPiSPI_startRF_communication();
	//wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
	bcm2835_spi_transfern(Buffer_RF.data, (uint32_t) Buffer_RF.dataLength);
	//copy data to tmp
	
	//while(Buffer128_allData(&Buffer_RF) == 0){}
	
	memcpy(&tmp[0], &(Buffer_RF.data[2]), nBytes);
	
	((uint8_t*)&status)[1]=Buffer_RF.data[0];
	((uint8_t*)&status)[0]=Buffer_RF.data[1];
	
	return status;
}

/*
 * wPiSPI_setRF_Command:
 *	receive data from RF module
 *********************************************************************************
 */

StatusBytesRF wPiSPI_setRF_Command(uint8_t cCommandCode)
{
	StatusBytesRF status;
	
	Buffer128_clean(&Buffer_RF);
	Buffer_RF.data[0] = COMMAND_HEADER; 		//set COMMAND_HEADER
	Buffer_RF.data[1] = cCommandCode; 		//set cCommandCode
	Buffer_RF.dataLength = 2;				//two Bytes will be sent
	
	//wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
	bcm2835_spi_transfern(Buffer_RF.data, (uint32_t) Buffer_RF.dataLength);
	
	return status;
}

/*
 * wPiSPI_setRF_FIFO:
 *	set FIFO data
 *********************************************************************************
 */

StatusBytesRF wPiSPI_setRF_FIFO(uint8_t* tmp, uint8_t nBytes)
{
	StatusBytesRF status;
	status = wPiSPI_setRF_Data(&(tmp[0]), LINEAR_FIFO_ADDRESS, nBytes);
	
	return status;
}

/*
 * wPiSPI_getRF_FIFO:
 *	get FIFO data
 *********************************************************************************
 */

StatusBytesRF wPiSPI_getRF_FIFO(uint8_t* tmp, uint8_t nBytes)
{
	StatusBytesRF status;
	status = wPiSPI_getRF_Data(&(tmp[0]), LINEAR_FIFO_ADDRESS, nBytes);
	
	return status;
}

void wPiSPI_init_RF(void)
{
	SGpioInit gpio3_Init = {
			SPIRIT_GPIO_3,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_IRQ      /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio2_Init = {
			SPIRIT_GPIO_2,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_FULL      /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio1_Init = {
			SPIRIT_GPIO_1,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_SLEEP_OR_STANDBY       /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio0_Init = {
			SPIRIT_GPIO_0,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_WUT_EXP     /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};


    SpiritBaseConfiguration();

	//SpiritCmdStrobeSabort();

	do
	{ 
		SpiritCmdStrobeSabort();
		SpiritRefreshStatus();
		printf("State: %x\n", g_xStatus.MC_STATE);
		if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
			//delay(1);
			SpiritCmdStrobeSres();
		delay(200);
	}while(g_xStatus.MC_STATE!=MC_STATE_READY);	

	printf("calibrate VCO...\n");
	SpiritVcoCalibration();
	printf("success!\n");
    /* Spirit IRQs enable */
    printf("IRQ deinit...\n");
    SpiritIrqDeInit(NULL);
    printf("success!\n");
//    SpiritIrq(RX_DATA_READY, S_ENABLE);
	printf("Enable IRQ...\n");
    SpiritIrq(TX_DATA_SENT, S_ENABLE);
    printf("success!\n");
//    SpiritIrq(RX_DATA_DISC, S_ENABLE);
//    SpiritIrq(READY, S_ENABLE);
//    SpiritIrq(STANDBY_DELAYED, S_ENABLE);
//    SpiritIrq(LOCK, S_ENABLE);
//    SpiritIrq(AES_END, S_ENABLE);

    /* Init the GPIO-Pin of the RF*/
    SpiritGpioInit(&gpio3_Init);
    SpiritGpioInit(&gpio2_Init);
    SpiritGpioInit(&gpio1_Init);
    SpiritGpioInit(&gpio0_Init);

// Sensor Board GPIO init
/*
    //make the MCU Pins as "only" inputs -> disable the pulldown
	GPIO_setAsInputPin(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    // Configure the Interrupt Edge 
    GPIO_selectInterruptEdge(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT, GPIO_HIGH_TO_LOW_TRANSITION);

    //P1.1 IFG cleared
    GPIO_clearInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    //P1.1 interrupt enabled
//    GPIO_enableInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
//    GPIO_enableInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
//    GPIO_enableInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_enableInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);
/*

    /* IRQ registers blanking */
    SpiritIrqClearStatus();
}

/*============================================================================*/
/*!
    \brief   spi_checkFIFO_IRQ_RF()
 	 	 	 check the FIFO of IRQ_RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
/*
void spi_checkFIFO_IRQ_RF(void)
{
	uint8_t tmp;
	uint8_t cRxData;
	uint8_t vectcRxBuff[96];

	if(CircularBuffer_Out(&tmp, &FIFO_IRQ_RF) == BUFFER_SUCCESS)
	{
		if(tmp == 0xAA)
		{
			//load the Status Registers
			SpiritIrqs irqStatus;
			SpiritIrqGetStatus(&irqStatus);

			//check the Status Registers and do something!
			//after this, clear the Flag
			if((irqStatus.IRQ_RX_DATA_READY) == true)
			{
				// Get the RX FIFO size 
				cRxData=SpiritLinearFifoReadNumElementsRxFifo();

				//Read the RX FIFO 
				SpiritSpiReadLinearFifo(cRxData, &(vectcRxBuff[0]));

				// Flush the RX FIFO 
				SpiritCmdStrobeFlushRxFifo();

				//if no ack has been request from the tx put the device in Rx now 
				if(SpiritPktStackGetReceivedNackRx()!=0)
				{
					SpiritCmdStrobeRx();
				} else
				{
					// go to ready state 
					SpiritCmdStrobeSabort();
				}

			}

			if((irqStatus.IRQ_RX_DATA_DISC) == true)
			{

				//Get the RX FIFO size 
				cRxData=SpiritLinearFifoReadNumElementsRxFifo();

				// Read the RX FIFO 
				SpiritSpiReadLinearFifo(cRxData, &(vectcRxBuff[0]));

				// Flush the RX FIFO 
				SpiritCmdStrobeFlushRxFifo();

				// go to ready state 
				SpiritCmdStrobeSabort();

			}

			if((irqStatus.IRQ_TX_DATA_SENT) == true)
			{
				// set the send flag 
				x_data_sent_flag = 1;

				//flush the TX FIFO
				SpiritCmdStrobeFlushTxFifo();


				//Put it in Ready-Mode
				SpiritCmdStrobeSabort();

			}

			if((irqStatus.IRQ_WKUP_TOUT_LDC) == true)
			{


			}

			if((irqStatus.IRQ_READY) == true)
			{


			}

			if((irqStatus.IRQ_STANDBY_DELAYED) == true)
			{


			}

			if((irqStatus.IRQ_LOCK) == true)
			{

			}

			if((irqStatus.IRQ_AES_END) == true)
			{


			}

			if((irqStatus.IRQ_RX_FIFO_ERROR) == true)
			{
				// Flush the RX FIFO 
				SpiritCmdStrobeFlushRxFifo();
			}

			if((irqStatus.IRQ_TX_FIFO_ERROR) == true)
			{
				// Flush the RX FIFO
//				SpiritCmdStrobeFlushTxFifo();
			}



			SpiritIrqClearStatus();

		}
	}
} // spi_checkFIFO_IRQ_RF()
*/
