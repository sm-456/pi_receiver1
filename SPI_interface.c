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

			SPIRIT_GPIO_DIG_OUT_IRQ     /* Specifies the I/O selection for the selected pins.
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

	printf("Base Config done!\n");
	//SpiritCmdStrobeSabort();

	do
	{ 
		SpiritCmdStrobeSabort();
		SpiritRefreshStatus();
		//printf("State: %x\n", g_xStatus.MC_STATE);
		if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
			//delay(1);
			SpiritCmdStrobeSres();
		//delay(200);
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
    SpiritIrq(RX_DATA_READY, S_ENABLE);
    printf("success!\n");
//    SpiritIrq(RX_DATA_DISC, S_ENABLE);
//    SpiritIrq(READY, S_ENABLE);
//    SpiritIrq(STANDBY_DELAYED, S_ENABLE);
//    SpiritIrq(LOCK, S_ENABLE);
//    SpiritIrq(AES_END, S_ENABLE);

    /* Init the GPIO-Pin of the RF*/
    SpiritGpioInit(&gpio3_Init);
    //SpiritGpioInit(&gpio2_Init);
    //SpiritGpioInit(&gpio1_Init);
    //SpiritGpioInit(&gpio0_Init);


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

void spi_checkFIFO_IRQ_RF(void)
{
	uint8_t tmp;
	uint8_t i;
	uint8_t cRxData;
	uint8_t vectcRxBuff[FIFO_BUFF];

	//if(CircularBuffer_Out(&tmp, &FIFO_IRQ_RF) == BUFFER_SUCCESS)
	if(1)
	{
		//if(tmp == 0xAA)
		if(1)
		{
			//load the Status Registers
			SpiritIrqs irqStatus;
			SpiritIrqGetStatus(&irqStatus);
			printf("IRQ: %X\n", irqStatus.IRQ_RX_DATA_READY);
			//check the Status Registers and do something!
			//after this, clear the Flag
			if((irqStatus.IRQ_RX_DATA_READY) == 1)
			{
				printf("RX data ready!\n");
				// Get the RX FIFO size 
				cRxData = SpiritLinearFifoReadNumElementsRxFifo();

				//Read the RX FIFO 
				SpiritSpiReadLinearFifo(cRxData, &(vectcRxBuff[0]));
				
				for(i=0;i<FIFO_BUFF;i++)
				{
					printf("%X ", vectcRxBuff[i]);
				}
				printf("\n");
				
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

//__________Register_Setting functions__________________________


/* This is the function that initializes the SPIRIT with the configuration 
that the user has exported using the GUI */
void SpiritBaseConfiguration(void)
{
  uint8_t tmp[7];

  /* Be sure that the registers config is default */
  SpiritSpiCommandStrobes(COMMAND_SRES);

  /* Extra current in after power on fix.
     In some samples, when a supply voltage below 2.6 V is applied to SPIRIT1 from a no power condition,
     an extra current is added to the typical current consumption.
     With this sequence, the extra current is erased.
  */
  tmp[0]=0xCA;SpiritSpiWriteRegisters(0xB2, 1, tmp); 
  tmp[0]=0x04;SpiritSpiWriteRegisters(0xA8, 1, tmp); 
  SpiritSpiReadRegisters(0xA8, 1, tmp);
  tmp[0]=0x00;SpiritSpiWriteRegisters(0xA8, 1, tmp);

  tmp[0] = 0xA3; /* reg. GPIO3_CONF (0x02) */
  SpiritSpiWriteRegisters(0x02, 1, tmp);
  
  tmp[0] = 0x36; /* reg. IF_OFFSET_ANA (0x07) */
  tmp[1] = 0x06; /* reg. SYNT3 (0x08) */
  tmp[2] = 0x82; /* reg. SYNT2 (0x09) */
  tmp[3] = 0x8F; /* reg. SYNT1 (0x0A) */
  tmp[4] = 0x59; /* reg. SYNT0 (0x0B) */
  
  tmp[5] = 0x01; /* reg. CH_SPACE (0x0C) */
  //tmp[5] = 0x0E; /* reg. CH_SPACE (0x0C) */
  
  tmp[6] = 0xAC; /* reg. IF_OFFSET_DIG (0x0D) */
  SpiritSpiWriteRegisters(0x07, 7, tmp);
  tmp[0] = 0x01; /* reg. PA_POWER[8] (0x10) */
  SpiritSpiWriteRegisters(0x10, 1, tmp);
  
  	//tmp[0] = 0x87; /* reg. PA_POWER[0] (0x18) */
	//SpiritSpiWriteRegisters(0x18, 1, tmp);
  
  tmp[0] = 0x93; /* reg. MOD1 (0x1A) */
  SpiritSpiWriteRegisters(0x1A, 1, tmp);
  tmp[0] = 0x13; /* reg. CHFLT (0x1D) */
  tmp[1] = 0xC8; /* reg. AFC2 (0x1E) */
  SpiritSpiWriteRegisters(0x1D, 2, tmp);
  tmp[0] = 0x62; /* reg. AGCCTRL1 (0x25) */
  SpiritSpiWriteRegisters(0x25, 1, tmp);
  tmp[0] = 0x15; /* reg. ANT_SELECT_CONF (0x27) */
  SpiritSpiWriteRegisters(0x27, 1, tmp);
  tmp[0] = 0x1B; /* reg. PCKTCTRL2 (0x32) */
  tmp[1] = 0x51; /* reg. PCKTCTRL1 (0x33) */
  SpiritSpiWriteRegisters(0x32, 2, tmp);
  
  
  tmp[0] = 0x00; /* reg. SYNC4 (0x36) */
  tmp[1] = 0x00; /* reg. SYNC3 (0x37) */
  SpiritSpiWriteRegisters(0x36, 2, tmp);
  //tmp[0] = 0x0A; /* reg. PCKTLEN0 (0x35) */
  //tmp[1] = 0x00; /* reg. SYNC4 (0x36) */
  //tmp[2] = 0x00; /* reg. SYNC3 (0x37) */
  //SpiritSpiWriteRegisters(0x35, 3, tmp);
  
  tmp[0] = 0x41; /* reg. PCKT_FLT_OPTIONS (0x4F) */
  tmp[1] = 0x40; /* reg. PROTOCOL[2] (0x50) */
  tmp[2] = 0x01; /* reg. PROTOCOL[1] (0x51) */
  SpiritSpiWriteRegisters(0x4F, 3, tmp);
  tmp[0] = 0x46; /* reg. RCO_VCO_CALIBR_IN[1] (0x6E) */
  tmp[1] = 0x47; /* reg. RCO_VCO_CALIBR_IN[0] (0x6F) */
  SpiritSpiWriteRegisters(0x6E, 2, tmp);
  tmp[0] = 0xA0; /* reg. SYNTH_CONFIG[0] (0x9F) */
  SpiritSpiWriteRegisters(0x9F, 1, tmp);
  tmp[0] = 0x35; /* reg. DEM_CONFIG (0xA3) */
  SpiritSpiWriteRegisters(0xA3, 1, tmp);

  /* VCO unwanted calibration workaround. 
     With this sequence, the PA is on after the eventual VCO calibration expires.
  */
  tmp[0]=0x22;SpiritSpiWriteRegisters(0xBC, 1, tmp);

}

/* This is a VCO calibration routine used to recalibrate the VCO of SPIRIT1 in a safe way.
 IMPORTANT: It must be called from READY state. */
void SpiritVcoCalibration(void)
{
  uint8_t tmp[4];
  uint8_t cal_words[2];
  uint8_t state;
	// state byte: bit 7:1 state, bit 0 XO_ON indicator
	// -> mask state byte with 0xFE to set LSB to zero

    
  SpiritSpiReadRegisters(0x9E, 1, tmp);
  tmp[0] |= 0x80;
  SpiritSpiWriteRegisters(0x9E, 1, tmp); /* REFDIV bit set (to be restored) */

  /* As a consequence we need to double the SYNT word to generate the target frequency */
  tmp[0] = 0x0D;
  tmp[1] = 0x05;
  tmp[2] = 0x1E;
  tmp[3] = 0xB1;
  SpiritSpiWriteRegisters(0x08, 4, tmp);


  tmp[0] = 0x25; SpiritSpiWriteRegisters(0xA1,1,tmp); /* increase VCO current (restore to 0x11) */
  
  SpiritSpiReadRegisters(0x50,1,tmp);
  tmp[0] |= 0x02; 
  SpiritSpiWriteRegisters(0x50,1,tmp); /* enable VCO calibration (to be restored) */
  
  printf("0\n");
  
  //SpiritSpiCommandStrobes(COMMAND_LOCKTX);
  do{
	SpiritSpiCommandStrobes(COMMAND_LOCKTX);
    SpiritSpiReadRegisters(0xC1, 1, &state);
    printf("state(0x1E): %x\n", state&0xFE);
    //delay(100);
  }while((state&0xFE) != 0x1E || (state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
  SpiritSpiReadRegisters(0xE5, 1, &cal_words[0]); /* calib out word for TX */
  printf("1\n");
  
  //SpiritSpiCommandStrobes(COMMAND_READY);
   do{
	SpiritSpiCommandStrobes(COMMAND_READY);
    SpiritSpiReadRegisters(0xC1, 1, &state);
    printf("state(0x06): %x\n", state&0xFE);
  }while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */
  printf("2\n");
  
  //SpiritSpiCommandStrobes(COMMAND_LOCKRX);
  do{
	SpiritSpiCommandStrobes(COMMAND_LOCKRX);
    SpiritSpiReadRegisters(0xC1, 1, &state);
    printf("state(0x1E): %x\n", state&0xFE);
  }while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
  SpiritSpiReadRegisters(0xE5, 1, &cal_words[1]); /* calib out word for RX */
  printf("3\n");
  
  //SpiritSpiCommandStrobes(COMMAND_READY);
   do{
	   SpiritSpiCommandStrobes(COMMAND_READY);
    SpiritSpiReadRegisters(0xC1, 1, &state);
    printf("state(0x06): %x\n", state&0xFE);
  }while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */
  printf("4\n");
  
  SpiritSpiReadRegisters(0x50,1,tmp);
  tmp[0] &= 0xFD; 
  SpiritSpiWriteRegisters(0x50,1,tmp); /* VCO calib restored to 0 */

  SpiritSpiReadRegisters(0x9E, 1, tmp);
  tmp[0] &= 0x7F;
  SpiritSpiWriteRegisters(0x9E, 1, tmp); /* REFDIV bit reset */

  
  tmp[0] = 0x06;
  tmp[1] = 0x82;
  tmp[2] = 0x8F;
  tmp[3] = 0x59;
  SpiritSpiWriteRegisters(0x08, 4, tmp); /* SYNTH WORD restored */

  
  SpiritSpiWriteRegisters(0x6E,2,cal_words); /* write both calibration words */

}

