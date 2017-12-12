/*
 * wiringPiSPI.c:
 *	Simplified SPI access routines
 *	Copyright (c) 2012-2015 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "buffer.h"
#include "globals.h"


// The SPI bus parameters
//	Variables as they need to be passed as pointers later on

static const char       *spiDev0  = "/dev/spidev0.0" ;
static const char       *spiDev1  = "/dev/spidev0.1" ;
static const uint8_t     spiBPW   = 8 ;
static const uint16_t    spiDelay = 0 ;

static uint32_t    spiSpeeds [2] ;
static int         spiFds [2] ;


/*
 * wiringPiSPIGetFd:
 *	Return the file-descriptor for the given channel
 *********************************************************************************
 */

int wiringPiSPIGetFd (int channel)
{
  return spiFds [channel & 1] ;
}


/*
 * wiringPiSPIDataRW:
 *	Write and Read a block of data over the SPI bus.
 *	Note the data is being read into the transmit buffer, so will
 *	overwrite it!
 *	This is also a full-duplex operation.
 *********************************************************************************
 */

int wiringPiSPIDataRW (int channel, uint8_t* data, int len)
{
  struct spi_ioc_transfer spi ;

  channel &= 1 ;

// Mentioned in spidev.h but not used in the original kernel documentation
//	test program )-:

  memset (&spi, 0, sizeof (spi)) ;

  spi.tx_buf        = (unsigned long)data ;
  spi.rx_buf        = (unsigned long)data ;
  spi.len           = len ;
  spi.delay_usecs   = spiDelay ;
  spi.speed_hz      = spiSpeeds [channel] ;
  spi.bits_per_word = spiBPW ;

  return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ;
}


/*
 * wiringPiSPISetupMode:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */

int wiringPiSPISetupMode (int channel, int speed, int mode)
{
  int fd ;

  mode    &= 3 ;	// Mode is 0, 1, 2 or 3
  channel &= 1 ;	// Channel is 0 or 1

  if ((fd = open (channel == 0 ? spiDev0 : spiDev1, O_RDWR)) < 0)
    return wiringPiFailure (WPI_ALMOST, "Unable to open SPI device: %s\n", strerror (errno)) ;

  spiSpeeds [channel] = speed ;
  spiFds    [channel] = fd ;

// Set SPI parameters.

  if (ioctl (fd, SPI_IOC_WR_MODE, &mode)            < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI Mode Change failure: %s\n", strerror (errno)) ;
  
  if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI BPW Change failure: %s\n", strerror (errno)) ;

  if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)   < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI Speed Change failure: %s\n", strerror (errno)) ;

  return fd ;
}


/*
 * wiringPiSPISetup:
 *	Open the SPI device, and set it up, etc. in the default MODE 0
 *********************************************************************************
 */

int wiringPiSPISetup (int channel, int speed)
{
  return wiringPiSPISetupMode (channel, speed, 0) ;
}


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
	wiringPiSPISetup(CHANNEL, SPEED);
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
	wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
	
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
	wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
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
	
	wiringPiSPIDataRW(CHANNEL,Buffer_RF.data,Buffer_RF.dataLength);
	
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
			SpiritCmdStrobeSres();
		delay(300);
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

