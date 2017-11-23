/*
 * wiringPiSPI.h:
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

#ifdef __cplusplus
extern "C" {
#endif

#include "MCU_Interface.h"

typedef SpiritStatus StatusBytesRF;

int wiringPiSPIGetFd     (int channel) ;
int wiringPiSPIDataRW    (int channel, unsigned char *data, int len) ;
int wiringPiSPISetupMode (int channel, int speed, int mode) ;
int wiringPiSPISetup     (int channel, int speed) ;

//        SPI functions added by smoehrin

/*============================================================================*/
/*!
    \brief   wPiSPI_Init()
    		 Initialize wiringPi SPI interface
    		 - channel set to 0
    		 - speed set to 500 kHz
    		 - ...
    \param	 None.
    \param	 None.

*/
/*============================================================================*/
void wPiSPI_Init (void);

/*============================================================================*/
/*!
    \brief   wPiSPI_Deinit()
    		 deinit SPI interface
    \param	 None.
    \param	 None.

*/
/*============================================================================*/
void wPiSPI_Deinit (void);

/*============================================================================*/
/*!
    \brief   wPiSPI_setRF_Data()
 	 	 	 send Data to the RF

    \param	 Pointer of data, address of RF-Register (write offset will be set), number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF wPiSPI_setRF_Data(pcBuffer, cRegAddress, cNbBytes);

/*============================================================================*/
/*!
    \brief   wPiSPI_getRF_Data()
 	 	 	 get Data of the RF

    \param	 Pointer for data, address of RF-Register, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF wPiSPI_getRF_Data(pcBuffer, cRegAddress, cNbBytes);

/*============================================================================*/
/*!
    \brief   wPiSPI_setRF_Command()
 	 	 	 Send a command

    \param	 cCommandCode: command code to be sent

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF wPiSPI_setRF_Command(cCommandCode);

/*============================================================================*/
/*!
    \brief   wPiSPI_setRF_FIFO()
 	 	 	 set Data to the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF wPiSPI_setRF_FIFO(pcBuffer, cNbBytes);

/*============================================================================*/
/*!
    \brief   wPiSPI_getRF_FIFO()
 	 	 	 get Data of the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF wPiSPI_getRF_FIFO(pcBuffer, cNbBytes);


#ifdef __cplusplus
}
#endif
