/**
 * @file    MCU_Interface.h
 * @author  VMA division - AMS
 * @version V2.0.2
 * @date    Febrary 7, 2015
 * @brief   Interface for the low level SPIRIT SPI driver.
 * @details
 * This header file constitutes an interface to the SPI driver used to
 * communicate with Spirit.
 * It exports some function prototypes to write/read registers and FIFOs
 * and to send command strobes.
 * Since the Spirit libraries are totally platform independent, the implementation
 * of these functions are not provided here. The user have to implement these functions
 * taking care to keep the exported prototypes.
 *
 * These functions are:
 *
 * <ul>
 * <li>SdkEvalSpiDeinit</li>
 * <li>SpiritSpiInit</li>
 * <li>SpiritSpiWriteRegisters</li>
 * <li>SpiritSpiReadRegisters</li>
 * <li>SpiritSpiCommandStrobes</li>
 * <li>SpiritSpiWriteLinearFifo</li>
 * <li>SpiritSpiReadLinearFifo</li>
 * </ul>
 *
 * @note An example of SPI driver implementation is available in the <i>Sdk_Eval</i> library.
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_INTERFACE_H
#define __MCU_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
//#include "../../SPI/SPI_SPIRIT1.h"	// SPI for MSP430
//#include "wiringPiSPI.h"		// SPI for RPi
#include "bcm2835.h"
#include "SPIRIT_Types.h"



#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SPIRIT_Libraries
 * @{
 */


/** @defgroup SPIRIT_SPI_Driver         SPI Driver
 * @brief Header file for low level SPIRIT SPI driver.
 * @details See the file <i>@ref MCU_Interface.h</i> for more details.
 * @{
 */



/** @defgroup SPI_Exported_Types        SPI Exported Types
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Constants    SPI Exported Constants
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */

/* removed and placed in spi.h -> PMoser*/
typedef SpiritStatus StatusBytesRF;		// declaration in SPIRIT_Types.h

//void SdkEvalEnterShutdown(void);
//void SdkEvalExitShutdown(void);
//SpiritFlagStatus SdkEvalCheckShutdown(void);

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

//#define SpiritEnterShutdown                                  SdkEvalEnterShutdown
//#define SpiritExitShutdown                                   SdkEvalExitShutdown
//#define SpiritCheckShutdown                                  (SpiritFlagStatus)SdkEvalCheckShutdown


#define SpiritSpiDeinit                                                wPiSPI_Deinit
#define SpiritSpiInit                                 				   wPiSPI_Init
#define SpiritSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)       wPiSPI_setRF_Data(pcBuffer, cRegAddress, cNbBytes)
#define SpiritSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)        wPiSPI_getRF_Data(pcBuffer, cRegAddress, cNbBytes)
#define SpiritSpiCommandStrobes(cCommandCode)                          wPiSPI_setRF_Command(cCommandCode)
#define SpiritSpiWriteLinearFifo(cNbBytes, pcBuffer)                   wPiSPI_setRF_FIFO(pcBuffer, cNbBytes)
#define SpiritSpiReadLinearFifo(cNbBytes, pcBuffer)                    wPiSPI_getRF_FIFO(pcBuffer, cNbBytes)

/**
 * @}
 */

/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
