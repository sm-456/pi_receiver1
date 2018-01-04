#ifndef __SPI_INTERFACE_H__
#define __SPI_INTERFACE_H__
#ifndef __DECL_SPI_INTERFACE_H__
#define __DECL_SPI_INTERFACE_H__ extern
#endif

// SPI interface header

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "MCU_Interface.h"
//include "SPIRIT_Config.h"
#include "SPIRIT_Types.h"
#include "SPIRIT_Irq.h"
#include "buffer.h"

#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/
#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/
typedef SpiritStatus StatusBytesRF;

/* list of the command codes of SPIRIT1 */
#define	COMMAND_TX                                          ((uint8_t)(0x60)) /*!< Start to transmit; valid only from READY */
#define	COMMAND_RX                                          ((uint8_t)(0x61)) /*!< Start to receive; valid only from READY */
#define	COMMAND_READY                                       ((uint8_t)(0x62)) /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define	COMMAND_STANDBY                                     ((uint8_t)(0x63)) /*!< Go to STANDBY; valid only from READY */
#define	COMMAND_SLEEP                                       ((uint8_t)(0x64)) /*!< Go to SLEEP; valid only from READY */
#define	COMMAND_LOCKRX                                      ((uint8_t)(0x65)) /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define	COMMAND_LOCKTX                                      ((uint8_t)(0x66)) /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define	COMMAND_SABORT                                      ((uint8_t)(0x67)) /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define	COMMAND_SRES                                        ((uint8_t)(0x70)) /*!< Reset of all digital part, except SPI registers */
#define	COMMAND_FLUSHRXFIFO                                 ((uint8_t)(0x71)) /*!< Clean the RX FIFO; valid from all states */
#define	COMMAND_FLUSHTXFIFO                                 ((uint8_t)(0x72)) /*!< Clean the TX FIFO; valid from all states */

void wPiSPI_Init (void);
void wPiSPI_Deinit (void);
StatusBytesRF wPiSPI_setRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);
StatusBytesRF wPiSPI_getRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);
StatusBytesRF wPiSPI_setRF_Command(uint8_t cCommandCode);
StatusBytesRF wPiSPI_setRF_FIFO(uint8_t* tmp, uint8_t nBytes);
StatusBytesRF wPiSPI_getRF_FIFO(uint8_t* tmp, uint8_t nBytes);
void wPiSPI_init_RF(void);
int spi_checkFIFO_IRQ_RF(void);
void SpiritBaseConfiguration(void);
void SpiritVcoCalibration(void);

#endif /* __SPI_SPIRIT1_H__ */
