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

void wPiSPI_Init (void);
void wPiSPI_Deinit (void);
StatusBytesRF wPiSPI_setRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);
StatusBytesRF wPiSPI_getRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);
StatusBytesRF wPiSPI_setRF_Command(uint8_t cCommandCode);
StatusBytesRF wPiSPI_setRF_FIFO(uint8_t* tmp, uint8_t nBytes);
StatusBytesRF wPiSPI_getRF_FIFO(uint8_t* tmp, uint8_t nBytes);
void wPiSPI_init_RF(void);
void spi_checkFIFO_IRQ_RF(void);

#endif /* __SPI_SPIRIT1_H__ */
