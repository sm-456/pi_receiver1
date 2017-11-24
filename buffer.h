#ifndef __BUFFER_H__
#define __BUFFER_H__

//#include "driverlib.h"
#include <stdint.h>

//Defines
#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1

#define CIRCULARBUFFER_SIZE 4 // muss 2^n betragen (8, 16, 32, 64 ...)
#define CIRCULARBUFFER_MASK (CIRCULARBUFFER_SIZE-1) // Klammern auf keinen Fall vergessen

#define BUFFER128_SIZE 128
#define BUFFER_SIZE_RF 100


//Typedefs
typedef struct {
  uint8_t data[CIRCULARBUFFER_SIZE];
  uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
  uint8_t write; // zeigt immer auf leeres Feld
} circularBuffer;

typedef struct {
  uint8_t data[BUFFER_SIZE_RF];
  uint8_t dataLength;
  uint8_t counter;
} buffer;

typedef struct {
  uint8_t data[BUFFER128_SIZE];
  uint8_t dataLength;
  uint8_t counter;
} buffer128;

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/** \defgroup buffer_api buffer controller API */
/* @{ */


/*============================================================================*/
/*!
    \brief    Init Buffer to empty
    \param    Buffer
    \return   None.
*/
/*============================================================================*/
void Buffer_clean(buffer * const p);


/*============================================================================*/
/*!
    \brief    check if Buffer is empty
    \param    Buffer
    \return   Empty = 0; NotEmpty = 1.
*/
/*============================================================================*/
uint8_t Buffer_NotEmpty(buffer * const p);

/*============================================================================*/
/*!
    \brief    check if all data are in Buffer
    \param    Buffer
    \return   not all data = 0; all data = 1.
*/
/*============================================================================*/
uint8_t Buffer_allData(buffer * const p);


/*============================================================================*/
/*!
    \brief    Init the Buffer128 to empty
    \param    None.
    \return   None.
*/
/*============================================================================*/
void Buffer128_clean();


/*============================================================================*/
/*!
    \brief    check if Buffer128 is empty
    \param    Buffer.
    \return   Empty = 0; NotEmpty = 1.
*/
/*============================================================================*/
uint8_t Buffer128_NotEmpty(buffer128 * const p);

/*============================================================================*/
/*!
    \brief    check if all data are in Buffer128
    \param    Buffer
    \return   not all data = 0; all data = 1.
*/
/*============================================================================*/
uint8_t Buffer128_allData(buffer128 * const p);

/*============================================================================*/
/*!
    \brief    Init the Buffer to empty -> recommend after init the Buffer
    \param    Buffer
    \return   None.
*/
/*============================================================================*/
void CircularBuffer_clean(circularBuffer * const p);



/* @} */

/*============================================================================*/

uint8_t CircularBuffer_In(uint8_t byte, circularBuffer * const p);
uint8_t CircularBuffer_Out(uint8_t *pByte, circularBuffer * const p);
uint8_t CircularBuffer_NotEmpty(circularBuffer *const p);

#endif
