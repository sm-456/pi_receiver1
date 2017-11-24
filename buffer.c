//#include <msp430.h>
//#include "driverlib.h"
//#include "globals.h"
#include "buffer.h"
#include <string.h>




/*============================================================================*/
/*!
    \brief    Init Buffer to empty
    \param    Buffer
    \return   None.
*/
/*============================================================================*/
void Buffer_clean(buffer * const p)
{
	p->counter = 0;
	p->dataLength = 0;

}/* Buffer_clean() */

/*============================================================================*/
/*!
    \brief    check if Buffer is empty
    \param    Buffer
    \return   Empty = 0; NotEmpty = 1.
*/
/*============================================================================*/
uint8_t Buffer_NotEmpty(buffer * const p)
{
	if(p->counter == 0)
	{
		return BUFFER_FAIL;
	}

	return BUFFER_SUCCESS;

}/* Buffer_NotEmpty() */

/*============================================================================*/
/*!
    \brief    check if all data are in Buffer
    \param    Buffer
    \return   not all data = 0; all data = 1.
*/
/*============================================================================*/
uint8_t Buffer_allData(buffer * const p)
{
	if((p->dataLength) <= (p->counter))
	{
		return BUFFER_SUCCESS;
	}

	return BUFFER_FAIL;

}/* Buffer_allData() */



/*============================================================================*/
/*!
    \brief    Init Buffer128 to empty
    \param    Buffer
    \return   None.
*/
/*============================================================================*/
void Buffer128_clean(buffer128 * const p)
{

	p->counter = 0;
	p->dataLength = 0;

}/* Buffer128_clean() */

/*============================================================================*/
/*!
    \brief    check if Buffer128 is empty
    \param    Buffer
    \return   Empty = 0; NotEmpty = 1.
*/
/*============================================================================*/
uint8_t Buffer128_NotEmpty(buffer128 * const p)
{
	if(p->counter == 0)
	{
		return BUFFER_FAIL;
	}

	return BUFFER_SUCCESS;

}/* Buffer128_NotEmpty() */


/*============================================================================*/
/*!
    \brief    check if all data are in Buffer128
    \param    Buffer
    \return   not all data = 0; all data = 1.
*/
/*============================================================================*/
uint8_t Buffer128_allData(buffer128 * const p)
{
	if((p->dataLength) <= (p->counter))
	{
		return BUFFER_SUCCESS;
	}

	return BUFFER_FAIL;

}/* Buffer128_allData() */


/*============================================================================*/
/*!
    \brief    Init the Buffer to empty -> recommend after init the Buffer
    \param    Buffer
    \return   None.
*/
/*============================================================================*/
void CircularBuffer_clean(circularBuffer * const p)
{
	p->read = p->write = 0;

}/* Buffer_clean() */


//
// Stellt 1 Byte in den Ringbuffer
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist voll. Es kann kein weiteres Byte gespeichert werden
//     BUFFER_SUCCESS    das Byte wurde gespeichert
//
uint8_t CircularBuffer_In(uint8_t byte, circularBuffer * const p)
{
  uint8_t next = ((p->write + 1) & CIRCULARBUFFER_MASK);

  if (p->read == next)
    return BUFFER_FAIL; // voll

  //p->data[p->write] = byte;
  p->data[p->write & CIRCULARBUFFER_MASK] = byte; // absolut Sicher
  p->write = next;

  return BUFFER_SUCCESS;
}

//
// Holt 1 Byte aus dem Ringbuffer, sofern mindestens eines abholbereit ist
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist leer. Es kann kein Byte geliefert werden.
//     BUFFER_SUCCESS    1 Byte wurde geliefert
//
uint8_t CircularBuffer_Out(uint8_t *pByte, circularBuffer * const p)
{
  if (p->read == p->write)
    return BUFFER_FAIL;

  *pByte = p->data[p->read];

  p->read = (p->read+1) & CIRCULARBUFFER_MASK;

  return BUFFER_SUCCESS;
}

//
// Gibt an, ob der Buffer leer ist
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist leer.
//     BUFFER_SUCCESS    im Ringbuffer befinden sich Daten
//
uint8_t CircularBuffer_NotEmpty(circularBuffer * const p)
{
  if (p->read == p->write)
    return BUFFER_FAIL;

  return BUFFER_SUCCESS;
}
