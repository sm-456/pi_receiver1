#ifndef __GLOBALS_H__
#define __GLOBALS_H__

//#include <msp430.h>
//#include "driverlib.h"

#include "buffer.h"
//#include "SPIRIT_Config.h"

//#include "bme280_defs.h"


/*extern defines*/
extern circularBuffer FIFO_IRQ_RF;
extern circularBuffer FIFO_TX_RTC;
extern circularBuffer FIFO_RX_RTC;
extern buffer128 Buffer_RF;
extern buffer128 Buffer_RTC;
extern buffer128 Buffer_Sensor;
extern uint8_t x_data_sent_flag;

extern uint16_t cyclic_wakeUpTime;

//extern struct bme280_dev dev;
//extern struct bme280_data data;



//define the "VCC_Button" Pin
#define pin_buttons					GPIO_PORT_P4, GPIO_PIN2
#define pin_enable_buttons()        GPIO_setOutputHighOnPin(pin_buttons)
#define pin_disable_buttons()       GPIO_setOutputLowOnPin(pin_buttons)

//define the DIPs and Buttons
#define pin_DIP1					GPIO_PORT_P3, GPIO_PIN0
#define pin_Value_DIP1()			GPIO_getInputPinValue(pin_DIP1)
#define pin_DIP2					GPIO_PORT_P1, GPIO_PIN2
#define pin_Value_DIP2()			GPIO_getInputPinValue(pin_DIP2)
#define pin_DIP3					GPIO_PORT_P1, GPIO_PIN1
#define pin_Value_DIP3()			GPIO_getInputPinValue(pin_DIP3)
#define pin_DIP4					GPIO_PORT_P1, GPIO_PIN0
#define pin_Value_DIP4()			GPIO_getInputPinValue(pin_DIP4)
#define pin_Button1					GPIO_PORT_P4, GPIO_PIN4			//SW101
#define pin_Value_Button1()			GPIO_getInputPinValue(pin_Button1)
#define pin_Button2					GPIO_PORT_P4, GPIO_PIN5			//SW101
#define pin_Value_Button2()			GPIO_getInputPinValue(pin_Button2)

//define the "StepDown-Control" Pins
#define pin_VSEL1					GPIO_PORT_P3, GPIO_PIN4
#define pin_VSEL2					GPIO_PORT_P3, GPIO_PIN5
#define pin_VSEL3					GPIO_PORT_P3, GPIO_PIN6
#define pin_VSEL4					GPIO_PORT_P3, GPIO_PIN7
#define pin_setLow_VSEL1()       	GPIO_setOutputLowOnPin(pin_VSEL1)
#define pin_setHigh_VSEL1()       	GPIO_setOutputHighOnPin(pin_VSEL1)
#define pin_setLow_VSEL2()        	GPIO_setOutputLowOnPin(pin_VSEL2)
#define pin_setHigh_VSEL2()       	GPIO_setOutputHighOnPin(pin_VSEL2)
#define pin_setLow_VSEL3()        	GPIO_setOutputLowOnPin(pin_VSEL3)
#define pin_setHigh_VSEL3()       	GPIO_setOutputHighOnPin(pin_VSEL3)
#define pin_setLow_VSEL4()        	GPIO_setOutputLowOnPin(pin_VSEL4)
#define pin_setHigh_VSEL4()       	GPIO_setOutputHighOnPin(pin_VSEL4)

// defines the Voltages of the StepDown
#define VOLTAGE_18					0x00	//StepDown "make" 1,8V
#define VOLTAGE_19					0x01
#define VOLTAGE_20					0x02
#define VOLTAGE_21					0x03
#define VOLTAGE_22					0x04
#define VOLTAGE_23					0x05
#define VOLTAGE_24					0x06
#define VOLTAGE_25					0x07
#define VOLTAGE_26					0x08
#define VOLTAGE_27					0x09
#define VOLTAGE_28					0x0A
#define VOLTAGE_29					0x0B
#define VOLTAGE_30					0x0C
#define VOLTAGE_31					0x0D
#define VOLTAGE_32					0x0E
#define VOLTAGE_33					0x0F	//StepDown "make" 3,3V
#define VOLTAGE_default				0x04	//default Voltage, defined by pullUp/-Down at the VSEL Pins of the TPS62740


//define the PowerGood Signals
	//external PG
	#define pin_ePG						GPIO_PORT_P4, GPIO_PIN1
	#define pin_Value_ePG()				GPIO_getInputPinValue(pin_ePG)

	//PG des StepDown
	#define pin_PG						GPIO_PORT_P4, GPIO_PIN3
	#define pin_Value_PG()				GPIO_getInputPinValue(pin_PG)

// wiringPi SPI parameters
#define CHANNEL		0
#define SPEED		500000


#endif
