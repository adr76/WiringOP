/*
 * sx1509.c:
 *	Extend wiringPi with the SX1509 I2C GPIO expander chip
 *	Copyright (c) 2014 Charlie Birks
 */

#include <stdio.h>
#include <pthread.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

#include "sx1509.h"
// #include "sx1509_includes/sx1509_registers.h"

static void reset(struct wiringPiNodeStruct *node, int hardware)
{
	// if hardware bool is set
	if(hardware)
	{
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		int regMisc = wiringPiI2CReadReg8(node->fd, REG_MISC);
		if(regMisc & (1<<2))
		{
			regMisc &= ~(1<<2);
			wiringPiI2CWriteReg8(node->fd, REG_MISC, regMisc);
		}
		// Reset the SX1509, the pin is active low
		pinMode(node->data1, OUTPUT);	// set reset pin as output
		digitalWrite(node->data1, LOW);	// pull reset pin low
		delay(1);	// Wait for the pin to settle
		digitalWrite(node->data1, HIGH);	// pull reset pin back high
	}
	else
	{
		// Software reset command sequence:
		wiringPiI2CWriteReg8(node->fd, REG_RESET, 0x12);
		wiringPiI2CWriteReg8(node->fd, REG_RESET, 0x34);
	}
}

static void ledDriverSetFreq(struct wiringPiNodeStruct *node, int freq, int log)
{
	int tempByte;

	// Enable oscillator (REG_CLOCK)
	tempByte = wiringPiI2CReadReg8(node->fd, REG_CLOCK);
	tempByte |= (1<<6);	// Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1<<5);	// Internal 2MHz oscillator part 2 (clear bit 5)
	wiringPiI2CWriteReg8(node->fd, REG_CLOCK, tempByte);

	// Configure LED driver clock and mode (REG_MISC)
	tempByte = wiringPiI2CReadReg8(node->fd, REG_MISC);
	if(log)
	{
		tempByte |= (1<<7);	// set logarithmic mode bank B
		tempByte |= (1<<3);	// set logarithmic mode bank A
	}
	else
	{
		tempByte &= ~(1<<7);	// set linear mode bank B
		tempByte &= ~(1<<3);	// set linear mode bank A
	}
	if(freq == 0)	// don't want it to be 0, that'll disable all led drivers
		freq = 1;
	freq = (freq & 0x07) << 4;	// freq should only be 3 bits from 6:4
	tempByte |= freq;
	wiringPiI2CWriteReg8(node->fd, REG_MISC, tempByte);

	node->data0 = 1;
}

static void ledDriverInit(struct wiringPiNodeStruct *node, int pin)
{
    int origPin = pin;

	unsigned int tempWord;
	int tempByte;

	pin -= node->pinBase;
    pin = pin < 8 ? pin + 8 : pin - 8; // bytes are swapped

	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	tempWord = wiringPiI2CReadReg16(node->fd, REG_INPUT_DISABLE_B);
	tempWord |= (1<<pin);
	wiringPiI2CWriteReg16(node->fd, REG_INPUT_DISABLE_B, tempWord);

	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	tempWord = wiringPiI2CReadReg16(node->fd, REG_PULL_UP_B);
	tempWord &= ~(1<<pin);
	wiringPiI2CWriteReg16(node->fd, REG_PULL_UP_B, tempWord);

	// Enable open-drain
	// Writing a 1 to the pin bit will enable open drain on that pin
	tempWord = wiringPiI2CReadReg16(node->fd, REG_OPEN_DRAIN_B);
	tempWord |= (1<<pin);
	wiringPiI2CWriteReg16(node->fd, REG_OPEN_DRAIN_B, tempWord);
	
	// Set direction to output (REG_DIR_B)
	pinMode(origPin, OUTPUT);

	// Enable oscillator (REG_CLOCK)
	tempByte = wiringPiI2CReadReg8(node->fd, REG_CLOCK);
	tempByte |= (1<<6);	// Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1<<5);	// Internal 2MHz oscillator part 2 (clear bit 5)
	wiringPiI2CWriteReg8(node->fd, REG_CLOCK, tempByte);

	if(!node->data0)
	    ledDriverSetFreq(node, 1, 0);

	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	tempWord = wiringPiI2CReadReg16(node->fd, REG_LED_DRIVER_ENABLE_B);
	tempWord |= (1<<pin);
	wiringPiI2CWriteReg16(node->fd, REG_LED_DRIVER_ENABLE_B, tempWord);
	
	// Set REG_DATA bit low ~ LED driver started
	tempWord = wiringPiI2CReadReg16(node->fd, REG_DATA_B);
	tempWord &= ~(1<<pin);
	wiringPiI2CWriteReg16(node->fd, REG_DATA_B, tempWord);
}

/*
 * myPinMode:
 *********************************************************************************
 */

static void myPinMode(struct wiringPiNodeStruct *node, int pin, int mode)
{
    unsigned int tempRegDir;

    if(mode == PWM_OUTPUT)
    {
	    ledDriverInit(node, pin);
	    return;
	}

    pin -= node->pinBase;
    pin = pin < 8 ? pin + 8 : pin - 8; // bytes are swapped

	tempRegDir = wiringPiI2CReadReg16(node->fd, REG_DIR_B);
	// The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
	//	0: IO is configured as an output
	//	1: IO is configured as an input
	if(mode == INPUT)
	    tempRegDir |= (1<<pin);
	else
	    tempRegDir &= ~(1<<pin);

	wiringPiI2CWriteReg16(node->fd, REG_DIR_B, tempRegDir);
}


/*
 * myPullUpDnControl:
 *********************************************************************************
 */

static void myPullUpDnControl(struct wiringPiNodeStruct *node, int pin, int mode)
{
	unsigned int tempPullUp = wiringPiI2CReadReg16(node->fd,REG_PULL_UP_B);
	unsigned int tempPullDown = wiringPiI2CReadReg16(node->fd,REG_PULL_DOWN_B);

    pin -= node->pinBase;
    pin = pin < 8 ? pin + 8 : pin - 8; // bytes are swapped

	if(mode == PUD_UP)	// pull-up, disable pull-down
	{
		tempPullUp |= (1<<pin);
		tempPullDown &= ~(1<<pin);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_UP_B, tempPullUp);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_DOWN_B, tempPullDown);
	}
	else if(mode == PUD_DOWN)	// pull-down, disable pull-up
	{
		tempPullDown |= (1<<pin);
		tempPullUp &= ~(1<<pin);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_UP_B, tempPullUp);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_DOWN_B, tempPullDown);
	}
	else    // disable both
	{
		tempPullUp &= ~(1<<pin);
		tempPullDown &= ~(1<<pin);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_UP_B, tempPullUp);
		wiringPiI2CWriteReg16(node->fd, REG_PULL_DOWN_B, tempPullDown);
	}
}


/*
 * myDigitalRead:
 *********************************************************************************
 */

static int myDigitalRead(struct wiringPiNodeStruct *node, int pin)
{
	unsigned int tempRegDir = wiringPiI2CReadReg16(node->fd, REG_DIR_B);

    pin -= node->pinBase;
    pin = pin < 8 ? pin + 8 : pin - 8; // bytes are swapped

	if(tempRegDir & (1<<pin))	// If the pin is an input
	{
		unsigned int tempRegData = wiringPiI2CReadReg16(node->fd, REG_DATA_B);
		if(tempRegData & (1<<pin))
			return 1;
	}
	
	return 0;
}


/*
 * myDigitalWrite:
 *********************************************************************************
 */

static void myDigitalWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
	unsigned int tempRegDir = wiringPiI2CReadReg16(node->fd, REG_DIR_B);

    pin -= node->pinBase;
    pin = pin < 8 ? pin + 8 : pin - 8; // bytes are swapped

	if((0xFFFF^tempRegDir)&(1<<pin))	// If the pin is an output, write high/low
	{
        unsigned int tempRegData = wiringPiI2CReadReg16(node->fd, REG_DATA_B);
        if(value)
            tempRegData |= (1<<pin);
        else
            tempRegData &= ~(1<<pin);

        wiringPiI2CWriteReg16(node->fd, REG_DATA_B, tempRegData);
    }
}


static void myPWMWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
    wiringPiI2CWriteReg8(node->fd, REG_I_ON[pin - node->pinBase], value);
}

//  ******************************************************************************
//  Export Functions
//  ******************************************************************************

/*
 * sx1509Setup:
 *	Create a new instance of an SX1509 I2C GPIO interface.
 *********************************************************************************
 */

// int sx1509Setup(const int pinBase, const int i2cAddress, const int interruptPin, const int resetPin, const int oscillatorPin)
int sx1509Setup(const int pinBase, const int i2cAddress)
{
    int fd;
    struct wiringPiNodeStruct *node;
    unsigned int testRegisters = 0;

    if((fd = wiringPiI2CSetup(i2cAddress)) < 0)
        return fd;

    node = wiringPiNewNode(pinBase, 16);

    node->fd              = fd;
    node->pinMode         = myPinMode;
    node->pullUpDnControl = myPullUpDnControl;
    node->digitalRead     = myDigitalRead;
    node->digitalWrite    = myDigitalWrite;
    node->pwmWrite        = myPWMWrite;

    node->data0           = 0;
    node->data1           = 0;  //resetPin;
    node->data2           = -1; //interruptPin;
    node->data3           = -1; //oscillatorPin;

    // if(interruptPin != -1)
    // {
	// 	pinMode(interruptPin, INPUT);
	// 	pullUpDnControl(interruptPin, PUD_UP);
	// }

	// If the reset pin is connected
	// if(resetPin != -1)
		// reset(node, 1);
	// else
	// Soft Reset Registers
	reset(node, 0);

	// Communication test. We'll read from two registers with different
	// default values to verify communication.
	testRegisters = wiringPiI2CReadReg16(fd, REG_INTERRUPT_MASK_A);	// This should return 0xFF00

	// Then read a byte that should be 0x00
	if(testRegisters == 0x00FF)
		return 0;
	else
		return -1;
}

void sx1509Reset(int pinBase)
{
	// Software reset:
	reset(wiringPiFindNode(pinBase), 0);
}

void sx1509LEDDriverSetFreq(int pinBase, int freq, int log)
{
    ledDriverSetFreq(wiringPiFindNode(pinBase), freq, log);
}
