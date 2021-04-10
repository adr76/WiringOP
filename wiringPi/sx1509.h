/*
 * sx1509.h:
 *	Extend wiringPi with the SX1509 I2C GPIO expander chip
 *	Copyright (c) 2014 Charlie Birks
 */

#ifdef __cplusplus
extern "C" {
#endif

// extern int sx1509Setup(const int pinBase, const int i2cAddress, const int interruptPin, const int resetPin, const int oscillatorPin);
extern int sx1509Setup(const int pinBase, const int i2cAddress);

extern void sx1509LEDDriverSetFreq(int pinBase, int freq, int log);
#ifdef __cplusplus
}
#endif
