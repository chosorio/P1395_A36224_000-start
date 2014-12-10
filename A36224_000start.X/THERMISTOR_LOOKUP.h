#ifndef __THERMISTOR_LOOKUP_H
#define __THERMISTOR_LOOKUP_H

static const int lookup[256];

/*Takes 16 bit digital value from ADC, converts to 16 bit temperature value.
 Temperature output is degrees C * 100.
 */
int ConvertDigitalToTemp(unsigned int);

#endif