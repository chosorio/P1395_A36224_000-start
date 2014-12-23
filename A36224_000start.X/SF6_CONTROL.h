/* 
 * File:   SF6_CONTROL.h
 * Author: cosorio
 *
 * Created on December 19, 2014, 11:49 AM
 */

#ifndef SF6_CONTROL_H
#define	SF6_CONTROL_H

#include "A36224_000.h"

#define SF6_CONTROL_STATE_IDLE      1
#define SF6_CONTROL_STATE_PUMPING   2
#define SF6_CONTROL_STATE_PAUSE     3
#define SF6_CONTROL_STATE_LOCKOUT   4

#define COOLANT_TEMPERATURE         global_data_A36224_000.analog_input_coolant_temp.reading_scaled_and_calibrated
#define SF6_PRESSURE                global_data_A36224_000.analog_input_SF6_pressure.reading_scaled_and_calibrated

#define SF6_PRESSURE_TARGET                         42*100          //42 psi
#define SF6_PRESSURE_CLEAR_LEVEL                    40*100          //Can clear an SF6 fault if above this level.
#define SF6_PRESSURE_LEAK_LEVEL                     30*100          //if less than 30 psi, there is likely a leak, and we don't pump SF6
#define COOLANT_TEMPERATURE_MINIMUM                 (20+273)*100 //Can't pulse SF6 under 20 degrees C

#define COUNTER_5S                                  500
#define SF6_PUMP_COUNT_MAX                          25
void DoSF6Control(void);
void OverrideSF6LowPressure(void);
void OverrideSF6PulseLimit(void);

#endif	/* SF6_CONTROL_H */

