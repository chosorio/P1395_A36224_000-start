#ifndef __A36224_H
#define __A36224_H

#include <p30f6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>

#include "ETM_CAN_PUBLIC.h"
#include "ETM_ANALOG.h"
#include "SF6_CONTROL.h"


/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by MCP4822 Modules

  Timer5 - Used for 10msTicToc

  ADC Module - See Below For Specifics

*/



// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN2  - Flow 0 - 
   AN3  - Flow 1 - 
   AN4  - Flow 2 - 
   AN5  - Flow 3 - 
   AN6  - Flow 4 - UNUSED
   AN7  - Flow 5 - UNUSED
   
   AN8  - Analog 0 - Coolant Temperature
   AN9  - Analog 1 - Cabinet Temperature
   AN10 - Analog 2 - SF6 Pressure
   AN11 - Analog 3 - Unused

   
*/
// -------- Analog Input Pins ----------//
#define PIN_A_IN_FLOW_0                         _RB2
#define PIN_A_IN_FLOW_1                         _RB3
#define PIN_A_IN_FLOW_2                         _RB4
#define PIN_A_IN_FLOW_3                         _RB5
#define PIN_A_IN_FLOW_4                         _RB6
//#define PIN_A_IN_FLOW_5                         _RB7  //Not used
#define PIN_A_IN_COOLANT_TEMPERATURE            _RB8
#define PIN_A_IN_CABINET_TEMPERATURE            _RB9
#define PIN_A_IN_SF6_PRESSURE                   _RB10
/*
  This sets up the ADC to work as following
  AUTO Sampling
  VDD / GND as reference
  
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 4 ADC Clock so total sample time is 9uS
  Conversion rate of 111KHz (27.8 Khz per Channel), 277 Samples per 10mS interrupt
  
  8 Samples per Interrupt, use alternating buffers
  

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN) //Why AN3? Shouldn't it just be a ground or negative rail?
#define ADPCFG_SETTING          (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN7 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 &SKIP_SCAN_AN15) //Modified to skip unused inputs





/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)



// -------------------- A36224_000 STATUS BIT CONFIGURATION ------------------------ //
#define STATUS_BIT_SF6_SOLENOID_RELAY_STATE                STATUS_BIT_USER_DEFINED_8



// -------------------- A36224_000 FAULTS/WARNINGS CONFIGURATION-------------------- //
#define FAULT_BIT_MAGNETRON_COOLANT_FLOW         FAULT_BIT_USER_DEFINED_1
#define FAULT_BIT_HVPS_COOLANT_FLOW              FAULT_BIT_USER_DEFINED_2
#define FAULT_BIT_CIRCULATOR_COOLANT_FLOW        FAULT_BIT_USER_DEFINED_3
#define FAULT_BIT_LINAC_COOLANT_FLOW             FAULT_BIT_USER_DEFINED_4
#define FAULT_BIT_HX_COOLANT_FLOW                FAULT_BIT_USER_DEFINED_5
#define FAULT_BIT_CABINET_TEMPERATURE_SWITCH     FAULT_BIT_USER_DEFINED_6

#define FAULT_BIT_CABINET_TEMP_ANALOG            FAULT_BIT_USER_DEFINED_7
#define FAULT_BIT_COOLANT_TEMP_OVER              FAULT_BIT_USER_DEFINED_8
#define FAULT_BIT_COOLANT_TEMP_UNDER             FAULT_BIT_USER_DEFINED_9
#define FAULT_BIT_LINAC_TEMP_ANALOG              FAULT_BIT_USER_DEFINED_10
#define FAULT_BIT_SF6_PRESSURE_SWITCH            FAULT_BIT_USER_DEFINED_11
#define FAULT_BIT_SF6_PRESSURE_ANALOG            FAULT_BIT_USER_DEFINED_12






// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 - Digital in 4
  RA13 - Digital in 4
  RA14 - Digital IN 5
  RA15 - Digital IN 5

  RB12 - Digital IN 0
  RB13 - Digital IN 1-Cabinet Temperature Switch
  RB14 - Digital IN 2
  RB15 - DIgital IN 3

  RF3  - Optical IN - This is Blue Wired

  --------- Pins that are overidden by a hardware module and should be left as inputs during port configuration ----
  RB0 - PROGRAM
  RB1 - PROGRAM

  RB2  - Analog Input - Flow 0
  RB3  - Analog Input - Flow 1
  RB4  - Analog Input - Flow 2
  RB5  - Analog Input - Flow 3
  RB6  - Analog Input - Flow 4 (Unused)
  RB7  - Analog Input - Flow 5 (Unused)
  RB8  - Analog Input - Analog Input 0, Coolant Temperature
  RB9  - Analog Input - Analog Input 1, Cabinet Temperature
  RB10 - Analog Input - Analog Input 2, SF6 Pressure
  RB11 - Analog Input - Analog Input 3, (Unused)

  RG0 - CAN 2
  RG1 - CAN 2
  RG6 - SPI 2
  RG8 - SPI 2

  -------- Pins that are configured by other software modules and should be left as inputs during port configuration -----------
  RD14 (DAC2 SELECT)
  RD15 (DAC1 SELECT)
  RF5  (DAC LDAC)


*/

#define A36224_TRISA_VALUE 0b1111000000000000
#define A36224_TRISB_VALUE 0b1111111111111111
#define A36224_TRISC_VALUE 0b0000000000000000
#define A36224_TRISD_VALUE 0b1100000000000000
#define A36224_TRISF_VALUE 0b0000000000100000
#define A36224_TRISG_VALUE 0b0000000101100011


//   ------------------  Digital Output Pins ---------------
/*

  RC1  - LED
  RC2  - LED
  RC3  - LED
  RC4  - LED

  RC14 - Optical Out

  RD0  - Digital Out 0
  RD1  - Digital Out 1
  RD2  - Digital Out 2
  RD3  - Digital Out 3
  RD4  - Refresh


  RG12 - LED Watchdog
  RG13 - LED COM
  RG14 - LED Power
  RG15 - LED Flow
*/


// -------- Digital Input Pins ----------//
#define PIN_D_IN_0_UNUSED                          _RB12
#define PIN_D_IN_1_CABINET_TEMP_SWITCH             _RB13
#define PIN_D_IN_2_UNUSED                          _RB14
#define PIN_D_IN_3_UNUSED                          _RB15
#define PIN_D_IN_4_UNUSED                          _RA12
#define PIN_D_IN_5_UNUSED                          _RA14
#define PIN_OPTICAL_IN_UNUSED                      _RF3

//#define ILL_POWER_SUPPLY_DISABLED                  1
//#define ILL_RELAY_OPEN                             1
//#define ILL_HEATER_OV                              0
#define CABINET_TEMP_SWITCH_FAULT                   1


// ------- Digital Output Pins ---------//

#define PIN_D_OUT_0_SOLENOID_RELAY                 _LATD0
#define PIN_D_OUT_1_OUTPUT_RELAY_1                 _LATD1
#define PIN_D_OUT_REFRESH                        _LATD4
#define OLL_CLOSE_RELAY                            1
#define OLL_OPEN_RELAY                             0


#define PIN_D_OUT_2_UNUSED                         _LATD2
#define PIN_D_OUT_3_UNUSED                         _LATD3
//#define OLL_ENABLE_SUPPLY                          1

#define PIN_LED_WATCHDOG                           _LATG12
#define PIN_LED_COM                                _LATG13
#define PIN_LED_POWER                              _LATG14
#define PIN_LED_FLOW                               _LATG15
#define PIN_LED_I2_A                               _LATC1
#define PIN_LED_I2_B                               _LATC2
#define PIN_LED_I2_C                               _LATC3
#define PIN_LED_I2_D                               _LATC4


typedef struct {
  // all currents are scaled to 1mA per lsb
  // all voltages are scaled to 1mV per lsb

  AnalogInput analog_input_flow_0;
  AnalogInput analog_input_flow_1;
  AnalogInput analog_input_flow_2;
  AnalogInput analog_input_flow_3; //Not used, but keeping as placeholders
  AnalogInput analog_input_flow_4; //not used, placeholder


  AnalogInput analog_input_coolant_temp;
  AnalogInput analog_input_cabinet_temp;
  AnalogInput analog_input_SF6_pressure;

  AnalogOutput analog_output_cabinet_temp_switch;
  AnalogOutput analog_output_coolant_thermistor;
  AnalogOutput analog_output_cabinet_thermistor;

  unsigned int  accumulator_counter;
  unsigned int SF6_pulse_counter;
  unsigned int SF6_bottle_counter;

} CoolingControlData;

extern CoolingControlData global_data_A36224_000;



#endif

//Scale factors
#define FLOWMETER_SCALE_FACTOR              .1431*3.78541 //Converts to milliliters/minute
#define SF6_PRESSURE_SCALE_FACTOR           .23842 //Tested and working. Converts to centiPSI
#define SF6_PRESSURE_OFFSET                 -10486 //Tested and working
#define ANALOG_OUT_SCALE_FACTOR             .5 //Tested and working. Converts mV to Volts for Analog outputs
#define ANALOG_OUT_INTERNAL_SCALE           1
//Fault Levels
#define CABINET_TEMPERATURE_OVER_TRIP_POINT         (45+273)*100    //Fault at 45 degrees C-> 1/100ths Kelvin
#define COOLANT_TEMPERATURE_OVER_TRIP_POINT         (35+273)*100
#define COOLANT_TEMPERATURE_UNDER_TRIP_POINT        (20+273)*100
#define MAGNETRON_FLOW_UNDER_TRIP_POINT             (.85*5.7*1000) //4.8 liters/minute
#define LINAC_FLOW_UNDER_TRIP_POINT                 (.85*11.3*1000)
#define CIRCULATOR_FLOW_UNDER_TRIP_POINT            (.85*11.3*1000)
#define SF6_PRESSURE_UNDER_TRIP_POINT               38*100          //38 psi
