#include "A36224_000.h"
#include "MCP4822.h"
#include "THERMISTOR_LOOKUP.h"
#include "SF6_CONTROL.h"
#include "ETM_EEPROM.h"
#include "FIRMWARE_VERSION.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



void DoStateMachine(void);
void InitializeA36224(void);
void DoA36224_000(void);

MCP4822 U42_MCP4822;
MCP4822 U44_MCP4822;

CoolingControlData global_data_A36224_000;


unsigned int control_state;

#define STATE_STARTUP                0x10
//#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_OPERATE                0x30


int main(void) {

 control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void){
    switch(control_state){
        case STATE_STARTUP:
            InitializeA36224();
            control_state=STATE_OPERATE;
            break;

        case STATE_OPERATE:
            DoA36224_000();
            //ETMCanSlaveDoCan();

            break;

        default:
            control_state = STATE_STARTUP;
            break;
    }
}

void DoA36224_000(){

  if(_T5IF){
    
    _T5IF=0;
    
    //Convert Flow Meter readings to flow data
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_0);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_1);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_2);
    // ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_3);//These aren't actually used. no need to scale garbage values.
    // ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_4);
    
    
    //Convert temperature sensors to digital
    //Uses an 8 bit lookup table, then linear interpolation to fill in the gaps.
    
    //Convert Temperature readings
    //Cabinet
    global_data_A36224_000.analog_input_cabinet_temp.reading_scaled_and_calibrated=ConvertDigitalToTemp(global_data_A36224_000.analog_input_cabinet_temp.filtered_adc_reading);
    //Coolant
    global_data_A36224_000.analog_input_coolant_temp.reading_scaled_and_calibrated=ConvertDigitalToTemp(global_data_A36224_000.analog_input_coolant_temp.filtered_adc_reading);

    // Flash the Refresh
    if (PIN_D_OUT_REFRESH) {
      PIN_D_OUT_REFRESH = 0;
    } else {
      PIN_D_OUT_REFRESH = 1;
    }

    //Convert SF6 pressure sensor to digital

    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_SF6_pressure);

    // -------------------- CHECK FOR FAULTS ------------------- //

    if (_SYNC_CONTROL_RESET_ENABLE) {
     _FAULT_REGISTER = 0x0000;
    }
    
    // Set the fault LED
    if (_CONTROL_NOT_READY) {
      // The board is faulted
        //This turns the light off
        PIN_D_OUT_SPARE_OPTICAL=0;
      PIN_LED_I2_C = 0;
    } else {
        //This turns the light on.
        PIN_D_OUT_SPARE_OPTICAL=1;
        PIN_LED_I2_C = 1;
    }

    unsigned int cabinet_temp_switch=PIN_D_IN_1_CABINET_TEMP_SWITCH;

    if(cabinet_temp_switch==CABINET_TEMP_SWITCH_FAULT){
        _FAULT_CABINET_TEMPERATURE_SWITCH=1;
    }
    else{
        _FAULT_CABINET_TEMPERATURE_SWITCH=0;
    }

    if(global_data_A36224_000.analog_input_coolant_temp.reading_scaled_and_calibrated>=COOLANT_TEMPERATURE_MINIMUM){
        if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_SF6_pressure)) {
        //Set fault if SF6 Pressure too low
            _FAULT_SF6_PRESSURE_ANALOG=1;
        }
    }
    
    if (_FAULT_SF6_PRESSURE_ANALOG==1)
    {
        //We can clear the fault if the SF6 pressure rises above 40psi
        if (global_data_A36224_000.analog_input_SF6_pressure.reading_scaled_and_calibrated>SF6_PRESSURE_CLEAR_LEVEL){
            _FAULT_SF6_PRESSURE_ANALOG=0;
        }
    }
  
    //Set Fault if there is not sufficient coolant flow
     if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_flow_0)) {
         _FAULT_MAGNETRON_COOLANT_FLOW=1;
    }
     else{
         _FAULT_MAGNETRON_COOLANT_FLOW=0;
     }


     if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_flow_1)) {
         _FAULT_LINAC_COOLANT_FLOW=1;
     }
     else{
         _FAULT_LINAC_COOLANT_FLOW=0;
     }

     if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_flow_2)) {
       _FAULT_CIRCULATOR_COOLANT_FLOW=1;
     }
     else{
         _FAULT_CIRCULATOR_COOLANT_FLOW=0;
     }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_000.analog_input_cabinet_temp)) {
        _FAULT_CABINET_TEMP_ANALOG=1;
    }
    else{
        _FAULT_CABINET_TEMP_ANALOG=0;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_000.analog_input_coolant_temp)) {
        _FAULT_COOLANT_TEMP_OVER=1;
    }
    else{
        _FAULT_COOLANT_TEMP_OVER=0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_coolant_temp)) {
        _FAULT_COOLANT_TEMP_UNDER=1;
    }
    else{
        _FAULT_COOLANT_TEMP_UNDER=0;
    }

    //Check temperature switch
    //Set fault bit if the switch has opened
    if (PIN_D_IN_1_CABINET_TEMP_SWITCH == CABINET_TEMP_SWITCH_FAULT) {
        _FAULT_CABINET_TEMPERATURE_SWITCH=1;
    }
    else{
        _FAULT_CABINET_TEMPERATURE_SWITCH=0;
    }
//For this stage of testing, we are just using the board to make sure the magnetron is recieving flow.
    if(_FAULT_MAGNETRON_COOLANT_FLOW){
        _CONTROL_NOT_READY=1;
    }
    else{
        _CONTROL_NOT_READY=0;
    }


#define EEPROM_REGISTER_BOTTLE_COUNTER          0x0210

   DoSF6Control( );
   //Check pulse counter
   //Maybe we should make a pulse counter fault and check that?
   ETMEEPromWriteWord(EEPROM_REGISTER_BOTTLE_COUNTER, global_data_A36224_000.SF6_bottle_counter);
   

    local_debug_data.debug_8 = global_data_A36224_000.analog_input_flow_0.reading_scaled_and_calibrated;
    local_debug_data.debug_9 = global_data_A36224_000.analog_input_flow_1.reading_scaled_and_calibrated;
    local_debug_data.debug_A = global_data_A36224_000.analog_input_flow_2.reading_scaled_and_calibrated;
    local_debug_data.debug_B = global_data_A36224_000.analog_input_coolant_temp.reading_scaled_and_calibrated;
    local_debug_data.debug_C = global_data_A36224_000.analog_input_SF6_pressure.reading_scaled_and_calibrated;
    local_debug_data.debug_D = global_data_A36224_000.SF6_pulse_counter;

    local_debug_data.debug_F = global_data_A36224_000.SF6_bottle_counter;

 
}

}

void InitializeA36224(){


  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;
  
  // Initialize the Analog Input * Output Scaling
  global_data_A36224_000.analog_input_flow_0.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(FLOWMETER_SCALE_FACTOR);
  global_data_A36224_000.analog_input_flow_0.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_flow_0.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_0.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_flow_0.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_0.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_flow_1.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(FLOWMETER_SCALE_FACTOR);
  global_data_A36224_000.analog_input_flow_1.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_flow_1.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_1.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_flow_1.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_1.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_flow_2.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(FLOWMETER_SCALE_FACTOR);
  global_data_A36224_000.analog_input_flow_2.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_flow_2.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_2.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_flow_2.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_2.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_flow_3.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(FLOWMETER_SCALE_FACTOR);
  global_data_A36224_000.analog_input_flow_3.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_flow_3.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_3.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_flow_3.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_flow_3.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_coolant_temp.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(1);
  global_data_A36224_000.analog_input_coolant_temp.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_coolant_temp.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_coolant_temp.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_coolant_temp.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_coolant_temp.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_cabinet_temp.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(1);
  global_data_A36224_000.analog_input_cabinet_temp.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_cabinet_temp.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_cabinet_temp.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_cabinet_temp.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_cabinet_temp.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_SF6_pressure.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(SF6_PRESSURE_SCALE_FACTOR);
  global_data_A36224_000.analog_input_SF6_pressure.fixed_offset                    = SF6_PRESSURE_OFFSET;
  global_data_A36224_000.analog_input_SF6_pressure.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_SF6_pressure.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_SF6_pressure.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_SF6_pressure.calibration_external_offset     = 0;


  global_data_A36224_000.analog_output_coolant_thermistor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
  global_data_A36224_000.analog_output_coolant_thermistor.fixed_offset                    = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_offset     = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.set_point                       = 4096; //4.096 Volts
  global_data_A36224_000.analog_output_coolant_thermistor.enabled                         = 1;

  global_data_A36224_000.analog_output_cabinet_temp_switch.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
  global_data_A36224_000.analog_output_cabinet_temp_switch.fixed_offset                    = 0;
  global_data_A36224_000.analog_output_cabinet_temp_switch.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
  global_data_A36224_000.analog_output_cabinet_temp_switch.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_output_cabinet_temp_switch.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_output_cabinet_temp_switch.calibration_external_offset     = 0;
  global_data_A36224_000.analog_output_cabinet_temp_switch.set_point                       = 8000;
  global_data_A36224_000.analog_output_cabinet_temp_switch.enabled                         = 1;


  global_data_A36224_000.analog_output_cabinet_thermistor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
  global_data_A36224_000.analog_output_cabinet_thermistor.fixed_offset                    = 0;
  global_data_A36224_000.analog_output_cabinet_thermistor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
  global_data_A36224_000.analog_output_cabinet_thermistor.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_output_cabinet_thermistor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_output_cabinet_thermistor.calibration_external_offset     = 0;
  global_data_A36224_000.analog_output_cabinet_thermistor.set_point                       = 4096;
  global_data_A36224_000.analog_output_cabinet_thermistor.enabled                         = 1;

  //Initialize SF6_pulse counter.Maybe this should actually be saved on ECB and sent down during startup.
  global_data_A36224_000.SF6_bottle_counter = ETMEEPromReadWord(EEPROM_REGISTER_BOTTLE_COUNTER);

  _FAULT_REGISTER=0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;


  //Initialize fault values
  global_data_A36224_000.analog_input_cabinet_temp.over_trip_point_absolute=CABINET_TEMPERATURE_OVER_TRIP_POINT;
  global_data_A36224_000.analog_input_coolant_temp.over_trip_point_absolute=COOLANT_TEMPERATURE_OVER_TRIP_POINT;
  global_data_A36224_000.analog_input_flow_0.under_trip_point_absolute=MAGNETRON_FLOW_UNDER_TRIP_POINT;
  global_data_A36224_000.analog_input_flow_1.under_trip_point_absolute=LINAC_FLOW_UNDER_TRIP_POINT;
  global_data_A36224_000.analog_input_flow_2.under_trip_point_absolute=CIRCULATOR_FLOW_UNDER_TRIP_POINT;
  global_data_A36224_000.analog_input_SF6_pressure.under_trip_point_absolute=SF6_PRESSURE_UNDER_TRIP_POINT;


  // Initialize Both MCP4822 DACs
  U42_MCP4822.pin_chip_select_not = _PIN_RD14;
  U42_MCP4822.pin_load_dac_not = _PIN_RF5;
  U42_MCP4822.spi_port = ETM_SPI_PORT_2;
  U42_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
  U42_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
  U42_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
  U42_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
  U42_MCP4822.fcy_clk = FCY_CLK;

  U44_MCP4822.pin_chip_select_not = _PIN_RD15;
  U44_MCP4822.pin_load_dac_not = _PIN_RF5;
  U44_MCP4822.spi_port = U42_MCP4822.spi_port;
  U44_MCP4822.spi_con1_value = U42_MCP4822.spi_con1_value;
  U44_MCP4822.spi_con2_value = U42_MCP4822.spi_con2_value;
  U44_MCP4822.spi_stat_value = U42_MCP4822.spi_stat_value;
  U44_MCP4822.spi_bit_rate = U42_MCP4822.spi_bit_rate;
  U44_MCP4822.fcy_clk = U42_MCP4822.fcy_clk;

  SetupMCP4822(&U42_MCP4822);
  SetupMCP4822(&U44_MCP4822);

  //Set up outputs as supplies for switches and thermistors
        // Set DAC outputs

       //Analog Out 0
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_cabinet_temp_switch);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_000.analog_output_cabinet_temp_switch.dac_setting_scaled_and_calibrated);

      //Analog Out 1
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_cabinet_thermistor);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, global_data_A36224_000.analog_output_cabinet_thermistor.dac_setting_scaled_and_calibrated);

      //Analog Out 2
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_coolant_thermistor);
      WriteMCP4822(&U44_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_000.analog_output_coolant_thermistor.dac_setting_scaled_and_calibrated);



  // Configure Inhibit Interrupt
  //I don't think that inhibit matters for this board. Do we ever turn cooling off?
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition

  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)


  // Initialize all I/O Registers
  TRISA = A36224_TRISA_VALUE;
  TRISB = A36224_TRISB_VALUE;
  TRISC = A36224_TRISC_VALUE;
  TRISD = A36224_TRISD_VALUE;
  TRISF = A36224_TRISF_VALUE;
  TRISG = A36224_TRISG_VALUE;

   // Initialize TMR5
  T5CON = T5CON_VALUE;
  TMR5  = 0;
  _T5IF = 0;
  PR5   = PR5_VALUE_10_MILLISECONDS;

  // Initialize integral ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;




  // Initialize the CAN module
  //ETMCanSlaveInitialize();


  // Flash LEDs at boot up
  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_D = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_C = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_B = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_A = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_FLOW = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_COM = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_WATCHDOG = 1;
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;


    // Copy Data From Buffer to RAM
    if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36224_000.analog_input_flow_0.adc_accumulator        += ADCBUF0;
      global_data_A36224_000.analog_input_flow_1.adc_accumulator        += ADCBUF1;
      global_data_A36224_000.analog_input_flow_2.adc_accumulator        += ADCBUF2;
      //flow 3,4,and 5 are unused.
      //global_data_A36224_000.analog_input_flow_3.adc_accumulator        += ADCBUF3;
      //global_data_A36224_000.analog_input_flow_4.adc_accumulator        += ADCBUF4;
      //global_data_A36224_000.analog_input_flow_5.adc_accumulator      += ADCBUF5;
      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator  += ADCBUF5;
      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator  += ADCBUF6;
      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator  += ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36224_000.analog_input_flow_0.adc_accumulator        += ADCBUF8;
      global_data_A36224_000.analog_input_flow_1.adc_accumulator        += ADCBUF9;
      global_data_A36224_000.analog_input_flow_2.adc_accumulator        += ADCBUFA;
      //global_data_A36224_000.analog_input_flow_3.adc_accumulator        += ADCBUFB;
      //global_data_A36224_000.analog_input_flow_4.adc_accumulator        += ADCBUFC;
      //global_data_A36224_000.analog_input_flow_5.adc_accumulator      += ADCBUFD;
      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator  += ADCBUFD;
      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator  += ADCBUFE;
      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator  += ADCBUFF;
    }

    local_debug_data.debug_1 = ADCBUF1;
    local_debug_data.debug_2 = ADCBUF2;
    local_debug_data.debug_3 = ADCBUF3;
    local_debug_data.debug_4 = ADCBUF4;
    local_debug_data.debug_5 = ADCBUF5;
    local_debug_data.debug_6 = ADCBUF6;
    local_debug_data.debug_7 = ADCBUF7;



    global_data_A36224_000.accumulator_counter += 1;

    //Changed to check after 128. This should make the timing the same as the heater/magnet board ~10ms.
    if (global_data_A36224_000.accumulator_counter >= 128) {

      global_data_A36224_000.analog_input_flow_0.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_flow_0.filtered_adc_reading = global_data_A36224_000.analog_input_flow_0.adc_accumulator;
      global_data_A36224_000.analog_input_flow_0.adc_accumulator = 0;

      global_data_A36224_000.analog_input_flow_1.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_flow_1.filtered_adc_reading = global_data_A36224_000.analog_input_flow_1.adc_accumulator;
      global_data_A36224_000.analog_input_flow_1.adc_accumulator = 0;

      global_data_A36224_000.analog_input_flow_2.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_flow_2.filtered_adc_reading = global_data_A36224_000.analog_input_flow_2.adc_accumulator;
      global_data_A36224_000.analog_input_flow_2.adc_accumulator = 0;
//
//      global_data_A36224_000.analog_input_flow_3.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
//      global_data_A36224_000.analog_input_flow_3.filtered_adc_reading = global_data_A36224_000.analog_input_flow_3.adc_accumulator;
//      global_data_A36224_000.analog_input_flow_3.adc_accumulator = 0;
//
//      global_data_A36224_000.analog_input_flow_4.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
//      global_data_A36224_000.analog_input_flow_4.filtered_adc_reading = global_data_A36224_000.analog_input_flow_4.adc_accumulator;
//      global_data_A36224_000.analog_input_flow_4.adc_accumulator = 0;

      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_coolant_temp.filtered_adc_reading = global_data_A36224_000.analog_input_coolant_temp.adc_accumulator;
      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator = 0;

      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_cabinet_temp.filtered_adc_reading = global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator;
      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator = 0;

      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_SF6_pressure.filtered_adc_reading = global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator;
      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator = 0;

      global_data_A36224_000.accumulator_counter = 0;
    }

  }




