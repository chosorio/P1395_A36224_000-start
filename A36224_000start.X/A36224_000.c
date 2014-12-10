#include "A36224_000.h"
#include "MCP4822.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
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
//Any reason for the numbering?
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

     /*   case STATE_WAITING_FOR_CONFIG:
            ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG);
            while (control_state == STATE_WAITING_FOR_CONFIG) {
              DoA36224_000();
              ETMCanDoCan();

              if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG)) {
                control_state = STATE_OPERATE;
              }
//What should this board do in a fault state? Should it just operate? Does it even really need config?
              if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
                control_state = STATE_STARTUP;
              }
            }
            break;
*/
        case STATE_OPERATE:
            DoA36224_000();
            ETMCanDoCan();

            break;

        default:
            control_state = STATE_STARTUP;
            break;


    }
}

void DoA36224_000(){


    //Convert Flow Meter readings to flow data
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_0);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_1);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_2);
   // ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_3);//These aren't actually used. no need to scale garbage values.
   // ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_flow_4);


    //Convert temperature sensors to digital
    //This will require some work. The sensors are non linear, Maarten says 5th order poly
    //Cabinet
    //Coolant

    // Flash the Refresh
    if (PIN_D_OUT_REFRESH) {
      PIN_D_OUT_REFRESH = 0;
    } else {
      PIN_D_OUT_REFRESH = 1;
    }

    //Convert SF6 pressure sensor to digital
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_000.analog_input_SF6_pressure);

    // -------------------- CHECK FOR FAULTS ------------------- //

 
    if (global_reset_faults) {
      etm_can_system_debug_data.debug_0++;
      etm_can_status_register.status_word_1 = 0x0000;
      global_reset_faults = 0;
    }

    //Set fault if SF6 Pressure too low

    //Can we change the ETMAnalog fault checking so it just checks all of the faults?
    //This board's faults aren't very specific. It just throws if its out of range.
    
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_000.analog_input_SF6_pressure)) {
        //Set Fault if there is not sufficient coolant flow
        ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_SF6_PRESSURE_ANALOG);
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_000.analog_input_cabinet_temp)) {
        //Set Fault if cabinet temperature too high
        ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_CABINET_TEMP_ANALOG);
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_000.analog_input_coolant_temp)) {
        //Set Fault if cabinet temperature too high
        ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_COOLANT_TEMP_ANALOG);
    }
    //Check temperature switch
    //Set fault bit if the switch has opened
    if (PIN_D_IN_1_CABINET_TEMP_SWITCH == CABINET_TEMP_SWITCH_FAULT) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_CABINET_TEMPERATURE_SWITCH);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_1, FAULT_BIT_CABINET_TEMPERATURE_SWITCH);
    }
    //Update Solenoid valve status?

    //Tell solenoid valve to open or close?

}

void InitializeA36224(){


  // Initialize the Analog Input * Output Scaling
  // Dparker need to read from EEPROM ????


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

  global_data_A36224_000.analog_input_coolant_temp.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(COOLANT_TEMP_SCALE_FACTOR);
  global_data_A36224_000.analog_input_coolant_temp.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_coolant_temp.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_coolant_temp.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_coolant_temp.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_coolant_temp.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_cabinet_temp.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(CABINET_TEMP_SCALE_FACTOR);
  global_data_A36224_000.analog_input_cabinet_temp.fixed_offset                    = 0;
  global_data_A36224_000.analog_input_cabinet_temp.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_cabinet_temp.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_input_cabinet_temp.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_input_cabinet_temp.calibration_external_offset     = 0;

  global_data_A36224_000.analog_input_SF6_pressure.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(SF6_PRESSURE_SCALE_FACTOR);
  global_data_A36224_000.analog_input_SF6_pressure.fixed_offset                    = 0;
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

  global_data_A36224_000.analog_output_coolant_thermistor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
  global_data_A36224_000.analog_output_coolant_thermistor.fixed_offset                    = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_offset     = 0;

  global_data_A36224_000.analog_output_coolant_thermistor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
  global_data_A36224_000.analog_output_coolant_thermistor.fixed_offset                    = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_internal_offset     = 0;
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_000.analog_output_coolant_thermistor.calibration_external_offset     = 0;

  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  etm_can_status_register.status_word_0_inhbit_mask = 0b0000000100000100;  // DPARKER move this to #define somewhere
  etm_can_status_register.status_word_1_fault_mask  = 0b0001111111111111;  // DParker move this to #define somewhere


  //Set up outputs as supplies for switches and thermistors
        // Set DAC outputs
       //Analog Out 0
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_cabinet_temp_switch);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_000.analog_output_cabinet_temp_switch.dac_setting_scaled_and_calibrated>>4);
      //Analog Out 1
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_cabinet_thermistor);
      WriteMCP4822(&U44_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_000.analog_output_cabinet_thermistor.dac_setting_scaled_and_calibrated>>4);
      //Analog Out 2
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_000.analog_output_coolant_thermistor);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, global_data_A36224_000.analog_output_coolant_thermistor.dac_setting_scaled_and_calibrated>>4);



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

  // Initialize the CAN module
  ETMCanInitialize();


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
      global_data_A36224_000.analog_input_flow_3.adc_accumulator        += ADCBUF3;
      global_data_A36224_000.analog_input_flow_4.adc_accumulator        += ADCBUF4;
      //global_data_A36224_000.analog_input_flow_5.adc_accumulator      += ADCBUF5;
      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator  += ADCBUF5;
      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator  += ADCBUF6;
      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator  += ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36224_000.analog_input_flow_0.adc_accumulator        += ADCBUF8;
      global_data_A36224_000.analog_input_flow_1.adc_accumulator        += ADCBUF9;
      global_data_A36224_000.analog_input_flow_2.adc_accumulator        += ADCBUFA;
      global_data_A36224_000.analog_input_flow_3.adc_accumulator        += ADCBUFB;
      global_data_A36224_000.analog_input_flow_4.adc_accumulator        += ADCBUFC;
      //global_data_A36224_000.analog_input_flow_5.adc_accumulator      += ADCBUFD;
      global_data_A36224_000.analog_input_coolant_temp.adc_accumulator  += ADCBUFD;
      global_data_A36224_000.analog_input_cabinet_temp.adc_accumulator  += ADCBUFE;
      global_data_A36224_000.analog_input_SF6_pressure.adc_accumulator  += ADCBUFF;
    }


    //etm_can_system_debug_data.debug_0 = ADCBUF0; //Why was this commented out? Ask Dan?
    etm_can_system_debug_data.debug_1 = ADCBUF1;
    etm_can_system_debug_data.debug_2 = ADCBUF2;
    etm_can_system_debug_data.debug_3 = ADCBUF3;
    etm_can_system_debug_data.debug_4 = ADCBUF4;
    etm_can_system_debug_data.debug_5 = ADCBUF5;
    etm_can_system_debug_data.debug_6 = ADCBUF6;
    etm_can_system_debug_data.debug_7 = ADCBUF7;
    etm_can_system_debug_data.debug_8 = ADCBUF8;
    etm_can_system_debug_data.debug_9 = ADCBUF9;
    etm_can_system_debug_data.debug_A = ADCBUFA;
    etm_can_system_debug_data.debug_B = ADCBUFB;
    etm_can_system_debug_data.debug_C = ADCBUFC;
    etm_can_system_debug_data.debug_D = ADCBUFD;
    etm_can_system_debug_data.debug_E = ADCBUFE;
    etm_can_system_debug_data.debug_F = ADCBUFF;


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

      global_data_A36224_000.analog_input_flow_3.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_flow_3.filtered_adc_reading = global_data_A36224_000.analog_input_flow_3.adc_accumulator;
      global_data_A36224_000.analog_input_flow_3.adc_accumulator = 0;

       global_data_A36224_000.analog_input_flow_4.adc_accumulator >>=3;  // This is now a 16 bit number average of previous 256 samples
      global_data_A36224_000.analog_input_flow_4.filtered_adc_reading = global_data_A36224_000.analog_input_flow_4.adc_accumulator;
      global_data_A36224_000.analog_input_flow_4.adc_accumulator = 0;

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




