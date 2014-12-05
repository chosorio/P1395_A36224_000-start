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
#define STATE_WAITING_FOR_CONFIG     0x20
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
            control_state=STATE_WAITING_FOR_CONFIG;
            break;
        case STATE_WAITING_FOR_CONFIG:
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

                default:
                    control_state = STATE_STARTUP;
                    break;


    }
}
//Not sure what to do here yet. I'm guessing its a lot of can stuff.
void DoA36224_000(){

    //Convert Flow Meter readings to flow data

    //Set Fault if there is not sufficient water flow

    //Convert temperature sensors to digital
    //Cabinet
    //Coolant
    //Linac

    //Set fault if temperature too high

    //Convert SF6 pressure sensor to digital

    //Set fault if SF6 Pressure too low

    //Check temperature switch
    //Do something with it?

    //Check SF6 Pressure switch
    //Do something with it?

    //Update Solenoid valve status?

    //Tell solenoid valve to open or close?

    //CAN update stuff?
    ETMCanDoCan();
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

  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  etm_can_status_register.status_word_0_inhbit_mask = 0b0000000100000100;  // DPARKER move this to #define somewhere
  etm_can_status_register.status_word_1_fault_mask  = 0b0001111111111111;  // DParker move this to #define somewhere


  //Set up outputs as supplies for switches and thermistors
  /*
   * Need to know which outputs we're using. Right now, its analog out 0 and 1, but I'd rather use digital
   *
   */

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




