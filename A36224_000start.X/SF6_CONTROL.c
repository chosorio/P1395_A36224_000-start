#include "SF6_CONTROL.h"
#include "A36224_000.h"

unsigned int SF6_control_state=SF6_CONTROL_STATE_IDLE;
unsigned int counter_5s=0;
unsigned int SF6_pressure_low_override=0;
unsigned int SF6_pulse_limit_override=0;

void DoSF6Control(void){
        local_debug_data.debug_E = SF6_pressure_low_override;
        
        //Check for faults
        if(COOLANT_TEMPERATURE>=COOLANT_TEMPERATURE_MINIMUM){
                _FAULT_COOLANT_TEMP_UNDER=0;
            if(SF6_PRESSURE<SF6_PRESSURE_LOW_LEVEL){
                _FAULT_SF6_PRESSURE_ANALOG=1;
            }
            else if(SF6_PRESSURE>=SF6_PRESSURE_CLEAR_LEVEL)
            {
                _FAULT_SF6_PRESSURE_ANALOG=0;
            }
        }


    switch(SF6_control_state){
        case SF6_CONTROL_STATE_IDLE:

            PIN_D_OUT_0_SOLENOID_RELAY=OLL_OPEN_RELAY;
            //Coolant must be warmed up before starting SF6 regulation
            if(COOLANT_TEMPERATURE>=COOLANT_TEMPERATURE_MINIMUM){
                if(SF6_PRESSURE<SF6_PRESSURE_CLEAR_LEVEL){
                    if((SF6_PRESSURE>SF6_PRESSURE_LEAK_LEVEL)||(SF6_pressure_low_override>0)){
                    SF6_control_state=SF6_CONTROL_STATE_PUMPING;

                    break;
                    }
                }
            counter_5s=0;
            }
            break;
        case SF6_CONTROL_STATE_PUMPING:
//            if(COOLANT_TEMPERATURE>=COOLANT_TEMPERATURE_MINIMUM){
//                SF6_control_state=SF6_CONTROL_STATE_IDLE;
//            }
            PIN_D_OUT_0_SOLENOID_RELAY=OLL_CLOSE_RELAY;
            counter_5s++;
            

//            if((SF6_PRESSURE<SF6_PRESSURE_LEAK_LEVEL)&&(SF6_pressure_low_override<=0)){
//                SF6_control_state=SF6_CONTROL_STATE_IDLE;
//                break;
//            }
            if(counter_5s>=COUNTER_5S){
                SF6_control_state=SF6_CONTROL_STATE_PAUSE;
                counter_5s=0;
                global_data_A36224_000.SF6_pulse_counter++;
                //need to decrease amount in bottle as well-these are separate variables.
                if( global_data_A36224_000.SF6_bottle_counter>0 && global_data_A36224_000.SF6_bottle_counter<700){
                global_data_A36224_000.SF6_bottle_counter--;
                }
                else{
                    global_data_A36224_000.SF6_bottle_counter=0;
                }
                if(SF6_pressure_low_override>0){
                    SF6_pressure_low_override--;
                }
            }
            break;
        case SF6_CONTROL_STATE_PAUSE:
            PIN_D_OUT_0_SOLENOID_RELAY=OLL_OPEN_RELAY;
            counter_5s++;
            if(global_data_A36224_000.SF6_pulse_counter>=SF6_PUMP_COUNT_MAX){
                if(SF6_PRESSURE>=SF6_PRESSURE_CLEAR_LEVEL){
                    SF6_control_state=SF6_CONTROL_STATE_IDLE;
                    break;
                }
                //set fault bit

                SF6_control_state=SF6_CONTROL_STATE_LOCKOUT;
                break;
            }
            if(SF6_PRESSURE>=SF6_PRESSURE_TARGET){
                SF6_control_state=SF6_CONTROL_STATE_IDLE;
                break;
            }
            else if(counter_5s>COUNTER_5S){
                counter_5s=0;
                if(SF6_PRESSURE<SF6_PRESSURE_LEAK_LEVEL){
                    SF6_control_state=SF6_CONTROL_STATE_IDLE;
                    break;
                }
                SF6_control_state=SF6_CONTROL_STATE_PUMPING;
                break;
            }
            break;
        case SF6_CONTROL_STATE_LOCKOUT:
             PIN_D_OUT_0_SOLENOID_RELAY=OLL_OPEN_RELAY;
            //check to see if fault bit has been cleared.
            if (SF6_pulse_limit_override){
                SF6_control_state=SF6_CONTROL_STATE_IDLE;
                global_data_A36224_000.SF6_pulse_counter=0;
                SF6_pulse_limit_override=0;
            }
            break;
        default:

            break;
    }

}

void OverrideSF6LowPressure(void){
    //I'm not sure when to reset this.
    SF6_pressure_low_override=SF6_PUMP_COUNT_MAX;
    global_data_A36224_000.SF6_pulse_counter=0;
}

void OverrideSF6PulseLimit(void){
    SF6_pulse_limit_override=1;
}