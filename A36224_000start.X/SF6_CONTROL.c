#include "SF6_CONTROL.h"

unsigned int SF6_control_state=SF6_CONTROL_STATE_IDLE;
unsigned int counter_5s=0;
unsigned int SF6_pressure_low_override=0;

void DoSF6Control(void){
    switch(SF6_control_state){
        case SF6_CONTROL_STATE_IDLE:
            if(COOLANT_TEMPERATURE>=COOLANT_TEMPERATURE_MINIMUM){
                if(SF6_PRESSURE<SF6_PRESSURE_CLEAR_LEVEL){
                    if((SF6_PRESSURE>SF6_PRESSURE_LEAK_LEVEL)||SF6_pressure_low_override){
                    SF6_control_state=SF6_CONTROL_STATE_PUMPING;
                    break;
                    }
                }
            counter_5s=0;
            }
            break;
        case SF6_CONTROL_STATE_PUMPING:
            //PIN_D_OUT_SOLENOID_RELAY=OLL_CLOSE_RELAY;
            counter_5s++;
            if(counter_5s>COUNTER_5S){
                SF6_control_state=SF6_CONTROL_STATE_PAUSE;
                counter_5s=0;
            }
            break;
        case SF6_CONTROL_STATE_PAUSE:
            //PIN_D_OUT_SOLENOID_RELAY=OLL_OPEN_RELAY;
            counter_5s++;
            if(SF6_PRESSURE>SF6_PRESSURE_CLEAR_LEVEL){
                SF6_control_state=SF6_CONTROL_STATE_IDLE;
            }
            if(counter_5s>COUNTER_5S){
                SF6_control_state=SF6_CONTROL_STATE_PUMPING;
            }
            break;

        default:
            break;
            
    }

}

void OverrideSF6LowPressure(void){
    SF6_pressure_low_override=1;
}
