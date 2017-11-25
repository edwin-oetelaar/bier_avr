#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <math.h>
// #include "led_monitor.h"
// #include "rc_interface_firmware.h"
// #include "states.h"
// #include "usiTwiSlave.h"
// #include "analog.h"
// #include "potmeter.h"
// #include "push_button.h"

// Some data about the transmitter (desired voltages for both controls,
// next to them are the empirically determined values of the output
// potmeters)(desired voltages = what control potmeters attached to the
// mechanical thingies of the transmitter have on their outputs in the
// extreme positions and neutral):
// STEER (Left, Neutral, Right):
//      L = 0.73 36 -> 30 (for symmetry purpose)
//      N = 2.52 130
//      R = 4.44 230
// THROTTLE (Forward, Neutral, Reverse):
//      R = 1.86 94
//      N = 2.44 125
//      F = 4.22 218
// 
// Next, we will make macro's that we can use to convert the full scale
// int8_t's we get over i2c bus (allowed = -127...0...+127) into the 
// range that is allowed in the transmitter:

// #define THROTTLE_NEUTRAL     125
// #define STEER_NEUTRAL        130

// #define THROTTLE_REVERSE_CORRECTION_FACTOR 0.2440 // = (125 - 94)  / 127
// #define THROTTLE_FORWARD_CORRECTION_FACTOR 0.7322 // = (218 - 125) / 127
// #define STEER_CORRECTION_FACTOR            0.7874 // = (230 - 130) / 127


// Prototypes:
// void common_init(void);
// void battery_check(void);

int main(int argc, char** argv)
{

    DDRC = 0xFF;         //PB as output
    PORTC= 0x00;         //keep all LEDs off

    while(1)
    {
        PORTC &= 0b11111110;       //turn LED off
        _delay_ms(500);   //wait for half second
        PORTC |= 0b00000001;       //turn LED on 
        _delay_ms(500);   //wait for half second
    };        
    // Following pointer will either be coupled to a real struct in the
    // analog handler section (for autonomous mode) or one in the TWI
    // communication handler, it's contents will be updated in the 
    // background so we do not have to worry about them here:
    // volatile struct Controller_values*  controller_values_ptr;
    
    // common_init();
    
	// switch (get_main_state())
    // {
    //     case (AUTONOMOUS): // Transmitter "listens" to normal controls:
    //     {                        
    //         // Special initialization for this state/functionality: 
    //         controller_values_ptr = start_analog_handler(); 
    //         sei(); 
            
    //         // After the following function returns, one can be sure that
    //         // both potmeter values have been read at least once:
    //         wait_for_analog_handler();   
            
    //         // Very important but boring job:    
    //         while(1)
    //         {
    //             battery_check();
                
    //             set_potmeter(STEER_ADDRESS, controller_values_ptr->steer);
    //             set_potmeter(THROTTLE_ADDRESS, controller_values_ptr->throttle);
    //         }
    //     }
    //     case (EXTERNAL_CONTROL): // Transmitter listens to I2C bus:
    //     {            
    //         union 
    //         {
    //             uint8_t unsigned_version;
    //             int8_t signed_version;
    //         } uint8_t_converter;
    //         // Special initialization for this state/function:
    //         start_analog_handler(); // Necessary for battery measurement,
    //                                 // however we do not use the pointer
    //                                 // it returns because in EXTERNAL 
    //                                 // mode the analog inputs are not 
    //                                 // relevant. 
    //         controller_values_ptr = usiTwiSlaveInit();
    //         sei();
    //         wait_for_analog_handler();   
    //         controller_values_ptr->steer    = 0x00;
    //         controller_values_ptr->throttle = 0x00;
    //         controller_values_ptr->IO_stuff = 0x00;
            
    //         // Bit more interesting but then again not really:
    //         while(1)
    //         {                                
    //             uint8_t new_value; // Makes stuff below little clearer...
                
    //             battery_check();        
                
    //             // First we handle the steer:                
    //             uint8_t_converter.unsigned_version = controller_values_ptr->steer;
                
    //             // Weird problem occured, steer direction was inverted, following 
    //             // line solved this:
    //             new_value = (int16_t)STEER_NEUTRAL + (int8_t)(STEER_CORRECTION_FACTOR * (float)uint8_t_converter.signed_version);
                
    //             set_potmeter(STEER_ADDRESS, new_value);                
                 
                
    //             // And then we handle the throttle:
    //             uint8_t_converter.unsigned_version = controller_values_ptr->throttle;
                
    //             new_value = THROTTLE_NEUTRAL + (int8_t)(((uint8_t_converter.signed_version > 0) ? THROTTLE_FORWARD_CORRECTION_FACTOR : THROTTLE_REVERSE_CORRECTION_FACTOR ) * (float)uint8_t_converter.signed_version);
                
    //             set_potmeter(THROTTLE_ADDRESS, new_value);   
                
    //             //// And finally we handle the IO byte that the trackball controller keeps sending us:
    //             if (controller_values_ptr->IO_stuff & IO_AUX_SWITCH_MASK)  
    //             {
    //                 actuate_aux();
    //                 set_aux_state(AUX_ACTIVE);
    //             }
    //             else
    //             {
    //                 release_aux();
    //                 set_aux_state(AUX_INACTIVE);
    //                 // Following is so that led flashing will be consistent
    //                 // when someone presses the button on the transmitter
    //                 // though it is not actuated by the tb controller:
    //                 if (get_button())
    //                     set_aux_state(AUX_ACTIVE);  
    //                 else                        
    //                     set_aux_state(AUX_INACTIVE);
                        
    //             }
    //         }
    //     }
    // }
	
	return 0;
}

// void common_init(void)
// {	
//     // Disable watchdog if enabled by bootloader/fuses:
// 	MCUSR &= ~(1 << WDRF);
// 	wdt_disable();	

//     if (get_button())
//         set_main_state(EXTERNAL_CONTROL);
//     else
//         set_main_state(AUTONOMOUS);
        
//     init_potmeters();
//     start_led_monitor();    
// }

// void battery_check(void)
// {
//     if (get_battery_state() == BATTERY_LOW)
//     {
//         // Stop the car before accidents happen...
//         set_potmeter(STEER_ADDRESS, 0);
//         set_potmeter(THROTTLE_ADDRESS, 0);  
//         while(1);
//     }
// }
