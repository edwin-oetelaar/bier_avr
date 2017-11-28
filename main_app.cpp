// #define __AVR_ATmega48PB__ (1)

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
#include "arduino.h"

/* macro voor bits zetten op IO poorten */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }


// int adc_enable();
// int adc_disable();

// int timer_start();
// int timer_stop();

// int init();
// int watchdog_enable();
// int watchdog_disable();

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

// constants won't change. Used here to set a pin number:
const int ledPin = LED_BUILTIN; // the number of the LED pin

// Variables will change:
int ledState = LOW; // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated

// constants won't change:
const long interval = 1000; // interval at which to blink (milliseconds)

void setup()
{
    // set the digital pin as output:
    pinMode(ledPin, OUTPUT);
}

void loop()
{
    // here is where you'd put code that needs to be running all the time.

    // check to see if it's time to blink the LED; that is, if the difference
    // between the current time and last time you blinked the LED is bigger than
    // the interval at which you want to blink the LED.
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
        {
            ledState = HIGH;
        }
        else
        {
            ledState = LOW;
        }

        // set the LED with the ledState of the variable:
        digitalWrite(ledPin, ledState);
    }
}

int main(int argc, char **argv)
{

    DDRC = 0xFF;  //PB as output
    PORTC = 0x00; //keep all LEDs off

    while (1)
    {
        //PORTC |= 0b00000001; //turn LED on
        sbi(PORTC, 1);
        cbi(PORTC, 0);
        _delay_ms(100); //wait for half second
        //PORTC &= 0b11111110; //turn LED off
        cbi(PORTC, 1);
        sbi(PORTC, 0);
        _delay_ms(100); //wait for half second
    };

    init();

    initVariant();

    setup();

    for (;;)
    {
        loop();
        if (serialEventRun)
            serialEventRun();
    }

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
