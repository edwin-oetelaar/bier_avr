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
// #include <math.h>
#include "wi_private.h"
#include "arduino.h"
#include "LiquidCrystal.h"

//#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))
//#define __must_be_array(a) BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
//+ __must_be_array(arr))

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*/*func*/)()) { return 0; }
void yield(void){};

// constants won't change. Used here to set a pin number:
const int ledPin = LED_BUILTIN; // the number of the LED pin
typedef enum LEDS {
    LED1 = 0,
    LED2 = 1
};

// #define LED1PORT PORTC
// #define LED1BIT 0

// #define LED2PORT PORTC
// #define LED2BIT 1

/*
 * PE3/ADC7 is input voor extern spanning
 * MOSI/PB3 is E 
 * MISO/PB4 is RS
 * PD2 is D4
 * PD3 is D5
 * PD4 is D6
 * PD5 is D7
 */

#define LED_1_ON       \
    {                  \
        sbi(PORTC, 0); \
    }
#define LED_1_OFF      \
    {                  \
        cbi(PORTC, 0); \
    }

#define LED_2_ON       \
    {                  \
        sbi(PORTC, 1); \
    }
#define LED_2_OFF      \
    {                  \
        cbi(PORTC, 1); \
    }


typedef enum { OFF = 0,
               ON = 1,
               BLINK = 2,
               BLINK_FAST = 3 } LEDSTATES;

// typedef struct
// {
//     volatile uint8_t *port;
//     uint8_t bit;
// } indicator_t;

// indicator_t indicator[] = {
//     {&PORTC, 0}, /* led 0 */
//     {&PORTC, 1}, /* led 1 */
// };

static volatile uint8_t led_state[2] = {0}; /* 2 leds, OFF per default */
uint8_t prev_led_state[2] = {0};            /* previous state of leds */

// Variables will change:
int ledState = LOW; // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated

// constants won't change:
const long interval = 250; // interval at which to blink (milliseconds)
// const int rs = 12, en = 11, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd; //(void); // rs, en, d4, d5, d6, d7);

void timercallbacks(void)
{ /* dit wordt iedere x ms aangeroepen */
    uint8_t changed = 0;
    /* controleer of user led status veranderde sinds vorige keer */
    for (uint8_t i = 0; i < ARRAY_SIZE(led_state); i++)
    {
        if (prev_led_state[i] != led_state[i])
        {
            changed = 1;
            break;
        }
    }

    /* als nodig zet alle leds aan of uit */
    if (changed)
    {
        /* alle leds aan of uit zetten */
        for (uint8_t i = 0; i < ARRAY_SIZE(led_state); i++)
        {
            if (led_state[i] == 0)
            {
                switch (i)
                {
                case 0:
                    LED_1_OFF;
                    break;
                case 1:
                    LED_2_OFF;
                    break;
                }
                /* zet led uit */
                //cbi(indicator[i].port, indicator[i].bit);
                //cbi(PORTC,1);
            }
            else if (led_state[i] == 1)
            {
                switch (i)
                {
                case 0:
                    LED_1_ON;
                    break;
                case 1:
                    LED_2_ON;
                    break;
                    /* zet led aan */
                    //sbi(indicator[i].port, indicator[i].bit);
                    //sbi(PORTC,1);
                }
            }
            /* update led state */
            prev_led_state[i] = led_state[i];
        }
    }
}
/* 
* ledno => LED1 LED2 
* value => 0:off, 255:on, 1:slow, 2:fast 
* return => 0:succes, -1:bad_led, -2:bad_value
*/
// int8_t set_led_state(uint8_t ledno, uint8_t value)
// {
//     uint8_t p; /* memory ref */
//     uint8_t b;
//     switch (ledno)
//     {

//     case LED1:
//         p = LED1PORT;
//         b = LED1BIT;
//         break;

//     case LED2:
//         p = LED2PORT;
//         b = LED2BIT;
//         break;

//     default:
//         return -2; /* led not found */
//     }

//     switch (value)
//     {

//     case 0: /* uit */
//         cbi(p, b);
//         break;

//     case 1: /* aan */
//         sbi(p, b);
//         break;

//     case 2: /* blink*/
//         break;

//     default:
//         return -1;
//     }

//     //     if (value == 0)
//     //     {
//     //     }
//     //     else if (value == 0xFF)
//     //     {
//     //         /* aan */
//     //     }
//     //     else if (value == 0x01)
//     //     {
//     //         /* slow */
//     //     }
//     //     else if (value == 0x02)
//     //     {
//     //         /* fast */
//     //     }
//     //     else
//     //     {
//     //         /* unsupported led state */
//     //         return -1;
//     //     }
//     // }
//     return 0;
// }

void setup()
{
    DDRC = 0xFF;  //PC as output
    PORTC = 0x00; //keep all LEDs off
    Serial.begin(9600);
    lcd.begin(16,2);
    lcd.print("hello folks");
}

int main(int argc, char **argv)
{

    init();

    setup();

    for (;;)
    {
        //cbi(PORTC, 0);
        loop();
        //sbi(PORTC, 0);
        if (serialEventRun)
            serialEventRun();
        if (timercallbacks)
            timercallbacks();
    }
}

void loop()
{
    // here is where you'd put code that needs to be running all the time.

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > interval)
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

        led_state[0] = ledState;
        Serial.println(previousMillis);
    }
}
