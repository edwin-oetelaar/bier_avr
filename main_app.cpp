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
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

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

    int ch = Serial.read();
    if (ch < 0)
    {
        /* nothing*/
    }
    else
    {
        if (ch = 65)
        {
            led_state[0] = 0;
        }
    }
}

void setup()
{
    DDRC = 0xFF;  //PC as output
    PORTC = 0x00; //keep all LEDs off
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.print("hello folks");
    /* set up ADC for port 7, internal ref
    atmel document 42176 page 334,  ADC Multiplexer Selection Register, Name:  ADMUX, Offset:  0x7C, Reset:  0x00 
    #define MUX0 0
    #define MUX1 1
    #define MUX2 2
    #define MUX3 3
    # 4 is reserved
    #define ADLAR 5
    #define REFS0 6
    #define REFS1 7
      */
    ADMUX = 0b11000111; // intern ref, mux is input ADC7, no ADLAR (left align data)
                        /*

#define ADCSRA  _SFR_MEM8(0x7A)
#define ADPS0   0 // ADC prescaler 0
#define ADPS1   1 // ADC prescaler 1 
#define ADPS2   2 // ADC prescaler 2
#define ADIE    3 // ADC interrupt enable
#define ADIF    4 // ADC interrupt flag 
#define ADATE   5 // ADC auto trigger enable 
#define ADSC    6 // ADC start conversion 
#define ADEN    7 // ADC enable 
ADPS[2:0] => Divsion factor 

000 2
001 2
010 4
011 8
100 16
101 32
110 64
111 128

*/

    // ADC Enable and prescaler of 32
    // 1e6 / 32 = 31250 very slow conversion
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);
}

uint16_t get_adc_value()
{

    // start the conversion
    sbi(ADCSRA, ADSC);
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC))
    {
        /* empty wait */
    };

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    uint8_t low, high;
    low = ADCL;
    high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
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
        lcd.setCursor(0, 1);
        lcd.print(previousMillis);
        /* read a value from the ADC input 7 */
        uint16_t adc_value = get_adc_value();
        lcd.setCursor(8, 1);
        lcd.print(adc_value);
        lcd.print("    ");
    }
}
