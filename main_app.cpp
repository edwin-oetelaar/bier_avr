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
// #include "LiquidCrystal.h"
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*/*func*/)()) { return 0; }
void yield(void){};

const uint32_t millis_in_24hr = 60 * 60 * 24 * 1000;
const uint32_t adc_to_volt = 173; /* multiplier: get_adc_value * adc_to_volt = volts (times 1E4) */

// constants won't change. Used here to set a pin number:
const int ledPin = LED_BUILTIN; // the number of the LED pin
// typedef enum LEDS {
//     LED1 = 0,
//     LED2 = 1
// };

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

static volatile uint8_t output_state[2] = {0}; /* 2 leds, OFF per default */
uint8_t prev_output_state[2] = {0};            /* previous state of leds */

static volatile uint8_t device_state = 0; /* after power on, we start here */
static uint8_t teller = 0;                // teller voor in state 1
static uint8_t range_counter = 0;         /* count adc values in range */

// Variables will change:
int ledState = LOW; // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated

// constants won't change:
const long interval = 250; // interval at which to blink (milliseconds)
// const int rs = 12, en = 11, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
// LiquidCrystal lcd; //(void); // rs, en, d4, d5, d6, d7);

uint8_t in_range(int val, int min, int max)
{
    if (val > max)
        return 0;
    if (val < min)
        return 0;
    return 1;
}

void timercallbacks(void)
{ /* dit wordt iedere x ms aangeroepen */
    uint8_t changed = 0;
    /* controleer of user led status veranderde sinds vorige keer
     * een routine mag enkel de globale variablen updaten die worden dan 
     * in deze timer callback ineens naar de outputs doorgestuurd, 
     * een routine mag niet rechtstreeks IO sturen, 
     * de mapping moet in deze functie gebeuren
     */
    for (uint8_t i = 0; i < ARRAY_SIZE(output_state); i++)
    {
        if (prev_output_state[i] != output_state[i])
        {
            changed = 1;
            break;
        }
    }

    /* als nodig zet alle leds aan of uit */
    if (changed)
    {
        /* alle leds aan of uit zetten */
        for (uint8_t i = 0; i < ARRAY_SIZE(output_state); i++)
        {
            if (output_state[i] == 0)
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
            else if (output_state[i] == 1)
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
            prev_output_state[i] = output_state[i];
        }
    }

    // int ch = Serial.read();
    // if (ch < 0)
    // {
    //     /* nothing*/
    // }
    // else
    // {
    //     if (ch == 65)
    //     {
    //         output_state[0] = 0;
    //     }
    // }
}

void setup()
{
    DDRC = 0xFF;  //PC as output
    PORTC = 0x00; //keep all LEDs off
    DDRE = 0x00;  // E is all inputs
    Serial.begin(9600);
    // lcd.begin(16, 2);
    //  lcd.print("hello folks");
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
        /* empty wait 
         * should do something else here instead of busy waiting 
         * */
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

void loop()
{
    // here is where you'd put code that needs to be running all the time.

    unsigned long currentMillis = millis();
    uint8_t ns = device_state; // maintain state if not changed
    switch (device_state)
    {
    case 0:
        /* initial state, init statemachine set outputs to sane value */
        // power on, we doen de blauwe led aan, de fet uit
        output_state[1] = LOW;  // fet
        output_state[0] = HIGH; // led
        ns = 1;
        teller = 0; // teller die we in state 1 gaan gebruiken

        break;

    case 1:
        /* led blinking and power checking state */
        /* check de spanning */
        /* ga naar state 2 als de spanning voor het eerst in range is tussen 10.5 (660) en 14.5 (840)  volt */

        if (currentMillis - previousMillis > 100L)
        {
            // save the last time you blinked the LED
            previousMillis = currentMillis;
            teller += 1; // tel de pulsen van de led
            // if the LED is off turn it on and vice-versa:
            ledState = (ledState == LOW) ? HIGH : LOW;

            output_state[0] = ledState;

            // Serial.println(previousMillis);
            //   lcd.setCursor(0, 1);
            //   lcd.print(previousMillis);
            /* read a value from the ADC input 7 */
            uint16_t adc_value = get_adc_value();
            if (in_range(adc_value, 660, 840))
            {
                // in range wil ik 10 keer zien alvorens door te gaan
                range_counter += 1;
            }
            else
            {
                range_counter = 0; /* we beginnen vooraan met tellen */
            }
            //   lcd.setCursor(8, 1);
            //   lcd.print(adc_value);
            //   lcd.print("    ");
        }

        if (teller > 20 && range_counter > 10)
            ns = 2;

        break;

    case 2:
        if (currentMillis - previousMillis > 1000L)
        {
            // save the last time you blinked the LED, slow blinking
            previousMillis = currentMillis;

            ledState = (ledState == LOW) ? HIGH : LOW;
            output_state[0] = ledState;
            // Serial.println(previousMillis);
            uint16_t adc_value = get_adc_value();
            Serial.println(adc_value * adc_to_volt); // 117813 => 11.7813 Volt
            if (in_range(adc_value, 660, 840))
            {
                // alles ok
                output_state[1] = HIGH; // fet on
            }
            else
            {
                // niet ok, zet alles uit, wacht tot spanning terugkomt
                output_state[1] = LOW; // fet UIT
                ns = 3;
            }
        }

        break;

    case 3: /* power is out of range */
        if (currentMillis - previousMillis > 100L)
        {
            // save the last time you blinked the LED, slow blinking
            previousMillis = currentMillis;

            ledState = (ledState == LOW) ? HIGH : LOW;
            output_state[0] = ledState;
            // Serial.println(previousMillis);
            uint16_t adc_value = get_adc_value();
            Serial.println(adc_value * adc_to_volt); // 117813 => 11.7813 Volt
            if (in_range(adc_value, 660, 840))
            {
                // alles ok ga naar 1
                ns = 0;
            }
            else
            {
                // niet ok, zet alles uit, wacht tot spanning terugkomt
                ns = 3;
            }
        }

        break;
    }

    device_state = ns;
}

/****** main loop *******/

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