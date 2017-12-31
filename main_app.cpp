/*
 * Power hub code
 * made for Ethernet power inserter
 * Designed and programmed by Edwin van den Oetelaar
 * All rights reserved
 * 30-12-2017
 * Cpu used ATMEGA168PB, no crystal, schematic Kicad POE_inserter.sch
 * to be used in BLM project for LDC BV
 * PE3/ADC7 is input voor extern spanning te meten via 15k/1k deler
 * Mosfet at PC1 output
 * LED indicator at PC0 output
 * Functies : 
 * 1) check input spanning in range alvorens BLM in te schakelen
 * 2) timer om BLM te power cyclen iedere 24hr
 */

#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include "wi_private.h"
#include "arduino.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*/*func*/)()) { return 0; }
void yield(void){};

const uint32_t secs_in_24hr = (uint32_t)(60L * 60L * 24L); /* power cycle after so many led blinks */

const uint32_t adc_to_volt = 173; /* multiplier: get_adc_value * adc_to_volt = volts (times 1E4) */

/*
 * PE3/ADC7 is input voor extern spanning
 */

/* led is inverted, pull down to light it up */

#define LED_1_ON       \
    {                  \
        cbi(PORTC, 0); \
    }
#define LED_1_OFF      \
    {                  \
        sbi(PORTC, 0); \
    }

/* fet is not inverted */

#define FET_ON         \
    {                  \
        sbi(PORTC, 1); \
    }
#define FET_OFF        \
    {                  \
        cbi(PORTC, 1); \
    }

static volatile uint8_t output_state[2] = {0}; /* 2 leds, OFF per default */
uint8_t prev_output_state[2] = {0};            /* previous state of leds */

static volatile uint8_t device_state = 0;   /* after power on, we start here */
static uint8_t teller = 0;                  // teller voor in state 1
static uint8_t range_counter = 0;           /* count adc values in range */
static uint32_t sec_counter = secs_in_24hr; /* tel de seconden, bij 0 is het resetten geblazen */
static uint8_t full_reset = 0;              /* flag, if 1 the WDT will reset the MCU */
int ledState = LOW;                         // ledState used to set the LED

unsigned long previousMillis = 0; // will store last time LED was updated

// const long interval = 250; // interval at which to blink (milliseconds)

/*
uint8_t in_range(int val, int min, int max)
range check helper, value between min and max
*/
uint8_t in_range(int val, int min, int max)
{
    if (val > max)
        return 0;
    if (val < min)
        return 0;
    return 1;
}

void timercallbacks(void) /* dit wordt iedere x ms aangeroepen */
{
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
        /* all outputs updated, when changed */
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
                    FET_OFF;
                    break;
                }
            }
            else if (output_state[i] == 1)
            {
                switch (i)
                {
                case 0:
                    LED_1_ON;
                    break;
                case 1:
                    FET_ON;
                    break;
                }
            }
            /* update output internal state */
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

/* user setup, called once on cpu start */
void setup()
{
    wdt_disable();

    DDRC = 0xFF;  //PC as output
    PORTC = 0x00; //keep all outputs LOW
    DDRE = 0x00;  // E is all inputs
    Serial.begin(9600);
    Serial.println("START : BLM pwr hub 1.0");
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
    /* enable watch dog timer ?? */
    wdt_enable(WDTO_500MS); /* 500 ms without wtd_reset will result in reboot ?? check this */
}

uint16_t get_adc_value()
{
    // start the conversion
    sbi(ADCSRA, ADSC);
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC))
    {
        /* empty wait 
         * should do something else here instead of busy waiting, if this hangs, the WDT will save us
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
    unsigned long currentMillis = millis();
    uint8_t ns = device_state; // maintain state if not changed
    uint16_t adc_value;
    switch (device_state)
    {
    case 0:
        /* initial state, init statemachine set outputs to sane value */
        // power on, we doen de blauwe led aan, de fet uit
        output_state[1] = LOW;  // fet
        output_state[0] = HIGH; // led
        ns = 1;
        teller = 0; // teller die we in state 1 gaan gebruiken
        Serial.println("wait for pwr good");
        break;

    case 1:
        /* led blinking and power checking state */
        /* check de spanning, 10x take our time before enable output */
        /* ga naar state 2 als de spanning voor het eerst in range is tussen 10.5 (660) en 14.5 (840)  volt */

        if (currentMillis - previousMillis > 100L)
        {
            // save the last time you blinked the LED
            previousMillis = currentMillis;
            teller += 1; // tel de pulsen van de led
            // if the LED is off turn it on and vice-versa:
            ledState = (ledState == LOW) ? HIGH : LOW;

            output_state[0] = ledState;

            /* read a value from the ADC input 7 */
            adc_value = get_adc_value();
            if (in_range(adc_value, 660, 840))
            {
                // in range wil ik 20 keer zonder onderbreking zien alvorens door te gaan
                range_counter += 1;
            }
            else
            {
                range_counter = 0; /* we beginnen vooraan met tellen */
            }
        }

        /* minimaal 20x knipperen en 10x in range, dan is alles OK */
        if (teller > 30 && range_counter > 20) {
            ns = 2;
            Serial.println("Power : good");
            Serial.println(adc_value * adc_to_volt);
        }

        break;

    case 2: /* normal working state, voltage in range, FET on, Blink SLOW */
        if (currentMillis - previousMillis > 1000L)
        {
            previousMillis += 1000; // currentMillis;

            ledState = (ledState == LOW) ? HIGH : LOW;
            output_state[0] = ledState;
            sec_counter--; /* count seconds down */
            if (sec_counter == 0)
            {
                Serial.println(adc_value * adc_to_volt);
                Serial.println("Timer tripped, reboot now");
                full_reset = 1; /* do not reset WDT anymore, we will do a hardware reset */
            }
            else
                Serial.println(sec_counter);
        }

        /* check input voltage as fast as we can, check value and react quickly 
          112969 TRIPPED
          On output short, polyfuse works, but voltage drops just below 11.3 volts
          We need to have hysteresis, so we trip out low at 8.0 volts => adc=462
          BLM units work at about 7.5 to 18 volts, so this is OK
        */
        adc_value = get_adc_value();

        if (in_range(adc_value, 462, 840))
        {
            // alles ok, input between 9.0 and 14.5 volt
            output_state[1] = HIGH; // fet on
        }
        else
        {
            // niet ok, zet alles uit, wacht tot spanning terugkomt
            output_state[1] = LOW;                   // fet UIT
            Serial.println(adc_value * adc_to_volt); // write trip voltage to serial
            Serial.println("voltage trip");
            ns = 3;
        }
        break;

    case 3: /* power is out of range, blink FAST */
        if (currentMillis - previousMillis > 100L)
        {
            // save the last time you blinked the LED,
            previousMillis = currentMillis;

            ledState = (ledState == LOW) ? HIGH : LOW;
            output_state[0] = ledState;

            adc_value = get_adc_value();
            Serial.println(adc_value * adc_to_volt); // 117813 => 11.7813 Volt
            if (in_range(adc_value, 660, 840))
            {
                // alles ok ga naar 1
                ns = 0;
                Serial.println("pwr returned");
            }
            else
            {
                // niet ok, zet alles uit, wacht tot spanning terugkomt
                // we blijven hier hangen voor eeuwig
                ns = 3;
            }
        }

        break;
    }

    device_state = ns;

    if (! full_reset) 
        wdt_reset(); /* we will live when we get here */
}

/****** main program entry point *******/

int main(int argc, char **argv)
{
    /* set up system hardware, in wiring.c */
    init();

    /* set up user hardware and initialisation, local code */
    setup();

    /* for ever do */
    for (;;)
    {
        /* user code loop, business logic */
        loop();

        /* system handle serials */
        if (serialEventRun)
            serialEventRun();

        /* system background task for timers, sets output, reads inputs  */
        if (timercallbacks)
            timercallbacks();
    }
}