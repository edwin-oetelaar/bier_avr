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

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*/*func*/)()) { return 0; }
void yield(void){};

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
    //    pinMode(ledPin, OUTPUT);

    DDRC = 0xFF;  //PB as output
    PORTC = 0x00; //keep all LEDs off
}

int main(int argc, char **argv)
{

    init();

    setup();

    for (;;)
    {
        cbi(PORTC, 0);
        loop();
        sbi(PORTC, 0);
        if (serialEventRun)
            serialEventRun();
    }
}

void loop()
{
    // here is where you'd put code that needs to be running all the time.

    // check to see if it's time to blink the LED; that is, if the difference
    // between the current time and last time you blinked the LED is bigger than
    // the interval at which you want to blink the LED.
    unsigned long currentMillis = millis();
    // delay(100);
    if (currentMillis - previousMillis >= interval)
    {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
        {
            ledState = HIGH;
            sbi(PORTC, 1);
            cbi(PORTC, 0);
        }
        else
        {
            ledState = LOW;
            sbi(PORTC, 0);
            cbi(PORTC, 1);
        }

        // set the LED with the ledState of the variable:
        // digitalWrite(ledPin, ledState);
    }
}
