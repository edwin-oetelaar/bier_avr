#include "LiquidCrystal.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "arduino.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal::LiquidCrystal()
{
  init();
}

void LiquidCrystal::init()

{

  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

  begin(16, 1);
}

void LiquidCrystal::begin(uint8_t cols, uint8_t lines, uint8_t dotsize)
{
  if (lines > 1)
  {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

  /*
 * PE3/ADC7 is input voor extern spanning
 * MOSI/PB3 is E 
 * MISO/PB4 is RS
 * PD2 is D4
 * PD3 is D5
 * PD4 is D6
 * PD5 is D7
 */

  DDRB |= 0b00011000; /* bit 3 en 4 zijn output */
  DDRD |= 0b00111100; /* bits 2,3,4,5 zijn output */

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  delayMicroseconds(50000);

  // Now we pull both RS and R/W low to begin commands
  PIN_RS_LOW;
  PIN_E_LOW;

  //put the LCD into 4 bit or 8 bit mode
  // if (! (_displayfunction & LCD_8BITMODE)) {
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(0x03);
  delayMicroseconds(4500); // wait min 4.1ms

  // second try
  write4bits(0x03);
  delayMicroseconds(4500); // wait min 4.1ms

  // third go!
  write4bits(0x03);
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  write4bits(0x02);

  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);
}

void LiquidCrystal::setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */
void LiquidCrystal::clear()
{
  command(LCD_CLEARDISPLAY); // clear display, set cursor position to zero
  delayMicroseconds(2000);   // this command takes a long time!
}

void LiquidCrystal::home()
{
  command(LCD_RETURNHOME); // set cursor position to zero
  delayMicroseconds(2000); // this command takes a long time!
}

void LiquidCrystal::setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if (row >= max_lines)
  {
    row = max_lines - 1; // we count rows starting w/0
  }
  if (row >= _numlines)
  {
    row = _numlines - 1; // we count rows starting w/0
  }

  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
// void LiquidCrystal::noDisplay()
// {
//   _displaycontrol &= ~LCD_DISPLAYON;
//   command(LCD_DISPLAYCONTROL | _displaycontrol);
// }
void LiquidCrystal::display()
{
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystal::noCursor()
{
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal::cursor()
{
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystal::noBlink()
{
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal::blink()
{
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal::command(uint8_t value)
{
  send(value, LOW);
}

inline size_t LiquidCrystal::write(uint8_t value)
{
  send(value, HIGH);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void LiquidCrystal::send(uint8_t value, uint8_t mode)
{
  if (mode)
    PIN_RS_HIGH
  else
    PIN_RS_LOW

  write4bits(value >> 4);
  write4bits(value);
}

void LiquidCrystal::pulseEnable(void)
{

  PIN_E_LOW;
  delayMicroseconds(1);
  PIN_E_HIGH;

  delayMicroseconds(1); // enable pulse must be >450ns
  PIN_E_LOW;

  delayMicroseconds(100); // commands need > 37us to settle
}

void LiquidCrystal::write4bits(uint8_t value)
{
  /* do not shif, just unroll the loop for 4 bits, set output bits */
  if (value & 0x01)
    PIN_D4_HIGH
  else
    PIN_D4_LOW

  if (value & 0x02)
    PIN_D5_HIGH
  else
    PIN_D5_LOW

  if (value & 0x04)
    PIN_D6_HIGH
  else
    PIN_D6_LOW

  if (value & 0x08)
    PIN_D7_HIGH
  else
    PIN_D7_LOW

  /* toggle E pin */
  pulseEnable();
}
