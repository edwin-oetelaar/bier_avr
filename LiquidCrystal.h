/* dit is onderdeel van Edwin van den Oetelaar BLM ding en nog andere projecten
* het lijkt op de arduino code maar het is het niet meer
* 1 december 2017 gemaakt
* alle bulk is eruit, we krijgen de hele app in 4k ipv 8k of 16k arduino
*/

#ifndef LiquidCrystal_h
#define LiquidCrystal_h

#include <inttypes.h>
#include "Print.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/*
 * PE3/ADC7 is input voor extern spanning
 * MOSI/PB3 is E 
 * MISO/PB4 is RS
 * PD2 is D4
 * PD3 is D5
 * PD4 is D6
 * PD5 is D7
 */

/* E is op MOSI pin, PB3 */
#define PIN_E_HIGH \
  {                \
    sbi(PORTB, 3); \
  }

#define PIN_E_LOW  \
  {                \
    cbi(PORTB, 3); \
  }

/* RS is op MISO pin, PB4 */
#define PIN_RS_HIGH \
  {                 \
    sbi(PORTB, 4);  \
  }

#define PIN_RS_LOW \
  {                \
    cbi(PORTB, 4); \
  }

#define PIN_D4_LOW \
  {                \
    cbi(PORTD, 2); \
  }

#define PIN_D4_HIGH \
  {                 \
    sbi(PORTD, 2);  \
  }

#define PIN_D5_LOW \
  {                \
    cbi(PORTD, 3); \
  }

#define PIN_D5_HIGH \
  {                 \
    sbi(PORTD, 3);  \
  }

#define PIN_D6_LOW \
  {                \
    cbi(PORTD, 4); \
  }

#define PIN_D6_HIGH \
  {                 \
    sbi(PORTD, 4);  \
  }

#define PIN_D7_LOW \
  {                \
    cbi(PORTD, 5); \
  }

#define PIN_D7_HIGH \
  {                 \
    sbi(PORTD, 5);  \
  }

class LiquidCrystal : public Print
{
public:
  LiquidCrystal(); /* constructor no parameters */

  void init();

  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void setRowOffsets(int row1, int row2, int row3, int row4);
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t);
  virtual size_t write(uint8_t);
  void command(uint8_t);

  using Print::write;

private:
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);

  void pulseEnable();

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _numlines;
  uint8_t _row_offsets[4];
};

#endif
