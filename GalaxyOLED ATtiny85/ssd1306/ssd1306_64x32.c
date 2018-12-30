/**
 * SSD1306xLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x64 displays
 *
 * @created: 2014-08-12
 * @author: Neven Boyanov
 *
 * This is part of the Tinusaur/SSD1306xLED project.
 *
 * Copyright (c) 2016 Neven Boyanov, Tinusaur Team. All Rights Reserved.
 * Distributed as open source software under MIT License, see LICENSE.txt file.
 * Please, as a favor, retain the link http://tinusaur.org to The Tinusaur Project.
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */
// Some code based on "IIC_without_ACK" by http://www.14blog.com/archives/1358
// ============================================================================

#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "ssd1306_64x32.h"
#include "font6x8.h"


#define DW_HI(PORT) PORTB |= (1 << PORT)
#define DW_LO(PORT) PORTB &= ~(1 << PORT)



#ifdef USE_SSD1306_64x32
  #define COLS    64
  #define PAGES   4
#endif
#ifdef USE_SSD1306_128x64
  #define COLS    128
  #define PAGES   8
#endif

#define USEABLE_WIDTH (COLS - CHAR_WIDTH)
#define CHAR_WIDTH 6



#ifdef USE_SSD1306_64x32
const uint8_t ssd1306_init_sequence [] PROGMEM = {  // Initialization Sequence
  0xA8, 0x1F,   // Set MUX Ratio, 0F-3F
  0xD3, 0x00,   // Set Display Offset
  0x40,         // Set Display Start line
  0xA1,         // Set Segment re-map, mirror, A0/A1
  0xC8,         // Set COM Output Scan Direction, flip, C0/C8
  0xDA, 0x12,   // Set Com Pins hardware configuration, Alternative
  0x81, 0x01,   // Set Contrast Control, 01-FF
  0xA4,         // Disable Entire Display On, 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
  0xA6,         // Set Display Mode. A6=Normal; A7=Inverse
  0xD5, 0x80,   // Set Osc Frequency
  0x8D, 0x14,   // Enable charge pump regulator
  0xAF          // Display ON in normal mode
};
#endif

#ifdef USE_SSD1306_128x64
const uint8_t ssd1306_init_sequence [] PROGMEM = {  // Initialization Sequence
  0xAE,         // Display OFF (sleep mode)
  0x20, 0b00,   // Set Memory Addressing Mode
                // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                // 10=Page Addressing Mode (RESET); 11=Invalid
  0xB0,         // Set Page Start Address for Page Addressing Mode, 0-7
  0xC8,         // Set COM Output Scan Direction
  0x00,         // ---set low column address
  0x10,         // ---set high column address
  0x40,         // --set start line address
  0x81, 0x3F,   // Set contrast control register
  0xA1,         // Set Segment Re-map. A0=address mapped; A1=address 127 mapped. 
  0xA6,         // Set display mode. A6=Normal; A7=Inverse
  0xA8, 0x3F,   // Set multiplex ratio(1 to 64)
  0xA4,         // Output RAM to Display
                // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
  0xD3, 0x00,   // Set display offset. 00 = no offset
  0xD5,         // --set display clock divide ratio/oscillator frequency
  0xF0,         // --set divide ratio
  0xD9, 0x22,   // Set pre-charge period
  0xDA, 0x12,   // Set com pins hardware configuration    
  0xDB,         // --set vcomh
  0x20,         // 0x20,0.77xVcc
  0x8D, 0x14,   // Set DC-DC enable
  0xAF          // Display ON in normal mode
};

#endif

// ----------------------------------------------------------------------------

void ssd1306_xfer_start(void)
{
  DW_HI(SSD1306_SCL);
  DW_HI(SSD1306_SDA);
  DW_LO(SSD1306_SDA);
  DW_LO(SSD1306_SCL);
}

void ssd1306_xfer_stop(void)
{
  DW_LO(SSD1306_SCL);
  DW_LO(SSD1306_SDA);
  DW_HI(SSD1306_SCL);
  DW_HI(SSD1306_SDA);
}

void ssd1306_send_byte(uint8_t byte)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((byte << i) & 0x80)
      DW_HI(SSD1306_SDA);
    else
      DW_LO(SSD1306_SDA);
    
    DW_HI(SSD1306_SCL);
    DW_LO(SSD1306_SCL);
  }
  DW_HI(SSD1306_SDA);
  DW_HI(SSD1306_SCL);
  DW_LO(SSD1306_SCL);
}

void ssd1306_send_command_start(void) {
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  // Slave address, SA0=0
  ssd1306_send_byte(0x00);  // write command
}

void ssd1306_send_command_stop(void) {
  ssd1306_xfer_stop();
}

void ssd1306_send_command(uint8_t command)
{
  ssd1306_send_command_start();
  ssd1306_send_byte(command);
  ssd1306_send_command_stop();
}

void ssd1306_send_data_start(void)
{
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);
  ssd1306_send_byte(0x40);  //write data
}

void ssd1306_send_data_stop(void)
{
  ssd1306_xfer_stop();
}

// ----------------------------------------------------------------------------

void ssd1306_init ()
{
  DDRB |= (1 << SSD1306_SDA);  // Set port as output
  DDRB |= (1 << SSD1306_SCL);  // Set port as output
  
  for (uint8_t i = 0; i < sizeof (ssd1306_init_sequence); i++) {
    ssd1306_send_command(pgm_read_byte(&ssd1306_init_sequence[i]));
  }
}

void ssd1306_setpos (uint8_t col, uint8_t page)
{
  ssd1306_send_command_start();
  //Column
  ssd1306_send_byte(0x21);
  #ifdef USE_SSD1306_64x32
    ssd1306_send_byte(0x20 + col); //0x20 is our x offset
    ssd1306_send_byte(0x20 + 64);  //col-64 Range
  #endif
  #ifdef USE_SSD1306_128x64
    ssd1306_send_byte(col);
    ssd1306_send_byte(127);  //col-128 Range
  #endif
  //Page
  ssd1306_send_byte(0x22);
  ssd1306_send_byte(page);
  ssd1306_send_byte(0); //Just this page (I haven't figured out how to config for auto-wrap)
  ssd1306_send_command_stop();
}

void ssd1306_fill_page (uint8_t page, uint8_t b)
{
  ssd1306_setpos(0, page);
  ssd1306_send_data_start();
  for (uint8_t col = 0; col < COLS; ++col) ssd1306_send_byte(b);
  ssd1306_send_data_stop();
  ssd1306_setpos(0, page);
}

void ssd1306_fill (uint8_t b)
{
  for (uint8_t page = 0; page < PAGES; ++page) ssd1306_fill_page(page, b);
  ssd1306_setpos(0, 0);
}

// ----------------------------------------------------------------------------

void ssd1306_char (char ch)
{
  uint8_t c = ch - 32;
  ssd1306_send_byte(0x00);
  for (uint8_t s = 0; s < 5; ++s) ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font6x8[c * 5 + s]));
}
