#include <stdlib.h>
#include <avr/io.h>
#define USE_SSD1306_128x64
#include "ssd1306/ssd1306_64x32.c"

#define PAGES   8
#define COLUMNS 128

#define NUM_BODIES 48
#define SPEED 1.
#define BOUNCE_PENALTY .1

float x[NUM_BODIES]    = {0};
float y[NUM_BODIES]    = {0};
float xVel[NUM_BODIES] = {0};
float yVel[NUM_BODIES] = {0};

void setup()
{
  //Initialise bodies
  float step = 6.28 / NUM_BODIES;
  float a = 0;
  for (uint8_t b = 1; b < NUM_BODIES; ++b) {
    x[b] = 64 + (sinf(a) * 10);
    y[b] = 32 + (cosf(a) * 10);
    a += step;
  }
  //Initialise display
  delay(40);
  ssd1306_init();
  ssd1306_fill(0x00);
}

static float ApproxSqrt (float z);

void loop()
{
  //Gravitate
  for (uint8_t b = 0; b < NUM_BODIES; ++b) {
    float xGross = 0, yGross = 0;
    for (uint8_t a = 0; a < NUM_BODIES; ++a) {
      if (a == b) continue;
      float xDiff = x[a] - x[b];
      float yDiff = y[a] - y[b];
      float dist = ApproxSqrt((xDiff * xDiff) + (yDiff * yDiff));
      float sDist = ApproxSqrt(dist);
      float xNorm = xDiff / dist;
      float yNorm = yDiff / dist;
      xGross += (xNorm * SPEED) / sDist;
      yGross += (yNorm * SPEED) / sDist;
    }
    xVel[b] += xGross / NUM_BODIES;
    yVel[b] += yGross / NUM_BODIES;
    //Limit speed
    if (xVel[b] > 2) xVel[b] = 2;
    if (yVel[b] > 2) yVel[b] = 2;
  }
  for (uint8_t b = 0; b < NUM_BODIES; ++b) {
    //Bounce
    if      (x[b] <= 1   &&  xVel[b] < 0) xVel[b] *= -BOUNCE_PENALTY;
    else if (x[b] >= 127 &&  xVel[b] > 0) xVel[b] *= -BOUNCE_PENALTY;
    if      (y[b] <= 1   &&  yVel[b] < 0) yVel[b] *= -BOUNCE_PENALTY;
    else if (y[b] >= 63  &&  yVel[b] > 0) yVel[b] *= -BOUNCE_PENALTY;
    //Move
    x[b] += xVel[b];
    y[b] += yVel[b];
  }
  //Draw
  for (uint8_t page = 0; page < PAGES; ++page) {
    uint8_t pageBuff[COLUMNS] = {0};
    for (uint8_t b = 0; b < NUM_BODIES; ++b) {
      if (uint8_t(y[b]) / 8 != page) continue; //Filter for bodies of this page
      pageBuff[uint8_t(x[b])] |= 1 << (uint8_t(y[b]) % 8);
    }
    ssd1306_setpos(0, page);
    for (uint8_t col = 0; col < COLUMNS; ++col) {
      ssd1306_send_data_start();
      ssd1306_send_byte(pageBuff[col]);
      ssd1306_send_data_stop();
    }
  }
}

union
{
  int i;
  float f;
} u;
static float ApproxSqrt (float z)
{
  u.f = z;
  u.i = ((u.i - (1 << 23)) >> 1) + (1 << 29); //((less 2^m) divide by 2) add ((b + 1) / 2) * 2^m
  return u.f;
}
