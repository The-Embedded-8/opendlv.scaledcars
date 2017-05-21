#include <Adafruit_NeoPixel.h>
 
#define PIN      6
#define N_LEDS 8
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
 
void setup() {
  strip.begin();
}
 
void loop() {
  //blinkLeft();
  blinkRight();
  //chase(strip.Color(255, 0, 0)); // Red
  //chase(strip.Color(0, 255, 0)); // Green
  //chase(strip.Color(0, 0, 255)); // Blue
}

void blinkLeft() {
    strip.setPixelColor(0, 255,165,0);
     strip.setPixelColor(1, 255,165,0);
    strip.show();
    delay(25);
    strip.setPixelColor(0, 0);
    strip.setPixelColor(1, 0);
    strip.show();
}

void blinkRight() {
    strip.setPixelColor(6, 255,165,0);
    strip.setPixelColor(7, 255,165,0);
    strip.show();
    delay(25);
    strip.setPixelColor(6, 0);
    strip.setPixelColor(7, 0);
    strip.show();
}

//static void chase(uint32_t c) {
//  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
//      strip.setPixelColor(i  , c); // Draw new pixel
//      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
//      strip.show();
//      delay(25);
//  }
}
