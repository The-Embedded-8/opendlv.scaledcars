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
  for(int i = 0;i<2;i++){
    strip.setPixelColor(i, 255,165,0);
    strip.show();
    delay(25);
    strip.setPixelColor(i, 0);
  }
}

void blinkRight() {
  for(int i = 6;i<8;i++){
    strip.setPixelColor(i, 255,165,0);
    strip.show();
    delay(25);
    strip.setPixelColor(i, 0);
  }
}

//static void chase(uint32_t c) {
//  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
//      strip.setPixelColor(i  , c); // Draw new pixel
//      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
//      strip.show();
//      delay(25);
//  }
}
