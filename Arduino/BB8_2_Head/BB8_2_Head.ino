#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      4

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second
int brightness = 0;
int fadeAmount = 5;
int switchState = 0;

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pinMode (2, INPUT_PULLUP);

  

  pixels.begin(); // This initializes the NeoPixel library.
}


void fade(int i) {  
  for(int j=0;j<255;j+=4) {
    pixels.setPixelColor(i, pixels.Color(j,j,j));
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(8);
  }
}
  
void fadedown(int i) {
  for(int j=255;j>0;j-=4) {
    pixels.setPixelColor(i, pixels.Color(j,j,j));
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(8);
  }
    pixels.setPixelColor(i, pixels.Color(0,0,0));
    pixels.show(); // This sends the updated pixel color to the hardware.
}

void loop() {
    
    switchState = digitalRead(2);
    
   
    if (switchState == 0) {
    digitalWrite(2, HIGH); // turn on LEDs
    fade(1);                // fade eye up  
    delay(500);
    fade(0);                //fade holo up
    delay(500); 
    fadedown(1);                // fade eye down  
    delay(500);
    fadedown(0);                //fade holo down
    delay(500);
    fade(1);                // fade eye up  
    delay(500);
    fade(2);                // fade logic1 up
    fade(3);                // fade logic2 up 
    delay(500);
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    delay(500);
    fadedown(2);                // fade logic1 up
    fadedown(3);                // fade logic2 up  
    fade(2);                // fade logic1 up
    fade(3);                // fade logic2 up
    fadedown(2);                // fade logic1 down
    fadedown(3);                // fade logic2 down 
    fade(2);                // fade logic1 up
    fade(3);                // fade logic2 up   
    delay(500); 
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    fade(0);                //fade holo up
    fadedown(0);                //fade holo down
    fadedown(1);                // fade eye down 
    fadedown(2);                // fade logic1 down
    fadedown(3);                // fade logic2 down
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    fadedown(1);                // fade eye down
    fade(1);                // fade eye up 
    fadedown(1);                // fade eye down    


    

  }
}
