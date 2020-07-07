#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN        6
#define NUMPIXELS 24

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 100

#define cEncoderTresh 330

int x = 0;
int y = 0;
float oldVal = 0.;
float fVal = 0.;
float history = 0.;

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pixels.begin();

  
}

void loop() {

  x = analogRead(A0);
  y = analogRead(A1);  
  float val = atan2(x/334.-1.,y/334.-1.);
  float newVal = constrain(val-oldVal,-0.5,0.5)*1000.;
  oldVal = val;
  newVal = history * (1. - 0.1) + newVal * 0.1;
  history = newVal;


  fVal+=newVal;
  fVal = constrain(fVal,0.,8192.);
  int iVal = (int)fVal/2;


  pixels.clear();
  
  for(int i=0; i<24; i++) {

    //pixels.setPixelColor(i, pixels.Color(i*8+2, i*4+1, 0));
    //pixels.setPixelColor(12-(i)+12, pixels.Color(i*8+2, i*4+1, 0));
    pixels.setPixelColor(i,pixels.Color(10,10,0));
  }
  //pixels.setPixelColor(iVal/341, pixels.Color(10, 5, (iVal%341)/4));
  //pixels.setPixelColor(24-(iVal/341+12), pixels.Color(10, 5, (iVal%341)/4));
  
  
     pixels.show(); 
  Serial.println(iVal);

}
