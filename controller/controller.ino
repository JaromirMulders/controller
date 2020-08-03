
#include <Adafruit_NeoPixel.h>
 
#define PIN      6
#define N_LEDS 192
#define N_LEDPR 12
#define N_POTS   16
#define A_RES   1024
#define C_PI    3.14159265359
#define C_TWOPI 6.28318530718

uint8_t potId[N_POTS] = {A6,A7,A4,A5,A2,A3,A0,A1,A8,A9,A10,A11,A12,A13,A14,A15};
int rawPotVal[N_POTS]; 
float potVal[N_POTS/2]; 
float oldVal[N_POTS/2];
float history[N_POTS/2];
int   sendVal[N_POTS/2];
int   ledOrder[N_POTS] =   {6 ,4 ,2 ,0, 7,5,3,1 ,8,10,12,14,9,11,13,15};
int   ledOffest[N_POTS] =  {11,11,11,11,5,5,5,11,0,0 ,11,0 ,5,5 ,4, 5};

 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
 
void setup() {
  for(int i = 0; i < N_POTS; i++){
    pinMode(potId[i],INPUT);
    rawPotVal[i] = 0;
    oldVal[i/2] = 0.0;
    potVal[i/2] = 0.0;
    history[i/2] = 0.0;
    sendVal[i/2] = 0;
   
  }//for

  Serial.begin(9600);
  //Serial2.begin(9600);
  strip.begin();
  
}//setup
 
void loop() {
  
  for(int i = 0; i < N_POTS/2; i++){
    int iter_1 = i*2;
    int iter_2 = iter_1 + 1;
    
    rawPotVal[iter_1] = analogRead(potId[iter_1]);
    rawPotVal[iter_2] = analogRead(potId[iter_2]);    

    //first scale potvalues from 0-1024. to -1.-1.
    //then calculate angle for potmeter
    float val = atan2(rawPotVal[iter_1] /512.-1.,rawPotVal[iter_2] /512.-1.);
    //subtract form oldval to get delta of potmeter
    float newVal  = constrain(val-oldVal[i],-0.5,0.5)*1000.;
    oldVal[i] = val;

    //filter with onepole filter to remove noise
    float filter_C = 0.025;
    val = history[i] * (1. - filter_C) + newVal * filter_C;
    history[i] = newVal;

    //add delta to currentvalue to get new value
    potVal[i]+=newVal;
    potVal[i]= constrain(potVal[i],0.0,8192.0);
    sendVal[i] = (int)potVal[i];

    //Serial.print(sendVal[i]);
    //Serial.print(" ");
 
  }//for
  //Serial.println();

  //send serial data to teensy
  sendSerial(sendVal,sizeof(sendVal));

  /*turn on leds
  this for loops turns on the current led and turns off the led next to it
  this way i don't have to loop trough all the leds to check which led
  have to be on or off
  */
  for(int i = 0; i < N_POTS/2; i++){
    
    uint8_t lo = ledOrder[i+8]*N_LEDPR;
    //divide potvals from 0-8192 to 0 - N_LEDPR 
    uint8_t potDivide = sendVal[i]/683;
    //Add offset for ledringId
    uint8_t ledVal = ledOffest[i+8]+potDivide;
    //calculate where next led needs to be
    //the % is beceasue of the circular leds
    uint8_t ledId  = (ledVal%N_LEDPR) + lo;
    //calculate where next led the needs to be off needs to be
    uint8_t ledOffId  = ((ledVal+1)%N_LEDPR) + lo;
    
    showLedDial(ledId,20*(potDivide+1),10,10);

    //don't turn off leds when the potDivide is N_LEDPR so ledring can be fully lit
    if(potDivide < N_LEDPR-1){
      showLedDial(ledOffId,0,0,0);    
    }//if
    
  }//for
  strip.show();  


}//loop

void sendSerial(int * sendArray,int sizeOfArray){
  
  //send id of controllerboard
  Serial2.write(200);

  //15 = 11110000
  byte bitMask = 15;
  
  for(int i = 0; i < sizeOfArray; i++){
    Serial2.write(sendArray[i]       &bitMask);
    Serial2.write(sendArray[i] >> 4  &bitMask);
    Serial2.write(sendArray[i] >> 8  &bitMask);
    Serial2.write(sendArray[i] >> 12 &bitMask);
  }//for
  
}//sendSerial


void showLedDial(uint8_t id, uint8_t r, uint8_t g, uint8_t b){
  strip.setPixelColor(id,r,g,b);
}//showLedDial
 
static void chase(uint32_t c) {

  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
      strip.setPixelColor(i  , c); // Draw new pixel
      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
      strip.show();
      delay(25);
  }//for
  
}//chase

//function for converting an array of bits to a byte
byte bitsToByte(bool * bitArray){

  byte b = 0;
  for(int i = 0; i < 8; i++){
    b = (b << 1) | (bitArray[i] & 1);
  }//for

  return b;

}//bitsToByte

//function for converting bytes to bits
void byteToBits(byte b, bool * bArray){

  for(int i = 0; i < 8; i++){
    bArray[i] = ((128 >> i) & b) >> (7-i);
  }//for

}//byteToBits
