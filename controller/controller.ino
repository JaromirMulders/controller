
#include <Adafruit_NeoPixel.h>
 
#define PIN      6
#define N_LEDS 192
#define N_LEDPR 12
#define N_POTS   16
#define A_RES   1024
#define C_PI    3.14159265359
#define C_TWOPI 6.28318530718

uint8_t potId[N_POTS] = {A6,A7,A4,A5,A2,A3,A0,A1,A8,A9,A10,A11,A12,A13,A14,A15};
int     rawPotVal[N_POTS]; 
float   potVal[N_POTS/2]; 
float   oldVal[N_POTS/2];
float   history[N_POTS/2];
int     sendVal[N_POTS/2];
int     ledOrder[N_POTS] =  {6 ,4 ,2 ,0, 7,5,3,1 ,8,10,12,14,9,11,13,15};
int     ledOffest[N_POTS] = {11,11,11,11,5,5,5,11,0,0 ,11,0 ,5,5 ,4, 5};
int     ledPotVal[N_POTS];
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
 
void setup() {
  for(int i = 0; i < N_POTS; i++){
    pinMode(potId[i],INPUT);
    rawPotVal[i] = 0;
    oldVal[i/2] = 0.0;
    potVal[i/2] = 0.0;
    history[i/2] = 0.0;
    sendVal[i/2] = 0;
    ledPotVal[i] = 0;
  }//for

  Serial.begin(9600);
  strip.begin();
  
}//setup
 
void loop() {

  for(int i = 0; i < N_POTS/2; i++){
    int iter_1 = i<<1;
    int iter_2 = iter_1 + 1;
    
    rawPotVal[iter_1] = analogRead(potId[iter_1]);
    rawPotVal[iter_2] = analogRead(potId[iter_2]);    

    //first scale potvalues from 0-1024. to 0.-Pi
    //then calculate angle for potmeter
    float val = atan2(rawPotVal[iter_1] /326.-1.,rawPotVal[iter_2] /326.-1.);
    //subtract form oldval to get direction of potmeter
    float newVal  = constrain(val-oldVal[i],-0.5,0.5)*1000.;
    oldVal[i] = val;

    //filter with onepole filter to remove noise
    float filter_C = 0.4;
    newVal = history[i] * (1. - filter_C) + newVal * filter_C;
    history[i] = newVal;

    //add direction to currentvalue to get new value
    potVal[i]+=newVal;
    potVal[i]= constrain(potVal[i],0.0,8192.0);
    sendVal[i] = (int)potVal[i];
    //divide to go from 0-8192 to 0 - 11
    ledPotVal[i+8] = sendVal[i]/700; 

  }//for

  for(int i = 0; i < N_POTS; i++){
    
    uint8_t lo = ledOrder[i]*N_LEDPR;
    
    for(int amount = 0; amount < N_LEDPR; amount++){
      uint8_t ledId = lo + ((ledOffest[i] + amount) % N_LEDPR );
      bool ledSwitch = (amount/(ledPotVal[i]+1));
      ledSwitch^=1;
      uint8_t brightness = (255/((N_LEDPR-amount)+1)) * ledSwitch;
      //switch color between top and bottom row of led rings
      bool colorSwitch = i>>3;       
      showLedDial(ledId,brightness*colorSwitch,0,brightness);

    }//for
        
  }//for
  
  strip.show();  

}//loop

void showLedDial(uint8_t id, uint8_t r, uint8_t g, uint8_t b){
  strip.setPixelColor(id,r,g,b);
}

void clearLedDial(uint8_t id){
  strip.setPixelColor(id,0,0,0);
}
