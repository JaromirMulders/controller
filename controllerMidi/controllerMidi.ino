#define N_POTS   16
#define A_RES   1024
#define C_PI    3.14159265359
#define C_TWOPI 6.28318530718
#define C_CCSTART 60
#define C_MIDICHANNEL 1

uint8_t potId[N_POTS] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};

int     rawPotVal[N_POTS]; 
float   potVal[N_POTS/2]; 
float   oldVal[N_POTS/2];
float   history[N_POTS/2];
int     sendVal[N_POTS/2];
int     oldSendVal[N_POTS/2];

void setup() {

  for(int i = 0; i < N_POTS; i++){
    pinMode(potId[i],INPUT);
    rawPotVal[i] = 0;
    oldVal[i/2] = 0.0;
    potVal[i/2] = 0.0;
    history[i/2] = 0.0;
    oldSendVal[i/2] = 0;
  }//for

  pinMode(13,OUTPUT);
  
  digitalWrite(13,HIGH);
  
  Serial.begin(9600);
}

void loop() {

  
  for(int i = 0; i < N_POTS/2; i++){
    int iter_1 = i<<1;
    int iter_2 = iter_1 + 1;
    
    rawPotVal[iter_1] = analogRead(potId[iter_1]);
    rawPotVal[iter_2] = analogRead(potId[iter_2]);    

    //first scale potvalues from 0-1024. to -1.-1
    //then calculate angle for potmeter
    float val = atan2((float)rawPotVal[iter_1] /512. -1.,(float)rawPotVal[iter_2] / 512. -1.);
    //subtract form oldval to get direction of potmeter
    float newVal  = constrain(val-oldVal[i],-0.5,0.5)*1000.;
    oldVal[i] = val;

    //filter with onepole filter to remove noise
    float filter_C = 0.025  ;
    newVal = history[i] * (1. - filter_C) + newVal * filter_C;
    history[i] = newVal;

    //add direction to currentvalue to get new value
    potVal[i]+=newVal;
    potVal[i]= constrain(potVal[i],0.0,8192.0);
    sendVal[i] = (int)potVal[i];

    //if value has changed
    if(sendVal[i] != oldSendVal[i]){
      //divide value over multiple midi channels
      int sendVal1 = sendVal[i]     & 15;
      int sendVal2 = sendVal[i]>>4  & 15;
      int sendVal3 = sendVal[i]>>8  & 15;
      int sendVal4 = sendVal[i]>>12 & 15;  
          
      usbMIDI.sendControlChange(C_CCSTART+i*4  , sendVal1, C_MIDICHANNEL);
      usbMIDI.sendControlChange(C_CCSTART+i*4+1, sendVal2, C_MIDICHANNEL);
      usbMIDI.sendControlChange(C_CCSTART+i*4+2, sendVal3, C_MIDICHANNEL);
      usbMIDI.sendControlChange(C_CCSTART+i*4+3, sendVal4, C_MIDICHANNEL);  
    }//if
    oldSendVal[i] = sendVal[i];

    
    //Serial.print(sendVal[i]);
    //Serial.print(" ");

  }//for
  //Serial.println();


  
  
}
