#define N_POTS   16
#define A_RES   1024
#define C_PI    3.14159265359
#define C_TWOPI 6.28318530718
#define C_CCUPSTART 60
#define C_CCDOWNSTART 10
#define C_MIDICHANNEL 1
#define N_BYTES 4
#define M_PI

uint8_t potId[N_POTS] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};

int     rawPotVal[N_POTS]; 
int     oldRawPotVal[N_POTS]; 
float   potVal[N_POTS/2]; 
double   oldVal[N_POTS/2];
double   history[N_POTS];
int     sendVal[N_POTS/2];
int     oldSendVal[N_POTS/2];
int     oldSerVal[N_POTS/2];
byte    receiveBytes[N_POTS/2][N_POTS/2 * N_BYTES];

void setup() {

  for(int i = 0; i < N_POTS; i++){
    pinMode(potId[i],INPUT);
    rawPotVal[i] = 0;
    oldVal[i/2] = 0.0;
    potVal[i/2] = 0.0;
    history[i/2] = 0.0;
    oldSendVal[i/2] = 0;  
  }//for

  //turn on led on teensy board
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  analogReference(EXTERNAL);
  usbMIDI.setHandleControlChange(myControlChange);

  Serial1.begin(38400);
  //Serial2.begin(9600);

}

void loop() {

  //check if arduino mega is sending data else check own data
  if (Serial1.available() > 0) {
    //get potmeter data from arduino mega
    getSerial(); 
  }else{
    usbMIDI.read(1);

    checkPots();
  }

}//loop

void getSerial(){
  static byte pot_id = 0;
  static byte bCount = 0;
  
  //if there is Serial data availible 
  //if (Serial1.available() > 0) {
    byte incomingbyte = Serial1.read();
    if(incomingbyte >= 200){
      pot_id = incomingbyte-200;
      bCount = 0;
    }else if(incomingbyte > -1){
      byte bytePos = pot_id*N_BYTES + bCount;
      //receiveBytes[pot_id][bytePos] = incomingbyte;

      //send received bytes trough to usbMidi
      usbMIDI.sendControlChange(C_CCDOWNSTART + bytePos ,incomingbyte, C_MIDICHANNEL);

      bCount++;
    }
  //}//if
  
}//getSerial

void checkPots(){

 for(int i = 0; i < N_POTS/2; i++){
    int iter_1 = i*2;
    int iter_2 = iter_1 + 1;

    int preFilter = 64;
    rawPotVal[iter_1] = 0;
    rawPotVal[iter_2] = 0;
    
    for(int j = 0; j < preFilter; j++){
      rawPotVal[iter_1]+= analogRead(potId[iter_1]);
      rawPotVal[iter_2]+= analogRead(potId[iter_2]);    
    }
    rawPotVal[iter_1]/=preFilter;
    rawPotVal[iter_2]/=preFilter;

    if(oldRawPotVal[iter_1] != rawPotVal[iter_1] || oldRawPotVal[iter_2] != rawPotVal[iter_2]){
  
      //let value go from -1. 1.
      double fRawVal1 = (float)rawPotVal[iter_1]/512.0 - 1.0;
      double fRawVal2 = (float)rawPotVal[iter_2]/512.0 - 1.0;
  
      //calculate angle for potmeter
      double val = atan2(fRawVal1,fRawVal2);
      //subtract form oldval to get direction of potmeter
      double newVal  = constrain(val-oldVal[i],-0.5,0.5)*1000.;
      oldVal[i] = val;
  
      //filter with onepole filter to remove noise
      double filter_C = 0.1;
      newVal = history[i] * (1. - filter_C) + newVal * filter_C;
      history[i] = newVal;
      
  
      //add direction to currentvalue to get new value
      potVal[i]+=newVal;
      potVal[i]= constrain(potVal[i],0.0,8192.0);
      sendVal[i] = (int)potVal[i];
  
  
  
      //if value has changed
      if(sendVal[i] != oldSendVal[i]){
        //divide value over multiple midi channels
        int sendVal1 = sendVal[i] &  127;
        int sendVal2 = sendVal[i] >> 7;
         
        usbMIDI.sendControlChange(C_CCUPSTART+i*4  , sendVal1, C_MIDICHANNEL);
        usbMIDI.sendControlChange(C_CCUPSTART+i*4+1, sendVal2, C_MIDICHANNEL);
        
        byte serSendVal = sendVal[i]/683;
          if(serSendVal != oldSerVal[i]){
          //send value to arduino mega for setting the leds first value is the id of the pot
          //second value is the led value of the pot
          Serial1.write(200+i);
          Serial1.write(serSendVal);
        
        }
        oldSerVal[i] = serSendVal;
  
      }//if
      oldSendVal[i] = sendVal[i];
    }//if
    oldRawPotVal[iter_1] = rawPotVal[iter_1];
    oldRawPotVal[iter_2] = rawPotVal[iter_2];
  
  }//for

  
}//checkpots

void myControlChange(byte channel, byte control, byte value){

  //set pots that are connected to this teensy
  if(control >= 60 && control < 68){ 
    //scale midi from 0-127 to 0 8192
    int id = control-60;
    potVal[id] = value*64;
    Serial1.write(200+id);   
    Serial1.write(value/10);
  }else if(control >= 68 && control < 76){
    Serial1.write(208+control-68);   
    Serial1.write(value);     
  }else if(control >= 0 && control < 48){
    Serial1.write(150+control);
    Serial1.write(value);
  }
  
  if(control == 127 && value == 127){
    //midi id for clearing all leds
    Serial1.write(255);
  }
  
}//myControlChange
