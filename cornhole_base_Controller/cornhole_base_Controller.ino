

//Mega ADK

// Libraries -----------------------------
#include <EnableInterrupt.h>    //rc intterupts on analog pins

//Remote --------------------------------
#define RC_NUM_CHANNELS  5
int lastsig = 0;

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3
#define RC_CH5_INPUT  A4

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

const byte numChars = 32;
char tempChars[numChars];

int armpwm = 0;
int onoff = 0;
int launch = 0;

//rc interrupts -------------------------
void rc_read_values() {
  noInterrupts();
  //need to fill all with zero if less lastsig < 0
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT);lastsig = 25; }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT);lastsig = 25; }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT);lastsig = 25; }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT);lastsig = 25; }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT);lastsig = 25; }


//=================================================================================================================
void setup(){
  rc_read_values();

  Serial.begin(9600);
  
  // enabling rc interrupts -------------------------------------------------------
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  
  sei(); // Enable all interrupts 
//=============================== End Setup ====================================================================================== 
 }
  
 void loop() {
  rc_read_values();
  /*Serial.write(mystr,5);
  if(Serial.available()>0){
    incomingByte = Serial.read();
    Serial.print("I recieved: ");
    Serial.print(incomingByte, DEC);
  }*/

  ArmMotor();
  drives();
  
  lastsig --;
  }



void ArmMotor(){
  //failsafe
    if(lastsig<0){
      armpwm = 0;
      launch = 0;
      onoff = 0;
      return;
    }
    
  int armval = rc_values[RC_CH1];
  int launchval = rc_values[RC_CH2];
  int onoffval = rc_values[RC_CH5];
  //take commands from remote
  if(onoffval<1300)
    onoff = 1;
  else
    onoff = 0;

  //launch controll
  if(launchval<1300){
     launch = 1;
  }
  else
    launch = 0;

  //arm motor value
  if(armval<1100){
     armval = 1100;
  }
  if(armval>1900){
     armval = 1900;
  }

  float pwmval = (armval-1100)/800.00;
  armpwm = pwmval*255;

  //Serial.print("channel 1: ");
  //Serial.print(pwmval);
  //Serial.print("\n");
  /*if(armpwm>1500 && lastsig>0){
      digitalWrite(12,HIGH);
      analogWrite(13,60); //signal
  }
  else{
      digitalWrite(12,HIGH);
      analogWrite(13,0); //signal
  }*/
}
void serialcall(int m1,int pwm1,int m2,int pwm2)
{
  String start = "$rc,";
  String cma = ",";
  //String lend = "#";
  String value = start + onoff + cma + launch + cma + m1 + cma + m2 + cma + armpwm + '\n';
  strcpy(tempChars, value.c_str());
    Serial.write(tempChars);
    delay(100);
  //Serial.print(tempChars);
}


void forward1(int one) //left
{
      //sideone
      digitalWrite(10,HIGH);
      analogWrite(11,one); //signal
}
void forward2(int two) //right
{
      //sidetwo
      digitalWrite(8,HIGH);
      analogWrite(9,two); //signal
}
void reverse1(int one) //left
{
      //sideone
      digitalWrite(10,LOW);
      analogWrite(11,one); //signal
}
void reverse2(int two) //right
{
      //sidetwo
      digitalWrite(8,LOW);
      analogWrite(9,two); //signal
}
//drive setup
void drives(){   
    //tank drive shit
      //failsafe
    if(lastsig<0){
      serialcall(0,0,0,0);
      //forward1(0);
      //forward2(0);
      return;
    }
    
    int xinput = rc_values[RC_CH4];
    int yinput = rc_values[RC_CH3];
    
    if(rc_values[RC_CH3]<900){
      yinput = 1500;
    }
    if(rc_values[RC_CH4]<900){
      xinput = 1500;
    }
    
    if(rc_values[RC_CH4]<1525 && rc_values[RC_CH4]>1475){
      xinput = 1500;
    }
    if(rc_values[RC_CH3]<1525 && rc_values[RC_CH3]>1475){
      yinput = 1500;
    }
    
    if(xinput<1100){
      xinput = 1100;
    }
    if(xinput>1900){
      xinput = 1900;
    }
    
    if(yinput<1100){
      yinput = 1100;
    }
    if(yinput>1900){
      yinput = 1900;
    }
    
    float xvalue = xinput-1500; 
    float yvalue = yinput-1500;
    float throttle = sqrt(abs(xvalue*xvalue) + abs(yvalue*yvalue));
    float percent = abs(xvalue/500);
    
  //end of tank drive shit
    //Serial.print("throttle: ");Serial.print(throttle);Serial.print(" \t");
    //Serial.print("xvalue: ");Serial.print(rc_values[RC_CH4]);Serial.print(" \t");
    //Serial.print("yvalue: ");Serial.print(yvalue);Serial.print(" \t");
  
  if(throttle < 10)
  {
    //Serial.print("zeroded: \t");

    serialcall(0,0,0,0);
    //forward1(0);
    //forward2(0);
    return;
  }
  else
  {
    float newthrottle = (throttle/400)*255;
    if(newthrottle> 255){
      newthrottle = 255;
    }
    //Serial.print("none:"); Serial.print(newthrottle); Serial.print("\t");
    
    if(percent < .06)
    {
      //Serial.print("none:"); Serial.print(newvalue); Serial.print("\t");
      if(rc_values[RC_CH3]>1500){
        serialcall(newthrottle*-1,0,newthrottle*-1,0);
        //reverse1(newthrottle);
        //reverse2(newthrottle);
      }
      else{
        serialcall(newthrottle,1,newthrottle,1);
        //forward1(newthrottle);
        //forward2(newthrottle);
      }
      
      return;
    }
    //side to side only
    else if(rc_values[RC_CH3]<1550 && rc_values[RC_CH3]>1450)
    {
      if(rc_values[RC_CH4]<1500)
      {
        serialcall(-percent*255,0,percent*255,1);
        //reverse1(255);
        //forward2(255);
        return;
      }
      else
      {
        //checkreverse(1500+throttle,1);
        serialcall(percent*255,1,-percent*255,0);
        //reverse2(255);
        //forward1(255);
        //checkreverse(1500-throttle,2);
        return;
      }
    }

    
    //left
    else if(rc_values[RC_CH4]<1450)
    {
      percent = 1-percent;
      
      if(rc_values[RC_CH3]>1500){
        serialcall(newthrottle*percent*-1,0,newthrottle*-1,0); //might be zero actually?
        //reverse1(newthrottle*percent);
        //reverse2(newthrottle);
      }
      else{
        serialcall(newthrottle*percent,1,newthrottle,1);
        //forward1(newthrottle*percent);
        //forward2(newthrottle);
      }
      
      return;
    }
    
    //right
    else if(rc_values[RC_CH4]>1550)
    {
      percent = 1-percent;
      
      if(rc_values[RC_CH3]>1500){
        serialcall(newthrottle*-1,0,newthrottle*percent*-1,0);
        //reverse1(newthrottle);
        //reverse2(newthrottle*percent);
      }
      else{
        serialcall(newthrottle,0,newthrottle*percent,0);
        //forward1(newthrottle);
        //forward2(newthrottle*percent);
      }
      
      return;
    }
    //done
  }
}
 
