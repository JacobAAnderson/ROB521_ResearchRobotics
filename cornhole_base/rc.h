#ifndef rc_h
#define rc_h
#include "Arduino.h"

typedef struct {
  bool io;
  bool trig;
  float motor1;
  float motor2;
  float motor3;
  char msg[60];
  int FailRc;
} RC;

RC rc;

void beginRC(){ 
  Serial3.begin(9600);
  rc.io = false;
  rc.trig = false;
  rc.motor1 = 0.0;
  rc.motor2 = 0.0;
  rc.motor3 = 0.0;
  rc.FailRc = 25;
  }


// Parse Serial mesage from RC controller ----------------------------------------
void Parce_Msg(){

  char * ID; 
  ID = strtok(rc.msg,",");

  char * io;
  io = strtok(NULL,",");
  rc.io = atof(io);

  if(rc.io) {
    char * trig;
    trig = strtok(NULL,",");
    rc.trig = atof(trig);
  
    char * m1;
    m1 = strtok(NULL,",");
    rc.motor1 = atof(m1);
  
    char * m2;
    m2 = strtok(NULL,",");
    rc.motor2 = atof(m2);

    char * m3;
    m3 = strtok(NULL,",");
    rc.motor3 = atof(m3);
    }
    
  else{
    rc.trig = false;
    rc.motor1 = 0.0;
    rc.motor2 = 0.0;
    rc.motor3 = 0.0;  
  }

  Serial.println(ID);
  Serial.println(rc.io); 
  Serial.println(rc.trig); 
  Serial.println(rc.motor1);      
  Serial.println(rc.motor2);
  Serial.println(rc.motor3);
  
}

// Read serial mesage from RC controler -------------------------------------------------------
void updateRC(){
  rc.FailRc = 25;

  static int ii = 0;
  while(Serial3.available()){
    char ch = Serial3.read();
    
    
    if(ch == '$' || ii >49) { // Begining characture or over flow
      memset(rc.msg, '\0',sizeof rc.msg);
      rc.msg[0] = ch;
      ii = 1;
      }

    else if (ch =='\n'){  // End of mesage
      Parce_Msg();
      ii = 0;
      break; 
      }

    else if( isAlphaNumeric(ch) || isPunct(ch)){ // Add info to  mesage
      rc.msg[ii] = ch;
      ii++;
      }

    else;       // We got noise
 //   delay(1);   // delay for stability
  }
}

#endif
