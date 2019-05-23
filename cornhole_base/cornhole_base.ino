/*   NOTES:
 *    Gear ratio for throwing arm: motor:17  arm:16
 * 
 */

// Libraries -----------------------------
#include <avr/wdt.h>                    // Watch dog timer library
#include "motor.h"
#include "rc.h"
#include "deffs.h"
//#include "pid.h"


enum STATE {  // Sate machine for operating the robot
  drive,      // Automomous Driving
  aim,        // Autonomous Aiming
  shoot,      // Autonomous Throwing
  teleop,     // Human tele-operation
  e_stop      // Saft State where everything stops moving
};


STATE state;

// Set up motors --------------------------
motor throwingMotor(PIN_ARM_PWM,     PIN_ARM_DIR);
motor leftDriveMotor(PIN_LEFT_PWM,  PIN_LEFT_DIR);
motor rightDriveMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR);

//=================================================================================================================
void setup(){

  Serial.begin(9600);
  beginRC();
  
  state = e_stop;
 
  // Set Up Encoders ---------------------------------------------------------------
  throwingMotor.encoder( PIN_ODOM_WHEEL_A, PIN_ODOM_WHEEL_B, ARM_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_WHEEL_A),  TM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_WHEEL_B),  TM_int, CHANGE);
  throwingMotor.dir = -1;

  leftDriveMotor.encoder( PIN_ODOM_LEFT_A, PIN_ODOM_LEFT_B, WHEEL_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_LEFT_A),  LM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_LEFT_B),  LM_int, CHANGE);

  rightDriveMotor.encoder( PIN_ODOM_RIGHT_A, PIN_ODOM_RIGHT_B, WHEEL_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_RIGHT_A),  RM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_RIGHT_B),  RM_int, CHANGE);
  rightDriveMotor.dir = -1;

  // Se Up PID controls -----------------------------------------------------------
  throwingMotor.setPID( 0.2, 0.01, 0.01);
  leftDriveMotor.setPID(  1, 0.001, 0.02);
  rightDriveMotor.setPID( 1, 0.001, 0.02);

 
  // Set up Watch dog timer -------------------------------------------------------
  cli();        // disable all interrupts 
  wdt_reset();  // reset the WDT timer 

  WDTCSR |= B00011000;  // Enter Watchdog Configuration mode: 
  WDTCSR = B01101001; // Set timer to restart the Arduino after  ~8 second

  sei(); // Enable all interrupts  


//=============================== End Setup ====================================================================================== 
 }

// ISR functions ---------------------------------------
void TM_int(){throwingMotor.updateOdom();}
void LM_int(){leftDriveMotor.updateOdom();}
void RM_int(){rightDriveMotor.updateOdom();}

// Main ---------------------------------------------------  
 void loop() {

  wdt_reset();     // Reset Watch dog timer
  
  updateRC();
  if(rc.io) state = teleop;
  
  switch(state){

    case teleop:
    Serial.println("\nTele - Op");
        if(rc.io){
          throwingMotor.angular_speed(rc.motor1 );
          leftDriveMotor.angular_speed(rc.motor2 );
          rightDriveMotor.angular_speed(rc.motor3);
        }
        else state = e_stop;            
        
        break;

    case e_stop:
    Serial.println("\nE-Stop");
        throwingMotor.angular_speed(0 );
        leftDriveMotor.angular_speed(0 );
        rightDriveMotor.angular_speed(0);
 //       state = aim;
        break;
           
    case aim:
    Serial.println("\nAim");
//        throwingMotor.to_theta(0);
        leftDriveMotor.to_theta(20* PI/180);
        rightDriveMotor.to_theta(-20* PI/180);
        break;

    case shoot:
    Serial.println("\nShoot");
        throwingMotor.angular_speed(0 );
        leftDriveMotor.angular_speed(0 );
        rightDriveMotor.angular_speed(0);
        break;

    case drive:
    Serial.println("\nDrive");
        break;

    default:
      Serial.println("\n\n\nI CANT DO IT !!!!!!!@\n\n\n");
      state = e_stop;
    }
    
 
  }
