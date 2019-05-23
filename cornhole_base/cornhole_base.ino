/*   NOTES:
 *    Gear ratio for throwing arm: motor:17  arm:16
 * 
 */



// Libraries -----------------------------
#include <avr/wdt.h>                    // Watch dog timer library
#include "pid.h"
#include "motor.h"

// Constants -----------------------------
#define WHEEL_ENCODER_INC  0.00225      // Drive wheel encoder step   [rad]
#define ARM_ENCODER_INC    0.0123      // Arm encoder step           [rad]   0.00744 
#define DRIVE_WHEEL_RADIUS 0.076        // Radius of the drive wheel  [m]
#define ARM_RADIUS         0.254
#define MAX_ALPHA          0.25         // Max Motor acceleration     [rad/s^2]
#define MAX_DRIVE_OMEGA    1.9          // Max Drving motor velocity  [rad/s]
#define MAX_ARM_OMEGA     35.6047167    // Max Thowing motor velocity [rad/s]
#define ARM_GEAR_RATIO     0.941176     // 16/17 Gear Ratio
// I/O Pins-------------------------------

#define PIN_ARM_DIR  9
#define PIN_ARM_PWM  8

#define PIN_LEFT_DIR  10
#define PIN_LEFT_PWM  11

#define PIN_RIGHT_DIR 13 
#define PIN_RIGHT_PWM 12

#define PIN_ODOM_WHEEL_A 3
#define PIN_ODOM_WHEEL_B 2

#define PIN_ODOM_LEFT_A  21
#define PIN_ODOM_LEFT_B  20

#define PIN_ODOM_RIGHT_A 19
#define PIN_ODOM_RIGHT_B 18



// Set up motors --------------------------
motor throwingMotor(PIN_ARM_PWM,     PIN_ARM_DIR);
//motor leftDriveMotor(PIN_LEFT_PWM,  PIN_LEFT_DIR);
//motor rightDriveMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR);



//=================================================================================================================
void setup(){

  Serial.begin(9600);
 
  // Set Up Encoders ---------------------------------------------------------------

  throwingMotor.encoder( PIN_ODOM_WHEEL_A, PIN_ODOM_WHEEL_B, ARM_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_WHEEL_A),  TM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_WHEEL_B),  TM_int, CHANGE);
  throwingMotor.dir = -1;
/*
  leftDriveMotor.encoder( PIN_ODOM_LEFT_A, PIN_ODOM_LEFT_B, WHEEL_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_LEFT_A),  LM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_LEFT_B),  LM_int, CHANGE);

  rightDriveMotor.encoder( PIN_ODOM_RIGHT_A, PIN_ODOM_RIGHT_B, WHEEL_ENCODER_INC  );
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_RIGHT_A),  RM_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ODOM_RIGHT_B),  RM_int, CHANGE);
  rightDriveMotor.dir = -1;
*/
  // Se Up PID controls -----------------------------------------------------------
  throwingMotor.setPID( 0.2, 0.01, 0.01);
//  leftDriveMotor.setPID( 0.2, 0.05, 0.05);
//  rightDriveMotor.setPID( 0.2, 0.05, 0.05);
  
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
// void LM_int(){leftDriveMotor.updateOdom();}
// void RM_int(){rightDriveMotor.updateOdom();}

// Main ---------------------------------------------------  
 void loop() {

  wdt_reset();     // Reset Watch dog timer

  throwingMotor.angular_speed(4*PI / ARM_GEAR_RATIO);  // Set motor to 10 rad/s
//  leftDriveMotor.angular_speed(2*PI );
//  rightDriveMotor.angular_speed(-2*PI );
  }
