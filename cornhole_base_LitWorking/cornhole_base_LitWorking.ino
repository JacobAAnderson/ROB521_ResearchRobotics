/*   NOTES:
 *      Gear ratio for throwing arm: motor:17  arm:16
 *    
 *   Run the ROS Serial Node: 
 *      rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
 */

// Libraries -----------------------------
#include <avr/wdt.h>                    // Watch dog timer library
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <math.h>
#include "motor.h"
#include "rc.h"
#include "deffs.h"
#include <Servo.h>

bool done = false;
char hello[50];
bool rosUpdate = false;
int count;

enum STATE {  // Sate machine for operating the robot
  drive,      // Automomous Driving
  aim,        // Autonomous Aiming
  spin,       // Spin up the wheel
  shoot,      // Autonomous Throwing
  teleop,     // Human tele-operation
  e_stop      // Saft State where everything stops moving
};


// ROS Stuff ------------------------------------------------------------
ros::NodeHandle      nh;
geometry_msgs::Point target;
std_msgs::String     str_msg;
// CallBack Functions
void targetCallBack(const geometry_msgs::Point& msg) {target = msg;
                                                      rosUpdate = true;}

ros::Subscriber <geometry_msgs::Point> sub("/target", targetCallBack);
ros::Publisher chatter("arduino_debug", &str_msg);
// -----------------------------------------------------------------------

STATE state;

// Set up motors --------------------------
motor throwingMotor(PIN_ARM_PWM,     PIN_ARM_DIR, true);
motor leftDriveMotor(PIN_LEFT_PWM,   PIN_LEFT_DIR, false);
motor rightDriveMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, false);

Servo triger;



//=================================================================================================================
void setup(){

  //Serial.begin(9600);
  beginRC();

  triger.attach(PIN_SERVO);
  triger.write(175);
  
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
  throwingMotor.setPID(   0.2, 0.01, 0.01);
  leftDriveMotor.setPID(  0.5, 0.001, 0.02);
  rightDriveMotor.setPID( 0.5, 0.001, 0.02);

  // Set up motor constriants -----------------------------------------------------
  leftDriveMotor.maxAlpha  = 4.0;   // Max angular acceleration
  rightDriveMotor.maxAlpha = 4.0;
  throwingMotor.maxAlpha   = 2.0;

  leftDriveMotor.maxVel  = 251.0 * RPM_RADS;  // Mav Angular Velocity
  rightDriveMotor.maxVel = 251.0 * RPM_RADS;
  throwingMotor.maxVel   = 340.0 * RPM_RADS;

  // Set up ROS Node --------------------------------------------------------------
  target.x = -1.0;
  target.y = -1.0;
  target.z = -1.0;

  //****************************uncomment******************************
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

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

  double theta;
  double Vo;

  float xx = target.x - 0.5; 
  float yy = target.y - 0.0;//- 0.135; 
  float zz = target.z - 0.34;
  char ch_state[8] = "none"; 

  static int count = 0;
  
  if(rc.io) state = teleop;
  
  switch(state){

    case teleop:
 //     Serial.println("\nTele - Op");      
       strcpy(ch_state,"Tele-Op");
        if(rc.io){

          if(rc.motor3<10) rc.motor3 = 0;
          
          throwingMotor.tele(-rc.motor3);
          leftDriveMotor.tele(rc.motor1);
          rightDriveMotor.tele(rc.motor2);

          if(rc.trig){ 
            triger.write(135);
            //Serial.flush();
          }
          else triger.write(175); 
        }
        else state = e_stop;
        break;

    case e_stop:
  //     Serial.println("\nE-Stop");
        triger.write(175);
        strcpy(ch_state, "stop");
        throwingMotor.tele(0);
        leftDriveMotor.tele(0);
        rightDriveMotor.tele(0);

        if ( !done && target.x > 0) state = aim;
        
        break;
           
    case aim:
 //       Serial.println("\nAim");
        strcpy(ch_state, "Aim");
        triger.write(175);
       
        theta = atan2(yy, xx);
        theta = theta * ROBOT_BASE_WIDTH /(2 * DRIVE_WHEEL_RADIUS);
        rosUpdate = false;
        
        //leftDriveMotor.to_theta(-theta, rosUpdate);
        //rightDriveMotor.to_theta(theta, rosUpdate);

        if(abs(theta) <= 3*ONE_DEG_IN_RAD ) state = spin; 
        
        break;

    case spin:
 //   Serial.println("\nShooting");
        strcpy(ch_state, "Spin");

        Vo = sqrt(abs(g * xx * xx /
                     (2 * ( yy * cos( THROW_ANGLE)*cos( THROW_ANGLE) - xx * cos( THROW_ANGLE) * sin( THROW_ANGLE)))));
 
        throwingMotor.angular_speed(Vo/ARM_RADIUS);

        leftDriveMotor.angular_speed(0 );
        rightDriveMotor.angular_speed(0);
        
        //(abs(theta) <= 3*ONE_DEG_IN_RAD)
        if( -.8>((Vo/ARM_RADIUS) - throwingMotor.UpDateVelocities()) && .8<((Vo/ARM_RADIUS) - throwingMotor.UpDateVelocities() ) ){ 
          triger.write(135);
          delay(50);
          if(count>1000) state = shoot;
          count += 10;
        }
        else{ 
          triger.write(160);
          count = 0;
        }
        break;

    case shoot:
        strcpy(ch_state, "Shoot");
        delay(6000);
        count = 0;
        state = e_stop;
        done = true;
        
    case drive:
 //   Serial.println("\nDrive");
        strcpy(ch_state, "Drive");
        break;

    default:
//      Serial.println("\n\n\nI CANT DO IT !!!!!!!@\n\n\n");
      strcpy(ch_state, "Err");
      state = e_stop;
    }

  
  char buff1[10];
  dtostrf( Vo, 2,2, buff1); //xx

  char buff2[10];
  dtostrf(theta, 2,2, buff2);

  char buff3[10];
  dtostrf(zz, 2,2, buff3);

  char buff4[10];
  dtostrf(leftDriveMotor.odom, 2,2, buff4);
 
  sprintf(hello,"%s: %s, %s, %s",ch_state, buff1, buff2, buff4);
  str_msg.data = hello; // buff1;
  
  
  //****************************uncomment******************************
  chatter.publish( &str_msg );

  nh.spinOnce();
  
  }



  
