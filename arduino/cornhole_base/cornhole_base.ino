/*   NOTES:
 *      Gear ratio for throwing arm: motor:17  arm:16
 *    
 *   Run the ROS Serial Node: 
 *      rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
 */

// Libraries -----------------------------
#include <avr/wdt.h>                    // Watch dog timer library
//#include <ArduinoHardware.h>
//#include <ros.h>
//#include <ros/time.h>
//#include <geometry_msgs/Point.h>
//#include <std_msgs/String.h>
#include "motor.h"
#include "rc.h"
#include "deffs.h"
#include <Servo.h>
//#include "pid.h"


enum STATE {  // Sate machine for operating the robot
  drive,      // Automomous Driving
  aim,        // Autonomous Aiming
  shoot,      // Autonomous Throwing
  teleop,     // Human tele-operation
  e_stop      // Saft State where everything stops moving
};

char hello[50];

/*/ ROS Stuff ------------------------------------------------------------
ros::NodeHandle      nh;
geometry_msgs::Point target;
std_msgs::String     str_msg;
// CallBack Functions
void targetCallBack(const geometry_msgs::Point& msg) {target = msg;}

ros::Subscriber <geometry_msgs::Point> sub("/target", targetCallBack);
ros::Publisher chatter("arduino_debug", &str_msg);
// -----------------------------------------------------------------------
*/
STATE state;

// Set up motors --------------------------
motor throwingMotor(PIN_ARM_PWM,     PIN_ARM_DIR);
motor leftDriveMotor(PIN_LEFT_PWM,  PIN_LEFT_DIR);
motor rightDriveMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR);

Servo triger;



//=================================================================================================================
void setup(){

  Serial.begin(9600);
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
  throwingMotor.setPID( 0.2, 0.01, 0.01);
  leftDriveMotor.setPID(  1, 0.001, 0.02);
  rightDriveMotor.setPID( 1, 0.001, 0.02);

  // Set up motor constriants -----------------------------------------------------
  leftDriveMotor.maxAlpha  = 4.0;   // Max angular acceleration
  rightDriveMotor.maxAlpha = 4.0;
  throwingMotor.maxAlpha   = 2.0;

  leftDriveMotor.maxVel  = 251.0 * RPM_RADS;  // Mav Angular Velocity
  rightDriveMotor.maxVel = 251.0 * RPM_RADS;
  throwingMotor.maxVel   = 340.0 * RPM_RADS;

  /*/ Set up ROS Node --------------------------------------------------------------
  target.x = 0.0;
  target.y = 0.0;
  target.z = 0.0;
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
*/
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
rc.io = false;
  //Serial.flush();
  wdt_reset();     // Reset Watch dog timer
  
  updateRC();
  if(rc.io) state = teleop;
  
  switch(state){

    case teleop:
    Serial.println("\nTele - Op");
        if(rc.io && rc.FailRc>0){
          Serial.println("motor val 1: ");
          Serial.println(rc.motor1);
          Serial.println("  2:  ");
          Serial.println(rc.motor2);
          Serial.println("\n");

          if(rc.motor3<10) rc.motor3 = 0;
          
          throwingMotor.tele(-rc.motor3);
          leftDriveMotor.tele(rc.motor1);
          rightDriveMotor.tele(rc.motor2);

          if(rc.trig){ 
            triger.write(135);
            //delay(100);  //wait for servo to get in pos
            //delay(100); //release one before slowdown
            //if(rc.motor3!=0)throwingMotor.tele(-(rc.motor3-20));  //shutdown
            //delay(200);
            Serial.flush();
          }
          else triger.write(175); 
        }
        else state = e_stop;            
        
        break;

    /*case e_stop:
    Serial.println("\nE-Stop");
        throwingMotor.angular_speed(0 );
        leftDriveMotor.angular_speed(0 );
        rightDriveMotor.angular_speed(0);
 //       state = aim;
        break;
           
    case aim:
    Serial.println("\nAim");
        triger.write(175);
//        throwingMotor.to_theta(0);
        leftDriveMotor.to_theta(20* PI/180);
        rightDriveMotor.to_theta(-20* PI/180);
        break;

    case shoot:
    Serial.println("\nShoot");
        throwingMotor.angular_speed(0 );
        leftDriveMotor.angular_speed(0 );
        rightDriveMotor.angular_speed(0);
        triger.write(135);
        break;

    case drive:
    Serial.println("\nDrive");
        break;
        */

    default:
      triger.write(175); 
      Serial.println("\n\n\nI CANT DO IT !!!!!!!@\n\n\n");
      state = e_stop;
    }

  /*float xx = target.x;
  float yy = target.y; 
  float zz = target.z;  

  char buff1[10];
  dtostrf(xx, 2,2, buff1);

  char buff2[10];
  dtostrf(yy, 2,2, buff2);

  char buff3[10];
  dtostrf(zz, 2,2, buff3);
 
  //sprintf(hello,"Target: %s, %s, %s", buff1, buff2, buff3);
  //str_msg.data = hello; // buff1;
  //chatter.publish( &str_msg );

  //nh.spinOnce();
*/
  throwingMotor.UpDateVelocities();
  }
