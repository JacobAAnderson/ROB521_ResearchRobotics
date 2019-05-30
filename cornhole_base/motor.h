#ifndef motor_h
#define motor_h

//#include "pid.h"
//#include <Encoder.h>

enum motorComs {    // Types of communication with motor controller
  singleMag,        // Analog pwm with a direction inidator pin
  locked_antiphase  // Servo style PWM
};


class motor {

  public:
    // Vars-------------------------------------
    float maxAlpha = 1.0;
    float maxVel = 10.0;
    float omega = 0.0;
    float odom = 0.0;
    int dir = 1;

    // Functions -------------------------------
    motor(int pwmPin, int dirPin);                      // Construtor
    void tele(int val);
    void encoder(int chA, int chB,  float encodInc);    // Encoder Steup
    void setPID(float Kp, float Ki, float Kd);
    void updateOdom();                                  // ISR for encoder
    void angular_speed(float set );                     // Give the motor a speed input
    bool to_theta(float theta);

    float UpDateVelocities(); //moved


  private:
    // Motor Comunication --------------------------------------
    int _pwmPin;
    int _dirPin = -1;
    motorComs _coms;

    // Encoder -------------------------------------------------
    bool _useEncoder = false;
    int _chA;
    int _chB;
    int _New = 0;
    unsigned long _UPV_lastTime = 0;
    float _lastOdom = 0.0; 
    float _encodInc;
   
    
//    static motor * object;
//    static void callBackGlue();
    
    
    // Velocity Ram ----------------------------------------------
    float velRamp( float velocity, float target );
    unsigned long _VR_lastTime = 0;


    // PID -------------------------------------------------------
    float _Kp = 0.5;
    float _Ki = 0.5;
    float _Kd = 0.5;
    float _error = 0.0;
    float _integral = 0.0;
    unsigned long _pid_lastTime = 0;

    float pid(float setPoint, float currentVal);

    // Drive the motor --------------------------------------------
    bool  _odomLock = false;
    float _odomStart;
    float _odomTarget;
    float _diff = 0.0;
    float _current_omega = 0.0;
    float _thataStart;
    
    void setMotor(float target);

};

#endif
