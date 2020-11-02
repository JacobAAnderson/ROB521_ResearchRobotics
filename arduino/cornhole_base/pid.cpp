
#include "pid.h"
#include "Arduino.h"

// Constructor
pid::pid();

void pid::setpid(float Kp, float Ki, float Kd){
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

float pid::updatePID(float setPoint, float currentVal){

  float err = setPoint - currentVal;
  float dT = (millis()  - _tLast)/ 0.001 ;
  _tLast = millis();
  
  Serial.print("\nIteration Time: ");
  Serial.println(dT);
 
  _integral   = _integral + err * dT;
  
  float derivative = (err - _error) / dT;

  _error = err;
  
  return _Kp*err + _Ki *_integral + _Kd * derivative;

}
