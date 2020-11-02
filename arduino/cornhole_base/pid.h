#ifndef pid_h
#define pid_h

class pid {
  public:
    pid();
    void  setpid(float Kp, float Ki, float Kd);
    float updatePID(float setPoint, float currentVal);

  private:
    float _Kp = 0.5;
    float _Ki = 0.5;
    float _Kd = 0.5;
    float _error;
    float _integral;
    
    unsigned long _tLast; 
    
};

#endif
