#ifndef _PID_H
#define _PID_H
#include "Arduino.h"

class Pid
{
    private:
    double kp = 0;
    double ti = 0;
    double td = 0;
    double error;
    double lastError;
    double input = 0;
    double output = 0; 
    double setPoint = 0;
    double derivada;
    double integral;
    double integralMin;
    double integralMax;
    int tick_pid;

    public:
    Pid(double p, double d,double sp, double tick);
    double ComputePid(double inp);
};
#endif