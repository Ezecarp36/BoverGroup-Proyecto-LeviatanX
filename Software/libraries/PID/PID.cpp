#include "PID.h"


Pid::Pid(double p, double i, double d,double sp, double tick)
{
    kp = p;
    ti = i;
    td = d;
    setPoint = sp;
    tick_pid = tick;
}


double Pid::ComputePid(double inp)
{
    if(millis() > currentTimePID + tick_pid)
    {
        currentTimePID = millis();
        input = inp;
        error = input - setPoint;
        if (integral < integralMin){
            integral = integralMin;
        }
        else if (integral > integralMax){
            integral = integralMax;
        }
        else 
            integral += error;

        derivada = error - lastError;
        double out = kp * error+ ti * integral + td * derivada;
        lastError = error;

        return out;
    }
    else return 0.0;
    
}
