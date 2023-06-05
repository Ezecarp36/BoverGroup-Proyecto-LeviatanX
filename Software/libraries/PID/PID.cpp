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
    if(millis() > currentTimePID + tick_pid)//realiza un millis cada cierto tiempo y actualiza la lectura pid . debe ser lo mas minimo 
    {
        currentTimePID = millis();
        input = inp; 
        error = input - setPoint; //calulo error
        //prevenir problemas de "windup"
        if (integral < integralMin){
            integral = integralMin;
        }
        else if (integral > integralMax){
            integral = integralMax;
        }
        else 
            integral += error; //suma los errores pasados

        derivada = error - lastError; //toma importancia en los cambios bruscos por eso error - error pasado 
        double out = kp * error+ ti * integral + td * derivada; //calculo salida pid
        lastError = error; // actualiza error pasado

        return out; //retorna pid  
    }
    else return 0.0;// si ocurre algo con millis devuelve un 0.0 como error
    
}
