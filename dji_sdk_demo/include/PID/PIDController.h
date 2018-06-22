#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

class PIDController
{
public:
    PIDController();
    PIDController(double kP,double kI,double kD,double maxPIDOut);
    ~PIDController();
    double kP, kI, kD;
    double maxPIDOut;
    double P, I, D;
    double prev_err, last_err, cur_err;
    double lastPIDOut;

    double getOutputByAdd(double curerr);//增量式PID控制
};

#endif



