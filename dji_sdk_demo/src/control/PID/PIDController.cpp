#include"PIDController.h"

PIDController::PIDController()
{
}

PIDController::PIDController(double kP,double kI,double kD,double maxPIDOut)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->maxPIDOut = maxPIDOut;
    P =  I = D = 0;
    prev_err = last_err = cur_err = 0;
    lastPIDOut = 0;
}

PIDController::~PIDController()
{
}

//增量式PID控制
double PIDController::getOutputByAdd(double curerr)
{
    prev_err = last_err;
	last_err = cur_err;
	cur_err  = curerr;
    P = kP *(cur_err-last_err);
    I = kI * cur_err;
    D = kD *(cur_err- 2*last_err + prev_err);
    double pwm_value = P + I + D;//增量结果
	
    if(pwm_value > 0.2)
       pwm_value = 0.2;
    if(pwm_value <= -0.2)
       pwm_value = -0.2;
	
	double PIDOut = lastPIDOut + pwm_value;
	lastPIDOut = PIDOut;
    if(PIDOut > maxPIDOut)
       PIDOut = maxPIDOut;
    if(PIDOut < -maxPIDOut)
       PIDOut = -maxPIDOut;
	return PIDOut;     
}

