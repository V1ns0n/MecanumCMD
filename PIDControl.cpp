#include "PIDControl.h"
#include <iostream>


PIDControl::PIDControl(double Kp_param, double Ki_param, double Kd_param)
{
   Kp=Kp_param;
   Ki=Ki_param;
   Kd=Kd_param;
   target=0;
   current=0;
   for(int i=0;i<3;i++)
   {
       error[i]=0;
   }
   output=0;
}


PIDControl::~PIDControl(void)
{
}

void PIDControl::UpdateCurrent(double current_param)
{
    current=current_param;
}

void PIDControl::UpdateTarget(double target_param)
{
    target=target_param;
}

double PIDControl::UpdateOutput()
{
    return output;
}

void PIDControl::PIDAlgorithm()
{
    error[2]=error[1];
    error[1]=error[0];
    error[0]=target-current;

    output=output+Kp*(error[0]-error[1])+Ki*error[0]+Kd*(error[0]-2*error[1]+error[2]);
}

double PIDControl::PIDLoop(double current_param)
{
     PIDControl::UpdateCurrent(current_param);
     PIDControl::PIDAlgorithm();
    return  PIDControl::UpdateOutput();
}

void PIDControl::UpdatePIDParam(double p, double i, double d)
{
	Kp=p;
	Ki=i;
	Kd=d;
}

double PIDControl::GetPIDParam(char param)
{
	switch(param)
	{
		case'p':
			return Kp;
		case'i':
			return Ki;
		case'd':
			return Kd;
		default:
			std::cout<<"Type error,use pid"<<std::endl;
			return 0;
	}
}

double PIDControl::GetError()
{
	return error[0];
}