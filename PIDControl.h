#pragma once
class PIDControl
{
public:
	PIDControl(double,double,double);
	~PIDControl(void);
	void UpdateCurrent(double);
    void UpdateTarget(double);
	void UpdatePIDParam(double,double,double);
	double GetPIDParam(char);
    void PIDAlgorithm();
    double PIDLoop(double);
    double UpdateOutput();
	double GetError();
private:
    double Kp,Ki,Kd;
    double target;
    double current;
    double error[3];
    double output;
};

