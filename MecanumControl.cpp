#include <Eigen/Dense>
#include "PIDControl.h"
#include "windows.h"
#include <iostream>
#include <motion.h>
#include <sstream>
#include <vector>
#include "MecanumControl.h"
#include <fstream>
#include <math.h>
#include "stdmsg.pb.h"
#include "protobuf.h"


using namespace std;
using namespace Eigen;

extern Vector3d pos_initial;
extern Vector4d wheel_theta_initial;
extern double la;
extern double lb;
extern double r;
extern double dperiod; //Control period time
extern PIDControl x_controller;
extern PIDControl y_controller;
extern PIDControl z_controller;
extern MatrixXd Ja;
extern MatrixXd Jb;
extern Vector3d v_car;//Target velocity when use vel control
extern Vector3d p_car;//Target position when use pos control
extern Vector4d w_car; //Velocity of each wheel send to the servo driver
extern Vector4d sensor_w_car; //Velocity of each wheel collected by sensor
extern Vector3d sensor_v_car;
extern Vector4d sensor_theta_car; //Rotate degree of each wheel collected by sensor
extern Vector3d sensor_pos_car; //The position of car calculate by algorithm
extern Vector4d last_sensor_w_car; //The variable to save the velocity of wheels last turn
extern bool slip_check_flag;	//Flag to indicate the car slip or not, true is slip
//extern ECAT::motion_network_t& net;
extern ECAT::motion_ctrl_t* motor[];
extern int test_obj;
extern size_t motor_number;
extern char stop_flag;
extern char reach_flag;
extern int param[];
extern double PI;
extern double w_limit;
extern stringstream line;
extern string cmd;
extern unsigned int time;
extern int send_flag;
extern int rec_flag;

void Wait(LONG ms)
{
	DWORD lT_old = GetTickCount();
	DWORD lT = lT_old;
	do
	{
		lT = GetTickCount();
	} while (lT - lT_old < (DWORD)ms);
}

void InitVariable()
{
	Ja << -1, 1, (la + lb),
		1, 1, -(la + lb),
		-1, 1, -(la + lb),
		1, 1, (la + lb);
	Ja = Ja*(1.0 / r);

	Jb << -(la + lb), (la + lb), -(la + lb), (la + lb),
		(la + lb), (la + lb), (la + lb), (la + lb),
		1, -1, -1, 1;
	Jb = Jb *(r / (4 * (la + lb)));
}

//Calculate the velocity of each wheel according to the kinematic model
void InverseSolution(const Vector3d &v_car, Vector4d &w_car)
{
	w_car = Ja*v_car;
	w_car = w_car * 40;
	for (int i = 0; i < 4; i++)//限制轮子的转速
	{
		if (w_car(i)>w_limit)
		{
			w_car(i) = w_limit;
		}
		if (w_car(i) < -w_limit)
		{
			w_car(i) = -w_limit;
		}
	}
}

void ForwardSolution(const Vector4d &sensor_w_car, Vector3d &sensor_v_car)
{
	sensor_v_car = Jb*sensor_w_car;
}

//Wheel slip detection
bool SlipCheck(const Vector4d &sensor_w_car)
{
	double slip_check = 0;
	double error = 0.01;
	slip_check = sensor_w_car(0) + sensor_w_car(1) - sensor_w_car(2) - sensor_w_car(3);
	if (slip_check<error)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//Dead reckonging, position control,based on floor coordinate
bool DeadReckonging(Vector4d &last_sensor_w_car,  Vector4d &sensor_w_car,  Vector4d &sensor_theta_car, Vector3d &sensor_pos_car)
{
	//Calc the theta of car
	for (int i = 0; i < 4; i++)
	{
		if ((sensor_w_car(i) < 0.01) && (sensor_w_car(i)>-0.01))
		{
			sensor_w_car(i) = 0;
		}
	}
	sensor_pos_car(2) = r / (4 * (la + lb))*(sensor_theta_car(0) - sensor_theta_car(1) - sensor_theta_car(2)
		+ sensor_theta_car(3)) + pos_initial(2)
		- r / (4 * (la + lb))*(wheel_theta_initial(0) - wheel_theta_initial(1) - wheel_theta_initial(2) + wheel_theta_initial(3));

	//Calc the x pos of car
	double a = cos(sensor_pos_car(2)) + sin(sensor_pos_car(2));
	double b = cos(sensor_pos_car(2)) - sin(sensor_pos_car(2));
	Vector4d temp_vector(-a, b, -a, b);
	sensor_pos_car(0) = sensor_pos_car(0) + dperiod*r / 8 * temp_vector.dot(last_sensor_w_car + sensor_w_car);

	//Calc the y pos of car
	temp_vector << b, a, b, a;
	sensor_pos_car(1) = sensor_pos_car(1) + dperiod*r / 8 * temp_vector.dot(last_sensor_w_car + sensor_w_car);

	//Save the velociy of wheels this turn
	last_sensor_w_car = sensor_w_car;

	//Check if the wheels slip
	return SlipCheck(sensor_w_car);
}

//Translate the floor coordinate to robot coordinate
Vector3d FloorToRobot(const Vector3d &input, double theta)
{
	Matrix3d VT;
	VT << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1;
	return VT*input;
}

//Control the car by position
bool PosControlLoop(Vector4d &last_sensor_w_car,  Vector4d &sensor_w_car, Vector4d &sensor_theta_car, Vector3d &sensor_pos_car, Vector4d &w_car)
{
	bool slip_check_flag = 0;
	slip_check_flag = DeadReckonging(last_sensor_w_car, sensor_w_car, sensor_theta_car, sensor_pos_car);
	Vector3d temp_vector;
	temp_vector(0) = x_controller.PIDLoop(sensor_pos_car(0));
	temp_vector(1) = y_controller.PIDLoop(sensor_pos_car(1));
	temp_vector(2) = z_controller.PIDLoop(sensor_pos_car(2));
	temp_vector = FloorToRobot(temp_vector, sensor_pos_car(2));
	InverseSolution(temp_vector, w_car);
	RadToVel(w_car);
	VelToEncoder(w_car);
	return slip_check_flag;
}

//Control the car by velocity
bool VelControlLoop(Vector3d &v_car, Vector4d &sensor_w_car, Vector4d &w_car, ECAT::motion_network_t& net)
{
	string temp;
	InverseSolution(v_car, w_car);//Update the vel target of each wheel
	RadToVel(w_car);
	VelToEncoder(w_car);
	temp = SetWheelVel(param,net);
	//cout << temp << endl;
	bool slip_check_flag = 0;
	slip_check_flag = SlipCheck(sensor_w_car);
	return slip_check_flag;
}

//Update the position target to PID controllers 
void UpdatePosTarget(Vector3d &p_car)
{
	x_controller.UpdateTarget(p_car(0));
	y_controller.UpdateTarget(p_car(1));
	z_controller.UpdateTarget(p_car(2));
}

void VelToEncoder(Vector4d &data)//转速r/s转换成驱动器的输入单位
{
	for (int i = 0; i < 4; i++)
	{
		data(i) = 10000.0 * data(i);
	}
}

void EncoderToVel(Vector4d &data)//驱动器的输入单位转换成转速r/s
{
	for (int i = 0; i < 4; i++)
	{
		data(i) = data(i)/10000.0;
	}
}

void RadToVel(Vector4d &data)//弧度制的角速度转换成转速r/s
{
	for (int i = 0; i < 4; i++)
	{
		data(i) =  data(i)/(2*PI);
	}
}

void VelToRad(Vector4d &data)//转速r/s转换成弧度制的角速度
{
	for (int i = 0; i < 4; i++)
	{
		data(i) = data(i) * (2 * PI);
	}
}

string SetWheelVel(int* param, ECAT::motion_network_t& net)
{
	char flag[4] = { 1, 1, 1, 1 };
	string temp;
	for (size_t i = 0; i < motor_number; i++)
	{
		flag[i]=motor[i]->set_velocity(pow(-1.0,*(param+i))*((int32_t)w_car(i)),100000,100000);
	}
	net.sync();
	net.sync();
	temp=flag[0]+flag[1]+flag[2]+flag[3];
	return temp;
}

void SaveWheelVel(string file_name)
{
	stop_flag = 0;
	ofstream outfile(file_name);
	int32_t spd;
	int32_t* spd_ptr = &spd;
	if (test_obj != 4)
	{
		while (stop_flag == 0)
		{
			motor[test_obj]->velocity(spd_ptr);
			outfile << spd << endl;
			Sleep(30);
		}
	}
	else
	{
		while (stop_flag == 0)
		{
			for (size_t i = 0; i < motor_number; i++)
			{

				motor[i]->velocity(spd_ptr);
				outfile << spd << " ";
			}
			outfile << endl;
			Sleep(30);
		}
	}
}

void StopMotor(ECAT::motion_network_t& net)
{
	for (size_t i = 0; i < motor_number; i++)
	{
		motor[i]->halt();
	}
	net.sync();
	net.sync();
}

string SwitchMotorStatus(ECAT::motion_network_t& net)//切换电机的使能状态，是否接受指令
{
	char temp;
	char* temp_ptr = &temp;
	char* status;
	status = new char[motor_number];
	motor[0]->is_enabled(temp_ptr);
	if (temp == 0)
	{
		for (size_t i = 0; i < motor_number; i++)
		{
			status[i]=motor[i]->enable();
		}
		net.sync();
		net.sync();
	}
	else
	{
		for (size_t i = 0; i < motor_number; i++)
		{
			status[i]=motor[i]->disable();
		}
		net.sync();
		net.sync();
	}
	string string;
	for (size_t i = 0; i < motor_number; i++)
	{
		string += status[i];
	}
	return string;
}

void UpdateCurrentState(int* param)//读取编码器的当前值
{
	int32_t spd;
	int32_t* spd_ptr = &spd;
	int32_t pos;
	int32_t* pos_ptr = &pos;
	for (size_t i = 0; i < motor_number; i++)
	{
		motor[i]->velocity(spd_ptr);
		sensor_w_car(i) = pow(-1.0,*(param+i))*((double)spd)/40.0;
		motor[i]->position(pos_ptr);
		sensor_theta_car(i) = pow(-1.0,*(param+i))*((double)pos)/40.0;
	}
	EncoderToVel(sensor_w_car);
	EncoderToVel(sensor_theta_car);
}

void CheckStatus(ECAT::motion_network_t& net)
{
	int32_t last_pos[4] = { 0, 0, 0, 0 };
	int32_t* last_pos_ptr = last_pos;
	int32_t pos[4] = { 0, 0, 0, 0 };
	int32_t* pos_ptr = pos;
	int check_rusult[4] = { 0, 0, 0, 0 };
	for (size_t i = 0; i < motor_number; i++)
	{
		motor[i]->position(last_pos_ptr+i);
	}
	w_car << 10000, 10000, 10000, 10000;
	SetWheelVel(param, net);
	Sleep(1000);
	w_car = Vector4d::Zero();
	SetWheelVel(param, net);
	for (size_t i = 0; i < motor_number; i++)
	{
		motor[i]->position(pos_ptr+i);
		if ((abs(*(pos_ptr+i)-*(last_pos_ptr+i))>10))
		{
			check_rusult[i] = 1;
			cout << "Driver "<<i+1<<" status:"<<check_rusult[i] << endl;
		}
	}
}

void ExitLoop(ECAT::motion_network_t* net)
{
	if (stop_flag == 1)
	{ 
		cout << "CMD:";
	}
	else{
		cout << "Looooooping......." << endl;
	}
	fflush(stdout);
	string temp;
	//cout << "Please input q(quit) to exit the loop." << endl;
	//fflush(stdout);
	getline(std::cin, temp);
	if (temp == "q")
	{
		stop_flag = 1;
		cout << "Exit Loop." << endl;
		cmd = "";
		for (size_t i = 0; i < motor_number; i++)
		{
			motor[i]->halt();
			(*net).sync();
			(*net).sync();
			/*motor[i]->disable();
			net.sync();
			net.sync();*/
		}
	}
	else if (temp == "l")//l for loop
	{
		stop_flag = 0;
		cmd = "";
	}
	else{
		line.str(temp);
		line >> cmd;
	}
	//Sleep(30);
}

void GoToPos(ofstream &outfile, ECAT::motion_network_t& net)
{
	unsigned int time = GetTickCount();
	UpdatePosTarget(p_car);
	while ((!stop_flag)&&(!reach_flag))
	{
		UpdateCurrentState(param);
		VelToRad(sensor_w_car);
		ForwardSolution(sensor_w_car, sensor_v_car);
		VelToRad(sensor_theta_car);
		dperiod = (double)(GetTickCount() - time) / 1000.0;
		time = GetTickCount();
		PosControlLoop(last_sensor_w_car, sensor_w_car, sensor_theta_car, sensor_pos_car, w_car);
		SaveData(outfile);
		/*if (abs(x_controller.GetError()) < 0.001 && abs(y_controller.GetError())< 0.001 && abs(z_controller.GetError())< 0.0005)
		{
			w_car = Vector4d::Zero();
			SetWheelVel(param);
			string temp;
			temp = "Pos target is reached.";
			cout << temp << endl;
			reach_flag = 1;
		}
		else{*/
		SetWheelVel(param, net);
		//}
		Sleep(20);
	}
}


void Square(double x, double y, int num, ofstream &outfile, ECAT::motion_network_t& net)
{
	for (int i = 0; i < num; i++)
	{
		p_car(0) = sensor_pos_car(0) + x;
		p_car(1) = sensor_pos_car(1);
		p_car(2) = sensor_pos_car(2);
		GoToPos(outfile, net);
		reach_flag = 0;

		p_car(0) = sensor_pos_car(0);
		p_car(1) = sensor_pos_car(1) + y;
		p_car(2) = sensor_pos_car(2);
		GoToPos(outfile, net);
		reach_flag = 0;

		p_car(0) = sensor_pos_car(0) - x;
		p_car(1) = sensor_pos_car(1);
		p_car(2) = sensor_pos_car(2);
		GoToPos(outfile, net);
		reach_flag = 0;

		p_car(0) = sensor_pos_car(0);
		p_car(1) = sensor_pos_car(1) - y;
		p_car(2) = sensor_pos_car(2);
		GoToPos(outfile, net);
		reach_flag = 0;
	}
}

void SaveData(ofstream &outfile)
{
	outfile << sensor_pos_car(0) << " " << sensor_pos_car(1) << " " << sensor_pos_car(2) << " "
		<< sensor_v_car(0) << " " << sensor_v_car(1) << " " << sensor_v_car(2) << " " << GetTickCount() << endl;
}

void Circle(double r, double T, int num, ofstream &outfile, ECAT::motion_network_t& net)
{
	unsigned int time = GetTickCount();
	double start_time = (double)GetTickCount() / 1000.0;
	double current_time = 0;;
	double w = 2 * PI / T;
	while (!stop_flag)
	{
		UpdateCurrentState(param);
		VelToRad(sensor_w_car);
		ForwardSolution(sensor_w_car, sensor_v_car);
		VelToRad(sensor_theta_car);
		dperiod = (double)(GetTickCount() - time) / 1000.0;
		time = GetTickCount();
		DeadReckonging(last_sensor_w_car, sensor_w_car, sensor_theta_car, sensor_pos_car);
		SaveData(outfile);
		current_time = (double)GetTickCount() / 1000.0 - start_time;
		v_car(0) = w*r*cos(w*current_time);
		v_car(1) = -w*r*sin(w*current_time);
		v_car(2) = 0;
		InverseSolution(v_car, w_car);//Update the vel target of each wheel
		RadToVel(w_car);
		VelToEncoder(w_car);
		SetWheelVel(param, net);
		Sleep(20);
	}
	w_car = Vector4d::Zero();
	SetWheelVel(param, net);
}

void UpdateCarState(SOCKET sockfd, const sockaddr* serverAddr, int len, double msg[], ECAT::motion_network_t& net)
{
	UpdateCurrentState(param);
	VelToRad(sensor_w_car);
	ForwardSolution(sensor_w_car, sensor_v_car);
	VelToRad(sensor_theta_car);
	dperiod = (double)(GetTickCount() - time) / 1000.0;
	time = GetTickCount();
	//cout << dperiod*1000.0 << endl;
	DeadReckonging(last_sensor_w_car, sensor_w_car, sensor_theta_car, sensor_pos_car);
	//double msg[6] = { 1.0, 2.0, 1.0, 2.0, 1.0, 2.0 };
	double msg1[6] = { sensor_pos_car(0), sensor_pos_car(1), sensor_pos_car(2), sensor_v_car(0), sensor_v_car(1), sensor_v_car(2)};
	Pose_Velocity* pMsg = new Pose_Velocity();
	CreateMsg(pMsg, msg1);
	send_flag = SendMsg(sockfd, serverAddr, len, pMsg);
	rec_flag = RecieveMsg(sockfd, (struct sockaddr *)&serverAddr, &len,net);
	Wait(20);
}
