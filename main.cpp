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

#pragma comment(lib, "Ws2_32.lib")      // socket lib

using namespace std;
using namespace Eigen;
using namespace stdmsg;

//全局变量
Vector3d pos_initial = Vector3d::Zero();
Vector4d wheel_theta_initial = Vector4d::Zero();
double la = 0.308;
double lb = 0.38;
double r = 0.127;
double dperiod = 0.05; //Control period time
PIDControl x_controller = PIDControl(1.2, 0, 0);
PIDControl y_controller = PIDControl(1.2, 0, 0);
PIDControl z_controller = PIDControl(1, 0, 0);
MatrixXd Ja(4, 3);
MatrixXd Jb(3, 4);
Vector3d v_car(0.0, 0.0, 0.0);//Target velocity when use vel control
Vector3d p_car(0.0, 0.0, 0.0);//Target position when use pos control
Vector4d w_car; //Velocity of each wheel send to the servo driver
Vector4d sensor_w_car = Vector4d::Zero(); //Velocity of each wheel collected by sensor
Vector4d sensor_theta_car = Vector4d::Zero(); //Rotate degree of each wheel collected by sensor
Vector3d sensor_v_car = Vector3d::Zero(); //The velocity of car calculate by algorithm
Vector3d sensor_pos_car = Vector3d::Zero(); //The position of car calculate by algorithm
Vector4d last_sensor_w_car = Vector4d::Zero(); //The variable to save the velocity of wheels last turn
bool slip_check_flag = 0;	//Flag to indicate the car slip or not, true is slip
//ECAT::motion_network_t& net=ECAT::motion_network_t::get_instance();
ECAT::motion_ctrl_t* motor[4];
int test_obj = 0;//命令模式下操作驱动器的编号
size_t motor_number;
char stop_flag = 1;
char reach_flag = 0;
double PI = 3.1415926;
double w_limit = 100.0; //此值除以2*PI为轴的转速限制，再除以40为轮子的转速限制
int param[4] = { 0, 1, 1, 0 };
stringstream line("");
std::string cmd;
static int portNumber = 8888;
int send_flag = 0;
int rec_flag = 0;
unsigned int time;

DWORD WINAPI Thread1(LPVOID pM)
{
	ECAT::motion_network_t* net = (ECAT::motion_network_t*) pM;
	while (1)
	{
		ExitLoop(net);
	}
	return 0;
}

DWORD WINAPI Thread2(LPVOID pM)
{
	ECAT::motion_network_t* net = (ECAT::motion_network_t*) pM;
	SOCKET sockfd;
	struct sockaddr_in serverAddr, clientAddr;
	int len;
	// Init winsock 
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		fprintf(stderr, "Could not open Windows connection.\n");
		exit(0);
	}

	// create socket to listen on
	sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);

	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	serverAddr.sin_port = htons(portNumber);
	len = sizeof(clientAddr);
	double msg[6] = { 1.0, 2.0, 1.0, 2.0, 1.0, 2.0 };
	while (1)
	{
		if (!stop_flag)
		{
			/*for (int i = 0; i < 6; i++)
			{
				msg[i] = msg[i] + 0.1;
			}*/
			UpdateCarState(sockfd, (struct sockaddr *)&serverAddr, len, msg, *net);
		}
	}
	return 0;
}

int main(int argc, char** argv)
{
	SOCKET sockfd;
	struct sockaddr_in serverAddr, clientAddr;
	int len;
	// Init winsock 
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		fprintf(stderr, "Could not open Windows connection.\n");
		exit(0);
	}

	// create socket to listen on
	sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);

	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	serverAddr.sin_port = htons(portNumber);
	len = sizeof(clientAddr);

	ECAT::motion_network_t& net = ECAT::motion_network_t::get_instance();
	cout << net.scan()<<endl;
	net.op();
	Sleep(100);

	InitVariable();
	char flag = false;
	int return_flag = 1;
	motor_number = net.motor_size();
	for (UINT32 i = 0; i <motor_number; i++)
	{
		motor[i] = net[i];
		UINT32 j = 0;
		while (!flag)
		{
			motor[i]->is_enabled(&flag);
			motor[i]->enable();
			net.sync();
			if (j++ > 1000)
			{
				cout << "Motor[" << i << "]:" << " Enable failed"<<endl;
				break;
			}
		}
		cout << "Motor[" << i << "]:" << "Enabled" << endl;
	}
	for (size_t i = 0; i < motor_number; i++)
	{
		motor[i]->set_mode(ECAT::motion_ctrl_t::PROFILED_VELOCITY);
		net.sync();
		net.sync();
	}
	
	time = GetTickCount();
	HANDLE handle = CreateThread(NULL, 0, Thread1, &net, 0, NULL);//另开一个线程接收控制指令
	HANDLE handle1 = CreateThread(NULL, 0, Thread2, &net, 0, NULL);//另开一个线程接收控制指令

	while (1)
	{
		if (cmd == "w")
		{
			int number;
			line>>number;
			int spd;
			line >> spd;
			return_flag=motor[number]->set_velocity(spd, 100000, 100000);
			net.sync();
			net.sync();
			cout << return_flag << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "isen")
		{
			int number;
			line >> number;
			char flag=false;
			
			motor[number]->is_enabled(&flag);
			cout << (int)flag << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "dis")
		{
			for (size_t i = 0; i < motor_number; i++)
			{
				motor[i]->disable();
			}
			cmd = "";
			line.clear();
		}
		else if (cmd == "en")
		{
			int number;
			line >> number;
			char flag = false;
			string temp;

			if (number == 0)
			{
				for (size_t i = 0; i < motor_number; i++)
				{
					motor[i]->enable();
					motor[number]->is_enabled(&flag);
					net.sync();
					temp += flag;
					temp += " ";
				}
				cout << temp << endl;
			}
			else
			{
				motor[number]->enable();
				motor[number]->is_enabled(&flag);
				net.sync();
				cout << flag << endl;
			}
			cmd = "";
			line.clear();
		}
		else if (cmd == "st")
		{
			for (size_t i = 0; i < motor_number; i++)
			{
				motor[i]->halt();
				net.sync();
				net.sync();
			}
			cmd = "";
			line.clear();
		}
		else if (cmd == "v")//速度模式，设置速度 命令实例：v 1 2 3
		{
			double vx, vy, w;
			line >> vx >> vy >> w;
			v_car << vx, vy, w;
			VelControlLoop(v_car, sensor_w_car, w_car,net);
			cout << vx << "," << vy << "," << w << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "p")//位置控制，设置目标点，命令实例： p 1 2 3
		{
			double px, py, w;
			string filename;
			line >> px >> py >> w >>filename;
			filename += ".txt";
			ofstream outfile(filename);
			p_car << px, py, w;//把目标点写入全局变量p_car
			stop_flag = 0;//清除暂停标志位
			reach_flag = 0;//清除上到达目标点的标志位
			GoToPos(outfile, net);//驶向目标点
			outfile.close();
			cmd = "";
			line.clear();
			stop_flag = 1;//抵达目标点后将暂停标志位置1
		}
		else if (cmd == "sq")
		{
			double x, y;
			int num;
			string filename;
			line >> x >> y >> num>>filename;
			filename += ".txt";
			ofstream outfile(filename);
			stop_flag = 0;
			Square(x, y, num, outfile, net);
			outfile.close();
			cmd = "";
			line.clear();
			stop_flag = 1;
		}
		else if (cmd == "cir")
		{
			double r, T;
			int num;
			string filename;
			line >> r >> T >> num >> filename;
			filename += ".txt";
			ofstream outfile(filename);
			stop_flag = 0;
			Circle(r, T, num, outfile, net);
			outfile.close();
			cmd = "";
			line.clear();
		}
		else if (cmd == "pos")
		{
			cout << sensor_pos_car << endl;
			UpdateCurrentState(param);
			cout << sensor_theta_car << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "clr")
		{
			pos_initial = Vector3d::Zero();
			sensor_pos_car = Vector3d::Zero();
			UpdateCurrentState(param);
			VelToRad(sensor_theta_car);
			for (size_t i = 0; i < motor_number; i++)
			{
				wheel_theta_initial = sensor_theta_car;
			}
			cmd = "";
			line.clear();
		}
		else if (cmd == "ck")
		{
			CheckStatus(net);
			cmd = "";
			line.clear();
		}
		else if (cmd == "pix")
		{
			double p, i;
			double d = 0;
			line >> p >> i;
			x_controller.UpdatePIDParam(p, i, d);
			cout << x_controller.GetPIDParam('p') << ' ' << x_controller.GetPIDParam('i') << ' ' << x_controller.GetPIDParam('d') << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "piy")
		{
			double p, i;
			double d = 0;
			line >> p >> i;
			y_controller.UpdatePIDParam(p, i, d);
			cout << y_controller.GetPIDParam('p') << ' ' << y_controller.GetPIDParam('i') << ' ' << y_controller.GetPIDParam('d') << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "piz")
		{
			double p, i;
			double d = 0;
			line >> p >> i;
			z_controller.UpdatePIDParam(p, i, d);
			cout << z_controller.GetPIDParam('p') << ' ' << z_controller.GetPIDParam('i') << ' ' << z_controller.GetPIDParam('d') << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd == "send")
		{
			double msg[6] = { 1.0, 2.0, 1.0, 2.0, 1.0, 2.0};
			Pose_Velocity* pMsg = new Pose_Velocity();
			CreateMsg(pMsg, msg);
			send_flag = SendMsg(sockfd, (struct sockaddr *)&serverAddr, len, pMsg);
			cout << "Send successfully" << endl;
			rec_flag = RecieveMsg(sockfd, (struct sockaddr *)&serverAddr, &len,net);
			cout << "Recieve successfully" << endl;
			cmd = "";
			line.clear();
		}
		else if (cmd=="rec")
		{
			rec_flag = RecieveMsg(sockfd, (struct sockaddr *)&serverAddr, &len,net);
			cout << "Recieve successfully" << endl;
			cmd = "";
			line.clear();
		}
	}
	//while (1){}
	return 0;
}