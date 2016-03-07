#include <Eigen/Dense>
#include "PIDControl.h"
#include "windows.h"
#include <iostream>
#include <motion.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;


void Wait(LONG ms);
void InitVariable();
void InverseSolution(const Vector3d &v_car, Vector4d &w_car);
void ForwardSolution(const Vector4d &sensor_w_car, Vector3d &sensor_v_car);
bool SlipCheck(const Vector4d &sensor_w_car);
bool DeadReckonging(Vector4d &last_sensor_w_car, Vector4d &sensor_w_car,  Vector4d &sensor_theta_car, Vector3d &sensor_pos_car);
Vector3d FloorToRobot(const Vector3d &input, double theta);
bool PosControlLoop(Vector4d &last_sensor_w_car,  Vector4d &sensor_w_car, Vector4d &sensor_theta_car, Vector3d &sensor_pos_car, Vector4d &w_car);
bool VelControlLoop(Vector3d &v_car, Vector4d &sensor_w_car, Vector4d &w_car, ECAT::motion_network_t& net);
void UpdatePosTarget(Vector3d &p_car);
void VelToEncoder(Vector4d &data);//转速转换成驱动器的输入单位
void EncoderToVel(Vector4d &data);
void RadToVel(Vector4d &data);
void VelToRad(Vector4d &data);
string SetWheelVel(int* param, ECAT::motion_network_t& net);//设置轴转速
void SaveWheelVel(string file_name);
void StopMotor(ECAT::motion_network_t& net);
string SwitchMotorStatus();
void UpdateCurrentState(int* param);
void CheckStatus(ECAT::motion_network_t& net);
void ExitLoop(ECAT::motion_network_t* net);
void GoToPos(ofstream &outfile, ECAT::motion_network_t& net);
void Square(double x, double y, int num, ofstream &outfile, ECAT::motion_network_t& net);
void SaveData(ofstream &outfile);
void Circle(double r, double T, int num, ofstream &outfile, ECAT::motion_network_t& net);
void UpdateCarState(SOCKET sockfd, const sockaddr* serverAddr, int len, double msg[], ECAT::motion_network_t& net,ofstream &outfile);