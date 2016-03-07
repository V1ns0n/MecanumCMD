#include "protobuf.h"

using namespace stdmsg;
using namespace std;

//extern ECAT::motion_network_t& net;
extern Vector4d w_car;
extern Vector4d sensor_w_car;
extern Vector3d v_car;

void CreateMsg(Pose_Velocity* pMsg, double msg[])
{
	pMsg->set_x(msg[0]);
	pMsg->set_y(msg[1]);
	pMsg->set_yaw(msg[2]);
	pMsg->set_vx(msg[3]);
	pMsg->set_vy(msg[4]);
	pMsg->set_w(msg[5]);
}

int SendMsg(SOCKET sockfd, const sockaddr* serverAddr, int len, Pose_Velocity* pMsg)
{
	string messageBuffer;
	int n;
	pMsg->SerializeToString(&messageBuffer);

	// send a message to the robot
	n = sendto(sockfd, messageBuffer.c_str(), messageBuffer.length(), 0, serverAddr, len);
	if (n < 0)
	{
		printf("Error send message\n");
	}
	return n;
}

void DisplayMsg(Pose_Velocity* pMsg, ECAT::motion_network_t& net)
{
	cout << pMsg->vx() / 10.0 << "," << pMsg->vy() / 10.0 << "," << pMsg->w() / 10.0 << endl;
	v_car << pMsg->vx() / 10.0, pMsg->vy() / 10.0, pMsg->w() / 10.0;
	VelControlLoop(v_car, sensor_w_car, w_car, net);
}

int RecieveMsg(SOCKET sockfd, sockaddr* serverAddr, int* len, ECAT::motion_network_t& net)
{
	int n;
	char protoMessage[1400];
	n = recvfrom(sockfd, protoMessage, 1400, 0, serverAddr, len);
	if (n < 0)
	{
		printf("Error receive message\n");
		return n;
	}
	// deserialize inbound message
	Pose_Velocity *pMsg = new Pose_Velocity();
	pMsg->ParseFromArray(protoMessage, n);
	DisplayMsg(pMsg,net);
	delete pMsg;
	return n;
}