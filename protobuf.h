#include "stdmsg.pb.h"
#include "motion.h"
#include <iostream>
#include "MecanumControl.h"

using namespace stdmsg;
using namespace std;

void CreateMsg(Pose_Velocity* pMsg, double pose[]);
int SendMsg(SOCKET sockfd, const sockaddr* serverAddr, int len, Pose_Velocity* pMsg);
void DisplayMsg(Pose_Velocity* pMsg, ECAT::motion_network_t& net);
int RecieveMsg(SOCKET sockfd, sockaddr* serverAddr, int* len, ECAT::motion_network_t& net);