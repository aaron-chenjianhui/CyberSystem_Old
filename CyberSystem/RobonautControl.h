#ifndef SIMUROBOCONTROL_H
#define SIMUROBOCONTROL_H

#define NOMINMAX		// In Order to Use max(a, b) and min(a, b)

#include "SocketBlockClient.h"
#include "CSocket.hpp"
#include "RobonautData.h"
#include <QMessageBox>
#include <QString>
#include <inlib.h>
#include <stdio.h>

#include <Windows.h>
#include <mmsystem.h>

#include <SAHandCtrlApi.h>


#define SETRANGE(X,MIN_DATA,MAX_DATA) if(X < MIN_DATA) X = MIN_DATA; if(X > MAX_DATA) X = MAX_DATA;
#define ADDCOUNT(X,MAXCOUNT) if(X < MAXCOUNT) ++X; else X = 0;

// define min and max degree for every joint
#define  ROBO_J0_MIN -60
#define  ROBO_J0_MAX 180
#define  ROBO_J1_MIN -110
#define  ROBO_J1_MAX 90
#define  ROBO_J2_MIN -120
#define  ROBO_J2_MAX 120
#define  ROBO_J3_MIN -130
#define  ROBO_J3_MAX 10
#define  ROBO_J4_MIN -120
#define  ROBO_J4_MAX 120
#define  ROBO_J5_MIN -90
#define  ROBO_J5_MAX 90
#define  ROBO_J6_MIN -120
#define  ROBO_J6_MAX 120



#define  ROBO_J0_MIN_SLI -6000
#define  ROBO_J0_MAX_SLI 18000
#define  ROBO_J1_MIN_SLI -11000
#define  ROBO_J1_MAX_SLI 9000
#define  ROBO_J2_MIN_SLI -12000
#define  ROBO_J2_MAX_SLI 12000
#define  ROBO_J3_MIN_SLI -13000
#define  ROBO_J3_MAX_SLI 1000
#define  ROBO_J4_MIN_SLI -12000
#define  ROBO_J4_MAX_SLI 12000
#define  ROBO_J5_MIN_SLI -9000
#define  ROBO_J5_MAX_SLI 9000
#define  ROBO_J6_MIN_SLI -12000
#define  ROBO_J6_MAX_SLI 12000

class RobonautControl
{
public:
	RobonautControl();
	~RobonautControl();
	enum HandMode{Position, Impedance, Soft, ZeroForce, Reset};

	//*********************** Consimu Options ***********************//
public:
	bool ConnConsimu(); // Initialize the connection
	bool DisConnConsimu(); // Disconnect
	bool SendConsimuMsg(); //	Return true: Send Success
private:	
	CSocketBlockClient m_ClientSocketPredictive; // 仿真通信


	//*********************** Robonaut Options ***********************//
public:
	bool ConnRobo(); // Initialize the control connection
	bool DisConnRobo(); // Disconnect
	bool SendRoboMsg(); // timer of RoboCtrl
	bool RecvRoboMsg();
	void RoboDataCnv(const int &src_flag, const CRobonautData &src_data, char dst_buf[]);
	void BuffParse();
private:	
	sockconn::CUdpClient m_ClientSocketRobo; // 机器人宇航员发送数据
	sockconn::CUdpServer m_ServerSocketRobo; // 机器人宇航员接收数据
	int m_DecideFlag;

	//*********************** Hand Options ***********************//
public:
	// 5 Hand Control
	bool ConnHand();
	bool DisConnHand();
	bool SendHandMsg(const CHandData &RHandCmd, const CHandData &LHandCmd, int HCount);
	bool RecvHandMsg(CHandData &RHandSensor, CHandData &LHandSensor);
	void setHandInit(bool);
	void setHandEnable(bool);
	void setHandEmergency(bool);
	void setHandMode(HandMode);

private:
	CSocketBlockClient m_SendClientHand;		// Send Hand Data
	CSocketBlockClient m_ReceiveClientHand;		// Receive Hand Data
	char m_SendHandCommandBuffer[1024];		// Hand Command Send Buffer

	int m_HandInit;		// 0: Out/1: Initialized
	int m_HandEnable;		// 0: Out/1: Initialized
	int m_HandStop;		// 1为急停，2为取消急停
	int m_Impedance_Index;		// 0: Out/1: Initialized
	int m_HandMode;



public:
	int m_nPCount; // consimulation communication counter
	bool m_bConsimuSockConn; //		True: Consimulation Socket is Created
	// TODO(CJH): Delete
	bool m_bRoboCtrlInitFini; // robonaut control connection is ok
	char cSendbufferp[ORDER_BUF_LEN];
	char cRevbufferp[ORDER_BUF_LEN];

	char cSendRobotCommandBuffer[ORDER_BUF_LEN];
	int m_nRCount; // robonaut control communication counter
};





#endif