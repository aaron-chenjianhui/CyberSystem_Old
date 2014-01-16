#ifndef SIMUROBOCONTROL_H
#define SIMUROBOCONTROL_H
// in order to use min(a, b) and max(a, b)
#define NOMINMAX

#include "SocketBlockClient.h"
#include "RobonautData.h"
#include <QMessageBox>
#include <QString>
#include <inlib.h>
//#include "QFile"
#include <stdio.h>

#include <Windows.h>
#include <mmsystem.h>

#include <SAHandCtrlApi.h>


// define min and max degree for every joint
#define  ROBO_J0_MIN -120
#define  ROBO_J0_MAX 85
#define  ROBO_J1_MIN -120
#define  ROBO_J1_MAX 0
#define  ROBO_J2_MIN -90
#define  ROBO_J2_MAX 90
#define  ROBO_J3_MIN -100
#define  ROBO_J3_MAX 45
#define  ROBO_J4_MIN -87
#define  ROBO_J4_MAX 113
#define  ROBO_J5_MIN -87
#define  ROBO_J5_MAX 113
#define  ROBO_J6_MIN -70
#define  ROBO_J6_MAX 70



class RobonautControl
{
public:
	RobonautControl();
	~RobonautControl();


public:
	CSocketBlockClient m_ClientSocketPredictive; // 仿真通信

	SOCKET m_hSocketSendCmd; //发送套接字
	SOCKADDR_IN addrSendCmdHost;  //发送套接字地址
	SOCKADDR_IN addrSendCmdRemote;
	WSABUF wsaSendBuf;

	
	void InitConsimuConn(); // Initialize the connection
	void DisConsimuConn(); // Disconnect
	void SimuControl(); // Start the con-simulation
	void SendConsimuMsg(); // timer of SimuControl

	void InitRoboCtrlConn(); // Initialize the control connection
	void DisRoboCtrlConn(); // Disconnect
	void RoboCtrl(); // send command data to robonaut
	void SendCmdData(); // timer of RoboCtrl

	// calculate inverse kinematics
	void CalInKine(float ArmCartes[], float ArmJoint[]);

	// 5 Hand Control
	void HandInit();
	void HandControl();
	void SendHandData();
	void setPosMode();
	void setImpMode();
	void setResMode();

public:
	int m_nRetConSer; // flag to judge whether consimulation is connected
	int m_nRetRoboCtrlSer; // flag to judge whether robonaut control is connected

	int m_nPCount; // consimulation communication counter
	bool m_bConsimuInitFini; // consimulation connection is ok
	bool m_bRoboCtrlInitFini; // robonaut control connection is ok
	char cSendbufferp[ORDER_BUF_LEN];
	char cRevbufferp[ORDER_BUF_LEN];

	char cSendRobotCommandBuffer[ORDER_BUF_LEN];
	int m_nRCount; // robonaut control communication counter

	// data save control
	FILE *fileToTxt; 
	BOOL m_bSaveData;
	BOOL m_bSaveDataFinish;

	// 5 hand control
	BOOL m_bGloveControl;

	double Force[5];

	float afPos1[3];
	float afPos2[3];
	float afPos3[3];
	float afPos4[3];
	float afPos5[3];

private:
	// 5 hand control
	CSAHandCtrlApi m_HandApi;
	SAH_DESIRED	HandDesirePos;
	
};





#endif