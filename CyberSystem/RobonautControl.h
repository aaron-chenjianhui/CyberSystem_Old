#ifndef SIMUROBOCONTROL_H
#define SIMUROBOCONTROL_H
// in order to use min(a, b) and max(a, b)
#define NOMINMAX

#include "SocketBlockClient.h"
#include "CSocket.hpp"
#include "RobonautData.h"
#include <QMessageBox>
#include <QString>
#include <inlib.h>
//#include "QFile"
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


	//*********************** Consimu Options ***********************//
public:
	bool ConnConsimu(); // Initialize the connection
	bool DisConnConsimu(); // Disconnect
	void SimuControl(); // Start the con-simulation
	bool SendConsimuMsg(); //	Return true: Send Success
private:	
	CSocketBlockClient m_ClientSocketPredictive; // 仿真通信
	SOCKET m_hSocketSendCmd; //发送套接字
	SOCKADDR_IN addrSendCmdHost;  //发送套接字地址
	SOCKADDR_IN addrSendCmdRemote;
	WSABUF wsaSendBuf;


	//*********************** Robonaut Options ***********************//
public:
	bool ConnRobo(); // Initialize the control connection
	bool DisConnRobo(); // Disconnect
	bool SendRoboMsg(); // timer of RoboCtrl
	bool RecvRoboMsg();
	void DataConvert(const int &src_flag, const CRobonautData &src_data, char dst_buf[]);
	void BuffParse();
private:	
	sockconn::CUdpClient m_ClientSocketRobo; // 机器人宇航员发送数据
	sockconn::CUdpServer m_ServerSocketRobo; // 机器人宇航员接收数据
	int m_DecideFlag;

	//*********************** Hand Options ***********************//
public:
	// 5 Hand Control
	void HandInit();
	void HandControl();
	void SendHandData();
	void setPosMode();
	void setImpMode();
	void setResMode();
private:
	// 5 hand control
	CSAHandCtrlApi m_HandApi;
	SAH_DESIRED	HandDesirePos;

public:
	int m_nPCount; // consimulation communication counter
	bool m_bConsimuSockConn; //		True: Consimulation Socket is Created
	// TODO(CJH): Delete
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


	
};





#endif