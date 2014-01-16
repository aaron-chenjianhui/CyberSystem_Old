#include "RobonautControl.h"
#include <atlstr.h>
#include <tchar.h>




// declare global variable for interconnection
extern CRobonautData g_RobotCmdDeg;     //全局机器人控制指令单位角度
extern CRobonautData g_SRobotCmdDeg;    //发送的安全机器人控制指令单位角度
extern CRobonautData g_RobotSensorDeg;  //全局机器人传感器数据单位角度

// declare global variable for interconnection
extern float rTrackerRealPose[6];
extern float lTrackerRealPose[6];
extern float m_rGloveRealData[5][3];
extern float m_lGloveRealData[5][3];

extern float g_rightArmJointBuf[7];	// global data from slider for joint control
extern float g_leftArmJointBuf[7];		// global data from slider for joint control


// declare global multimedia timer
UINT PTimerId=-1;
void PASCAL TimerProcPredictDis(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

UINT STimerId=-1;
void PASCAL TimerProcSendCmd(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

UINT HTimerId=-1;
void PASCAL TimerProcHandCtrl(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

RobonautControl *g_pRobonautCtrl;

// Collision flag
int g_nCollisionFlag=0;

//爆炸标志位333关，222开
int g_nRunFlag=222; 

// 0 means out of control, 1 means use cyber to control, 2 means use slider to control
int g_RobotCtrlMode;		



RobonautControl::RobonautControl()
{
	g_pRobonautCtrl = this;

	// Initialize data
	m_bConsimuInitFini = false;
	m_nPCount = 0;

	m_hSocketSendCmd = NULL;
	m_nRCount = 0;

	// m_nRetConSer != 0, means consimulation is disconnected
	m_nRetConSer = -1;
	// m_nRetRoboCtrlSer != 0, means robonaut control  is disconnected
	m_nRetRoboCtrlSer = -1;

	for (int i = 0; i<ORDER_BUF_LEN; i++)
	{
		cSendbufferp[i] = '\0';
		cRevbufferp[i] = '\0';
		cSendRobotCommandBuffer[i] = '\0';
	}

	m_bSaveData = FALSE;
	m_bSaveDataFinish = FALSE;

	// Initialize control mode
	g_RobotCtrlMode = 0;

	fileToTxt = fopen("Record\\Data.txt","w+");

	// initialize 5 hand control data
	for (int i=0;i<3;i++)
	{
		afPos1[i] = 0;
		afPos2[i] = 0;
		afPos3[i] = 0;
		afPos4[i] = 0;
		afPos5[i] = 0;
	}

	m_bGloveControl = false;

	for (int i=0;i<5;i++)
	{
		Force[i] = 0;
	}
}

RobonautControl::~RobonautControl()
{
	closesocket(m_ClientSocketPredictive.m_hSocket);
	WSACleanup();
	m_ClientSocketPredictive.m_hSocket = NULL;

	timeKillEvent(PTimerId);
}



//*********************** Consimulation Options ***********************//
void RobonautControl::InitConsimuConn()
{
	if (m_bConsimuInitFini == false)
	{
		UINT PREDICTIVE_Port = PREDICTIVE_PORT; 
		char *pPREDICTIVE_IP = PREDICTIVE_IP;

		m_nRetConSer = m_ClientSocketPredictive.ConnectServer(pPREDICTIVE_IP,PREDICTIVE_Port);

		if (m_nRetConSer == 0)
		{
			QMessageBox::about(NULL, "About", "Communication is connected");
			m_bConsimuInitFini = true;
		}
	}
}

void RobonautControl::DisConsimuConn()
{
	m_bConsimuInitFini = false;
}


void RobonautControl::SimuControl()
{
	if (m_bConsimuInitFini == true)
	{
		PTimerId = timeSetEvent(250,1,(LPTIMECALLBACK)TimerProcPredictDis,0,TIME_PERIODIC);
	}
}


void PASCAL TimerProcPredictDis(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	g_pRobonautCtrl->SendConsimuMsg();
}

void RobonautControl::SendConsimuMsg()
{
	if (m_bConsimuInitFini == true)
	{
		// judge control mode
		if(g_RobotCtrlMode == 1)
		{
			// calculate the inverse kinematics
			CalInKine(rTrackerRealPose, g_RobotCmdDeg.rightArmJoint);
			CalInKine(lTrackerRealPose, g_RobotCmdDeg.leftArmJoint);
		}

		if (g_RobotCtrlMode == 2)
		{
			for (int i=0;i<7;i++)
			{
				g_RobotCmdDeg.rightArmJoint[i] = g_rightArmJointBuf[i];
				g_RobotCmdDeg.leftArmJoint[i] = g_leftArmJointBuf[i];
			}
		}

		if (g_RobotCtrlMode == 0)
		{
			return;
		}

		m_nPCount++;

		// get the format data sending to simulation
		sprintf(cSendbufferp, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",m_nPCount,
			g_RobotCmdDeg.leftArmJoint[0],g_RobotCmdDeg.leftArmJoint[1],g_RobotCmdDeg.leftArmJoint[2],g_RobotCmdDeg.leftArmJoint[3],
			g_RobotCmdDeg.leftArmJoint[4],g_RobotCmdDeg.leftArmJoint[5],g_RobotCmdDeg.leftArmJoint[6],g_RobotCmdDeg.rightArmJoint[0],
			g_RobotCmdDeg.rightArmJoint[1],g_RobotCmdDeg.rightArmJoint[2],g_RobotCmdDeg.rightArmJoint[3],g_RobotCmdDeg.rightArmJoint[4],
			g_RobotCmdDeg.rightArmJoint[5],g_RobotCmdDeg.rightArmJoint[6],g_RobotCmdDeg.headJoint[0],g_RobotCmdDeg.headJoint[1],
			g_RobotCmdDeg.headJoint[2],g_RobotCmdDeg.waistJoint[0],g_RobotCmdDeg.waistJoint[1]);

		int retSendData= m_ClientSocketPredictive.SendData(cSendbufferp);

		int retReceiveData = m_ClientSocketPredictive.ReceiveData(cRevbufferp);

		sscanf(cRevbufferp, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&g_nCollisionFlag,
			&g_SRobotCmdDeg.leftArmJoint[0],&g_SRobotCmdDeg.leftArmJoint[1],&g_SRobotCmdDeg.leftArmJoint[2],&g_SRobotCmdDeg.leftArmJoint[3],
			&g_SRobotCmdDeg.leftArmJoint[4],&g_SRobotCmdDeg.leftArmJoint[5],&g_SRobotCmdDeg.leftArmJoint[6],&g_SRobotCmdDeg.rightArmJoint[0],
			&g_SRobotCmdDeg.rightArmJoint[1],&g_SRobotCmdDeg.rightArmJoint[2],&g_SRobotCmdDeg.rightArmJoint[3],&g_SRobotCmdDeg.rightArmJoint[4],
			&g_SRobotCmdDeg.rightArmJoint[5],&g_SRobotCmdDeg.rightArmJoint[6],&g_SRobotCmdDeg.headJoint[0],&g_SRobotCmdDeg.headJoint[1],
			&g_SRobotCmdDeg.headJoint[2],&g_SRobotCmdDeg.waistJoint[0],&g_SRobotCmdDeg.waistJoint[1]);

		if(retReceiveData==0)
		{
			QString strReceiveData;
			strReceiveData = QString("CollisionFlag: %1\r\n").arg(g_nCollisionFlag);

			if (g_nCollisionFlag!=0)
			{
				//				       g_RobotCmdDeg.flag=FALSE;		
			}
			//SetDlgItemText(IDC_EDITCOLLISIONFLAG,strReceiveData);

		}	
		float mmmionij=0.0;

		if (g_nCollisionFlag!=0)
		{
			mmmionij=1;
		}
		else
		{

			mmmionij=0;
		}
	}	
}



//*********************** Robonaut control Options ***********************//
void RobonautControl::InitRoboCtrlConn()
{
	if (m_bRoboCtrlInitFini == false)
	{
		if(m_hSocketSendCmd == NULL)
		{
			int ret;

			WSADATA wsaData;

			ret = WSAStartup(0x202, &wsaData);

			if(ret != 0)
			{
				//TRACE("WSAStartup Error!");
				QMessageBox::about(NULL, "About", "WSASocket1");

			}


			//创建套接字
			m_hSocketSendCmd = WSASocket(AF_INET,SOCK_DGRAM,IPPROTO_UDP,NULL,0,WSA_FLAG_OVERLAPPED);

			if(m_hSocketSendCmd == INVALID_SOCKET)
			{
				QMessageBox::about(NULL, "About", "WSASocket2");
			}

			memset(&addrSendCmdHost,0,sizeof(addrSendCmdHost));
			memset(&addrSendCmdRemote,0,sizeof(addrSendCmdRemote));

			//将套接字绑定到本机端口
			unsigned short port = ROB_TELE_PORT;
			addrSendCmdHost.sin_family = PF_INET;
			addrSendCmdHost.sin_port = htons(port);
			addrSendCmdHost.sin_addr.s_addr = inet_addr(ROB_TELE_IP);		

			//////远程IP和端口
			addrSendCmdRemote.sin_family = PF_INET;
			addrSendCmdRemote.sin_port = htons(port);
			addrSendCmdRemote.sin_addr.s_addr = inet_addr(ROB_CONTRL_IP);	

			m_nRetRoboCtrlSer = bind(m_hSocketSendCmd,(struct sockaddr*)&addrSendCmdHost,sizeof(addrSendCmdHost));

			if(m_nRetRoboCtrlSer == SOCKET_ERROR)
			{
				QMessageBox::about(NULL, "About", "WSASocket3");
			}	

			if(m_nRetRoboCtrlSer == 0)
			{
				QMessageBox::about(NULL, "About", "Robonaut Control is connected");
				m_bRoboCtrlInitFini = true;
			}
		}
		else
		{
			closesocket(m_hSocketSendCmd);
			WSACleanup();
			m_hSocketSendCmd = NULL;
			timeKillEvent(STimerId);		
		}
	}	
}



void RobonautControl::DisRoboCtrlConn()
{
	m_bRoboCtrlInitFini = false;
}



void RobonautControl::RoboCtrl()
{
	if (m_bRoboCtrlInitFini == true)
	{
		STimerId = timeSetEvent(250,1,(LPTIMECALLBACK)TimerProcSendCmd,0,TIME_PERIODIC);
		if (STimerId==NULL)
		{
			//	MessageBox("TimerId error");
		}
	}
}



void PASCAL TimerProcSendCmd(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	//	CTaskPlanDlg* pDlg = (CTaskPlanDlg*)dwUser;
	g_pRobonautCtrl->SendCmdData();

}

void RobonautControl::SendCmdData()
{
	// judge control mode
	if(g_RobotCtrlMode == 1)
	{
		// calculate the inverse kinematics
		CalInKine(rTrackerRealPose, g_RobotCmdDeg.rightArmJoint);
		CalInKine(lTrackerRealPose, g_RobotCmdDeg.leftArmJoint);
	}

	if (g_RobotCtrlMode == 2)
	{
		for (int i=0;i<7;i++)
		{
			g_RobotCmdDeg.rightArmJoint[i] = g_rightArmJointBuf[i];
			g_RobotCmdDeg.leftArmJoint[i] = g_leftArmJointBuf[i];
		}
	}

	if (g_RobotCtrlMode == 0)
	{
		return;
	}
	
	
	if(m_nRCount > 99999)
		m_nRCount = 0;
	else				
		m_nRCount++;
	//////////////////////////////////////////////////////////////////////////
	//************限位***********//		
	if (g_RobotCmdDeg.leftArmJoint[0] < -49.0)
	{
		g_RobotCmdDeg.leftArmJoint[0] = -49.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[0] >170.0)
	{
		g_RobotCmdDeg.leftArmJoint[0] = 170.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[1]< -100.0)  //
	{
		g_RobotCmdDeg.leftArmJoint[1] = -100.0;  //
	}
	else if (g_RobotCmdDeg.leftArmJoint[1] > 70.0) //
	{
		g_RobotCmdDeg.leftArmJoint[1] = 70.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[2] < -120.0)   //
	{
		g_RobotCmdDeg.leftArmJoint[2] = -120.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[2] > 120.0)  // 
	{
		g_RobotCmdDeg.leftArmJoint[2] = 120.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[3] > 0.0)
	{
		g_RobotCmdDeg.leftArmJoint[3] = 0.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[3] < -129.0)
	{
		g_RobotCmdDeg.leftArmJoint[3] = -129.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[4] < -120.0)
	{
		g_RobotCmdDeg.leftArmJoint[4] = -120.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[4] > 120.0)
	{
		g_RobotCmdDeg.leftArmJoint[4] = 120.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[5] > 90.0)
	{
		g_RobotCmdDeg.leftArmJoint[5] = 90.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[5] < -90.0)
	{
		g_RobotCmdDeg.leftArmJoint[5] = -90.0;
	}

	if (g_RobotCmdDeg.leftArmJoint[6] > 120.0)
	{
		g_RobotCmdDeg.leftArmJoint[6] = 120.0;
	}
	else if (g_RobotCmdDeg.leftArmJoint[6] < -120.0)
	{
		g_RobotCmdDeg.leftArmJoint[6] = -120.0;
	}

	g_RobotCmdDeg.count++;

	if(g_RobotCmdDeg.count >= 1000)
		g_RobotCmdDeg.count = 0;

	//////////////////////////////////////////////////////////////////////////
	//碰撞时在预测仿真端显示实际角度和碰撞角度闪烁
	// 			if(g_nCollisionFlag!=0)  
	// 			{
	// 				for (int i=0;i<7;i++)
	// 				{
	// 					g_RobotCmdDeg.leftArmJoint[i]=g_RobotCmdDeg.leftArmJoint[i];
	// 					g_RobotCmdDeg.rightArmJoint[i]=g_RobotCmdDeg.rightArmJoint[i];
	// 				}
	// 				for (i=0;i<3;i++)
	// 				{
	// 					g_RobotCmdDeg.headJoint[i]=g_RobotCmdDeg.headJoint[i];
	// 				}
	// 
	// 				for (i=0;i<2;i++)
	// 				{
	// 					g_RobotCmdDeg.waistJoint[i]=g_RobotCmdDeg.waistJoint[i];
	// 				}
	// 			
	// 			}

	//////////////////////////////////////////////////////////////////////////
	//			
	cSendRobotCommandBuffer[0] = LOBYTE(LOWORD(g_RobotCmdDeg.count));         //??????
	cSendRobotCommandBuffer[1] = HIBYTE(LOWORD(g_RobotCmdDeg.count));         //??????

	cSendRobotCommandBuffer[2] = LOBYTE(LOWORD(g_RobotCmdDeg.CtlMode));         //????:??(1)/????(2)
	cSendRobotCommandBuffer[3] = HIBYTE(LOWORD(g_RobotCmdDeg.CtlMode));         //????:??(1)/????(2)

	cSendRobotCommandBuffer[4] = LOBYTE(LOWORD(g_nRunFlag));  //
	cSendRobotCommandBuffer[5] = HIBYTE(LOWORD(g_nRunFlag));   

	cSendRobotCommandBuffer[6] = LOBYTE(HIWORD(g_nRunFlag));  //
	cSendRobotCommandBuffer[7] = HIBYTE(HIWORD(g_nRunFlag));   

	//??
	cSendRobotCommandBuffer[8] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[0]*10000)));         //??1???
	cSendRobotCommandBuffer[9] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[0]*10000)));         
	cSendRobotCommandBuffer[10] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[0]*10000)));         
	cSendRobotCommandBuffer[11] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[0]*10000)));         

	cSendRobotCommandBuffer[12] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[1]*10000)));         //??2???
	cSendRobotCommandBuffer[13] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[1]*10000)));         
	cSendRobotCommandBuffer[14] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[1]*10000)));        
	cSendRobotCommandBuffer[15] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[1]*10000)));        

	cSendRobotCommandBuffer[16] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[2]*10000)));        //??3???
	cSendRobotCommandBuffer[17] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[2]*10000)));        
	cSendRobotCommandBuffer[18] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[2]*10000)));        
	cSendRobotCommandBuffer[19] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[2]*10000)));  

	cSendRobotCommandBuffer[20] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[3]*10000)));        //??4???
	cSendRobotCommandBuffer[21] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[3]*10000)));        
	cSendRobotCommandBuffer[22] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[3]*10000)));        
	cSendRobotCommandBuffer[23] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[3]*10000)));        

	cSendRobotCommandBuffer[24] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[4]*10000)));        //??5???  
	cSendRobotCommandBuffer[25] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[4]*10000)));        
	cSendRobotCommandBuffer[26] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[4]*10000)));        
	cSendRobotCommandBuffer[27] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[4]*10000)));        

	cSendRobotCommandBuffer[28] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[5]*10000)));        //??6???   
	cSendRobotCommandBuffer[29] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[5]*10000)));        
	cSendRobotCommandBuffer[30] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[5]*10000)));        
	cSendRobotCommandBuffer[31] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[5]*10000)));       

	cSendRobotCommandBuffer[32] = LOBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[6]*10000)));          //?????   
	cSendRobotCommandBuffer[33] = HIBYTE(LOWORD(int(g_RobotCmdDeg.leftArmJoint[6]*10000)));         
	cSendRobotCommandBuffer[34] = LOBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[6]*10000)));         
	cSendRobotCommandBuffer[35] = HIBYTE(HIWORD(int(g_RobotCmdDeg.leftArmJoint[6]*10000))); 	

	//??
	cSendRobotCommandBuffer[36] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[0]*10000)));         //??1???
	cSendRobotCommandBuffer[37] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[0]*10000)));         
	cSendRobotCommandBuffer[38] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[0]*10000)));         
	cSendRobotCommandBuffer[39] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[0]*10000)));         

	cSendRobotCommandBuffer[40] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[1]*10000)));         //??2???
	cSendRobotCommandBuffer[41] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[1]*10000)));         
	cSendRobotCommandBuffer[42] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[1]*10000)));        
	cSendRobotCommandBuffer[43] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[1]*10000)));        

	cSendRobotCommandBuffer[44] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[2]*10000)));        //??3???
	cSendRobotCommandBuffer[45] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[2]*10000)));        
	cSendRobotCommandBuffer[46] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[2]*10000)));        
	cSendRobotCommandBuffer[47] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[2]*10000)));  

	cSendRobotCommandBuffer[48] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[3]*10000)));        //??4???
	cSendRobotCommandBuffer[49] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[3]*10000)));        
	cSendRobotCommandBuffer[50] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[3]*10000)));        
	cSendRobotCommandBuffer[51] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[3]*10000)));        

	cSendRobotCommandBuffer[52] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[4]*10000)));        //??5???  
	cSendRobotCommandBuffer[53] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[4]*10000)));        
	cSendRobotCommandBuffer[54] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[4]*10000)));        
	cSendRobotCommandBuffer[55] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[4]*10000)));        

	cSendRobotCommandBuffer[56] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[5]*10000)));        //??6???   
	cSendRobotCommandBuffer[57] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[5]*10000)));        
	cSendRobotCommandBuffer[58] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[5]*10000)));        
	cSendRobotCommandBuffer[59] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[5]*10000)));       

	cSendRobotCommandBuffer[60] = LOBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[6]*10000)));          
	cSendRobotCommandBuffer[61] = HIBYTE(LOWORD(int(g_RobotCmdDeg.rightArmJoint[6]*10000)));         
	cSendRobotCommandBuffer[62] = LOBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[6]*10000)));         
	cSendRobotCommandBuffer[63] = HIBYTE(HIWORD(int(g_RobotCmdDeg.rightArmJoint[6]*10000))); 

	//??
	cSendRobotCommandBuffer[64] = LOBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[0]*10000)));         //??1???
	cSendRobotCommandBuffer[65] = HIBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[0]*10000)));         
	cSendRobotCommandBuffer[66] = LOBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[0]*10000)));         
	cSendRobotCommandBuffer[67] = HIBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[0]*10000)));         

	cSendRobotCommandBuffer[68] = LOBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[1]*10000)));         //??2???
	cSendRobotCommandBuffer[69] = HIBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[1]*10000)));         
	cSendRobotCommandBuffer[70] = LOBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[1]*10000)));        
	cSendRobotCommandBuffer[71] = HIBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[1]*10000)));        

	cSendRobotCommandBuffer[72] = LOBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[2]*10000)));        //??3???
	cSendRobotCommandBuffer[73] = HIBYTE(LOWORD(int(g_RobotCmdDeg.headJoint[2]*10000)));        
	cSendRobotCommandBuffer[74] = LOBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[2]*10000)));        
	cSendRobotCommandBuffer[75] = HIBYTE(HIWORD(int(g_RobotCmdDeg.headJoint[2]*10000)));

	//??

	cSendRobotCommandBuffer[76] = LOBYTE(LOWORD(int(g_RobotCmdDeg.waistJoint[0]*10000)));         //??1???
	cSendRobotCommandBuffer[77] = HIBYTE(LOWORD(int(g_RobotCmdDeg.waistJoint[0]*10000)));         
	cSendRobotCommandBuffer[78] = LOBYTE(HIWORD(int(g_RobotCmdDeg.waistJoint[0]*10000)));         
	cSendRobotCommandBuffer[79] = HIBYTE(HIWORD(int(g_RobotCmdDeg.waistJoint[0]*10000)));         

	cSendRobotCommandBuffer[80] = LOBYTE(LOWORD(int(g_RobotCmdDeg.waistJoint[1]*10000)));         //??2???
	cSendRobotCommandBuffer[81] = HIBYTE(LOWORD(int(g_RobotCmdDeg.waistJoint[1]*10000)));         
	cSendRobotCommandBuffer[82] = LOBYTE(HIWORD(int(g_RobotCmdDeg.waistJoint[1]*10000)));        
	cSendRobotCommandBuffer[83] = HIBYTE(HIWORD(int(g_RobotCmdDeg.waistJoint[1]*10000))); 



	//unsigned long cbRet = 0;

	int nReturnCode = -1;		

	DWORD dFlag = 0;
	DWORD BytesofSend;

	int iLen = sizeof(SOCKADDR_IN);  

	int LastError = 0;

	wsaSendBuf.buf = cSendRobotCommandBuffer;
	wsaSendBuf.len = sizeof(cSendRobotCommandBuffer);

	

	nReturnCode = WSASendTo(m_hSocketSendCmd,&wsaSendBuf,1,&BytesofSend,dFlag,(struct sockaddr*)&addrSendCmdRemote,iLen,NULL,NULL);			

	if(nReturnCode == 0)
	{	
		CString str;
		str.Format(_T("%s:%d %d %d %d\r\nRight: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\r\nLeft: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f"),//%d\r\n%d",
			ROB_CONTRL_IP, ROB_TELE_PORT,
			MAKEWORD(cSendRobotCommandBuffer[0],cSendRobotCommandBuffer[1]),
			MAKEWORD(cSendRobotCommandBuffer[2],cSendRobotCommandBuffer[3]),
			MAKEWORD(cSendRobotCommandBuffer[4],cSendRobotCommandBuffer[5]),
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[36],cSendRobotCommandBuffer[37]),MAKEWORD(cSendRobotCommandBuffer[38],cSendRobotCommandBuffer[39]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[40],cSendRobotCommandBuffer[41]),MAKEWORD(cSendRobotCommandBuffer[42],cSendRobotCommandBuffer[43]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[44],cSendRobotCommandBuffer[45]),MAKEWORD(cSendRobotCommandBuffer[46],cSendRobotCommandBuffer[47]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[48],cSendRobotCommandBuffer[49]),MAKEWORD(cSendRobotCommandBuffer[50],cSendRobotCommandBuffer[51]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[52],cSendRobotCommandBuffer[53]),MAKEWORD(cSendRobotCommandBuffer[54],cSendRobotCommandBuffer[55]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[56],cSendRobotCommandBuffer[57]),MAKEWORD(cSendRobotCommandBuffer[58],cSendRobotCommandBuffer[59]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[60],cSendRobotCommandBuffer[61]),MAKEWORD(cSendRobotCommandBuffer[62],cSendRobotCommandBuffer[63]))*0.0001,

			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[8],cSendRobotCommandBuffer[9]),MAKEWORD(cSendRobotCommandBuffer[10],cSendRobotCommandBuffer[11]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[12],cSendRobotCommandBuffer[13]),MAKEWORD(cSendRobotCommandBuffer[14],cSendRobotCommandBuffer[15]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[16],cSendRobotCommandBuffer[17]),MAKEWORD(cSendRobotCommandBuffer[18],cSendRobotCommandBuffer[19]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[20],cSendRobotCommandBuffer[21]),MAKEWORD(cSendRobotCommandBuffer[22],cSendRobotCommandBuffer[23]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[24],cSendRobotCommandBuffer[25]),MAKEWORD(cSendRobotCommandBuffer[26],cSendRobotCommandBuffer[27]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[28],cSendRobotCommandBuffer[29]),MAKEWORD(cSendRobotCommandBuffer[30],cSendRobotCommandBuffer[31]))*0.0001,
			MAKELONG(MAKEWORD(cSendRobotCommandBuffer[32],cSendRobotCommandBuffer[33]),MAKEWORD(cSendRobotCommandBuffer[34],cSendRobotCommandBuffer[35]))*0.0001
			// 				m_nVICount,
			// 				m_VMotion
			);
		//SetDlgItemText(IDC_SENDCMDROB,str);
	}
	else     
	{
		m_nRCount = 0;
		QMessageBox::about(NULL, "About", "Send Error!");  
	}

	//保存数据
	CString cstemp; 
	cstemp.Format(_T("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n"),
		g_RobotCmdDeg.rightArmJoint[0],g_RobotCmdDeg.rightArmJoint[1],g_RobotCmdDeg.rightArmJoint[2],
		g_RobotCmdDeg.rightArmJoint[3],g_RobotCmdDeg.rightArmJoint[4],g_RobotCmdDeg.rightArmJoint[5],
		g_RobotCmdDeg.rightArmJoint[6],
		g_RobotSensorDeg.rightArmJoint[0],g_RobotSensorDeg.rightArmJoint[1],g_RobotSensorDeg.rightArmJoint[2],
		g_RobotSensorDeg.rightArmJoint[3],g_RobotSensorDeg.rightArmJoint[4],g_RobotSensorDeg.rightArmJoint[5],
		g_RobotSensorDeg.rightArmJoint[6],
		g_RobotSensorDeg.RightJointFT[0],g_RobotSensorDeg.RightJointFT[1],g_RobotSensorDeg.RightJointFT[2],
		g_RobotSensorDeg.RightJointFT[3],g_RobotSensorDeg.RightJointFT[4],g_RobotSensorDeg.RightJointFT[5],
		g_RobotSensorDeg.RightJointFT[6]);

	// 			CString cstemparmangle;
	// 			cstemparmangle.Format("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
	// 				m_farmanglerange1[0],m_farmanglerange1[1],m_farmanglerange1[2],m_farmanglerange1[3],m_farmanglerange1[4],m_farmanglerange1[5],
	// 				m_farmanglerange2[0],m_farmanglerange2[1],m_farmanglerange2[2],m_farmanglerange2[3],m_farmanglerange2[4],m_farmanglerange2[5],
	// 				m_farmanglerange3[0],m_farmanglerange3[1],m_farmanglerange3[2],m_farmanglerange3[3],m_farmanglerange3[4],m_farmanglerange3[5],
	// 				m_farmanglerange4[0],m_farmanglerange4[1],m_farmanglerange4[2],m_farmanglerange4[3],m_farmanglerange4[4],m_farmanglerange4[5],
	// 				optarmangle1[2],optarmangle[0]
	//			);

	if(m_bSaveData)
	{
		fwrite(cstemp,1,cstemp.GetLength(),fileToTxt);
		// 	fwrite(cstemparmangle,1,cstemparmangle.GetLength(),fileToTxt);
	}
	if (m_bSaveDataFinish)
	{
		fclose(fileToTxt);
	}
}



//*********************** Kinematics Calculation ***********************//
// Transfer in coordinates in Cartesian axis, transfer out coordinates in Joint axis
void RobonautControl::CalInKine(float ArmCartes[], float ArmJoint[])
{
// 	ArmJoint[0] = 0;
// 	ArmJoint[1] = 0;
// 	ArmJoint[2] = 0;
// 	ArmJoint[3] = 0;
// 	ArmJoint[4] = 0;
// 	ArmJoint[5] = 0;
// 	ArmJoint[6] = 0;

	ArmJoint[0] = rTrackerRealPose[0];
	ArmJoint[1] = rTrackerRealPose[1];
	ArmJoint[2] = rTrackerRealPose[2];
	ArmJoint[3] = 0;
	ArmJoint[4] = 0;
	ArmJoint[5] = 0;
	ArmJoint[6] = 0;
}


//*********************** 5 Hand Control ***********************//
void RobonautControl::HandInit()
{
	int res;

	//===> Initialize the Hand
	res=m_HandApi.SAHandInit();

	//===> If the Return Value is negative then it is Error Code
	//===> Verify the error codes in the programmers guide
	if (res<=0)
	{
		//===> Do appropriate Error Handling Action
		QMessageBox::about(NULL, "Error", "Init Failed.\n!!");
	}

	m_HandApi.SetFingerEnable(PORT_1,THUMB,TRUE);
	m_HandApi.SetFingerEnable(PORT_1,FIRST_FINGER,TRUE);
	m_HandApi.SetFingerEnable(PORT_1,MIDDLE_FINGER,TRUE);
	m_HandApi.SetFingerEnable(PORT_1,RING_FINGER,TRUE);
	m_HandApi.SetFingerEnable(PORT_1,LITTLE_FINGER,TRUE);
}


void RobonautControl::HandControl()
{
	HTimerId = timeSetEvent(250,1,(LPTIMECALLBACK)TimerProcHandCtrl,0,TIME_PERIODIC);
	if (HTimerId==NULL)
	{
		//	MessageBox("TimerId error");
	}
}

void PASCAL TimerProcHandCtrl(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	//	CTaskPlanDlg* pDlg = (CTaskPlanDlg*)dwUser;
	g_pRobonautCtrl->SendHandData();

}

void RobonautControl::SendHandData()
{
	for(int i=0; i<2; i++)
	{
		afPos1[i] = m_rGloveRealData[0][1-i];
		afPos2[i] = m_rGloveRealData[1][1-i];
		afPos3[i] = m_rGloveRealData[2][1-i];
		afPos4[i] = m_rGloveRealData[3][1-i];
		afPos5[i] = m_rGloveRealData[4][1-i];
	}
	afPos1[2] = m_rGloveRealData[0][2];
	afPos2[2] = m_rGloveRealData[1][2];
	afPos3[2] = m_rGloveRealData[2][2];
	afPos4[2] = m_rGloveRealData[3][2];
	afPos5[2] = m_rGloveRealData[4][2];

	if(m_bGloveControl)
	{
		//	m_HandApi.MoveHand(PORT_1, &HandDesirePos);		

		//	float afPos[3]={20.0f,20.0f,5.0f};
		float afVel[3]={80.0f,80.0f,80.0f};

		m_HandApi.MoveFinger(PORT_1,THUMB,afPos1,afVel,0.05);
		m_HandApi.MoveFinger(PORT_1,FIRST_FINGER,afPos2,afVel,0.05);
		m_HandApi.MoveFinger(PORT_1,MIDDLE_FINGER,afPos3,afVel,0.05);
		m_HandApi.MoveFinger(PORT_1,RING_FINGER,afPos4,afVel,0.05);
		m_HandApi.MoveFinger(PORT_1,LITTLE_FINGER,afPos5,afVel,0.05);

		float torque1[3],torque2[3],torque3[3],torque4[3],torque5[3];

		m_HandApi.GetJointTorque(PORT_1,THUMB,torque1);
		m_HandApi.GetJointTorque(PORT_1,FIRST_FINGER,torque2);
		m_HandApi.GetJointTorque(PORT_1,MIDDLE_FINGER,torque3);
		m_HandApi.GetJointTorque(PORT_1,RING_FINGER,torque4);
		m_HandApi.GetJointTorque(PORT_1,LITTLE_FINGER,torque5);

		Force[0] = (torque1[0]+torque1[1])*1.5;
		Force[1] = (torque2[0]+torque2[1])*0.6;

		Force[2] = (torque3[0]+torque3[1])*0.6;
		Force[3] = (torque4[0]+torque4[1])*0.6;
		Force[4] = (torque5[0]+torque5[1])*1;

		for(int i=0; i<5; i++)
		{
			if(Force[i] >= 2.5)
				Force[i] = 2.5;
		}

		//			grasp->setForce(Force);

// 		CString str;
// 		str.Format("F1 %.2f F2 %.2f F3 %.2f F4 %.2f F5 %.2f",
// 			Force[0],Force[1],Force[2],Force[3],Force[4]);
// 
// 		SetDlgItemText(IDC_EDIT2,str);
	}
}

void RobonautControl::setPosMode()
{
	m_HandApi.SetController(PORT_1,CONTROLLER_POSITION);
}

void RobonautControl::setImpMode()
{
	m_HandApi.SetController(PORT_1,CONTROLLER_IMPEDANCE);
}

void RobonautControl::setResMode()
{
	m_HandApi.SetController(PORT_1,CONTROLLER_RESET);
}
