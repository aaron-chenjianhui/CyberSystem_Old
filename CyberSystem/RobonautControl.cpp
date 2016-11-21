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




// TODO(CJH): Add to cybersystem.cpp
UINT HTimerId=-1;
void PASCAL TimerProcHandCtrl(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

RobonautControl *g_pRobonautCtrl;

// Collision flag
int g_nCollisionFlag=0;

//爆炸标志位333关，222开
int g_nRunFlag=222; 


RobonautControl::RobonautControl()
{
	g_pRobonautCtrl = this;

	// Initialize data
	m_bConsimuSockConn = false;
	m_nPCount = 0;

	m_hSocketSendCmd = NULL;
	m_nRCount = 0;

	for (int i = 0; i<ORDER_BUF_LEN; i++)
	{
		cSendbufferp[i] = '\0';
		cRevbufferp[i] = '\0';
		cSendRobotCommandBuffer[i] = '\0';
	}

	m_bSaveData = FALSE;
	m_bSaveDataFinish = FALSE;

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

	m_ServerSocketRobo.CloseSock();
	m_ClientSocketRobo.CloseSock();
}



//*********************** Consimulation Options ***********************//
// When there is no Socket Conncted(m_bConsimuSockConn == false), then Connect
// If Connect Success, Return True
// When there existing Socket Connected, Return last Connection Status
bool RobonautControl::ConnConsimu()
{
	if (m_bConsimuSockConn == false)
	{
		UINT PREDICTIVE_Port = PREDICTIVE_PORT; 
		char *pPREDICTIVE_IP = PREDICTIVE_IP;

		int ret = m_ClientSocketPredictive.ConnectServer(pPREDICTIVE_IP,PREDICTIVE_Port);
		if (ret == 0)
		{
			QMessageBox::about(NULL, "About", "Communication is connected");
			m_bConsimuSockConn = true;
		}
	}
	return m_bConsimuSockConn;
}



// TODO(CJH): Add Disconnect Function
bool RobonautControl::DisConnConsimu()
{
	m_bConsimuSockConn = false;
	return true;
}



// TODO(CJH): Do not Use Global Variable
// Send Consimulation Msg, when Consimulation Socket is Connected
bool RobonautControl::SendConsimuMsg()
{
	if (m_bConsimuSockConn == true)
	{
		for (int i=0;i<7;i++)
		{
			g_RobotCmdDeg.rightArmJoint[i] = g_rightArmJointBuf[i];
			g_RobotCmdDeg.leftArmJoint[i] = g_leftArmJointBuf[i];
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
		
		// Send and Recv Success
		if (retSendData == 0 && retReceiveData == 0)
		{
			return true;
		}
		else{
			return false;
		}
	}

	// Socket is not Connected
	else{
		return false;
	}
}



//*********************** Robonaut control Options ***********************//
bool RobonautControl::ConnRobo()
{
	bool ret_server = m_ServerSocketRobo.Init(ROB_TELE_IP, ROB_TELE_PORT);
	bool ret_client = m_ClientSocketRobo.Init(ROB_CONTRL_IP, ROB_CONTRL_PORT);
	return (ret_server && ret_client);
}


// TODO(CJH): How to disconnect a socket
bool RobonautControl::DisConnRobo()
{
	bool ret_server = m_ServerSocketRobo.CloseSock();
	bool ret_client = m_ClientSocketRobo.CloseSock();
	return (ret_server && ret_client);
}


// TODO(CJH): Do not use global variable
bool RobonautControl::RecvRoboMsg()
{
	bool ret_flag = m_ServerSocketRobo.Recv(cRevbufferp,sizeof(cRevbufferp));

	if (ret_flag == true)
	{
		g_RobotSensorDeg.count = MAKEWORD(cRevbufferp[0],cRevbufferp[1]);
		g_RobotSensorDeg.CtlMode = MAKEWORD(cRevbufferp[2],cRevbufferp[3]);

		//左臂
		g_RobotSensorDeg.leftArmJoint[0] = MAKELONG(MAKEWORD(cRevbufferp[4],cRevbufferp[5]),MAKEWORD(cRevbufferp[6],cRevbufferp[7]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[1] = MAKELONG(MAKEWORD(cRevbufferp[8],cRevbufferp[9]),MAKEWORD(cRevbufferp[10],cRevbufferp[11]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[2] = MAKELONG(MAKEWORD(cRevbufferp[12],cRevbufferp[13]),MAKEWORD(cRevbufferp[14],cRevbufferp[15]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[3] = MAKELONG(MAKEWORD(cRevbufferp[16],cRevbufferp[17]),MAKEWORD(cRevbufferp[18],cRevbufferp[19]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[4] = MAKELONG(MAKEWORD(cRevbufferp[20],cRevbufferp[21]),MAKEWORD(cRevbufferp[22],cRevbufferp[23]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[5] = MAKELONG(MAKEWORD(cRevbufferp[24],cRevbufferp[25]),MAKEWORD(cRevbufferp[26],cRevbufferp[27]))*0.0001;
		g_RobotSensorDeg.leftArmJoint[6] = MAKELONG(MAKEWORD(cRevbufferp[28],cRevbufferp[29]),MAKEWORD(cRevbufferp[30],cRevbufferp[31]))*0.0001;		

		//右臂
		g_RobotSensorDeg.rightArmJoint[0] = MAKELONG(MAKEWORD(cRevbufferp[32],cRevbufferp[33]),MAKEWORD(cRevbufferp[34],cRevbufferp[35]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[1] = MAKELONG(MAKEWORD(cRevbufferp[36],cRevbufferp[37]),MAKEWORD(cRevbufferp[38],cRevbufferp[39]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[2] = MAKELONG(MAKEWORD(cRevbufferp[40],cRevbufferp[41]),MAKEWORD(cRevbufferp[42],cRevbufferp[43]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[3] = MAKELONG(MAKEWORD(cRevbufferp[44],cRevbufferp[45]),MAKEWORD(cRevbufferp[46],cRevbufferp[47]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[4] = MAKELONG(MAKEWORD(cRevbufferp[48],cRevbufferp[49]),MAKEWORD(cRevbufferp[50],cRevbufferp[51]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[5] = MAKELONG(MAKEWORD(cRevbufferp[52],cRevbufferp[53]),MAKEWORD(cRevbufferp[54],cRevbufferp[55]))*0.0001;
		g_RobotSensorDeg.rightArmJoint[6] = MAKELONG(MAKEWORD(cRevbufferp[56],cRevbufferp[57]),MAKEWORD(cRevbufferp[58],cRevbufferp[59]))*0.0001;	

		//头部
		g_RobotSensorDeg.headJoint[0] = MAKELONG(MAKEWORD(cRevbufferp[60],cRevbufferp[61]),MAKEWORD(cRevbufferp[62],cRevbufferp[63]))*0.0001;
		g_RobotSensorDeg.headJoint[1] = MAKELONG(MAKEWORD(cRevbufferp[64],cRevbufferp[65]),MAKEWORD(cRevbufferp[66],cRevbufferp[67]))*0.0001;
		g_RobotSensorDeg.headJoint[2] = MAKELONG(MAKEWORD(cRevbufferp[68],cRevbufferp[69]),MAKEWORD(cRevbufferp[70],cRevbufferp[71]))*0.0001;

		//腰部
		g_RobotSensorDeg.waistJoint[0] = MAKELONG(MAKEWORD(cRevbufferp[72],cRevbufferp[73]),MAKEWORD(cRevbufferp[74],cRevbufferp[75]))*0.0001;
		g_RobotSensorDeg.waistJoint[1] = MAKELONG(MAKEWORD(cRevbufferp[76],cRevbufferp[77]),MAKEWORD(cRevbufferp[78],cRevbufferp[79]))*0.0001;

		//右臂关节力
		g_RobotSensorDeg.RightJointFT[0] = MAKELONG(MAKEWORD(cRevbufferp[108],cRevbufferp[109]),MAKEWORD(cRevbufferp[110],cRevbufferp[111]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[1] = MAKELONG(MAKEWORD(cRevbufferp[112],cRevbufferp[113]),MAKEWORD(cRevbufferp[114],cRevbufferp[115]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[2] = MAKELONG(MAKEWORD(cRevbufferp[116],cRevbufferp[117]),MAKEWORD(cRevbufferp[118],cRevbufferp[119]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[3] = MAKELONG(MAKEWORD(cRevbufferp[120],cRevbufferp[121]),MAKEWORD(cRevbufferp[122],cRevbufferp[123]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[4] = MAKELONG(MAKEWORD(cRevbufferp[124],cRevbufferp[125]),MAKEWORD(cRevbufferp[126],cRevbufferp[127]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[5] = MAKELONG(MAKEWORD(cRevbufferp[128],cRevbufferp[129]),MAKEWORD(cRevbufferp[130],cRevbufferp[131]))*0.0001;;
		g_RobotSensorDeg.RightJointFT[6] = MAKELONG(MAKEWORD(cRevbufferp[132],cRevbufferp[133]),MAKEWORD(cRevbufferp[134],cRevbufferp[135]))*0.0001;;

		//左臂关节力
		g_RobotSensorDeg.LeftJointFT[0] = MAKELONG(MAKEWORD(cRevbufferp[80],cRevbufferp[81]),MAKEWORD(cRevbufferp[82],cRevbufferp[83]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[1] = MAKELONG(MAKEWORD(cRevbufferp[84],cRevbufferp[85]),MAKEWORD(cRevbufferp[86],cRevbufferp[87]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[2] = MAKELONG(MAKEWORD(cRevbufferp[88],cRevbufferp[89]),MAKEWORD(cRevbufferp[90],cRevbufferp[91]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[3] = MAKELONG(MAKEWORD(cRevbufferp[92],cRevbufferp[93]),MAKEWORD(cRevbufferp[94],cRevbufferp[95]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[4] = MAKELONG(MAKEWORD(cRevbufferp[96],cRevbufferp[97]),MAKEWORD(cRevbufferp[98],cRevbufferp[99]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[5] = MAKELONG(MAKEWORD(cRevbufferp[100],cRevbufferp[101]),MAKEWORD(cRevbufferp[102],cRevbufferp[103]))*0.0001;
		g_RobotSensorDeg.LeftJointFT[6] = MAKELONG(MAKEWORD(cRevbufferp[104],cRevbufferp[105]),MAKEWORD(cRevbufferp[106],cRevbufferp[107]))*0.0001;

		//力判断标志
		m_DecideFlag = MAKELONG(MAKEWORD(cRevbufferp[136],cRevbufferp[137]),MAKEWORD(cRevbufferp[138],cRevbufferp[139]));
	}
	return ret_flag;
}

// TODO(CJH): change
// Now: get global data, and send to robonaut
bool RobonautControl::SendRoboMsg()
{
	// Communication Count
	ADDCOUNT(m_nRCount, 99999);

	for (int i=0;i<7;i++)
	{	
		g_RobotCmdDeg.rightArmJoint[i] = g_rightArmJointBuf[i];
		g_RobotCmdDeg.leftArmJoint[i] = g_leftArmJointBuf[i];
	}
	
	// 限位
	SETRANGE(g_RobotCmdDeg.leftArmJoint[0], -49, 170);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[1], -100, 70);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[2], -120, 120);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[3], -129, 0);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[4], -120, 120);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[5], -90, 90);
	SETRANGE(g_RobotCmdDeg.leftArmJoint[6], -120, 120);
	ADDCOUNT(g_RobotCmdDeg.count, 1000);

	// 转换数据为可发送的buffer		
	DataConvert(g_nRunFlag, g_RobotCmdDeg, cSendRobotCommandBuffer);
	
	bool ret_flag = m_ClientSocketRobo.Send(cSendRobotCommandBuffer, sizeof(cSendRobotCommandBuffer)); 

	// TODO(CJH): change ip and port
	if(ret_flag == true)
	{	
		return true;
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
		g_RobotSensorDeg.RightJointFT[6]
	);

	// 保存数据
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

// TODO(CJH): change function input name
void RobonautControl::DataConvert(const int &g_nRunFlag, const CRobonautData &g_RobotCmdDeg, char cSendRobotCommandBuffer[])
{
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
}

// TODO(CJH): Parse Buffer from command buffer
void RobonautControl::BuffParse()
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
