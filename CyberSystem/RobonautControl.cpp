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

	m_nRCount = 0;

	for (int i = 0; i<ORDER_BUF_LEN; i++)
	{
		cSendbufferp[i] = '\0';
		cRevbufferp[i] = '\0';
		cSendRobotCommandBuffer[i] = '\0';
	}

	m_Impedance_Index = 0;
	m_HandInit = 0;
	m_HandEnable = 0;
	m_HandStop = 0;
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
	RoboDataCnv(g_nRunFlag, g_RobotCmdDeg, cSendRobotCommandBuffer);
	
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
}

// TODO(CJH): change function input name
void RobonautControl::RoboDataCnv(const int &g_nRunFlag, const CRobonautData &g_RobotCmdDeg, char cSendRobotCommandBuffer[])
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

//*********************** Hand Control ***********************//
bool RobonautControl::ConnHand()
{
	char *Hand_ip = HAND_SERVER_IP;
	UINT HandCommand_Port = HAND_COMMAND_PORT; 
	UINT HandSensor_Port = HAND_SENSE_PORT;

	int send_ret = m_SendClientHand.ConnectServer((LPSTR)(LPCTSTR)Hand_ip,HandCommand_Port);
	int recv_ret = m_ReceiveClientHand.ConnectServer(Hand_ip,HandSensor_Port);

	return (!send_ret && !recv_ret);
}

bool RobonautControl::DisConnHand()
{
	return false;
}

bool RobonautControl::SendHandMsg(const CHandData &RHandSensor, const CHandData &LHandSensor, int HCount)
{
	float Vel = 80.0;

	int RobotRunFlag = 0x9888c1;

	sprintf(m_SendHandCommandBuffer, "PackageID:  %d\r\nPackageHead: %x %d\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nint   %x, %d, %x, %d, %x, 0, 0, 0, 0\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nfloat %f, %f, %f, %f, %f, %f\r\nint   %x, %d, %x, %d, %x, %d, %d, %d, %d\r\nPackageEnd: %x\r\n",
		HCount,
		PackageHead,0,
		RHandSensor.joint[0][0], RHandSensor.joint[0][1], RHandSensor.joint[0][2], Vel, Vel, Vel,
		RHandSensor.joint[1][0], RHandSensor.joint[1][1], RHandSensor.joint[1][2], Vel, Vel, Vel,
		RHandSensor.joint[2][0], RHandSensor.joint[2][1], RHandSensor.joint[2][2], Vel, Vel, Vel,
		RHandSensor.joint[3][0], RHandSensor.joint[3][1], RHandSensor.joint[3][2], Vel, Vel, Vel,
		RHandSensor.joint[4][0], RHandSensor.joint[4][1], RHandSensor.joint[4][2], Vel, Vel, Vel,
		CONTROLLER_IMPEDANCE,5,THUMB_BRAKE,90,RobotRunFlag,
		LHandSensor.joint[0][0], LHandSensor.joint[0][1], LHandSensor.joint[0][2], Vel, Vel, Vel,
		LHandSensor.joint[1][0], LHandSensor.joint[1][1], LHandSensor.joint[1][2], Vel, Vel, Vel,
		LHandSensor.joint[2][0], LHandSensor.joint[2][1], LHandSensor.joint[2][2], Vel, Vel, Vel,
		LHandSensor.joint[3][0], LHandSensor.joint[3][1], LHandSensor.joint[3][2], Vel, Vel, Vel,
		LHandSensor.joint[4][0], LHandSensor.joint[4][1], LHandSensor.joint[4][2], Vel, Vel, Vel,
		CONTROLLER_IMPEDANCE,5,THUMB_BRAKE,90,RobotRunFlag,m_HandInit,m_HandEnable,m_HandStop,m_Impedance_Index,
		PackageEnd);

	int ret = m_SendClientHand.SendData2(m_SendHandCommandBuffer);

	m_HandInit = 0;
	m_HandEnable = 0;
	m_HandStop = 0;

	return !ret;
}

bool RobonautControl::RecvHandMsg(CHandData &RHandSensor, CHandData &LHandSensor)
{
	char buffer[2048]={0};

	int RealRobotRunFlag=0;
        
	int ret = m_ReceiveClientHand.ReceiveData2(buffer);
	int RightFlag,LeftFlag;
				
	if(ret==0)
	{					
		sscanf(buffer,"Package-ID: %d\r\nF1:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF2:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF3:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF4:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF5:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF1:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF2:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF3:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF4:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\nF5:[Angle] %f, %f, %f\r\n  [Torque] %f, %f, %f\r\n   [State] %x, %d, %d\r\n",

			&(RHandSensor.count),

			&(RHandSensor.joint[0][0]),&(RHandSensor.joint[0][1]),&(RHandSensor.joint[0][2]),
			&(RHandSensor.torque[0][0]),&(RHandSensor.torque[0][1]),&(RHandSensor.torque[0][2]),
			&(RHandSensor.joint[1][0]),&(RHandSensor.joint[1][1]),&(RHandSensor.joint[1][2]),
			&(RHandSensor.torque[1][0]),&(RHandSensor.torque[1][1]),&(RHandSensor.torque[1][2]),
			&(RHandSensor.joint[2][0]),&(RHandSensor.joint[2][1]),&(RHandSensor.joint[2][2]),
			&(RHandSensor.torque[2][0]),&(RHandSensor.torque[2][1]),&(RHandSensor.torque[2][2]),
			&(RHandSensor.joint[3][0]),&(RHandSensor.joint[3][1]),&(RHandSensor.joint[3][2]),
			&(RHandSensor.torque[3][0]),&(RHandSensor.torque[3][1]),&(RHandSensor.torque[3][2]),
			&(RHandSensor.joint[4][0]),&(RHandSensor.joint[4][1]),&(RHandSensor.joint[4][2]),
			&(RHandSensor.torque[4][0]),&(RHandSensor.torque[4][1]),&(RHandSensor.torque[4][2]),

			&(LHandSensor.joint[0][0]),&(LHandSensor.joint[0][1]),&(LHandSensor.joint[0][2]),
			&(LHandSensor.torque[0][0]),&(LHandSensor.torque[0][1]),&(LHandSensor.torque[0][2]),
			&(LHandSensor.joint[1][0]),&(LHandSensor.joint[1][1]),&(LHandSensor.joint[1][2]),
			&(LHandSensor.torque[1][0]),&(LHandSensor.torque[1][1]),&(LHandSensor.torque[1][2]),
			&(LHandSensor.joint[2][0]),&(LHandSensor.joint[2][1]),&(LHandSensor.joint[2][2]),
			&(LHandSensor.torque[2][0]),&(LHandSensor.torque[2][1]),&(LHandSensor.torque[2][2]),
			&(LHandSensor.joint[3][0]),&(LHandSensor.joint[3][1]),&(LHandSensor.joint[3][2]),
			&(LHandSensor.torque[3][0]),&(LHandSensor.torque[3][1]),&(LHandSensor.torque[3][2]),
			&(LHandSensor.joint[4][0]),&(LHandSensor.joint[4][1]),&(LHandSensor.joint[4][2]),
			&(LHandSensor.torque[4][0]),&(LHandSensor.torque[4][1]),&(LHandSensor.torque[4][2]),

			&RealRobotRunFlag,&RightFlag,&LeftFlag);

		return true;
	}

	else{
		return false;
	}
}

void RobonautControl::setHandInit(bool ctrl_flag)
{
	if (ctrl_flag == true)
	{
		m_HandInit = 1;
	} 
	else
	{
		m_HandInit = 0;
	}
}

void RobonautControl::setHandEnable(bool ctrl_flag)
{
	if (ctrl_flag == true)
	{
		m_HandEnable = 1;
	} 
	else
	{
		m_HandEnable = 0;
	}
}

void RobonautControl::setHandEmergency(bool ctrl_flag)
{
	if (ctrl_flag == true)
	{
		m_HandStop = 1;
	} 
	else
	{
		m_HandStop = 2;
	}
}

void RobonautControl::setHandMode(HandMode handmode)
{
	switch(handmode)
	{
	case Position:
		m_HandMode = CONTROLLER_POSITION;
		break;
	case Impedance:
		m_HandMode = CONTROLLER_IMPEDANCE;
		m_Impedance_Index = 0;
		break;
	case Soft:
		m_HandMode = CONTROLLER_IMPEDANCE;
		m_Impedance_Index = 1;
		break;
	case  ZeroForce:
		m_HandMode = CONTROLLER_IMPEDANCE;
		m_Impedance_Index = 2;
		break;
	case  Reset:
		m_HandMode = CONTROLLER_RESET;
		break;
	}
}