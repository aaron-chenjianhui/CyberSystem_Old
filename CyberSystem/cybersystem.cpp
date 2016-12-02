#include "cybersystem.h"
#include "qregexp.h"
#include <iostream>
#include <sstream>

#include "kine7.hpp"

#define DEG2ANG(x) (x*180/3.14)
#define	ANG2DEG(x) (x*3.14/180)


// 初始关节角
// #define INIT_JOINT1 -39.1323
// #define INIT_JOINT2 -11.2267
// #define INIT_JOINT3 96.994
// #define INIT_JOINT4 -102.1614
// #define INIT_JOINT5 -82.8184
// #define INIT_JOINT6 41.125
// #define INIT_JOINT7 -27.5551


#define INIT_JOINT1 33.095
#define INIT_JOINT2 -24.239
#define INIT_JOINT3 35.020
#define INIT_JOINT4 -92.413
#define INIT_JOINT5 14.749
#define INIT_JOINT6 -28.523
#define INIT_JOINT7 -29.902

// 初始位置
#define POS11 -0.176
#define POS12 -0.984
#define POS13 0.01
#define POS14 -219.62
#define POS21 -0.7
#define POS22 0.119
#define POS23 -0.7
#define POS24 -550
#define POS31 0.688
#define POS32 -0.13
#define POS33 -0.71
#define POS34 44.192
#define POS41 0
#define POS42 0
#define POS43 0
#define POS44 1



// Timer for Robonaut Control
UINT SendTimerId = NULL;
void PASCAL TimerProcSendCmd(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

UINT RecvTimerId = NULL;
void PASCAL TimerProcRecvSensor(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

UINT HSendTimerId = NULL;
void PASCAL TimerProcHandSend(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2);

UINT HRecvTimerId = NULL;
void PASCAL TimerProcHandRecv(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);
// Define to Use Multimedia Timer
CyberSystem *g_pCyberSys;


extern CRobonautData g_RobotCmdDeg;     //全局机器人控制指令单位角度
extern CRobonautData g_SRobotCmdDeg;    //发送的安全机器人控制指令单位角度
extern CRobonautData g_RobotSensorDeg;  //全局机器人传感器数据单位角度

float g_rightArmJointBuf[7];	// global data from slider for joint control
float g_leftArmJointBuf[7];		// global data from slider for joint control


//typedef rpp::kine::Kine7<double> kine_type;
rpp::kine::Kine7<double> m_Kine;
rpp::kine::SingularityHandler<double> sh;
rpp::kine::Kine7<double>::angular_interval_vector joint_limits;

CyberSystem::CyberSystem(QWidget *parent)
	: QMainWindow(parent), m_DisThread(this)/*, m_RobonautCtrlThread(this)*/
{
	ui.setupUi(this);

	g_pCyberSys = this;

	m_fRRealMat.open("./data/RightRealMat.txt");
	m_fRRealMat << "Count Start:";
	RRealCount = 0;
	RRawCount = 0;

	// set up main widget size
//	ui.centralWidget->setFixedSize(1920,1080);

	// check out CyberTracker data input
	// You must input point to satisfy the check out
	QRegExp TraRegExp("[-]?[0-9][0-9]{0,2}[.][0-9][0-9]{0,2}|[-]?[0-9][0-9]{0,2}");
	QRegExp CliRegExp("[-]?[0-9][0-9]{0,2}[.][0-9][0-9]{0,2}|[-]?[0-9][0-9]{0,2}");
	ui.m_pRPosXLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pRPosYLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pRPosZLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pROriXLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pROriYLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pROriZLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLPosXLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLPosYLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLPosZLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLOriXLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLOriYLiEd->setValidator(new QRegExpValidator(TraRegExp, this));
	ui.m_pLOriZLiEd->setValidator(new QRegExpValidator(TraRegExp, this));

	ui.m_pRJo1Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo2Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo3Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo4Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo5Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo6Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pRJo7Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo1Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo2Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo3Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo4Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo5Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo6Line->setValidator(new QRegExpValidator(CliRegExp, this));
	ui.m_pLJo7Line->setValidator(new QRegExpValidator(CliRegExp, this));

	//*********************** Initialize all data ***********************//
	// initialize cyber workstation connection status
	m_RGloContr = false;
	m_LGloContr = false;
	m_RTraContr = false;
	m_LTraContr = false;
	
	// CyberGlove data display logical control
	// whether to display glove data
	m_bDisGloData = false;
	// choose which glove data to display
	m_bRGloCaliFin = false;
	m_bLGloCaliFin = false;

	// CyberTracker data display logical control
	// whether to display tracker data
	m_bDisTraData = false;	
	// determine whether side is over
	m_bRTraCaliFini = false;
	m_bLTraCaliFini = false;
	// Connection Logical Control
	m_bRoboConn = false;
	m_bConsimuConn = false;
	m_bHandConn = false;
	m_bHandInit = false;
	m_bHandEnable = false;
	m_bHandStop = false;
	// Control Mode
	m_CtrlMode = OUT_CTRL;
	m_HandCtrlMode = HAND_OUT_CTRL;

	// tab display 
	ui.m_pRTraTab->setEnabled(true);
	ui.m_pRArmTab->setEnabled(true);

	// Initialize slider data
	for (int i = 0; i<7; i++)
	{
		g_rightArmJointBuf[i] = 0;
		g_leftArmJointBuf[i] = 0;
	}

	// Initialize position cmd data
	for (int i = 0; i<6; i++)
	{
		m_rightArmPos[i] = 0;
		m_leftArmPos[i] = 0;
	}

	for(int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m_RGloRealData[i][j] = 0;
			m_LGloRealData[i][j] = 0;
		}
	}

	m_last_arm_angle = 0;
	m_last_joint_angle << ANG2DEG( INIT_JOINT1 ), ANG2DEG( INIT_JOINT2 ), ANG2DEG( INIT_JOINT3 ), 
						  ANG2DEG( INIT_JOINT4 ), ANG2DEG( INIT_JOINT5 ), ANG2DEG( INIT_JOINT6 ), 
						  ANG2DEG( INIT_JOINT7 );
// 	m_RTraRealMat << 1, 0, 0, -154,
// 					  0, 0, -1, -490,
// 					  0, 1, 0, 360,
// 					  0, 0, 0, 1;

	m_RTraRealMat << POS11, POS12, POS13, POS14,
					POS21, POS22, POS23, POS24,
					POS31, POS32, POS33, POS34,
					POS41, POS42, POS43, POS44;

	// Initialize kinetic calculate
	InitKine();

	// Robonaut joint move management
	ui.m_pRJo0Sli->setRange(ROBO_J0_MIN_SLI,ROBO_J0_MAX_SLI);
	ui.m_pRJo1Sli->setRange(ROBO_J1_MIN_SLI,ROBO_J1_MAX_SLI);
	ui.m_pRJo2Sli->setRange(ROBO_J2_MIN_SLI,ROBO_J2_MAX_SLI);
	ui.m_pRJo3Sli->setRange(ROBO_J3_MIN_SLI,ROBO_J3_MAX_SLI);
	ui.m_pRJo4Sli->setRange(ROBO_J4_MIN_SLI,ROBO_J4_MAX_SLI);
	ui.m_pRJo5Sli->setRange(ROBO_J5_MIN_SLI,ROBO_J5_MAX_SLI);
	ui.m_pRJo6Sli->setRange(ROBO_J6_MIN_SLI,ROBO_J6_MAX_SLI);

	ui.m_pLJo0Sli->setRange(ROBO_J0_MIN_SLI,ROBO_J0_MAX_SLI);
	ui.m_pLJo1Sli->setRange(ROBO_J1_MIN_SLI,ROBO_J1_MAX_SLI);
	ui.m_pLJo2Sli->setRange(ROBO_J2_MIN_SLI,ROBO_J2_MAX_SLI);
	ui.m_pLJo3Sli->setRange(ROBO_J3_MIN_SLI,ROBO_J3_MAX_SLI);
	ui.m_pLJo4Sli->setRange(ROBO_J4_MIN_SLI,ROBO_J4_MAX_SLI);
	ui.m_pLJo5Sli->setRange(ROBO_J5_MIN_SLI,ROBO_J5_MAX_SLI);
	ui.m_pLJo6Sli->setRange(ROBO_J6_MIN_SLI,ROBO_J6_MAX_SLI);

	ui.m_pRJo1Line->setText("0");
	ui.m_pRJo2Line->setText("0");
	ui.m_pRJo3Line->setText("0");
	ui.m_pRJo4Line->setText("0");
	ui.m_pRJo5Line->setText("0");
	ui.m_pRJo6Line->setText("0");
	ui.m_pRJo7Line->setText("0");

	ui.m_pLJo1Line->setText("0");
	ui.m_pLJo2Line->setText("0");
	ui.m_pLJo3Line->setText("0");
	ui.m_pLJo4Line->setText("0");
	ui.m_pLJo5Line->setText("0");
	ui.m_pLJo6Line->setText("0");
	ui.m_pLJo7Line->setText("0");




	//*********************** Initialize all data ***********************//


// 	// create timers
// 	m_pGloDispTimer = new QTimer(this);


	//*********************** Signals and Slots ***********************//	
	// signals and slots for connection
	connect(ui.m_pInitBtn, SIGNAL(clicked()), this, SLOT(InitSystem()));
	connect(ui.m_pInitRHBtn, SIGNAL(clicked()), this, SLOT(InitRHand()));
	connect(ui.m_pInitRTBtn, SIGNAL(clicked()), this, SLOT(InitRTracker()));
	connect(ui.m_pInitLHBtn, SIGNAL(clicked()), this, SLOT(InitLHand()));
	connect(ui.m_pInitLTBtn, SIGNAL(clicked()), this, SLOT(InitLTracker()));

	connect(ui.m_pOpenTraCali, SIGNAL(clicked()), this, SLOT(LoadTraCaliData()));
	connect(ui.m_pSaveTraCali, SIGNAL(clicked()), this, SLOT(SaveCaliData()));
	connect(ui.m_pOpenGloCali, SIGNAL(clicked()), this, SLOT(LoadGloCaliData()));
	connect(ui.m_pSaveGloCali, SIGNAL(clicked()), this, SLOT(SaveGloCaliData()));

	// signals and slots for glove calibration
	connect(ui.m_pGlStartBtn, SIGNAL(clicked()), this, SLOT(InitGloveCali()));
	connect(this, SIGNAL(InsertTraText(const QString &)), ui.m_pTraDataDisBs, SLOT(setText(const QString &))); // display glove data when calibration begin
	connect(this, SIGNAL(InsertGloText(const QString &)), ui.m_pGloDataDisBs, SLOT(setText(const QString &)));
	connect(this, SIGNAL(InsertRoboText(const QString &)), ui.m_pRoboDataDisBs, SLOT(setText(const QString &)));
	connect(this, SIGNAL(InsertCmdStr(const QString &)), ui.m_pCommadBs, SLOT(setText(const QString &)));
	connect(ui.m_pCommadBs, SIGNAL(textChanged()), this, SLOT(BrowserMoveEnd()));


	connect(ui.m_pGloFinshBtn, SIGNAL(clicked()), this, SLOT(FinGloveCali()));
	connect(ui.m_pGesBtn_one, SIGNAL(clicked()), this, SLOT(GesOneData()));
	connect(ui.m_pGesBtn_two, SIGNAL(clicked()), this, SLOT(GesTwoData()));
	connect(ui.m_pGesBtn_three, SIGNAL(clicked()), this, SLOT(GesThrData()));
	connect(ui.m_pGesBtn_four, SIGNAL(clicked()), this, SLOT(GesFourData()));

	// signals and slots for tracker calibration
	connect(ui.m_pTraStartBtn, SIGNAL(clicked()), this, SLOT(InitTraCali()));
	connect(ui.m_pTraCalBtn, SIGNAL(clicked()), this, SLOT(CalTraData()));
	// you can push Calculate button when all lineEdit text is written correctly
	connect(ui.m_pRPosXLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pRPosYLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pRPosZLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pROriXLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pROriYLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pROriZLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLPosXLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLPosYLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLPosZLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLOriXLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLOriYLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));
	connect(ui.m_pLOriZLiEd, SIGNAL(textChanged(const QString &)), this, SLOT(lineEdit_textChanged()));

	// signals and slots for  consimulation
	connect(ui.m_pSimuConnBtn, SIGNAL(clicked()), this, SLOT(ConsimuConnCtrl()));

	// signals and slots for robonaut control
	connect(ui.m_pRoboConnBtn, SIGNAL(clicked()), this, SLOT(RoboConnCtrl()));
	connect(ui.m_pCyberCtrlBtn, SIGNAL(clicked()), this, SLOT(CyberCtrl()));
	connect(ui.m_pCyberStartBtn, SIGNAL(clicked()), this, SLOT(CyberCmd()));
	connect(ui.m_pCyberPauseBtn, SIGNAL(clicked()), this, SLOT(CyberStop()));

	// signals and slots for robonaut joint control
	connect(ui.m_pJointCtrlBtn, SIGNAL(clicked()), this, SLOT(ClickCtrl()));

	// signals and slots for sending joint control data
	connect(ui.m_pSdDataBtn, SIGNAL(clicked()), this, SLOT(getSliData()));
	connect(ui.m_pUpdataBtn, SIGNAL(clicked()), this, SLOT(SliUpdata()));

	// signals and slots for slider and spin box
	connect(ui.m_pRJo0Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo1Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo2Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo3Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo4Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo5Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pRJo6Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));

	connect(ui.m_pLJo0Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo1Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo2Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo3Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo4Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo5Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	connect(ui.m_pLJo6Sli, SIGNAL(valueChanged(int)), this, SLOT(SlitoLine()));
	
	connect(ui.m_pRJo1Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo2Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo3Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo4Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo5Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo6Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pRJo7Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));

	connect(ui.m_pLJo1Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo2Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo3Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo4Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo5Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo6Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));
	connect(ui.m_pLJo7Line, SIGNAL(textChanged(const QString &)), this, SLOT(LinetoSli()));


	// signals and slots for hand control
	connect(ui.m_pHandConnBtn, SIGNAL(clicked()), this, SLOT(InitHand()));
	connect(ui.m_pHandEnableBtn, SIGNAL(clicked()), this, SLOT(EnableHand()));
	connect(ui.m_pHandRunBtn, SIGNAL(clicked()), this, SLOT(StartHand()));
	connect(ui.m_pEmgBtn, SIGNAL(clicked()), this, SLOT(EmergHand()));

	
	//*********************** Signals and Slots ***********************//


}

CyberSystem::~CyberSystem()
{ 
	timeKillEvent(SendTimerId);
	SendTimerId = NULL;
	timeKillEvent(RecvTimerId);
	RecvTimerId = NULL;


	m_DisThread.stop();
	m_DisThread.wait();	

	m_fRRealMat.clear();
	m_fRRealMat.close();
}


//*********************** Initialization options ***********************//
void CyberSystem::InitSystem()
{
	std::string r_glo_err_str, l_glo_err_str;
	std::string r_tra_err_str, l_tra_err_str;

	m_CmdStr += "Connecting System......";
	emit InsertCmdStr(m_CmdStr);

	m_RGloContr = m_CyberStation.RHandConn(r_glo_err_str);
	m_LGloContr = m_CyberStation.LHandConn(l_glo_err_str);
	m_RTraContr = m_CyberStation.RTraConn(r_tra_err_str);
	m_LTraContr = m_CyberStation.LTraConn(l_tra_err_str);

	if ((m_RTraContr && m_LTraContr && m_RGloContr && m_LGloContr) == true)
	{
		m_CmdStr += "OK!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		ui.m_pGlStartBtn->setEnabled(true);
		ui.m_pTraStartBtn->setEnabled(true);
	} 
	else
	{
		m_CmdStr += "Failed!!!\r\n";
		if (m_RTraContr == false)
		{
			m_CmdStr += m_CmdStr.fromStdString(r_tra_err_str);
			emit InsertCmdStr(m_CmdStr);
		}
		if (m_LTraContr == false)
		{
			m_CmdStr += m_CmdStr.fromStdString(l_tra_err_str);
			emit InsertCmdStr(m_CmdStr);
		}
		if (m_RGloContr == false)
		{
			m_CmdStr += m_CmdStr.fromStdString(r_glo_err_str);
			emit InsertCmdStr(m_CmdStr);
		}
		if (m_LGloContr == false)
		{
			m_CmdStr += m_CmdStr.fromStdString(l_glo_err_str);
			emit InsertCmdStr(m_CmdStr);
		}
	}
}

void CyberSystem::InitRHand()
{
	std::string err_str;

	m_CmdStr += "Connecting Right Hand......";
	emit InsertCmdStr(m_CmdStr);

	m_RGloContr = m_CyberStation.RHandConn(err_str);

	if (m_RGloContr == true)
	{
		m_CmdStr += "OK!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		ui.m_pGlStartBtn->setEnabled(true);
	} 
	else
	{
		m_CmdStr += "Failed!!!\r\n";
		m_CmdStr += m_CmdStr.fromStdString(err_str);
		m_CmdStr += "!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::InitLHand()
{
	m_CmdStr += "Connecting Left Hand......";
	emit InsertCmdStr(m_CmdStr);

	std::string err_str;
	m_LGloContr = m_CyberStation.LHandConn(err_str);

	if (m_LGloContr == true)
	{
		m_CmdStr += "OK!!!\r\n";
		emit InsertCmdStr(m_CmdStr);

		ui.m_pGlStartBtn->setEnabled(true);
	} 
	else
	{
		m_CmdStr += "Failed!!!\r\n";
		m_CmdStr += m_CmdStr.fromStdString(err_str);
		emit InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::InitRTracker()
{
	std::string err_str;

	m_CmdStr += "Connecting Right Tracker......";
	emit InsertCmdStr(m_CmdStr);

	m_RTraContr = m_CyberStation.RTraConn(err_str);

	if (m_RTraContr == true)
	{
		m_CmdStr += "OK!\r\n";
		emit InsertCmdStr(m_CmdStr);

		ui.m_pTraStartBtn->setEnabled(true);
	}
	else{
		m_CmdStr += "Failed!\r\n";
		m_CmdStr += m_CmdStr.fromStdString(err_str);
		emit InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::InitLTracker()
{
	std::string err_str;

	m_CmdStr += "Connecting Left Tracker......";
	emit InsertCmdStr(m_CmdStr);

	m_LTraContr = m_CyberStation.LTraConn(err_str);

	if (m_LTraContr == true)
	{
		m_CmdStr += "OK!\r\n";
		emit InsertCmdStr(m_CmdStr);

		ui.m_pTraStartBtn->setEnabled(true);
	} 
	else
	{
		m_CmdStr += "Failed!\r\n";
		m_CmdStr += m_CmdStr.fromStdString(err_str);
		emit InsertCmdStr(m_CmdStr);
	}
}

//*********************** Data Display Control ***********************//
void CyberSystem::DisCyberData()
{
	while(m_DisThread.m_bDisThreadStop == false)
	{
		DisGloData();
		DisTraData();
	
		m_DisThread.msleep(250);
	}
	// ensure that you will get into the thread next time
	m_DisThread.m_bDisThreadStop = false;
}

void CyberSystem::DisTraData()
{
	QString Str = NULL;
	QString RTraStr = NULL;
	QString LTraStr = NULL;
	// Push Tracker Start Button
	if (m_bDisTraData == true)
	{
		// Connected Right Tracker
		if (m_RTraContr == true)
		{
			// Right Tracker is Calibrated
			if (m_bRTraCaliFini == true)
			{
				m_RTraRealMat = m_CyberStation.GetRTraRealData();

				// display stream
				std::ostringstream r_tra_stream;
				r_tra_stream << "Real TransMat of Right Tracker is: " << std::endl;
				r_tra_stream << std::fixed << std::left;
				r_tra_stream.precision(3);
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						r_tra_stream.width(12);
						r_tra_stream << m_RTraRealMat(i, j);
					}
					r_tra_stream << std::endl;
				}

				std::string r_tra_str = r_tra_stream.str();
				RTraStr = RTraStr.fromStdString(r_tra_str);
// 					RTraStr = QString("rightTracker_RealTransMat:\r\n%1 %2 %3 %4\r\n%5 %6 %7 %8\r\n%9 %10 %11 %12\r\n%13 %14 %15 %16\r\n")
// 						.arg(m_RTraRealMat(0,0)).arg(m_RTraRealMat(0,1)).arg(m_RTraRealMat(0,2)).arg(m_RTraRealMat(0,3))
// 						.arg(m_RTraRealMat(1,0)).arg(m_RTraRealMat(1,1)).arg(m_RTraRealMat(1,2)).arg(m_RTraRealMat(1,3))
// 						.arg(m_RTraRealMat(2,0)).arg(m_RTraRealMat(2,1)).arg(m_RTraRealMat(2,2)).arg(m_RTraRealMat(2,3))
// 						.arg(m_RTraRealMat(3,0)).arg(m_RTraRealMat(3,1)).arg(m_RTraRealMat(3,2)).arg(m_RTraRealMat(3,3));

				// file stream
				RRealCount ++;
				m_fRRealMat << "Count " << RRealCount << std::endl;
				m_fRRealMat << r_tra_str << std::endl;
			}
			// Right Tracker is not Calibrated
			else{
				//RTraStr = QString("rightTracker_RawTransMat:\r\n%1 %2 %3 %4\r\n%5 %6 %7 %8\r\n%9 %10 %11 %12\r\n%13 %14 %15 %16\r\n")
				//	.arg(m_RTraRawMat(0,0)).arg(m_RTraRawMat(0,1)).arg(m_RTraRawMat(0,2)).arg(m_RTraRawMat(0,3))
				//	.arg(m_RTraRawMat(1,0)).arg(m_RTraRawMat(1,1)).arg(m_RTraRawMat(1,2)).arg(m_RTraRawMat(1,3))
				//	.arg(m_RTraRawMat(2,0)).arg(m_RTraRawMat(2,1)).arg(m_RTraRawMat(2,2)).arg(m_RTraRawMat(2,3))
				//	.arg(m_RTraRawMat(3,0)).arg(m_RTraRawMat(3,1)).arg(m_RTraRawMat(3,2)).arg(m_RTraRawMat(3,3));

				// Get Raw Transmatrix
				m_RTraRawMat = m_CyberStation.GetRRawTraData();
				// Data Display Stream
				std::ostringstream r_tra_stream;
				r_tra_stream << "Raw TransMat of Right Tracker is: " << std::endl;
				r_tra_stream << std::fixed << std::left;
				r_tra_stream.precision(3);
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						r_tra_stream.width(12);
						r_tra_stream << m_RTraRawMat(i, j);
					}
					r_tra_stream << std::endl;
				}

				std::string r_tra_str = r_tra_stream.str();
				RTraStr = RTraStr.fromStdString(r_tra_str);
			}
		}

		// Connected Left Tracker
		if (m_LTraContr == true)
		{
			if (m_bLTraCaliFini == true)
			{
			} 
			else
			{
			}
		}

		Str = RTraStr + LTraStr;
		emit InsertTraText(Str);
	}
}



void CyberSystem::DisGloData()
{
	QString Str = NULL;
	QString RGloStr = NULL;
	QString LGloStr = NULL;

	// Push Glove Start Button
	if (m_bDisGloData == true)
	{
		// Connected Right Glove
		if (m_RGloContr == true)
		{
			// Right Glove is Calibrated
			if (m_bRGloCaliFin == true)
			{
				m_CyberStation.GetRRealGloData(m_RGloRealData);

				// display stream
				std::ostringstream r_glo_stream;
				r_glo_stream << "Real Joints of Right Glove is: " << std::endl;
				r_glo_stream << std::fixed << std::left;
				r_glo_stream.precision(3);

				r_glo_stream.width(8);
				r_glo_stream << "Thumb";
				r_glo_stream.width(8);
				r_glo_stream << "Index";
				r_glo_stream.width(8);
				r_glo_stream << "Middle";
				r_glo_stream.width(8);
				r_glo_stream << "Ring";
				r_glo_stream.width(8);
				r_glo_stream << "Little" << std::endl;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 5; j++)
					{
						r_glo_stream.width(8);
						r_glo_stream << m_RGloRealData[j][i];
					}
					r_glo_stream << std::endl;
				}

				std::string r_glo_str = r_glo_stream.str();
				RGloStr = RGloStr.fromStdString(r_glo_str);
			}
			// Right Tracker is not Calibrated
			else{
				m_CyberStation.GetRRawGloData(m_RGloRawData);

				// display stream
				std::ostringstream r_glo_stream;
				r_glo_stream << "Raw Joints of Right Glove is: " << std::endl;
				r_glo_stream << std::fixed << std::left;
				r_glo_stream.precision(3);
				
				r_glo_stream.width(8);
				r_glo_stream << "Thumb";
				r_glo_stream.width(8);
				r_glo_stream << "Index";
				r_glo_stream.width(8);
				r_glo_stream << "Middle";
				r_glo_stream.width(8);
				r_glo_stream << "Ring";
				r_glo_stream.width(8);
				r_glo_stream << "Little" << std::endl;
//				for (int i = 0; i < 9; i++)
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 5; j++)
					{
						r_glo_stream.width(8);
						r_glo_stream << m_RGloRawData[j][i];
					}
					r_glo_stream << std::endl;
				}

				std::string r_glo_str = r_glo_stream.str();
				RGloStr = RGloStr.fromStdString(r_glo_str);
			}
		}

		// Connected Left Tracker
		if (m_LGloContr == true)
		{
			if (m_bLGloCaliFin == true)
			{
			} 
			else
			{
			}
		}
		Str = RGloStr + LGloStr;
		emit InsertGloText(Str);
	}
}

// Move Command Browser's Cursor to end
void CyberSystem::BrowserMoveEnd()
{
	ui.m_pCommadBs->moveCursor(QTextCursor::End);
}
//*********************** CyberGlove Calibration Options ***********************//
// start display thread
void CyberSystem::InitGloveCali()
{
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Glove?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
	{ 
		m_CmdStr += "Start Glove Display......OK!\r\n";
		emit InsertCmdStr(m_CmdStr);
		if (!(m_DisThread.isRunning()))
		{
			m_bDisGloData = true;
			m_DisThread.start();
		} 
		else
		{
			m_bDisGloData = true;
		}
		ui.m_pGesBtn_one->setEnabled(true);
		ui.m_pGesBtn_two->setEnabled(true);
		ui.m_pGesBtn_three->setEnabled(true);
		ui.m_pGesBtn_four->setEnabled(true);
		ui.m_pGloFinshBtn->setEnabled(true);
	}  
	else{}  
}


void CyberSystem::LoadGloCaliData()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Glove Calibration Data"), ".",
		tr("XML files (*.xml)"));

	if (!fileName.isEmpty())
	{
		std::string str_filename = fileName.toStdString();
		rapidxml::file<> fCalibration(str_filename.c_str());

		rapidxml::xml_document<> Calibration;
		Calibration.parse<0>(fCalibration.data());

		//获取根节点
		rapidxml::xml_node<>* GloveCalibration = Calibration.first_node();
		//获取相关矩阵
		rapidxml::xml_node<>* CorrelationMatrix = GloveCalibration->first_node("CorrelationMatrix");
		rapidxml::xml_attribute<>* CorrelationMatrix_DataType = CorrelationMatrix->first_attribute();
		//拇指
		rapidxml::xml_node<>* CorThumb = CorrelationMatrix->first_node("Thumb");
		rapidxml::xml_node<>* CorThumbPIP = CorThumb->first_node("PIP");
		m_RGloCaliK[0][0] = atof(CorThumbPIP->value());
		rapidxml::xml_node<>* CorThumbMP = CorThumb->first_node("MP");
		m_RGloCaliK[0][1] = atof(CorThumbMP->value());
		rapidxml::xml_node<>* CorThumbABP = CorThumb->first_node("ABP");
		m_RGloCaliK[0][2] = atof(CorThumbABP->value());
		//食指
		rapidxml::xml_node<>* CorIndex = CorrelationMatrix->first_node("Index");
		rapidxml::xml_node<>* CorIndexPIP = CorIndex->first_node("PIP");
		m_RGloCaliK[1][0] = atof(CorIndexPIP->value());
		rapidxml::xml_node<>* CorIndexMP = CorIndex->first_node("MP");
		m_RGloCaliK[1][1] = atof(CorIndexMP->value());
		rapidxml::xml_node<>* CorIndexABP = CorIndex->first_node("ABP");
		m_RGloCaliK[1][2] = atof(CorIndexABP->value());
		//中指
		rapidxml::xml_node<>* CorMiddle = CorrelationMatrix->first_node("Middle");
		rapidxml::xml_node<>* CorMiddlePIP = CorMiddle->first_node("PIP");
		m_RGloCaliK[2][0] = atof(CorMiddlePIP->value());
		rapidxml::xml_node<>* CorMiddleMP = CorMiddle->first_node("MP");
		m_RGloCaliK[2][1] = atof(CorMiddleMP->value());
		rapidxml::xml_node<>* CorMiddleABP = CorMiddle->first_node("ABP");
		m_RGloCaliK[2][2] = atof(CorMiddleABP->value());
		//无名指
		rapidxml::xml_node<>* CorRing = CorrelationMatrix->first_node("Ring");
		rapidxml::xml_node<>* CorRingPIP = CorRing->first_node("PIP");
		m_RGloCaliK[3][0] = atof(CorRingPIP->value());
		rapidxml::xml_node<>* CorRingMP = CorRing->first_node("MP");
		m_RGloCaliK[3][1] = atof(CorRingMP->value());
		rapidxml::xml_node<>* CorRingABP = CorRing->first_node("ABP");
		m_RGloCaliK[3][2] = atof(CorRingABP->value());
		//小指
		rapidxml::xml_node<>* CorLittle = CorrelationMatrix->first_node("Little");
		rapidxml::xml_node<>* CorLittlePIP = CorLittle->first_node("PIP");
		m_RGloCaliK[4][0] = atof(CorLittlePIP->value());
		rapidxml::xml_node<>* CorLittleMP = CorLittle->first_node("MP");
		m_RGloCaliK[4][1] = atof(CorLittleMP->value());
		rapidxml::xml_node<>* CorLittleABP = CorLittle->first_node("ABP");
		m_RGloCaliK[4][2] = atof(CorLittleABP->value());

		//获取偏置矩阵
		rapidxml::xml_node<>* BiasMatrix = GloveCalibration->first_node("BiasMatrix");
		rapidxml::xml_attribute<>* BiasMatrix_DataType = BiasMatrix->first_attribute();
		//拇指
		rapidxml::xml_node<>* BiasThumb = BiasMatrix->first_node("Thumb");
		rapidxml::xml_node<>* BiasThumbPIP = BiasThumb->first_node("PIP");
		m_RGloCaliB[0][0] = atof(BiasThumbPIP->value());
		rapidxml::xml_node<>* BiasThumbMP = BiasThumb->first_node("MP");
		m_RGloCaliB[0][1] = atof(BiasThumbMP->value());
		rapidxml::xml_node<>* BiasThumbABP = BiasThumb->first_node("ABP");
		m_RGloCaliB[0][2] = atof(BiasThumbABP->value());
		//食指
		rapidxml::xml_node<>* BiasIndex = BiasMatrix->first_node("Index");
		rapidxml::xml_node<>* BiasIndexPIP = BiasIndex->first_node("PIP");
		m_RGloCaliB[1][0] = atof(BiasIndexPIP->value());
		rapidxml::xml_node<>* BiasIndexMP = BiasIndex->first_node("MP");
		m_RGloCaliB[1][1] = atof(BiasIndexMP->value());
		rapidxml::xml_node<>* BiasIndexABP = BiasIndex->first_node("ABP");
		m_RGloCaliB[1][2] = atof(BiasIndexABP->value());
		//中指
		rapidxml::xml_node<>* BiasMiddle = BiasMatrix->first_node("Middle");
		rapidxml::xml_node<>* BiasMiddlePIP = BiasMiddle->first_node("PIP");
		m_RGloCaliB[2][0] = atof(BiasMiddlePIP->value());
		rapidxml::xml_node<>* BiasMiddleMP = BiasMiddle->first_node("MP");
		m_RGloCaliB[2][1] = atof(BiasMiddleMP->value());
		rapidxml::xml_node<>* BiasMiddleABP = BiasMiddle->first_node("ABP");
		m_RGloCaliB[2][2] = atof(BiasMiddleABP->value());
		//无名指
		rapidxml::xml_node<>* BiasRing = BiasMatrix->first_node("Ring");
		rapidxml::xml_node<>* BiasRingPIP = BiasRing->first_node("PIP");
		m_RGloCaliB[3][0] = atof(BiasRingPIP->value());
		rapidxml::xml_node<>* BiasRingMP = BiasRing->first_node("MP");
		m_RGloCaliB[3][1] = atof(BiasRingMP->value());
		rapidxml::xml_node<>* BiasRingABP = BiasRing->first_node("ABP");
		m_RGloCaliB[3][2] = atof(BiasRingABP->value());
		//小指
		rapidxml::xml_node<>* BiasLittle = BiasMatrix->first_node("Little");
		rapidxml::xml_node<>* BiasLittlePIP = BiasLittle->first_node("PIP");
		m_RGloCaliB[4][0] = atof(BiasLittlePIP->value());
		rapidxml::xml_node<>* BiasLittleMP = BiasLittle->first_node("MP");
		m_RGloCaliB[4][1] = atof(BiasLittleMP->value());
		rapidxml::xml_node<>* BiasLittleABP = BiasLittle->first_node("ABP");
		m_RGloCaliB[4][2] = atof(BiasLittleABP->value());

		m_CyberStation.UpdateRGloCoeff(m_RGloCaliK, m_RGloCaliB);
		m_bRGloCaliFin = true;
		ui.m_pHandConnBtn->setEnabled(true);
		m_CmdStr = "Load Config Success!!!\r\n" + m_CmdStr;
		InsertCmdStr(m_CmdStr);
	}
	else{
		m_CmdStr = "Config File is Empty!!!\r\n" + m_CmdStr;
		InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::SaveGloCaliData()
{	
	m_CyberStation.GetRGloCoeff(m_RGloCaliK, m_RGloCaliB);


	char ch_Correlation[5][3][8],ch_Bias[5][3][8];

	for (int i=0;i<5;i++){
		for(int j=0;j<=2;j++){
			gcvt(m_RGloCaliK[i][j],5,ch_Correlation[i][j]);
			gcvt(m_RGloCaliB[i][j],5,ch_Bias[i][j]);
		}
	}

	//建立.xml文件节点
	rapidxml::xml_document<> Calibration;
	//建立声明节点
	rapidxml::xml_node<>* prolog = Calibration.allocate_node(rapidxml::node_pi,Calibration.allocate_string("xml version='1.0' encoding='utf-8' standalone='yes'"));
	Calibration.append_node(prolog);
	//建立根节点
	rapidxml::xml_node<>* GloveCalibration = Calibration.allocate_node(rapidxml::node_element,"GloveCalibration");
	Calibration.append_node(GloveCalibration);

	//建立相关矩阵节点
	rapidxml::xml_node<>* CorrelationMatrix = Calibration.allocate_node(rapidxml::node_element,"CorrelationMatrix");
	GloveCalibration->append_node(CorrelationMatrix);
	rapidxml::xml_attribute<>* CorrelationMatrixAttribute = Calibration.allocate_attribute("DataType","float");
	CorrelationMatrix->append_attribute(CorrelationMatrixAttribute);


	//拇指
	rapidxml::xml_node<>* CorThumb = Calibration.allocate_node(rapidxml::node_element,"Thumb");
	CorrelationMatrix->append_node(CorThumb);
	CorThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Correlation[0][0]));
	CorThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Correlation[0][1]));
	CorThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Correlation[0][2]));
	//食指
	rapidxml::xml_node<>* CorIndex = Calibration.allocate_node(rapidxml::node_element,"Index");
	CorrelationMatrix->append_node(CorIndex);
	CorIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Correlation[1][0]));
	CorIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Correlation[1][1]));
	CorIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Correlation[1][2]));
	//中指
	rapidxml::xml_node<>* CorMiddle = Calibration.allocate_node(rapidxml::node_element,"Middle");
	CorrelationMatrix->append_node(CorMiddle);
	CorMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Correlation[2][0]));
	CorMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Correlation[2][1]));
	CorMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Correlation[2][2]));
	//无名指
	rapidxml::xml_node<>* CorRing = Calibration.allocate_node(rapidxml::node_element,"Ring");
	CorrelationMatrix->append_node(CorRing);
	CorRing->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Correlation[3][0]));
	CorRing->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Correlation[3][1]));
	CorRing->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Correlation[3][2]));
	//小指
	rapidxml::xml_node<>* CorLittle = Calibration.allocate_node(rapidxml::node_element,"Little");
	CorrelationMatrix->append_node(CorLittle);
	CorLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Correlation[4][0]));
	CorLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Correlation[4][1]));
	CorLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Correlation[4][2]));

	//建立偏置矩阵节点
	rapidxml::xml_node<>* BiasMatrix = Calibration.allocate_node(rapidxml::node_element,"BiasMatrix");
	GloveCalibration->append_node(BiasMatrix);
	rapidxml::xml_attribute<>* BiasMatrixAttribute = Calibration.allocate_attribute("DataType","float");
	BiasMatrix->append_attribute(BiasMatrixAttribute);
	//拇指
	rapidxml::xml_node<>* BiasThumb = Calibration.allocate_node(rapidxml::node_element,"Thumb");
	BiasMatrix->append_node(BiasThumb);
	BiasThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Bias[0][0]));
	BiasThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Bias[0][1]));
	BiasThumb->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Bias[0][2]));
	//食指
	rapidxml::xml_node<>* BiasIndex = Calibration.allocate_node(rapidxml::node_element,"Index");
	BiasMatrix->append_node(BiasIndex);
	BiasIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Bias[1][0]));
	BiasIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Bias[1][1]));
	BiasIndex->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Bias[1][2]));
	//中指
	rapidxml::xml_node<>* BiasMiddle = Calibration.allocate_node(rapidxml::node_element,"Middle");
	BiasMatrix->append_node(BiasMiddle);
	BiasMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Bias[2][0]));
	BiasMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Bias[2][1]));
	BiasMiddle->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Bias[2][2]));
	//无名指
	rapidxml::xml_node<>* BiasRing = Calibration.allocate_node(rapidxml::node_element,"Ring");
	BiasMatrix->append_node(BiasRing);
	BiasRing->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Bias[3][0]));
	BiasRing->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Bias[3][1]));
	BiasRing->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Bias[3][2]));
	//小指
	rapidxml::xml_node<>* BiasLittle = Calibration.allocate_node(rapidxml::node_element,"Little");
	BiasMatrix->append_node(BiasLittle);
	BiasLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"PIP",ch_Bias[4][0]));
	BiasLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"MP",ch_Bias[4][1]));
	BiasLittle->append_node(Calibration.allocate_node(rapidxml::node_element,"ABP",ch_Bias[4][2]));

	//写入到.xml文件
	std::string PrintXml;
	rapidxml::print(std::back_inserter(PrintXml), Calibration, 0);

	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Calibration Data"), "./Data/Untitled.XML",
		tr("XML files (*.xml)"));

	QFile XmlFile(fileName);
	if(!XmlFile.open(QIODevice::ReadWrite | QIODevice::Truncate))
	{
		QMessageBox::about(NULL, "About", "Fail to create .XML file");
	}
	else
	{
		XmlFile.write((char*)PrintXml.c_str(),strlen(PrintXml.c_str()));
	}
	XmlFile.flush();
	XmlFile.close();
	QMessageBox::about(NULL, "About", "Finish writing calibration data!");
}


// Push Different Gesture Button to Store Glove Data
void CyberSystem::GesOneData()
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m_RGloCaliData[0][i][j] = m_RGloRawData[i][j];
		}
	}
	ui.m_pGesBtn_one->setEnabled(false);
}
void CyberSystem::GesTwoData()
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m_RGloCaliData[1][i][j] = m_RGloRawData[i][j];
		}
	}
	ui.m_pGesBtn_two->setEnabled(false);
}
void CyberSystem::GesThrData()
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m_RGloCaliData[2][i][j] = m_RGloRawData[i][j];
		}
	}
	ui.m_pGesBtn_three->setEnabled(false);
}
void CyberSystem::GesFourData()
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m_RGloCaliData[3][i][j] = m_RGloRawData[i][j];
		}
	}
	ui.m_pGesBtn_four->setEnabled(false);
}

// Push Finish Button to Calculate Glove Calibration Data
void CyberSystem::FinGloveCali()
{
	m_CyberStation.CalRGloCoeff(m_RGloCaliData);
	// control real data display and calibration finish
	m_bRGloCaliFin = true;
	ui.m_pHandConnBtn->setEnabled(true);
}


//*********************** CyberTracker Calibration Options ***********************//
// start display
void CyberSystem::InitTraCali()
{
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Tracker?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
	{
		m_CmdStr += "Start Tracker Display......OK!\r\n";
		emit InsertCmdStr(m_CmdStr);
		if (!(m_DisThread.isRunning()))
		{
			// new thread to display raw data
			m_bDisTraData = true;
			m_DisThread.start();
		}
		else{
			m_bDisTraData = true;
		}
	}  
	else{}  
}

// push calculate button to calculate the coefficients
void CyberSystem::CalTraData()
{
	//bool bRTraTab = ui.m_pRTraTab->isEnabled();
	if (ui.m_pRTraTab->isEnabled())
	{
		QString XPos = ui.m_pRPosXLiEd->text();
		QString YPos = ui.m_pRPosYLiEd->text();
		QString ZPos = ui.m_pRPosZLiEd->text();
		QString XOri = ui.m_pROriXLiEd->text();
		QString YOri = ui.m_pROriYLiEd->text();
		QString ZOri = ui.m_pROriZLiEd->text();

		double x = 0;
		double y = 100;
		double z = 0;

		// TODO(CJH): Add Calibration Data
		double pose[6];
		pose[0] = 0;
		pose[1] = -33.7;
		pose[2] = 30.7;

		m_CyberStation.CalRTraCoef(pose, 6);
		m_bRTraCaliFini = true;

		QMessageBox::about(NULL, "About", "CyberTracker calibration is finished");
		ui.m_pSimuConnBtn->setEnabled(true);
		ui.m_pRoboConnBtn->setEnabled(true);
	} 
	else
	{
		QString XPos = ui.m_pLPosXLiEd->text();
		QString YPos = ui.m_pLPosYLiEd->text();
		QString ZPos = ui.m_pLPosZLiEd->text();
		QString XOri = ui.m_pLOriXLiEd->text();
		QString YOri = ui.m_pLOriYLiEd->text();
		QString ZOri = ui.m_pLOriZLiEd->text();

		double pose[6];

		m_CyberStation.CalLTraCoef(pose, sizeof(pose));
		m_bLTraCaliFini = true;
	}
}

void CyberSystem::LoadTraCaliData()
{
	Eigen::Matrix<double, 4, 4> TransMat;
	Eigen::Matrix<double, 3, 3> RotMat;

	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Calibration Data"), ".",
		tr("XML files (*.xml)"));

	if (!fileName.isEmpty()){
		std::string str_filename = fileName.toStdString();
		//******************************* Read in XML *******************************//
		rapidxml::file<> fCalibration(str_filename.c_str());

		rapidxml::xml_document<> Calibration;
		Calibration.parse<0>(fCalibration.data());

		// 获取根节点
		rapidxml::xml_node<>* TrackerCalibration = Calibration.first_node();

		// 获取相关矩阵
		rapidxml::xml_node<>* TransMatrix = TrackerCalibration->first_node("TransMatrix");
		rapidxml::xml_attribute<>* TransMatrix_DataType = TransMatrix->first_attribute();
		// 第一行
		rapidxml::xml_node<>* TransFirst = TransMatrix->first_node("TransFirst");
		rapidxml::xml_node<>* TransFirstA = TransFirst->first_node("A");
		TransMat(0, 0) = atof(TransFirstA->value());
		rapidxml::xml_node<>* TransFirstB = TransFirst->first_node("B");
		TransMat(0, 1) = atof(TransFirstB->value());
		rapidxml::xml_node<>* TransFirstC = TransFirst->first_node("C");
		TransMat(0, 2) = atof(TransFirstC->value());
		rapidxml::xml_node<>* TransFirstD = TransFirst->first_node("D");
		TransMat(0, 3) = atof(TransFirstD->value());

		// 第二行
		rapidxml::xml_node<>* TransSecond = TransMatrix->first_node("TransSecond");
		rapidxml::xml_node<>* TransSecondA = TransSecond->first_node("A");
		TransMat(1, 0) = atof(TransSecondA->value());
		rapidxml::xml_node<>* TransSecondB = TransSecond->first_node("B");
		TransMat(1, 1) = atof(TransSecondB->value());
		rapidxml::xml_node<>* TransSecondC = TransSecond->first_node("C");
		TransMat(1, 2) = atof(TransSecondC->value());
		rapidxml::xml_node<>* TransSecondD = TransSecond->first_node("D");
		TransMat(1, 3) = atof(TransSecondD->value());

		// 第三行
		rapidxml::xml_node<>* TransThird = TransMatrix->first_node("TransThird");
		rapidxml::xml_node<>* TransThirdA = TransThird->first_node("A");
		TransMat(2, 0) = atof(TransThirdA->value());
		rapidxml::xml_node<>* TransThirdB = TransThird->first_node("B");
		TransMat(2, 1) = atof(TransThirdB->value());
		rapidxml::xml_node<>* TransThirdC = TransThird->first_node("C");
		TransMat(2, 2) = atof(TransThirdC->value());
		rapidxml::xml_node<>* TransThirdD = TransThird->first_node("D");
		TransMat(2, 3) = atof(TransThirdD->value());

		// 第四行
		rapidxml::xml_node<>* TransFourth = TransMatrix->first_node("TransFourth");
		rapidxml::xml_node<>* TransFourthA = TransFourth->first_node("A");
		TransMat(3, 0) = atof(TransFourthA->value());
		rapidxml::xml_node<>* TransFourthB = TransFourth->first_node("B");
		TransMat(3, 1) = atof(TransFourthB->value());
		rapidxml::xml_node<>* TransFourthC = TransFourth->first_node("C");
		TransMat(3, 2) = atof(TransFourthC->value());
		rapidxml::xml_node<>* TransFourthD = TransFourth->first_node("D");
		TransMat(3, 3) = atof(TransFourthD->value());


		//
		rapidxml::xml_node<>* RotMatrix = TrackerCalibration->first_node("RotMatrix");
		rapidxml::xml_attribute<>* RotMatrix_DataType = RotMatrix->first_attribute();
		//第一行
		rapidxml::xml_node<>* RotFirst = RotMatrix->first_node("RotFirst");
		rapidxml::xml_node<>* RotFirstA = RotFirst->first_node("A");
		RotMat(0, 0) = atof(RotFirstA->value());
		rapidxml::xml_node<>* RotFirstB = RotFirst->first_node("B");
		RotMat(0, 1) = atof(RotFirstB->value());
		rapidxml::xml_node<>* RotFirstC = RotFirst->first_node("C");
		RotMat(0, 2) = atof(RotFirstC->value());
		//第二行
		rapidxml::xml_node<>* RotSecond = RotMatrix->first_node("RotSecond");
		rapidxml::xml_node<>* RotSecondA = RotSecond->first_node("A");
		RotMat(1, 0) = atof(RotSecondA->value());
		rapidxml::xml_node<>* RotSecondB = RotSecond->first_node("B");
		RotMat(1, 1) = atof(RotSecondB->value());
		rapidxml::xml_node<>* RotSecondC = RotSecond->first_node("C");
		RotMat(1, 2) = atof(RotSecondC->value());
		//第三行
		rapidxml::xml_node<>* RotThird = RotMatrix->first_node("RotThird");
		rapidxml::xml_node<>* RotThirdA = RotThird->first_node("A");
		RotMat(2, 0) = atof(RotThirdA->value());
		rapidxml::xml_node<>* RotThirdB = RotThird->first_node("B");
		RotMat(2, 1) = atof(RotThirdB->value());
		rapidxml::xml_node<>* RotThirdC = RotThird->first_node("C");
		RotMat(2, 2) = atof(RotThirdC->value());


		m_CyberStation.UpdateTraCali(TransMat, RotMat);
		m_bRTraCaliFini = true;
		m_CmdStr = "Load Config Success!!!\r\n" + m_CmdStr;
		InsertCmdStr(m_CmdStr);
	}
	else{
		m_CmdStr = "Config File is Empty!!!\r\n" + m_CmdStr;
		InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::SaveCaliData()
{
	Eigen::Matrix<double, 4, 4> TransMat;
	Eigen::Matrix<double, 3, 3> RotMat;
	m_CyberStation.GetCaliCoef(TransMat, RotMat);

	// XML中需要储存char型的变量
	char ch_TransMat[4][4][8];
	char ch_RotMat[3][3][8];

	
	std::ostringstream out_str;

	for (int i = 0; i < 4; ++i){
		for(int j = 0; j < 4; ++j){
//			gcvt(TransMat(i, j),5,ch_TransMat[i][j]);

			out_str << TransMat(i, j);
			memcpy(ch_TransMat[i][j], out_str.str().c_str(), 8);
			out_str.str("");
		}
	}

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
//			gcvt(RotMat(i,j),5,ch_RotMat[i][j]);			

 			out_str << RotMat(i, j);
 			memcpy(ch_RotMat[i][j], out_str.str().c_str(), 8);
 			out_str.str("");
		}
	}


	//建立.xml文件节点
	rapidxml::xml_document<> Calibration;
	//建立声明节点
	rapidxml::xml_node<>* prolog = Calibration.allocate_node(rapidxml::node_pi,Calibration.allocate_string("xml version='1.0' encoding='utf-8' standalone='yes'"));
	Calibration.append_node(prolog);
	//建立根节点
	rapidxml::xml_node<>* TrackerCalibration = Calibration.allocate_node(rapidxml::node_element,"TrackerCalibration");
	Calibration.append_node(TrackerCalibration);


	//建立相关矩阵节点
	rapidxml::xml_node<>* TransMatrix = Calibration.allocate_node(rapidxml::node_element,"TransMatrix");
	TrackerCalibration->append_node(TransMatrix);
	rapidxml::xml_attribute<>* TransMatrixAttribute = Calibration.allocate_attribute("DataType","float");
	TransMatrix->append_attribute(TransMatrixAttribute);

	//第一行
	rapidxml::xml_node<>* TransFirst = Calibration.allocate_node(rapidxml::node_element,"TransFirst");
	TransMatrix->append_node(TransFirst);
	TransFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_TransMat[0][0]));
	TransFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_TransMat[0][1]));
	TransFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_TransMat[0][2]));
	TransFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"D",ch_TransMat[0][3]));
	//第二行
	rapidxml::xml_node<>* TransSecond = Calibration.allocate_node(rapidxml::node_element,"TransSecond");
	TransMatrix->append_node(TransSecond);
	TransSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_TransMat[1][0]));
	TransSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_TransMat[1][1]));
	TransSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_TransMat[1][2]));
	TransSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"D",ch_TransMat[1][3]));
	//第三行
	rapidxml::xml_node<>* TransThird = Calibration.allocate_node(rapidxml::node_element,"TransThird");
	TransMatrix->append_node(TransThird);
	TransThird->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_TransMat[2][0]));
	TransThird->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_TransMat[2][1]));
	TransThird->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_TransMat[2][2]));
	TransThird->append_node(Calibration.allocate_node(rapidxml::node_element,"D",ch_TransMat[2][3]));
	//第四行
	rapidxml::xml_node<>* TransFourth = Calibration.allocate_node(rapidxml::node_element,"TransFourth");
	TransMatrix->append_node(TransFourth);
	TransFourth->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_TransMat[3][0]));
	TransFourth->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_TransMat[3][1]));
	TransFourth->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_TransMat[3][2]));
	TransFourth->append_node(Calibration.allocate_node(rapidxml::node_element,"D",ch_TransMat[3][3]));



	// 旋转标定矩阵
	rapidxml::xml_node<>* RotMatrix = Calibration.allocate_node(rapidxml::node_element,"RotMatrix");
	TrackerCalibration->append_node(RotMatrix);
	rapidxml::xml_attribute<>* RotMatrixAttribute = Calibration.allocate_attribute("DataType","float");
	RotMatrix->append_attribute(RotMatrixAttribute);
	// 第一行
	rapidxml::xml_node<>* RotFirst = Calibration.allocate_node(rapidxml::node_element,"RotFirst");
	RotMatrix->append_node(RotFirst);
	RotFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_RotMat[0][0]));
	RotFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_RotMat[0][1]));
	RotFirst->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_RotMat[0][2]));
	// 第二行
	rapidxml::xml_node<>* RotSecond = Calibration.allocate_node(rapidxml::node_element,"RotSecond");
	RotMatrix->append_node(RotSecond);
	RotSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_RotMat[1][0]));
	RotSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_RotMat[1][1]));
	RotSecond->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_RotMat[1][2]));
	// 第三行
	rapidxml::xml_node<>* RotThird = Calibration.allocate_node(rapidxml::node_element,"RotThird");
	RotMatrix->append_node(RotThird);
	RotThird->append_node(Calibration.allocate_node(rapidxml::node_element,"A",ch_RotMat[2][0]));
	RotThird->append_node(Calibration.allocate_node(rapidxml::node_element,"B",ch_RotMat[2][1]));
	RotThird->append_node(Calibration.allocate_node(rapidxml::node_element,"C",ch_RotMat[2][2]));
	

	//写入到.xml文件
	std::string PrintXml;
	rapidxml::print(std::back_inserter(PrintXml), Calibration, 0);


	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Calibration Data"), "./Data/Untitled.XML",
		tr("XML files (*.xml)"));


	QFile Tra_XmlFile(fileName);
	if(!Tra_XmlFile.open(QIODevice::ReadWrite | QIODevice::Truncate))
	{
		QMessageBox::about(NULL, "About", "Fail to create .XML file");
	}
	else
	{
		Tra_XmlFile.write((char*)PrintXml.c_str(),strlen(PrintXml.c_str()));
	}
	Tra_XmlFile.flush();
	Tra_XmlFile.close();
	QMessageBox::about(NULL, "About", "Finish writing calibration data!");
}






void CyberSystem::lineEdit_textChanged()
{
//	bool bRTraTab = ui.m_pRTraTab->isEnabled();
	if (ui.m_pRTraTab->isEnabled())
	{
// 		bool bRPosX = ui.m_pRPosXLiEd->hasAcceptableInput();
// 		bool bRPosY = ui.m_pRPosYLiEd->hasAcceptableInput();
// 		bool bRPosZ = ui.m_pRPosZLiEd->hasAcceptableInput();
// 		bool bROriX = ui.m_pROriXLiEd->hasAcceptableInput();
// 		bool bROriY = ui.m_pROriYLiEd->hasAcceptableInput();
// 		bool bROriZ = ui.m_pROriZLiEd->hasAcceptableInput();
// 		bool bRTraBtn = bRPosX && bRPosY && bRPosZ && bROriX && bROriY && bROriZ;
// 		ui.m_pTraCalBtn->setEnabled(bRTraBtn);
		ui.m_pTraCalBtn->setEnabled(ui.m_pRPosXLiEd->hasAcceptableInput() && ui.m_pRPosYLiEd->hasAcceptableInput()
								&& ui.m_pRPosZLiEd->hasAcceptableInput() && ui.m_pROriXLiEd->hasAcceptableInput()
								&& ui.m_pROriYLiEd->hasAcceptableInput() && ui.m_pROriZLiEd->hasAcceptableInput());
	} 
	else
	{
		ui.m_pTraCalBtn->setEnabled(ui.m_pLPosXLiEd->hasAcceptableInput() && ui.m_pLPosYLiEd->hasAcceptableInput()
								&& ui.m_pLPosZLiEd->hasAcceptableInput() && ui.m_pLOriXLiEd->hasAcceptableInput()
								&& ui.m_pLOriYLiEd->hasAcceptableInput() && ui.m_pLOriZLiEd->hasAcceptableInput());
	}	
}


// Receive Robonaut Data
void CyberSystem::DisRoboData()
{
	std::ostringstream send_out_str;
	std::ostringstream sensor_out_str;

	send_out_str << "********* Send Data ********" << std::endl;
	send_out_str << std::fixed << std::left;
	send_out_str.precision(3);

	send_out_str << "Right Arm Joints: " << std::endl;
	for (int i = 0; i < 7; ++i)
	{
		send_out_str.width(8);
		send_out_str << g_RobotCmdDeg.rightArmJoint[i];
	}
	send_out_str << std::endl;
	send_out_str << "Left Arm Joints: " << std::endl;
	for ( int i = 0; i < 7; ++i)
	{
		send_out_str.width(8);
		send_out_str << g_RobotCmdDeg.leftArmJoint[i];
	}
	send_out_str << std::endl;

	sensor_out_str << "******** Receive Data ********" << std::endl;
	sensor_out_str << std::fixed << std::left;
	sensor_out_str.precision(3);

	sensor_out_str << "Right Arm Joints: " << std::endl;
	for (int i = 0; i < 7; ++i)
	{
		sensor_out_str.width(8);
		sensor_out_str << g_RobotSensorDeg.rightArmJoint[i];
	}
	sensor_out_str << std::endl;
	sensor_out_str << "Left Arm Joints: " << std::endl;
	for ( int i = 0; i < 7; ++i)
	{
		sensor_out_str.width(8);
		sensor_out_str << g_RobotSensorDeg.leftArmJoint[i];
	}
	sensor_out_str << std::endl;

	// 没有连接宇航员，不现实传感器数据
	if (m_bRoboConn == false && m_bConsimuConn == true){
		m_RoboStr = m_RoboStr.fromStdString(send_out_str.str());
	}
	// 连接了宇航员，显示发送到数据和传感器数据
	else if (m_bRoboConn == true){
		m_RoboStr = m_RoboStr.fromStdString((send_out_str.str() + sensor_out_str.str()));
	}

	m_DisDataMutex.lock();
	m_RoboTotalStr = m_RoboStr + m_HandStr;
	emit InsertRoboText(m_RoboTotalStr);
	m_DisDataMutex.unlock();
}


void PASCAL TimerProcRecvSensor(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	g_pCyberSys->RecvSensor();
}

void PASCAL TimerProcSendCmd(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	g_pCyberSys->SendCmd();
}

// 收数循环
void CyberSystem::RecvSensor()
{
	if (m_bRoboConn == true)
	{
		bool ret = m_RobonautControl.RecvRoboMsg();
	}
}


// 发数控制循环
void CyberSystem::SendCmd()
{
	// 使用Cyber控制循环
	if (m_CtrlMode == CYBER_CTRL_ALL || m_CtrlMode == CYBER_CTRL_ROBO || m_CtrlMode == CYBER_CTRL_SIMU)
	{
		Eigen::Matrix<double, 7, 1> q = CalKine(m_RTraRealMat, m_last_arm_angle, m_last_joint_angle);

		// g_rightArmJointBuf is the Command Buf
		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = DEG2ANG(q(i));
			g_leftArmJointBuf[i] = 0;
		}

		if (m_CtrlMode == CYBER_CTRL_ALL)
		{
			bool consimu_ret = m_RobonautControl.SendConsimuMsg();
			bool robo_ret = m_RobonautControl.SendRoboMsg();
		}
		else if(m_CtrlMode == CYBER_CTRL_ROBO){
			bool ret = m_RobonautControl.SendRoboMsg();
		}
		else{
			bool ret = m_RobonautControl.SendConsimuMsg();
		}
	}

	// 使用Click控制的循环
	else if (m_CtrlMode == CLICK_CTRL_ALL || m_CtrlMode == CLICK_CTRL_ROBO || m_CtrlMode == CLICK_CTRL_SIMU)
	{
		if (m_CtrlMode == CLICK_CTRL_ALL)
		{
			bool consimu_ret = m_RobonautControl.SendConsimuMsg();
			bool robo_ret = m_RobonautControl.SendRoboMsg();
		}
		else if(m_CtrlMode == CLICK_CTRL_ROBO){
			bool ret = m_RobonautControl.SendRoboMsg();
		}
		else{
			bool ret = m_RobonautControl.SendConsimuMsg();
		}
	}

	// 无控制
	// 如果连接了宇航员，发送宇航员传感器数据；如果没有连接，发送0角度
	else{
		if (m_bRoboConn == true)
		{
			for (int i = 0; i < 7; ++i)
			{
				g_rightArmJointBuf[i] = g_RobotSensorDeg.rightArmJoint[i];
				g_leftArmJointBuf[i] = g_RobotSensorDeg.leftArmJoint[i];
			}
			if (m_bConsimuConn == true)
			{
				bool consimu_ret = m_RobonautControl.SendConsimuMsg();
				bool robo_ret = m_RobonautControl.SendRoboMsg();
			}
			else{
				bool robo_ret = m_RobonautControl.SendRoboMsg();
			}
		} 
		else if (m_bConsimuConn == true && m_bRoboConn == false)
		{
			for (int i = 0; i < 7; ++i)
			{
				g_rightArmJointBuf[i] = 0;
				g_leftArmJointBuf[i] = 0;
			}
			bool consimu_ret = m_RobonautControl.SendConsimuMsg();
		}
		else{
			m_CmdStr += "No Connection! Check Error!!!";
			emit InsertCmdStr(m_CmdStr);

			timeKillEvent(RecvTimerId);
			RecvTimerId = NULL;
			timeKillEvent(SendTimerId);
			SendTimerId = NULL;
		}
	}
	DisRoboData();
}


// Connect and DisConnect Consimu
void CyberSystem::ConsimuConnCtrl()
{
	m_CmdStr += "Connecting Consimu......";
	emit InsertCmdStr(m_CmdStr);

	if (m_bConsimuConn == false)
	{
		bool ret = m_RobonautControl.ConnConsimu();
		if (ret == true)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);

			m_bConsimuConn = true;
			ui.m_pSimuConnBtn->setText("Disconn Consimu");
			ui.m_pJointCtrlBtn->setEnabled(true);
		}
		else{
			m_CmdStr += "Failed!\r\n";
			emit InsertCmdStr(m_CmdStr);

			QMessageBox::critical(NULL, "Error", "Consimu Connection Failed");
		}
	}

	// TODO(CJH): Add Disconnected options
	else{
		m_CmdStr += "DisConnecting Consimu......";
		emit InsertCmdStr(m_CmdStr);

		bool ret = m_RobonautControl.DisConnConsimu();
		if (ret == true)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);
			m_bConsimuConn = false;
			ui.m_pSimuConnBtn->setText("Conn Consimu");
			ui.m_pJointCtrlBtn->setEnabled(false);
		}
	}
}


// Connect Robonaut
void CyberSystem::RoboConnCtrl()
{
	if (m_bRoboConn == false)
	{
		m_CmdStr += "Creating Robonaut Connection......";
		emit InsertCmdStr(m_CmdStr);

		bool ret = m_RobonautControl.ConnRobo();
		if (ret == true)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);

			m_bRoboConn = true;
			ui.m_pRoboConnBtn->setText("Disconn Robonaut");
			ui.m_pJointCtrlBtn->setEnabled(true);
		}
		else{
			m_CmdStr += "Failed!!!";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
	}

	else{
		// TODO(CJH): add bool
		// Disconnct function is not add
		m_CmdStr += "DisConnecting Robonaut......";
		emit InsertCmdStr(m_CmdStr);

		bool ret = m_RobonautControl.DisConnRobo();
		if (ret == true)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);

			m_bRoboConn = false;
			ui.m_pRoboConnBtn->setText("Conn Robonaut");
		}
		else{
			m_CmdStr += "Failed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	}
}

// TODO(CJH):Add Connection check and Calibration check
// 点击Cyber按钮，选择使用Cyber的模式
void CyberSystem::CyberCtrl()
{
	if (m_bRTraCaliFini == false && m_bLTraCaliFini == false)
	{
		m_CmdStr += "Calibrate Tracker First!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		return;
	}
	// 点击Cyber按钮
	if (m_CtrlMode == OUT_CTRL && (m_bRoboConn == true || m_bConsimuConn == true) && (ui.m_pCyberCtrlBtn->text() == "Cyber"))
	{
		m_CmdStr += "Initializing DataSend Timer......";
		emit InsertCmdStr(m_CmdStr);
		// Initialize Send Timer
		if (SendTimerId == NULL)
		{
			SendTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Send Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
		
		if (SendTimerId != NULL)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);
		} 
		else
		{
			m_CmdStr += "Failed!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		m_CmdStr += "Initializing DataRecv Timer......";
		emit InsertCmdStr(m_CmdStr);
		// Initialize Receive Timer
		if (RecvTimerId == NULL)
		{
			RecvTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcRecvSensor, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Recv Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
		
		if (RecvTimerId != NULL)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);

			ui.m_pCyberStartBtn->setEnabled(true);
			ui.m_pJointCtrlBtn->setEnabled(false);
			ui.m_pCyberCtrlBtn->setText("Stop");
		} 
		else
		{
			m_CmdStr += "Failed!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
	}

	// 点击Stop按钮
	else
	{
		m_CtrlMode = OUT_CTRL;
		Sleep(250);

		m_CmdStr += "Stop Cyber Control......";
		emit InsertCmdStr(m_CmdStr);

		UINT ret_send = timeKillEvent(SendTimerId);
		SendTimerId = NULL;
		UINT ret_recv = timeKillEvent(RecvTimerId);
		RecvTimerId = NULL;

		if (ret_send == TIMERR_NOERROR && ret_recv == TIMERR_NOERROR)
		{
			m_CmdStr += "OK!!!\r\n";
			emit InsertCmdStr(m_CmdStr);

			ui.m_pCyberCtrlBtn->setText("Cyber");
			ui.m_pCyberStartBtn->setEnabled(false);
			ui.m_pCyberPauseBtn->setEnabled(false);
			ui.m_pCyberCtrlBtn->setEnabled(true);
			ui.m_pJointCtrlBtn->setEnabled(true);
		} 
		else
		{
			m_CmdStr += "Failed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	}
}


// 点击Start按钮，开始控制
void CyberSystem::CyberCmd()
{
	m_CmdStr += "Start Cyber Control!!!\r\n";
	emit InsertCmdStr(m_CmdStr);

	if (m_bRoboConn == true && m_bConsimuConn == true)
	{
		m_CtrlMode = CYBER_CTRL_ALL;
	} 
	else if(m_bRoboConn == true && m_bConsimuConn == false)
	{
		m_CtrlMode = CYBER_CTRL_ROBO;
	}
	else{
		m_CtrlMode = CYBER_CTRL_SIMU;
	}

	ui.m_pCyberPauseBtn->setEnabled(true);
	ui.m_pCyberStartBtn->setEnabled(false);
}

// 暂停使用Cyber控制
void CyberSystem::CyberStop()
{
	m_CmdStr += "Pause Cyber Control!!!\r\n";
	emit InsertCmdStr(m_CmdStr);

	m_CtrlMode = OUT_CTRL;

	ui.m_pCyberStartBtn->setEnabled(true);
	ui.m_pCyberPauseBtn->setEnabled(false);
}




// 点击Click按钮，启动定时器
void CyberSystem::ClickCtrl()
{
	if ((m_CtrlMode == OUT_CTRL) && ((m_bConsimuConn || m_bRoboConn) == true))
	{
		// Control Mode
		if (m_bConsimuConn == true && m_bRoboConn == false){
			m_CtrlMode = CLICK_CTRL_SIMU;
		}
		else if(m_bConsimuConn == false && m_bRoboConn == true){
			m_CtrlMode = CLICK_CTRL_ROBO;
		}
		else{
			m_CtrlMode = CLICK_CTRL_ALL;
		}

		// DataSend Timer
		m_CmdStr += "Initializing DataSend Timer......";
		emit InsertCmdStr(m_CmdStr);

		if (SendTimerId == NULL)
		{
			SendTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Send Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		if (SendTimerId != NULL)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);
		} 
		else
		{
			m_CmdStr += "Failed!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		// DataRecv Timer
		m_CmdStr += "Initializing DataRecv Timer......";
		emit InsertCmdStr(m_CmdStr);

		if (RecvTimerId == NULL)
		{
			RecvTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcRecvSensor, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Recv Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		if (RecvTimerId != NULL)
		{
			m_CmdStr += "OK!\r\n";
			emit InsertCmdStr(m_CmdStr);

			ui.m_pSdDataBtn->setEnabled(true);
			ui.m_pUpdataBtn->setEnabled(true);
			ui.m_pCyberCtrlBtn->setEnabled(false);
			SetCliUIVisi(true);
			ui.m_pJointCtrlBtn->setText("Stop");
		} 
		else
		{
			m_CmdStr += "Failed!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
	}
	else{
		m_CtrlMode = OUT_CTRL;
		timeKillEvent(SendTimerId);
		SendTimerId = NULL;
		timeKillEvent(RecvTimerId);
		RecvTimerId = NULL;

		m_CmdStr += "Stop Click Control!!!\r\n";
		emit InsertCmdStr(m_CmdStr);

		ui.m_pSdDataBtn->setEnabled(false);
		ui.m_pUpdataBtn->setEnabled(false);
		ui.m_pCyberCtrlBtn->setEnabled(true);

		SetCliUIVisi(false);
		ui.m_pJointCtrlBtn->setText("Click");
	}
}


// 获取slider数据，更新g_rightArmJointBuf
void CyberSystem::getSliData()
{
	// Get Joint Data
	if (ui.m_pRArmTab->isVisible() || ui.m_pLArmTab->isVisible())
	{
		ui.m_pCommadBs->backward();

		m_CmdStr += "Sending Joints Data!!!\r\n";
		emit InsertCmdStr(m_CmdStr);

		float RJoTmp[7],LJoTmp[7];
		RJoTmp[0] = ui.m_pRJo0Sli->value();
		RJoTmp[1] = ui.m_pRJo1Sli->value();
		RJoTmp[2] = ui.m_pRJo2Sli->value();
		RJoTmp[3] = ui.m_pRJo3Sli->value();
		RJoTmp[4] = ui.m_pRJo4Sli->value();
		RJoTmp[5] = ui.m_pRJo5Sli->value();
		RJoTmp[6] = ui.m_pRJo6Sli->value();

		LJoTmp[0] = ui.m_pLJo0Sli->value();
		LJoTmp[1] = ui.m_pLJo1Sli->value();
		LJoTmp[2] = ui.m_pLJo2Sli->value();
		LJoTmp[3] = ui.m_pLJo3Sli->value();
		LJoTmp[4] = ui.m_pLJo4Sli->value();
		LJoTmp[5] = ui.m_pLJo5Sli->value();
		LJoTmp[6] = ui.m_pLJo6Sli->value();

		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = RJoTmp[i]/100.000;
			g_leftArmJointBuf[i] = LJoTmp[i]/100.000;
		}
	}
	// Get Position Data
	else
	{
		m_CmdStr += "Sending Pose Data!!!\r\n";
		emit InsertCmdStr(m_CmdStr);

		if (ui.m_pRPoseCmd->text() != "")
		{
			std::string rpos_str = ui.m_pRPoseCmd->text().toStdString();
			std::istringstream r_istr(rpos_str);

			int r_count = 0;
			for (int i = 0; i < 7 && (!r_istr.eof()); ++i)
			{
				r_istr >> m_rightArmPos[i];
				++r_count;
			}
			if (r_count != 7)
			{
				m_CmdStr += "There is No Enough Inputs for Right Tracker!!!\r\n";
				emit InsertCmdStr(m_CmdStr);
				return;
			}

			// calculate inverse kinetics
			Eigen::Matrix<double, 4, 4> T;

			QuaterToTrans(m_rightArmPos, T);
			Eigen::Matrix<double, 7, 1> q = CalKine(T, m_last_arm_angle, m_last_joint_angle);

			// final data
			for (int i = 0; i<7; i++)
			{		
				g_rightArmJointBuf[i] = DEG2ANG(q(i));
			}
		}
		// TODO(CJH): if input left arm data
		if (ui.m_pLPoseCmd->text() != "")
		{

			std::string lpos_str = ui.m_pLPoseCmd->text().toStdString();
			std::istringstream l_istr(lpos_str);
			int l_count = 0;
			for (int i = 0; i < 7 && (!l_istr.eof()); ++i)
			{
				l_istr >> m_leftArmPos[i];
				++l_count;
			}
			if (l_count != 7)
			{
				m_CmdStr += "There is No Enough Inputs for Left Tracker!!!\r\n";
				emit InsertCmdStr(m_CmdStr);
				return;
			}
			// calculate inverse kinetics
			Eigen::Matrix<double, 4, 4> T;

			QuaterToTrans(m_rightArmPos, T);
			Eigen::Matrix<double, 7, 1> q = CalKine(T, m_last_arm_angle, m_last_joint_angle);

			// final data
			for (int i = 0; i<7; i++)
			{		
				g_leftArmJointBuf[i] = DEG2ANG(q(i));
			}
		}



	}
}

// Calculate Unit Quaternion
// n = arr_in[3]	ex = arr_in[4]
// ey = arr_in[5]	ez = arr_in[6]
void CyberSystem::QuaterToTrans(const double arr_in[7], Eigen::Matrix<double, 4, 4> &mat_out)
{
	// romat_outamat_oute mamat_outrix
	mat_out(0,0) = 2*(pow(arr_in[3], 2) + pow(arr_in[4], 2)) - 1;
	mat_out(0,1) = 2*(arr_in[4]*arr_in[5] - arr_in[3]*arr_in[6]);
	mat_out(0,2) = 2*(arr_in[4]*arr_in[6] + arr_in[3]*arr_in[5]);
	mat_out(1,0) = 2*(arr_in[4]*arr_in[5] + arr_in[3]*arr_in[6]);
	mat_out(1,1) = 2*(pow(arr_in[3], 2) + pow(arr_in[5], 2)) - 1;
	mat_out(1,2) = 2*(arr_in[5]*arr_in[6] - arr_in[3]*arr_in[4]);
	mat_out(2,0) = 2*(arr_in[4]*arr_in[6] - arr_in[3]*arr_in[5]);
	mat_out(2,1) = 2*(arr_in[5]*arr_in[6] + arr_in[3]*arr_in[4]);
	mat_out(2,2) = 2*(pow(arr_in[3], 2) + pow(arr_in[6], 2)) - 1;
	// mat_outranspos mamat_outrix
	mat_out(0,3) = arr_in[0];
	mat_out(1,3) = arr_in[1];
	mat_out(2,3) = arr_in[2];
	// vision mamat_outrix
	mat_out(3,0) = 0;
	mat_out(3,1) = 0;
	mat_out(3,2) = 0;
	mat_out(3,3) = 1;
}


// Click Update Button, using global Robot Sensor Data to update slider data
void CyberSystem::SliUpdata()
{
	if (m_bRoboConn == true)
	{
		m_CmdStr += "Updating Sensor Data to Slider!!!\r\n";
		emit InsertCmdStr(m_CmdStr);

		float RJoTmp[7],LJoTmp[7];
		for (int i = 0; i < 7; ++i)
		{
			RJoTmp[i] = g_RobotSensorDeg.rightArmJoint[i]*100;
			LJoTmp[i] = g_RobotSensorDeg.leftArmJoint[i]*100;
		}

		SetSliVal(RJoTmp, RJoTmp);
	}
	else{
		m_CmdStr += "Robonaut is not Connected, Can't Update!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
	}



}

// 设置Slider数据
void CyberSystem::SetSliVal(float RJo[], float LJo[])
{
	// Robonaut joint move management
	ui.m_pRJo0Sli->setValue(RJo[0]);
	ui.m_pRJo1Sli->setValue(RJo[1]);
	ui.m_pRJo2Sli->setValue(RJo[2]);
	ui.m_pRJo3Sli->setValue(RJo[3]);
	ui.m_pRJo4Sli->setValue(RJo[4]);
	ui.m_pRJo5Sli->setValue(RJo[5]);
	ui.m_pRJo6Sli->setValue(RJo[6]);

	ui.m_pLJo0Sli->setValue(LJo[0]);
	ui.m_pLJo1Sli->setValue(LJo[1]);
	ui.m_pLJo2Sli->setValue(LJo[2]);
	ui.m_pLJo3Sli->setValue(LJo[3]);
	ui.m_pLJo4Sli->setValue(LJo[4]);
	ui.m_pLJo5Sli->setValue(LJo[5]);
	ui.m_pLJo6Sli->setValue(LJo[6]);
}

// 设置Slider可见
void CyberSystem::SetCliUIVisi(bool flag)
{
	ui.m_pRJo0Sli->setEnabled(flag);
	ui.m_pRJo1Sli->setEnabled(flag);
	ui.m_pRJo2Sli->setEnabled(flag);
	ui.m_pRJo3Sli->setEnabled(flag);
	ui.m_pRJo4Sli->setEnabled(flag);
	ui.m_pRJo5Sli->setEnabled(flag);
	ui.m_pRJo6Sli->setEnabled(flag);

	ui.m_pLJo0Sli->setEnabled(flag);
	ui.m_pLJo1Sli->setEnabled(flag);
	ui.m_pLJo2Sli->setEnabled(flag);
	ui.m_pLJo3Sli->setEnabled(flag);
	ui.m_pLJo4Sli->setEnabled(flag);
	ui.m_pLJo5Sli->setEnabled(flag);
	ui.m_pLJo6Sli->setEnabled(flag);

	ui.m_pRJo1Line->setEnabled(flag);
	ui.m_pRJo2Line->setEnabled(flag);
	ui.m_pRJo3Line->setEnabled(flag);
	ui.m_pRJo4Line->setEnabled(flag);
	ui.m_pRJo5Line->setEnabled(flag);
	ui.m_pRJo6Line->setEnabled(flag);
	ui.m_pRJo7Line->setEnabled(flag);

	ui.m_pLJo1Line->setEnabled(flag);
	ui.m_pLJo2Line->setEnabled(flag);
	ui.m_pLJo3Line->setEnabled(flag);
	ui.m_pLJo4Line->setEnabled(flag);
	ui.m_pLJo5Line->setEnabled(flag);
	ui.m_pLJo6Line->setEnabled(flag);
	ui.m_pLJo7Line->setEnabled(flag);
}


void CyberSystem::SlitoLine()
{
	float RJoTmp[7],LJoTmp[7];
	RJoTmp[0] = ui.m_pRJo0Sli->value();
	RJoTmp[1] = ui.m_pRJo1Sli->value();
	RJoTmp[2] = ui.m_pRJo2Sli->value();
	RJoTmp[3] = ui.m_pRJo3Sli->value();
	RJoTmp[4] = ui.m_pRJo4Sli->value();
	RJoTmp[5] = ui.m_pRJo5Sli->value();
	RJoTmp[6] = ui.m_pRJo6Sli->value();

	LJoTmp[0] = ui.m_pLJo0Sli->value();
	LJoTmp[1] = ui.m_pLJo1Sli->value();
	LJoTmp[2] = ui.m_pLJo2Sli->value();
	LJoTmp[3] = ui.m_pLJo3Sli->value();
	LJoTmp[4] = ui.m_pLJo4Sli->value();
	LJoTmp[5] = ui.m_pLJo5Sli->value();
	LJoTmp[6] = ui.m_pLJo6Sli->value();

	float RJoAng[7],LJoAng[7];
	for (int i = 0; i < 7; ++i)
	{
		RJoAng[i] = RJoTmp[i]/100.000;
		LJoAng[i] = LJoTmp[i]/100.000;
	}

	QString RStr[7],LStr[7];
	for (int i = 0; i < 7; ++i)
	{
		RStr[i] = QString("%1").arg(RJoAng[i]);
		LStr[i] = QString("%1").arg(LJoAng[i]);
	}
	ui.m_pRJo1Line->setText(RStr[0]);
	ui.m_pRJo2Line->setText(RStr[1]);
	ui.m_pRJo3Line->setText(RStr[2]);
	ui.m_pRJo4Line->setText(RStr[3]);
	ui.m_pRJo5Line->setText(RStr[4]);
	ui.m_pRJo6Line->setText(RStr[5]);
	ui.m_pRJo7Line->setText(RStr[6]);

	ui.m_pLJo1Line->setText(LStr[0]);
	ui.m_pLJo2Line->setText(LStr[1]);
	ui.m_pLJo3Line->setText(LStr[2]);
	ui.m_pLJo4Line->setText(LStr[3]);
	ui.m_pLJo5Line->setText(LStr[4]);
	ui.m_pLJo6Line->setText(LStr[5]);
	ui.m_pLJo7Line->setText(LStr[6]);
}

// TODO(CJH): Change editline data to slider data
void CyberSystem::LinetoSli()
{
	QString RStr[7],LStr[7];
	float RJo[7],LJo[7];

	RStr[0] = ui.m_pRJo1Line->text();
	RStr[1] = ui.m_pRJo2Line->text();
	RStr[2] = ui.m_pRJo3Line->text();
	RStr[3] = ui.m_pRJo4Line->text();
	RStr[4] = ui.m_pRJo5Line->text();
	RStr[5] = ui.m_pRJo6Line->text();
	RStr[6] = ui.m_pRJo7Line->text();

	LStr[0] = ui.m_pLJo1Line->text();
	LStr[1] = ui.m_pLJo2Line->text();
	LStr[2] = ui.m_pLJo3Line->text();
	LStr[3] = ui.m_pLJo4Line->text();
	LStr[4] = ui.m_pLJo5Line->text();
	LStr[5] = ui.m_pLJo6Line->text();
	LStr[6] = ui.m_pLJo7Line->text();

	for (int i = 0; i < 7; ++i)
	{
		RJo[i] = RStr[i].toFloat()*100.00;
		LJo[i] = LStr[i].toFloat()*100.00;
	}
	ui.m_pRJo0Sli->setValue(RJo[0]);
	ui.m_pRJo1Sli->setValue(RJo[1]);
	ui.m_pRJo2Sli->setValue(RJo[2]);
	ui.m_pRJo3Sli->setValue(RJo[3]);
	ui.m_pRJo4Sli->setValue(RJo[4]);
	ui.m_pRJo5Sli->setValue(RJo[5]);
	ui.m_pRJo6Sli->setValue(RJo[6]);

	ui.m_pLJo0Sli->setValue(LJo[0]);
	ui.m_pLJo1Sli->setValue(LJo[1]);
	ui.m_pLJo2Sli->setValue(LJo[2]);
	ui.m_pLJo3Sli->setValue(LJo[3]);
	ui.m_pLJo4Sli->setValue(LJo[4]);
	ui.m_pLJo5Sli->setValue(LJo[5]);
	ui.m_pLJo6Sli->setValue(LJo[6]);

}


//*********************** Hand Control ***********************//

void CyberSystem::InitHand()
{

	bool ret = m_RobonautControl.ConnHand();
	m_HandDataCount = 0;
	if (ret == true)
	{
		if (HSendTimerId == NULL)
		{
			HSendTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcHandSend, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Send Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
		if (HRecvTimerId == NULL)
		{
			HRecvTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcHandRecv, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Receive Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
		m_bHandConn = true;
		m_bHandInit = true;

		m_RobonautControl.setHandInit(m_bHandInit);

		ui.m_pHandEnableBtn->setEnabled(true);
	}
}

void CyberSystem::EnableHand()
{
	if (m_bHandEnable == false && ui.m_pHandEnableBtn->text() == "Enable")
	{
		m_bHandEnable = true;
		m_RobonautControl.setHandEnable(m_bHandEnable);
		ui.m_pHandRunBtn->setEnabled(true);
		ui.m_pEmgBtn->setEnabled(true);
	} 
	else
	{
		m_bHandEnable = false;
		m_RobonautControl.setHandEnable(m_bHandEnable);
		m_HandCtrlMode = HAND_OUT_CTRL;
		ui.m_pHandRunBtn->setText("Run");
		ui.m_pHandRunBtn->setEnabled(false);
		ui.m_pEmgBtn->setEnabled(false);
	}

}

void CyberSystem::EmergHand()
{
	if (m_bHandStop == false)
	{
		m_bHandStop = true;
		m_RobonautControl.setHandEmergency(m_bHandStop);
		
		ui.m_pEmgBtn->setText("Go");
	} 
	else
	{
		m_bHandStop = false;
		m_RobonautControl.setHandEmergency(m_bHandStop);
		
		ui.m_pEmgBtn->setText("Emergency");
	}
}

void CyberSystem::StartHand()
{
	if (m_HandCtrlMode == HAND_OUT_CTRL && ui.m_pHandRunBtn->text() == "Run")
	{
		m_HandCtrlMode = HAND_CYBER_CTRL;
		ui.m_pHandRunBtn->setText("Stop");
	} 
	else
	{
		m_HandCtrlMode = HAND_OUT_CTRL;
		ui.m_pHandRunBtn->setText("Run");
	}
}

void CyberSystem::SendHandCmd()
{
	ADDCOUNT(m_HandDataCount, 99999);
	if (ui.m_pNormalRd->isChecked()){
		m_RobonautControl.setHandMode(RobonautControl::Impedance);
	}
	else if (ui.m_pSoftRd->isCheckable()){
		m_RobonautControl.setHandMode(RobonautControl::Soft);
	}
	else{
		m_RobonautControl.setHandMode(RobonautControl::ZeroForce);
	}

	if (m_HandCtrlMode == OUT_CTRL)
	{	
		CHandData RHandData, LHandData;
		for(int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				RHandData.joint[i][j] = 0;
				LHandData.joint[i][j] = 0;
			}
		}
		bool ret = m_RobonautControl.SendHandMsg(RHandData, LHandData, m_HandDataCount);
		if (ret = false)
		{
			m_CmdStr += "Hand Data Send Error!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	} 
	else
	{
		CHandData RHandData, LHandData;
		for(int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				RHandData.joint[i][j] = m_RGloRealData[i][j];
				LHandData.joint[i][j] = m_LGloRealData[i][j];
			}
		}
		bool ret = m_RobonautControl.SendHandMsg(RHandData, LHandData, m_HandDataCount);

		if (ret = false)
		{
			m_CmdStr += "Hand Data Send Error!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	}
}

void CyberSystem::DisHandData()
{
	std::ostringstream r_hand_stream;
	r_hand_stream << "********* Hand Receive Data ********" << std::endl;
	r_hand_stream << std::fixed << std::left;
	r_hand_stream.precision(3);

	r_hand_stream.width(8);
	r_hand_stream << "Thumb";
	r_hand_stream.width(8);
	r_hand_stream << "Index";
	r_hand_stream.width(8);
	r_hand_stream << "Middle";
	r_hand_stream.width(8);
	r_hand_stream << "Ring";
	r_hand_stream.width(8);
	r_hand_stream << "Little" << std::endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			r_hand_stream.width(8);
			r_hand_stream << m_RHandRecvJoint[j][i];
		}
		r_hand_stream << std::endl;
	}
	r_hand_stream << "Torque: " << std::endl;
	for (int i = 0; i < 5; ++i)
	{
		r_hand_stream.width(8);
		r_hand_stream << m_RHandRecvTorque[i][0];
	}
	r_hand_stream << std::endl;

	std::string r_hand_str = r_hand_stream.str();
	m_HandStr = m_HandStr.fromStdString(r_hand_str);

	m_DisDataMutex.lock();
	m_RoboTotalStr = m_RoboStr + m_HandStr;
	emit InsertRoboText(m_RoboTotalStr);
	m_DisDataMutex.unlock();
}

void CyberSystem::RecvHandSensor()
{
	bool ret = m_RobonautControl.RecvHandMsg(m_RHandData, m_LHandData);
	if (ret = true)
	{
		for (int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				m_RHandRecvJoint[i][j] = m_RHandData.joint[i][j];
				m_RHandRecvTorque[i][j] = m_RHandData.torque[i][j];
				m_LHandRecvJoint[i][j] = m_LHandData.joint[i][j];
				m_LHandRecvTorque[i][j] = m_LHandData.torque[i][j];
			}
		}

		double Force[5];
		Force[0] = (m_RHandData.torque[0][0]+m_RHandData.torque[0][1])*1.0;
		Force[1] = (m_RHandData.torque[1][0]+m_RHandData.torque[1][1])*0.3;
		Force[2] = (m_RHandData.torque[2][0]+m_RHandData.torque[2][1])*0.3;
		Force[3] = (m_RHandData.torque[3][0]+m_RHandData.torque[3][1])*0.3;
		Force[4] = (m_RHandData.torque[4][0]+m_RHandData.torque[4][1])*0.5;

		for(int i=0; i<5; i++)
		{
			if(Force[i] >= 1.0)
				Force[i] = 1.0;
		
			if(Force[i] <= 0.0)
				Force[i] = 0.0;
		}

		m_CyberStation.setGraspForce(Force);

		DisHandData();
	} 
	else
	{
		m_CmdStr += "Hand Receive Error!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
	}
}

void PASCAL TimerProcHandSend(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	g_pCyberSys->SendHandCmd();
}
void PASCAL TimerProcHandRecv(UINT wTimerID, UINT msg,DWORD dwUser,DWORD dwl,DWORD dw2)
{
	g_pCyberSys->RecvHandSensor();
}

//*********************** Kinetics Calculate ***********************//
void CyberSystem::InitKine()
{
	// joint limitation
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.0472, 3.1416));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.9199, 1.5708));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.0944, 2.0944));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.2689, 0.1745));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.0944, 2.0944));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.5708, 1.5708));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.0944, 2.0944));

	// Kine7是求解正逆运动学的基础类
	m_Kine = rpp::kine::Kine7<double>(400, 285, 300, 0, joint_limits);
	// SingularHandler处理逆运动学计算中的奇异问题
	sh = rpp::kine::SingularityHandler<double>(joint_limits);
}


Eigen::Matrix<double, 7, 1> CyberSystem::CalKine(const Eigen::Matrix<double, 4, 4> &T, double &last_arm_angle, 
	Eigen::Matrix<double, 7, 1> &last_joint_angle)
{
	Eigen::Matrix<double, 7, 1> ref_angle;
	ref_angle << (-1.0472 + 3.1416)/2, (-1.9199 + 1.5708)/2, (-2.0944 + 2.0944)/2, 
		(-2.2689 + 0.1745)/2, (-2.0944 + 2.0944)/2, (-1.5708 + 1.5708)/2, (-2.0944 + 2.0944)/2;
//	ref_angle = last_joint_angle;

	// 判断有无自运动，如果没有，直接返回上一次的值
	auto self_motions = m_Kine.inverse(T);
	if (self_motions.empty())
	{
		m_CmdStr += "No self_motion!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		return last_joint_angle;
	}

	// 选择一个肘关节角度在关节限位内的自运动（< 0）
	auto it_self_motions = self_motions.begin();
	for (; it_self_motions != self_motions.end(); ++it_self_motions)
	{
		if (joint_limits[3].contains(it_self_motions->elbow_joint()))
		{
			break;
		}
	}

	// 取得该自运动可能的臂角范围
	auto arm_angle_ranges = it_self_motions->arm_angle_range();

	Eigen::Matrix<double, 7, 1> jo;
	int jo_count = 0;
	for (auto range = arm_angle_ranges.begin(); range != arm_angle_ranges.end(); ++range)
	{
		auto arm_angle = (range->lower() + range->upper())/2.0;

		auto joints = it_self_motions->get_joints(arm_angle, sh);
		// 遍历所有臂角可能取得的关节角度，选择和上一次最接近的角度
		for (auto it_joints = joints.begin(); it_joints != joints.end(); ++it_joints)
		{
			auto q = *(it_joints);

			// 解的合理性判断
			auto q_valid = true;
			for (int i = 0; i < 7; ++i)
			{
				q_valid = q_valid && joint_limits[i].contains(q[i]);
			}
			if (!q_valid)
			{
				continue;
			}
			auto TT = m_Kine.forward(q);
			if ((TT - T).norm() > 0.05)
			{
				continue;
			}
			else{
				// 如果jo_count > 0， 则至少存在一个解
				jo_count++;
			}

			// 判断是否第一次进入
			if (jo_count == 1)
			{
				jo = q;
			} 
			else
			{
				if ((q - ref_angle).norm() < (jo - ref_angle).norm())
				{
					jo = q;
				}
			}


		}
	}

	// 存在解
	if (jo_count > 0)
	{
		last_joint_angle = jo;
		return jo;
	} 
	else
	{
		m_CmdStr += "Self_motions have no Reasonable Solution!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		return last_joint_angle;
	}
}



//calculate inverse kinetics
//Eigen::Matrix<double, 7, 1> CyberSystem::CalKine(const Eigen::Matrix<double, 4, 4> &T, double &last_arm_angle, 
//												 Eigen::Matrix<double, 7, 1> &last_joint_angle)
//{
//	auto self_motions = m_Kine.inverse(T);
//
//	// if there is no reasonable solution
//	if (self_motions.empty())
//	{
//		m_CmdStr = "NO Solution!!!\r\n" + m_CmdStr ;
//		emit InsertCmdStr(m_CmdStr);
//
//		return last_joint_angle;
//	}
//
//	// elbow angle contains in the limitation, and 
//	auto it_self_motion = self_motions.begin();
//	for (auto it = it_self_motion + 1; it != self_motions.end(); ++it)
//	{
//		if (joint_limits[3].contains(it->elbow_joint()))
//		{
//			if (abs(it->elbow_joint() - last_joint_angle(3, 0)) < 
//				abs(it_self_motion->elbow_joint() - last_joint_angle(3, 0)))
//			{
//				it_self_motion = it;
//			}
//		}
//	}
//
//	auto it_self_motion = self_motions.begin();
//	auto tmp = it_self_motion->arm_angle_range();
//	auto it_arm_angle = tmp.begin();
//	auto ave = (it_arm_angle->lower() + it_arm_angle->upper())/2.0;
//
//	for (auto it_self_motion_tmp = it_self_motion; it_self_motion_tmp != self_motions.end(); ++it_self_motion_tmp)
//	{
//		auto it_arm_angle_tmp = it_self_motion_tmp->arm_angle_range().begin();
//		auto tmp = it_self_motion_tmp->arm_angle_range().end();
//		auto tmp_2 = it_self_motion_tmp->arm_angle_range();
//		++it_arm_angle_tmp;
//
//		for (auto it_arm_angle_tmp = it_self_motion_tmp->arm_angle_range().begin(); it_arm_angle_tmp != it_self_motion_tmp->arm_angle_range().end(); ++it_arm_angle_tmp)
//		{
//			auto ave_tmp = (it_arm_angle_tmp->lower() + it_arm_angle_tmp->upper())/2.0;
//			if (abs(ave - last_arm_angle) > abs(ave_tmp - last_arm_angle))
//			{
//				it_self_motion = it_self_motion_tmp;
//				it_arm_angle = it_arm_angle_tmp;
//				ave = ave_tmp;
//			}
//		}
//	}
//	last_arm_angle = ave;
//
//
//
//	// 
//	auto arm_angle_ranges = it_self_motion->arm_angle_range();
//
//	// TODO(CJH): out of workspace
//	if (arm_angle_ranges.empty())
//	{
//		return m_last_joint_angle;
//	}
//
//	auto it_arm_angle = arm_angle_ranges.begin();
//	auto ave = (it_arm_angle->lower() + it_arm_angle->upper())/2.0;
//
//	// choose the arm angle that is most similar to the last one
//	for (auto range = it_arm_angle + 1; range != arm_angle_ranges.end(); ++range)
//	{
//		auto tmp_ave = (range->lower() + range->upper())/2.0;
//		if (abs(tmp_ave - last_arm_angle) < abs(ave - last_arm_angle))
//		{
//			it_arm_angle = range;
//			ave = tmp_ave;
//		}
//	}
//
//	auto joints = it_self_motion->get_joints(ave, sh);
//
//	// choose a reasonable joint angle
//	auto it_joint = joints.begin();
//	for (bool flag = true; (it_joint != joints.end()) && (flag == true); ++it_joint)
//	{
//		auto q = *(it_joint);
//		auto q_valid = true;
//		for (int i = 0; i < 7; ++i)
//		{
//			q_valid = q_valid && joint_limits[i].contains(q[i]);
//		}
//
//		if (q_valid)
//		{
//			auto TT = m_Kine.forward(q);
//
//			// first reasonable joint angle
//			if ((TT - T).norm() < 0.05)
//			{
//				flag = false;
//				for (auto it = it_joint + 1; it != joints.end(); ++it)
//				{
//					auto tmp_q = *(it);
//
//					auto tmp_q_valid = true;
//					for (int i = 0; i < 7; ++i)
//					{
//						tmp_q_valid = tmp_q_valid && joint_limits[i].contains(tmp_q[i]);
//					}
//
//					if (tmp_q_valid)
//					{
//						auto tmp_TT = m_Kine.forward(tmp_q);
//
//						if((tmp_TT - T).norm() < 1)
//						{
//							// 
//							if ((last_joint_angle - tmp_q).norm() < (last_joint_angle - q).norm())
//							{
//								it_joint = it;
//								q = tmp_q;
//							}
//						}
//					}
//				}
//				last_joint_angle = q;
//				return q;
//			}
//		}
//	}
//
//	// if there is no reasonable joint angle
//	m_CmdStr = "NO Solution!!!\r\n" + m_CmdStr;
//	emit InsertCmdStr(m_CmdStr);
//
//	return last_joint_angle;
//
//}