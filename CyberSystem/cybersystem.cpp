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
	m_bGloDisReal = false;
	m_bGloCaliFin = false;

	// CyberTracker data display logical control
	// whether to display tracker data
	m_bDisTraData = false;	
	// determine whether side is over
	m_bRTraCaliFini = false;
	m_bLTraCaliFini = false;
	// choose which tracker data to display
	m_bRTraDisReal = false;
	m_bLTraDisReal = false;
	// Connection Logical Control
	m_bRoboConn = false;
	m_bConsimuConn = false;
	// Control Mode
	m_CtrlMode = OUT_CTRL;

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

	// Command Browser initialization
	m_CommandString = '\0';


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

	connect(ui.m_pOpenCali, SIGNAL(clicked()), this, SLOT(LoadCaliData()));
	connect(ui.m_pSaveCali, SIGNAL(clicked()), this, SLOT(SaveCaliData()));

	// signals and slots for glove calibration
	connect(ui.m_pGlStartBtn, SIGNAL(clicked()), this, SLOT(InitGloveCali()));
	connect(this, SIGNAL(InsertGloText(const QString &)), ui.m_pTraDataDisBs, SLOT(setText(const QString &))); // display glove data when calibration begin
	connect(this, SIGNAL(InsertRoboText(const QString &)), ui.m_pRoboDataDisBs, SLOT(setText(const QString &)));
	connect(this, SIGNAL(InsertCmdStr(const QString &)), ui.m_pCommadBs, SLOT(setText(const QString &)));
	connect(ui.m_pFinshBtn, SIGNAL(clicked()), this, SLOT(FinGloveCali()));
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


	// signals and slots for 5 hand control
	connect(ui.m_pInitHandBtn, SIGNAL(clicked()), this, SLOT(InitHand()));

	connect(ui.m_pPosRdBtn, SIGNAL(clicked()), this, SLOT(PositionMode()));
	connect(ui.m_pImpRdBtn, SIGNAL(clicked()), this, SLOT(ImpedanceMode()));
	connect(ui.m_pResRdBtn, SIGNAL(clicked()), this, SLOT(ResetMode()));

	connect(ui.m_pRunHandBtn, SIGNAL(clicked()), this, SLOT(HandCtrl()));

	
// 	// signals and slots for timer
// 	connect(m_pGloDispTimer, SIGNAL(timeout()), this, SLOT(DisData()));
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
	std::string err_str;

	m_CmdStr += "Connecting System......";
	emit InsertCmdStr(m_CmdStr);

	m_CyberStation.RHandConn();
	m_CyberStation.LHandConn();
	
	
	m_RTraContr = m_CyberStation.RTraConn(err_str);
	
	m_CyberStation.LTraConn();
	m_RGloContr = true;
	m_LGloContr = true;

	m_LTraContr = true;

	ui.m_pGlStartBtn->setEnabled(true);
	ui.m_pTraStartBtn->setEnabled(true);

}

void CyberSystem::InitRHand()
{
	m_CyberStation.RHandConn();
	m_RGloContr = true;
	ui.m_pGlStartBtn->setEnabled(true);
}

void CyberSystem::InitLHand()
{
	m_CyberStation.LHandConn();
	m_LGloContr = true;
	ui.m_pGlStartBtn->setEnabled(true);
}

// TODO(CJH): Add judgement
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
		emit InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::InitLTracker()
{
	m_CyberStation.LTraConn();
	m_LTraContr = true;
	ui.m_pTraStartBtn->setEnabled(true);
}

//*********************** Data Display Control ***********************//
// TODO(CJH): Only Display Tracker Data
void CyberSystem::DisTraData()
{
	while(m_DisThread.m_bDisThreadStop == false)
	{
		QString RGloStr = NULL;
		QString LGloStr = NULL;
		QString RTraStr = NULL;
		QString LTraStr = NULL;
		// indicate that CyberGlove device is connected
		if (m_bDisGloData == true)
		{
			// right CyberGlove is connected
			if (m_RGloContr == true)
			{
				m_CyberStation.GetRGloData();
				m_CyberStation.RealGloData(m_bGloCaliFin);
				RGloStr = m_CyberStation.GloDisData(m_bGloDisReal);
			}
			// left CyberGlove is connected
			else if(m_LGloContr == true)
			{
				m_CyberStation.GetLGloData();
				m_CyberStation.RealGloData(m_bGloCaliFin);
				LGloStr = m_CyberStation.GloDisData(m_bGloDisReal);
			}
		}
		
		// indicated that CyberTracker device has connected
		if (m_bDisTraData == true)
		{
			if (m_RTraContr == true)
			{
				// TODO(CJH): Delete
//				// old interface
//				m_CyberStation.GetRTraData();			
//				RTraStr = m_CyberStation.RTraDisData(m_bRTraDisReal);

				if (m_bRTraDisReal == true)
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
				else
				{
					m_RTraRawMat = m_CyberStation.GetRRawTraData();
					RTraStr = QString("rightTracker_RawTransMat:\r\n%1 %2 %3 %4\r\n%5 %6 %7 %8\r\n%9 %10 %11 %12\r\n%13 %14 %15 %16\r\n")
						.arg(m_RTraRawMat(0,0)).arg(m_RTraRawMat(0,1)).arg(m_RTraRawMat(0,2)).arg(m_RTraRawMat(0,3))
						.arg(m_RTraRawMat(1,0)).arg(m_RTraRawMat(1,1)).arg(m_RTraRawMat(1,2)).arg(m_RTraRawMat(1,3))
						.arg(m_RTraRawMat(2,0)).arg(m_RTraRawMat(2,1)).arg(m_RTraRawMat(2,2)).arg(m_RTraRawMat(2,3))
						.arg(m_RTraRawMat(3,0)).arg(m_RTraRawMat(3,1)).arg(m_RTraRawMat(3,2)).arg(m_RTraRawMat(3,3));


// 					//TODO(CJH)
// 					m_file << "raw mat" << std::endl;
// 					for (int i = 0; i < 4; i++)
// 					{
// 						for (int j = 0; j < 4; j++)
// 						{
// 							m_file << m_RTraRawMat(i, j) << " ";
// 						}
// 						m_file << std::endl;
// 					}
// 					m_file << std::endl;
				}
			}

			// TODO(CJH): without writing left tracker
			if (m_LTraContr == true)
			{
//				m_CyberStation.GetLTraData();
				LTraStr = m_CyberStation.LTraDisData(m_bLTraDisReal);
			}
		}
		QString Str = RGloStr + LGloStr + RTraStr + LTraStr;
		emit InsertGloText(Str);	
		m_DisThread.msleep(250);
	}
	// ensure that you will get into the thread next time
	m_DisThread.m_bDisThreadStop = false;
}

void CyberSystem::DisGloData()
{

}


//*********************** CyberGlove Calibration Options ***********************//
// start display thread
void CyberSystem::InitGloveCali()
{
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Calibration?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
	{  
		//m_pGloDispTimer->start(200); // use timer to display glove raw data
		// new thread to display raw data
		m_bDisGloData = true;
		m_DisThread.start();
		ui.m_pGesBtn_one->setEnabled(true);
	}  
	else{}  
}



// use right glove for calibration
void CyberSystem::GesOneData()
{
	m_CyberStation.GetGesOneData();
	ui.m_pGesBtn_two->setEnabled(true);
}

void CyberSystem::GesTwoData()
{
	m_CyberStation.GetGesTwoData();
	ui.m_pGesBtn_three->setEnabled(true);
}

void CyberSystem::GesThrData()
{
	m_CyberStation.GetGesThrData();
	ui.m_pGesBtn_four->setEnabled(true);
}

void CyberSystem::GesFourData()
{
	m_CyberStation.GetGesFourData();
	m_CyberStation.CalGloCoef();
	ui.m_pFinshBtn->setEnabled(true);
//	m_pGloDispTimer->stop();
	
	// stop the thread
	m_DisThread.stop();
	m_DisThread.wait();	
}

void CyberSystem::FinGloveCali()
{
//	m_pGloDispTimer->start(200);
	// control real data display and calibration finish
	m_bGloDisReal = true;
	m_bGloCaliFin = true;
	
	// thread to display real data
	m_DisThread.start();

	ui.m_pInitHandBtn->setEnabled(true);
}


//*********************** CyberTracker Calibration Options ***********************//
// start display
void CyberSystem::InitTraCali()
{
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Tracker?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
	{  
		// new thread to display raw data
		m_bDisTraData = true;
		m_DisThread.start();

		m_CmdStr += "Start Display......OK!\r\n";
		emit InsertCmdStr(m_CmdStr);
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

	// if two sides are calibrated, display real data and pop-up a dialogue box
// 	if ((m_bRTraCaliFini && m_bLTraCaliFini) == true)
// 	{
// 		QMessageBox::about(NULL, "About", "CyberTracker calibration is finished");
// 	}
	if (m_bRTraCaliFini == true)
	{
		QMessageBox::about(NULL, "About", "CyberTracker calibration is finished");
		m_bRTraDisReal = true;

		ui.m_pSimuConnBtn->setEnabled(true);
		ui.m_pRoboConnBtn->setEnabled(true);
	}


	// initialize kinetics

}

void CyberSystem::LoadCaliData()
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


		m_CyberStation.UpdataCaliCoef(TransMat, RotMat);
		m_bRTraDisReal = true;
		m_bRTraCaliFini = true;
		m_CmdStr += "Load Config Success!!!";
		InsertCmdStr(m_CmdStr);
	}
	else{
		m_CmdStr += "Config File is Empty!!!";
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
		send_out_str.width(10);
		send_out_str << g_RobotCmdDeg.rightArmJoint[i];
	}
	send_out_str << std::endl;
	send_out_str << "Left Arm Joints: " << std::endl;
	for ( int i = 0; i < 7; ++i)
	{
		send_out_str.width(10);
		send_out_str << g_RobotCmdDeg.leftArmJoint[i];
	}
	send_out_str << std::endl;

	sensor_out_str << "******** Receive Data ********" << std::endl;
	sensor_out_str << std::fixed << std::left;
	sensor_out_str.precision(3);

	sensor_out_str << "Right Arm Joints: " << std::endl;
	for (int i = 0; i < 7; ++i)
	{
		sensor_out_str.width(10);
		sensor_out_str << g_RobotSensorDeg.rightArmJoint[i];
	}
	sensor_out_str << std::endl;
	sensor_out_str << "Left Arm Joints: " << std::endl;
	for ( int i = 0; i < 7; ++i)
	{
		sensor_out_str.width(10);
		sensor_out_str << g_RobotSensorDeg.leftArmJoint[i];
	}
	sensor_out_str << std::endl;

	QString qstr;
	// 没有连接宇航员，不现实传感器数据
	if (m_bRoboConn == false && m_bConsimuConn == true){
		qstr = qstr.fromStdString(send_out_str.str());
	}
	// 连接了宇航员，显示发送到数据和传感器数据
	else if (m_bRoboConn == true){
		qstr = qstr.fromStdString((send_out_str.str() + sensor_out_str.str()));
	}
	emit InsertRoboText(qstr);
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
			m_CmdStr += "Failed!!!";
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
		m_CmdStr += "Calibrate Tracker First!!!";
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
		m_CmdStr = "Sending Joints Data!!!\r\n" + m_CmdStr;
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
		m_CmdStr = "Sending Pose Data!!!\r\n" + m_CmdStr;
		emit InsertCmdStr(m_CmdStr);

		QString RPosBuf[7],LPosBuf[7];
		RPosBuf[0] = ui.m_pRPosXCmd->text();
		RPosBuf[1] = ui.m_pRPosYCmd->text();
		RPosBuf[2] = ui.m_pRPosZCmd->text();
		RPosBuf[3] = ui.m_pROri1Cmd->text();
		RPosBuf[4] = ui.m_pROri2Cmd->text();
		RPosBuf[5] = ui.m_pROri3Cmd->text();
		RPosBuf[6] = ui.m_pROri4Cmd->text();
		LPosBuf[0] = ui.m_pLPosXCmd->text();
		LPosBuf[1] = ui.m_pLPosYCmd->text();
		LPosBuf[2] = ui.m_pLPosZCmd->text();
		LPosBuf[3] = ui.m_pLOri1Cmd->text();
		LPosBuf[4] = ui.m_pLOri2Cmd->text();
		LPosBuf[5] = ui.m_pLOri3Cmd->text();
		LPosBuf[6] = ui.m_pLOri4Cmd->text();

		for (int i = 0; i<7; i++)
		{
			m_rightArmPos[i] = RPosBuf[i].toDouble();
			m_leftArmPos[i] = LPosBuf[i].toDouble();
		}

		// unit quaternion
		/*********************
		n = m_rightArmPos[3];
		ex = m_rightArmPos[4];
		ey = m_rightArmPos[5];
		ez = m_rightArmPos[6];
		*********************/
		// calculate inverse kinetics
		Eigen::Matrix<double, 4, 4> T;
		// rotate matrix
		T(0,0) = 2*(pow(m_rightArmPos[3], 2) + pow(m_rightArmPos[4], 2)) - 1;
		T(0,1) = 2*(m_rightArmPos[4]*m_rightArmPos[5] - m_rightArmPos[3]*m_rightArmPos[6]);
		T(0,2) = 2*(m_rightArmPos[4]*m_rightArmPos[6] + m_rightArmPos[3]*m_rightArmPos[5]);
		T(1,0) = 2*(m_rightArmPos[4]*m_rightArmPos[5] + m_rightArmPos[3]*m_rightArmPos[6]);
		T(1,1) = 2*(pow(m_rightArmPos[3], 2) + pow(m_rightArmPos[5], 2)) - 1;
		T(1,2) = 2*(m_rightArmPos[5]*m_rightArmPos[6] - m_rightArmPos[3]*m_rightArmPos[4]);
		T(2,0) = 2*(m_rightArmPos[4]*m_rightArmPos[6] - m_rightArmPos[3]*m_rightArmPos[5]);
		T(2,1) = 2*(m_rightArmPos[5]*m_rightArmPos[6] + m_rightArmPos[3]*m_rightArmPos[4]);
		T(2,2) = 2*(pow(m_rightArmPos[3], 2) + pow(m_rightArmPos[6], 2)) - 1;
		// transpos matrix
		T(0,3) = m_rightArmPos[0];
		T(1,3) = m_rightArmPos[1];
		T(2,3) = m_rightArmPos[2];
		// vision matrix
		T(3,0) = 0;
		T(3,1) = 0;
		T(3,2) = 0;
		T(3,3) = 1;

		Eigen::Matrix<double, 7, 1> q = CalKine(T, m_last_arm_angle, m_last_joint_angle);

		// final data
		for (int i = 0; i<7; i++)
		{		
			g_rightArmJointBuf[i] = DEG2ANG(q(i));
		}
	}
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


//*********************** Control Command Display ***********************//

void CyberSystem::InitHand()
{
	m_RobonautControl.HandInit();
	ui.m_pPosRdBtn->setEnabled(true);
	ui.m_pImpRdBtn->setEnabled(true);
	ui.m_pResRdBtn->setEnabled(true);
	ui.m_pRunHandBtn->setEnabled(true);
}

void CyberSystem::PositionMode() 
{
	m_RobonautControl.setPosMode();
}

void CyberSystem::ImpedanceMode() 
{
	m_RobonautControl.setImpMode();
}

void CyberSystem::ResetMode() 
{
	m_RobonautControl.setResMode();
}

void CyberSystem::HandCtrl()
{
	if (ui.m_pRunHandBtn->text() == "Run")
	{
		m_RobonautControl.m_bGloveControl = TRUE;
		m_RobonautControl.HandControl();
		ui.m_pRunHandBtn->setText("Stop");

		return;
	}

	if (ui.m_pRunHandBtn->text() == "Stop")
	{
		m_RobonautControl.m_bGloveControl = FALSE;
		ui.m_pRunHandBtn->setText("Run");

		return;
	}
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
		m_CmdStr = "No self_motion!!!\r\n" + m_CmdStr;
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
		m_CmdStr = "Self_motions have no Reasonable Solution!!!\r\n" + m_CmdStr;
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