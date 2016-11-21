#include "cybersystem.h"
#include "qregexp.h"
#include <iostream>
#include <sstream>

#include "kine7.hpp"

#define DEG2ANG(x) x*180/3.14
#define	ANG2DEG(x) x*3.14/180

#define INIT_JOINT1 -39.1323
#define INIT_JOINT2 -11.2267
#define INIT_JOINT3 96.994
#define INIT_JOINT4 -102.1614
#define INIT_JOINT5 -82.8184
#define INIT_JOINT6 41.125
#define INIT_JOINT7 -27.5551



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
	m_RTraRealMat << 1, 0, 0, -154,
					  0, 0, -1, -490,
					  0, 1, 0, 360,
					  0, 0, 0, 1;

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
	connect(ui.m_pCyberStopBtn, SIGNAL(clicked()), this, SLOT(CyberStop()));

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
	timeKillEvent(RecvTimerId);


	m_DisThread.stop();
	m_DisThread.wait();	

	m_fRRealMat.clear();
	m_fRRealMat.close();
}


//*********************** Initialization options ***********************//
void CyberSystem::InitSystem()
{
	m_CyberStation.RHandConn();
	m_CyberStation.LHandConn();
	m_CyberStation.RTraConn();
	m_CyberStation.LTraConn();
	m_RGloContr = true;
	m_LGloContr = true;
	m_RTraContr = true;
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
	m_CmdStr += "Connecting Right Tracker......";
	emit InsertCmdStr(m_CmdStr);

	m_CyberStation.RTraConn();
	m_RTraContr = true;

	m_CmdStr += "OK!\r\n";
	emit InsertCmdStr(m_CmdStr);

	ui.m_pTraStartBtn->setEnabled(true);
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


					// TODO(CJH)
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
		m_DisThread.msleep(100);
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
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Calibration?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
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

		m_CyberStation.CalTraCoef(true, XPos, YPos, ZPos, XOri, YOri, ZOri);
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

		m_CyberStation.CalTraCoef(false, XPos, YPos, ZPos, XOri, YOri, ZOri);
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
	if (m_bRoboConn == false && m_bConsimuConn == true){
		qstr = qstr.fromStdString(send_out_str.str());
	}
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
	else{
		QMessageBox::critical(NULL, "Error", "Out of Control!!!");
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
		m_CmdStr += "Connecting Robonaut......";
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
			QMessageBox::critical(NULL,"Error","Robo Connection Failed");
		}
	}

	else{
		// TODO(CJH): add bool
		// Disconnct function is not add
		bool ret = m_RobonautControl.DisConnRobo();
		if (ret == true)
		{
			m_bRoboConn = false;
			ui.m_pRoboConnBtn->setText("Conn Robonaut");
			ui.m_pJointCtrlBtn->setEnabled(false);
		}
	}
}

// TODO(CJH):Add Connection check and Calibration check
// 点击Cyber按钮，选择使用Cyber的模式
void CyberSystem::CyberCtrl()
{
	// 点击Cyber按钮
	if (m_CtrlMode == OUT_CTRL && (m_bRoboConn == true || m_bConsimuConn == true))
	{
		if (m_bRoboConn == true && m_bConsimuConn == false)
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

		ui.m_pCyberStartBtn->setEnabled(true);
		ui.m_pJointCtrlBtn->setEnabled(false);
		ui.m_pCyberCtrlBtn->setText("Stop");

	}
	// 点击Stop按钮
	else
	{
		ui.m_pCyberCtrlBtn->setText("Cyber");
		ui.m_pCyberStartBtn->setEnabled(false);
		ui.m_pCyberStopBtn->setEnabled(false);
		ui.m_pCyberCtrlBtn->setEnabled(true);

		m_CtrlMode = OUT_CTRL;
	}
}


// 点击Start按钮，开始控制
void CyberSystem::CyberCmd()
{
	m_CmdStr += "Initializing DataSend Timer......";
	emit InsertCmdStr(m_CmdStr);
	// Initialize Send Timer
	SendTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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
	RecvTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcRecvSensor, 0, TIME_PERIODIC);
	if (RecvTimerId != NULL)
	{
		m_CmdStr += "OK!\r\n";
		emit InsertCmdStr(m_CmdStr);

		ui.m_pCyberStopBtn->setEnabled(true);
		ui.m_pCyberStartBtn->setEnabled(false);
	} 
	else
	{
		m_CmdStr += "Failed!\r\n";
		emit InsertCmdStr(m_CmdStr);
		return;
	}
}

// 停止使用Cyber控制
void CyberSystem::CyberStop()
{
	m_CmdStr += "Stop Cyber Control!!!\r\n";
	emit InsertCmdStr(m_CmdStr);
	timeKillEvent(SendTimerId);
	timeKillEvent(RecvTimerId);

	ui.m_pCyberStartBtn->setEnabled(true);
	ui.m_pCyberStopBtn->setEnabled(false);
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

		SendTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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

		RecvTimerId = timeSetEvent(250, 1, (LPTIMECALLBACK)TimerProcRecvSensor, 0, TIME_PERIODIC);
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
		timeKillEvent(RecvTimerId);

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

	ui.m_pRJo1Line->setEnabled(flag);
	ui.m_pRJo2Line->setEnabled(flag);
	ui.m_pRJo3Line->setEnabled(flag);
	ui.m_pRJo4Line->setEnabled(flag);
	ui.m_pRJo5Line->setEnabled(flag);
	ui.m_pRJo6Line->setEnabled(flag);
	ui.m_pRJo7Line->setEnabled(flag);

	ui.m_pLJo0Sli->setEnabled(flag);
	ui.m_pLJo1Sli->setEnabled(flag);
	ui.m_pLJo2Sli->setEnabled(flag);
	ui.m_pLJo3Sli->setEnabled(flag);
	ui.m_pLJo4Sli->setEnabled(flag);
	ui.m_pLJo5Sli->setEnabled(flag);
	ui.m_pLJo6Sli->setEnabled(flag);

	ui.m_pRJo1Line->setEnabled(flag);
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

// calculate inverse kinetics
Eigen::Matrix<double, 7, 1> CyberSystem::CalKine(const Eigen::Matrix<double, 4, 4> &T, double &last_arm_angle, 
												 Eigen::Matrix<double, 7, 1> &last_joint_angle)
{
	auto self_motions = m_Kine.inverse(T);

	// if there is no reasonable solution
	if (self_motions.empty())
	{
		return last_joint_angle;
	}

	// elbow angle contains in the limitation, and 
	auto it_self_motion = self_motions.begin();
	for (auto it = it_self_motion + 1; it != self_motions.end(); ++it)
	{
		if (joint_limits[3].contains(it->elbow_joint()))
		{
			if (abs(it->elbow_joint() - last_joint_angle(3, 0)) < 
				abs(it_self_motion->elbow_joint() - last_joint_angle(3, 0)))
			{
				it_self_motion = it;
			}
		}
	}

	// 
	auto arm_angle_ranges = it_self_motion->arm_angle_range();

	// TODO(CJH): out of workspace
	if (arm_angle_ranges.empty())
	{
		return m_last_joint_angle;
	}

	auto it_arm_angle = arm_angle_ranges.begin();
	auto ave = (it_arm_angle->lower() + it_arm_angle->upper())/2.0;

	// choose the arm angle that is most similar to the last one
	for (auto range = it_arm_angle + 1; range != arm_angle_ranges.end(); ++range)
	{
		auto tmp_ave = (range->lower() + range->upper())/2.0;
		if (abs(tmp_ave - last_arm_angle) < abs(ave - last_arm_angle))
		{
			it_arm_angle = range;
			ave = tmp_ave;
		}
	}

	auto joints = it_self_motion->get_joints(ave, sh);

	// choose a reasonable joint angle
	auto it_joint = joints.begin();
	for (bool flag = true; (it_joint != joints.end()) && (flag == true); ++it_joint)
	{
		auto q = *(it_joint);
		auto q_valid = true;
		for (int i = 0; i < 7; ++i)
		{
			q_valid = q_valid && joint_limits[i].contains(q[i]);
		}

		if (q_valid)
		{
			auto TT = m_Kine.forward(q);

			// first reasonable joint angle
			if ((TT - T).norm() < 0.05)
			{
				flag = false;
				for (auto it = it_joint + 1; it != joints.end(); ++it)
				{
					auto tmp_q = *(it);

					auto tmp_q_valid = true;
					for (int i = 0; i < 7; ++i)
					{
						tmp_q_valid = tmp_q_valid && joint_limits[i].contains(tmp_q[i]);
					}

					if (tmp_q_valid)
					{
						auto tmp_TT = m_Kine.forward(tmp_q);

						if((tmp_TT - T).norm() < 0.05)
						{
							// 
							if ((last_joint_angle - tmp_q).norm() < (last_joint_angle - q).norm())
							{
								it_joint = it;
								q = tmp_q;
							}
						}
					}
				}
				last_joint_angle = q;
				return q;
			}
		}
	}

	// if there is no reasonable joint angle
	return last_joint_angle;
}