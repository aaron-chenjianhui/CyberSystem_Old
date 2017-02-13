#include "cybersystem.h"
#include "qregexp.h"
#include <iostream>
#include <sstream>

#include "kine7.hpp"


#define ArmDebug 0

#define DEG2ANG(x) (x*180/3.1415926535898)
#define	ANG2DEG(x) (x*3.1415926535898/180)
#define SGN(X) ((X>=0) ? 1:(-1))


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

// Define to Use Multimedia Timer
CyberSystem *g_pCyberSys;

// Communication with Master Controller
UINT SendTimerId = NULL;
void PASCAL TimerProcSendCmd(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
{
	g_pCyberSys->SendCmd();
	g_pCyberSys->RecvSensor();
	g_pCyberSys->DisRoboData();
}

// Send and Receive Hand Data
UINT HSendTimerId = NULL;
void PASCAL TimerProcHandSend(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
{
	g_pCyberSys->SendHandCmd();
}
UINT HRecvTimerId = NULL;
void PASCAL TimerProcHandRecv(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
{
	g_pCyberSys->RecvHandSensor();
}

// Receive Cameral Data
UINT VisionTimerId = NULL;
void PASCAL TimerProcVisionRecv(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
{
	g_pCyberSys->RecvVision();
}

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
	: QMainWindow(parent), m_DisThread(this), m_CamThread(this)/*, m_RobonautCtrlThread(this)*/
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

	//*********************** Initialize all data ***********************//
	// initialize cyber workstation connection status
	m_RGloConn = false;
	m_LGloConn = false;
	m_RTraContr = false;
	m_LTraContr = false;

	// CyberGlove data display logical control
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
	// Grasp Mode Initialzie
	m_bHandGrasp = false;

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

	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m_RHandSendData[i][j] = 0.0;
			m_LHandSendData[i][j] = 0.0;
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
	m_RJacoMat << 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;

	// Initialize kinetic calculate
	InitKine();
	m_bJacoIsInit = false;

	// Vision Control
	// Reference Pose
	RefEulerPose[0] = 0;		// Z Euler
	RefEulerPose[1] = 0;		// Y Euler
	RefEulerPose[2] = 0;		// X Euler
	RefEulerPose[3] = 0;		// X Position
	RefEulerPose[4] = 0;		// Y Position
	RefEulerPose[5] = 300;		// Z Position
	RefFinAppr = 250;
	// Cut-Off Pose
	EulerPoseCut[0] = ANG2DEG(5);		// Z Euler
	EulerPoseCut[1] = ANG2DEG(5);		// Y Euler
	EulerPoseCut[2] = ANG2DEG(5);		// X Euler
	EulerPoseCut[3] = 10;		// X Position
	EulerPoseCut[4] = 10;		// Y Position
	EulerPoseCut[5] = 10;		// Z Position
	ApprCut = 10;


	m_bConnCam = false;
	for (int i = 0; i < 7; ++i)
	{
		m_ViRecv[i] = 0.0;
	}
	m_bTrackFinish = false;
	m_bApprFlag = false;

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

	ui.m_pRJo1Spin->setRange(ROBO_J0_MIN, ROBO_J0_MAX);
	ui.m_pRJo2Spin->setRange(ROBO_J1_MIN, ROBO_J1_MAX);
	ui.m_pRJo3Spin->setRange(ROBO_J2_MIN, ROBO_J2_MAX);
	ui.m_pRJo4Spin->setRange(ROBO_J3_MIN, ROBO_J3_MAX);
	ui.m_pRJo5Spin->setRange(ROBO_J4_MIN, ROBO_J4_MAX);
	ui.m_pRJo6Spin->setRange(ROBO_J5_MIN, ROBO_J5_MAX);
	ui.m_pRJo7Spin->setRange(ROBO_J6_MIN, ROBO_J6_MAX);

	ui.m_pLJo1Spin->setRange(ROBO_J0_MIN, ROBO_J0_MAX);
	ui.m_pLJo2Spin->setRange(ROBO_J1_MIN, ROBO_J1_MAX);
	ui.m_pLJo3Spin->setRange(ROBO_J2_MIN, ROBO_J2_MAX);
	ui.m_pLJo4Spin->setRange(ROBO_J3_MIN, ROBO_J3_MAX);
	ui.m_pLJo5Spin->setRange(ROBO_J4_MIN, ROBO_J4_MAX);
	ui.m_pLJo6Spin->setRange(ROBO_J5_MIN, ROBO_J5_MAX);
	ui.m_pLJo7Spin->setRange(ROBO_J6_MIN, ROBO_J6_MAX);

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
	connect(ui.m_pSaveTraCali, SIGNAL(clicked()), this, SLOT(SaveTraCaliData()));
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

	// signals and slots for robonaut planning control
	connect(ui.m_pPlanCtrlBtn, SIGNAL(clicked()), this, SLOT(PlanCtrl()));
	connect(ui.m_pReadPoseBtn, SIGNAL(clicked()), this, SLOT(getPlanData()));
	connect(ui.m_pExecPlanBtn, SIGNAL(clicked()), this, SLOT(ExecPlan()));

	// signals and slots for vision control
	connect(ui.m_pViCtrlBtn, SIGNAL(clicked()), this, SLOT(VisionCtrl()));
	connect(ui.m_pViStartBtn, SIGNAL(clicked()), this, SLOT(VisionStart()));
	connect(ui.m_pViPauseBtn, SIGNAL(clicked()), this, SLOT(VisionStop()));
	connect(ui.m_pViApprBtn, SIGNAL(clicked()), this, SLOT(VisionAppr()));

	// signals and slots for vision serve
	connect(ui.m_pConnViBtn, SIGNAL(clicked()), this, SLOT(ConnVision()));

	// signals and slots for sending joint control data
	connect(ui.m_pSdDataBtn, SIGNAL(clicked()), this, SLOT(getSliData()));
	connect(ui.m_pUpdataBtn, SIGNAL(clicked()), this, SLOT(SliUpdata()));

	// signals and slots for slider and spin box
	connect(ui.m_pRJo0Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo1Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo2Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo3Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo4Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo5Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pRJo6Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));

	connect(ui.m_pLJo0Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo1Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo2Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo3Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo4Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo5Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));
	connect(ui.m_pLJo6Sli, SIGNAL(valueChanged(int)), this, SLOT(SliderToSpin()));

	connect(ui.m_pRJo1Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo2Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo3Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo4Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo5Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo6Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pRJo7Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));

	connect(ui.m_pLJo1Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo2Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo3Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo4Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo5Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo6Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));
	connect(ui.m_pLJo7Spin, SIGNAL(valueChanged(double)), this, SLOT(SpinToSlider()));


	// signals and slots for hand control
	connect(ui.m_pHandConnBtn, SIGNAL(clicked()), this, SLOT(InitHand()));
	connect(ui.m_pHandEnableBtn, SIGNAL(clicked()), this, SLOT(EnableHand()));
	connect(ui.m_pEmgBtn, SIGNAL(clicked()), this, SLOT(EmergHand()));

	// Cyber Control
	connect(ui.m_pHandCyberBtn, SIGNAL(clicked()), this, SLOT(CyberHandMode()));
	connect(ui.m_pHandRunBtn, SIGNAL(clicked()), this, SLOT(StartCyberHand()));
	connect(ui.m_pHandPauseBtn, SIGNAL(clicked()), this, SLOT(PauseCyberHand()));
	// Click Control
	connect(ui.m_pHandClickBtn, SIGNAL(clicked()), this, SLOT(ClickHandMode()));
	connect(ui.m_pHandReadBtn, SIGNAL(clicked()), this, SLOT(UpdateHandSpin()));
	connect(ui.m_pHandSendBtn, SIGNAL(clicked()), this, SLOT(SendHandSpin()));
	// Grasp Control
	connect(ui.m_pHandGraspBtn, SIGNAL(clicked()), this, SLOT(GraspHandMode()));
	connect(ui.m_pGraspBtn, SIGNAL(clicked()), this, SLOT(GraspHand()));
	connect(ui.m_pRlseBtn, SIGNAL(clicked()), this, SLOT(ReleaseHand()));
	//*********************** Signals and Slots ***********************//


}

CyberSystem::~CyberSystem()
{ 
	timeKillEvent(SendTimerId);
	SendTimerId = NULL;


	m_DisThread.stop();
	m_DisThread.wait();	

	m_CamThread.stop();
	m_CamThread.wait();

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

	m_RGloConn = m_CyberStation.RHandConn(r_glo_err_str);
	m_LGloConn = m_CyberStation.LHandConn(l_glo_err_str);
	m_RTraContr = m_CyberStation.RTraConn(r_tra_err_str);
	m_LTraContr = m_CyberStation.LTraConn(l_tra_err_str);

	if ((m_RTraContr && m_LTraContr && m_RGloConn && m_LGloConn) == true)
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
		if (m_RGloConn == false)
		{
			m_CmdStr += m_CmdStr.fromStdString(r_glo_err_str);
			emit InsertCmdStr(m_CmdStr);
		}
		if (m_LGloConn == false)
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

	m_RGloConn = m_CyberStation.RHandConn(err_str);

	if (m_RGloConn == true)
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
	m_LGloConn = m_CyberStation.LHandConn(err_str);

	if (m_LGloConn == true)
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

	// Connected Right Glove
	if (m_RGloConn == true)
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
	if (m_LGloConn == true)
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
		m_CmdStr += "Start Glove Calibration......OK!\r\n";
		emit InsertCmdStr(m_CmdStr);
		if (!(m_DisThread.isRunning()))
		{
			m_DisThread.start();
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
	mat4x4 TransMat;
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

void CyberSystem::SaveTraCaliData()
{
	mat4x4 TransMat;
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

	// 没有连接宇航员，不现实传感器数据
	if (m_bRoboConn == false && m_bConsimuConn == true){
		m_RoboStr = m_RoboStr.fromStdString(send_out_str.str());
	}
	// 连接了宇航员，显示发送到数据和传感器数据
	else if (m_bRoboConn == true){
		m_RoboStr = m_RoboStr.fromStdString((send_out_str.str() + sensor_out_str.str()));
	}

	//	m_DisDataMutex.lock();
	m_RoboTotalStr = m_RoboStr + m_HandStr;
	emit InsertRoboText(m_RoboTotalStr);
	//	m_DisDataMutex.unlock();
}





// 收数循环
void CyberSystem::RecvSensor()
{
	if (m_bRoboConn == true)
	{
		bool ret = m_RobonautControl.RecvRoboMsg();
	}
}

void CyberSystem::CyberCtrlMode()
{
	// joint angle
	mat7x1 q;
	if (m_bJacoIsInit == false)
	{
		q = CalKine(m_RTraRealMat, m_last_arm_angle, m_last_joint_angle);
		m_bJacoIsInit = true;
	} 
	else
	{
		TransToQuater(m_last_RTraRealMat, m_last_RTraRealQuat);
		TransToQuater(m_RTraRealMat, m_RTraRealQuat);
		bool ret = DiffKine(m_RTraRealQuat, m_last_RTraRealQuat, m_last_joint_angle, q);  

		m_last_RTraRealMat = m_RTraRealMat;
		m_last_joint_angle = q;
	}


	// g_rightArmJointBuf is the Command Buffer
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


void CyberSystem::ClickCtrlMode()
{
	// 切换到其他控制，首先将微分运动学置为初始状态
	m_bJacoIsInit = false;

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

void CyberSystem::OutCtrlMode()
{
	// 切换到其他控制，首先将微分运动学置为初始状态
	m_bJacoIsInit = false;

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

		timeKillEvent(SendTimerId);
		SendTimerId = NULL;
	}
}

void CyberSystem::PlanCtrlMode()
{
	if (m_plan_count < m_plan_count_max)
	{
		mat7x1 q;

		m_plan_count++;

		PosePlan(m_RPlanStartQuat, m_RPlanEndQuat, m_plan_count_max, m_plan_count, m_RTraRealQuat);
		bool ret = DiffKine(m_RTraRealQuat, m_last_RTraRealQuat, m_last_joint_angle, q);  

		m_last_RTraRealQuat = m_RTraRealQuat;	
		m_last_joint_angle = q;


		// g_rightArmJointBuf is the Command Buffer
		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = DEG2ANG(q(i));
			g_leftArmJointBuf[i] = 0;
		}
		if (m_CtrlMode == PLAN_CTRL_ALL)
		{
			bool consimu_ret = m_RobonautControl.SendConsimuMsg();
			bool robo_ret = m_RobonautControl.SendRoboMsg();
		}
		else if(m_CtrlMode == PLAN_CTRL_ROBO){
			bool ret = m_RobonautControl.SendRoboMsg();
		}
		else{
			bool ret = m_RobonautControl.SendConsimuMsg();
		}
	} 
	else
	{
		m_CmdStr += "Plan Finished!!!\r\n";
		InsertCmdStr(m_CmdStr);

		m_CtrlMode = PLAN_WAIT;
	}
}

void CyberSystem::VisionCtrlMode()
{
	// 以上一次发送角度作参考
	// 如果与传感器数据差别过大，使用传感器数据重新更新
	mat7x1 jo_err;
	for (int i = 0; i < 7; ++i)
	{
		jo_err(i) = last_cmd_jo(i) - ANG2DEG(g_RobotSensorDeg.rightArmJoint[i]);
	}
	if (jo_err.norm() > 0.35)
	{
		for (int i = 0; i < 7; ++i)
		{
			m_RArmJo(i) = g_RobotSensorDeg.rightArmJoint[i];
			m_RArmJo(i) = ANG2DEG(m_RArmJo(i));
		}
	} 
	else
	{
		m_RArmJo = last_cmd_jo;
	}

	//
	mat7x1 cmd_jo;

	CalDirectKine(m_RArmJo, m_ViTransNow);
	TransToQuater(m_ViTransNow, m_ViQuatNow);

	int vi_flag = (int) m_ViRecv[0];
	if (vi_flag == 1)		// 相机数据有效
	{
		double fabs_pose_err[6];
		for (int i = 0; i < 6; ++i)
		{
			fabs_pose_err[i] = fabs(m_ViEulerRecv[i+1] - RefEulerPose[i]);
		}
		bool OriTrackFin = (fabs_pose_err[0] < EulerPoseCut[0]) && (fabs_pose_err[1] < EulerPoseCut[1]) && (fabs_pose_err[2] < EulerPoseCut[2]);
		bool PoseTrackFin = (fabs_pose_err[3] < EulerPoseCut[3]) && (fabs_pose_err[4] < EulerPoseCut[4]) && (fabs_pose_err[5] < EulerPoseCut[5]);

		if (OriTrackFin == false)	// 如果完成没完成姿态的调整
		{
			double ViQuatNow[7], ViQuatNext[7];
			GetViNextOri(m_ViEulerRecv, ViQuatNow, ViQuatNext);

			bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);
		}
		else
		{
			if (PoseTrackFin == false)	// 完成了姿态的调整，但是没有完成位置的调整
			{
				GetViNextPos(m_ViRecv, m_ViQuatNow, m_ViQuatNext);

				bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);
			} 
			else	// 完成位置和姿态的调整
			{
				cmd_jo = m_RArmJo;
			}
		}
	} 
	else	// 相机传回无效数据
	{
		cmd_jo = last_cmd_jo;
	}

	last_cmd_jo = cmd_jo;

	for (int i = 0; i < 7; ++i)
	{
		g_rightArmJointBuf[i] = DEG2ANG(cmd_jo(i));
		g_leftArmJointBuf[i] = 0;
	}

	bool ret = m_RobonautControl.SendRoboMsg();
}

// 发数控制循环
void CyberSystem::SendCmd()
{
	// 使用Cyber控制循环
	if (m_CtrlMode == CYBER_CTRL_ALL || m_CtrlMode == CYBER_CTRL_ROBO || m_CtrlMode == CYBER_CTRL_SIMU)
	{
		// joint angle
		mat7x1 q;
		if (m_bJacoIsInit == false)
		{
			q = CalKine(m_RTraRealMat, m_last_arm_angle, m_last_joint_angle);
			m_bJacoIsInit = true;
		} 
		else
		{
			TransToQuater(m_last_RTraRealMat, m_last_RTraRealQuat);
			TransToQuater(m_RTraRealMat, m_RTraRealQuat);
			bool ret = DiffKine(m_RTraRealQuat, m_last_RTraRealQuat, m_last_joint_angle, q);  

			m_last_RTraRealMat = m_RTraRealMat;
			m_last_joint_angle = q;
		}


		// g_rightArmJointBuf is the Command Buffer
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
		// 切换到其他控制，首先将微分运动学置为初始状态
		m_bJacoIsInit = false;

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
	else if (m_CtrlMode == OUT_CTRL)
	{
		// 切换到其他控制，首先将微分运动学置为初始状态
		m_bJacoIsInit = false;

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

			timeKillEvent(SendTimerId);
			SendTimerId = NULL;
		}
	}

	else if (m_CtrlMode == PLAN_CTRL_ALL || m_CtrlMode == PLAN_CTRL_ROBO || m_CtrlMode == PLAN_CTRL_SIMU)
	{
		if (m_plan_count < m_plan_count_max)
		{
			mat7x1 q;

			m_plan_count++;

			PosePlan(m_RPlanStartQuat, m_RPlanEndQuat, m_plan_count_max, m_plan_count, m_RTraRealQuat);
			bool ret = DiffKine(m_RTraRealQuat, m_last_RTraRealQuat, m_last_joint_angle, q);  

			m_last_RTraRealQuat = m_RTraRealQuat;	
			m_last_joint_angle = q;


			// g_rightArmJointBuf is the Command Buffer
			for (int i = 0; i < 7; ++i)
			{
				g_rightArmJointBuf[i] = DEG2ANG(q(i));
				g_leftArmJointBuf[i] = 0;
			}
			if (m_CtrlMode == PLAN_CTRL_ALL)
			{
				bool consimu_ret = m_RobonautControl.SendConsimuMsg();
				bool robo_ret = m_RobonautControl.SendRoboMsg();
			}
			else if(m_CtrlMode == PLAN_CTRL_ROBO){
				bool ret = m_RobonautControl.SendRoboMsg();
			}
			else{
				bool ret = m_RobonautControl.SendConsimuMsg();
			}
		} 
		else
		{
			m_CmdStr += "Plan Finished!!!\r\n";
			InsertCmdStr(m_CmdStr);

			m_CtrlMode = PLAN_WAIT;
		}
	}
	else if (m_CtrlMode == PLAN_WAIT)
	{

	}

	else if (m_CtrlMode == VISION_CTRL)
	{
#if ArmDebug
		m_RArmJo = last_cmd_jo;

		mat7x1 cmd_jo;

		CalDirectKine(m_RArmJo, m_ViTransNow);
		TransToQuater(m_ViTransNow, m_ViQuatNow);

		double ViQuatNow[7], ViQuatNext[7];
		for (int i = 0; i < 7; ++i)
		{
			ViQuatNow[i] = m_ViQuatNow(i);
		}
		GetViNextOri(m_ViEulerRecv, ViQuatNow, ViQuatNext);

		for (int i = 0; i < 7; ++i)
		{
			m_ViQuatNext(i) = ViQuatNext[i];
		}

		bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);

		if (ret == false)		// 运动学计算错误
		{
			m_CmdStr += "Kinetics Calculation Occur Error!!!\r\n";
			InsertCmdStr(m_CmdStr);
			m_CtrlMode = OUT_CTRL;
			return;
		}

		last_cmd_jo = cmd_jo;

		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = DEG2ANG(cmd_jo(i));
			g_leftArmJointBuf[i] = 0;
		}

		// 虚拟仿真测试
		//bool send_ret = m_RobonautControl.SendConsimuMsg();

		// 机器人测试
		bool send_ret = m_RobonautControl.SendRoboMsg();
#else
		// 以上一次发送角度作参考
		// 如果与传感器数据差别过大，使用传感器数据重新更新
		mat7x1 jo_err;
		for (int i = 0; i < 7; ++i)
		{
			jo_err(i) = last_cmd_jo(i) - ANG2DEG(g_RobotSensorDeg.rightArmJoint[i]);
		}
		//if (jo_err.norm() > 0.5)
		if (0)
		{
			for (int i = 0; i < 7; ++i)
			{
				m_RArmJo(i) = g_RobotSensorDeg.rightArmJoint[i];
				m_RArmJo(i) = ANG2DEG(m_RArmJo(i));
			}
// 			m_CmdStr += "Revise Data!!!\r\n";
// 			InsertCmdStr(m_CmdStr);
		} 
		else
		{
			m_RArmJo = last_cmd_jo;
		}

		//
		mat7x1 cmd_jo;
		CalDirectKine(m_RArmJo, m_ViTransNow);
		TransToQuater(m_ViTransNow, m_ViQuatNow);


		int vi_flag = (int) m_ViRecv[0];
		if (vi_flag == 1)		// 相机数据有效
		{
			// 判断位姿是否调整完成
			double fabs_pose_err[6];
			for (int i = 0; i < 6; ++i)
			{
				fabs_pose_err[i] = fabs(m_ViEulerRecv[i+1] - RefEulerPose[i]);
			}
			double fabs_appr_err = fabs(m_ViEulerRecv[6] - RefFinAppr);
			bool OriTrackFin = (fabs_pose_err[0] < EulerPoseCut[0]) && (fabs_pose_err[1] < EulerPoseCut[1]) && (fabs_pose_err[2] < EulerPoseCut[2]);
			bool PoseTrackFin = (fabs_pose_err[3] < EulerPoseCut[3]) && (fabs_pose_err[4] < EulerPoseCut[4]) && (fabs_pose_err[5] < EulerPoseCut[5]);
			bool ApprTrackFin = (fabs_appr_err < ApprCut);

			//
			//OriTrackFin = false;
			//PoseTrackFin = true;


			if (OriTrackFin == false)	// 如果完成没完成姿态的调整
			{
				double ViQuatNow[7], ViQuatNext[7];
				for (int i = 0; i < 7; ++i)
				{
					ViQuatNow[i] = m_ViQuatNow(i);
				}
				GetViNextOri(m_ViEulerRecv, ViQuatNow, ViQuatNext);
				for (int i = 0; i < 7; ++i)
				{
					m_ViQuatNext(i) = ViQuatNext[i];
				}

				bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);
				if (ret == false)		// 运动学计算错误
				{
					m_CmdStr += "Kinetics Calculation Occur Error!!!\r\n";
					InsertCmdStr(m_CmdStr);
					m_CtrlMode = OUT_CTRL;
					return;
				}
			}
			else
			{
				if (PoseTrackFin == false)	// 完成了姿态的调整，但是没有完成位置的调整
				{
					GetViNextPos(m_ViRecv, m_ViQuatNow, m_ViQuatNext);

					bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);
					if (ret == false)		// 运动学计算错误
					{
						m_CmdStr += "Kinetics Calculation Occur Error!!!\r\n";
						InsertCmdStr(m_CmdStr);
						m_CtrlMode = OUT_CTRL;
						return;
					}
				} 
				else
				{
					ui.m_pViApprBtn->setEnabled(true);
					if (ApprTrackFin == false)	// 完成位置和姿态调整，未完成抓取接近
					{
						if (m_bApprFlag == true)	// 可以进行接近
						{
							double ViQuatNow[7], ViQuatNext[7];
							for (int i = 0; i < 7; ++i)
							{
								ViQuatNow[i] = m_ViQuatNow(i);
							}
							GetViNextAppr(m_ViEulerRecv, ViQuatNow, ViQuatNext);
							for (int i = 0; i < 7; ++i)
							{
								m_ViQuatNext(i) = ViQuatNext[i];
							}

							bool ret = DiffKine(m_ViQuatNext, m_ViQuatNow, m_RArmJo, cmd_jo);
							if (ret == false)		// 运动学计算错误
							{
								m_CmdStr += "Kinetics Calculation Occur Error!!!\r\n";
								InsertCmdStr(m_CmdStr);
								m_CtrlMode = OUT_CTRL;
								return;
							}
						} 
						else		// 不可以进行接近
						{
							cmd_jo = m_RArmJo;
						}
						
					}
					else	// 完成所有的调整
					{
						m_bApprFlag = false;
						cmd_jo = m_RArmJo;
					}
				}
			}
		} 
		else	// 相机传回无效数据
		{
			cmd_jo = last_cmd_jo;
		}

		last_cmd_jo = cmd_jo;

		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = DEG2ANG(cmd_jo(i));
			g_leftArmJointBuf[i] = 0;
		}

		bool ret = m_RobonautControl.SendRoboMsg();
		//bool send_ret = m_RobonautControl.SendConsimuMsg();
#endif
	}
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
			SendTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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

		if (ret_send == TIMERR_NOERROR)
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
			SendTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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
			mat4x4 T;

			QuaterToTrans(m_rightArmPos, T);
			mat7x1 q = CalKine(T, m_last_arm_angle, m_last_joint_angle);

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
			mat4x4 T;

			QuaterToTrans(m_rightArmPos, T);
			mat7x1 q = CalKine(T, m_last_arm_angle, m_last_joint_angle);

			// final data
			for (int i = 0; i<7; i++)
			{		
				g_leftArmJointBuf[i] = DEG2ANG(q(i));
			}
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

	ui.m_pRJo1Spin->setEnabled(flag);
	ui.m_pRJo2Spin->setEnabled(flag);
	ui.m_pRJo3Spin->setEnabled(flag);
	ui.m_pRJo4Spin->setEnabled(flag);
	ui.m_pRJo5Spin->setEnabled(flag);
	ui.m_pRJo6Spin->setEnabled(flag);
	ui.m_pRJo7Spin->setEnabled(flag);

	ui.m_pLJo1Spin->setEnabled(flag);
	ui.m_pLJo2Spin->setEnabled(flag);
	ui.m_pLJo3Spin->setEnabled(flag);
	ui.m_pLJo4Spin->setEnabled(flag);
	ui.m_pLJo5Spin->setEnabled(flag);
	ui.m_pLJo6Spin->setEnabled(flag);
	ui.m_pLJo7Spin->setEnabled(flag);
}


void CyberSystem::SliderToSpin()
{
	double RJoTmp[7],LJoTmp[7];
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

	double RJoAng[7],LJoAng[7];
	for (int i = 0; i < 7; ++i)
	{
		RJoAng[i] = RJoTmp[i]/100.000;
		LJoAng[i] = LJoTmp[i]/100.000;
	}

	ui.m_pRJo1Spin->setValue(RJoAng[0]);
	ui.m_pRJo2Spin->setValue(RJoAng[1]);
	ui.m_pRJo3Spin->setValue(RJoAng[2]);
	ui.m_pRJo4Spin->setValue(RJoAng[3]);
	ui.m_pRJo5Spin->setValue(RJoAng[4]);
	ui.m_pRJo6Spin->setValue(RJoAng[5]);
	ui.m_pRJo7Spin->setValue(RJoAng[6]);

	ui.m_pLJo1Spin->setValue(LJoAng[0]);
	ui.m_pLJo2Spin->setValue(LJoAng[1]);
	ui.m_pLJo3Spin->setValue(LJoAng[2]);
	ui.m_pLJo4Spin->setValue(LJoAng[3]);
	ui.m_pLJo5Spin->setValue(LJoAng[4]);
	ui.m_pLJo6Spin->setValue(LJoAng[5]);
	ui.m_pLJo7Spin->setValue(LJoAng[6]);
}

// TODO(CJH): Change editline data to slider data
void CyberSystem::SpinToSlider()
{
	double RJo[7],LJo[7];
	double RJoTmp[7],LJoTmp[7];

	RJoTmp[0] = ui.m_pRJo1Spin->value();
	RJoTmp[1] = ui.m_pRJo2Spin->value();
	RJoTmp[2] = ui.m_pRJo3Spin->value();
	RJoTmp[3] = ui.m_pRJo4Spin->value();
	RJoTmp[4] = ui.m_pRJo5Spin->value();
	RJoTmp[5] = ui.m_pRJo6Spin->value();
	RJoTmp[6] = ui.m_pRJo7Spin->value();

	LJoTmp[0] = ui.m_pLJo1Spin->value();
	LJoTmp[1] = ui.m_pLJo2Spin->value();
	LJoTmp[2] = ui.m_pLJo3Spin->value();
	LJoTmp[3] = ui.m_pLJo4Spin->value();
	LJoTmp[4] = ui.m_pLJo5Spin->value();
	LJoTmp[5] = ui.m_pLJo6Spin->value();
	LJoTmp[6] = ui.m_pLJo7Spin->value();

	for (int i = 0; i < 7; ++i)
	{
		RJo[i] = RJoTmp[i]*100.00;
		LJo[i] = LJoTmp[i]*100.00;
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
			HSendTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcHandSend, 0, TIME_PERIODIC);
		} 
		else
		{
			m_CmdStr += "Send Timer has Existed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}
		if (HRecvTimerId == NULL)
		{
			HRecvTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcHandRecv, 0, TIME_PERIODIC);
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
		ui.m_pEmgBtn->setEnabled(true);


		ui.m_pHandClickBtn->setEnabled(true);
		ui.m_pHandCyberBtn->setEnabled(true);
		ui.m_pHandGraspBtn->setEnabled(true);
	} 
	else
	{
		m_bHandEnable = false;
		m_RobonautControl.setHandEnable(m_bHandEnable);
		m_HandCtrlMode = HAND_OUT_CTRL;

		ui.m_pEmgBtn->setEnabled(false);

		ui.m_pHandClickBtn->setEnabled(false);
		ui.m_pHandCyberBtn->setEnabled(false);
		ui.m_pHandGraspBtn->setEnabled(false);
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

	// 切换控制模式
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
	else if(m_HandCtrlMode == HAND_CYBER_CTRL)
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
	else if(m_HandCtrlMode == HAND_CLICK_CTRL)
	{
		CHandData RHandData, LHandData;
		for(int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				RHandData.joint[i][j] = m_RHandSendData[i][j];
				LHandData.joint[i][j] = m_LHandSendData[i][j];
			}
		}
		bool ret = m_RobonautControl.SendHandMsg(RHandData, LHandData, m_HandDataCount);

		if (ret = false)
		{
			m_CmdStr += "Hand Data Send Error!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	}
	else if (m_HandCtrlMode == HAND_GRASP_CTRL)
	{
		CHandData RHandData, LHandData;

		if (m_bHandGrasp == true)		// 发出抓取命令
		{
			for(int i = 0; i < 5; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					RHandData.joint[i][j] = m_HandGraspJoint[i][j];
					LHandData.joint[i][j] = m_HandReleaseJoint[i][j];
				}
			}
		}
		else		//	发出释放命令
		{
			for(int i = 0; i < 5; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					RHandData.joint[i][j] = m_HandReleaseJoint[i][j];
					LHandData.joint[i][j] = m_HandReleaseJoint[i][j];
				}
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

		if (m_HandCtrlMode == HAND_CYBER_CTRL)		// 如果处在Cyber控制的模式下，施加反馈力
		{
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
		}

		DisHandData();
	} 
	else
	{
		m_CmdStr += "Hand Receive Error!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
	}
}

void CyberSystem::ClickHandMode()
{
	if (ui.m_pHandClickBtn->text() == "Click Mode")		// 进入Click Mode
	{
		// Initialize Click Hand Data
		for (int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				m_RHandSendData[i][j] = 0.0;
				m_LHandSendData[i][j] = 0.0;
			}
		}

		// 
		m_HandCtrlMode = HAND_CLICK_CTRL;

		ui.m_pHandReadBtn->setEnabled(true);
		ui.m_pHandSendBtn->setEnabled(true);

		ui.m_pHandCyberBtn->setEnabled(false);
		ui.m_pHandGraspBtn->setEnabled(false);

		ui.m_pHandClickBtn->setText("Stop Click");
	}
	else if (ui.m_pHandClickBtn->text() == "Stop Click")		// 离开Click Mode
	{
		m_HandCtrlMode = HAND_OUT_CTRL;

		ui.m_pHandReadBtn->setEnabled(false);
		ui.m_pHandSendBtn->setEnabled(false);

		ui.m_pHandCyberBtn->setEnabled(true);
		ui.m_pHandGraspBtn->setEnabled(true);

		ui.m_pHandClickBtn->setText("Click Mode");
	}
}

void CyberSystem::SendHandSpin()
{
	double RHand[5][3], LHand[5][3];
	ReadHandSpinData(RHand, LHand);

	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m_RHandSendData[i][j] = RHand[i][j];
			m_LHandSendData[i][j] = LHand[i][j];
		}
	}
}

void CyberSystem::UpdateHandSpin()
{
	SetHandSpinData(m_RHandRecvJoint, m_LHandRecvJoint);
}

void CyberSystem::CyberHandMode()
{
	if (ui.m_pHandCyberBtn->text() == "Cyber Mode")		// 进入Cyber Mode
	{
		ui.m_pHandRunBtn->setEnabled(true);
		ui.m_pHandPauseBtn->setEnabled(true);

		ui.m_pHandClickBtn->setEnabled(false);
		ui.m_pHandGraspBtn->setEnabled(false);

		ui.m_pHandCyberBtn->setText("Stop Cyber");
	}
	else if (ui.m_pHandCyberBtn->text() == "Stop Cyber")		// 离开Cyber Mode
	{
		m_HandCtrlMode = HAND_OUT_CTRL;

		ui.m_pHandRunBtn->setEnabled(false);
		ui.m_pHandPauseBtn->setEnabled(false);

		ui.m_pHandClickBtn->setEnabled(true);
		ui.m_pHandGraspBtn->setEnabled(true);

		ui.m_pHandCyberBtn->setText("Cyber Mode");
	}
}

void CyberSystem::StartCyberHand()
{
	m_HandCtrlMode = HAND_CYBER_CTRL;
}

void CyberSystem::PauseCyberHand()
{
	m_HandCtrlMode = HAND_OUT_CTRL;
}



void CyberSystem::GraspHandMode()
{
	if (ui.m_pHandGraspBtn->text() == "Grasp Mode")		// 进入Grasp Mode
	{
		m_bHandGrasp = false;
		m_HandCtrlMode = HAND_GRASP_CTRL;

		ui.m_pGraspBtn->setEnabled(true);
		ui.m_pRlseBtn->setEnabled(true);

		ui.m_pHandClickBtn->setEnabled(false);
		ui.m_pHandGraspBtn->setEnabled(false);

		ui.m_pHandGraspBtn->setText("Stop Grasp");
	}
	else if (ui.m_pHandGraspBtn->text() == "Stop Grasp")		// 离开Grasp Mode
	{
		m_HandCtrlMode = HAND_OUT_CTRL;

		ui.m_pGraspBtn->setEnabled(false);
		ui.m_pRlseBtn->setEnabled(false);

		ui.m_pHandClickBtn->setEnabled(true);
		ui.m_pHandGraspBtn->setEnabled(true);

		ui.m_pHandGraspBtn->setText("Grasp Mode");
	}
}

void CyberSystem::GraspHand()
{
	m_bHandGrasp = true;
}

void CyberSystem::ReleaseHand()
{
	m_bHandGrasp = false;
}




void CyberSystem::ReadHandSpinData(double RHandData[5][3], double LHandData[5][3])
{
	// 右手
	// 大拇指
	RHandData[0][2] = ui.m_pRThumbBiasSpin->value();
	RHandData[0][0] = ui.m_pRThumbBaseSpin->value();
	RHandData[0][1] = ui.m_pRThumbOutSpin->value();
	// 食指
	RHandData[1][2] = ui.m_pRIndexBiasSpin->value();
	RHandData[1][0] = ui.m_pRIndexBaseSpin->value();
	RHandData[1][1] = ui.m_pRIndexOutSpin->value();
	// 中指
	RHandData[2][2] = ui.m_pRMidBiasSpin->value();
	RHandData[2][0] = ui.m_pRMidBaseSpin->value();
	RHandData[2][1] = ui.m_pRMidOutSpin->value();
	// 无名指
	RHandData[3][2] = ui.m_pRRingBiasSpin->value();
	RHandData[3][0] = ui.m_pRRingBaseSpin->value();
	RHandData[3][1] = ui.m_pRRingOutSpin->value();
	// 小指
	RHandData[4][2] = ui.m_pRLitBiasSpin->value();
	RHandData[4][0] = ui.m_pRLitBaseSpin->value();
	RHandData[4][1] = ui.m_pRLitOutSpin->value();

	// 左手
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			LHandData[i][j] = 0.0;
		}
	}
}

void CyberSystem::SetHandSpinData(const double RData[5][3], const double LData[5][3])
{
		// 拇指
	ui.m_pRThumbBiasSpin->setValue(m_RHandRecvJoint[0][2]);
	ui.m_pRThumbBaseSpin->setValue(m_RHandRecvJoint[0][0]);
	ui.m_pRThumbOutSpin->setValue(m_RHandRecvJoint[0][1]);
	// 食指
	ui.m_pRIndexBiasSpin->setValue(m_RHandRecvJoint[1][2]);
	ui.m_pRIndexBaseSpin->setValue(m_RHandRecvJoint[1][0]);
	ui.m_pRIndexOutSpin->setValue(m_RHandRecvJoint[1][1]);
	// 中指
	ui.m_pRMidBiasSpin->setValue(m_RHandRecvJoint[2][2]);
	ui.m_pRMidBaseSpin->setValue(m_RHandRecvJoint[2][0]);
	ui.m_pRMidOutSpin->setValue(m_RHandRecvJoint[2][1]);
	// 无名指
	ui.m_pRRingBiasSpin->setValue(m_RHandRecvJoint[3][2]);
	ui.m_pRRingBaseSpin->setValue(m_RHandRecvJoint[3][0]);
	ui.m_pRRingOutSpin->setValue(m_RHandRecvJoint[3][1]);
	// 小指
	ui.m_pRLitBiasSpin->setValue(m_RHandRecvJoint[4][2]);
	ui.m_pRLitBaseSpin->setValue(m_RHandRecvJoint[4][0]);
	ui.m_pRLitOutSpin->setValue(m_RHandRecvJoint[4][1]);
}

void CyberSystem::SetGraspInit()
{
	m_HandReleaseJoint[0][0] = 0.0;
	m_HandReleaseJoint[0][1] = 0.0;
	m_HandReleaseJoint[0][2] = 0.0;
	m_HandReleaseJoint[1][0] = 0.0;
	m_HandReleaseJoint[1][1] = 0.0;
	m_HandReleaseJoint[1][2] = 0.0;
	m_HandReleaseJoint[2][0] = 0.0;
	m_HandReleaseJoint[2][1] = 0.0;
	m_HandReleaseJoint[2][2] = 0.0;
	m_HandReleaseJoint[3][0] = 0.0;
	m_HandReleaseJoint[3][1] = 0.0;
	m_HandReleaseJoint[3][2] = 0.0;
	m_HandReleaseJoint[4][0] = 0.0;
	m_HandReleaseJoint[4][1] = 0.0;
	m_HandReleaseJoint[4][2] = 0.0;

	m_HandGraspJoint[0][0] = 0.0;
	m_HandGraspJoint[0][1] = 0.0;
	m_HandGraspJoint[0][2] = 0.0;
	m_HandGraspJoint[1][0] = 0.0;
	m_HandGraspJoint[1][1] = 0.0;
	m_HandGraspJoint[1][2] = 0.0;
	m_HandGraspJoint[2][0] = 0.0;
	m_HandGraspJoint[2][1] = 0.0;
	m_HandGraspJoint[2][2] = 0.0;
	m_HandGraspJoint[3][0] = 0.0;
	m_HandGraspJoint[3][1] = 0.0;
	m_HandGraspJoint[3][2] = 0.0;
	m_HandGraspJoint[4][0] = 0.0;
	m_HandGraspJoint[4][1] = 0.0;
	m_HandGraspJoint[4][2] = 0.0;
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

	//	m_DisDataMutex.lock();
	m_RoboTotalStr = m_RoboStr + m_HandStr;
	emit InsertRoboText(m_RoboTotalStr);
	//	m_DisDataMutex.unlock();
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


mat7x1 CyberSystem::CalKine(const mat4x4 &T, double &last_arm_angle, mat7x1 &last_joint_angle)
{
	mat7x1 ref_angle;
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

	mat7x1 jo;
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




//*********************** Difference Kinetics ***********************//
// Calculate Jacobi Matrix and SVD 
void CyberSystem::CalJaco(const mat7x1 &mat_jo,mat6x7 &mat_jaco, double &sv_min)
{
	double q1 = mat_jo(0);
	double q2 = mat_jo(1);
	double q3 = mat_jo(2);
	double q4 = mat_jo(3);
	double q5 = mat_jo(4);
	double q6 = mat_jo(5);
	double q7 = mat_jo(6);

	mat_jaco(0,0) = sin(q1)*sin(q2)*-4.0E2-cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))*3.0E2-sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*3.0E2-sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))*2.85E2-cos(q4)*sin(q1)*sin(q2)*2.85E2;
	mat_jaco(0,1) = cos(q1)*(cos(q2)*8.0E1+cos(q2)*cos(q4)*5.7E1+cos(q2)*cos(q4)*cos(q6)*6.0E1-cos(q3)*sin(q2)*sin(q4)*5.7E1-cos(q3)*cos(q6)*sin(q2)*sin(q4)*6.0E1-cos(q2)*cos(q5)*sin(q4)*sin(q6)*6.0E1+sin(q2)*sin(q3)*sin(q5)*sin(q6)*6.0E1-cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*6.0E1)*5.0;
	mat_jaco(0,2) = sin(q4)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))*-2.85E2+sin(q6)*(sin(q5)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q4)*cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*3.0E2-cos(q6)*sin(q4)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))*3.0E2;
	mat_jaco(0,3) = cos(q6)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))*-3.0E2-cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))*2.85E2+cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))*3.0E2-cos(q1)*sin(q2)*sin(q4)*2.85E2;
	mat_jaco(0,4) = sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*3.0E2;
	mat_jaco(0,5) = sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))*3.0E2-cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*3.0E2;

	mat_jaco(1,0) = cos(q1)*sin(q2)*4.0E2-cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))*3.0E2-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*3.0E2-sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))*2.85E2+cos(q1)*cos(q4)*sin(q2)*2.85E2;
	mat_jaco(1,1) = sin(q1)*(cos(q2)*8.0E1+cos(q2)*cos(q4)*5.7E1+cos(q2)*cos(q4)*cos(q6)*6.0E1-cos(q3)*sin(q2)*sin(q4)*5.7E1-cos(q3)*cos(q6)*sin(q2)*sin(q4)*6.0E1-cos(q2)*cos(q5)*sin(q4)*sin(q6)*6.0E1+sin(q2)*sin(q3)*sin(q5)*sin(q6)*6.0E1-cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*6.0E1)*5.0;
	mat_jaco(1,2) = sin(q4)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))*2.85E2-sin(q6)*(sin(q5)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-cos(q4)*cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*3.0E2+cos(q6)*sin(q4)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))*3.0E2;
	mat_jaco(1,3) = cos(q6)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))*3.0E2+cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))*2.85E2-sin(q1)*sin(q2)*sin(q4)*2.85E2-cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))*3.0E2;
	mat_jaco(1,4) = sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*-3.0E2;
	mat_jaco(1,5) = sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))*-3.0E2+cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*3.0E2;

	mat_jaco(2,1) = sin(q2)*-4.0E2-cos(q4)*sin(q2)*2.85E2+sin(q6)*(cos(q5)*(sin(q2)*sin(q4)-cos(q2)*cos(q3)*cos(q4))+cos(q2)*sin(q3)*sin(q5))*3.0E2-cos(q6)*(cos(q4)*sin(q2)+cos(q2)*cos(q3)*sin(q4))*3.0E2-cos(q2)*cos(q3)*sin(q4)*2.85E2;
	mat_jaco(2,2) = sin(q2)*(sin(q3)*sin(q4)*1.9E1+cos(q6)*sin(q3)*sin(q4)*2.0E1+cos(q3)*sin(q5)*sin(q6)*2.0E1+cos(q4)*cos(q5)*sin(q3)*sin(q6)*2.0E1)*1.5E1;
	mat_jaco(2,3) = cos(q2)*sin(q4)*-2.85E2-cos(q6)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))*3.0E2-cos(q5)*sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))*3.0E2-cos(q3)*cos(q4)*sin(q2)*2.85E2;
	mat_jaco(2,4) = sin(q6)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))*3.0E2;
	mat_jaco(2,5) = cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))*-3.0E2-sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))*3.0E2;

	mat_jaco(3,1) = -sin(q1);
	mat_jaco(3,2) = cos(q1)*sin(q2);
	mat_jaco(3,3) = -cos(q3)*sin(q1)-cos(q1)*cos(q2)*sin(q3);
	mat_jaco(3,4) = -sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*cos(q4)*sin(q2);
	mat_jaco(3,5) = sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3));
	mat_jaco(3,6) = -cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));

	mat_jaco(4,1) = cos(q1);
	mat_jaco(4,2) = sin(q1)*sin(q2);
	mat_jaco(4,3) = cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3);
	mat_jaco(4,4) = sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2);
	mat_jaco(4,5) = -sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3));
	mat_jaco(4,6) = cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))+sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));

	mat_jaco(5,0) = 1.0;
	mat_jaco(5,2) = cos(q2);
	mat_jaco(5,3) = sin(q2)*sin(q3);
	mat_jaco(5,4) = cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4);
	mat_jaco(5,5) = sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3);
	mat_jaco(5,6) = -sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4));

	mat6x1 sing_mat = mat_jaco.jacobiSvd().singularValues();
	sv_min = sing_mat(5);
}

// Damper Least Square Method
double CyberSystem::DampLS(double svmin)
{
	double sv_thr = 0.05;
	double lambda_max = 5;
	double lambda;

	if (svmin >= sv_thr)
	{
		lambda = 0;
	} 
	else
	{
		lambda = pow(lambda_max, 2)*(1 - pow(svmin/sv_thr, 2));
		if (lambda > lambda_max)
		{
			lambda = lambda_max;
		}
	}

	return lambda;
}

// Calculate Direct Kinetics
void CyberSystem::CalDirectKine(const mat7x1 &mat_jo, mat4x4 &mat_trans)
{
	double q1 = mat_jo(0);
	double q2 = mat_jo(1);
	double q3 = mat_jo(2);
	double q4 = mat_jo(3);
	double q5 = mat_jo(4);
	double q6 = mat_jo(5);
	double q7 = mat_jo(6);

	mat_trans(0,0) = cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))))+sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
	mat_trans(0,1) = -sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))))+cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
	mat_trans(0,2) = -cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
	mat_trans(0,3) = cos(q1)*sin(q2)*4.0E2-cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))*3.0E2-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*3.0E2-sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))*2.85E2+cos(q1)*cos(q4)*sin(q2)*2.85E2;
	mat_trans(1,0) = -cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))))-sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
	mat_trans(1,1) = sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))))-cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
	mat_trans(1,2) = cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))+sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
	mat_trans(1,3) = sin(q1)*sin(q2)*4.0E2+cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))*3.0E2+sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*3.0E2+sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))*2.85E2+cos(q4)*sin(q1)*sin(q2)*2.85E2;
	mat_trans(2,0) = sin(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))-cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)));
	mat_trans(2,1) = cos(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))+sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)));
	mat_trans(2,2) = -sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4));
	mat_trans(2,3) = cos(q2)*4.0E2+cos(q2)*cos(q4)*2.85E2-sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))*3.0E2+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))*3.0E2-cos(q3)*sin(q2)*sin(q4)*2.85E2;
	mat_trans(3,0) = mat_trans(3,1) = mat_trans(3,2) = 0;
	mat_trans(3,3) = 1.0;
}



// 判断关节角的范围
// 如果在范围内，返回true；如果不在范围内，返回false 
bool CyberSystem::AngleRange(const mat7x1 &jo)
{
	bool ret = true;

	// 	double min_angle[7] = {ROBO_J0_MIN, ROBO_J1_MIN, ROBO_J2_MIN, ROBO_J3_MIN, ROBO_J4_MIN, ROBO_J5_MIN, ROBO_J6_MIN};
	// 	double max_angle[7] = {ROBO_J0_MAX, ROBO_J1_MAX, ROBO_J2_MAX, ROBO_J3_MAX, ROBO_J4_MAX, ROBO_J5_MAX, ROBO_J6_MAX};
	double min_angle[7] = {-180, -180, -180, -180, -180, -180, -180};
	double max_angle[7] = {180, 180, 180, 180, 180, 180, 180};

	for (int i = 0; i < 7; ++i)
	{
		ret = ret & ((jo(i) < ANG2DEG(max_angle[i])) && (jo(i) > ANG2DEG(min_angle[i])));
	}

	return ret;
}



// 微分运动学函数
// 输入目标姿态pose_new，参考姿态pose_ref和参考角度joint_ref；求解逆运动学，更新joint变量
// 输出逆运动学计算是否成功
bool CyberSystem::DiffKine(const mat7x1 &pose_new, const mat7x1 &pose_ref, const mat7x1 &joint_ref, mat7x1 &joint)
{
	int cal_count = 200;		// 计算次数
	double cal_period = 0.001;		// 计算周期，单位：秒

	mat7x1 quat_ref = pose_ref;			// 当前姿态
	mat4x4 trans_new;		// 目标姿态的旋转矩阵表示
	mat7x1 quat_next;		// 下一步的目标姿态

	mat7x1 joint_feed = joint_ref;		// 反馈角度
	mat4x4 trans_feed;		// 反馈姿态的旋转矩阵表示
	mat7x1 quat_feed = pose_ref;		// 反馈姿态的四元数表示,初始化为当前姿态

	double sv_min;		// 最小奇异值
	double lambda = 0;		// 阻尼最小二乘系数


	double pose_new_arr[7];
	for (int i = 0; i < 7; ++i)
	{
		pose_new_arr[i] = pose_new(i);
	}
	QuaterToTrans(pose_new_arr, trans_new);

	for (int i = 0; i < cal_count; ++i)
	{
		PosePlan(pose_ref, pose_new, cal_count, (i+1), quat_next);

		// Transform delta quaternion to delta omega
		double Eta = quat_ref(3);
		double Epsilon1 = quat_ref(4);
		double Epsilon2 = quat_ref(5);
		double Epsilon3 = quat_ref(6);

		m_RQuatoOmega(0,0) = Eta;
		m_RQuatoOmega(0,1) = Epsilon1;
		m_RQuatoOmega(0,2) = Epsilon2;
		m_RQuatoOmega(0,3) = Epsilon3;

		m_RQuatoOmega(1,0) = -Epsilon1;
		m_RQuatoOmega(1,1) = Eta;
		m_RQuatoOmega(1,2) = Epsilon3;
		m_RQuatoOmega(1,3) = -Epsilon2;

		m_RQuatoOmega(2,0) = -Epsilon2;
		m_RQuatoOmega(2,1) = -Epsilon3;
		m_RQuatoOmega(2,2) = Eta;
		m_RQuatoOmega(2,3) = Epsilon1;

		m_RQuatoOmega(3,0) = -Epsilon3;
		m_RQuatoOmega(3,1) = Epsilon2;
		m_RQuatoOmega(3,2) = -Epsilon1;
		m_RQuatoOmega(3,3) = Eta;

		// 规划的速度参数
		for (int i = 0; i < 4; ++i)
		{
			m_RdQuat(i) = (quat_next(i + 3) - quat_ref(i + 3))/cal_period;
		}

		m_ROmega = 2*m_RQuatoOmega*m_RdQuat;
		for (int i = 0; i < 3; ++i)
		{
			m_RVel(i) = (quat_next(i) - quat_ref(i))/cal_period;
			m_RVel(i + 3) = m_ROmega(i + 1);
		}


		// 输入控制误差
		des_Epsilon << quat_next(4), quat_next(5), quat_next(6);
		feed_Epsilon << quat_feed(4), quat_feed(5), quat_feed(6);
		CrossMat << 0, -des_Epsilon(2), des_Epsilon(1),
			des_Epsilon(2), 0, -des_Epsilon(0),
			-des_Epsilon(1), des_Epsilon(0), 0;
		m_RRotErr = quat_feed(3)*feed_Epsilon - quat_next(3)*des_Epsilon - CrossMat*feed_Epsilon;

		double K_Pos = 1;		// 位置误差调节 
		double K_Ori = 100;		// 姿态误差调节
		for (int i = 0; i < 3; ++i)
		{
			m_RPoseErr(i) = K_Pos*(quat_next(i) - quat_feed(i));		// 位置误差反馈
			m_RPoseErr(i + 3) = K_Ori*m_RRotErr(i);		// 姿态误差反馈
		}

		// calculate jacobi
		CalJaco(joint_feed, m_RJacoMat, sv_min);

		// damper least square
		lambda = DampLS(fabs(sv_min));		// 使用sv_min的绝对值
		auto damp_coeff = m_RJacoMat.transpose()*(m_RJacoMat*m_RJacoMat.transpose() + lambda*Eigen::MatrixXd::Identity(6,6)).inverse();

		// gradient
		double optimcoeff = 10;
		mat7x1 jo_min, jo_max, jo_mid;
		jo_min << ROBO_J0_MIN, ROBO_J1_MIN, ROBO_J2_MIN, ROBO_J3_MIN, ROBO_J4_MIN, ROBO_J5_MIN, ROBO_J6_MIN;
		jo_max << ROBO_J0_MAX, ROBO_J1_MAX, ROBO_J2_MAX, ROBO_J3_MAX, ROBO_J4_MAX, ROBO_J5_MAX, ROBO_J6_MAX;

		// transform joint angle to joint degree
		for (int i = 0; i < 7; ++i)
		{
			jo_min(i) = ANG2DEG(jo_min(i));
			jo_max(i) = ANG2DEG(jo_max(i));
		}

		jo_mid = (jo_min + jo_max)/2;
		mat7x1 dq0;
		for (int i = 0; i < 7; ++i)
		{
			// 改变梯度投影法的参数k，原来为100
			dq0(i) = -1*(1.0/7.0)*(joint_ref(i) - jo_mid(i))/pow((jo_max(i) - jo_min(i)),2);
		}
		auto grad_coeff = optimcoeff*(Eigen::MatrixXd::Identity(7,7) - m_RJacoMat.transpose()*(m_RJacoMat*m_RJacoMat.transpose()).inverse()*m_RJacoMat)*dq0;

		// control
		auto dx = m_RVel + m_RPoseErr;
		auto dq = damp_coeff*dx + grad_coeff;

		// integrate
		joint = joint_feed + dq*cal_period;

		// 判断计算的角度是否在机械臂的关节限位以内
		bool ret = AngleRange(joint);
		if (ret == false)
		{
			m_CmdStr += "Arm Joints Out of Range!!!\r\n";
			InsertCmdStr(m_CmdStr);
			return false;
		}

		// 正运动学反馈
		joint_feed = joint;
		CalDirectKine(joint_feed, trans_feed);
		TransToQuater(trans_feed, quat_feed);
		quat_ref = quat_next;
	}

	// 检测输出
	CalDirectKine(joint, m_RFeedTrans);
	auto err = (m_RFeedTrans - trans_new).norm();

	if (lambda == 0)		// 阻尼最小二乘不起作用
	{
		if ((m_RFeedTrans - trans_new).norm() < 0.5)
		{
			m_CmdStr += "In Normal mode!!!\r\n";
			InsertCmdStr(m_CmdStr);
			return true;
		} 
		else
		{
			m_CmdStr += "In normal mode, Deviation is too Large!!!\r\n";
			InsertCmdStr(m_CmdStr);
			return false;
		}
	}
	else		// 阻尼最小二乘起作用
	{
		if ((m_RFeedTrans - trans_new).norm() < 2)
		{
			m_CmdStr += "In Damper mode!!!\r\n";
			InsertCmdStr(m_CmdStr);
			return true;
		} 
		else
		{
			m_CmdStr += "In Damper mode, Deviation is too Large!!!\r\n";
			InsertCmdStr(m_CmdStr);
			return false;
		}
	}
}

// 规划总步数PlanTotalCount，目前规划步数Plancount，规划起点StartPose，规划终点EndPose
// 输出每一步规划的位姿
void CyberSystem::PosePlan(const mat7x1 &StartPose, const mat7x1 &EndPose, const int PlanTotalCount, const int PlanCount, mat7x1 &PoseOut)
{
	for (int i = 0; i < 3; ++i)
	{
		PoseOut(i) = StartPose(i) + (EndPose(i) - StartPose(i))/PlanTotalCount*PlanCount;
	}

	mat4x1 Ori_tmp;
	for (int i = 0; i < 4; ++i)
	{
		Ori_tmp(i) = StartPose(i+3) + (EndPose(i+3) - StartPose(i+3))/PlanTotalCount*PlanCount;
	}
	Ori_tmp = Ori_tmp/(Ori_tmp.norm());
	for (int i = 0; i < 4; ++i)
	{
		PoseOut(i+3) = Ori_tmp(i);
	}
}

// 点击Execute按钮，进入一次路径规划控制,初始化参考角度
void CyberSystem::ExecPlan()
{
	// 	m_CtrlMode = PLAN_WAIT;
	// 	m_last_RTraRealQuat = m_RPlanStartQuat;
	// 	m_plan_count = 0;
	// 	m_plan_count_max = m_RPlanTime/(RobonautCommPd/1000.0);
	// 
	// 	// 初始化参考角度
	// 	double quat[7];
	// 	for (int i = 0; i < 7; ++i)
	// 	{
	// 		quat[i] = m_RPlanStartQuat(i);
	// 	}
	// 	QuaterToTrans(quat, m_RTraRealMat);
	// 	mat7x1 q = CalKine(m_RTraRealMat, m_last_arm_angle, m_last_joint_angle);
	// 
	// 	// 发送初始角度
	// 	// g_rightArmJointBuf is the Command Buffer
	// 	for (int i = 0; i < 7; ++i)
	// 	{
	// 		g_rightArmJointBuf[i] = DEG2ANG(q(i));
	// 		g_leftArmJointBuf[i] = 0;
	// 	}
	// 	if (m_bRoboConn == true)
	// 	{
	// 		bool robo_ret = m_RobonautControl.SendRoboMsg();
	// 	}
	// 	if (m_bConsimuConn == true)
	// 	{
	// 		bool consimu_ret = m_RobonautControl.SendConsimuMsg();
	// 	}
	// 
	// 	// 待机械臂来到初始的位置
	// 	m_CmdStr += "Moving to init Pose......";
	// 	InsertCmdStr(m_CmdStr);
	// 	Sleep(500);
	// 	m_CmdStr += "Success!!!\r\n";
	// 	InsertCmdStr(m_CmdStr);

	// 改变控制模式，定时器进入周期性规划
	if (m_bRoboConn == true && m_bConsimuConn == true)
	{
		m_CtrlMode = PLAN_CTRL_ALL;
	} 
	else if(m_bRoboConn == true && m_bConsimuConn == false)
	{
		m_CtrlMode = PLAN_CTRL_ROBO;
	}
	else{
		m_CtrlMode = PLAN_CTRL_SIMU;
	}
}

// 点击Plan按钮，进入Plan控制模式
void CyberSystem::PlanCtrl()
{
	if (m_CtrlMode == OUT_CTRL && (m_bRoboConn == true || m_bConsimuConn == true) && (ui.m_pPlanCtrlBtn->text() == "Plan"))
	{
		m_CmdStr += "Initializing DataSend Timer......";
		emit InsertCmdStr(m_CmdStr);
		// Initialize Send Timer
		if (SendTimerId == NULL)
		{
			SendTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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

			ui.m_pReadPoseBtn->setEnabled(true);
			ui.m_pExecPlanBtn->setEnabled(true);

			ui.m_pJointCtrlBtn->setEnabled(false);
			ui.m_pJointCtrlBtn->setEnabled(false);

			ui.m_pPlanCtrlBtn->setText("Stop");
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

		m_CmdStr += "Stop Plan Control......";
		emit InsertCmdStr(m_CmdStr);

		UINT ret_send = timeKillEvent(SendTimerId);
		SendTimerId = NULL;

		if (ret_send == TIMERR_NOERROR)
		{
			m_CmdStr += "OK!!!\r\n";
			emit InsertCmdStr(m_CmdStr);

			ui.m_pPlanCtrlBtn->setText("Plan");

			ui.m_pReadPoseBtn->setEnabled(false);
			ui.m_pExecPlanBtn->setEnabled(false);
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

// TODO(CJH): Add Left
void CyberSystem::getPlanData()
{
	if ((ui.m_pRPlanStartPose->text() != "") && (ui.m_pRPlanEndPose->text() != "") && (ui.m_pRPlanTime->text() != ""))
	{
		m_CmdStr += "Start Reading Data......";
		InsertCmdStr(m_CmdStr);

		std::string rstartpose_str = ui.m_pRPlanStartPose->text().toStdString();
		std::string rendpose_str = ui.m_pRPlanEndPose->text().toStdString();
		std::string rtime_str = ui.m_pRPlanTime->text().toStdString();

		std::istringstream r_start_istr(rstartpose_str);
		std::istringstream r_end_istr(rendpose_str);
		std::istringstream r_time_istr(rtime_str);

		// Read Plannig Start Pose
		int start_count = 0;
		for (int i = 0; i < 7 && (!r_start_istr.eof()); ++i)
		{
			r_start_istr >> m_RPlanStartQuat(i);
			++start_count;
		}
		if (start_count != 7)
		{
			m_CmdStr += "There is No Enough Inputs for Right Planning!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		// Read Planning End Pose
		int end_count = 0;
		for (int i = 0; i < 7 && (!r_end_istr.eof()); ++i)
		{
			r_end_istr >> m_RPlanEndQuat(i);
			++end_count;
		}
		if (end_count != 7)
		{
			m_CmdStr += "There is No Enough Inputs for Right Planning!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
			return;
		}

		// Read Time
		r_time_istr >> m_RPlanTime;

		m_CmdStr += "OK!!!\r\n";
		InsertCmdStr(m_CmdStr);



		m_CtrlMode = PLAN_WAIT;
		m_last_RTraRealQuat = m_RPlanStartQuat;
		m_plan_count = 0;
		m_plan_count_max = m_RPlanTime/(RobonautCommPd/1000.0);

		// 初始化参考角度
		double quat[7];
		for (int i = 0; i < 7; ++i)
		{
			quat[i] = m_RPlanStartQuat(i);
		}
		QuaterToTrans(quat, m_RTraRealMat);
		mat7x1 q = CalKine(m_RTraRealMat, m_last_arm_angle, m_last_joint_angle);

		// 发送初始角度
		// g_rightArmJointBuf is the Command Buffer
		for (int i = 0; i < 7; ++i)
		{
			g_rightArmJointBuf[i] = DEG2ANG(q(i));
			g_leftArmJointBuf[i] = 0;
		}
		if (m_bRoboConn == true)
		{
			bool robo_ret = m_RobonautControl.SendRoboMsg();
		}
		if (m_bConsimuConn == true)
		{
			bool consimu_ret = m_RobonautControl.SendConsimuMsg();
		}

		// 待机械臂来到初始的位置
		m_CmdStr += "Moving to init Pose......";
		InsertCmdStr(m_CmdStr);
		Sleep(500);
		m_CmdStr += "Success!!!\r\n";
		InsertCmdStr(m_CmdStr);
	} 
	else
	{
	}
}






//*********************** Vision Control ***********************//
//void CyberSystem::ConnVision()
//{
//	m_CmdStr += "Connecting Vision......";
//	InsertCmdStr(m_CmdStr);
//
//	char *Vision_ip = VISION_SENSE_IP;
//	UINT Vision_Port = VISION_SENSE_PORT; 
//
//	int recv_ret = m_VisionRecv_Client.ConnectServer(VISION_SENSE_IP,VISION_SENSE_PORT);
//
//	if (recv_ret == 0)
//	{
//		m_CmdStr += "Success!\r\nCreating Vision Timer......";
//		InsertCmdStr(m_CmdStr);
//
//		VisionTimerId = timeSetEvent(30, 1, (LPTIMECALLBACK)TimerProcVisionRecv, 0, TIME_PERIODIC);
//
//		if (VisionTimerId != NULL)
//		{
//			m_CmdStr += "Success!\r\n";
//			InsertCmdStr(m_CmdStr);
//
//			// Connect Cameral Success
//			m_bConnCam = true;
//		} 
//		else
//		{
//			m_CmdStr += "Failed!\r\n";
//			InsertCmdStr(m_CmdStr);
//		}
//	}
//
//	else{
//		m_CmdStr += "Failed!\r\n";
//		InsertCmdStr(m_CmdStr);
//		return;
//	}
//}





// Use While Loop to Receive Cameral Data
void CyberSystem::ViCycle()
{
	while(m_CamThread.m_bCamThreadStop == false)
	{
		RecvVision();
		m_CamThread.msleep(30);
	}
	// ensure that you will get into the thread next time
	m_CamThread.m_bCamThreadStop = false;
}

//Receive Vision Data from camera, then display in the QLineEdit
void CyberSystem::RecvVision()
{
	int ret = m_VisionRecv_Client.RecvViData(m_ViRecv);

	ViRecvTrans(m_ViRecv, m_ViEulerRecv);

	std::ostringstream vision_stream; 

	// Original Representation
	// 	for (int i = 0; i < 7; ++i)
	// 	{
	// 		vision_stream << m_ViRecv[i] << " ";
	// 	}

	// Euler Representation
	vision_stream << m_ViEulerRecv[0] << " ";
	for (int i = 0; i < 3; ++i)
	{
		vision_stream << DEG2ANG(m_ViEulerRecv[i+1]) << " ";
	}
	for (int i = 0; i < 3; ++i)
	{
		vision_stream << m_ViEulerRecv[i+4] << " ";
	}

	vision_stream << std::endl;
	std::string vision_str = vision_stream.str();
	QString q_VisionStr = q_VisionStr.fromStdString(vision_str);

	ui.m_pViDataDisEd->setText(q_VisionStr);
}



void CyberSystem::ConnVision()
{
	m_CmdStr += "Connecting Vision......";
	InsertCmdStr(m_CmdStr);

	char *Vision_ip = VISION_SENSE_IP;
	UINT Vision_Port = VISION_SENSE_PORT; 

	int recv_ret = m_VisionRecv_Client.ConnectServer(VISION_SENSE_IP,VISION_SENSE_PORT);

	if (recv_ret == 0)
	{
		m_CmdStr += "Success!\r\nCreating Vision Timer......";
		InsertCmdStr(m_CmdStr);

		if (!(m_CamThread.isRunning()))
		{
			m_CamThread.start();
			m_bConnCam = true;
		} 
		else{}
	}

	else{
		m_CmdStr += "Failed!\r\n";
		InsertCmdStr(m_CmdStr);
		return;
	}
}




void CyberSystem::VisionCtrl()
{

#if ArmDebug


#else
	if (m_bConnCam == false)
	{
		m_CmdStr += "Connect Cameral First!!!\r\n";
		emit InsertCmdStr(m_CmdStr);
		return;
	}
#endif


	// 
	if (m_CtrlMode == OUT_CTRL && (ui.m_pViCtrlBtn->text() == "ViCtrl"))
	{
		m_CmdStr += "Initializing DataSend Timer......";
		emit InsertCmdStr(m_CmdStr);
		// Initialize Send Timer
		if (SendTimerId == NULL)
		{
			SendTimerId = timeSetEvent(RobonautCommPd, 1, (LPTIMECALLBACK)TimerProcSendCmd, 0, TIME_PERIODIC);
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

			ui.m_pJointCtrlBtn->setEnabled(false);
			ui.m_pCyberCtrlBtn->setEnabled(false);
			ui.m_pPlanCtrlBtn->setEnabled(false);

			ui.m_pViStartBtn->setEnabled(true);

			ui.m_pViCtrlBtn->setText("Stop");
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

		m_CmdStr += "Stop Vision Control......";
		emit InsertCmdStr(m_CmdStr);

		UINT ret_send = timeKillEvent(SendTimerId);
		SendTimerId = NULL;

		if (ret_send == TIMERR_NOERROR)
		{
			m_CmdStr += "OK!!!\r\n";
			emit InsertCmdStr(m_CmdStr);

			ui.m_pViCtrlBtn->setText("ViCtrl");

			ui.m_pJointCtrlBtn->setEnabled(true);
			ui.m_pCyberCtrlBtn->setEnabled(true);
			ui.m_pPlanCtrlBtn->setEnabled(true);

			ui.m_pViStartBtn->setEnabled(false);
			ui.m_pViPauseBtn->setEnabled(false);
		} 
		else
		{
			m_CmdStr += "Failed!!!\r\n";
			emit InsertCmdStr(m_CmdStr);
		}
	}
}

void CyberSystem::VisionStart()
{

	m_CmdStr += "Start Vision Control!!!\r\n";
	emit InsertCmdStr(m_CmdStr);

#if ArmDebug

	// 虚拟仿真测试
	//last_cmd_jo << ANG2DEG(-62.0591), ANG2DEG(-49.0389), ANG2DEG(-8.618), ANG2DEG(-57.4985), ANG2DEG(15.345), ANG2DEG(-37.3492), ANG2DEG(-21.9836);
	// 初始化关节角
	for (int i = 0; i < 7; ++i)
	{
	 	last_cmd_jo(i) = ANG2DEG(g_RobotSensorDeg.rightArmJoint[i]);
	}

	mat7x1 cmd_jo;

	for (int i = 0; i < 7; ++i)
	{
		cmd_jo(i) = last_cmd_jo(i);
	}

	for (int i = 0; i < 7; ++i)
	{
		g_rightArmJointBuf[i] = DEG2ANG(cmd_jo(i));
		g_leftArmJointBuf[i] = 0;
	}

	// 机器人测试
	// 初始化关节角
	// 	for (int i = 0; i < 7; ++i)
	// 	{
	// 		last_cmd_jo(i) = g_RobotSensorDeg.rightArmJoint[i];
	// 	}
	// 
	// 	// 更新命令发送到缓冲数组
	// 	for (int i = 0; i < 7; ++i)
	// 	{
	// 		g_rightArmJointBuf[i] = DEG2ANG(last_cmd_jo(i));
	// 		g_leftArmJointBuf[i] = 0;
	// 	}
	// 
	// 	bool ret = m_RobonautControl.SendRoboMsg();
	// 
	// 	Sleep(1000);		// 等待机械臂动到相应的位置
#else
	//last_cmd_jo << ANG2DEG(-62.1245), ANG2DEG(-40.6436), ANG2DEG(-8.7226), ANG2DEG(-59.5151), ANG2DEG(14.3206), ANG2DEG(-38.3765), ANG2DEG(-21.8541);
	// 初始化关节角
	for (int i = 0; i < 7; ++i)
	{
		last_cmd_jo(i) = g_RobotSensorDeg.rightArmJoint[i];
		last_cmd_jo(i) = ANG2DEG(last_cmd_jo(i));
	}

	// 更新命令发送到缓冲数组
	for (int i = 0; i < 7; ++i)
	{
		g_rightArmJointBuf[i] = DEG2ANG(last_cmd_jo(i));
		g_leftArmJointBuf[i] = 0;
	}

	bool ret = m_RobonautControl.SendRoboMsg();
	//bool send_ret = m_RobonautControl.SendConsimuMsg();

	Sleep(1000);		// 等待机械臂动到相应的位置
#endif

	// 进入控制循环
	m_CtrlMode = VISION_CTRL;

	ui.m_pViStartBtn->setEnabled(false);
	ui.m_pViPauseBtn->setEnabled(true);
}


void CyberSystem::VisionStop()
{
	m_CmdStr += "Pause Vision Control!!!\r\n";
	emit InsertCmdStr(m_CmdStr);

	m_CtrlMode = OUT_CTRL;

	ui.m_pViStartBtn->setEnabled(true);
	ui.m_pViPauseBtn->setEnabled(false);
}

void CyberSystem::VisionAppr()
{
	m_bApprFlag = true;
}

// 
void CyberSystem::GetViNextAppr(const double ViEulerRecv[], const double Quat_Ref[7], double Quat_New[7])
{
	double ApprRef = RefFinAppr;		// 相机中，需要达到的接近距离（Z向）
	double ApprNow = ViEulerRecv[6];		// 目前相机中的接近距离（Z向）
	double move_step = 1;
	
	if (fabs(ApprNow) > ApprRef)		// 距离调整未完成
	{
		mat4x4 trans_ref;		// 末端当前位姿
		mat4x4 trans_new;		// 末端更新后位姿
		QuaterToTrans(Quat_Ref, trans_ref);
		mat4x4 move_mat;		// 参考机械臂末端移动矩阵
		move_mat << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, SGN(ApprNow - ApprRef)*move_step,
			0, 0, 0, 1;

		trans_new = trans_ref*move_mat;
		mat7x1 Quat_New_Mat;
		TransToQuater(trans_new, Quat_New_Mat);
		for (int i = 0; i < 7; ++i)
		{
			Quat_New[i] = Quat_New_Mat(i);
		}
	}
	else		// 距离调整完成
	{
		for (int i = 0; i < 7; ++i)
		{
			Quat_New[i] = Quat_Ref[i];
		}
	}
}


// Get New Pose from Vision Data
// Quat_Ref is the Current Pose
// Quat_New is the Pose Planning to 
void CyberSystem::GetViNextPos(const double ViRecv[], const mat7x1 &Quat_Ref, mat7x1 &Quat_New)
{
	// camera reference data
	double ViRefPos[3];
	ViRefPos[0] = RefEulerPose[3];
	ViRefPos[1] = RefEulerPose[4];
	ViRefPos[2] = RefEulerPose[5];
	mat3x1 ViMove;
	mat3x1 BaseMove;
	mat3x1 EndMove;
	ViMove << (ViRecv[4] - ViRefPos[0]), (ViRecv[5] - ViRefPos[1]), (ViRecv[6] - ViRefPos[2]);
	// 相机坐标系到机械臂末端坐标系的变换
	EndMove(0) = -ViMove(1);
	EndMove(1) = ViMove(0);
	EndMove(2) = ViMove(2);

	mat3x3 RotTrans;
	
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			RotTrans(i, j) = m_ViTransNow(i, j);
		}
	}
	BaseMove = RotTrans*EndMove;


	// two different control distance
	double pos_far = 50.0;
	double pos_cut = 10.0;
	//	double pos_cut = 5.0;

	// two different distance change in a control period
	double dis_max = 10;
	double dis_min = 2;

	// Vision Receive Data Flag
	int Vision_Flag = (int) ViRecv[0];
	// TODO(CJH):Change Axis Representation to quaternion

	m_bTrackFinish = false;
	// Use Vision Data to Calculate New Quaternion Data
	if (Vision_Flag == 1)
	{
		// Position
		// X Position
		if (fabs(BaseMove(0)) > pos_far)
		{
			Quat_New(0) = Quat_Ref(0) + SGN(BaseMove(0))*dis_max;
		}
		else if ((fabs(BaseMove(0)) < pos_far) && (fabs(BaseMove(0)) > pos_cut))
		{
			Quat_New(0) = Quat_Ref(0) + SGN(BaseMove(0))*dis_min;
		}
		else
		{
			Quat_New(0) = Quat_Ref(0);
		}

		// Y Position
		if (fabs(BaseMove(1)) > pos_far)
		{
			Quat_New(1) = Quat_Ref(1) + SGN(BaseMove(1))*dis_max;
		}
		else if ((fabs(BaseMove(1)) < pos_far) && (fabs(BaseMove(1)) > pos_cut))
		{
			Quat_New(1) = Quat_Ref(1) + SGN(BaseMove(1))*dis_min;
		}
		else
		{
			Quat_New(1) = Quat_Ref(1);
		}

		// Z Position
		if (fabs(BaseMove(2)) > pos_far)
		{
			Quat_New(2) = Quat_Ref(2) + SGN(BaseMove(2))*dis_max;
		}
		else if ((fabs(BaseMove(2)) < pos_far) && (fabs(BaseMove(2)) > pos_cut))
		{
			Quat_New(2) = Quat_Ref(2) + SGN(BaseMove(2))*dis_min;
		}
		else
		{
			Quat_New(2) = Quat_Ref(2);
		}

		// Orientation


		for (int i = 0; i < 4; ++i)
		{
			Quat_New(i+3) = Quat_Ref(i+3);
		}

		// Move Finish
		if ((fabs(BaseMove(0)) < pos_cut) && (fabs(BaseMove(1)) < pos_cut) && (fabs(BaseMove(2)) < pos_cut))
		{
			m_bTrackFinish = true;
		}
	} 

	// No Target
	else
	{
		for (int i = 0; i < 7; ++i)
		{
			Quat_New(i) = Quat_Ref(i);
		}
	}
}



// void CyberSystem::GetViNextOri(const double ViEulerRecv[], const double Quat_Ref[7], double Quat_New[7])
// {
// 	double move_step = ANG2DEG(0.5);
// 	double ZEulerDegCut = EulerPoseCut[0];
// 	double YEulerDegCut = EulerPoseCut[1];
// 	double XEulerDegCut = EulerPoseCut[2];
// 	double ZEulerDegRef = RefEulerPose[0];
// 	double YEulerDegRef = RefEulerPose[1];
// 	double XEulerDegRef = RefEulerPose[2];
// 
// 	double Euler[3];
// 	for (int i = 0; i < 3; ++i)
// 	{
// 		Euler[i] = ViEulerRecv[i+1];
// 	}
// 
// 	// 机械臂末端到相机的变换矩阵
// 	mat4x4 EndToCamMat;
// 	EndToCamMat << 1, 0, 0, 0, 
// 		0, 1, 0, 0, 
// 		0, 0, 1, 0, 
// 		0, 0, 0, 1;
// 
// 	mat4x4 BaseToCamMat_Ref;	// 相机相对于基座
// 	mat4x4 BaseToCamMat_New;
// 	mat4x4 BaseToEndMat_Ref;	// 机械臂末端相对于基座
// 	mat4x4 BaseToEndMat_New;
// 
// 	double adjust_euler[6] = {0, 0, 0, 0, 0, 0};		// 角度调节欧拉角
// 	mat4x4 AdjustMat;		// 角度调整矩阵
// 
// 	QuaterToTrans(Quat_Ref, BaseToEndMat_Ref);
// 	BaseToCamMat_Ref = BaseToEndMat_Ref*EndToCamMat;
// 
// 
// 
// #if ArmDebug		// 调试使用，每次循环更新绕末端坐标系旋转一个角度
// 	adjust_euler[3] = ANG2DEG(1);
// 	//	adjust_euler[4] = ANG2DEG(1);
// 	//	adjust_euler[5] = ANG2DEG(1);
// 
// #else
// 	if (fabs(Euler[0] - ZEulerDegRef) > ZEulerDegCut)		// 调整Z角
// 	{
// 		adjust_euler[3] = SGN(Euler[0] - ZEulerDegRef)*move_step;
// 		m_CmdStr += "Adjusting Z\r\n";
// 		InsertCmdStr(m_CmdStr);
// 	}
// 	else		// Z角调整完成
// 	{
// 		if (fabs(Euler[1] - YEulerDegRef) > YEulerDegCut)		// 调整Y角
// 		{
// 			adjust_euler[4] = SGN(Euler[1] - YEulerDegRef)*move_step;
// 			m_CmdStr += "Adjusting Y\r\n";
// 			InsertCmdStr(m_CmdStr);
// 		}
// 		else		// Y角调整完成
// 		{
// 			if (fabs(Euler[2] - XEulerDegRef) > XEulerDegCut)		// 调整X角
// 			{
// 				adjust_euler[5] = SGN(Euler[2] - XEulerDegRef)*move_step;
// 				m_CmdStr += "Adjusting X\r\n";
// 				InsertCmdStr(m_CmdStr);
// 			} 
// 			else		// 姿态调整完成
// 			{
// 				for (int i = 0; i < 7; ++i)
// 				{
// 					Quat_New[i] = Quat_Ref[i];
// 				}
// 				m_CmdStr += "Finish\r\n";
// 				InsertCmdStr(m_CmdStr);
// 				return;
// 			}
// 		}
// 	}
// #endif
// 
// 	EulerToTrans(adjust_euler, AdjustMat);
// 
// 	BaseToCamMat_New = BaseToCamMat_Ref*AdjustMat;
// 	BaseToEndMat_New = BaseToCamMat_New*(EndToCamMat.inverse());
// 
// 	mat7x1 Quat_New_Mat;
// 	TransToQuater(BaseToEndMat_New, Quat_New_Mat);
// 	for (int i = 0; i < 7; ++i)
// 	{
// 		Quat_New[i] = Quat_New_Mat(i);
// 	}
// }


// Planning in Euler Representation
void CyberSystem::GetViNextOri(const double ViEulerRecv[], const double Quat_Ref[7], double Quat_New[7])
{
	double move_step = ANG2DEG(1);
	double ZEulerDegCut = EulerPoseCut[0];
	double YEulerDegCut = EulerPoseCut[1];
	double XEulerDegCut = EulerPoseCut[2];
	double ZEulerDegRef = RefEulerPose[0];
	double YEulerDegRef = RefEulerPose[1];
	double XEulerDegRef = RefEulerPose[2];

	double Euler[3];
	for (int i = 0; i < 3; ++i)
	{
		Euler[i] = ViEulerRecv[i+1];
	}
	double Quat_Ref_Rot[4];
	for (int i = 0; i < 4; ++i)
	{
		Quat_Ref_Rot[i] = Quat_Ref[i+3];
	}

	mat4x4 EndToCamMat;		// 机械臂末端到相机的变换矩阵
	EndToCamMat << 0, -1, 0, 0, 
		1, 0, 0, 275, 
		0, 0, 1, 0, 
		0, 0, 0, 1;

	mat3x3 BaseToCamRot_Ref;
	mat3x3 BaseToCamRot_New;
	mat3x3 BaseToEndRot_Ref;
	mat3x3 BaseToEndRot_New;


	// 
	mat4x4 BaseToCamMat_Ref;	// 相机相对于基座
	mat4x4 BaseToCamMat_New;
	mat4x4 BaseToEndMat_Ref;	// 机械臂末端相对于基座
	mat4x4 BaseToEndMat_New;

	QuaterToTrans(Quat_Ref, BaseToEndMat_Ref);
	BaseToCamMat_Ref = BaseToEndMat_Ref*EndToCamMat;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			BaseToCamRot_Ref(i,j) = BaseToCamMat_Ref(i,j);
		}
	}


	double adjust_euler[3] = {0, 0, 0};		// 角度调节欧拉角
	mat3x3 AdjustMat;		// 角度调整矩阵





#if ArmDebug		// 调试使用，每次循环更新绕末端坐标系旋转一个角度
	//adjust_euler[0] = ANG2DEG(1);
	//adjust_euler[1] = ANG2DEG(1);
	//adjust_euler[2] = ANG2DEG(1);

#else
	if (fabs(Euler[0] - ZEulerDegRef) > ZEulerDegCut)		// 调整Z角
	{
		adjust_euler[0] = SGN(Euler[0] - ZEulerDegRef)*move_step;
		m_CmdStr += "Adjusting Z\r\n";
		InsertCmdStr(m_CmdStr);
	}
	else		// Z角调整完成
	{
		if (fabs(Euler[1] - YEulerDegRef) > YEulerDegCut)		// 调整Y角
		{
			adjust_euler[1] = SGN(Euler[1] - YEulerDegRef)*move_step;
			m_CmdStr += "Adjusting Y\r\n";
			InsertCmdStr(m_CmdStr);
		}
		else		// Y角调整完成
		{
			if (fabs(Euler[2] - XEulerDegRef) > XEulerDegCut)		// 调整X角
			{
				adjust_euler[2] = SGN(Euler[2] - XEulerDegRef)*move_step;
				m_CmdStr += "Adjusting X\r\n";
				InsertCmdStr(m_CmdStr);
			} 
			else		// 姿态调整完成
			{
				for (int i = 0; i < 7; ++i)
				{
					Quat_New[i] = Quat_Ref[i];
				}
				m_CmdStr += "Finish\r\n";
				InsertCmdStr(m_CmdStr);
				return;
			}
		}
	}
#endif

	EulerToRot(adjust_euler, AdjustMat);

	BaseToCamRot_New = BaseToCamRot_Ref*AdjustMat;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			BaseToCamMat_New(i, j) = BaseToCamRot_New(i, j);
		}
	}
	BaseToCamMat_New(0,3) = BaseToCamMat_Ref(0,3);
	BaseToCamMat_New(1,3) = BaseToCamMat_Ref(1,3);
	BaseToCamMat_New(2,3) = BaseToCamMat_Ref(2,3);
	BaseToCamMat_New(3,0) = BaseToCamMat_New(3,1) = BaseToCamMat_New(3,2) = 0;
	BaseToCamMat_New(3,3) = 1;

	BaseToEndMat_New = BaseToCamMat_New*(EndToCamMat.inverse());

	mat7x1 Quat_New_Mat;
	TransToQuater(BaseToEndMat_New, Quat_New_Mat);
	for (int i = 0; i < 7; ++i)
	{
		Quat_New[i] = Quat_New_Mat(i);
	}
}

void CyberSystem::ViRecvTrans(const double ViRecv[7], double ViEulerRecv[7])
{
	// flag
	ViEulerRecv[0] = ViRecv[0];

	// position
	for (int i = 0; i < 3; ++i)
	{
		ViEulerRecv[i+4] = ViRecv[i+4];
	}

	// orientation
	mat3x1 Rod;
	mat3x3 CamNowRot;
	mat3x3 CamDesRot;		// 最终相机坐标系下要到达的姿态
	mat3x3 RotCoeff;
	double Euler[3];
	//	double Euler[3];
	RotCoeff << 0, 1, 0,
		1, 0, 0,
		0, 0, -1;

	for (int i = 0; i < 3; ++i)
	{
		Rod(i) = ViRecv[i+1];
	}
	RodToTrans(Rod, CamNowRot);
	CamDesRot = CamNowRot*RotCoeff;
	TransToEuler(CamDesRot, Euler);
	for (int i = 0; i < 3; ++i)
	{
		ViEulerRecv[i+1] = Euler[i];
	}
}


//*********************** Pose Representation Transfer ***********************//
void CyberSystem::QuaterToRot(const double arr_in[4], mat3x3 &mat_out)
{
	// romat_outamat_oute mamat_outrix
	mat_out(0,0) = 2*(pow(arr_in[0], 2) + pow(arr_in[1], 2)) - 1;
	mat_out(0,1) = 2*(arr_in[1]*arr_in[2] - arr_in[0]*arr_in[3]);
	mat_out(0,2) = 2*(arr_in[1]*arr_in[3] + arr_in[0]*arr_in[2]);
	mat_out(1,0) = 2*(arr_in[1]*arr_in[2] + arr_in[0]*arr_in[3]);
	mat_out(1,1) = 2*(pow(arr_in[0], 2) + pow(arr_in[2], 2)) - 1;
	mat_out(1,2) = 2*(arr_in[2]*arr_in[3] - arr_in[0]*arr_in[1]);
	mat_out(2,0) = 2*(arr_in[1]*arr_in[3] - arr_in[0]*arr_in[2]);
	mat_out(2,1) = 2*(arr_in[2]*arr_in[3] + arr_in[0]*arr_in[1]);
	mat_out(2,2) = 2*(pow(arr_in[0], 2) + pow(arr_in[3], 2)) - 1;
}

// Calculate Unit Quaternion
// n = arr_in[3]	ex = arr_in[4]
// ey = arr_in[5]	ez = arr_in[6]
void CyberSystem::QuaterToTrans(const double arr_in[7], mat4x4 &mat_out)
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

void CyberSystem::TransToQuater(const mat4x4 &mat_in, mat7x1 &mat_out)
{
	// Translate Position
	mat_out(0) = mat_in(0,3);
	mat_out(1) = mat_in(1,3);
	mat_out(2) = mat_in(2,3);

	// Translate Orientation
	mat_out(3) = 0.5*sqrt(mat_in(0,0) + mat_in(1,1) + mat_in(2,2) + 1);
	mat_out(4) = 0.5*SGN(mat_in(2,1) - mat_in(1,2))*sqrt(mat_in(0,0) - mat_in(1,1) - mat_in(2,2) + 1);
	mat_out(5) = 0.5*SGN(mat_in(0,2) - mat_in(2,0))*sqrt(mat_in(1,1) - mat_in(2,2) - mat_in(0,0) + 1);
	mat_out(6) = 0.5*SGN(mat_in(1,0) - mat_in(0,1))*sqrt(mat_in(2,2) - mat_in(0,0) - mat_in(1,1) + 1);
}

void CyberSystem::RodToQuater(const mat3x1 &mat_in, mat4x1 &mat_out)
{
	mat_out(3) = 1/(sqrt(1 + pow(mat_in(0), 2) + pow(mat_in(1), 2) + pow(mat_in(2), 2)));
	mat_out(0) = mat_in(0)*mat_out(3);
	mat_out(1) = mat_in(1)*mat_out(3);
	mat_out(2) = mat_in(2)*mat_out(3);
}


void CyberSystem::RodToTrans(const mat3x1 &Rodrg, mat3x3 &Trans)
{
	double theta = Rodrg.norm();
	double unit_ro[3];
	for (int i = 0; i < 3; ++i)
	{
		unit_ro[i] = Rodrg(i)/theta;
	}

	mat3x3 CrossMat;
	CrossMat(0,0) = CrossMat(1,1) = CrossMat(2,2) = 0;
	CrossMat(0,1) = -unit_ro[2];
	CrossMat(0,2) = unit_ro[1];
	CrossMat(1,0) = unit_ro[2];
	CrossMat(1,2) = -unit_ro[0];
	CrossMat(2,0) = -unit_ro[1];
	CrossMat(2,1) = unit_ro[0];

	Trans = Eigen::MatrixXd::Identity(3,3) + sin(theta)*CrossMat + (1 - cos(theta))*CrossMat*CrossMat;
}

void CyberSystem::TransToRod(const mat3x3 &Trans, mat3x1 &Rodrg)
{
	double coeff;
	coeff = 1 + Trans(0,0) + Trans(1,1) + Trans(2,2);
	Rodrg(0) = (Trans(2,1) - Trans(1,2))/coeff;
	Rodrg(1) = (Trans(0,2) - Trans(2,0))/coeff;
	Rodrg(2) = (Trans(1,0) - Trans(0,1))/coeff;
}

// ZYX Euler Angle
void CyberSystem::RodToEuler(const mat3x1 &Rodrg, mat3x1 &Euler)
{
	double middle_value[3];
	middle_value[0] = 2*(Rodrg(0)*Rodrg(1) + Rodrg(2))/(1 + pow(Rodrg(0),2) - pow(Rodrg(1),2) - pow(Rodrg(2),2));
	middle_value[1] = 2*(Rodrg(1) - Rodrg(2)*Rodrg(0))/(1 + pow(Rodrg(0),2) + pow(Rodrg(1),2) + pow(Rodrg(2),2));
	middle_value[2] = 2*(Rodrg(1)*Rodrg(2) + Rodrg(0))/(1 - pow(Rodrg(0),2) - pow(Rodrg(1),2) + pow(Rodrg(2),2));

	Euler(0) = atan(middle_value[0]);
	Euler(1) = asin(middle_value[1]);
	Euler(2) = atan(middle_value[2]);
}


// Orientation calculate, using ZYX euler angle
void CyberSystem::EulerToRot(const double Euler[], mat3x3 &Trans)
{
	double s1 = sin(Euler[0]);
	double c1 = cos(Euler[0]);
	double s2 = sin(Euler[1]);
	double c2 = cos(Euler[1]);
	double s3 = sin(Euler[2]);
	double c3 = cos(Euler[2]);

	Trans(0,0) = c1*c2;
	Trans(0,1) = c1*s2*s3 - s1*c3;
	Trans(0,2) = c1*s2*c3 + s1*s3;
	Trans(1,0) = s1*c2;
	Trans(1,1) = s1*s2*s3 + c1*c3;
	Trans(1,2) = s1*s2*c3 - c1*s3;
	Trans(2,0) = -s2;
	Trans(2,1) = c2*s3;
	Trans(2,2) = c2*c3;
}



void CyberSystem::EulerToTrans(const double Euler[], mat4x4 &Trans)
{
	double s1 = sin(Euler[3]);
	double c1 = cos(Euler[3]);
	double s2 = sin(Euler[4]);
	double c2 = cos(Euler[4]);
	double s3 = sin(Euler[5]);
	double c3 = cos(Euler[5]);

	Trans(0,0) = c1*c2;
	Trans(0,1) = c1*s2*s3 - s1*c3;
	Trans(0,2) = c1*s2*c3 + s1*s3;
	Trans(1,0) = s1*c2;
	Trans(1,1) = s1*s2*s3 + c1*c3;
	Trans(1,2) = s1*s2*c3 - c1*s3;
	Trans(2,0) = -s2;
	Trans(2,1) = c2*s3;
	Trans(2,2) = c2*c3;

	Trans(0,3) = Euler[0];
	Trans(1,3) = Euler[1];
	Trans(2,3) = Euler[2];

	Trans(3,0) = Trans(3,1) = Trans(3,2) = 0;
	Trans(3,3) = 1;
}


void CyberSystem::TransToEuler(const mat3x3 &Trans, double Euler[])
{
	Euler[0] = atan2(Trans(1,0), Trans(0,0));
	Euler[1] = atan2(-Trans(2,0), sqrt(pow(Trans(2,1),2) + pow(Trans(2,2),2)));
	Euler[2] = atan2(Trans(2,1), Trans(2,2));
}


//calculate inverse kinetics
//mat7x1 CyberSystem::CalKine(const mat4x4 &T, double &last_arm_angle, mat7x1 &last_joint_angle)
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
