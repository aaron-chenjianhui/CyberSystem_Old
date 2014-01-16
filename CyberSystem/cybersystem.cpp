#include "cybersystem.h"
#include "qregexp.h"
#include <iostream>

#include "kine7.hpp"

#define DEG2ANG(x) x*180/3.14
#define	ANG2DEG(x) x*3.14/180

extern CRobonautData g_RobotCmdDeg;     //全局机器人控制指令单位角度
extern CRobonautData g_SRobotCmdDeg;    //发送的安全机器人控制指令单位角度
extern CRobonautData g_RobotSensorDeg;  //全局机器人传感器数据单位角度

float g_rightArmJointBuf[7];	// global data from slider for joint control
float g_leftArmJointBuf[7];		// global data from slider for joint control

extern int g_RobotCtrlMode;		// 0 means out of control, 1 means use cyber to control, 2 means use slider to control

//typedef rpp::kine::Kine7<double> kine_type;
rpp::kine::Kine7<double> m_Kine;
rpp::kine::SingularityHandler<double> sh;
rpp::kine::Kine7<double>::angular_interval_vector joint_limits;

CyberSystem::CyberSystem(QWidget *parent)
	: QMainWindow(parent), m_DisThread(this)/*, m_RobonautCtrlThread(this)*/
{
	ui.setupUi(this);

	// set up main widget size
	ui.centralWidget->setFixedSize(1154,640);

	// check out CyberTracker data input
	// You must input point to satisfy the check out
	QRegExp regExp("[-]?[0-9][0-9]{0,2}[.][0-9][0-9]{0,2}|[-]?[0-9][0-9]{0,2}");
	ui.m_pRPosXLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pRPosYLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pRPosZLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pROriXLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pROriYLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pROriZLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLPosXLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLPosYLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLPosZLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLOriXLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLOriYLiEd->setValidator(new QRegExpValidator(regExp, this));
	ui.m_pLOriZLiEd->setValidator(new QRegExpValidator(regExp, this));

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

	// Initialize kinetic calculate
	InitKine();

	// Robonaut joint move management
	ui.m_pRJo0Sli->setRange(ROBO_J0_MIN,ROBO_J0_MAX);
	ui.m_pRJo0SpBox->setRange(ROBO_J0_MIN,ROBO_J0_MAX);	
	ui.m_pRJo1Sli->setRange(ROBO_J1_MIN,ROBO_J1_MAX);
	ui.m_pRJo1SpBox->setRange(ROBO_J1_MIN,ROBO_J1_MAX);
	ui.m_pRJo2Sli->setRange(ROBO_J2_MIN,ROBO_J2_MAX);
	ui.m_pRJo2SpBox->setRange(ROBO_J2_MIN,ROBO_J2_MAX);
	ui.m_pRJo3Sli->setRange(ROBO_J3_MIN,ROBO_J3_MAX);
	ui.m_pRJo3SpBox->setRange(ROBO_J3_MIN,ROBO_J3_MAX);
	ui.m_pRJo4Sli->setRange(ROBO_J4_MIN,ROBO_J4_MAX);
	ui.m_pRJo4SpBox->setRange(ROBO_J4_MIN,ROBO_J4_MAX);
	ui.m_pRJo5Sli->setRange(ROBO_J5_MIN,ROBO_J5_MAX);
	ui.m_pRJo5SpBox->setRange(ROBO_J5_MIN,ROBO_J5_MAX);
	ui.m_pRJo6Sli->setRange(ROBO_J6_MIN,ROBO_J6_MAX);
	ui.m_pRJo6SpBox->setRange(ROBO_J6_MIN,ROBO_J6_MAX);


	ui.m_pLJo0Sli->setRange(ROBO_J0_MIN,ROBO_J0_MAX);
	ui.m_pLJo0SpBox->setRange(ROBO_J0_MIN,ROBO_J0_MAX);	
	ui.m_pLJo1Sli->setRange(ROBO_J1_MIN,ROBO_J1_MAX);
	ui.m_pLJo1SpBox->setRange(ROBO_J1_MIN,ROBO_J1_MAX);
	ui.m_pLJo2Sli->setRange(ROBO_J2_MIN,ROBO_J2_MAX);
	ui.m_pLJo2SpBox->setRange(ROBO_J2_MIN,ROBO_J2_MAX);
	ui.m_pLJo3Sli->setRange(ROBO_J3_MIN,ROBO_J3_MAX);
	ui.m_pLJo3SpBox->setRange(ROBO_J3_MIN,ROBO_J3_MAX);
	ui.m_pLJo4Sli->setRange(ROBO_J4_MIN,ROBO_J4_MAX);
	ui.m_pLJo4SpBox->setRange(ROBO_J4_MIN,ROBO_J4_MAX);
	ui.m_pLJo5Sli->setRange(ROBO_J5_MIN,ROBO_J5_MAX);
	ui.m_pLJo5SpBox->setRange(ROBO_J5_MIN,ROBO_J5_MAX);
	ui.m_pLJo6Sli->setRange(ROBO_J6_MIN,ROBO_J6_MAX);
	ui.m_pLJo6SpBox->setRange(ROBO_J6_MIN,ROBO_J6_MAX);

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
	connect(this, SIGNAL(InsertGloText(const QString &)), ui.m_pDataDisBs, SLOT(setText(const QString &))); // display glove data when calibration begin
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
	connect(ui.m_pSimuConnBtn, SIGNAL(clicked()), this, SLOT(ConsimuConnContr()));
	connect(ui.m_pSimuRunBtn, SIGNAL(clicked()), this, SLOT(CyberSimuControl()));

	// signals and slots for robonaut control
	connect(ui.m_pRoboConnBtn, SIGNAL(clicked()), this, SLOT(RobonautConnCtrl()));
	connect(ui.m_pRoboRunBtn, SIGNAL(clicked()), this, SLOT(RobonautCtrl()));

	// signals and slots for robonaut joint control
	connect(ui.m_pJointCtrlBtn, SIGNAL(clicked()), this, SLOT(RobonautJoCtrl()));

	// signals and slots for sending joint control data
	connect(ui.m_pSdDataBtn, SIGNAL(clicked()), this, SLOT(SendJoData()));

	// signals and slots for slider and spin box
	connect(ui.m_pRJo0Sli, SIGNAL(valueChanged(int)), ui.m_pRJo0SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo0SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo0Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo1Sli, SIGNAL(valueChanged(int)), ui.m_pRJo1SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo1SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo1Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo2Sli, SIGNAL(valueChanged(int)), ui.m_pRJo2SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo2SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo2Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo3Sli, SIGNAL(valueChanged(int)), ui.m_pRJo3SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo3SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo3Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo4Sli, SIGNAL(valueChanged(int)), ui.m_pRJo4SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo4SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo4Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo5Sli, SIGNAL(valueChanged(int)), ui.m_pRJo5SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo5SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo5Sli, SLOT(setValue(int)));
	connect(ui.m_pRJo6Sli, SIGNAL(valueChanged(int)), ui.m_pRJo6SpBox, SLOT(setValue(int)));
	connect(ui.m_pRJo6SpBox, SIGNAL(valueChanged(int)), ui.m_pRJo6Sli, SLOT(setValue(int)));

	connect(ui.m_pLJo0Sli, SIGNAL(valueChanged(int)), ui.m_pLJo0SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo0SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo0Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo1Sli, SIGNAL(valueChanged(int)), ui.m_pLJo1SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo1SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo1Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo2Sli, SIGNAL(valueChanged(int)), ui.m_pLJo2SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo2SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo2Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo3Sli, SIGNAL(valueChanged(int)), ui.m_pLJo3SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo3SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo3Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo4Sli, SIGNAL(valueChanged(int)), ui.m_pLJo4SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo4SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo4Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo5Sli, SIGNAL(valueChanged(int)), ui.m_pLJo5SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo5SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo5Sli, SLOT(setValue(int)));
	connect(ui.m_pLJo6Sli, SIGNAL(valueChanged(int)), ui.m_pLJo6SpBox, SLOT(setValue(int)));
	connect(ui.m_pLJo6SpBox, SIGNAL(valueChanged(int)), ui.m_pLJo6Sli, SLOT(setValue(int)));

	// signals and slots for Control Command Display
	connect(this, SIGNAL(InsetCommStr(const QString &)), this, SLOT(CommadBsDisplay(const QString &)));

	connect(ui.m_pInitBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pInitRHBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pInitRTBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pInitLHBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pInitLTBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));

	connect(ui.m_pGlStartBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pGesBtn_one, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pGesBtn_two, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pGesBtn_three, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pGesBtn_four, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pFinshBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	
	connect(ui.m_pTraStartBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pTraCalBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));

	connect(ui.m_pSimuConnBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pRoboConnBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pSimuRunBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pRoboRunBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pJointCtrlBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pSdDataBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));

	connect(ui.m_pPosRdBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pImpRdBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pResRdBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pInitHandBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));
	connect(ui.m_pRunHandBtn, SIGNAL(clicked()), this, SLOT(CommandStrSelect()));

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

void CyberSystem::InitRTracker()
{
	m_CyberStation.RTraConn();
	m_RTraContr = true;
	ui.m_pTraStartBtn->setEnabled(true);
}

void CyberSystem::InitLTracker()
{
	m_CyberStation.LTraConn();
	m_LTraContr = true;
	ui.m_pTraStartBtn->setEnabled(true);
}



//*********************** CyberGlove Calibration Options ***********************//
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

// 
void CyberSystem::DisData()
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
				m_CyberStation.GetRTraData();
				RTraStr = m_CyberStation.RTraDisData(m_bRTraDisReal);
			}
			if (m_LTraContr == true)
			{
				m_CyberStation.GetLTraData();
				LTraStr = m_CyberStation.LTraDisData(m_bLTraDisReal);
			}
		}
		QString Str = RGloStr + LGloStr + RTraStr + LTraStr;
		emit InsertGloText(Str);		
	}
	// ensure that you will get into the thread next time
	m_DisThread.m_bDisThreadStop = false;
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
void CyberSystem::InitTraCali()
{
	if (QMessageBox::Yes == QMessageBox::question(this, tr("Question"), tr("Start Calibration?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))  
	{  
		// new thread to display raw data
		m_bDisTraData = true;
		m_DisThread.start();
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




//*********************** Consimulation Options ***********************//
void CyberSystem::ConsimuConnContr()
{
	if (ui.m_pSimuConnBtn->text() == "Conn ConSimu")
	{
		m_RobonautControl.InitConsimuConn();
		
		if (m_RobonautControl.m_nRetConSer == 0)
		{
			ui.m_pSimuConnBtn->setText("Disconn ConSimu");
			ui.m_pSimuRunBtn->setEnabled(true);
			ui.m_pJointCtrlBtn->setEnabled(true);
		}
		return;
	}

	if (ui.m_pSimuConnBtn->text() == "Disconn ConSimu")
	{
		m_RobonautControl.DisConsimuConn();

		ui.m_pSimuConnBtn->setText("Conn ConSimu");
		ui.m_pSimuRunBtn->setEnabled(false);
		ui.m_pJointCtrlBtn->setEnabled(false);

		return;
	}
}

void CyberSystem::CyberSimuControl()
{
	g_RobotCtrlMode = 1;
	m_RobonautControl.SimuControl();
}

//*********************** Robonaut Control Options ***********************//
void CyberSystem::RobonautConnCtrl()
{
	if (ui.m_pRoboConnBtn->text() == "Conn Robonaut")
	{
		m_RobonautControl.InitRoboCtrlConn();

		if (m_RobonautControl.m_nRetRoboCtrlSer ==0)
		{
			ui.m_pRoboConnBtn->setText("Disconn Robonaut");
			ui.m_pRoboRunBtn->setEnabled(true);
			ui.m_pJointCtrlBtn->setEnabled(true);
		}
		return;
	}

	if (ui.m_pRoboConnBtn->text() == "Disconn Robonaut")
	{
		m_RobonautControl.DisRoboCtrlConn();

		ui.m_pRoboConnBtn->setText("Conn Robonaut");
		ui.m_pRoboRunBtn->setEnabled(false);
		ui.m_pJointCtrlBtn->setEnabled(false);

		return;
	}
}


void CyberSystem::RobonautCtrl()
{
	g_RobotCtrlMode = 1;
	m_RobonautControl.RoboCtrl();
}


//*********************** Robonaut Joints Control Options ***********************//
void CyberSystem::getSliData()
{
	if (ui.m_pRArmTab->isVisible() || ui.m_pLArmTab->isVisible())
	{
		// 	while(m_RobonautCtrlThread.m_bRoboCtrlThreadStop == false)
		// 	{
		g_rightArmJointBuf[0] = ui.m_pRJo0Sli->value();
		g_rightArmJointBuf[1] = ui.m_pRJo1Sli->value();
		g_rightArmJointBuf[2] = ui.m_pRJo2Sli->value();
		g_rightArmJointBuf[3] = ui.m_pRJo3Sli->value();
		g_rightArmJointBuf[4] = ui.m_pRJo4Sli->value();
		g_rightArmJointBuf[5] = ui.m_pRJo5Sli->value();
		g_rightArmJointBuf[6] = ui.m_pRJo6Sli->value();

		g_leftArmJointBuf[0] = ui.m_pLJo0Sli->value();
		g_leftArmJointBuf[1] = ui.m_pLJo1Sli->value();
		g_leftArmJointBuf[2] = ui.m_pLJo2Sli->value();
		g_leftArmJointBuf[3] = ui.m_pLJo3Sli->value();
		g_leftArmJointBuf[4] = ui.m_pLJo4Sli->value();
		g_leftArmJointBuf[5] = ui.m_pLJo5Sli->value();
		g_leftArmJointBuf[6] = ui.m_pLJo6Sli->value();
		/*	}	*/
	}
	else
	{
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

		Eigen::Matrix<double, 7, 1> q = CalKine(T, m_last_arm_angle);

		// final data
		for (int i = 0; i<7; i++)
		{		
			g_rightArmJointBuf[i] = DEG2ANG(q(i));
		}
	}
}


void CyberSystem::RobonautJoCtrl()
{
/*	m_RobonautCtrlThread.start();*/
	ui.m_pSdDataBtn->setEnabled(true);

	ui.m_pRJo0Sli->setEnabled(true);
	ui.m_pRJo1Sli->setEnabled(true);
	ui.m_pRJo2Sli->setEnabled(true);
	ui.m_pRJo3Sli->setEnabled(true);
	ui.m_pRJo4Sli->setEnabled(true);
	ui.m_pRJo5Sli->setEnabled(true);
	ui.m_pRJo6Sli->setEnabled(true);

	ui.m_pRJo0SpBox->setEnabled(true);
	ui.m_pRJo1SpBox->setEnabled(true);
	ui.m_pRJo2SpBox->setEnabled(true);
	ui.m_pRJo3SpBox->setEnabled(true);
	ui.m_pRJo4SpBox->setEnabled(true);
	ui.m_pRJo5SpBox->setEnabled(true);
	ui.m_pRJo6SpBox->setEnabled(true);

	ui.m_pLJo0Sli->setEnabled(true);
	ui.m_pLJo1Sli->setEnabled(true);
	ui.m_pLJo2Sli->setEnabled(true);
	ui.m_pLJo3Sli->setEnabled(true);
	ui.m_pLJo4Sli->setEnabled(true);
	ui.m_pLJo5Sli->setEnabled(true);
	ui.m_pLJo6Sli->setEnabled(true);

	ui.m_pLJo0SpBox->setEnabled(true);
	ui.m_pLJo1SpBox->setEnabled(true);
	ui.m_pLJo2SpBox->setEnabled(true);
	ui.m_pLJo3SpBox->setEnabled(true);
	ui.m_pLJo4SpBox->setEnabled(true);
	ui.m_pLJo5SpBox->setEnabled(true);
	ui.m_pLJo6SpBox->setEnabled(true);
}


void CyberSystem::SendJoData()
{
	getSliData();

	// robonaut is connected
	if (m_RobonautControl.m_nRetRoboCtrlSer == 0)
	{
		g_RobotCtrlMode = 2;
		m_RobonautControl.RoboCtrl();
	}

	// consimulation is connected
	else if (m_RobonautControl.m_nRetConSer == 0)
	{
		g_RobotCtrlMode = 2;
		m_RobonautControl.SimuControl();
	}

	// error
	else
	{
		QMessageBox::about(NULL, "Error", "Robonaut is not connected");
	}
}


//*********************** Control Command Display ***********************//
// judge sender, and send data respectively
void CyberSystem::CommandStrSelect()
{
	// Connection options
	if (ui.m_pInitBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Haptic Station...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pInitRHBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Right Hand...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pInitRTBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Right Tracker...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pInitLHBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Left Hand...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pInitLTBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Left Tracker...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}

	// Glove calibration options
	if (ui.m_pGlStartBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Start Glove Calibration...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pGesBtn_one == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Save Gesture one data...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pGesBtn_two == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Save Gesture two data...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pGesBtn_three == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Save Gesture three data...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pGesBtn_four == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Save Gesture four data...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pFinshBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Finish Glove Calibration...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	
	// Tracker Calibration options
	if (ui.m_pTraStartBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Start Tracker Calibration...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pTraCalBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Calculate coefficients of Tracker...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}

	// Robonaut control options
	if (ui.m_pSimuConnBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Con-Simulation robonaut...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pRoboConnBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect robonaut...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pSimuRunBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Start Con-Simulation control...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pRoboRunBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Start robonaut control...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pJointCtrlBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Connect Joint control...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pSdDataBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Send Joint data...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}

	// 5 hand control
	if (ui.m_pPosRdBtn == dynamic_cast<QRadioButton*>(sender()))
	{
		QString Str("Hand in Position Control Mode...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pImpRdBtn == dynamic_cast<QRadioButton*>(sender()))
	{
		QString Str("Hand in Impedance Control Mode...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pResRdBtn == dynamic_cast<QRadioButton*>(sender()))
	{
		QString Str("Hand in Reset Control Mode...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pInitHandBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Initialize Hand Control...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
	if (ui.m_pRunHandBtn == dynamic_cast<QPushButton*>(sender()))
	{
		QString Str("Start Hand Control...\r\n\r\n");
		emit(InsetCommStr(Str));
		return;
	}
}

void CyberSystem::CommadBsDisplay(const QString & Str)
{   
	m_CommandString = m_CommandString + Str;
	ui.m_pCommadBs->setText(m_CommandString);
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
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.10, 1.48));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-2.10, 0.01));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.57, 1.57));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.74, 0.01));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.52, 1.97));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.22, 1.22));
	joint_limits.push_back(rpp::kine::Kine7<double>::angular_interval(-1.05, 1.05));
	// Kine7是求解正逆运动学的基础类
	m_Kine = rpp::kine::Kine7<double>(400, 285, 300, 0, joint_limits);
	// SingularHandler处理逆运动学计算中的奇异问题
	sh = rpp::kine::SingularityHandler<double>(joint_limits);
}

Eigen::Matrix<double, 7, 1> CyberSystem::CalKine(const Eigen::Matrix<double, 4, 4> &T, double &last_arm_angle)
{
	auto self_motions = m_Kine.inverse(T);
	for (auto it = self_motions.begin(); it != self_motions.end(); ++it)
	{
		// 自运动必须在关节限位以内
		if (joint_limits[3].contains(it->elbow_joint()))
		{
			auto arm_angle_ranges = it->arm_angle_range();
			auto it_arm_angle = arm_angle_ranges.begin();
			auto ave = abs((it_arm_angle->lower() + it_arm_angle->upper())/2 - last_arm_angle);

			// 取和上一次臂角最为相近的臂角
			for (auto range = arm_angle_ranges.begin(); range != arm_angle_ranges.end(); ++range)
			{
				auto tmp_ave = abs((range->lower() + range->upper())/2 - last_arm_angle);
				if (tmp_ave < ave)
				{
					it_arm_angle = range;
					ave = tmp_ave;
				}
			}

			auto joints = it->get_joints(ave, sh);
			last_arm_angle = ave;
			for (auto it2 = joints.begin(); it2 != joints.end(); ++it2)
			{
				auto q = *(it2);
				auto q_valid = true;
				for (int i = 0; i < 7; ++i)
				{
					q_valid = q_valid && joint_limits[i].contains(q[i]);
				}
				if (!q_valid)
				{
					std::cout << "joint angle out of range: " << q << std::endl;
					exit(-1);
				}
				// 检查末端位置是否与T重合
				auto TT = m_Kine.forward(q);
				if ((TT - T).norm() > 0.05)
				{
					std::cout << "tool tip position out of range: " << std::endl << T << std::endl
						<< TT << std::endl; 
					std::cout << "joints: " << q.transpose() << std::endl;
					exit(-1);
				}
				else
				{
					return q;
				}
			}
		}
	}
}

