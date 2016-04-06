#include "cybersystem.h"
#include "qregexp.h"

CyberSystem::CyberSystem(QWidget *parent)
	: QMainWindow(parent), m_DisThread(this)
{
	ui.setupUi(this);

	// set up main widget size
	ui.centralWidget->setFixedSize(1154,575);

	// check out CyberTracker data input
	// You must input point to satisfy the check out
	QRegExp regExp("[-]{0,1}[1-9][0-9]{0,2}[.][1-9][0-9]{0,2}");
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
	bool bRTraTab = ui.m_pRTraTab->isEnabled();
	if (ui.m_pRTraTab->isEnabled())
	{
		QString XPos = ui.m_pRPosXLiEd->text();
		QString YPos = ui.m_pRPosYLiEd->text();
		QString ZPos = ui.m_pRPosZLiEd->text();
		QString XOri = ui.m_pROriXLiEd->text();
		QString YOri = ui.m_pROriYLiEd->text();
		QString ZOri = ui.m_pROriZLiEd->text();

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
	}
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