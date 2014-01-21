#include "CyberStation.h"

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"
#include "rapidxml/rapidxml_utils.hpp"

#include <math.h>

#define SINANG(X) sin(X/180*3.1415926)
#define COSANG(X) cos(X/180*3.1415926)

// endpoint position for calibration
#define END_POS_X 100
#define END_POS_Y 100
#define END_POS_Z 100
#define END_ORI_X 0
#define END_ORI_Y 0
#define END_ORI_Z 0



// declare global variable for interconnection
float rTrackerRealPose[6];
float lTrackerRealPose[6];
float m_rGloveRealData[5][3];
float m_lGloveRealData[5][3];


CyberStation::CyberStation()
{
	//******************Initialize all data******************//
	// rightTracker
	m_pRightTrackerConn = NULL;
	m_pRightTracker = NULL;
	m_pRightRcvr = NULL;
	// leftTracker
	m_pLeftTrackerConn = NULL;
	m_pLeftTracker = NULL;
	m_pLeftRcvr = NULL;
	// rightGrasp
	m_pRightGraspConn = NULL;
	m_pRightGrasp = NULL;
	// leftGrasp
	m_pLeftGraspConn = NULL;
	m_pLeftGrasp = NULL;
	// rightGlove
	m_pRightGloveConn = NULL;
	m_pRightGlove = NULL;
	// leftGlove
	m_pLeftGloveConn = NULL;
	m_pLeftGlove = NULL;

	// raw pose data and real pose data for tracker
	for (int i = 0; i<6; i++)
	{
		rTrackerRawPose[i] = 0;
		rTrackerRealPose[i] = 0;
		rTrans[i] = 0;
		lTrackerRawPose[i] = 0;
		lTrackerRealPose[i] = 0;
		lTrans[i] = 0;
	}
	
	// raw data and real data for glove
	for (int i = 0; i<5; i++)
	{
		for (int j = 0; j<4; j++)
		{
			m_rGloveRawData[i][j] = 0;
			m_rGloveRealData[i][j] = 0;
			m_lGloveRawData[i][j] = 0;
			m_lGloveRealData[i][j] = 0;
			m_GesOneData[i][j] = 0;
			m_GesTwoData[i][j] = 0;
			m_GesThrData[i][j] = 0;
			m_GesFourData[i][j] = 0;
			m_GloCaliK[i][j] = 0;
			m_GloCaliB[i][j] = 0;
		}
	}

	m_file.open("test.txt");

// 	// true means CyberGlove calibration is over
// 	bGloCaliContr = false;
// 	// true means getting calibraton data is over
// 	bGesOneContr = false;
// 	bGesTwoContr = false;
// 	bGesThrContr = false;
// 	bGesFourContr = false;
}

CyberStation::~CyberStation()
{
	m_file.close();
}


//****************************** Connection Management ******************************//
void CyberStation::RTraConn()
{
	// Connect to the rightTracker
	m_pRightTrackerConn = vhtIOConn::getDefault("RightForce");
	m_pRightTracker = NULL;
	try
	{
		m_pRightTracker = new vhtTracker(m_pRightTrackerConn);
	}
	catch(vhtBaseException *e)
	{
		DisplayMessage("Error with rightTracker: ", e);
		return;
	}

	// Extract the receiver 0,1 of the rightTracker
	m_pRightRcvr = m_pRightTracker->getLogicalDevice(0);
}

void CyberStation::LTraConn()
{
	// Connect to the rightTracker
	m_pLeftTrackerConn = vhtIOConn::getDefault("LeftForce");
	m_pLeftTracker = NULL;
	try
	{
		m_pLeftTracker = new vhtTracker(m_pLeftTrackerConn);
	}
	catch(vhtBaseException *e)
	{
		DisplayMessage("Error with leftTracker: ", e);
		return;
	}

	// Extract the receiver 0,1 of the rightTracker
	m_pLeftRcvr = m_pLeftTracker->getLogicalDevice(0);	
}

void CyberStation::RHandConn()
{
	// Connect to the right grasp
	m_pRightGraspConn = vhtIOConn::getDefault( "RightGrasp" );
	try
	{
		m_pRightGrasp = new vhtCyberGrasp( m_pRightGraspConn );
	}
	catch ( vhtBaseException *e )
	{
		DisplayMessage("Error with rightGrasp: ", e);
		return;
	}
	m_pRightGrasp->setMode( GR_CONTROL_FORCE );


	// Connect to right glove
	//GHM::Handedness gloveHandedness = GHM::rightHand;
	try{
		m_pRightGloveConn = vhtIOConn::getDefault( "RightGlove" );
		m_pRightGlove = new vhtCyberGlove(m_pRightGloveConn,true);
	}
	catch (vhtBaseException* e){
		DisplayMessage("Error with CyberGlove: ", e);
	}
}

void CyberStation::LHandConn()
{
	// Connect to the right grasp
	m_pLeftGraspConn = vhtIOConn::getDefault( "LeftGrasp" );
	try
	{
		m_pLeftGrasp = new vhtCyberGrasp( m_pLeftGraspConn );
	}
	catch ( vhtBaseException *e )
	{
		DisplayMessage("Error with leftGrasp: ", e);
		return;
	}
	m_pLeftGrasp->setMode( GR_CONTROL_FORCE );


	// Connect to left glove
	//GHM::Handedness gloveHandedness = GHM::leftHand;
	try{
		m_pLeftGloveConn = vhtIOConn::getDefault( "LeftGlove" );
		m_pLeftGlove = new vhtCyberGlove(m_pLeftGloveConn,true);
	}
	catch (vhtBaseException* e){
		DisplayMessage("Error with CyberGlove: ", e);
	}
}

void CyberStation::DisplayMessage(const char *msg, vhtBaseException *e)
{
	char buf[200];
	strcpy(buf, msg);
	strcat(buf, e->getMessage());
	QMessageBox::about(NULL, "Error", buf);
	exit(0);
}
//****************************** Connection Management is over ******************************//




//****************************** CyberGlove Calibration ******************************//
// 左右手读数的原始数据是否一致，看看能否只标定一个手套数据
void CyberStation::GetRGloData()
{
	m_pRightGlove->update();
	for(int finger=0;finger<GHM::nbrFingers;finger++ )
	{ 		
		for(int joint=0;joint<=GHM::nbrJoints;joint++ )
		{
			m_rGloveRawData[finger][joint] = m_pRightGlove->getRawData((GHM::Fingers)finger,(GHM::Joints)joint);
		}
	}
}

void CyberStation::GetLGloData()
{
	m_pRightGlove->update();
	for(int finger=0;finger<GHM::nbrFingers;finger++ )
	{ 		
		for(int joint=0;joint<=GHM::nbrJoints;joint++ )
		{
			m_lGloveRawData[finger][joint] = m_pLeftGlove->getRawData((GHM::Fingers)finger,(GHM::Joints)joint);
		}
	}
}


QString CyberStation::GloDisData(bool bGloDisReal)
{
	QString str;

	if (bGloDisReal == false)
	{
		str = QString("\r\nThumbRawData: %1, %2, %3\r\nIndexRawData: %4, %5, %6\r\nMiddleRawData: %7, %8, %9\r\nRingRawData: %10, %11, %12\r\nLittleRawData: %13, %14, %15\r\n")
			.arg(m_rGloveRawData[0][1]).arg(m_rGloveRawData[0][2]).arg(m_rGloveRawData[0][3])
			.arg(m_rGloveRawData[1][0]).arg(m_rGloveRawData[1][1]).arg(m_rGloveRawData[2][3])
			.arg(m_rGloveRawData[2][0]).arg(m_rGloveRawData[2][1]).arg(m_rGloveRawData[2][3])
			.arg(m_rGloveRawData[3][0]).arg(m_rGloveRawData[3][1]).arg(m_rGloveRawData[3][3])
			.arg(m_rGloveRawData[4][0]).arg(m_rGloveRawData[4][1]).arg(m_rGloveRawData[4][3]);
	} 
	else
	{
		str = QString("\r\nThumbRealData: %1, %2, %3\r\nIndexRealData: %4, %5, %6\r\nMiddleRealData: %7, %8, %9\r\nRingRealData: %10, %11, %12\r\nLittleRealData: %13, %14, %15\r\n")
			.arg(m_rGloveRealData[0][0]).arg(m_rGloveRealData[0][1]).arg(m_rGloveRealData[0][2])
			.arg(m_rGloveRealData[1][0]).arg(m_rGloveRealData[1][1]).arg(m_rGloveRealData[1][2])
			.arg(m_rGloveRealData[2][0]).arg(m_rGloveRealData[2][1]).arg(m_rGloveRealData[2][2])
			.arg(m_rGloveRealData[3][0]).arg(m_rGloveRealData[3][1]).arg(m_rGloveRealData[3][2])
			.arg(m_rGloveRealData[4][0]).arg(m_rGloveRealData[4][1]).arg(m_rGloveRealData[4][2]);
	}
	return str;
}


void CyberStation::GetGesOneData()
{
	m_GesOneData[0][0] = m_rGloveRawData[0][1];
	m_GesOneData[0][1] = m_rGloveRawData[0][2];
	m_GesOneData[0][2] = m_rGloveRawData[0][3];

	m_GesOneData[1][0] = m_rGloveRawData[1][0];
	m_GesOneData[1][1] = m_rGloveRawData[1][1];

	m_GesOneData[2][0] = m_rGloveRawData[2][0];
	m_GesOneData[2][1] = m_rGloveRawData[2][1];
	m_GesOneData[2][2] = m_rGloveRawData[2][3];

	m_GesOneData[3][0] = m_rGloveRawData[3][0];
	m_GesOneData[3][1] = m_rGloveRawData[3][1];
	m_GesOneData[3][2] = m_rGloveRawData[3][3];

	m_GesOneData[4][0] = m_rGloveRawData[4][0];
	m_GesOneData[4][1] = m_rGloveRawData[4][1];
	m_GesOneData[4][2] = m_rGloveRawData[4][3];

//	bGesOneContr = true;
}

void CyberStation::GetGesTwoData()
{
	m_GesTwoData[0][0] = m_rGloveRawData[0][2];

	m_GesTwoData[1][0] = m_rGloveRawData[1][0];
	m_GesTwoData[1][1] = m_rGloveRawData[1][1];

	m_GesTwoData[2][0] = m_rGloveRawData[2][0];
	m_GesTwoData[2][1] = m_rGloveRawData[2][1];

	m_GesTwoData[3][0] = m_rGloveRawData[3][0];
	m_GesTwoData[3][1] = m_rGloveRawData[3][1];

	m_GesTwoData[4][0] = m_rGloveRawData[4][0];
	m_GesTwoData[4][1] = m_rGloveRawData[4][1];

//	bGesTwoContr = true;
}

void CyberStation::GetGesThrData()
{
	m_GesThrData[0][0] = m_rGloveRawData[0][1];
	m_GesThrData[0][1] = m_rGloveRawData[0][2];
	m_GesThrData[0][2] = m_rGloveRawData[0][3];

	m_GesThrData[1][0] = m_rGloveRawData[1][0];
	m_GesThrData[1][1] = m_rGloveRawData[1][1];

	m_GesThrData[2][0] = m_rGloveRawData[2][0];
	m_GesThrData[2][1] = m_rGloveRawData[2][1];
	m_GesThrData[2][2] = m_rGloveRawData[2][3];
 
	m_GesThrData[3][0] = m_rGloveRawData[3][0];
	m_GesThrData[3][1] = m_rGloveRawData[3][1];
	m_GesThrData[3][2] = m_rGloveRawData[3][3];

	m_GesThrData[4][0] = m_rGloveRawData[4][0];
	m_GesThrData[4][1] = m_rGloveRawData[4][1];
	m_GesThrData[4][2] = m_rGloveRawData[4][3];

//	bGesThrContr = true;
}

void CyberStation::GetGesFourData()
{
	m_GesFourData[0][0] = m_rGloveRawData[0][1];

//	bGesFourContr = true;
}

void CyberStation::CalGloCoef()
{
	//****************************** Calculate Coefficient ******************************//
	// calculate the bent angles
	float HandAcces = 75.0 - 0.0;

	for(int i=0;i<5;i++)
	{
		for(int j=0; j<2; j++)
		{
			m_GloCaliK[i][j] = HandAcces/(m_GesTwoData[i][j] - m_GesThrData[i][j]);
			m_GloCaliB[i][j] = 0.0 - m_GloCaliK[i][j]*m_GesThrData[i][j];
		}
	}
	m_GloCaliK[0][0] = HandAcces/(m_GesFourData[0][0] - m_GesThrData[0][0]);
	m_GloCaliB[0][0] = 0.0 - m_GloCaliK[0][0]*m_GesThrData[0][0];

	// calculate side rotation angles
	m_GloCaliK[0][2] = 0;
	m_GloCaliB[0][2] = 0;
	m_GloCaliK[1][2] = 15.0/(m_GesThrData[2][2] - m_GesOneData[2][2]);
	m_GloCaliB[1][2] = 0.0 - m_GloCaliK[1][2]*m_GesThrData[2][2];
	m_GloCaliK[2][2] = 0;
	m_GloCaliB[2][2] = 0;
	m_GloCaliK[3][2] = 5.0/(m_GesThrData[3][2] - m_GesOneData[3][2]);
	m_GloCaliB[3][2] = 0.0 - m_GloCaliK[3][2]*m_GesThrData[3][2];
	m_GloCaliK[4][2] = 15.0/(m_GesThrData[4][2] - m_GesOneData[4][2]);
	m_GloCaliB[4][2] = 0.0 - m_GloCaliK[4][2]*m_GesThrData[4][2];


	//****************************** Write in XML ******************************//
	char ch_Correlation[5][3][8],ch_Bias[5][3][8];

	for (int i=0;i<5;i++){
		for(int j=0;j<=2;j++){
			gcvt(m_GloCaliK[i][j],5,ch_Correlation[i][j]);
			gcvt(m_GloCaliB[i][j],5,ch_Bias[i][j]);
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

	QFile XmlFile("Calibration.xml");
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


void CyberStation::RealGloData(bool bGloCaliFin)
{	
	if (bGloCaliFin == true)
	{
		//******************************* Read in XML *******************************//
		rapidxml::file<> fCalibration("Calibration.xml");

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
		m_GloCaliK[0][0] = atof(CorThumbPIP->value());
		rapidxml::xml_node<>* CorThumbMP = CorThumb->first_node("MP");
		m_GloCaliK[0][1] = atof(CorThumbMP->value());
		rapidxml::xml_node<>* CorThumbABP = CorThumb->first_node("ABP");
		m_GloCaliK[0][2] = atof(CorThumbABP->value());
		//食指
		rapidxml::xml_node<>* CorIndex = CorrelationMatrix->first_node("Index");
		rapidxml::xml_node<>* CorIndexPIP = CorIndex->first_node("PIP");
		m_GloCaliK[1][0] = atof(CorIndexPIP->value());
		rapidxml::xml_node<>* CorIndexMP = CorIndex->first_node("MP");
		m_GloCaliK[1][1] = atof(CorIndexMP->value());
		rapidxml::xml_node<>* CorIndexABP = CorIndex->first_node("ABP");
		m_GloCaliK[1][2] = atof(CorIndexABP->value());
		//中指
		rapidxml::xml_node<>* CorMiddle = CorrelationMatrix->first_node("Middle");
		rapidxml::xml_node<>* CorMiddlePIP = CorMiddle->first_node("PIP");
		m_GloCaliK[2][0] = atof(CorMiddlePIP->value());
		rapidxml::xml_node<>* CorMiddleMP = CorMiddle->first_node("MP");
		m_GloCaliK[2][1] = atof(CorMiddleMP->value());
		rapidxml::xml_node<>* CorMiddleABP = CorMiddle->first_node("ABP");
		m_GloCaliK[2][2] = atof(CorMiddleABP->value());
		//无名指
		rapidxml::xml_node<>* CorRing = CorrelationMatrix->first_node("Ring");
		rapidxml::xml_node<>* CorRingPIP = CorRing->first_node("PIP");
		m_GloCaliK[3][0] = atof(CorRingPIP->value());
		rapidxml::xml_node<>* CorRingMP = CorRing->first_node("MP");
		m_GloCaliK[3][1] = atof(CorRingMP->value());
		rapidxml::xml_node<>* CorRingABP = CorRing->first_node("ABP");
		m_GloCaliK[3][2] = atof(CorRingABP->value());
		//小指
		rapidxml::xml_node<>* CorLittle = CorrelationMatrix->first_node("Little");
		rapidxml::xml_node<>* CorLittlePIP = CorLittle->first_node("PIP");
		m_GloCaliK[4][0] = atof(CorLittlePIP->value());
		rapidxml::xml_node<>* CorLittleMP = CorLittle->first_node("MP");
		m_GloCaliK[4][1] = atof(CorLittleMP->value());
		rapidxml::xml_node<>* CorLittleABP = CorLittle->first_node("ABP");
		m_GloCaliK[4][2] = atof(CorLittleABP->value());

		//获取偏置矩阵
		rapidxml::xml_node<>* BiasMatrix = GloveCalibration->first_node("BiasMatrix");
		rapidxml::xml_attribute<>* BiasMatrix_DataType = BiasMatrix->first_attribute();
		//拇指
		rapidxml::xml_node<>* BiasThumb = BiasMatrix->first_node("Thumb");
		rapidxml::xml_node<>* BiasThumbPIP = BiasThumb->first_node("PIP");
		m_GloCaliB[0][0] = atof(BiasThumbPIP->value());
		rapidxml::xml_node<>* BiasThumbMP = BiasThumb->first_node("MP");
		m_GloCaliB[0][1] = atof(BiasThumbMP->value());
		rapidxml::xml_node<>* BiasThumbABP = BiasThumb->first_node("ABP");
		m_GloCaliB[0][2] = atof(BiasThumbABP->value());
		//食指
		rapidxml::xml_node<>* BiasIndex = BiasMatrix->first_node("Index");
		rapidxml::xml_node<>* BiasIndexPIP = BiasIndex->first_node("PIP");
		m_GloCaliB[1][0] = atof(BiasIndexPIP->value());
		rapidxml::xml_node<>* BiasIndexMP = BiasIndex->first_node("MP");
		m_GloCaliB[1][1] = atof(BiasIndexMP->value());
		rapidxml::xml_node<>* BiasIndexABP = BiasIndex->first_node("ABP");
		m_GloCaliB[1][2] = atof(BiasIndexABP->value());
		//中指
		rapidxml::xml_node<>* BiasMiddle = BiasMatrix->first_node("Middle");
		rapidxml::xml_node<>* BiasMiddlePIP = BiasMiddle->first_node("PIP");
		m_GloCaliB[2][0] = atof(BiasMiddlePIP->value());
		rapidxml::xml_node<>* BiasMiddleMP = BiasMiddle->first_node("MP");
		m_GloCaliB[2][1] = atof(BiasMiddleMP->value());
		rapidxml::xml_node<>* BiasMiddleABP = BiasMiddle->first_node("ABP");
		m_GloCaliB[2][2] = atof(BiasMiddleABP->value());
		//无名指
		rapidxml::xml_node<>* BiasRing = BiasMatrix->first_node("Ring");
		rapidxml::xml_node<>* BiasRingPIP = BiasRing->first_node("PIP");
		m_GloCaliB[3][0] = atof(BiasRingPIP->value());
		rapidxml::xml_node<>* BiasRingMP = BiasRing->first_node("MP");
		m_GloCaliB[3][1] = atof(BiasRingMP->value());
		rapidxml::xml_node<>* BiasRingABP = BiasRing->first_node("ABP");
		m_GloCaliB[3][2] = atof(BiasRingABP->value());
		//小指
		rapidxml::xml_node<>* BiasLittle = BiasMatrix->first_node("Little");
		rapidxml::xml_node<>* BiasLittlePIP = BiasLittle->first_node("PIP");
		m_GloCaliB[4][0] = atof(BiasLittlePIP->value());
		rapidxml::xml_node<>* BiasLittleMP = BiasLittle->first_node("MP");
		m_GloCaliB[4][1] = atof(BiasLittleMP->value());
		rapidxml::xml_node<>* BiasLittleABP = BiasLittle->first_node("ABP");
		m_GloCaliB[4][2] = atof(BiasLittleABP->value());

		// bent angle
		for (int i = 0; i<5; i++)
		{
			for (int j = 0; j<2; j++)
			{
				m_rGloveRealData[i][j] = m_GloCaliK[i][j]*m_rGloveRawData[i][j] + m_GloCaliB[i][j];
			}
		}
		m_rGloveRealData[0][0] = m_GloCaliK[0][0]*m_rGloveRawData[0][1] + m_GloCaliB[0][0];
		m_rGloveRealData[0][1] = m_GloCaliK[0][1]*m_rGloveRawData[0][2] + m_GloCaliB[0][1];	
		// side rotation angle
		for (int i =0; i<5; i++)
		{
			m_rGloveRealData[i][2] = m_GloCaliK[i][2]*m_rGloveRawData[i][3] + m_GloCaliB[i][2];
		}
		m_rGloveRealData[1][2] = m_GloCaliK[1][2]*m_rGloveRawData[2][3] + m_GloCaliB[1][2];
		m_rGloveRealData[4][2] = m_GloCaliK[4][2]*m_rGloveRawData[4][3] + m_GloCaliB[4][2] + m_rGloveRealData[3][2];

		// 	// side rotation angle is calculated without calibration
		// 	m_rGloveRealData[0][2] = 20*(m_rGloveRawData[0][3] - 109)/(70 - 172);
		// 	m_rGloveRealData[1][2] = 8*(m_rGloveRawData[2][3] - 145)/(74 - 145);
		// 	m_rGloveRealData[2][2] = -5*(m_rGloveRawData[2][3] - 145)/(74 - 145) + (m_rGloveRawData[4][3] - 155)/(155 - 116);
		// 	if ((m_rGloveRawData[4][3] - 155) <= 0)
		// 	{
		// 		m_rGloveRealData[3][2] = 4*(m_rGloveRawData[4][3] - 155)/(155 - 116);
		// 		m_rGloveRealData[4][2] = 10*(m_rGloveRawData[4][3] - 155)/(155 - 116);
		// 	} 
		// 	else
		// 	{
		// 		m_rGloveRealData[3][2] = 4*(m_rGloveRawData[4][3] - 155)/(155 - 116);
		// 		m_rGloveRealData[4][2] = 0.9*4*(m_rGloveRawData[4][3] - 155)/(155 - 116);
		// 	}

		// protections
		for (int i=0;i<5;i++)
		{
			if(fabs(m_rGloveRealData[i][2]) < 1)
			{
				m_rGloveRealData[i][2] = 0;
			}
		}

		if (m_rGloveRealData[2][2] < -3)
		{
			m_rGloveRealData[2][2] = -3;
		}
		if (m_rGloveRealData[2][2] > 3)
		{
			m_rGloveRealData[2][2] = 3;
		}

		for(int i=0;i<5;i++)
		{
			if(m_rGloveRealData[i][1] <= 8.0)
			{
				m_rGloveRealData[i][1] = 8.0;
			}
		}

		for (int i=0;i<5;i++)
		{
			if (m_rGloveRealData[i][0] <= 5.0)
			{
				m_rGloveRealData[i][0] = 5.0;
			}
		}

		for(int i=0;i<5;i++)
		{
			for(int j=0; j<2; j++)
			{
				if(m_rGloveRealData[i][j] >= 75)
					m_rGloveRealData[i][j] = 75;
			}
		}

		if (m_rGloveRealData[0][0] >= 40)
		{
			m_rGloveRealData[0][0] = 40;
		}

		if (m_rGloveRealData[1][0] >= 45)
		{
			m_rGloveRealData[1][0] = 45;
		}

		if (m_rGloveRealData[2][0] >= 60)
		{
			m_rGloveRealData[2][0] = 60;
		}

		// 	if (!g_TouchForceCtrl){
		// 		double forces[6] = {0};
		// 		int switchVa[6] = {0};
		// 
		// 		for (int i=0;i<5;i++){
		// 			switchVa[i] = (fabs(pDlg->_FingerTorque[i][1]) > 0.1) ? 1:0;
		// 		}
		// 		for (int i=0;i<5;i++){
		// 			forces[i] = switchVa[i]*0.5*fabs(pDlg->_FingerTorque[i][1]);
		// 		}
		// 		pDlg->_pTouch->setVibrationAmplitude(forces);
		// 	}
	}
	else
	{}
}
//****************************** CyberGlove Calibration is over ******************************//



//****************************** CyberTracker Calibration ******************************//
// get raw data from CyberTracker
void CyberStation::GetRRawTraData(double RT_arr[6], int RT_size)
{
	m_pRightRcvr->update();
	for (int i = 0; i<RT_size; i++)
	{
		RT_arr[i] = m_pRightRcvr->getRawData((vht6DofDevice::Freedom)i);
	}
}

void CyberStation::GetLRawTraData(double LT_arr[6], int LT_size)
{
	m_pLeftRcvr->update();
	for (int i = 0; i<LT_size; i++)
	{
		LT_arr[i] = m_pLeftRcvr->getRawData((vht6DofDevice::Freedom)i);
	}
}

// TODO(CJH): Use vector to store calibration data
// Replace the old calibration function
void CyberStation::CalTrackerCoef(tracali_type r_data, tracali_type l_data)
{
	// read data
	auto r_it = r_data.end();
	double* r_realpos_first = r_it->real_pos;
	double* r_realori_first = r_it->real_ori;
	double* r_rawpos_first = r_it->raw_pos;
	double* r_rawori_first = r_it->raw_ori;
	r_it--;
	double* r_realpos_second = r_it->real_pos;
	double* r_realori_second = r_it->real_ori;
	double* r_rawpos_second = r_it->raw_pos;
	double* r_rawori_second = r_it->raw_ori;

	auto l_it = l_data.end();
	double* l_realpos_first = l_it->real_pos;
	double* l_realori_first = l_it->real_ori;
	double* l_rawpos_first = l_it->raw_pos;
	double* l_rawori_first = l_it->raw_ori;
	l_it--;
	double* l_realpos_second = l_it->real_pos;
	double* l_realori_second = l_it->real_ori;
	double* l_rawpos_second = l_it->raw_pos;
	double* l_rawori_second = l_it->raw_ori;

	// calculate coefficients
	//m_rTraCoef;
	//m_lTraCoef;
}



void CyberStation::GetRTraData()
{
	m_pRightRcvr->update();
	for (int i = 0; i < 3; i++)
	{
		rTrackerRawPose[i] = m_pRightRcvr->getRawData((vht6DofDevice::Freedom)i);
	}
	for (int i = 3; i < 6; i++)
	{
		rTrackerRawPose[i] = m_pRightRcvr->getRawData((vht6DofDevice::Freedom)i);
		rTrackerRawPose[i] = rTrackerRawPose[i]*180/3.14;
	}
	m_pRightTracker->getLogicalDevice(0)->getTransform(&trackerXForm);
	trackerXForm.getTransform(FormMat);
	
//	m_pRightTracker->getLogicalDevice(0)->setRawData()

}

void CyberStation::GetLTraData()
{
	m_pLeftRcvr->update();
	for (int i = 0; i<6; i++)
	{
		lTrackerRawPose[i] = m_pLeftRcvr->getRawData((vht6DofDevice::Freedom)i);
	}
}

// Recieve six pose data, calculate translation matrix
Eigen::Matrix<double, 4, 4> CyberStation::CalTransMat(const double Trans[])
{
	Eigen::Matrix<double, 4, 4> TransMat;

	// Positon calculate
	TransMat(0,3) = Trans[0];
	TransMat(1,3) = Trans[1];
	TransMat(2,3) = Trans[2];

	// Orientation calculate, using ZYX euler angle
	float s1 = SINANG(Trans[4]);
	float c1 = COSANG(Trans[4]);
	float s2 = SINANG(Trans[4]);
	float c2 = COSANG(Trans[4]);
	float s3 = SINANG(Trans[3]);
	float c3 = COSANG(Trans[3]);

	TransMat(0,0) = c1*c2;
	TransMat(0,1) = c1*s2*s3 - s1*c3;
	TransMat(0,2) = c1*s2*c3 + s1*s3;
	TransMat(1,0) = s1*c2;
	TransMat(1,1) = s1*s2*s3 + c1*c3;
	TransMat(1,2) = s1*s2*c3 - c1*s3;
	TransMat(2,0) = -s2;
	TransMat(2,1) = c2*s3;
	TransMat(2,2) = c2*c3;

	// vision transformation
	TransMat(3,0) = 0;
	TransMat(3,1) = 0;
	TransMat(3,2) = 0;
	TransMat(3,3) = 1;

	return TransMat;
}

// calculate the transformation matrix
void CyberStation::CalTraCoef(bool bRTraisChecked, QString PosX, QString PosY, QString PosZ, QString OriX, QString OriY, QString OriZ)
{
	if (bRTraisChecked == true)
	{
		double RPosX = PosX.toDouble();
		double RPosY = PosY.toDouble();
		double RPosZ = PosZ.toDouble();
		double ROriX = OriX.toDouble();
		double ROriY = OriY.toDouble();
		double ROriZ = OriZ.toDouble();

		// position data of a special pose
		RPosX = 0;
		RPosY = -33.7;
		RPosZ = 30.7;

		// orient data of a special pose
		Eigen::Matrix<double, 3, 3> ROriMat;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				ROriMat(i, j) = 0;
			}
		}
		ROriMat(0, 0) = 1;
		ROriMat(1, 2) = 1;
		ROriMat(2, 1) = -1;

		// position matrix calculate
		TransCoeffMat(0, 0) = TransCoeffMat(0, 2) = 0;
		TransCoeffMat(1, 0) = TransCoeffMat(1, 1) = 0;
		TransCoeffMat(2, 1) = TransCoeffMat(2, 2) = 0;
		TransCoeffMat(3, 0) = TransCoeffMat(3, 1) = TransCoeffMat(3, 2) = 0;
		TransCoeffMat(0, 1) = TransCoeffMat(1, 2) = TransCoeffMat(2, 0) = TransCoeffMat(3, 3) = 1;
		TransCoeffMat(0, 3) = RPosX - rTrackerRawPose[1];
		TransCoeffMat(1, 3) = RPosY - rTrackerRawPose[2];
		TransCoeffMat(2, 3) = RPosZ - rTrackerRawPose[0];

		// orientation matrix calculate
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				RotOnPMat(i, j) = ROriMat(i, j);
			}
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				RotOOnMat(i, j) = 0;
			}
		}
		RotOOnMat(0, 1) = 1;
		RotOOnMat(1, 2) = 1;
		RotOOnMat(2, 0) = 1;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				RotOSMat(i, j) = FormMat[i][j];
			}
		}

		RotCoeffMat = RotOSMat.inverse()*RotOOnMat*RotOnPMat;

// 		m_file << "Start to Calculate:" << std::endl;
// 		for (int i = 0; i < 3; i++)
// 		{
// 			for (int j = 0; j < 3; j++)
// 			{
// 				m_file << FormMat[i][j] << " ";
// 			}
// 			m_file << std::endl;
// 		}
// 		m_file << std::endl;
	} 
	else
	{
		double LPosX = PosX.toDouble();
		double LPosY = PosY.toDouble();
		double LPosZ = PosZ.toDouble();
		double LOriX = OriX.toDouble();
		double LOriY = OriY.toDouble();
		double LOriZ = OriZ.toDouble();

// 		lTrans[0] = LPosX - 0.01*lTrackerRawPose[0];
// 		lTrans[1] = LPosY - 0.01*lTrackerRawPose[1];
// 		lTrans[2] = LPosZ - 0.01*lTrackerRawPose[2];
// 		lTrans[3] = LOriX - 0.01*lTrackerRawPose[3];
// 		lTrans[4] = LOriY - 0.01*lTrackerRawPose[4];
// 		lTrans[5] = LOriZ - 0.01*lTrackerRawPose[5];

		lTrans[0] = LPosX - lTrackerRawPose[0];
		lTrans[1] = LPosY - lTrackerRawPose[1];
		lTrans[2] = LPosZ - lTrackerRawPose[2];
		lTrans[3] = LOriX - lTrackerRawPose[3];
		lTrans[4] = LOriY - lTrackerRawPose[4];
		lTrans[5] = LOriZ - lTrackerRawPose[5];

	}
}

// CyberTracker display control
QString CyberStation::RTraDisData(bool bRTraDisReal)
{ 
	QString Str;
	if (bRTraDisReal == true)
	{
//		m_rTraRawMat = CalTransMat(rTrackerRawPose);
//		m_rTraRealMat = m_rTraRawMat*CoeffMat;

		// translation data
		m_rTraRealMat(0, 3) = TransCoeffMat(0, 0)*FormMat[0][3] + TransCoeffMat(0, 1)*FormMat[1][3]
							  + TransCoeffMat(0, 2)*FormMat[2][3] + TransCoeffMat(0, 3);
		m_rTraRealMat(1, 3) = TransCoeffMat(1, 0)*FormMat[0][3] + TransCoeffMat(1, 1)*FormMat[1][3]
							  + TransCoeffMat(1, 2)*FormMat[2][3] + TransCoeffMat(1, 3);
		m_rTraRealMat(2, 3) = TransCoeffMat(2, 0)*FormMat[0][3] + TransCoeffMat(2, 1)*FormMat[1][3]
							  + TransCoeffMat(2, 2)*FormMat[2][3] + TransCoeffMat(2, 3);
		
		// vision data
		m_rTraRealMat(3, 0) = m_rTraRealMat(3, 1) = m_rTraRealMat(3, 2) = 0;
		m_rTraRealMat(3, 3) = 1;

		// rotation data
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				RotOSMat(i, j) = FormMat[i][j];
			}
		}
		RotOnPMat = RotOOnMat.inverse()*RotOSMat*RotCoeffMat;
		for (int i = 0; i<3; i++)
		{
			for (int j = 0; j<3; j++)
			{
				m_rTraRealMat(i, j) = RotOnPMat(i, j);
			}
		}


		// 
		for (int i = 0; i<4; i++)
		{
			for (int j = 0; j<4; j++)
			{
				if (abs(m_rTraRealMat(i,j))<1e-5)
				{
					m_rTraRealMat(i,j) = 0;
				}
			}
		}

//		m_file << "Data after calibration:" << std::endl;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				m_file << m_rTraRealMat(i,j) << " ";
			}
			m_file << std::endl;
		}
		m_file << std::endl;

// 		Str = QString("rightTracker_Real:\r\n %1 %2 %3 %4 %5 %6\r\nrightTracker_RealTransMat:\r\n%7 %8 %9\r\n%10 %11 %12\r\n%13 %14 %15\r\n")
// 			.arg(rTrackerRealPose[0]).arg(rTrackerRealPose[1]).arg(rTrackerRealPose[2])
// 			.arg(rTrackerRealPose[3]).arg(rTrackerRealPose[4]).arg(rTrackerRealPose[5]);
		// Display real transformation matrix
		Str = QString("rightTracker_RealTransMat:\r\n%1 %2 %3 %4\r\n%5 %6 %7 %8\r\n%9 %10 %11 %12\r\n%13 %14 %15 %16\r\n")
			.arg(m_rTraRealMat(0,0)).arg(m_rTraRealMat(0,1)).arg(m_rTraRealMat(0,2)).arg(m_rTraRealMat(0,3))
			.arg(m_rTraRealMat(1,0)).arg(m_rTraRealMat(1,1)).arg(m_rTraRealMat(1,2)).arg(m_rTraRealMat(1,3))
			.arg(m_rTraRealMat(2,0)).arg(m_rTraRealMat(2,1)).arg(m_rTraRealMat(2,2)).arg(m_rTraRealMat(2,3))
			.arg(m_rTraRealMat(3,0)).arg(m_rTraRealMat(3,1)).arg(m_rTraRealMat(3,2)).arg(m_rTraRealMat(3,3));
	} 
	else
	{
// 		Str = QString("rightTracker_Raw:\r\n %1 %2 %3 %4 %5 %6\r\n")
// 			.arg(rTrackerRawPose[0]).arg(rTrackerRawPose[1]).arg(rTrackerRawPose[2])
// 			.arg(rTrackerRawPose[3]).arg(rTrackerRawPose[4]).arg(rTrackerRawPose[5]);

//		m_rTraRawMat = CalTransMat(rTrackerRawPose);

// 		Str = QString("rightTracker_Raw:\r\n %1 %2 %3 %4 %5 %6\r\nrightTracker_RawTransMat:\r\n%7 %8 %9\r\n%10 %11 %12\r\n%13 %14 %15\r\n")
// 			.arg(rTrackerRawPose[0]).arg(rTrackerRawPose[1]).arg(rTrackerRawPose[2])
// 			.arg(rTrackerRawPose[3]).arg(rTrackerRawPose[4]).arg(rTrackerRawPose[5])
// 			.arg(m_rTraRawMat(0,0)).arg(m_rTraRawMat(0,1)).arg(m_rTraRawMat(0,2))
// 			.arg(m_rTraRawMat(1,0)).arg(m_rTraRawMat(1,1)).arg(m_rTraRawMat(1,2))
// 			.arg(m_rTraRawMat(2,0)).arg(m_rTraRawMat(2,1)).arg(m_rTraRawMat(2,2));

		Str = QString("rightTracker_RawTransMat:\r\n%1 %2 %3 %4\r\n%5 %6 %7 %8\r\n%9 %10 %11 %12\r\n%13 %14 %15 %16\r\n")
			.arg(FormMat[0][0]).arg(FormMat[0][1]).arg(FormMat[0][2]).arg(FormMat[0][3])
			.arg(FormMat[1][0]).arg(FormMat[1][1]).arg(FormMat[1][2]).arg(FormMat[1][3])
			.arg(FormMat[2][0]).arg(FormMat[2][1]).arg(FormMat[2][2]).arg(FormMat[2][3])
			.arg(FormMat[3][0]).arg(FormMat[3][1]).arg(FormMat[3][2]).arg(FormMat[3][3]);	
	}
	return Str;
}

QString CyberStation::LTraDisData(bool bLTraDisReal)
{
	QString Str;
	if (bLTraDisReal == true)
	{
		for (int i = 0; i<6; i++)
		{
			/*lTrackerRealPose[i] = 0.01*lTrackerRawPose[i] + lTrans[i];*/

			lTrackerRealPose[i] = lTrackerRawPose[i] + lTrans[i];
		}
		Str = QString("leftTracker_Real:\r\n %1 %2 %3 %4 %5 %6\r\n")
			.arg(lTrackerRealPose[0]).arg(lTrackerRealPose[1]).arg(lTrackerRealPose[2])
			.arg(lTrackerRealPose[3]).arg(lTrackerRealPose[4]).arg(lTrackerRealPose[5]);
	} 
	else
	{
		Str = QString("leftTracker_Raw:\r\n %1 %2 %3 %4 %5 %6\r\n")
			.arg(lTrackerRawPose[0]).arg(lTrackerRawPose[1]).arg(lTrackerRawPose[2])
			.arg(lTrackerRawPose[3]).arg(lTrackerRawPose[4]).arg(lTrackerRawPose[5]);
	}
	return Str;
}

//****************************** CyberTracker Calibration is over ******************************//