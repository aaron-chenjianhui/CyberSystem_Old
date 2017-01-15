#include "CyberStation.h"



#include <math.h>

#define SINANG(X) (sin(X/180*3.1415926))
#define COSANG(X) (cos(X/180*3.1415926))

// endpoint position for calibration
#define END_POS_X 100
#define END_POS_Y 100
#define END_POS_Z 100
#define END_ORI_X 0
#define END_ORI_Y 0
#define END_ORI_Z 0


// TODO(CJH): Delete
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
		rTrackerRealPose[i] = 0;
		lTrackerRawPose[i] = 0;
		lTrackerRealPose[i] = 0;
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
			m_RGloCaliK[i][j] = 0;
			m_RGloCaliB[i][j] = 0;
		}
	}

	// Flag for Calibration
	m_bRTraCaliFin = false;
	m_bLTraCaliFin = false;
	m_bRGloCaliFini = false;
	m_bLGloCaliFini = false;
}

CyberStation::~CyberStation()
{
//	m_file.close();
}


//****************************** Connection Management ******************************//
bool CyberStation::RTraConn(std::string &err_str)
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
//		DisplayMessage("Error with rightTracker: ", e);
		err_str = e->getMessage();
		return false;
	}

	// Extract the receiver 0,1 of the rightTracker
	m_pRightRcvr = m_pRightTracker->getLogicalDevice(0);
	err_str = "Success";
	return true;
}



bool CyberStation::LTraConn(std::string &err_str)
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
		err_str = e->getMessage();
		return false;
	}

	// Extract the receiver 0,1 of the rightTracker
	m_pLeftRcvr = m_pLeftTracker->getLogicalDevice(0);	
	err_str = "Success";
	return true;
}



bool CyberStation::RHandConn(std::string &err_str)
{
	// Connect to the right grasp
	m_pRightGraspConn = vhtIOConn::getDefault( "RightGrasp" );
	m_pRightGrasp = NULL;
	try
	{
		m_pRightGrasp = new vhtCyberGrasp( m_pRightGraspConn );
	}
	catch ( vhtBaseException *e )
	{
//		DisplayMessage("Error with rightGrasp: ", e);
		err_str = e->getMessage();
		return false;
	}
	m_pRightGrasp->setMode( GR_CONTROL_FORCE );


	// Connect to right glove
	//GHM::Handedness gloveHandedness = GHM::rightHand;
	try{
		m_pRightGloveConn = vhtIOConn::getDefault( "RightGlove" );
		m_pRightGlove = new vhtCyberGlove(m_pRightGloveConn,true);
	}
	catch (vhtBaseException* e){
//		DisplayMessage("Error with CyberGlove: ", e);
		err_str = e->getMessage();
		return false;
	}
	err_str = "Success";
	return true;
}

bool CyberStation::LHandConn(std::string &err_str)
{
	// Connect to the right grasp
	m_pLeftGraspConn = vhtIOConn::getDefault( "LeftGrasp" );
	try
	{
		m_pLeftGrasp = new vhtCyberGrasp( m_pLeftGraspConn );
	}
	catch ( vhtBaseException *e )
	{
		err_str = e->getMessage();
		return false;
	}
	m_pLeftGrasp->setMode( GR_CONTROL_FORCE );

	// Connect to left glove
	//GHM::Handedness gloveHandedness = GHM::leftHand;
	try{
		m_pLeftGloveConn = vhtIOConn::getDefault( "LeftGlove" );
		m_pLeftGlove = new vhtCyberGlove(m_pLeftGloveConn,true);
	}
	catch (vhtBaseException* e){
		err_str = e->getMessage();
		return false;
	}
	err_str = "Success";
	return true;
}

//****************************** Connection Management is over ******************************//




//****************************** CyberGlove Calibration ******************************//
void CyberStation::GetRRawGloData(double RGlo[5][4])
//void CyberStation::GetRRawGloData(double RGlo[5][9])
{
	m_pRightGlove->update();
	for (int finger = 0; finger < GHM::nbrFingers; finger++)
	{
		for (int joint = 0; joint <= GHM::nbrJoints; joint++)
		//for (int joint = 0; joint < 9; joint++)
		{
			RGlo[finger][joint] = m_pRightGlove->getRawData((GHM::Fingers)finger,(GHM::Joints)joint);
		}
	}
}

void CyberStation::GetLRawGloData(double LGlo[5][4])
{
	m_pLeftGlove->update();
	for (int finger = 0; finger < GHM::nbrFingers; finger++)
	{
		for (int joint = 0; joint <= GHM::nbrJoints; joint++)
		{
			LGlo[finger][joint] = m_pLeftGlove->getRawData((GHM::Fingers)finger,(GHM::Joints)joint);
		}
	}
}





// Overload Function Use STL feature
void CyberStation::CalRGloCoeff(const hand_cali_type &RGloCaliData)
{
	hand_type RHand;
	auto hand_iter = RGloCaliData.find("Gestrue_One");
	if (hand_iter != RGloCaliData.end())
	{
		auto gesture_one = hand_iter->second;

		for (auto finger_iter = gesture_one.begin(); finger_iter < gesture_one.end(); ++ finger_iter)
		{
			auto joints = finger_iter->second;
			for (auto joint_iter = joints.begin(); joint_iter != joints.end(); ++joint_iter)
			{
				*joint_iter;
			}
		}
	}
	float HandAcces = 75.0 - 0.0;

	for(int i=0;i<5;i++)
	{
		for(int j=0; j<2; j++)
		{
			m_RGloCaliK[i][j] = HandAcces/(m_GesTwoData[i][j] - m_GesThrData[i][j]);
			m_RGloCaliB[i][j] = 0.0 - m_RGloCaliK[i][j]*m_GesThrData[i][j];
		}
	}
	m_RGloCaliK[0][0] = HandAcces/(m_GesFourData[0][0] - m_GesThrData[0][0]);
	m_RGloCaliB[0][0] = 0.0 - m_RGloCaliK[0][0]*m_GesThrData[0][0];

	// calculate side rotation angles
	m_RGloCaliK[0][2] = 0;
	m_RGloCaliB[0][2] = 0;
	m_RGloCaliK[1][2] = 15.0/(m_GesThrData[2][2] - m_GesOneData[2][2]);
	m_RGloCaliB[1][2] = 0.0 - m_RGloCaliK[1][2]*m_GesThrData[2][2];
	m_RGloCaliK[2][2] = 0;
	m_RGloCaliB[2][2] = 0;
	m_RGloCaliK[3][2] = 5.0/(m_GesThrData[3][2] - m_GesOneData[3][2]);
	m_RGloCaliB[3][2] = 0.0 - m_RGloCaliK[3][2]*m_GesThrData[3][2];
	m_RGloCaliK[4][2] = 15.0/(m_GesThrData[4][2] - m_GesOneData[4][2]);
	m_RGloCaliB[4][2] = 0.0 - m_RGloCaliK[4][2]*m_GesThrData[4][2];

	m_bRGloCaliFini = true;
}

void CyberStation::CalRGloCoeff(const double RGloCaliData[4][5][4])
{
	float HandAcces = 75.0 - 5.0;
	for (int i = 0; i < 4; ++i)
	{
		m_GesData[i][0][0] = RGloCaliData[i][0][2];
		m_GesData[i][0][1] = RGloCaliData[i][0][1];
		m_GesData[i][0][2] = RGloCaliData[i][0][3];

		m_GesData[i][1][0] = RGloCaliData[i][1][1];
		m_GesData[i][1][1] = RGloCaliData[i][1][0];
		m_GesData[i][1][2] = RGloCaliData[i][2][3];

		m_GesData[i][2][0] = RGloCaliData[i][2][1];
		m_GesData[i][2][1] = RGloCaliData[i][2][0];
		m_GesData[i][2][2] = 0;

		m_GesData[i][3][0] = RGloCaliData[i][3][1];
		m_GesData[i][3][1] = RGloCaliData[i][3][0];
		m_GesData[i][3][2] = RGloCaliData[i][3][3];

		m_GesData[i][4][0] = RGloCaliData[i][4][1];
		m_GesData[i][4][1] = RGloCaliData[i][4][0];
		m_GesData[i][4][2] = RGloCaliData[i][4][3];
	}

	// 四手指弯曲角度标定
	for(int i = 1; i < 5; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			m_RGloCaliK[i][j] = HandAcces/(m_GesData[1][i][j] - m_GesData[2][i][j]);
			m_RGloCaliB[i][j] = 75.0 - m_RGloCaliK[i][j]*m_GesData[1][i][j];
		}
	}
	// 大拇指弯曲角度标定
	m_RGloCaliK[0][0] = HandAcces/(m_GesData[1][0][0] - m_GesData[2][0][0]);
	m_RGloCaliB[0][0] = 75.0 - m_RGloCaliK[0][0]*m_GesData[1][0][0];
	m_RGloCaliK[0][1] = HandAcces/(m_GesData[3][0][1] - m_GesData[2][0][1]);
	m_RGloCaliB[0][1] = 75.0 - m_RGloCaliK[0][1]*m_GesData[3][0][1];
	// 各手指侧摆角度
	m_RGloCaliK[0][2] = 0;
	m_RGloCaliB[0][2] = 0;
	m_RGloCaliK[1][2] = 12.0/(71.0 - 167.0);
	m_RGloCaliB[1][2] = -12.0*147.0/(71.0 - 167.0);
	m_RGloCaliK[2][2] = 0;
	m_RGloCaliB[2][2] = 0;
	m_RGloCaliK[3][2] = 4.0/20.0;
	m_RGloCaliB[3][2] = -4.0*120.0/20.0;
	m_RGloCaliK[4][2] = -10.0/(66.0 - 140.0);
	m_RGloCaliB[4][2] = 10.0*140.0/(66.0 - 140.0);

	m_bRGloCaliFini = true;
}

// Load Glove Calibration Coefficients from Outside
void CyberStation::UpdateRGloCoeff(const double in_RGloCaliK[5][3], const double in_RGloCaliB[5][3])
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m_RGloCaliK[i][j] = in_RGloCaliK[i][j];
			m_RGloCaliB[i][j] = in_RGloCaliB[i][j];
		}
	}
	m_bRGloCaliFini = true;
}

// For Outside to Get Glove Calibration Coefficients
void CyberStation::GetRGloCoeff(double RGloCaliK[5][3], double RGloCaliB[5][3])
{
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			RGloCaliK[i][j] = m_RGloCaliK[i][j];
			RGloCaliB[i][j] = m_RGloCaliB[i][j];
		}
	}
}



void CyberStation::GetRRealGloData(double RGlo[5][3])
{
	// Right Glove is Calibrated
	if (m_bRGloCaliFini == true)
	{
		GetRRawGloData(m_rGloveRawData);
		// bent angle
		for (int i = 0; i<5; i++)
		{
			m_rGloveRealData[i][0] = m_RGloCaliK[i][0]*m_rGloveRawData[i][1] + m_RGloCaliB[i][0];
			m_rGloveRealData[i][1] = m_RGloCaliK[i][1]*m_rGloveRawData[i][0] + m_RGloCaliB[i][1];
		}
		m_rGloveRealData[0][0] = m_RGloCaliK[0][0]*m_rGloveRawData[0][2] + m_RGloCaliB[0][0];
		m_rGloveRealData[0][1] = m_RGloCaliK[0][1]*m_rGloveRawData[0][1] + m_RGloCaliB[0][1];	
		// side rotation angle
		m_rGloveRealData[0][2] = 0;
		m_rGloveRealData[1][2] = m_RGloCaliK[1][2]*m_rGloveRawData[2][3] + m_RGloCaliB[1][2];
//		m_rGloveRealData[2][2] = 0;
		m_rGloveRealData[3][2] = m_RGloCaliK[3][2]*m_rGloveRawData[3][3] + m_RGloCaliB[3][2];
		m_rGloveRealData[2][2] = 0.8*m_rGloveRealData[3][2] + 0.5*m_rGloveRealData[1][2];
		m_rGloveRealData[4][2] = m_RGloCaliK[4][2]*m_rGloveRawData[4][3] + m_RGloCaliB[4][2] + m_rGloveRealData[3][2];

		//m_rGloveRealData[0][2] = 0;
		//m_rGloveRealData[1][2] = 12*(m_rGloveRawData[2][3] - 147)/(71-167);
		//m_rGloveRealData[2][2] = 0;
		//m_rGloveRealData[3][2] = 4*(m_rGloveRawData[3][3]-120)/(120-100);
		//m_rGloveRealData[4][2] = -10*(m_rGloveRawData[4][3]-140)/(66-140);


		// protections
		for (int i=0;i<5;i++)
		{
			SETCUT(m_rGloveRealData[i][2], 1);
		}
		SETRANGE(m_rGloveRealData[2][2], -3, 3);
		for(int i = 0; i < 5; ++i)
		{
			SETMIN(m_rGloveRealData[i][1], 8.0);
			SETMIN(m_rGloveRealData[i][0], 5.0);
			for(int j = 0; j < 2; ++j){
				SETMAX(m_rGloveRealData[i][j], 75);
			}
		}
		SETMAX(m_rGloveRealData[0][0], 40);
		SETMAX(m_rGloveRealData[1][0], 45);
		SETMAX(m_rGloveRealData[2][0], 60);


		for (int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				RGlo[i][j] = m_rGloveRealData[i][j];
			}
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
	// Right Glove is not Calibrated
	else{
		for (int i = 0; i < 5; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				RGlo[i][j] = 0.0;
			}
		}
	}
}


void CyberStation::setGraspForce(double Force[5])
{
	m_pRightGrasp->setForce(Force);
}
//****************************** CyberGlove Calibration is over ******************************//



//****************************** CyberTracker Calibration ******************************//
// get raw data from CyberTracker
mat4x4 CyberStation::GetRRawTraData()
{
	m_pRightRcvr->update();
	m_pRightTracker->getLogicalDevice(0)->getTransform(&RTraXForm);
	RTraXForm.getTransform(RFormMat);

	mat4x4 Mat;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			Mat(i, j) = RFormMat[i][j];
		}
	}
	return Mat;
}



// Calculate Transformation Matrxi,
// TODO(CJH): Without Add Left Tracker Calibration
void CyberStation::CalRTraCoef(const double RPose[], const int &len_RPose)
{
	// Use Euler Angle
	if (len_RPose == 6)
	{

		// Device Calibration: Step 1
		// position matrix calculate
		TransCoeffMat(0, 0) = TransCoeffMat(0, 2) = 0;
		TransCoeffMat(1, 0) = TransCoeffMat(1, 1) = 0;
		TransCoeffMat(2, 1) = TransCoeffMat(2, 2) = 0;
		TransCoeffMat(3, 0) = TransCoeffMat(3, 1) = TransCoeffMat(3, 2) = 0;
		TransCoeffMat(0, 1) = TransCoeffMat(1, 2) = TransCoeffMat(2, 0) = TransCoeffMat(3, 3) = 1;
		TransCoeffMat(0, 3) = RPose[0] - RFormMat[1][3];
		TransCoeffMat(1, 3) = RPose[1] - RFormMat[2][3];
		TransCoeffMat(2, 3) = RPose[2] - RFormMat[0][3];

		// Device Calibration: Step 2
		// orient data of a special pose
		mat3x3 ROriMat;
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
				RotOSMat(i, j) = RFormMat[i][j];
			}
		}

		RotCoeffMat = RotOSMat.inverse()*RotOOnMat*RotOnPMat;

		// Motion Mapping
		CalRTraMapCoeff();

		// Indicate: Right Tracker Calibration is Finished
		m_bRTraCaliFin = true;
	}

	// Use Unit Quaternion
	else if (len_RPose == 7)
	{

	}

	else{

	}
}



void CyberStation::CalRTraMapCoeff()
{
	// Oriten Mapping
	mat3x3 RRotMapOut;
	mat3x3 RRotMapIn;
	RRotMapOut << -0.176, -0.984, 0.01,
				-0.7, 0.119, -0.7,
				0.688, -0.13, -0.71;
	RRotMapIn << 1, 0, 0,
				0, 0, 1,
				0, -1, 0;

	m_RotMapCoeff = RRotMapOut*RRotMapIn.inverse();

	//		for (int i = 1; i<3; i++)
	//		{
	//			for (int j = 0; j<3; j++)
	//			{
	////				RTransMat(i, j) = -RTransMat(i, j);
	//				RTransMat(i, j) = RTransMat(i, j);
	//			}
	//		}

	// Position Mapping
	double RPosIn[3] = {0, -33.7, 30.7};
	double RPosOut[3] = {-219.62, -550, 44.192};

	m_TransMapCoeff[0] = RPosOut[0]- RPosIn[0]*10;
	m_TransMapCoeff[1] = RPosOut[1] - RPosIn[1]*10;
	m_TransMapCoeff[2] = RPosOut[2] - RPosIn[2]*10;
}

// Use Transmatrix to Calculate
void CyberStation::CalRTraCoef(const mat4x4 &R_Mat)
{

}


void CyberStation::CalLTraCoef(const double LPose[], const int &len_LPose)
{

}

void CyberStation::CalLTraCoef(const mat4x4 &L_Mat)
{

}

void CyberStation::UpdateTraCali(const mat4x4 &RTransMat, const mat3x3 &RRotMat)
{
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

	mat3x3 ROriMat;
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


	TransCoeffMat = RTransMat;
	RotCoeffMat = RRotMat;
	
	// Motion Mapping
	CalRTraMapCoeff();


	// Indicate: Right Tracker Calibration is Finished
	m_bRTraCaliFin = true;
}

bool CyberStation::GetCaliCoef(mat4x4 &TransMat, mat3x3 &RotMat)
{
	if (m_bRTraCaliFin == true)
	{
		TransMat = TransCoeffMat;
		RotMat = RotCoeffMat;
	} 
	else
	{
	}
	return m_bRTraCaliFin;
}

// Get Right Tracker's Real Transformation Matrix, 
// Must After a Tracker Calibration
mat4x4 CyberStation::GetRTraRealData()
{
	if (m_bRTraCaliFin == true){
		// update raw data
		m_pRightRcvr->update();
		m_pRightTracker->getLogicalDevice(0)->getTransform(&RTraXForm);
		RTraXForm.getTransform(RFormMat);

		// Get Cyber Data in a Special Coordination
		// translation data
		mat4x4 RTransMat;		// 映射后
		mat4x4 RTransMat_bf;		// 映射前
		RTransMat_bf(0, 3) = TransCoeffMat(0, 0)*RFormMat[0][3] + TransCoeffMat(0, 1)*RFormMat[1][3]
		+ TransCoeffMat(0, 2)*RFormMat[2][3] + TransCoeffMat(0, 3);
		RTransMat_bf(1, 3) = TransCoeffMat(1, 0)*RFormMat[0][3] + TransCoeffMat(1, 1)*RFormMat[1][3]
		+ TransCoeffMat(1, 2)*RFormMat[2][3] + TransCoeffMat(1, 3);
		RTransMat_bf(2, 3) = TransCoeffMat(2, 0)*RFormMat[0][3] + TransCoeffMat(2, 1)*RFormMat[1][3]
		+ TransCoeffMat(2, 2)*RFormMat[2][3] + TransCoeffMat(2, 3);

		// vision data
		RTransMat_bf(3, 0) = RTransMat_bf(3, 1) = RTransMat_bf(3, 2) = 0;
		RTransMat_bf(3, 3) = 1;

		// rotation data
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				RotOSMat(i, j) = RFormMat[i][j];
			}
		}
		RotOnPMat = RotOOnMat.inverse()*RotOSMat*RotCoeffMat;
		for (int i = 0; i<3; i++)
		{
			for (int j = 0; j<3; j++)
			{
				RTransMat_bf(i, j) = RotOnPMat(i, j);
			}
		}

		// 得到映射后的值
		RTraMapping(RTransMat_bf, RTransMat);	

		// 死区
		for (int i = 0; i<4; i++)
		{
			for (int j = 0; j<4; j++)
			{
				if (abs(RTransMat(i,j))<1e-5)
				{
					RTransMat(i,j) = 0;
				}
			}
		}

		return RTransMat;
	}

	// TODO(CJH): If Calibration is not finished,
	// DO Something!!!
	else{
		mat4x4 mat;
		mat << 1, 0, 0, 0,
			0, 1, 0, 0, 
			0, 0, 1, 0, 
			0, 0, 0, 1;
		return mat;
	}
}


void CyberStation::RTraMapping(const mat4x4 &Mat_In, mat4x4 &Mat_Out)
{
	// 位置映射
	for (int i = 0; i < 3; ++i){
		Mat_Out(i, 3) = Mat_In(i, 3)*10 + m_TransMapCoeff[i];
	}

// 	// 变角度映射
// 	mat3x3 RTrans_tmp;
// 	for (int i = 0; i<3; i++)
// 	{
// 		for (int j = 0; j<3; j++)
// 		{
// 			RTrans_tmp(i, j) = Mat_In(i, j);
// 		}
// 	}
// 	RTrans_tmp = m_RotMapCoeff*RTrans_tmp;
// 	for (int i = 0; i<3; i++)
// 	{
// 		for (int j = 0; j<3; j++)
// 		{
// 			Mat_Out(i, j) = RTrans_tmp(i, j);
// 		}
// 	}

	// 固定角度映射
	mat3x3 RoboRot;
	RoboRot << -0.176314, -0.984285, 0.00985975,
		-0.703848, 0.119064, -0.700301,
		0.688121, -0.130412, -0.71378;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			Mat_Out(i, j) = RoboRot(i, j);
		}
	}

	// 视觉矩阵
	Mat_Out(3, 0) = Mat_Out(3, 1) = Mat_Out(3, 2) = 0;
	Mat_Out(3, 3) = 1;
}







// TODO(CJH): Use vector to store calibration data
// Replace the old calibration function
// perhaps useless
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
}

//****************************** CyberTracker Calibration is over ******************************//