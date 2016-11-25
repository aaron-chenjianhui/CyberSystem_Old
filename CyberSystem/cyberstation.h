#ifndef _CyberStation_H
#define _CyberStation_H

#define SETMIN(X, MIN_DATA) if(X < MIN_DATA) X = MIN_DATA;
#define SETMAX(X, MAX_DATA) if(X > MAX_DATA) X = MAX_DATA;
#define SETCUT(X, CUT) if(X < abs(CUT)) X = 0;

#include <fstream>

#include "inlib.h"
#include "RobonautControl.h"

#include "eigen3/Eigen/Eigen"

#include <vhtIOConn.h>
#include <vhtTracker.h>
#include <vht6DofDevice.h>
#include <vhtGenHandModel.h>
#include <vhtCyberGlove.h>
#include <vhtCyberGrasp.h>
#include <vhtBaseException.h>
#include <vhtTransform3D.h>
#include <vhtTrackerData.h>

#include <QString>
#include <QMessageBox>
#include <QFile>
#include <QThread>

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"
#include "rapidxml/rapidxml_utils.hpp"

struct CaliData
{
	double raw_pos[3];
	double real_pos[3];
	double raw_ori[3];
	double real_ori[3];
};

class CyberStation
{
private:
	typedef std::vector<CaliData> tracali_type;
	// rightTracker
	vhtIOConn *m_pRightTrackerConn;
	vhtTracker *m_pRightTracker;
	vht6DofDevice *m_pRightRcvr;
	// leftTracker
	vhtIOConn *m_pLeftTrackerConn;
	vhtTracker *m_pLeftTracker;
	vht6DofDevice *m_pLeftRcvr;
	// rightGrasp
	vhtIOConn *m_pRightGraspConn;
	vhtCyberGrasp *m_pRightGrasp;
	// leftGrasp
	vhtIOConn *m_pLeftGraspConn;
	vhtCyberGrasp *m_pLeftGrasp;
	// rightGlove
	vhtIOConn *m_pRightGloveConn;
	vhtCyberGlove *m_pRightGlove;
	// leftGlove
	vhtIOConn *m_pLeftGloveConn;
	vhtCyberGlove *m_pLeftGlove;


	//*********************** Tracker Data ***********************//
	// Cyber System Container
	vhtTransform3D RTraXForm;
	double RFormMat[4][4];
	// Matrix Data
	Eigen::Matrix<double, 4, 4> m_rTraRealMat;
	Eigen::Matrix<double, 4, 4> m_rTraRawMat;
	// Matrix coefficient
	Eigen::Matrix<double, 4, 4> TransCoeffMat;
	Eigen::Matrix<double, 3, 3> RotCoeffMat;
	Eigen::Matrix<double, 3, 3> RotOSMat;
	Eigen::Matrix<double, 3, 3> RotOOnMat;
	Eigen::Matrix<double, 3, 3> RotOnPMat;
	// Mapping Coefficient
	Eigen::Matrix<double, 3, 3> m_RotMapCoeff;
	double m_TransMapCoeff[3];

	// Tracker Calibration Finish Flag
	bool m_bRTraCaliFin;
	bool m_bLTraCaliFin;
	

	//*********************** Glove Data ***********************//
	// raw data and real data for glove
	double m_rGloveRawData[5][4];
//	double m_rGloveRealData[5][3];
	double m_lGloveRawData[5][4];
//	double m_lGloveRealData[5][3];
	// calibration data for glove
 	double m_GesOneData[5][4];
	double m_GesTwoData[5][4];
	double m_GesThrData[5][4];
	double m_GesFourData[5][4];
// 	double m_TouchGrasp[5][4];
 	double m_GloCaliK[5][4];
 	double m_GloCaliB[5][4]; 


	// TODO(CJH): delete these data
	double lTrackerRawPose[6];

public:
	CyberStation();
	~CyberStation();

	// display information for connection
	void DisplayMessage(const char *msg, vhtBaseException *e);

	//*********************** Tracker Control ***********************//
	// Connect Tracker
	bool RTraConn(std::string &);
	void LTraConn();
	// Get right tracker data
	Eigen::Matrix<double, 4, 4> GetRRawTraData();
	Eigen::Matrix<double, 4, 4> GetRTraRealData();
	// Calculate Transformation Matrix
	Eigen::Matrix<double, 4, 4> CalTransMat(const double Trans[]);
	// 

	// TODO(CJH): change these function
	// calculate coefficient for cyber tracker calibration
	void CalRTraCoef(const double *, const int &);
	void CalRTraCoef(const Eigen::Matrix<double, 4, 4> &);
	void CalRTraMapCoeff();
	void RTraMapping(const Eigen::Matrix<double, 4, 4> &in, Eigen::Matrix<double, 4, 4> &out);

	void CalLTraCoef(const double *, const int &);
	void CalLTraCoef(const Eigen::Matrix<double, 4, 4> &);
	void LTraMapping();

	void UpdataCaliCoef(const Eigen::Matrix<double, 4, 4> &RTransMat, const Eigen::Matrix<double, 3, 3> &RRotMat);
	bool GetCaliCoef(Eigen::Matrix<double, 4, 4> &RTransMat, Eigen::Matrix<double, 3, 3> &RRotMat);

	void CalTrackerCoef(tracali_type, tracali_type);
	void GetLTraRealData(Eigen::Matrix<double, 4, 4> &);
	void GetLRawTraData(double arr[], int arr_size = 6);



	//*********************** Glove Control ***********************//
	// connect devices
	void RHandConn();
	void LHandConn();

	// cyber glove calibration information 
	// TODO(CJH): use switch to choose
	void SaveCaliData(int );
	void GetGesOneData();
	void GetGesTwoData();
	void GetGesThrData();
	void GetGesFourData();
	// calculate two coefficient for cyber glove calibration
	void CalGloCoef();
	// get real cyberglove data 
	void RealGloData(bool);
	// get data from glove
	void GetRGloData();
	void GetLGloData();
	// decide which data(raw or real) to display
	QString GloDisData(bool);

	
	// TODO(CJH): delete them all
	void GetLTraData();	
//	QString RTraDisData(bool);
	QString LTraDisData(bool);

           

};
#endif