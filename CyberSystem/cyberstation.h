#ifndef _CyberStation_H
#define _CyberStation_H

#define SETMIN(X, MIN_DATA) if(X < MIN_DATA) X = MIN_DATA;
#define SETMAX(X, MAX_DATA) if(X > MAX_DATA) X = MAX_DATA;
#define SETCUT(X, CUT) if(abs(X) < CUT) X = 0;
#define SETRANGE(X,MIN_DATA,MAX_DATA) if(X < MIN_DATA) X = MIN_DATA; if(X > MAX_DATA) X = MAX_DATA;

#include "inlib.h"

#include <vector>
#include <map>

#include <vhtIOConn.h>
#include <vhtTracker.h>
#include <vht6DofDevice.h>
#include <vhtGenHandModel.h>
#include <vhtCyberGlove.h>
#include <vhtCyberGrasp.h>
#include <vhtBaseException.h>
#include <vhtTransform3D.h>
#include <vhtTrackerData.h>



#include "eigen3/Eigen/Eigen"




typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 4, 4> mat4x4;


struct CaliData
{
	double raw_pos[3];
	double real_pos[3];
	double raw_ori[3];
	double real_ori[3];
};

class CyberStation
{
public:
	typedef std::vector<double> joint_type;	
	typedef std::pair<std::string, std::vector<double>> finger_type;
	typedef std::vector<finger_type> hand_type;
	typedef std::map<std::string, hand_type> hand_cali_type;


	CyberStation();
	~CyberStation();

private:
	typedef std::vector<CaliData> tracali_type;
	//*********************** System Data ***********************//
	// Right Tracker
	vhtIOConn *m_pRightTrackerConn;
	vhtTracker *m_pRightTracker;
	vht6DofDevice *m_pRightRcvr;
	vhtTransform3D RTraXForm;		// Cyber System Container
	double RFormMat[4][4];			// Raw Transformation
	// Left Tracker
	vhtIOConn *m_pLeftTrackerConn;
	vhtTracker *m_pLeftTracker;
	vht6DofDevice *m_pLeftRcvr;
	vhtTransform3D LTraXForm;		// Cyber System Container
	double LFormMat[4][4];			// Raw Transformation
	// Right Grasp
	vhtIOConn *m_pRightGraspConn;
	vhtCyberGrasp *m_pRightGrasp;
	// Left Grasp
	vhtIOConn *m_pLeftGraspConn;
	vhtCyberGrasp *m_pLeftGrasp;
	// Right Glove
	vhtIOConn *m_pRightGloveConn;
	vhtCyberGlove *m_pRightGlove;
	// Left Glove
	vhtIOConn *m_pLeftGloveConn;
	vhtCyberGlove *m_pLeftGlove;


	//*********************** Tracker Data ***********************//
public:
	// Connect Tracker
	bool RTraConn(std::string &err_str);
	bool LTraConn(std::string &err_str);
	// Get right tracker data
	mat4x4 GetRRawTraData();
	mat4x4 GetRTraRealData();

	// TODO(CJH): change these function
	// calculate coefficient for cyber tracker calibration
	void CalRTraCoef(const double *, const int &);
	void CalRTraCoef(const mat4x4 &);
	void CalRTraMapCoeff();
	void RTraMapping(const mat4x4 &in, mat4x4 &out);

	void CalLTraCoef(const double *, const int &);
	void CalLTraCoef(const mat4x4 &);
	void LTraMapping();

	void UpdateTraCali(const mat4x4 &RTransMat, const mat3x3 &RRotMat);
	bool GetCaliCoef(mat4x4 &RTransMat, mat3x3 &RRotMat);

	void CalTrackerCoef(tracali_type, tracali_type);



private:
	// Matrix Data
	mat4x4 m_rTraRealMat;
	mat4x4 m_rTraRawMat;
	// Matrix coefficient
	mat4x4 TransCoeffMat;
	mat3x3 RotCoeffMat;
	mat3x3 RotOSMat;
	mat3x3 RotOOnMat;
	mat3x3 RotOnPMat;
	// Mapping Coefficient
	mat3x3 m_RotMapCoeff;
	double m_TransMapCoeff[3];

	// Tracker Calibration Finish Flag
	bool m_bRTraCaliFin;
	bool m_bLTraCaliFin;
	

	//*********************** Glove Data ***********************//
public:
	// connect devices
	bool RHandConn(std::string &err_str);
	bool LHandConn(std::string &err_str);

	// cyber glove calibration information 
	void GetRRawGloData(double RGlo[5][4]);
//	void GetRRawGloData(double RGlo[5][9]);
	void GetLRawGloData(double LGlo[5][4]);
	void GetRRealGloData(double RGlo[5][3]);
	void GetLRealGloData(double LGlo[5][3]);
	void CalRGloCoeff(const hand_cali_type &RGloCaliData);
	void CalRGloCoeff(const double RGloCaliData[4][5][4]);
	void CalLGloCoeff(const hand_cali_type &LGloCaliData);
	void CalLGloCoeff(const double LGloCaliData[4][5][4]);

	void GetRGloCoeff(double out_RGloCaliK[5][3], double out_RGloCaliB[5][3]);
	void GetLGloCoeff(double out_LGloCaliK[5][3], double out_LGloCaliB[5][3]);
	void UpdateRGloCoeff(const double in_RGloCaliK[5][3], const double in_RGloCaliB[5][3]);
	void UpdateLGloCoeff(double in_LGloCaliK[5][3], double in_LGloCaliB[5][3]);

	void setGraspForce(double Force[5]);



private:
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

	double m_GesData[4][5][3];

// 	double m_TouchGrasp[5][4];
 	double m_RGloCaliK[5][3];
 	double m_RGloCaliB[5][3]; 

	bool m_bRGloCaliFini;
	bool m_bLGloCaliFini;

	// TODO(CJH): delete these data
	double lTrackerRawPose[6];        

};
#endif