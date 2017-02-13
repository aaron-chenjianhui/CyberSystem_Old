#ifndef _CYBERSYSTEM_H
#define _CYBERSYSTEM_H


#include "ui_cybersystem.h"

#include "RobonautControl.h"
#include "CyberStation.h"

#include <QtWidgets/QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QTimer>
#include <QMutex>
#include <QString>
#include <QFile>
#include <QThread>

#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/SVD"

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"
#include "rapidxml/rapidxml_utils.hpp"

enum CTRLMODE {OUT_CTRL, CLICK_CTRL_SIMU, CLICK_CTRL_ROBO, CLICK_CTRL_ALL, 
				CYBER_CTRL_SIMU, CYBER_CTRL_ROBO, CYBER_CTRL_ALL,
				PLAN_CTRL_SIMU, PLAN_CTRL_ROBO, PLAN_CTRL_ALL, PLAN_WAIT,
				VISION_CTRL};
enum HANDCTRLMODE {HAND_OUT_CTRL, HAND_CYBER_CTRL, HAND_GRASP_CTRL, HAND_CLICK_CTRL};

// must use 250ms
const int RobonautCommPd = 250;


typedef Eigen::Matrix<double, 6, 7> mat6x7;
typedef Eigen::Matrix<double, 7, 1> mat7x1;
typedef Eigen::Matrix<double, 4, 1> mat4x1;
typedef Eigen::Matrix<double, 6, 1> mat6x1;
typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 3, 1> mat3x1;


class CyberSystem : public QMainWindow
{
	Q_OBJECT
private:

public:
	// A thread controls cyber data display
	class Dis_Thread : public QThread
	{
	public:
		Dis_Thread(CyberSystem *parent)
		{
			_parent = parent;
			m_bDisThreadStop = false;
		}
		void run()
		{
			_parent->DisCyberData();
		}
		void stop()
		{
			m_bDisThreadStop = true;
		}
	private:
		CyberSystem *_parent;
	public:
		bool m_bDisThreadStop;
	};

	// Thread to Receive Cameral Data
	class CamThread: public QThread
	{
	public:
		CamThread(CyberSystem *parent)
		{
			_parent = parent;
			m_bCamThreadStop = false;
		}

		void run()
		{
			_parent->ViCycle();
		}
		void stop()
		{
			m_bCamThreadStop = true;
		}
	private:
		CyberSystem *_parent;
	public:
		bool m_bCamThreadStop;
	};



public:
	CyberSystem(QWidget *parent = 0);
	~CyberSystem();

	//*********************** Hand ***********************//
private slots:
	void InitGloveCali();
	void FinGloveCali();
	void InitHand();
	void EnableHand();
	void StartCyberHand();
	void CyberHandMode();
	void PauseCyberHand();
	void EmergHand();
	void ClickHandMode();
	void UpdateHandSpin();
	void SendHandSpin();
	void GraspHandMode();
	void GraspHand();
	void ReleaseHand();

private:
	double m_RGloRawData[5][4];
//	double m_RGloRawData[5][9];
	double m_RGloRealData[5][3];		// 0是基关节，1是指尖关节，2是侧摆
	double m_LGloRawData[5][4];
	double m_LGloRealData[5][3];
	double m_RGloCaliK[5][3];
	double m_RGloCaliB[5][3];
	double m_RGloCaliData[4][5][4];
	double m_RHandRecvJoint[5][3];		// 0是基关节，1是指尖关节，2是侧摆
	double m_LHandRecvJoint[5][3];		// 0是基关节，1是指尖关节，2是侧摆
	double m_RHandRecvTorque[5][3];
	double m_LHandRecvTorque[5][3];
	double m_RHandSendData[5][3];
	double m_LHandSendData[5][3];
	double m_HandGraspJoint[5][3];		// 抓取时手指的关节角数据
	double m_HandReleaseJoint[5][3];		// 释放时手指的关节角数据

	bool m_bHandConn;		// 灵巧手连接标志
	bool m_bHandInit;		// 初始化标志
	bool m_bHandStop;		// 急停标志
	bool m_bHandEnable;		// 灵巧手使能标志

	bool m_bHandGrasp;		// 抓取模式下切换标志

	HANDCTRLMODE m_HandCtrlMode;
	CHandData m_RHandData, m_LHandData;
	int m_HandDataCount;

	bool m_RGloConn;		// Right Glove Connection
	bool m_LGloConn;		// Left Glove Connection
	
public:
	void SetGraspInit();		// 设置抓取时手指角度的初始化数据
	void SendHandCmd();		// Multimedia Timer Function
	void RecvHandSensor();	
	void DisHandData();
	void ReadHandSpinData(double RData[5][3], double LData[5][3]);
	void SetHandSpinData(const double RData[5][3], const double LData[5][3]);
	
	//*********************** Click Control ***********************//
public:
	void SetCliUIVisi(bool);
	bool GetRoboData();
	void SetSliVal(float *, float *);

private slots:
 	void ClickCtrl();
	void SliderToSpin();
	void SpinToSlider();
	void SliUpdata();
	void getSliData();

	//*********************** Robonaut Control ***********************//
public:
	void SendCmd();		// Multimedia Timer Function
	void CyberCtrlMode();
	void ClickCtrlMode();
	void OutCtrlMode();
	void PlanCtrlMode();
	void VisionCtrlMode();

	void RecvSensor();	
	void DisRoboData();

private slots:
	void RoboConnCtrl();		// Robonaut Connection Ctrol
	void CyberCtrl();

	void CyberCmd();
	void CyberStop();

private:
	bool m_bRoboConn;		// True: Means Connected
	double m_rightArmPos[7];
	double m_leftArmPos[7];
	double m_last_arm_angle;


	mat7x1 m_last_joint_angle;

	mat4x4 m_RTraRealMat;
	mat7x1 m_RTraRealQuat;
	mat4x4 m_last_RTraRealMat;
	mat7x1 m_last_RTraRealQuat;
	mat4x4 m_RTraRawMat;
	//*********************** Consimu Control ***********************//
private slots:
	void ConsimuConnCtrl();		// Connect Consimulate

private:
	bool m_bConsimuConn;		// True: Means Connected


	//*********************** Vision Serve ***********************//
private:
	// 申明为const
	double RefEulerPose[6];
	double RefFinAppr;
	double EulerPoseCut[6];
	double ApprCut;

	CSocketBlockClient m_VisionRecv_Client;
	mat4x4 m_ViTransNow;
	mat4x4 m_ViTransNext;
	mat7x1 m_ViQuatNow;
	mat7x1 m_ViQuatNext;

	// Robot Arm Joints
	mat7x1 m_RArmJo;
	mat7x1 m_LArmJo;

	mat7x1 last_cmd_jo;

	double m_ViRecv[7];
	double m_ViEulerRecv[7];

	bool m_bConnCam;
	bool m_bViKineInit;
	bool m_bTrackFinish;
	bool m_bApprFlag;		// 如果m_bApprFlag = true, 表示可以在Z轴方向接近

	CamThread m_CamThread;

public:
	void ViCycle();
	void RecvVision();
	void GetViNextPos(const double ViRecv[], const mat7x1 &Quat_Ref, mat7x1 &Quat_New);
	void GetViNextOri(const double ViEulerRecv[], const double Quat_Ref[7], double Quat_New[7]);
	void GetViNextAppr(const double ViEulerRecv[], const double Quat_Ref[7], double Quat_New[7]);
	void ViRecvTrans(const double ViRecv_in[], double ViRecv_out_Euler[]);

private slots:
	void ConnVision();
	void VisionCtrl();
	void VisionStart();
	void VisionStop();
	void VisionAppr();

	//*********************** Planning ***********************//
public:
	// Return next plan pose
	void PosePlan(const mat7x1 &StartPose, const mat7x1 &EndPose, const int PlanTotalCount, const int PlanCount, mat7x1 &PoseOut);


private:
	mat7x1 m_RPlanStartQuat;
	mat7x1 m_RPlanEndQuat;
	double m_RPlanTime;

	int m_plan_count;
	int m_plan_count_max;

private slots:
	void PlanCtrl();
	void getPlanData();
	void ExecPlan();

	//*********************** Different Kinetics ***********************//
public:
	void InitKine();
	mat7x1 CalKine(const mat4x4 &, double &, mat7x1 &);

	void CalJaco(const mat7x1 &mat_jo, mat6x7 &mat_jaco, double &sv_min);
	void CalDirectKine(const mat7x1 &mat_jo_deg, mat4x4 &mat_trans);
	bool DiffKine(const mat7x1 &quat_now, const mat7x1 &quat_last, const mat7x1 &last_joint, mat7x1 &joint);
	double DampLS(double svmin);
	bool AngleRange(const mat7x1 &jo);

	void QuaterToRot(const double arr_in[4], mat3x3 &mat_out);
	void QuaterToTrans(const double arr_in[7], mat4x4 &mat_out);
	void TransToQuater(const mat4x4 &mat_in, mat7x1 &mat_out);
	void RodToQuater(const mat3x1 &mat_in, mat4x1 &mat_out);
	void RodToTrans(const mat3x1 &mat_in, mat3x3 &mat_out);
	void TransToRod(const mat3x3 &mat_in, mat3x1 &mat_out);
	void RodToEuler(const mat3x1 &mat_in, mat3x1 &mat_out);
	void EulerToRot(const double arr_in[3], mat3x3 &mat_out);
	void EulerToTrans(const double arr_in[6], mat4x4 &mat_out);
	void TransToEuler(const mat3x3 &mat_in, double mat_out[]);


private:
	mat6x7 m_RJacoMat;
	mat4x4 m_RQuatoOmega;
	mat4x1 m_ROmega;
	mat4x1 m_RdQuat;
	mat6x1 m_RVel;

	mat4x4 m_RFeedTrans;
	mat7x1 m_RFeedQuat;
	mat3x1 m_RRotErr;
	mat6x1 m_RPoseErr;
	mat3x3 CrossMat;
	mat3x1 des_Epsilon;
	mat3x1 feed_Epsilon;

	bool m_bJacoIsInit;		// if m_bJacoIsInit == 0, there is no reference joint for jacobi kine; if m_bJacoIsInit == 1 ......

	//*********************** Data Display ***********************//
public:
	void DisCyberData();
	void DisGloData();
	void DisTraData();

private:
	QString m_CmdStr;
	QString m_HandStr;		// Hand Receive Data
	QString m_RoboStr;
	QString m_RoboTotalStr;

	//QMutex m_DisDataMutex;		// Create a Mutex to lock m_RoboStr
	//QMutex m_ViMutex;

private slots:
	// text Browser
	void BrowserMoveEnd();
	// initialize devices
	void InitSystem();
	void InitRHand(); 
	void InitLHand();
	void InitRTracker();
	void InitLTracker();

	void LoadTraCaliData();
	void SaveTraCaliData();
	void LoadGloCaliData();
	void SaveGloCaliData();


	void GesOneData();
	void GesTwoData();
	void GesThrData();
	void GesFourData();

	// tracker calibration
	void InitTraCali();
	void CalTraData();
	void lineEdit_textChanged();






//*********************** QBrowser Display Signals ***********************//
signals:
	void InsertGloText(const QString &);
	void InsertTraText(const QString &);
	void InsertRoboText(const QString &);
	void InsertCmdStr(const QString &);




private:
	// Children Class
	Ui::CyberSystemClass ui;
	CyberStation m_CyberStation;
	RobonautControl m_RobonautControl;

	// Multi_Thread to Display Data
	Dis_Thread m_DisThread;
	

	//*********************** Device Logical Control ***********************//
	bool m_bRGloCaliFin;		// Right Glove Finish Calibration
	bool m_bLGloCaliFin;		// Left Glove Finish Calibration

	// For CyberTracker Control
	bool m_RTraContr;		// Right Tracker Connection
	bool m_LTraContr;		// Left Tracker Connection
	bool m_bDisTraData;		// Push Tracker Start Button
	bool m_bRTraCaliFini;		// Right Tracker Finish Calibration
	bool m_bLTraCaliFini;		// Left Tracker Finish Calibration
	CTRLMODE m_CtrlMode;		// Choose Control Mode

	// For File Save
	std::ofstream m_fRRealMat;
	std::ofstream m_fRRawMat;
	int RRealCount;
	int RRawCount;

};

#endif // CYBERSYSTEM_H
