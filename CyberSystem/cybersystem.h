#ifndef _CYBERSYSTEM_H
#define _CYBERSYSTEM_H


#include "ui_cybersystem.h"
#include "CyberStation.h"

#include <QtWidgets/QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QMutex>

#include "eigen3/Eigen/Eigen"

enum CTRLMODE {OUT_CTRL, CLICK_CTRL_SIMU, CLICK_CTRL_ROBO, CLICK_CTRL_ALL, 
				CYBER_CTRL_SIMU, CYBER_CTRL_ROBO, CYBER_CTRL_ALL};
enum HANDCTRLMODE {HAND_OUT_CTRL, HAND_CYBER_CTRL};

class CyberSystem : public QMainWindow
{
	Q_OBJECT

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

public:
	CyberSystem(QWidget *parent = 0);
	~CyberSystem();

	//*********************** Hand ***********************//
private slots:
	void InitGloveCali();
	void FinGloveCali();
	void InitHand();
	void EnableHand();
	void StartHand();
	void EmergHand();

private:
	double m_RGloRawData[5][4];
//	double m_RGloRawData[5][9];
	double m_RGloRealData[5][3];
	double m_LGloRawData[5][4];
	double m_LGloRealData[5][3];
	double m_RGloCaliK[5][3];
	double m_RGloCaliB[5][3];
	double m_RGloCaliData[4][5][4];
	double m_RHandRecvJoint[5][3];
	double m_LHandRecvJoint[5][3];
	double m_RHandRecvTorque[5][3];
	double m_LHandRecvTorque[5][3];

	bool m_bHandConn;
	bool m_bHandInit;
	bool m_bHandStop;
	bool m_bHandEnable;

	HANDCTRLMODE m_HandCtrlMode;
	CHandData m_RHandData, m_LHandData;
	int m_HandDataCount;

public:
	void SendHandCmd();		// Multimedia Timer Function
	void RecvHandSensor();	
	void DisHandData();
	
	//*********************** Click Control ***********************//
public:
	void SetCliUIVisi(bool);
	bool GetRoboData();
	void SetSliVal(float *, float *);

private slots:
 	void ClickCtrl();
	void SlitoLine();
	void LinetoSli();
	void SliUpdata();
	void getSliData();

	//*********************** Robonaut Control ***********************//
public:
	void SendCmd();		// Multimedia Timer Function
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
	Eigen::Matrix<double, 7, 1> m_last_joint_angle;
	Eigen::Matrix<double, 4, 4> m_RTraRealMat;
	Eigen::Matrix<double, 4, 4> m_RTraRawMat;
	//*********************** Consimu Control ***********************//
private slots:
	void ConsimuConnCtrl();		// Connect Consimulate

private:
	bool m_bConsimuConn;		// True: Means Connected

	//*********************** Kinetics ***********************//
public:
	void InitKine();
	Eigen::Matrix<double, 7, 1> CalKine(const Eigen::Matrix<double, 4, 4> &, double &, Eigen::Matrix<double, 7, 1> &);
	void QuaterToTrans(const double arr_in[7], Eigen::Matrix<double, 4, 4> &mat_out);
	

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

	QMutex m_DisDataMutex;		// Create a Mutex to lock m_RoboStr

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
	void SaveCaliData();
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
	// For CyberGlove Control
	bool m_RGloContr;		// Right Glove Connection
	bool m_LGloContr;		// Left Glove Connection
	bool m_bDisGloData;		// Push Glove Start Button
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
