#ifndef _CYBERSYSTEM_H
#define _CYBERSYSTEM_H


#include "ui_cybersystem.h"
#include "CyberStation.h"

#include <QtWidgets/QMainWindow>
#include <QMessageBox>
#include <QTimer>

#include "eigen3/Eigen/Eigen"


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
			_parent->DisData();
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


// 	// A thread controls slider data getting
// 	class RobonautCtrl_Thread : public QThread
// 	{
// 	public:
// 		RobonautCtrl_Thread(CyberSystem *parent)
// 		{
// 			_parent = parent;
// 			m_bRoboCtrlThreadStop = false;
// 		}
// 		void run()
// 		{
// 			_parent->getSliData();
// 		}
// 		void stop()
// 		{
// 			m_bRoboCtrlThreadStop = true;
// 		}
// 	private:
// 		CyberSystem *_parent;
// 	public:
// 		bool m_bRoboCtrlThreadStop;
// 	};

public:
	CyberSystem(QWidget *parent = 0);
	~CyberSystem();

	void DisData();
	void getSliData();
	// calculate kinetics and inverse kinetics
	void InitKine();
	Eigen::Matrix<double, 7, 1> CalKine(const Eigen::Matrix<double, 4, 4> &, double &);



private slots:
	// initialize devices
	void InitSystem();
	void InitRHand(); 
	void InitLHand();
	void InitRTracker();
	void InitLTracker();

	// glove calibration
	void InitGloveCali();
	void FinGloveCali();
	void GesOneData();
	void GesTwoData();
	void GesThrData();
	void GesFourData();

	// tracker calibration
	void InitTraCali();
	void CalTraData();
	void lineEdit_textChanged();

	// consimulation
	void ConsimuConnContr();
	void CyberSimuControl();

	// robonaut control
	void RobonautConnCtrl();
	void RobonautCtrl();

	// robonaut joint control
 	void RobonautJoCtrl();
	void SendJoData();

	// command browser display
	void CommadBsDisplay(const QString &);
	void CommandStrSelect();

	// 5 hand control
	void InitHand();
	void PositionMode();
	void ImpedanceMode();
	void ResetMode();
	void HandCtrl();





signals:
	void InsertGloText(const QString &);
	void InsetCommStr(const QString &);




private:
	Ui::CyberSystemClass ui;
	CyberStation m_CyberStation;

	// define multi-thread to display data
	Dis_Thread m_DisThread;
// 	// define multi-thread to get slider data
// 	RobonautCtrl_Thread m_RobonautCtrlThread;

	//*********************** Device Logical Control ***********************//
	// cyber workstation connection control
	bool m_RGloContr;
	bool m_LGloContr;
	bool m_RTraContr;
	bool m_LTraContr;

	// CyberGlove calibration control
// 	// define timer
// 	QTimer *m_pGloDispTimer;
	// whether to display glove data
	bool m_bDisGloData;
	// which kind of data to display
	bool m_bGloDisReal;
	bool m_bGloCaliFin;

	// CyberTracker calibration control
	// whether to display tracker data
	bool m_bDisTraData;
	// which kind of data to display
	bool m_bRTraCaliFini;
	bool m_bLTraCaliFini;
	bool m_bRTraDisReal;
	bool m_bLTraDisReal;

	// Robonaut control
	RobonautControl m_RobonautControl;

	// Command browser display string
	QString m_CommandString;

	//*********************** Control Data ***********************//
	double m_rightArmPos[7];
	double m_leftArmPos[7];
	double m_last_arm_angle;

};

#endif // CYBERSYSTEM_H
