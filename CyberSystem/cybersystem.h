#ifndef CYBERSYSTEM_H
#define CYBERSYSTEM_H

#include <QtWidgets/QMainWindow>
#include "ui_cybersystem.h"
#include "CyberStation.h"
#include <QMessageBox>
#include <QTimer>



class CyberSystem : public QMainWindow
{
	Q_OBJECT

public:
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

public:
	CyberSystem(QWidget *parent = 0);
	~CyberSystem();

	void DisData();



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



signals:
	void InsertGloText(const QString &);




private:
	Ui::CyberSystemClass ui;
	CyberStation m_CyberStation;

	// define multi-thread to display data
	Dis_Thread m_DisThread;

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
};

#endif // CYBERSYSTEM_H
