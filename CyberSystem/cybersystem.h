#ifndef CYBERSYSTEM_H
#define CYBERSYSTEM_H

#include <QtWidgets/QMainWindow>
#include "ui_cybersystem.h"

class CyberSystem : public QMainWindow
{
	Q_OBJECT

public:
	CyberSystem(QWidget *parent = 0);
	~CyberSystem();

private:
	Ui::CyberSystemClass ui;
};

#endif // CYBERSYSTEM_H
