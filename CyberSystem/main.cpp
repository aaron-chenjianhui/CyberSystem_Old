#include "cybersystem.h"
#include <QtWidgets/QApplication>
#include "qprocess.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	QProcess *DrvProcess = new QProcess(&a);
	std::string Drvpath;
	Drvpath = "C:\\Program Files (x86)\\CyberGlove Systems\\VirtualHand Drivers\\Applications\\winnt_386\\Device Manager\\Master.exe";
	DrvProcess->start(QString(Drvpath.data()), QStringList());

	while(QProcess::Running != DrvProcess->state())
	{
		QThread::currentThread()->msleep(10);
	}

	QThread::currentThread()->msleep(100);

	CyberSystem w;
	w.show();
	const int res = a.exec();

	if (DrvProcess != 0)
	{
		DrvProcess->close();
		while(QProcess::NotRunning !=  DrvProcess->state())
		{
			QThread::currentThread()->msleep(10);
		}
	}

	return res;
}
