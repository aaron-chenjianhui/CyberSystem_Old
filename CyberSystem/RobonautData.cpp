// RobonautData.cpp: implementation of the CRobonautData class.
//
//////////////////////////////////////////////////////////////////////

/*#include "stdafx.h"*/
// #include "RoTeleoperation.h"
#include "RobonautData.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// declare global variable for interconnection
CRobonautData g_RobotCmdDeg;     //ȫ�ֻ����˿���ָ�λ�Ƕ�
CRobonautData g_SRobotCmdDeg;    //���͵İ�ȫ�����˿���ָ�λ�Ƕ�
CRobonautData g_RobotSensorDeg;  //ȫ�ֻ����˴��������ݵ�λ�Ƕ�


CRobonautData::CRobonautData()
{
	count=0;
	
	for (int i=0;i<7;i++)
	{
		leftArmJoint[i]=0.0;
		rightArmJoint[i]=0.0;
	}
	
	for (int i=0;i<6;i++)
	{
		leftArmCartes[i]=0.0;
		rightArmCartes[i]=0.0;
	}
	
	for(int i=0;i<2;i++)
	{
		waistJoint[i]=0.0;
	}
	
	for(int i=0;i<3;i++)
	{
		headJoint[i]=0.0;
	}

	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			leftArmMatrix[i][j]=0.0;
            rightArmMatrix[i][j]=0.0;
			
			if (i==j)
			{
				leftArmMatrix[i][j]=1.0;
                rightArmMatrix[i][j]=1.0;
			}
		}
	}
	CtlMode=1;
//	flag=TRUE;
}

CRobonautData::~CRobonautData()
{

}
