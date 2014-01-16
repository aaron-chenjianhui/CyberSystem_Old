// RobonautData.h: interface for the CRobonautData class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ROBONAUTDATA_H__73D7F6AB_B63B_4C80_A9DC_E7DA699D79C4__INCLUDED_)
#define AFX_ROBONAUTDATA_H__73D7F6AB_B63B_4C80_A9DC_E7DA699D79C4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CHandData
{
public:
	int count;
	float joint[5][3];
	float torque[5][3];
};

class CHandDataList
{
public:
	CHandData m_HandData[50];
	int count;
};

class CRobonautData  
{
public:
	CRobonautData();
	virtual ~CRobonautData();
public:
	int count;
//	BOOL flag;
	float leftArmJoint[7];   //��۹ؽڽǶ�
	float leftArmCartes[6];  //��۵ѿ�������
    float rightArmJoint[7];  //�ұ۹ؽڽǶ�
    float rightArmCartes[6];  //�ұ۵ѿ�������
	float leftArmMatrix[4][4];
    float rightArmMatrix[4][4];
	CHandData leftHandJoint;   //���ֹؽڽǶ�
	CHandData rightHandJoint;  //���ֹؽڽǶ� 

	float waistJoint[2];   //�����ؽڽǶ� 0��ת 1����
	float headJoint[3];   //ͷ���ؽڽǶ� 0���� 1��� 2��ת

	float RightJointFT[7];//�ұ۹ؽ�����
	float LeftJointFT[7];//��۹ؽ�����

	float RightCasteFT[6];//�ұ۵ѿ�������
	float LeftCasteFT[6];//��۵ѿ�������

	int CtlMode; //���Ʒ�ʽ1��λ�ã�2���迹
	
};

#endif // !defined(AFX_ROBONAUTDATA_H__73D7F6AB_B63B_4C80_A9DC_E7DA699D79C4__INCLUDED_)
