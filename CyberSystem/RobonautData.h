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
	float leftArmJoint[7];   //左臂关节角度
	float leftArmCartes[6];  //左臂笛卡尔坐标
    float rightArmJoint[7];  //右臂关节角度
    float rightArmCartes[6];  //右臂笛卡尔坐标
	float leftArmMatrix[4][4];
    float rightArmMatrix[4][4];
	CHandData leftHandJoint;   //左手关节角度
	CHandData rightHandJoint;  //右手关节角度 

	float waistJoint[2];   //腰部关节角度 0旋转 1俯仰
	float headJoint[3];   //头部关节角度 0俯仰 1侧摆 2旋转

	float RightJointFT[7];//右臂关节力矩
	float LeftJointFT[7];//左臂关节力矩

	float RightCasteFT[6];//右臂笛卡尔力矩
	float LeftCasteFT[6];//左臂笛卡尔力矩

	int CtlMode; //控制方式1是位置，2是阻抗
	
};

#endif // !defined(AFX_ROBONAUTDATA_H__73D7F6AB_B63B_4C80_A9DC_E7DA699D79C4__INCLUDED_)
