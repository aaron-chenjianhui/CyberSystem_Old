/*
******************************************************************************************************************
*                                                                                                          #     *
* SAHandCtrlApi.H                                                                                         ##     *
*                                                                                                        # #     *
*                                                                                                     #########  *
*                                                                                                    #  # #  #   *
*                                                                                                   #  # #  #    *
*                                                                                                  #########     *
*                                                                                                     # #        *
*                                                                                                     ##         *
*                                                                                                     #          *
* Content: Class prototype of "SAH"  Hand control Api functions                                                  *
* ============================================================================================================== *
* (C) by Fizulrahman JamalMohamed                                                                                *
*        Mai 2006,	Nov 2006											                                         *
*        DLR (Oberpfaffenhofen), Institut fuer Robotik und Mechatronik                                           *
* -------------------------------------------------------------------------------------------------------------- *
* ============================================================================================================== *
* Language:		C++ header file											    									 * 
* -------------------------------------------------------------------------------------------------------------- *
* Projekt:		Schunk Anthropomorphic Hand																         *
* ============================================================================================================== *
* Modificatios:                                                                                                  *
* ---------------                                                                                                *
*                                                                                                                *
* Datum        | Name     | Version | Durchgefuehrte Aenderungen                                                 *
* -------------+----------+---------+--------------------------------------------------------------------------- *
* 22.06.2006   |          |  1.0	|  First Version Released													 *
* 22.11.2006   |          |  1.1	|  limit setting included and some bugs fixed								 *                                                                                                              *
******************************************************************************************************************
 */


/*======================================================================*\
 * SA Hand Control Defines												* 
\*======================================================================*/

#ifndef SAHANDCTRLAPIDEF_H		//For compiler..
#define SAHANDCTRLAPIDEF_H		// ,,
/*===============================================*/
																			
/*=== BOOL data type verification ===============*/
 
#ifndef BOOL	
typedef int BOOL;
#endif

#ifndef TRUE	
#define TRUE (1)
#endif
#ifndef FALSE	
#define FALSE (0)
#endif


/*=== For Brake =================================*/

#define BRAKE	(1)
#define RUN		(0)

/*=== Finger Defines=============================*/

#define THUMB			(0)
#define FIRST_FINGER	(1)
#define MIDDLE_FINGER	(2)
#define RING_FINGER		(3)
#define LITTLE_FINGER	(4)

/*=== Hand======================================*/

#define HAND_CONFIG_RIGHT	(1)
#define HAND_CONFIG_LEFT	(2)

/*=== Controllers================================*/

#define CONTROLLER_RESET		(0)
#define CONTROLLER_DIRECT		(1)
#define CONTROLLER_POSITION		(2)
#define CONTROLLER_IMPEDANCE	(3)


/*=== For PORTs =================================*/
 #define PORT_1 (1)
 #define PORT_2 (2)


/*=== To Set the Desired Angles with Desired ====
 *=== Acceleration Times of Each Joint===========

	Joint[0] => Finger Distal Joint
	Joint[1] => Base joint 1 ( Front and back direction)
	Joint[2] => Base Joint 2 ( Sideway Direction )

 =================================================*/

typedef struct _SAHFINGERDESIRED {
	volatile float Angle[3];		// Desired Joint Angles
	volatile float Velocity[3];		// Desired Acceleration Time in Seconds
	volatile float Stiffness[3];	// Stiffness factor
	volatile float Interval;		// Interval of time. unit is ms.
	volatile int   CommandID;		// command ID.
} SAHFINGERDESIRED, *LPSAHFINGERDESIRED;

typedef struct _SAH_DESIRED {
	SAHFINGERDESIRED	Thumb;
	SAHFINGERDESIRED	FirstFinger;
	SAHFINGERDESIRED	MiddleFinger;
	SAHFINGERDESIRED	RingFinger;
	SAHFINGERDESIRED	LittleFinger;		// Added on ... 2007.04.05
} SAH_DESIRED, *LPSAH_DESIRED;	// 65


///////////////////////////////////////////////////////////////////
// Struct(1)  For G2HITBOARDINFO.                                //
///////////////////////////////////////////////////////////////////
typedef struct _G2HITBOARDINFO
{
	volatile int 	BoardVer;			// PCI-DSP Board Version. (7..4)
	volatile int 	CommSoftVer;		// Main(15..12) + Minor(11..5) + Extention(4..0),	LikeThis:   1.0.1	1.0.2
	volatile int 	CommSoftDate;		// Year(15..9) + Month(8..5) + Day(4..0)	//110 0101 00110
	volatile int 	CommStatus;			// xxxx-xxxx-xxxx-P2F(H)-P2D-P1H-P1D
	volatile int 	DspIdleCnt;			// xxxx-xxxx-xxxx-P2F(H)-P2D-P1H-P1D
	volatile int	Reserved[12];
} G2HITBOARDINFO, *LPG2HITBOARDINFO;			// 17 Words.

#define BOARDINFO_PORT1D_COMM_MASK	0x0001
#define BOARDINFO_PORT1H_COMM_MASK	0x0002
#define BOARDINFO_PORT1_COMM_MASK	0x0003
#define BOARDINFO_PORT2D_COMM_MASK	0x0004
#define BOARDINFO_PORT2H_COMM_MASK	0x0008
#define BOARDINFO_PORT2_COMM_MASK	0x000C
///////////////////////////////////////////////////////////////////
// End Struct(1)  For G2HITBOARDINFO.                            //
///////////////////////////////////////////////////////////////////


/*======================================================================*\
 * Class Prototype														* 
 * Refer to Hand Api Function Reference Guide for Details.				*
\*======================================================================*/

class CSAHandCtrlApi  
{
public:
	CSAHandCtrlApi();
	virtual ~CSAHandCtrlApi();

public:

	short SAHandInit();

	short SetFingerEnable(int iPort, int iFinger, int iState);
	short SetEmergencyStop(int iPort, int iBrakeState);
	short SetController(int iPort, int iController);

	short SetThumbAngle(int iPort, int iThumbAngle);
	short SetThumbBrake(int iPort, int iBrakeState);

	//short MoveFinger(int iPort, int iFinger, float* pafAngle, float* pafVel, float interval);	// Changed, 2008.08.05
	short MoveFinger(int iPort, int iFinger, float* pafAngle, float* pafVel, float interval);
	short MoveHand(int iPort, SAH_DESIRED* pafDesired);


	short SetStiffnessFactor(int iPort, int iFinger, float* pafStiffnessFactor);
	short SetVelocityLimits(int iPort, int iFinger,float* pafUpperLimit);
	short SetPositionLimits(int iPort, int iFinger,float* pafUpperLimit, float* pafLowerLimit);
	short ResetPositionLimits(int iPort, int iFinger);

	short ClearTorqueSensorOffset(int iPort,int iFinger);
	
	/****** Get Commands ***********************************/
	short GetEmergencyStop(int iPort, int* piBrakeState);
	short GetController(int iPort, int* piController);
	short GetThumbAngle(int iPort, int* piAngle);
	short GetThumbBrake(int iPort, int* piBrakeState);

	short GetJointAngle(int iPort, int iFinger,float* pafAngle);
	short GetJointSpeed(int iPort, int iFinger,float* pafSpeed);
	short GetJointTorque(int iPort, int iFinger,float* pafTorque);

	short GetCommandedValues(int iPort, int iFinger, float* pafAngle,float* pafVelocity);
	short GetFingerEnableState(int iPort, int iFinger, int* piEnableState);
	short GetHandConfig(int iPort, int* piConfig);
	short GetFingerLimitsStatus(int iPort,int iFinger,int* paiLimitStatus);

	short GetFingerTipFT(int iPort, int iFinger,float* pafFTData);
	short GetTipTemp(int iPort, int iFinger,float* pfTipTemp);
	short GetFingerTemp(int iPort, int iFinger,float* pfFingerTemp);
	short GetPalmTemp(int iPort, float* pfPalmTemp);

	short GetPositionID(int iPort, int iFinger, int* iPosID);
	short GetCounter(int* iCounter);
	short SetEnableInterrupt(int iState);
	short SetDirectControlPWM(int iPort, int iFinger, int* iPWM);

	short GetBoardInfo(G2HITBOARDINFO *pBoardInfo);		// moved from private. 2007.04.05
	short ResetDSP();									// Added, 2007.04.06
};

#endif

