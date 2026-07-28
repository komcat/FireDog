#pragma once
#include "MOTION_DEFINE.h"

//------------------------------------------------------------------
//                 Device Type Defines.
//------------------------------------------------------------------
#ifndef DEVTYPE_EZI_STEP_PLUS_R_ST
#define	DEVTYPE_EZI_STEP_PLUS_R_ST			20
#define DEVNAME_EZI_STEP_PLUS_R_ST			"Ezi-STEP Plus-R-ST"
#endif

//------------------------------------------------------------------
//                 Axis Status Flag Defines.
//------------------------------------------------------------------

typedef union
{
	DWORD	dwValue;
	struct
	{
		unsigned	FFLAG_ERRORALL			: 1; // = 0x00000001;
		unsigned	FFLAG_HWPOSILMT			: 1; // = 0x00000002;
		unsigned	FFLAG_HWNEGALMT			: 1; // = 0x00000004;
		unsigned	FFLAG_SWPOGILMT			: 1; // = 0x00000008;
		unsigned	FFLAG_SWNEGALMT			: 1; // = 0x00000010;
		unsigned	FFLAG_RESERVED0			: 1; // = 0x00000020;
		unsigned	FFLAG_RESERVED1			: 1; // = 0x00000040;
		unsigned	FFLAG_ERRSTEPALARM		: 1; // = 0x00000080;
		unsigned	FFLAG_ERROVERCURRENT	: 1; // = 0x00000100;
		unsigned	FFLAG_ERROVERSPEED		: 1; // = 0x00000200;
		unsigned	FFLAG_ERRSTEPOUT		: 1; // = 0x00000400;
		unsigned	FFLAG_RESERVED2			: 1; // = 0x00000800;
		unsigned	FFLAG_ERROVERHEAT		: 1; // = 0x00001000;
		unsigned	FFLAG_ERRBACKEMF		: 1; // = 0x00002000;
		unsigned	FFLAG_ERRMOTORPOWER		: 1; // = 0x00004000;
		unsigned	FFLAG_ERRLOWPOWER		: 1; // = 0x00008000;
		unsigned	FFLAG_EMGSTOP			: 1; // = 0x00010000;
		unsigned	FFLAG_SLOWSTOP			: 1; // = 0x00020000;
		unsigned	FFLAG_ORIGINRETURNING	: 1; // = 0x00040000;
		unsigned	FFLAG_RESERVED3			: 1; // = 0x00080000;
		unsigned	FFLAG_RESERVED4			: 1; // = 0x00100000;
		unsigned	FFLAG_ALARMRESET		: 1; // = 0x00200000;
		unsigned	FFLAG_PTSTOPPED			: 1; // = 0x00400000;
		unsigned	FFLAG_ORIGINSENSOR		: 1; // = 0x00800000;
		unsigned	FFLAG_ZPULSE			: 1; // = 0x01000000;
		unsigned	FFLAG_ORIGINRETOK		: 1; // = 0x02000000;
		unsigned	FFLAG_MOTIONDIR			: 1; // = 0x04000000;
		unsigned	FFLAG_MOTIONING			: 1; // = 0x08000000;
		unsigned	FFLAG_MOTIONPAUSE		: 1; // = 0x10000000;
		unsigned	FFLAG_MOTIONACCEL		: 1; // = 0x20000000;
		unsigned	FFLAG_MOTIONDECEL		: 1; // = 0x40000000;
		unsigned	FFLAG_MOTIONCONST		: 1; // = 0x80000000;
	};
} EZISTEP_AXISSTATUS;

//------------------------------------------------------------------
//                 Input/Output Defines.
//------------------------------------------------------------------

// Input Bit-mask list.
static const DWORD	STEP_IN_BITMASK_LIMITP			= 0x00000001;
static const DWORD	STEP_IN_BITMASK_LIMITN			= 0x00000002;
static const DWORD	STEP_IN_BITMASK_ORIGIN			= 0x00000004;
static const DWORD	STEP_IN_BITMASK_CLEARPOSITION	= 0x00000008;
static const DWORD	STEP_IN_BITMASK_PTA0			= 0x00000010;
static const DWORD	STEP_IN_BITMASK_PTA1			= 0x00000020;
static const DWORD	STEP_IN_BITMASK_PTA2			= 0x00000040;
static const DWORD	STEP_IN_BITMASK_PTA3			= 0x00000080;
static const DWORD	STEP_IN_BITMASK_PTA4			= 0x00000100;
static const DWORD	STEP_IN_BITMASK_PTA5			= 0x00000200;
static const DWORD	STEP_IN_BITMASK_PTA6			= 0x00000400;
static const DWORD	STEP_IN_BITMASK_PTA7			= 0x00000800;
static const DWORD	STEP_IN_BITMASK_PTSTART			= 0x00001000;
static const DWORD	STEP_IN_BITMASK_STOP			= 0x00002000;
static const DWORD	STEP_IN_BITMASK_PJOG			= 0x00004000;
static const DWORD	STEP_IN_BITMASK_NJOG			= 0x00008000;
static const DWORD	STEP_IN_BITMASK_ALARMRESET		= 0x00010000;
//static const DWORD	STEP_IN_BITMASK_RESERVED	= 0x00020000;
static const DWORD	STEP_IN_BITMASK_PAUSE			= 0x00040000;
static const DWORD	STEP_IN_BITMASK_ORIGINSEARCH	= 0x00080000;
static const DWORD	STEP_IN_BITMASK_TEACHING		= 0x00100000;
static const DWORD	STEP_IN_BITMASK_ESTOP			= 0x00200000;
static const DWORD	STEP_IN_BITMASK_JPTIN0			= 0x00400000;
static const DWORD	STEP_IN_BITMASK_JPTIN1			= 0x00800000;
static const DWORD	STEP_IN_BITMASK_JPTIN2			= 0x01000000;
static const DWORD	STEP_IN_BITMASK_JPTSTART		= 0x02000000;
static const DWORD	STEP_IN_BITMASK_USERIN0			= 0x04000000;
static const DWORD	STEP_IN_BITMASK_USERIN1			= 0x08000000;
static const DWORD	STEP_IN_BITMASK_USERIN2			= 0x10000000;
static const DWORD	STEP_IN_BITMASK_USERIN3			= 0x20000000;
static const DWORD	STEP_IN_BITMASK_USERIN4			= 0x40000000;
static const DWORD	STEP_IN_BITMASK_USERIN5			= 0x80000000;
static const DWORD	STEP_IN_BITMASK_USERIN6			= 0x00000200;
static const DWORD	STEP_IN_BITMASK_USERIN7			= 0x00000400;
static const DWORD	STEP_IN_BITMASK_USERIN8			= 0x00000800;

// Output Bit-mask list.
static const DWORD	STEP_OUT_BITMASK_COMPAREOUT		= 0x00000001;
//static const DWORD	STEP_OUT_BITMASK_RESERVED	= 0x00000002;
static const DWORD	STEP_OUT_BITMASK_ALARM			= 0x00000004;
static const DWORD	STEP_OUT_BITMASK_RUNSTOP		= 0x00000008;
static const DWORD	STEP_OUT_BITMASK_ACCDEC			= 0x00000010;
static const DWORD	STEP_OUT_BITMASK_ACK			= 0x00000020;
static const DWORD	STEP_OUT_BITMASK_END			= 0x00000040;
static const DWORD	STEP_OUT_BITMASK_ALARMBLINK		= 0x00000080;
static const DWORD	STEP_OUT_BITMASK_ORGSEARCHOK	= 0x00000100;
//static const DWORD	STEP_OUT_BITMASK_RESERVED	= 0x00000200;
//static const DWORD	STEP_OUT_BITMASK_RESERVED	= 0x00000400;
static const DWORD	STEP_OUT_BITMASK_BRAKE			= 0x00000800;
static const DWORD	STEP_OUT_BITMASK_PTOUT0			= 0x00001000;
static const DWORD	STEP_OUT_BITMASK_PTOUT1			= 0x00002000;
static const DWORD	STEP_OUT_BITMASK_PTOUT2			= 0x00004000;
static const DWORD	STEP_OUT_BITMASK_USEROUT0		= 0x00008000;
static const DWORD	STEP_OUT_BITMASK_USEROUT1		= 0x00010000;
static const DWORD	STEP_OUT_BITMASK_USEROUT2		= 0x00020000;
static const DWORD	STEP_OUT_BITMASK_USEROUT3		= 0x00040000;
static const DWORD	STEP_OUT_BITMASK_USEROUT4		= 0x00080000;
static const DWORD	STEP_OUT_BITMASK_USEROUT5		= 0x00100000;
static const DWORD	STEP_OUT_BITMASK_USEROUT6		= 0x00200000;
static const DWORD	STEP_OUT_BITMASK_USEROUT7		= 0x00400000;
static const DWORD	STEP_OUT_BITMASK_USEROUT8		= 0x00800000;

typedef union
{
	DWORD dwValue;
	struct
	{
		unsigned BIT_LIMITP		: 1;
		unsigned BIT_LIMITN		: 1;
		unsigned BIT_ORIGIN		: 1;

		unsigned BIT_CLEARPOSITION	: 1;
		unsigned BIT_PTA0			: 1;
		unsigned BIT_PTA1			: 1;
		unsigned BIT_PTA2			: 1;
		unsigned BIT_PTA3			: 1;
		unsigned BIT_PTA4			: 1;
		unsigned BIT_PTA5			: 1;
		unsigned BIT_PTA6			: 1;
		unsigned BIT_PTA7			: 1;
		unsigned BIT_PTSTART		: 1;
		unsigned BIT_STOP			: 1;
		unsigned BIT_PJOG			: 1;
		unsigned BIT_NJOG			: 1;
		unsigned BIT_ALARMRESET		: 1;
		unsigned BIT_RESERVED0		: 1;
		unsigned BIT_PAUSE			: 1;
		unsigned BIT_ORIGINSEARCH	: 1;
		unsigned BIT_TEACHING		: 1;
		unsigned BIT_ESTOP			: 1;
		unsigned BIT_JPTIN0			: 1;
		unsigned BIT_JPTIN1			: 1;
		unsigned BIT_JPTIN2			: 1;
		unsigned BIT_JPTSTART		: 1;
		unsigned BIT_USERIN0		: 1;
		unsigned BIT_USERIN1		: 1;
		unsigned BIT_USERIN2		: 1;
		unsigned BIT_USERIN3		: 1;
		unsigned BIT_USERIN4		: 1;
		unsigned BIT_USERIN5		: 1;
	};
} EZISTEP_INLOGIC;

typedef union
{
	DWORD dwValue;
	struct
	{
		unsigned BIT_COMP			: 1;

		unsigned BIT_RESERVED0		: 1;
		unsigned BIT_ALARM			: 1;
		unsigned BIT_RUNSTOP		: 1;
		unsigned BIT_ACCDEC			: 1;
		unsigned BIT_ACK			: 1;
		unsigned BIT_END			: 1;
		unsigned BIT_ALARMBLINK		: 1;
		unsigned BIT_ORGSEARCHOK	: 1;
		unsigned BIT_RESERVED1		: 1;
		unsigned BIT_RESERVED2		: 1;
		unsigned BIT_BRAKE			: 1;
		unsigned BIT_PTOUT0			: 1;
		unsigned BIT_PTOUT1			: 1;
		unsigned BIT_PTOUT2			: 1;
		unsigned BIT_USEROUT0		: 1;
		unsigned BIT_USEROUT1		: 1;
		unsigned BIT_USEROUT2		: 1;
		unsigned BIT_USEROUT3		: 1;
		unsigned BIT_USEROUT4		: 1;
		unsigned BIT_USEROUT5		: 1;
		unsigned BIT_USEROUT6		: 1;
		unsigned BIT_USEROUT7		: 1;
		unsigned BIT_USEROUT8		: 1;
	};
} EZISTEP_OUTLOGIC;

//------------------------------------------------------------------
//                 Input/Output Assigning Defines.
//------------------------------------------------------------------

typedef enum
{
	STEP_IN_PREASSIGN_LIMITP = 1,
	STEP_IN_PREASSIGN_LIMITN,
	STEP_IN_PREASSIGN_ORIGIN,

	STEP_IN_LOGIC_CLEARPOSITION,
	STEP_IN_LOGIC_PTA0,
	STEP_IN_LOGIC_PTA1,
	STEP_IN_LOGIC_PTA2,
	STEP_IN_LOGIC_PTA3,
	STEP_IN_LOGIC_PTA4,
	STEP_IN_LOGIC_PTA5,
	STEP_IN_LOGIC_PTA6,
	STEP_IN_LOGIC_PTA7,
	STEP_IN_LOGIC_PTSTART,
	STEP_IN_LOGIC_STOP,
	STEP_IN_LOGIC_PJOG,
	STEP_IN_LOGIC_NJOG,
	STEP_IN_LOGIC_ALARMRESET,
	STEP_IN_LOGIC_RESERVED0,
	STEP_IN_LOGIC_PAUSE,
	STEP_IN_LOGIC_ORIGINSEARCH,
	STEP_IN_LOGIC_TEACHING,
	STEP_IN_LOGIC_ESTOP,
	STEP_IN_LOGIC_JPTIN0,
	STEP_IN_LOGIC_JPTIN1,
	STEP_IN_LOGIC_JPTIN2,
	STEP_IN_LOGIC_JPTSTART,
	STEP_IN_LOGIC_USERIN0,
	STEP_IN_LOGIC_USERIN1,
	STEP_IN_LOGIC_USERIN2,
	STEP_IN_LOGIC_USERIN3,
	STEP_IN_LOGIC_USERIN4,
	STEP_IN_LOGIC_USERIN5,

	MAX_STEP_IN_LOGIC
} EZISTEP_INLOGIC_LIST;

typedef enum
{
	STEP_OUT_PREASSIGN_COMP = 1,

	STEP_OUT_LOGIC_RESERVED0,
	STEP_OUT_LOGIC_ALARM,
	STEP_OUT_LOGIC_RUNSTOP,
	STEP_OUT_LOGIC_ACCDEC,
	STEP_OUT_LOGIC_ACK,
	STEP_OUT_LOGIC_END,
	STEP_OUT_LOGIC_ALARMBLINK,
	STEP_OUT_LOGIC_ORGSEARCHOK,
	STEP_OUT_LOGIC_RESERVED1,
	STEP_OUT_LOGIC_RESERVED2,
	STEP_OUT_LOGIC_BRAKE,
	STEP_OUT_LOGIC_PTOUT0,
	STEP_OUT_LOGIC_PTOUT1,
	STEP_OUT_LOGIC_PTOUT2,
	STEP_OUT_LOGIC_USEROUT0,
	STEP_OUT_LOGIC_USEROUT1,
	STEP_OUT_LOGIC_USEROUT2,
	STEP_OUT_LOGIC_USEROUT3,
	STEP_OUT_LOGIC_USEROUT4,
	STEP_OUT_LOGIC_USEROUT5,
	STEP_OUT_LOGIC_USEROUT6,
	STEP_OUT_LOGIC_USEROUT7,
	STEP_OUT_LOGIC_USEROUT8,

	MAX_STEP_OUT_LOGIC
} EZISTEP_OUTLOGIC_LIST;

static const int	STEP_IN_PIN_CNT	= 12;
static const int	STEP_OUT_PIN_CNT	= 10;

static const int	STEP_IN_LOGIC_CNT			= (MAX_STEP_IN_LOGIC - 1);
static const int	STEP_MAX_PREASSIGNED_IN		= (STEP_IN_PREASSIGN_ORIGIN);

static const int	STEP_OUT_LOGIC_CNT			= (MAX_STEP_OUT_LOGIC - 1);
static const int	STEP_MAX_PREASSIGNED_OUT	= (STEP_OUT_PREASSIGN_COMP);

//------------------------------------------------------------------
//                 Parameters Defines.
//------------------------------------------------------------------

typedef enum
{
	STEP_PULSEPERREVOLUTION = 0,
	STEP_AXISMAXSPEED,
	STEP_AXISSTARTSPEED,
	STEP_AXISACCTIME,
	STEP_AXISDECTIME,

	STEP_SPEEDOVERRIDE,
	STEP_JOGHIGHSPEED,
	STEP_JOGLOWSPEED,
	STEP_JOGACCDECTIME,

	STEP_ALARMLOGIC,
	STEP_RUNSTOPSIGNALLOGIC,		//SERVO_SERVOONLOGIC,
	STEP_RESETLOGIC,

	STEP_SWLMTPLUSVALUE,
	STEP_SWLMTMINUSVALUE,
	STEP_SOFTLMTSTOPMETHOD,
	STEP_HARDLMTSTOPMETHOD,
	STEP_LIMITSENSORLOGIC,

	STEP_ORGSPEED,
	STEP_ORGSEARCHSPEED,
	STEP_ORGACCDECTIME,
	STEP_ORGMETHOD,
	STEP_ORGDIR,
	STEP_ORGOFFSET,
	STEP_ORGPOSITIONSET,
	STEP_ORGSENSORLOGIC,

	STEP_STOPCURRENT,
	STEP_MOTIONDIR,

	STEP_LIMITSENSORDIR,
	STEP_ENCODERMULTIVALUE,

	STEP_ENCODERDIR,
	STEP_POSVALUECOUNTINGMETHOD,

	MAX_STEP_PARAM

} FM_EZISTEP_PARAM;

//------------------------------------------------------------------
//                 POSITION TABLE Defines.
//------------------------------------------------------------------

#define STEP_MAX_ITEM_COUNT		256
