
#pragma once
#include <tchar.h>

#include "MOTION_EziSERVO_DEFINE.h"
#include "MOTION_EziSERVO_ADC_DEFINE.h"
#include "MOTION_EziSERVO_ALL_DEFINE.h"
#include "MOTION_EziSERVO_ALL_ABS_DEFINE.h"
#include "MOTION_EziSERVO_mini_DEFINE.h"

#include "MOTION_EziSTEP_DEFINE.h"
#include "MOTION_EziSTEP_ALL_DEFINE.h"
#include "MOTION_EziSTEP_mini_DEFINE.h"

#include "MOTION_EziMotionLink_DEFINE.h"
#include "MOTION_EziMotionLink2_DEFINE.h"

#include "MOTION_EziSERVO2_DEFINE.h"
#include "MOTION_SSERVO_DEFINE.h"
#include "MOTION_EziSERVO2_86_DEFINE.h"
#include "MOTION_EziSERVO2_TO_DEFINE.h"
#include "MOTION_EziSTEP2_DEFINE.h"

#include "MOTION_EziSERVO_ALL_28_DEFINE.h"
#include "MOTION_EziSERVO_ALL_28_V2_DEFINE.h"

//------------------------------------------------------------------
//                 Device Type Defines.
//------------------------------------------------------------------
#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ST
#define	DEVTYPE_EZI_SERVO_PLUS_R_ST				1
#define DEVNAME_EZI_SERVO_PLUS_R_ST				"Ezi-SERVO Plus-R-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ADC_ST
#define	DEVTYPE_EZI_SERVO_PLUS_R_ADC_ST			3
#define DEVNAME_EZI_SERVO_PLUS_R_ADC_ST			"Ezi-SERVO Plus-R-ADC"
#endif

#ifndef DEVTYPE_EZI_MOTIONLINK
#define	DEVTYPE_EZI_MOTIONLINK					10
#define DEVNAME_EZI_MOTIONLINK					"Ezi-MotionLink"
#endif

#ifndef DEVTYPE_EZI_MOTIONLINK2
#define	DEVTYPE_EZI_MOTIONLINK2					11
#define DEVNAME_EZI_MOTIONLINK2					"Ezi-MotionLink2"
#endif

#ifndef DEVTYPE_EZI_STEP_PLUS_R_ST
#define	DEVTYPE_EZI_STEP_PLUS_R_ST				20
#define DEVNAME_EZI_STEP_PLUS_R_ST				"Ezi-STEP Plus-R-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ALL_ST
#define	DEVTYPE_EZI_SERVO_PLUS_R_ALL_ST			30
#define DEVNAME_EZI_SERVO_PLUS_R_ALL_ST			"Ezi-SERVO Plus-R-ALL"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ALL_ABS
#define	DEVTYPE_EZI_SERVO_PLUS_R_ALL_ABS		35
#define DEVNAME_EZI_SERVO_PLUS_R_ALL_ABS		"Ezi-SERVO Plus-R-ALL Abs"
#endif

#ifndef DEVTYPE_EZI_STEP_PLUS_R_ALL_ST
#define	DEVTYPE_EZI_STEP_PLUS_R_ALL_ST			40
#define DEVNAME_EZI_STEP_PLUS_R_ALL_ST			"Ezi-STEP Plus-R-ALL"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_MINI
#define	DEVTYPE_EZI_SERVO_PLUS_R_MINI			50
#define DEVNAME_EZI_SERVO_PLUS_R_MINI			"Ezi-SERVO Plus-R-Mini"
#endif

#ifndef DEVTYPE_EZI_STEP_PLUS_R_MINI
#define	DEVTYPE_EZI_STEP_PLUS_R_MINI			60
#define DEVNAME_EZI_STEP_PLUS_R_MINI			"Ezi-STEP Plus-R-Mini"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ALL60I
#define	DEVTYPE_EZI_SERVO_PLUS_R_ALL60I			31
#define DEVNAME_EZI_SERVO_PLUS_R_ALL60I			"Ezi-SERVO Plus-R-ALL 60i"
#endif

#ifndef DEVTYPE_EZI_SERVO2_PLUS_R_ST
#define	DEVTYPE_EZI_SERVO2_PLUS_R_ST			100
#define DEVNAME_EZI_SERVO2_PLUS_R_ST			"Ezi-SERVO II Plus-R-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO2_PLUS_R_TO
#define	DEVTYPE_EZI_SERVO2_PLUS_R_TO			180
#define DEVNAME_EZI_SERVO2_PLUS_R_TO			"Ezi-SERVO II Plus-R TO"
#endif

#ifndef DEVTYPE_S_SERVO_PLUS_R_ST
#define	DEVTYPE_S_SERVO_PLUS_R_ST				102
#define DEVNAME_S_SERVO_PLUS_R_ST				"S-SERVO Plus-R-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO2_PLUS_E_ST
#define	DEVTYPE_EZI_SERVO2_PLUS_E_ST			100
#define DEVNAME_EZI_SERVO2_PLUS_E_ST			"Ezi-SERVO II Plus-E-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO2_PLUS_E_ST_86
#define	DEVTYPE_EZI_SERVO2_PLUS_E_ST_86			101
#define DEVNAME_EZI_SERVO2_PLUS_E_ST_86			"Ezi-SERVO II Plus-E-ST-86"
#endif

#ifndef DEVTYPE_EZI_SERVO2_PLUS_E_ALL60
#define	DEVTYPE_EZI_SERVO2_PLUS_E_ALL60			90
#define DEVNAME_EZI_SERVO2_PLUS_E_ALL60			"Ezi-SERVO II Plus-E-ALL60"
#endif

#ifndef DEVTYPE_S_SERVO_PLUS_E_ST
#define	DEVTYPE_S_SERVO_PLUS_E_ST				102
#define DEVNAME_S_SERVO_PLUS_E_ST				"S-SERVO Plus-E-ST"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ALL_28
#define	DEVTYPE_EZI_SERVO_PLUS_R_ALL_28			105
#define DEVNAME_EZI_SERVO_PLUS_R_ALL_28			"Ezi-SERVO Plus-R ALL-28"
#endif

#ifndef DEVTYPE_EZI_SERVO_PLUS_R_ALL_28_V2
#define	DEVTYPE_EZI_SERVO_PLUS_R_ALL_28_V2		108
#define DEVNAME_EZI_SERVO_PLUS_R_ALL_28_V2		"Ezi-SERVO Plus-R ALL-28 V2"
#endif

#ifndef DEVTYPE_EZI_SERVO2_CC_LINK_ST
#define	DEVTYPE_EZI_SERVO2_CC_LINK_ST			120
#define DEVNAME_EZI_SERVO2_CC_LINK_ST			"Ezi-SERVO II CC-LINK-ST"
#endif

#ifndef DEVTYPE_EZI_STEP2_PLUS_R_ST
#define	DEVTYPE_EZI_STEP2_PLUS_R_ST				130
#define DEVNAME_EZI_STEP2_PLUS_R_ST				"Ezi-STEP II Plus-R-ST"
#endif

#ifndef DEVTYPE_EZI_STEP2_PLUS_E_ST
#define	DEVTYPE_EZI_STEP2_PLUS_E_ST				130
#define DEVNAME_EZI_STEP2_PLUS_E_ST				"Ezi-STEP II Plus-E-ST"
#endif

//------------------------------------------------------------------
//                 Device Type Defines. (for V8)
//------------------------------------------------------------------
// Driver Type A group : Ezi-SERVO Plus-R Inc. Family
#ifndef DEVTYPEV8_EZI_SERVO_PLUS_R_ST
#define	DEVTYPEV8_EZI_SERVO_PLUS_R_ST			1
#define DEVNAMEV8_EZI_SERVO_PLUS_R_ST			"Ezi-SERVO Plus-R Inc."
#endif

// Driver Type B group : only Motion Controller (without Drive) family
#ifndef DEVTYPEV8_EZI_MOTIONLINK
#define	DEVTYPEV8_EZI_MOTIONLINK				10
#define DEVNAMEV8_EZI_MOTIONLINK				"Ezi-MotionLink"
#endif

// Driver Type C group : Ezi-STEP Plus-R Family
#ifndef DEVTYPEV8_EZI_STEP_PLUS_R_ST
#define	DEVTYPEV8_EZI_STEP_PLUS_R_ST			20
#define DEVNAMEV8_EZI_STEP_PLUS_R_ST			"Ezi-STEP Plus-R"
#endif

// Driver Type D group : Ezi-SERVO Plus-R Abs. Family
#ifndef DEVTYPEV8_EZI_SERVO_PLUS_R_ABS_ST
#define	DEVTYPEV8_EZI_SERVO_PLUS_R_ABS_ST		30
#define DEVNAMEV8_EZI_SERVO_PLUS_R_ABS_ST		"Ezi-SERVO Plus-R Abs."
#endif

// Driver Type E group : Ezi-SERVO Plus-R BLDC Family
#ifndef DEVTYPEV8_EZI_SERVO_PLUS_R_BLDC_ST
#define	DEVTYPEV8_EZI_SERVO_PLUS_R_BLDC_ST		40
#define DEVNAMEV8_EZI_SERVO_PLUS_R_BLDC_ST		"Ezi-SERVO Plus-R BLDC F60"
#endif

// Driver Type F group : only Motion Controller (without Drive) family
#ifndef DEVTYPEV8_EZI_MOTIONLINK2
#define	DEVTYPEV8_EZI_MOTIONLINK2				110
#define DEVNAMEV8_EZI_MOTIONLINK2				"Ezi-MotionLink2"
#endif

//------------------------------------------------------------------
//                 Device Type Defines. (Ezi-IO)
//------------------------------------------------------------------

#ifndef DEVTYPE_EZI_IO_PLUS_R_IN16
#define	DEVTYPE_EZI_IO_PLUS_R_IN16			150
#define DEVNAME_EZI_IO_PLUS_R_IN16			"Ezi-IO RS-485-IN16"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_R_OUT16
#define	DEVTYPE_EZI_IO_PLUS_R_OUT16			160
#define DEVNAME_EZI_IO_PLUS_R_OUT16			"Ezi-IO RS-485-OUT16"
#endif


#ifndef DEVTYPE_EZI_IO_PLUS_R_I8O8
#define	DEVTYPE_EZI_IO_PLUS_R_I8O8			155
#define DEVNAME_EZI_IO_PLUS_R_I8O8			"Ezi-IO RS-485-I8O8"
#endif


#ifndef DEVTYPE_EZI_IO_PLUS_E_IN16
#define	DEVTYPE_EZI_IO_PLUS_E_IN16			150
#define DEVNAME_EZI_IO_PLUS_E_IN16			"Ezi-IO Ethernet-IN16"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_E_IN32
#define	DEVTYPE_EZI_IO_PLUS_E_IN32			151
#define DEVNAME_EZI_IO_PLUS_E_IN32			"Ezi-IO Ethernet-IN32"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_E_OUT16
#define	DEVTYPE_EZI_IO_PLUS_E_OUT16			160
#define DEVNAME_EZI_IO_PLUS_E_OUT16			"Ezi-IO Ethernet-OUT16"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_E_OUT32
#define	DEVTYPE_EZI_IO_PLUS_E_OUT32			161
#define DEVNAME_EZI_IO_PLUS_E_OUT32			"Ezi-IO Ethernet-OUT32"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_E_I8O8
#define	DEVTYPE_EZI_IO_PLUS_E_I8O8			155
#define DEVNAME_EZI_IO_PLUS_E_I8O8			"Ezi-IO Ethernet-I8O8"
#endif

#ifndef DEVTYPE_EZI_IO_PLUS_E_I16O16
#define	DEVTYPE_EZI_IO_PLUS_E_I16O16		156
#define DEVNAME_EZI_IO_PLUS_E_I16O16		"Ezi-IO Ethernet-I16O16"
#endif


//------------------------------------------------------------------
//                 Device Type Defines. (Etc)
//------------------------------------------------------------------
#define DEVTYPE_BOOT_ROM						0xFF
#define DEVTYPE_BOOT_ROM_2						0xFE

//------------------------------------------------------------------
//                 Motion Direction Defines.
//------------------------------------------------------------------

static const int	DIR_INC = 1;
static const int	DIR_DEC = 0;

//------------------------------------------------------------------
//                 Axis Status Flag Defines.
//------------------------------------------------------------------

static const int	MAX_AXIS_STATUS		= 32;

//------------------------------------------------------------------
//           GetAllStatusEx Function
//------------------------------------------------------------------

#define ALLSTATUSEX_ITEM_COUNT		12

#define STATUSEX_TYPE_NONE			0
#define STATUSEX_TYPE_INPUT			1
#define STATUSEX_TYPE_OUTPUT		2
#define STATUSEX_TYPE_AXISSTATUS	3
#define STATUSEX_TYPE_CMDPOS		4
#define STATUSEX_TYPE_ACTPOS		5
#define STATUSEX_TYPE_ACTVEL		6
#define STATUSEX_TYPE_POSERR		7
#define STATUSEX_TYPE_PTNO			8
#define STATUSEX_TYPE_ALARMTYPE		9
#define STATUSEX_TYPE_TEMPERATURE	10
#define STATUSEX_TYPE_CURRENT		11
#define STATUSEX_TYPE_LOAD			12
#define STATUSEX_TYPE_PEAKLOAD		13
#define STATUSEX_TYPE_ENCVEL		14
#define STATUSEX_TYPE_INPUT_HIGH	15
#define STATUSEX_TYPE_PTNO_RUNNING	16
#define STATUSEX_TYPE_ADVALUE0		30	// Ezi-IO AD: Ch0 & Ch1 AD Value
#define STATUSEX_TYPE_ADVALUE2		31	// Ezi-IO AD: Ch2 & Ch3 AD Value
#define STATUSEX_TYPE_ADVALUE4		32	// Ezi-IO AD: Ch4 & Ch5 AD Value
#define STATUSEX_TYPE_ADVALUE6		33	// Ezi-IO AD: Ch6 & Ch7 AD Value
#define STATUSEX_TYPE_CNTSTATUS		40	// Ezi-IO COUNTER: Status
#define STATUSEX_TYPE_CNTCH1_COUNT	41	// Ezi-IO COUNTER: CH1 COUNT
#define STATUSEX_TYPE_CNTCH1_LATCHA	42	// Ezi-IO COUNTER: CH1 LATCHA
#define STATUSEX_TYPE_CNTCH1_LATChB	43	// Ezi-IO COUNTER: CH1 LATCHB
#define STATUSEX_TYPE_CNTCH1_ZLATCH	44	// Ezi-IO COUNTER: CH1 Z LATCH
#define STATUSEX_TYPE_CNTCH2_COUNT	45	// Ezi-IO COUNTER: CH2 COUNT
#define STATUSEX_TYPE_CNTCH2_LATCHA	46	// Ezi-IO COUNTER: CH2 LATCHA
#define STATUSEX_TYPE_CNTCH2_LATChB	47	// Ezi-IO COUNTER: CH2 LATCHB
#define STATUSEX_TYPE_CNTCH2_ZLATCH	48	// Ezi-IO COUNTER: CH2 Z LATCH


//------------------------------------------------------------------
//                 Input/Output Assigning Defines.
//------------------------------------------------------------------

static const BYTE	LEVEL_LOW_ACTIVE	= 0;
static const BYTE	LEVEL_HIGH_ACTIVE	= 1;

static const BYTE	IN_LOGIC_NONE	= 0;
static const BYTE	OUT_LOGIC_NONE	= 0;

//------------------------------------------------------------------
//                 POSITION TABLE Defines.
//------------------------------------------------------------------

static const WORD	MAX_LOOP_COUNT	= 100;
static const WORD	MAX_WAIT_TIME	= 60000;

static const WORD	PUSH_RATIO_MIN	= 20;
static const WORD	PUSH_RATIO_MAX	= 90;

static const DWORD	PUSH_SPEED_MIN	= 1;
static const DWORD	PUSH_SPEED_MAX	= 100000;

static const DWORD	PUSH_PULSECOUNT_MIN	= 1;
static const DWORD	PUSH_PULSECOUNT_MAX	= 10000;

typedef enum
{
	CMD_ABS_LOWSPEED = 0,
	CMD_ABS_HIGHSPEED,
	CMD_ABS_HIGHSPEEDDECEL,
	CMD_ABS_NORMALMOTION,
	CMD_INC_LOWSPEED,
	CMD_INC_HIGHSPEED,
	CMD_INC_HIGHSPEEDDECEL,
	CMD_INC_NORMALMOTION,
	CMD_MOVE_ORIGIN,
	CMD_COUNTERCLEAR,
	CMD_PUSH_ABSMOTION,
	CMD_STOP,

	CMD_MAX_COUNT,

	CMD_NO_COMMAND = 0xFFFF,
} COMMAND_LIST;

#ifndef	DEFINE_ITEM_NODE
#define DEFINE_ITEM_NODE

#pragma pack(2)

typedef union
{
	WORD	wBuffer[32];		// 64 bytes

	struct
	{
		LONG	lPosition;

		DWORD	dwStartSpd;
		DWORD	dwMoveSpd;

		WORD	wAccelRate;
		WORD	wDecelRate;

		WORD	wCommand;
		WORD	wWaitTime;
		WORD	wContinuous;
		WORD	wBranch;

		WORD	wCond_branch0;
		WORD	wCond_branch1;
		WORD	wCond_branch2;
		WORD	wLoopCount;
		WORD	wBranchAfterLoop;
		WORD	wPTSet;
		WORD	wLoopCountCLR;

		WORD	bCheckInpos;		// 0 : Check Inpos, 1 : Don't Check.

		LONG	lTriggerPos;
		WORD	wTriggerOnTime;

		WORD	wPushRatio;
		DWORD	dwPushSpeed;
		LONG	lPushPosition;
		WORD	wPushMode;
	};
} ITEM_NODE, *LPITEM_NODE;

#pragma pack()

#endif

static const WORD	OFFSET_POSITION		= 0;
static const WORD	OFFSET_LOWSPEED		= 4;
static const WORD	OFFSET_HIGHSPEED	= 8;
static const WORD	OFFSET_ACCELTIME	= 12;
static const WORD	OFFSET_DECELTIME	= 14;
static const WORD	OFFSET_COMMAND		= 16;
static const WORD	OFFSET_WAITTIME		= 18;
static const WORD	OFFSET_CONTINUOUS	= 20;
static const WORD	OFFSET_JUMPTABLENO	= 22;
static const WORD	OFFSET_JUMPPT0		= 24;
static const WORD	OFFSET_JUMPPT1		= 26;
static const WORD	OFFSET_JUMPPT2		= 28;
static const WORD	OFFSET_LOOPCOUNT		= 30;
static const WORD	OFFSET_LOOPJUMPTABLENO	= 32;
static const WORD	OFFSET_PTSET			= 34;
static const WORD	OFFSET_LOOPCOUNTCLEAR	= 36;
static const WORD	OFFSET_CHECKINPOSITION	= 38;
static const WORD	OFFSET_TRIGGERPOSITION	= 40;
static const WORD	OFFSET_TRIGGERONTIME	= 44;
static const WORD	OFFSET_PUSHRATIO		= 46;
static const WORD	OFFSET_PUSHSPEED		= 48;
static const WORD	OFFSET_PUSHPOSITION		= 52;
static const WORD	OFFSET_PUSHMODE			= 56;

static const WORD	OFFSET_BLANK			= 58;

//------------------------------------------------------------------
//                 EX Commands Option Defines.
//------------------------------------------------------------------
#pragma pack(1)

typedef union
{
	BYTE	byBuffer[32];

	struct
	{
		union
		{
			DWORD dwOptionFlag;
			struct
			{
				unsigned BIT_IGNOREEXSTOP	: 1;

				unsigned BIT_USE_CUSTOMACCEL	: 1;
				unsigned BIT_USE_CUSTOMDECEL	: 1;

				//unsigned BITS_RESERVED	: 13;
			};
		} flagOption;

		WORD	wCustomAccelTime;
		WORD	wCustomDecelTime;

		//BYTE	buffReserved[24];
	};
} MOTION_OPTION_EX;

typedef union
{
	BYTE	byBuffer[32];

	struct
	{
		union
		{
			DWORD dwOptionFlag;
			struct
			{
				unsigned BIT_IGNOREEXSTOP	: 1;
				unsigned BIT_USE_CUSTOMACCDEC	: 1;

				//unsigned BITS_RESERVED	: 14;
			};
		} flagOption;

		WORD	wCustomAccDecTime;

		//BYTE	buffReserved[26];
	};
} VELOCITY_OPTION_EX;

#pragma pack()

//------------------------------------------------------------------
//                 Alarm Type Defines.
//------------------------------------------------------------------
typedef enum
{
	ALARM_NONE = 0,

	/*  1 */ ALARM_OVERCURRENT,
	/*  2 */ ALARM_OVERSPEED,
	/*  3 */ ALARM_STEPOUT,
	/*  4 */ ALARM_OVERLOAD,
	/*  5 */ ALARM_OVERTEMPERATURE,
	/*  6 */ ALARM_OVERBACKEMF,
	/*  7 */ ALARM_MOTORCONNECT,
	/*  8 */ ALARM_ENCODERCONNECT,
	/*  9 */ ALARM_LOWMOTORPOWER,
	/* 10 */ ALARM_INPOSITION,
	/* 11 */ ALARM_SYSTEMHALT,
	/* 12 */ ALARM_ROMDEVICE,
	/* 13 */ ALARM_RESERVED0,
	/* 14 */ ALARM_HIGHINPUTVOLTAGE,
	/* 15 */ ALARM_POSITIONOVERFLOW,
	/* 16 */ ALARM_POSITIONCHANGED,

	MAX_ALARM_NUM
} ALARM_TYPE;

static LPCTSTR ALARM_DESCRIPTION[MAX_ALARM_NUM] =
{
	NULL,
	/*  1 */ _T("Over Current"),
	/*  2 */ _T("Over Speed"),
	/*  3 */ _T("Position Tracking"),
	/*  4 */ _T("Over Load"),
	/*  5 */ _T("Over Temperature"),
	/*  6 */ _T("Over Back EMF"),
	/*  7 */ _T("No Motor Connect"),
	/*  8 */ _T("No Encoder Connect"),
	/*  9 */ _T("Low Motor Power"),
	/* 10 */ _T("Inposition Error"),
	/* 11 */ _T("System Halt"),
	/* 12 */ _T("ROM Device Error"),
	/* 13 */ NULL,
	/* 14 */ _T("High Input Voltage"),
	/* 15 */ _T("Position Overflow"),
	/* 16 */ _T("Position Changed")
};

#define MAX_ALARM_LOG		(30)

typedef struct
{
	BYTE nAlarmCount;
	BYTE nAlarmLog[MAX_ALARM_LOG];
} ALARM_LOG;

//------------------------------------------------------------------
//                 Drive Information Defines.
//------------------------------------------------------------------
#pragma pack(1)

typedef struct _DRIVE_INFO
{
	unsigned short nVersionNo[4];	// Drive Version Number (Major Ver/Minor Ver/Bug fix/Build No.) (?)
	char sVersion[30];				// Drive Version string

	unsigned short nDriveType;		// Drive Model
	unsigned short nMotorType;		// Motor Model
	char sMotorInfo[20];			// Motor Info.(?)

	unsigned short nInPinNo;		// Input Pin Number
	unsigned short nOutPinNo;		// Output Pin Number

	unsigned short nPTNum;			// Position Table Item Number

	unsigned short nFirmwareType;	// Firmware Type Information
} DRIVE_INFO;

#pragma pack()

//------------------------------------------------------------------
//                 I/O Module Defines.
//------------------------------------------------------------------

#pragma pack(2)

typedef union
{
	BYTE	byBuffer[12];

	struct
	{
		unsigned short	wPeriod;
		unsigned short	wReserved1;
		unsigned short	wOnTime;
		unsigned short	wReserved2;
		unsigned long	wCount;
	};
} TRIGGER_INFO;

#pragma pack()

//------------------------------------------------------------------
//                 Ezi-IO AD Defines
//------------------------------------------------------------------

#define MAX_AD_CHANNEL		16

enum AD_RANGE
{
	ADRANGE_10_to_10 = 0,	//  -10V ~  10V [2.441mV]
	ADRANGE_5_to_5,			//   -5V ~   5V [1.22mV]
	ADRANGE_2_5_to_2_5,		// -2.5V ~ 2.5V [0.61mV]
	ADRANGE_0_to_10,		//    0V ~  10V [1.22mV]
	ADRANGE_0_to_20_C,		//   0mA ~ 20mA [2.44uA]

	ADRANGE_EX_10_to_10 = 0x10,	// (Read Only, External Switch)  -10V ~  10V [2.441mV]
	ADRANGE_EX_5_to_5,			// (Read Only, External Switch)   -5V ~   5V [1.22mV]
	ADRANGE_EX_2_5_to_2_5,		// (Read Only, External Switch) -2.5V ~ 2.5V [0.61mV]
	ADRANGE_EX_0_to_10,			// (Read Only, External Switch)    0V ~  10V [1.22mV]
};

#pragma pack(1)

typedef union
{
	BYTE	byBuffer[112];

	struct DATA
	{
		char	range;
		short	rawdata;
		float	converted;
	} channel[16];
} AD_RESULT;

enum AD_DATA_TYPE
{
	TYPE_AD_RANGE = 0,
	TYPE_AD_FILTER_LENGTH,
	TYPE_AD_FILTER_OFFSET,
};

typedef union
{
	BYTE	byBuffer[16];

	short	channel[8];
} AD_BUFFER;

#pragma pack()

//------------------------------------------------------------------
//                 Ezi-IO DA Defines
//------------------------------------------------------------------

enum DA_RANGE
{
	DARANGE_0_to_5 = 0,		//    0V ~   5V (0 ~ 25000)
	DARANGE_10_to_10,		//  -10V ~  10V (-25000 ~ 25000)
	DARANGE_0_to_10,		//    0V ~  10V (0 ~ 25000)
	DARANGE_1_to_5,			//    1V ~   5V (0 ~ 25000)
	DARANGE_0_to_20_C,		//   0mA ~ 20mA (0 ~ 25000)
	DARANGE_4_to_20_C,		//   4mA ~ 20mA (0 ~ 25000)

	DARANGE_EX_0_to_5 = 0x10,	// (Read Only, External Switch)    0V ~   5V (0 ~ 25000)
	DARANGE_EX_10_to_10,		// (Read Only, External Switch)  -10V ~  10V (-25000 ~ 25000)
	DARANGE_EX_0_to_20_C,		// (Read Only, External Switch)   0mA ~ 20mA (0 ~ 25000)
	DARANGE_EX_4_to_20_C,		// (Read Only, External Switch)   4mA ~ 20mA (0 ~ 25000)
};

enum DAC_CONFIG
{
	DAC_RANGE = 0,

	DAC_CALIBRATION_HIGH,
	DAC_CALIBRATION_LOW,
};

//------------------------------------------------------------------
//                 Ezi-IO COUNTER Defines
//------------------------------------------------------------------

enum COUNTER_CMD
{
	CNTCMD_CH_ENABLE = 0,
	CNTCMD_LATCHA_ENABLE,
	CNTCMD_LATCHB_ENABLE,
	CNTCMD_ZLATCH_ENABLE,

	CNTCMD_RESET_ALL,
	CNTCMD_RESET_COUNT,
	CNTCMD_RESET_LATCH,
};

enum COUNTER_VALUE
{
	CNT_COUNT_VALUE = 0,
	CNT_LATCHA_VALUE,
	CNT_LATCHB_VALUE,
	CNT_ZLATCH_VALUE,
};

#define CNTSTATUS_CH1_ENABLED			0x00000001
#define CNTSTATUS_CH1_LTCA_EN			0x00000002
#define CNTSTATUS_CH1_LTCB_EN			0x00000004
#define CNTSTATUS_CH1_ZLTC_EN			0x00000008
#define CNTSTATUS_CH1_LTCA_LATCHED		0x00000010
#define CNTSTATUS_CH1_LTCB_LATCHED		0x00000020
#define CNTSTATUS_CH1_ZLTC_LATCHED		0x00000040
#define CNTSTATUS_CH1_RESET				0x00000080
#define CNTSTATUS_CH1_TRIGGER			0x00000100
#define CNTSTATUS_CH1_COMPARISON		0x00000200
#define CNTSTATUS_CH2_ENABLED			0x00010000
#define CNTSTATUS_CH2_LTCA_EN			0x00020000
#define CNTSTATUS_CH2_LTCB_EN			0x00040000
#define CNTSTATUS_CH2_ZLTC_EN			0x00080000
#define CNTSTATUS_CH2_LTCA_LATCHED		0x00100000
#define CNTSTATUS_CH2_LTCB_LATCHED		0x00200000
#define CNTSTATUS_CH2_ZLTC_LATCHED		0x00400000
#define CNTSTATUS_CH2_RESET				0x00800000
#define CNTSTATUS_CH2_TRIGGER			0x01000000
#define CNTSTATUS_CH2_COMPARISON		0x02000000

enum COUNTER_CONFIG
{
	CFG_INPUT_MODE = 0,		// Counter Input Mode (0: Quadrature, 1: 2-Pulse, 2: 1-Pulse)
	CFG_COUNT_DIRECTION,	// Count Direction (0: CW, 1: CCW)
	CFG_LATCHA_MODE,		// Latch A Mode (0: Single, 1: Continue)
	CFG_LATCHB_MODE,		// Latch B Mode (0: Single, 1: Continue)
	CFG_ZLATCH_MODE,		// Z-Phase Latch Mode (0: Single, 1: Continue, 2: Reset)
	CFG_LATCHA_LOGIC,		// Latch A Logic (0: Low-Active, 1: High-Active)
	CFG_LATCHB_LOGIC,		// Latch B Logic (0: Low-Active, 1: High-Active)
	CFG_ZLATCH_LOGIC,		// Z-Phase Latch Logic (0: Low-Active, 1: High-Active)
	CFG_RESET_LOGIC,		// (external input) Reset Logic (0: Low-Active, 1: High-Active)
	CFG_CP_LOGIC,			// (external output) Comparison Output Logic (0: Low-Active, 1: High-Active)
};

//------------------------------------------------------------------
//                 LOG Level Defines
//------------------------------------------------------------------

enum LOG_LEVEL
{
	LOG_LEVEL_COMM = 0,		// Communication Log only
	LOG_LEVEL_PARAM,		// Communication Log and parameter functions
	LOG_LEVEL_MOTION,		// Communication Log and parameter, motion, I/O functions
	LOG_LEVEL_ALL,			// Communication Log and all functions
};
