#pragma once
#include <Windows.h>

#ifdef EZI_SERVO_PLUSE_EXPORTS
	#define EZI_PLUSE_API __declspec(dllexport)

	#include "MOTION_DEFINE.h"
#else
	#define EZI_PLUSE_API __declspec(dllimport)
	#pragma comment( lib, "EziMOTIONPlusE.lib" )

	#include "RETURNCODES_DEFINE.h"
	#include "MOTION_DEFINE.h"
#endif

namespace PE
{
//------------------------------------------------------------------------------
//			Connection Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API BOOL WINAPI	FAS_Connect(BYTE sb1, BYTE sb2, BYTE sb3, BYTE sb4, int iBdID);		// UDP Protocol
EZI_PLUSE_API BOOL WINAPI	FAS_ConnectTCP(BYTE sb1, BYTE sb2, BYTE sb3, BYTE sb4, int iBdID);	// TCP Protocol

EZI_PLUSE_API BOOL WINAPI	FAS_IsBdIDExist(int iBdID, BYTE* sb1, BYTE* sb2, BYTE* sb3, BYTE* sb4);
EZI_PLUSE_API BOOL WINAPI	FAS_IsIPAddressExist(BYTE sb1, BYTE sb2, BYTE sb3, BYTE sb4, int* iBdID);

EZI_PLUSE_API BOOL WINAPI	FAS_Reconnect(int iBdID);
EZI_PLUSE_API void WINAPI	FAS_SetAutoReconnect(BOOL bSET);

EZI_PLUSE_API void WINAPI	FAS_Close(int iBdID);

EZI_PLUSE_API BOOL WINAPI	FAS_IsSlaveExist(int iBdID);

//------------------------------------------------------------------------------
//			Ethernet Address Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GetEthernetAddr(int iBdID, unsigned long* gateway, unsigned long* subnet, unsigned long* ip);
EZI_PLUSE_API int WINAPI	FAS_SetEthernetAddr(int iBdID, unsigned long gateway, unsigned long subnet, unsigned long ip);

EZI_PLUSE_API int WINAPI	FAS_GetMACAddress(int iBdID, unsigned long long* MACAddress);

EZI_PLUSE_API int WINAPI	FAS_EnableTwoTCPPort(int iBdID, BOOL bEnable);

//------------------------------------------------------------------------------
//			Log Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API void WINAPI	FAS_EnableLog(BOOL bEnable);
EZI_PLUSE_API void WINAPI	FAS_SetLogLevel(enum LOG_LEVEL level);
EZI_PLUSE_API BOOL WINAPI	FAS_SetLogPath(LPCWSTR lpPath);

EZI_PLUSE_API void WINAPI	FAS_PrintCustomLog(int iBdID, enum LOG_LEVEL level, LPCTSTR lpszMsg);

//------------------------------------------------------------------------------
//			Info Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GetSlaveInfo(int iBdID, BYTE* pType, LPSTR lpBuff, int nBuffSize);
EZI_PLUSE_API int WINAPI	FAS_GetMotorInfo(int iBdID, BYTE* pType, LPSTR lpBuff, int nBuffSize);
EZI_PLUSE_API int WINAPI	FAS_GetSlaveInfoEx(int iBdID, DRIVE_INFO* lpDriveInfo);

//------------------------------------------------------------------------------
//			Parameter Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_SaveAllParameters(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_SetParameter(int iBdID, BYTE iParamNo, long lParamValue);
EZI_PLUSE_API int WINAPI	FAS_GetParameter(int iBdID, BYTE iParamNo, long* lParamValue);
EZI_PLUSE_API int WINAPI	FAS_GetROMParameter(int iBdID, BYTE iParamNo, long* lRomParam);

//------------------------------------------------------------------------------
//			IO Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_SetIOInput(int iBdID, DWORD dwIOSETMask, DWORD dwIOCLRMask);
EZI_PLUSE_API int WINAPI	FAS_GetIOInput(int iBdID, DWORD* dwIOInput);

EZI_PLUSE_API int WINAPI	FAS_SetIOOutput(int iBdID, DWORD dwIOSETMask, DWORD dwIOCLRMask);
EZI_PLUSE_API int WINAPI	FAS_GetIOOutput(int iBdID, DWORD* dwIOOutput);

EZI_PLUSE_API int WINAPI	FAS_GetIOAssignMap(int iBdID, BYTE iIOPinNo, DWORD* dwIOLogicMask, BYTE* bLevel);
EZI_PLUSE_API int WINAPI	FAS_SetIOAssignMap(int iBdID, BYTE iIOPinNo, DWORD dwIOLogicMask, BYTE bLevel);

EZI_PLUSE_API int WINAPI	FAS_IOAssignMapReadROM(int iBdID);

//------------------------------------------------------------------------------
//			Servo Driver Control Functions
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_ServoEnable(int iBdID, BOOL bOnOff);
EZI_PLUSE_API int WINAPI	FAS_ServoAlarmReset(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_StepAlarmReset(int iBdID, BOOL bReset);
EZI_PLUSE_API int WINAPI	FAS_BrakeSet(int iBdID, BOOL bSet, int* nResult);

//------------------------------------------------------------------------------
//			Read Status and Position
//------------------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GetAxisStatus(int iBdID, DWORD* dwAxisStatus);
EZI_PLUSE_API int WINAPI	FAS_GetIOAxisStatus(int iBdID, DWORD* dwInStatus, DWORD* dwOutStatus, DWORD* dwAxisStatus);
EZI_PLUSE_API int WINAPI	FAS_GetMotionStatus(int iBdID, long* lCmdPos, long* lActPos, long* lPosErr, long* lActVel, WORD* wPosItemNo);
EZI_PLUSE_API int WINAPI	FAS_GetAllStatus(int iBdID, DWORD* dwInStatus, DWORD* dwOutStatus, DWORD* dwAxisStatus, long* lCmdPos, long* lActPos, long* lPosErr, long* lActVel, WORD* wPosItemNo);
EZI_PLUSE_API int WINAPI	FAS_GetAllStatusEx(int iBdID, BYTE* pTypes, long* pDatas);

EZI_PLUSE_API int WINAPI	FAS_SetCommandPos(int iBdID, long lCmdPos);
EZI_PLUSE_API int WINAPI	FAS_SetActualPos(int iBdID, long lActPos);
EZI_PLUSE_API int WINAPI	FAS_ClearPosition(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_GetCommandPos(int iBdID, long* lCmdPos);
EZI_PLUSE_API int WINAPI	FAS_GetActualPos(int iBdID, long* lActPos);
EZI_PLUSE_API int WINAPI	FAS_GetPosError(int iBdID, long* lPosErr);
EZI_PLUSE_API int WINAPI	FAS_GetActualVel(int iBdID, long* lActVel);

EZI_PLUSE_API int WINAPI	FAS_GetAlarmType(int iBdID, BYTE* nAlarmType);

//------------------------------------------------------------------
//			Motion Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_MoveStop(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_EmergencyStop(int iBdID);

EZI_PLUSE_API int WINAPI	FAS_MovePause(int iBdID, BOOL bPause);

EZI_PLUSE_API int WINAPI	FAS_MoveOriginSingleAxis(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_MoveSingleAxisAbsPos(int iBdID, long lAbsPos, DWORD lVelocity);
EZI_PLUSE_API int WINAPI	FAS_MoveSingleAxisIncPos(int iBdID, long lIncPos, DWORD lVelocity);
EZI_PLUSE_API int WINAPI	FAS_MoveToLimit(int iBdID, DWORD lVelocity, int iLimitDir);
EZI_PLUSE_API int WINAPI	FAS_MoveVelocity(int iBdID, DWORD lVelocity, int iVelDir);

EZI_PLUSE_API int WINAPI	FAS_PositionAbsOverride(int iBdID, long lOverridePos);
EZI_PLUSE_API int WINAPI	FAS_PositionIncOverride(int iBdID, long lOverridePos);
EZI_PLUSE_API int WINAPI	FAS_VelocityOverride(int iBdID, DWORD lVelocity);

EZI_PLUSE_API int WINAPI	FAS_MoveLinearAbsPos(BYTE nNoOfBds, int* iBdID, long* lplAbsPos, DWORD lFeedrate, WORD wAccelTime);
EZI_PLUSE_API int WINAPI	FAS_MoveLinearIncPos(BYTE nNoOfBds, int* iBdID, long* lplIncPos, DWORD lFeedrate, WORD wAccelTime);

EZI_PLUSE_API int WINAPI	FAS_MoveLinearAbsPos2(BYTE nNoOfBds, int* iBdID, long* lplAbsPos, DWORD lFeedrate, WORD wAccelTime);
EZI_PLUSE_API int WINAPI	FAS_MoveLinearIncPos2(BYTE nNoOfBds, int* iBdID, long* lplIncPos, DWORD lFeedrate, WORD wAccelTime);

EZI_PLUSE_API int WINAPI	FAS_MoveCircleAbsPos1(BYTE nNoOfBds, int* iBdID, long* lplCirEndAbs, long* lplCirCenterAbs, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);
EZI_PLUSE_API int WINAPI	FAS_MoveCircleIncPos1(BYTE nNoOfBds, int* iBdID, long* lplCirEndInc, long* lplCirCenterInc, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);
EZI_PLUSE_API int WINAPI	FAS_MoveCircleAbsPos2(BYTE nNoOfBds, int* iBdID, long* lplCirEndAbs, DWORD lRadius, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);
EZI_PLUSE_API int WINAPI	FAS_MoveCircleIncPos2(BYTE nNoOfBds, int* iBdID, long* lplCirEndInc, DWORD lRadius, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);
EZI_PLUSE_API int WINAPI	FAS_MoveCircleAbsPos3(BYTE nNoOfBds, int* iBdID, long* lplCirCenterAbs, DWORD nAngle, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);
EZI_PLUSE_API int WINAPI	FAS_MoveCircleIncPos3(BYTE nNoOfBds, int* iBdID, long* lplCirCenterInc, DWORD nAngle, int iDirection, DWORD lFeedrate, WORD wAccelTime, int bSCurve);

EZI_PLUSE_API int WINAPI	FAS_TriggerOutput_RunA(int iBdID, BOOL bStartTrigger, long lStartPos, DWORD dwPeriod, DWORD dwPulseTime);
EZI_PLUSE_API int WINAPI	FAS_TriggerOutput_Status(int iBdID, BYTE* bTriggerStatus);

EZI_PLUSE_API int WINAPI	FAS_SetTriggerOutputEx(int iBdID, BYTE nOutputNo, BYTE bRun, WORD wOnTime, BYTE nTriggerCount, long* arrTriggerPosition);
EZI_PLUSE_API int WINAPI	FAS_GetTriggerOutputEx(int iBdID, BYTE nOutputNo, BYTE* bRun, WORD* wOnTime, BYTE* nTriggerCount, long* arrTriggerPosition);

EZI_PLUSE_API int WINAPI	FAS_MovePush(int iBdID, DWORD dwStartSpd, DWORD dwMoveSpd, long lPosition, WORD wAccel, WORD wDecel, WORD wPushRate, DWORD dwPushSpd, long lEndPosition, WORD wPushMode);
EZI_PLUSE_API int WINAPI	FAS_GetPushStatus(int iBdID, BYTE* nPushStatus);

//------------------------------------------------------------------
//			Ex-Motion Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_MoveSingleAxisAbsPosEx(int iBdID, long lAbsPos, DWORD lVelocity, MOTION_OPTION_EX* lpExOption);
EZI_PLUSE_API int WINAPI	FAS_MoveSingleAxisIncPosEx(int iBdID, long lIncPos, DWORD lVelocity, MOTION_OPTION_EX* lpExOption);
EZI_PLUSE_API int WINAPI	FAS_MoveVelocityEx(int iBdID, DWORD lVelocity, int iVelDir, VELOCITY_OPTION_EX* lpExOption);

//------------------------------------------------------------------
//			Position Table Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_PosTableReadItem(int iBdID, WORD wItemNo, LPITEM_NODE lpItem);
EZI_PLUSE_API int WINAPI	FAS_PosTableWriteItem(int iBdID, WORD wItemNo, LPITEM_NODE lpItem);
EZI_PLUSE_API int WINAPI	FAS_PosTableWriteROM(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_PosTableReadROM(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_PosTableRunItem(int iBdID, WORD wItemNo);

EZI_PLUSE_API int WINAPI	FAS_PosTableReadOneItem(int iBdID, WORD wItemNo, WORD wOffset, long* lPosItemVal);
EZI_PLUSE_API int WINAPI	FAS_PosTableWriteOneItem(int iBdID, WORD wItemNo, WORD wOffset, long lPosItemVal);

EZI_PLUSE_API int WINAPI	FAS_PosTableSingleRunItem(int iBdID, BOOL bNextMove, WORD wItemNo);

//------------------------------------------------------------------
//			Gap Control Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GapControlEnable(int iBdID, WORD wItemNo, long lGapCompSpeed, long lGapAccTime, long lGapDecTime, long lGapStartSpeed);
EZI_PLUSE_API int WINAPI	FAS_GapControlDisable(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_IsGapControlEnable(int iBdID, BOOL* bIsEnable, WORD* wCurrentItemNo);

EZI_PLUSE_API int WINAPI	FAS_GapControlGetADCValue(int iBdID, long* lADCValue);
EZI_PLUSE_API int WINAPI	FAS_GapOneResultMonitor(int iBdID, BYTE* bUpdated, long* iIndex, long* lGapValue, long* lCmdPos, long* lActPos, long* lCompValue, long* lReserved);

//------------------------------------------------------------------
//			Alarm Type History Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GetAlarmLogs(int iBdID, ALARM_LOG* pAlarmLog);
EZI_PLUSE_API int WINAPI	FAS_ResetAlarmLogs(int iBdID);

//------------------------------------------------------------------
//			I/O Module Functions.
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_GetInput(int iBdID, unsigned long* uInput, unsigned long* uLatch);

EZI_PLUSE_API int WINAPI	FAS_ClearLatch(int iBdID, unsigned long uLatchMask);
EZI_PLUSE_API int WINAPI	FAS_GetLatchCount(int iBdID, unsigned char iInputNo, unsigned long* uCount);
EZI_PLUSE_API int WINAPI	FAS_GetLatchCountAll(int iBdID, unsigned long** ppuAllCount);
EZI_PLUSE_API int WINAPI	FAS_GetLatchCountAll32(int iBdID, unsigned long** ppuAllCount);
EZI_PLUSE_API int WINAPI	FAS_ClearLatchCount(int iBdID, unsigned long uInputMask);

EZI_PLUSE_API int WINAPI	FAS_GetOutput(int iBdID, unsigned long* uOutput, unsigned long* uStatus);
EZI_PLUSE_API int WINAPI	FAS_SetOutput(int iBdID, unsigned long uSet, unsigned long uClear);

EZI_PLUSE_API int WINAPI	FAS_SetTrigger(int iBdID, unsigned char uOutputNo, TRIGGER_INFO* pTrigger);
EZI_PLUSE_API int WINAPI	FAS_SetRunStop(int iBdID, unsigned long uRun, unsigned long uStop);
EZI_PLUSE_API int WINAPI	FAS_GetTriggerCount(int iBdID, unsigned char uOutputNo, unsigned long* uCount);

EZI_PLUSE_API int WINAPI	FAS_GetIOLevel(int iBdID, unsigned long* uIOLevel);
EZI_PLUSE_API int WINAPI	FAS_SetIOLevel(int iBdID, unsigned long uIOLevel);
EZI_PLUSE_API int WINAPI	FAS_LoadIOLevel(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_SaveIOLevel(int iBdID);

EZI_PLUSE_API int WINAPI	FAS_GetInputFilter(int iBdID, unsigned short* filter);
EZI_PLUSE_API int WINAPI	FAS_SetInputFilter(int iBdID, unsigned short filter);

EZI_PLUSE_API int WINAPI	FAS_GetIODirection(int iBdID, unsigned long* direction);
EZI_PLUSE_API int WINAPI	FAS_SetIODirection(int iBdID, unsigned long direction);

//------------------------------------------------------------------
//			Ezi-IO AD Functions
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_SetADConfig(int iBdID, BYTE channel, enum AD_DATA_TYPE type, long value, long* recv);
EZI_PLUSE_API int WINAPI	FAS_GetADConfig(int iBdID, BYTE channel, enum AD_DATA_TYPE type, long* value);

EZI_PLUSE_API int WINAPI	FAS_LoadADConfig(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_SaveADConfig(int iBdID);

EZI_PLUSE_API int WINAPI	FAS_ReadADValue(int iBdID, BYTE channel, short* advalue);
EZI_PLUSE_API int WINAPI	FAS_ReadADAllValue(int iBdID, BYTE offset, AD_BUFFER* adbuffer);

EZI_PLUSE_API int WINAPI	FAS_GetAllADResult(int iBdID, AD_RESULT* result);
EZI_PLUSE_API int WINAPI	FAS_GetADResult(int iBdID, BYTE channel, float* adresult);
EZI_PLUSE_API int WINAPI	FAS_SetADRange(int iBdID, BYTE channel, enum AD_RANGE range);

//------------------------------------------------------------------
//			Ezi-IO DA Functions
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_SetDACConfig(int iBdID, BYTE channel, DAC_CONFIG type, long data, long* recv);
EZI_PLUSE_API int WINAPI	FAS_GetDACConfig(int iBdID, BYTE channel, DAC_CONFIG type, long* data);

EZI_PLUSE_API int WINAPI	FAS_LoadDACConfig(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_SaveDACConfig(int iBdID);

EZI_PLUSE_API int WINAPI	FAS_SetDACValue(int iBdID, BYTE channel, BOOL bEnable, long value);
EZI_PLUSE_API int WINAPI	FAS_GetDACValue(int iBdID, BYTE channel, BOOL* bEnable, long* value);

//------------------------------------------------------------------
//			Ezi-IO Counter Functions
//------------------------------------------------------------------
EZI_PLUSE_API int WINAPI	FAS_CounterCommand(int iBdID, BYTE channel, COUNTER_CMD cmd, long data);

EZI_PLUSE_API int WINAPI	FAS_GetCounterValue(int iBdID, BYTE channel, COUNTER_VALUE type, long* value);
EZI_PLUSE_API int WINAPI	FAS_GetCounterStatus(int iBdID, DWORD* dwStatus);

EZI_PLUSE_API int WINAPI	FAS_SetCounterTrigger(int iBdID, BYTE channel, BOOL bStartTrigger, long lStartPos, DWORD dwPeriod, DWORD dwPulseTime, DWORD dwTriggerCnt);

EZI_PLUSE_API int WINAPI	FAS_SetCounterConfig(int iBdID, BYTE channel, COUNTER_CONFIG type, long data, long* recv);
EZI_PLUSE_API int WINAPI	FAS_GetCounterConfig(int iBdID, BYTE channel, COUNTER_CONFIG type, long* data);

EZI_PLUSE_API int WINAPI	FAS_LoadCounterConfig(int iBdID);
EZI_PLUSE_API int WINAPI	FAS_SaveCounterConfig(int iBdID);

} // namespace PE
