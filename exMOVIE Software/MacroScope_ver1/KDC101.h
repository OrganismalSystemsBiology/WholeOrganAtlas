#pragma once
// Example_KDC101.cpp : Defines the entry point for the console application.
//
#include "Thorlabs.MotionControl.KCube.DCServo2.h"


int Focus_stage_connection(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	if (KCubeDCServo::TLI_BuildDeviceList() == 0)
	{
		// get device list size 
		short n = KCubeDCServo::TLI_GetDeviceListSize();
		// get KDC serial numbers
		char serialNos[100];
		KCubeDCServo::TLI_GetDeviceListByTypeExt(serialNos, 100, 27);
		char *searchContext = nullptr;
		char *p = strtok_s(serialNos, ",", &searchContext);

		while (p != nullptr)
		{
			KCubeDCServo::TLI_DeviceInfo deviceInfo;
			// get device info from device
			KCubeDCServo::TLI_GetDeviceInfo(p, &deviceInfo);
			// get strings from device info structure
			char desc[65];
			strncpy_s(desc, deviceInfo.description, 64);
			desc[64] = '\0';
			char serialNo[9];
			strncpy_s(serialNo, deviceInfo.serialNo, 8);
			serialNo[8] = '\0';
			// output
			printf("Found Device %s, Serial :%s \r\n", p, desc, SerialNo);
			p = strtok_s(nullptr, ",", &searchContext);
		}

		// open device
		if (KCubeDCServo::CC_Open(SerialNo) == 0)
		{
			// start the device polling at 200ms intervals
			KCubeDCServo::CC_StartPolling(SerialNo, 200);
			Sleep(3000);
			// Home device
			KCubeDCServo::CC_ClearMessageQueue(SerialNo);
			//KCubeDCServo::CC_Home(testSerialNo);
			//printf("Device %s homing\r\n", testSerialNo);

			// wait for completion
			/*
			WORD messageType;
			WORD messageId;
			DWORD messageData;
			KCubeDCServo::CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
			while (messageType != 2 || messageId != 0)
			{
				KCubeDCServo::CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
			}*/

		}
		// get actual poaition
		int pos = KCubeDCServo::CC_GetPosition(SerialNo);
	
		return pos;

	}

}
int Get_Focus_Fos(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	int pos = KCubeDCServo::CC_GetPosition(SerialNo);
	printf("FocusPos: %d\r\n", pos);
	return pos;
}
void Close_Focus_stage(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	KCubeDCServo::CC_StopPolling(SerialNo);
	KCubeDCServo::CC_Close(SerialNo);

}
int Fstage_GotoPos(int serialNo, int  position) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	// move to position (channel 1)
	KCubeDCServo::CC_ClearMessageQueue(SerialNo);
	KCubeDCServo::CC_MoveToPosition(SerialNo, position);
	printf("Focus Stage %s moving\r\n", SerialNo);

	WORD messageType;
	WORD messageId;
	DWORD messageData;
	// wait for completion
	KCubeDCServo::CC_WaitForMessage(SerialNo, &messageType, &messageId, &messageData);
	while (messageType != 2 || messageId != 1)
	{
		KCubeDCServo::CC_WaitForMessage(SerialNo, &messageType, &messageId, &messageData);
	}

	// get actual poaition
	int pos = KCubeDCServo::CC_GetPosition(SerialNo);
	printf("Focus Stage %s moved to %d\r\n", SerialNo, pos);
	return pos;


}

/*
<seealso cref = "CC_RequestJogParams(char const * serialNo)" / >
/// <seealso cref="CC_SetJogMode(char const * serialNo, MOT_JogModes mode, MOT_StopModes stopMode)" />
/// <seealso cref="CC_GetJogStepSize(char const * serialNo)" />
/// <seealso cref="CC_SetJogStepSize(char const * serialNo, unsigned int stepSize)" />
/// <seealso cref="CC_SetJogVelParams(char const * serialNo, int acceleration, int maxVelocity)" />
/// <seealso cref="CC_GetJogVelParams(char const * serialNo, int * acceleration, int * maxVelocity)" />
/// <seealso cref="CC_GetJogParamsBlock(char const * serialNo, MOT_JogParameters *jogParams)" />
/// <seealso cref="CC_SetJogParamsBlock(char const * serialNo, MOT_JogParameters *jogParams)" />
/// <seealso cref="CC_MoveJog(char const * serialNo, MOT_TravelDirection jogDirection)" />
*/
void JOG_F(int serialNo, int acc, int maxVel) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	KCubeDCServo::CC_SetJogVelParams(SerialNo, acc, maxVel);
	KCubeDCServo::MOT_JogModes mode = KCubeDCServo:: MOT_Continuous;
	KCubeDCServo::MOT_StopModes stopMode = KCubeDCServo::MOT_Immediate;
	KCubeDCServo::CC_SetJogMode(SerialNo, mode,  stopMode);
	KCubeDCServo::MOT_TravelDirection jogDirection = KCubeDCServo::MOT_Forwards;
	KCubeDCServo::CC_MoveJog(SerialNo, jogDirection);
	}
void JOG_R(int serialNo, int acc, int maxVel) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	KCubeDCServo::CC_SetJogVelParams(SerialNo, acc, maxVel);
	KCubeDCServo::MOT_JogModes mode = KCubeDCServo::MOT_Continuous;
	KCubeDCServo::MOT_StopModes stopMode = KCubeDCServo::MOT_Immediate;
	KCubeDCServo::CC_SetJogMode(SerialNo, mode, stopMode);
	KCubeDCServo::MOT_TravelDirection jogDirection = KCubeDCServo::MOT_Backwards;
	KCubeDCServo::CC_MoveJog(SerialNo, jogDirection);

}
void Fstage_STOP(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	KCubeDCServo::CC_StopImmediate(SerialNo);
}


