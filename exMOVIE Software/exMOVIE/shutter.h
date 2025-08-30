#pragma once
// Example_TSC001.cpp : Defines the entry point for the console application.
//
#include "Thorlabs.MotionControl.TCube.Solenoid.h"

void Shutter_conneection(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);

	if (TCubeSolenoid::TLI_BuildDeviceList() == 0)
	{
		// get device list size 
		short n = TCubeSolenoid::TLI_GetDeviceListSize();
		// get BBD serial numbers
		char serialNos[100];
		TCubeSolenoid::TLI_GetDeviceListByTypeExt(serialNos, 100, 85);

		// output list of matching devices
		{
			char *searchContext = nullptr;
			char *p = strtok_s(serialNos, ",", &searchContext);
			while (p != nullptr)
			{
				TCubeSolenoid::TLI_DeviceInfo deviceInfo;
				// get device info from device
				TCubeSolenoid::TLI_GetDeviceInfo(p, &deviceInfo);
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
			if (TCubeSolenoid::SC_Open(SerialNo) == 0)
			{
				// start the device polling at 200ms intervals
				TCubeSolenoid::SC_StartPolling(SerialNo, 200);

				Sleep(1000);

				// set operating mode to SC_OperatingModes::SC_Manual
				//TCubeSolenoid::SC_SetOperatingMode(SerialNo, TCubeSolenoid::SC_OperatingModes::SC_Manual);
				//Sleep(1000);
			
				// set operating state to SC_OperatingStates::SC_Inactive
				TCubeSolenoid::SC_SetOperatingState(SerialNo, TCubeSolenoid::SC_OperatingStates::SC_Inactive);
				Sleep(200);
				int state1 = TCubeSolenoid::SC_GetOperatingState(SerialNo);
				int state2 = TCubeSolenoid::SC_GetSolenoidState(SerialNo);
				printf("Device %s state is %d, solenoid is %d\r\n", SerialNo, state1, state2);
			}

		}
	}
}
void Shutter_Open_func(int serialNo, int &state) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	TCubeSolenoid::SC_SetOperatingState(SerialNo, TCubeSolenoid::SC_OperatingStates::SC_Active);
	Sleep(200);
	int state1 = TCubeSolenoid::SC_GetOperatingState(SerialNo);
	state = TCubeSolenoid::SC_GetSolenoidState(SerialNo);
	printf("Device %s state is %d, solenoid is %d\r\n", SerialNo, state1, state);


}
void Shutter_Close_func(int serialNo, int &state) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	TCubeSolenoid::SC_SetOperatingState(SerialNo, TCubeSolenoid::SC_OperatingStates::SC_Inactive);
	Sleep(200);
	int state1 = TCubeSolenoid::SC_GetOperatingState(SerialNo);
	state = TCubeSolenoid::SC_GetSolenoidState(SerialNo);
	printf("Device %s state is %d, solenoid is %d\r\n", SerialNo, state1, state);

}
void Shutter_disconnection(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
	// stop polling
	TCubeSolenoid::SC_StopPolling(SerialNo);
	// close device
	TCubeSolenoid::SC_Close(SerialNo);
}
int Shutter_Pos(int serialNo) {
	char SerialNo[16];
	sprintf_s(SerialNo, "%d", serialNo);
//	int state1 = TCubeSolenoid::SC_GetOperatingState(SerialNo);
	int state2 = TCubeSolenoid::SC_GetSolenoidState(SerialNo);
	printf("Device %s state is %d\r\n", SerialNo,  state2);
	return state2;
}


