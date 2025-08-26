#pragma once
// Example_MFF101.cpp : Defines the entry point for the console application.
//
#include "Thorlabs.MotionControl.FilterFlipper.h"

void Flipper_Open(int serialNo, int &position, int targetpos)
{
	// identify and access device
	char testSerialNo[16];
	sprintf_s(testSerialNo, "%d", serialNo);
	// Build list of connected device
	if (FilterFlipper::TLI_BuildDeviceList() == 0)
	{
		// get device list size 
		short n = FilterFlipper::TLI_GetDeviceListSize();
		// get MFF serial numbers
		char serialNos[100];
		FilterFlipper::TLI_GetDeviceListByTypeExt(serialNos, 100, 37);
		// output list of matching devices
		{
			char *searchContext = nullptr;
			char *p = strtok_s(serialNos, ",", &searchContext);

			while (p != nullptr)
			{
				FilterFlipper::TLI_DeviceInfo deviceInfo;
				// get device info from device
				FilterFlipper::TLI_GetDeviceInfo(p, &deviceInfo);
				// get strings from device info structure
				char desc[65];
				strncpy_s(desc, deviceInfo.description, 64);
				desc[64] = '\0';
				char SerialNo[9];
				strncpy_s(SerialNo, deviceInfo.serialNo, 8);
				SerialNo[8] = '\0';

				// output
				printf("Found Device %s, Serial :%s \r\n", p, desc,SerialNo);
				p = strtok_s(nullptr, ",", &searchContext);
			}
		}

		// open device
		if (FilterFlipper::FF_Open(testSerialNo) == 0)
		{
			FilterFlipper::FF_StartPolling(testSerialNo, 200);
			Sleep(300);
			position = FilterFlipper::FF_GetPosition(testSerialNo);
			if (position != targetpos) {
				FilterFlipper::FF_MoveToPosition(testSerialNo, position == 1 ? FilterFlipper::FF_Positions::Position2 : FilterFlipper::FF_Positions::Position1);

			}
			position = targetpos;
		}
	}
}
void Flipper_Change(int serialNo, int &position, int targetpos) {
	position = 0;
	char testSerialNo[16];
	sprintf_s(testSerialNo, "%d", serialNo);
	////if (TLI_BuildDeviceList() == 0)
	//{
		//if (FF_Open(testSerialNo) == 0)
	//	{
	//		FF_StartPolling(testSerialNo, 200);
	//		Sleep(500);
	position = FilterFlipper::FF_GetPosition(testSerialNo);
	if (position != targetpos) {
		//	while((position != targetpos)){

		FilterFlipper::FF_MoveToPosition(testSerialNo, position == 1 ? FilterFlipper::FF_Positions::Position2 : FilterFlipper::FF_Positions::Position1);
		position = targetpos;
		//		}
			//Sleep(500);
			//	position = FF_GetPosition(testSerialNo);
			//Sleep(500);
	}
	//		FF_StopPolling(testSerialNo);
	//		FF_Close(testSerialNo);
	//	}
	//}
}
void Flipper_Close(int serialNo) {
	char testSerialNo[16];
	sprintf_s(testSerialNo, "%d", serialNo);

	FilterFlipper::FF_StopPolling(testSerialNo);
	FilterFlipper::FF_Close(testSerialNo);
}
int Flipper_GetPos(int serialNo) {
	int position = 999;
	char testSerialNo[16];
	sprintf_s(testSerialNo, "%d", serialNo);

	position = FilterFlipper::FF_GetPosition(testSerialNo);
	return position;
}


