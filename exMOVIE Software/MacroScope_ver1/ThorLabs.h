#pragma once

//#include "stdafx.h"

#include <stdlib.h>
#include <conio.h>
#include "Flipper.h"
#include "KDC101.h"
#include "shutter.h"

struct Thorlabs_Dev {
	int FilterFlipper_Serial = 37874011;
	int Fstage_Serial = 27000155;
	int Shutter_Serial = 85860885;
};


void Open_ThorLabs(Thorlabs_Dev &serialNo, int &x10Filterpos, int &Fpos, int &Shutter_state )
{
	if (FilterFlipper::TLI_BuildDeviceList() == 0)
	{
		// get device list size 
		short n = FilterFlipper::TLI_GetDeviceListSize();
		cout << "Get: " << n << " ThorLabs Devices" << endl;
		// get MFF serial numbers
		Flipper_Open(serialNo.FilterFlipper_Serial, x10Filterpos, 1);
		Shutter_conneection(serialNo.Shutter_Serial);
		Fpos = Focus_stage_connection(serialNo.Fstage_Serial);
		Shutter_state = Shutter_Pos(serialNo.Shutter_Serial);
		cout << "Focus Stage Pos: " << Fpos << endl;
	}
}
void Close_ThorLabs(Thorlabs_Dev &serialNo) {
	Flipper_Close(serialNo.FilterFlipper_Serial);
	Shutter_disconnection(serialNo.Shutter_Serial);
	Close_Focus_stage(serialNo.Fstage_Serial);

}

