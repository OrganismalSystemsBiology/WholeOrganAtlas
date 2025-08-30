#pragma once
#include "atcore.h"
#include <iostream> 
#include <string>

using namespace std;

std::string wide_to_multi_capi(std::wstring const& src)
{
	std::size_t converted{};
	std::vector<char> dest(src.size() * sizeof(wchar_t) + 1, '\0');
	if (::_wcstombs_s_l(&converted, dest.data(), dest.size(), src.data(), _TRUNCATE, ::_create_locale(LC_ALL, "jpn")) != 0) {
		throw std::system_error{ errno, std::system_category() };
	}
	dest.resize(std::char_traits<char>::length(dest.data()));
	dest.shrink_to_fit();
	return std::string(dest.begin(), dest.end());
}

string serialnumber1()
{
	string serial;
	int i_retCode;
	cout << "Initialising Camera_1..." << endl << endl;
	i_retCode = AT_InitialiseLibrary();
	if (i_retCode != AT_SUCCESS) {
		cout << "Error initialising library" << endl << endl;
	}
	else {
		AT_64 iNumberDevices = 0;
		AT_GetInt(AT_HANDLE_SYSTEM, L"Device Count", &iNumberDevices);
		if (iNumberDevices <= 0) {
			cout << "No cameras detected" << endl;
		}
		else {
			AT_H Hndl;
			i_retCode = AT_Open(0, &Hndl);
			if (i_retCode != AT_SUCCESS) {
				cout << "Error condition, could not initialise camera" << endl << endl;
			}
			else {
				cout << "Successfully initialised camera" << endl << endl;
			}
			AT_WC szValue[64];
			i_retCode = AT_GetString(Hndl, L"Serial Number", szValue, 64);
			if (i_retCode == AT_SUCCESS) {
				//The serial number of the camera is szValue
				wcout << L"The serial number is " << szValue << endl << endl;
				for (int i = 0; i <10; ++i) {
					serial[i] = szValue[i];
				}

			
				double temperature = 0;

				AT_SetBool(Hndl, L"SensorCooling", AT_TRUE);
				int temperatureCount = 0;
				AT_GetEnumCount(Hndl, L"TemperatureControl", &temperatureCount);
				AT_SetEnumIndex(Hndl, L"TemperatureControl", temperatureCount - 1);
				int temperatureStatusIndex = 0;
				AT_WC temperatureStatus[256];
				AT_GetEnumIndex(Hndl, L"TemperatureStatus", &temperatureStatusIndex);
				AT_GetEnumStringByIndex(Hndl, L"TemperatureStatus", temperatureStatusIndex, temperatureStatus, 256);
				return serial;
			}
			else {
				cout << "Error obtaining Serial number" << endl << endl;
			}
			AT_Close(Hndl);
		}
		AT_FinaliseLibrary();
	}

}
string serialnumber2()
{
	string serial;
	int i_retCode;
	cout << "Initialising Camera_2..." << endl << endl;
	i_retCode = AT_InitialiseLibrary();
	if (i_retCode != AT_SUCCESS) {
		cout << "Error initialising library" << endl << endl;
	}
	else {
		AT_64 iNumberDevices = 1;
		AT_GetInt(AT_HANDLE_SYSTEM, L"Device Count", &iNumberDevices);
		if (iNumberDevices <= 0) {
			cout << "No cameras detected" << endl;
		}
		else {
			AT_H Hndl;
			i_retCode = AT_Open(1, &Hndl);
			if (i_retCode != AT_SUCCESS) {
				cout << "Error condition, could not initialise camera" << endl << endl;
			}
			else {
				cout << "Successfully initialised camera" << endl << endl;
			}
			AT_WC szValue[64];
			i_retCode = AT_GetString(Hndl, L"Serial Number", szValue, 64);
			if (i_retCode == AT_SUCCESS) {
				//The serial number of the camera is szValue
				wcout << L"The serial number is " << szValue << endl << endl;
				for (int i = 0; i <10; ++i) {
					serial[i] = szValue[i];
				}

				/*
				double temperature = 0;

				AT_SetBool(Hndl, L"SensorCooling", AT_TRUE);
				int temperatureCount = 0;
				AT_GetEnumCount(Hndl, L"TemperatureControl", &temperatureCount);
				AT_SetEnumIndex(Hndl, L"TemperatureControl", temperatureCount - 1);
				int temperatureStatusIndex = 0;
				AT_WC temperatureStatus[256];
				AT_GetEnumIndex(Hndl, L"TemperatureStatus", &temperatureStatusIndex);
				AT_GetEnumStringByIndex(Hndl, L"TemperatureStatus", temperatureStatusIndex, temperatureStatus, 256);
				
				//Wait for temperature to stabalise
				while (wcscmp(L"Stabilised", temperatureStatus) != 0) {
				AT_GetEnumIndex(Hndl, L"TemperatureStatus", &temperatureStatusIndex);
				AT_GetEnumStringByIndex(Hndl, L"TemperatureStatus", temperatureStatusIndex,
				temperatureStatus, 256);
				}
				

				*/
				return serial;

			}
			else {
				cout << "Error obtaining Serial number" << endl << endl;
			}
			AT_Close(Hndl);
		}
		AT_FinaliseLibrary();
	}

	cout << endl << "Press any key then enter to close" << endl;
	char ch;
}




