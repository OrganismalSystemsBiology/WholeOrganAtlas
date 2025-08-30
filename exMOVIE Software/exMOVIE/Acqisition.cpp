#pragma once
#include "stdlib.h"
#include <iostream> 
#include <sstream>
#include "atcore.h"

//#include <tiffio.h>
//#include <tiff.h>


using namespace std;

#define EXTRACTLOWPACKED(SourcePtr) ( (SourcePtr[0] << 4) + (SourcePtr[1] & 0xF) )
#define EXTRACTHIGHPACKED(SourcePtr) ( (SourcePtr[2] << 4) + (SourcePtr[1] >> 4) )
/*
void SaveFrameToDisc(AT_H H, int frameNumber, unsigned char *frameP, int columnCount, int rowCount) {
	stringstream file_name;
	double exposureTime = 0;
	AT_GetFloat(H, L"ExposureTime", &exposureTime);

	file_name << "PixelValues-Image-" << exposureTime << "-" << frameNumber << ".tiff";

	TIFF *hndl_out_file;
	hndl_out_file = TIFFOpen(file_name.str().c_str(), "w");
	if (hndl_out_file == NULL) {
		std::cout << "Could not open output file!" << std::endl;
		return;
	}

	// tif output file settings
	TIFFSetField(hndl_out_file, TIFFTAG_IMAGELENGTH, rowCount);
	TIFFSetField(hndl_out_file, TIFFTAG_IMAGEWIDTH, columnCount);
	TIFFSetField(hndl_out_file, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(hndl_out_file, TIFFTAG_BITSPERSAMPLE, 16);
	TIFFSetField(hndl_out_file, TIFFTAG_PLANARCONFIG, 1);
	TIFFSetField(hndl_out_file, TIFFTAG_PHOTOMETRIC, 1);
	/*
	unsigned char *puc_out_image;
	puc_out_image = (unsigned char *)_TIFFmalloc(columnCount * rowCount*);
	if (puc_out_image == NULL) {
		std::cout << "Could not allocate memory!" << std::endl;
	}

	for (int row = 0; row < rowCount; row++) {
		uint32_t i_err = TIFFWriteScanline(hndl_out_file, &frameP[row * columnCount * 2], row, 0);
		if (i_err != 1) {
			std::cout << "Failed to write to tif file!" << std::endl;
		}
	}

	//_TIFFfree(puc_out_image);
	TIFFClose(hndl_out_file);
}
*/
void Abort_camera() {
	AT_Command(0, L"AcquisitionStop");
	AT_Command(1, L"AcquisitionStop");
}



int Acqisition_test_original(int c_num, float expo_time, int PictureNum, string exportpath)
{
	int i_retCode = AT_SUCCESS;
	AT_64 iNumberDevices = 2;
	AT_H Hndl;
	AT_InitialiseLibrary();

	//checkExistenceOfFolder(exportpath + '\\' + "Image_" + to_string(c_num));

	AT_Open(c_num, &Hndl);


	AT_WC szValue[64];
	i_retCode = AT_GetString(Hndl, L"Serial Number", szValue, 64);
	if (i_retCode == AT_SUCCESS) {
		//The serial number of the camera is szValue
		wcout << L"The serial number is " << szValue << endl << endl;
	}
	//Set the pixel Encoding to the desired settings Mono12Packed Data
	AT_SetEnumeratedString(Hndl, L"Pixel Encoding", L"Mono16");

	//Set the pixel Readout Rate to slowest
	i_retCode = AT_SetEnumeratedString(Hndl, L"Pixel Readout Rate", L"540 MHz");
	if (i_retCode != AT_SUCCESS) {
		cout << "Pixel Readout Rate setting error !!!" << endl;
	}
	//Set the exposure time for this camera to 10 milliseconds
	i_retCode = AT_SetFloat(Hndl, L"Exposure Time", expo_time);
	if (i_retCode != AT_SUCCESS) {
		cout << "Exposure Time setting error !!!" << endl;
	}


	//Set AOI

	AT_SetInt(Hndl, L"AOIHBin", 1);
	AT_SetInt(Hndl, L"AOIWidth", 2048);
	AT_SetInt(Hndl, L"AOILeft", 1);
	AT_SetInt(Hndl, L"AOIVBin", 1);
	AT_SetInt(Hndl, L"AOIHeight", 1024);
	AT_SetInt(Hndl, L"AOITop", 1);



	i_retCode = AT_SetEnumString(Hndl, L"SensorReadoutMode", L"Top Down Sequential");
	if (i_retCode != AT_SUCCESS) {
		cout << "SensorReadoutMode setting error !!!" << endl;
	}
	//"ScanSpeedControlEnable
	i_retCode = AT_SetBool(Hndl, L"ScanSpeedControlEnable", AT_TRUE);
	if (i_retCode != AT_SUCCESS) {
		cout << "ScanSpeedControlEnable setting error !!!" << endl;
	}
	//Configures the time in seconds to read a single row.
	float Readtime = expo_time / 10000;// ms->s  x 1/10
	i_retCode = AT_SetFloat(Hndl, L"RowReadTime ", Readtime);
	if (i_retCode != AT_SUCCESS) {
		cout << "RowReadTime setting error !!!" << endl;
	}
	//Set the scan speed
	float Linespeed = 1024 / expo_time; //  rows/s
	i_retCode = AT_SetFloat(Hndl, L"LineScanSpeed ", Linespeed);
	if (i_retCode != AT_SUCCESS) {
		cout << "Set the scan speed error !!!" << endl;
	}



	//Get the number of bytes required to store one frame
	AT_64 ImageSizeBytes;
	AT_GetInt(Hndl, L"Image Size Bytes", &ImageSizeBytes);
	int BufferSize = static_cast<int>(ImageSizeBytes);


	//AT_SetEnumString(Hndl, L"TriggerMode", L"External Exposure");
	AT_SetEnumString(Hndl, L"TriggerMode", L"Internal");
	//AT_SetEnumString(Hndl, L"TriggerMode", L"SoftWare");
	AT_SetEnumString(Hndl, L"ElectronicShutteringMode", L"Rolling");

	int Mode;
	AT_GetEnumIndex(Hndl, L"TriggerMode", &Mode);
	double Exposuretime;
	AT_GetFloat(Hndl, L"ExposureTime", &Exposuretime);


	//Allocate a number of memory buffers to store frames
	int NumberOfBuffers = 100;
	int NumberOfFrames = PictureNum;


	int iError;
	// Get the number of bytes required to store one frame

	AT_GetInt(Hndl, L"ImageSizeBytes", &ImageSizeBytes);


	unsigned char** AcqBuffers = new unsigned char*[NumberOfBuffers];
	unsigned char** AlignedBuffers = new unsigned char*[NumberOfBuffers];
	for (int i = 0; i < NumberOfBuffers; i++) {
		AcqBuffers[i] = new unsigned char[BufferSize + 8];
		AlignedBuffers[i] = reinterpret_cast<unsigned char*>((reinterpret_cast<unsigned long long>(AcqBuffers[i% NumberOfBuffers]) + 7) & ~7);
	}

	//Pass these buffers to the SDK
	for (int i = 0; i < NumberOfBuffers; i++) {
		AT_QueueBuffer(Hndl, AlignedBuffers[i], BufferSize);
	}
	AT_SetEnumString(Hndl, L"CycleMode", L"Continuous");
	AT_Command(Hndl, L"Acquisition Start");

	Sleep(100);

	cout << "ExposureTime " << Exposuretime << endl;;
	cout << NumberOfFrames << "flames" << endl;
	cout << "ImageSizeBytes = " << ImageSizeBytes << endl;
	cout << "mode " << Mode << endl << endl;


	cout << "Waiting for acquisition ..." << endl << endl;

	unsigned char* pBuf;
	int BufSize;
	//auto th1 = std::thread([&] { Data_exporting(Hndl, NumberOfFrames, NumberOfBuffers, BufferSize, AlignedBuffers); });

	for (int i = 0; i < NumberOfFrames; ++i) {
		AT_WaitBuffer(Hndl, &pBuf, &BufSize, AT_INFINITE); //AT_INFINITE
		//Re-queue the buffers
		AT_QueueBuffer(Hndl, AlignedBuffers[i%NumberOfBuffers], BufferSize);

		//FileExport^ dlge= gcnew FileExport(Data_exporting)->BeginInvoke(c_num,i, exportpath, BufSize, AlignedBuffers[i%NumberOfBuffers]);
	}

	//Stop the acquisition
	cout << "finish" << endl;
	AT_Command(Hndl, L"AcquisitionStop");
	AT_Flush(Hndl);


	FILE* fpw;
	for (int i = 0; i < NumberOfFrames; ++i) {
		fopen_s(&fpw, (exportpath + '\\' + "Image_" + to_string(c_num) + '\\' + to_string(i) + ".tiff").c_str(), "a+");
		for (auto j = 0; AlignedBuffers[j] != '\0'; ++j) {
			fprintf(fpw, "%ld,", AlignedBuffers[j]);
		}
		fprintf(fpw, "\n");
		fclose(fpw);
	}



	for (int i = 0; i < NumberOfBuffers; i++) {
		delete[] AcqBuffers[i];
	}
	delete[] AlignedBuffers;
	delete[] AcqBuffers;

	AT_Close(Hndl);
	AT_FinaliseLibrary();

	AT_Close(Hndl);
	return 0;
}
/*
int Acqisition_Axial(int c_num, float expo_time, int PictureNum, string exportpath, int LR)
{

	int i_retCode = AT_SUCCESS;
	AT_64 iNumberDevices = 2;
	AT_H Hndl;
	AT_InitialiseLibrary();

	i_retCode = AT_Open(c_num, &Hndl);
	if (i_retCode != AT_SUCCESS) {
		cout << "Error condition, could not initialise camera" << endl << endl;
	}
	else {

		AT_WC szValue[64];
		i_retCode = AT_GetString(Hndl, L"Serial Number", szValue, 64);
		if (i_retCode == AT_SUCCESS) {
			//The serial number of the camera is szValue
			wcout << L"The serial number is " << szValue << endl << endl;
		}
		//Set the pixel Encoding to the desired settings Mono12Packed Data
		AT_SetEnumeratedString(Hndl, L"Pixel Encoding", L"Mono16");

		//Set the pixel Readout Rate to slowest
		i_retCode = AT_SetEnumeratedString(Hndl, L"PixelReadoutRate", L"540MHz");
		if (i_retCode == AT_SUCCESS) {
			cout << "Pixel Readout Rate set to 540MHz" << endl << endl;
		}
		else if (i_retCode != AT_SUCCESS) {
			cout << "Pixel Readout Rate setting error" << endl << endl;
		}
		//Set the exposure time for this camera to 10 milliseconds
		//i_retCode = AT_SetFloat(Hndl, L"ExposureTime", expo_time);
		//cout << "exposure time set" << expo_time << endl;
		//if (i_retCode != AT_SUCCESS) {
		//	cout << "exposure time set error" << endl << endl;
		//	}

		i_retCode = AT_SetEnumString(Hndl, L"TriggerMode", L"Internal");
		if (i_retCode != AT_SUCCESS) {
			cout << "TriggerMode setting error !!!" << endl;
		}
		if (LR == 0) {
			i_retCode = AT_SetEnumString(Hndl, L"SensorReadoutMode", L"Top Down Sequential");
			if (i_retCode != AT_SUCCESS) {
				cout << "SensorReadoutMode setting error !!!" << endl;
			}
		}
		else if (LR == 1) {
			i_retCode = AT_SetEnumString(Hndl, L"SensorReadoutMode", L"Bottom Up Sequential");
			if (i_retCode != AT_SUCCESS) {
				cout << "SensorReadoutMode setting error !!!" << endl;
			}
		}


		//"ScanSpeedControlEnable
		i_retCode = AT_SetBool(Hndl, L"ScanSpeedControlEnable", AT_TRUE);
		if (i_retCode != AT_SUCCESS) {
			cout << "ScanSpeedControlEnable setting error !!!" << endl;
		}
		//Configures the time in seconds to read a single row.
		float Readtime = expo_time / 20;
		cout << "RowReadTime; " << Readtime << endl;
		i_retCode = AT_SetFloat(Hndl, L"RowReadTime", Readtime);
		if (i_retCode != AT_SUCCESS) {
			cout << "RowReadTime setting error !!!" << endl;
		}
		//Configures the number of rows read per second.
		float Linespeed = 2048 / expo_time; // 
		i_retCode = AT_SetFloat(Hndl, L"LineScanSpeed", Linespeed);
		if (i_retCode != AT_SUCCESS) {
			cout << "Set the scan speed error !!!" << endl;
		}
		//Get the number of bytes required to store one frame
		AT_64 ImageSizeBytes;
		AT_GetInt(Hndl, L"Image Size Bytes", &ImageSizeBytes);
		int BufferSize = static_cast<int>(ImageSizeBytes);
		cout << "ImageSizeBytes = " << ImageSizeBytes << endl;

		//Allocate a number of memory buffers to store frames
		int NumberOfBuffers = PictureNum;
		int NumberOfFrames = PictureNum;


		unsigned char** AcqBuffers = new unsigned char*[NumberOfBuffers];
		unsigned char** AlignedBuffers = new unsigned char*[NumberOfBuffers];
		for (int i = 0; i < NumberOfBuffers; i++) {
			AcqBuffers[i] = new unsigned char[BufferSize + 7];
			AlignedBuffers[i] = reinterpret_cast<unsigned char*>((reinterpret_cast<unsigned long long>(AcqBuffers[i% NumberOfBuffers]) + 7) & ~7);
		}

		//Pass these buffers to the SDK
		for (int i = 0; i < NumberOfBuffers; i++) {
			AT_QueueBuffer(Hndl, AlignedBuffers[i], BufferSize);
		}

		AT_SetEnumString(Hndl, L"CycleMode", L"Continuous");

		cout << "Waiting for acquisition ..." << endl << endl;

		//AT_SetEnumString(Hndl, L"TriggerMode", L"External Exposure");
		cout << "************************************" << endl;
		cout << "Camera setting" << endl;
		cout << "************************************" << endl;

		int Mode;
		AT_GetEnumIndex(Hndl, L"TriggerMode", &Mode);
		cout << "TriggerMode  " << Mode << endl;
		double Exposuretime;
		AT_GetFloat(Hndl, L"ExposureTime", &Exposuretime);
		cout << "ExposureTime  " << Exposuretime << endl;;
		double RowReadTime;
		AT_GetFloat(Hndl, L"RowReadTime", &RowReadTime);
		cout << "RowReadTime  " << RowReadTime << endl;;




		//Start the Acquisition running
		AT_Command(Hndl, L"Acquisition Start");
		Sleep(100);
		wcout << L"The serial number is " << szValue << endl << endl;

		unsigned char* pBuf;
		int BufSize;

		cout << "Waiting for acquisition ..." << endl << endl;
		for (int i = 0; i < NumberOfFrames; ++i) {
			if (AT_WaitBuffer(Hndl, &pBuf, &BufSize, AT_INFINITE) == AT_SUCCESS) {
				//Application specific data processing goes here..
				//Re-queue the buffers			
				AT_QueueBuffer(Hndl, AlignedBuffers[i], BufferSize);
			}
		}
		//Stop the acquisition
		cout << "finish" << endl;
		AT_Command(Hndl, L"AcquisitionStop");
		AT_Flush(Hndl);


		FILE *fpw;
		cout << "export" << endl;

		for (int i = 0; i < NumberOfFrames; i++) {
			//fpw = fopen((exportpath + "\\" + to_string(i) + ".tiff").c_str(), "wb");
			fpw = fopen((exportpath + "\\" + to_string(i) + ".tiff").c_str(), "wt");
			fwrite(&AlignedBuffers[i], sizeof(AlignedBuffers[i][0]), BufferSize, fpw);
			fclose(fpw);
		}



		for (int i = 0; i < NumberOfBuffers; i++) {
			delete[] AcqBuffers[i];
		}
		delete[] AlignedBuffers;
		delete[] AcqBuffers;
		AT_Close(Hndl);

	}
	AT_Close(Hndl);
	AT_FinaliseLibrary();
	return 0;
}

int Acqisition_column(int c_num, float expo_time, int PictureNum, string exportpath)
{

	int i_retCode = AT_SUCCESS;
	AT_64 iNumberDevices = 2;
	AT_H Hndl;
	AT_InitialiseLibrary();

	i_retCode = AT_Open(c_num, &Hndl);
	if (i_retCode != AT_SUCCESS) {
		cout << "Error condition, could not initialise camera" << endl << endl;
	}
	else {

		AT_WC szValue[64];
		i_retCode = AT_GetString(Hndl, L"Serial Number", szValue, 64);
		if (i_retCode == AT_SUCCESS) {
			//The serial number of the camera is szValue
			wcout << L"The serial number is " << szValue << endl << endl;
		}
		//Set the pixel Encoding to the desired settings Mono12Packed Data
		AT_SetEnumeratedString(Hndl, L"Pixel Encoding", L"Mono16");

		//Set the pixel Readout Rate to slowest
		i_retCode = AT_SetEnumeratedString(Hndl, L"PixelReadoutRate", L"540MHz");
		if (i_retCode == AT_SUCCESS) {
			cout << "Pixel Readout Rate set to 540MHz" << endl << endl;
		}
		else if (i_retCode != AT_SUCCESS) {
			cout << "Pixel Readout Rate setting error" << endl << endl;
		}
		//Set the exposure time for this camera to 10 milliseconds
		i_retCode = AT_SetFloat(Hndl, L"ExposureTime", expo_time);
		if (i_retCode != AT_SUCCESS) {
			cout << "exposure time set error" << endl << endl;
		}

		i_retCode = AT_SetEnumString(Hndl, L"TriggerMode", L"External Exposure");
		if (i_retCode != AT_SUCCESS) {
			cout << "TriggerMode setting error !!!" << endl;
		}

		i_retCode = AT_SetEnumString(Hndl, L"SensorReadoutMode", L"Centre Out Simultaneous");
		if (i_retCode != AT_SUCCESS) {
			cout << "SensorReadoutMode setting error !!!" << endl;
		}


		//Get the number of bytes required to store one frame
		AT_64 ImageSizeBytes;
		AT_GetInt(Hndl, L"Image Size Bytes", &ImageSizeBytes);
		int BufferSize = static_cast<int>(ImageSizeBytes);
		cout << "ImageSizeBytes = " << ImageSizeBytes << endl;

		//Allocate a number of memory buffers to store frames
		int NumberOfBuffers = PictureNum;
		int NumberOfFrames = PictureNum;


		unsigned char** AcqBuffers = new unsigned char*[NumberOfBuffers];
		unsigned char** AlignedBuffers = new unsigned char*[NumberOfBuffers];
		for (int i = 0; i < NumberOfBuffers; i++) {
			AcqBuffers[i] = new unsigned char[BufferSize + 7];
			AlignedBuffers[i] = reinterpret_cast<unsigned char*>((reinterpret_cast<unsigned long long>(AcqBuffers[i% NumberOfBuffers]) + 7) & ~7);
		}

		//Pass these buffers to the SDK
		for (int i = 0; i < NumberOfBuffers; i++) {
			AT_QueueBuffer(Hndl, AlignedBuffers[i], BufferSize);
		}

		AT_SetEnumString(Hndl, L"CycleMode", L"Continuous");

		cout << "Waiting for acquisition ..." << endl << endl;

		//AT_SetEnumString(Hndl, L"TriggerMode", L"External Exposure");
		cout << "************************************" << endl;
		cout << "Camera setting" << endl;
		cout << "************************************" << endl;

		int Mode;
		AT_GetEnumIndex(Hndl, L"TriggerMode", &Mode);
		cout << "TriggerMode  " << Mode << endl;
		double Exposuretime;
		AT_GetFloat(Hndl, L"ExposureTime", &Exposuretime);
		cout << "ExposureTime  " << Exposuretime << endl;;
		double RowReadTime;


		//Start the Acquisition running
		AT_Command(Hndl, L"Acquisition Start");
		Sleep(100);
		wcout << L"The serial number is " << szValue << endl << endl;

		unsigned char* pBuf;
		int BufSize;

		cout << "Waiting for acquisition ..." << endl << endl;
		for (int i = 0; i < NumberOfFrames; ++i) {
			if (AT_WaitBuffer(Hndl, &pBuf, &BufSize, AT_INFINITE) == AT_SUCCESS) {
				//Application specific data processing goes here..
				//Re-queue the buffers			
				AT_QueueBuffer(Hndl, AlignedBuffers[i], BufferSize);

			}
		}
		//Stop the acquisition
		cout << "finish" << endl;
		AT_Command(Hndl, L"AcquisitionStop");
		AT_Flush(Hndl);


		FILE *fpw;
		cout << "export" << endl;

		for (int i = 0; i < NumberOfFrames; i++) {
			fpw = fopen((exportpath + "\\" + to_string(i) + ".tiff").c_str(), "wt");
			fwrite(&AlignedBuffers[i], sizeof(AlignedBuffers[i][0]), BufferSize, fpw);
			fclose(fpw);

		}



		for (int i = 0; i < NumberOfBuffers; i++) {
			delete[] AcqBuffers[i];
		}
		delete[] AlignedBuffers;
		delete[] AcqBuffers;
		AT_Close(Hndl);

	}
	AT_Close(Hndl);
	AT_FinaliseLibrary();
	return 0;
}
*/