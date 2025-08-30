#pragma once

#include "CheckWindow.h"
#include <iostream> 
#include <math.h>
#include <fstream>
#include <string>
#include <ctime>
#include <iostream>
#include <locale.h >
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <algorithm>
#include <conio.h>
#include "ImageAnalysis.h"
#include <cctype>
#include <chrono>
#include <numeric>
#include <cmath>
#include <vector>
#include <iomanip>
#include <windows.h>
#include <process.h>
#include <atlstr.h>
#include <time.h>
#include <atcore.h>
#include <direct.h>
#include <memory>
#include <CheckTheFolder.h>
#include <get_file_path_in_dir.h>
#include "NIDAQmx.h"
#include "msclr/marshal.h"
#include "msclr/marshal_windows.h"
#include "msclr/marshal_cppstd.h"
#include "msclr/marshal_atl.h"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <gl/gl.h> 
#include <gl/glu.h> 
#include <gl/glut.h>
#include <shlobj.h>
#include <tchar.h>
#include <fftw3.h>
#include "C:\libtiff\libtiff\tiff.h"
#include "C:\libtiff\libtiff\tiffio.h"
#include <stdexcept>
#include <cassert>
#include <filesystem>
#include "ThorLabs.h"
#include <fftw3.h>
#include <winnt.h>
#include <marshal_cppstd.h>

#include <../../inc/dcamapi4.h>
#include <../../inc/dcamprop.h>
#include <misc\console4.h>
#include <misc\common.cpp>


#using<system.dll>
#define pi 3.14159

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
#define DAQmxErrChk2(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error2; else
#define SHEET_AF_SPAN 5  //MOVIE^focus update threshold
#define INITIAL_AF_SPAN 8.0
#define INITIAL_AF_NUM 21
#define INITIAL_AXIAL_AF_SPAN 0.1
#define PIXELHIGHT 2304
#define PIXELWIDTH 2304

#define TIMEOUT_TIME 100000;
#define BUF (2304*2304+1)
#define IMG (2304*2304+1)
#define BIT unsigned short
#define ZOOMRATE 1.3
#define MASKSIZE 128
#define MASK (128*128+1)
#define ROISIZE 250

#define CAMERA_DAQ "cDAQ1Mod1/ao1\0"
#define ETL_DAQ "cDAQ1Mod1/ao0\0"
#define SHEET_GALV_R "cDAQ1Mod1/ao2\0"
#define SHEET_GALV_L "cDAQ1Mod1/ao3\0"
#define DAQ1_ALL "cDAQ1Mod1/ao0,cDAQ1Mod1/ao1,cDAQ1Mod1/ao2,cDAQ1Mod1/ao3\0"
#define DAQ2_ALL "cDAQ2Mod1/ao0,cDAQ2Mod1/ao1\0"
#define Z_GALV_DAQ "cDAQ2Mod1/ao0,cDAQ1Mod1/ao0\0"
#define Z_GALV_DAQ_L "cDAQ2Mod1/ao0\0"
#define Z_GALV_DAQ_R "cDAQ2Mod1/ao1\0"

#define PIXELSIZE 6.5
#define DECONV_SIZE 50
#define LP488MAX 200
#define LP488MIN 20
#define LP592MAX 500
#define LP592MAX 500
#define LP592MIN 50
#define LP647MAX 140
#define LP647MIN 15
#define B_EXTRA 50
#define MAG 10
#define Theta_deg 0.002;
#define Overlap_xy 74

#define CAMERABUFNUM 200
//#define WINDOWSIZE 20.0

HANDLE Daq1Mutex = CreateMutex(NULL, FALSE, NULL);
HANDLE Daq2Mutex = CreateMutex(NULL, FALSE, NULL);
HANDLE hMutex = CreateMutex(NULL, FALSE, NULL);
HANDLE hMutex_Thread = CreateMutex(NULL, FALSE, NULL);
HANDLE mtx_camera = CreateMutex(NULL, FALSE, NULL);
int wait = 0;
CRITICAL_SECTION cs; /* main() で呼び出される InitializeCriticalSection */
HANDLE mtx_MOVIEFocus;  /* main() で呼び出される CreateMutex */
int multicolorcount = 0;
std::vector<std::string>CoboltCOMlist;

struct Orca_Param {
	int bufflag = 0;
	HDCAM hdcam;
	HDCAMWAIT hwait;
	int32 number_of_buffer = CAMERABUFNUM;
	int32 Pixeltype = 0;
	int32 width = 2304;
	int32 height = 2304;
	int32 rowbyte = 2;
	int32 bufframebytes = 2304 * 2304 * 2;
	double pixelwidth = 6.5; //um
	double pixelheight = 6.5; //um
	char* buf;
	void** bufAddr;
	char* MergeImg;
	void** MergeImgAddr;
	int buf_flag = 0;
	int cam;
	int bufpos = 0; //raw image pos on bufAddr
	int MergePos = -1;//Merged image pos on MergeImgAddr
	double temp = 0;
	double scanspeed = 0;
} Orca;

int Close_Flag = 0;
int filterpos = 0;
BIT BestImage[2304 * 2304];
double EXTRA_TIME = 0;
chrono::system_clock::time_point  startG, endG, startm, endm;
BIT ImageArray[16 * 16];

double MicroStepZ = 5;
double MicroStepS = 500;
bool acqdata_flag = false;

struct ThreadsCondition {
	bool CameraSetting = false;
	bool Camera = false;
	bool Acquisition = false;
	bool DataExport = false;
	bool Laser = false;
	bool AF = false;
	bool MOVIE_focus = false;
	bool MOVIE_skip = false;
	bool MOVIE_skip_calc = false;
	bool MOVIE_scan = false;
	bool DaqPrep = false;
	bool Daq = false;
	bool MOVIE_Thread = false;

}threads;

int skipcalc_flag = 0;

struct MOVIEFocusImageSet {
	cv::Mat Image1;
	cv::Mat Image2;
}MOVIEFocusImages;

struct mouseParam {
	int x;
	int y;
	int event;
	int flags;
};
mouseParam mparam;

struct skipinfo {
	int z;
	int zi;
	int Samplearea = 0;
	bool sample;
	bool AF;
};
vector<vector<skipinfo>>MOVIESkipdata;
struct AxialSet {
	float ETL_Start = 0;
	float ETL_End = 0;
};

struct Axial_ETL_Param {
	double Laser488[4];//Left-left, Left-right, Right-left,Right-right
	double Laser592[4];
	double Laser647[4];
}Axial_ETL_param;
struct Stagepos {
	int x = 0;
	double y = 0;
	int z = 0;
	double s = 0;
	bool move_x = false;
	bool move_y = false;
	bool move_z = false;
	bool move_s = false;
};
Stagepos Stagepos_target;
Stagepos Stagepos_current;
struct StageStat {
	int x;
	int y;
	int z;
	int s;
}stage_stat;

struct PreScan_param {
	int imagesize = 0;
	int background = 0;
};
struct AF_param {
	double Zpos = 0;
	double Focus_score = 0;
	double Axial_start = 0;
	double Axial_end = 0;
};
vector<AF_param> AF_stats(90);
struct AF_axial_param {
	bool take = false;
	double ETLpos = 0;
	double Focus_score_L = 0;
	double Focus_score_R = 0;
};
vector<AF_axial_param> AF_Axial_stats(100);

struct FW_RV_PosList {
	int FW_xpos;
	int FW_ypos;
	int RV_xpos;
	int RV_ypos;
	char FW_LR = 'L';
	char RV_LR = 'R';
	string dir_FW;
	string dir_RV;
};
vector<FW_RV_PosList> DirList;
struct MOVIE_param {
	int dataspeed = 0;
	int stage = 0;
	int camera = 0;
	int data = 0;
	int DAQ = 0;
	int StartTile = 0;
	int SingleExposure = 0;
	int StageSpeed = 10;
	int External = 0;
	int LF_Set_count = 0;
	int tilenum = 0;
	int VB_ready = 0;
	int restart_flag = 0;
	int cancel_flag = 0;
	bool skip = false;//true ->skip false -> take;
	bool Skip_Thread = false;
	int AFPOS_F = 0;
	int AFPOS_R = 0;
	int FR = 0;//0:F, 1:R
	bool AF_sample = false;
	bool retry = false;
	int retrycount = 0;
	int MOVIE_Focus = 0;
	int MOVIE_Focus_pos = 0;
	int Abort = 0;
	int MOVIE_skip_thread = 0;
	int ETL_reset_flag = 0;
	int ImagingStartNum = 0;
	int ImagingEndNum = 0;
	double InitialFocus_L = 0;
	double InitialFocus_R = 0;
	int ExpStartPos = 0;
	int ExpEndPos = 0;
	int SkipStartPos = 0;
	int SkipEndPos = 0;
	AxialSet Axial_param;
	AxialSet Axial_param_v;
	int MOVIE_Retry_int = 200;
	double Step = 2.5;
	int TargetImageNum = 1;
	int TargetImageNum_FR = 1;
	int TargetImageNum_RV = 1;
	int passed_image_num1 = 0;
	int passed_image_num2 = 0;
	double *MOVIE_focus_list;
	double *Stage_pos_list;
	double *MOVIE_skip_list;
	int ExposureFlag = 0;
	int TTLs = 0;
	int MaxImageNum = 1;
	bool Skiptile = false;

}MOVIE_stats;

struct Laser {
	float LP = 0;
	float LP_current = 0;
	int LPmax = 200;
	int LPmin = 20;
	bool stat = false;
	bool stat_Q = false;
	int Q = 0; // 1:Laser on-off,2:LP,8:temp
	string Serial;
	string COM;
	bool COMOpen = false;
	int res = 0;
	double temp = 25;
};
struct LaserInfo {
	Laser L488;
	Laser L592;
	Laser L647;
} Laser_Info;

std::vector<int>MOVIEFocuslist(3000, 0);

struct BlockInfo {
	int n = 0;
	int m = 0;
	double L2 = 0;
	bool blankcheck = false;
	double max1 = 0;
	double min1 = 0;
	double max2 = 0;
	double min2 = 0;
	//bool operator<(const  BlockInfo &right) constDCTcompList
	//{
	//	return L2 < right.L2;
	//};

};

bool asc(const BlockInfo& left, const BlockInfo& right) {
	return left.L2 > right.L2;
}
bool des(const BlockInfo& left, const BlockInfo& right) {
	return left.L2 < right.L2;
}
vector<BlockInfo> L2list(256);
int DCTBlockCalc_flag = 0;
long DCTBlockCalc_num = 0;
vector<double> DCTBlockCalc_scores(24, 0);



struct Camera_handl {
	AT_H Hndl;
	string Serial;
	int Cameranum;
};
Camera_handl Camera1;
Camera_handl Camera2;


struct ThreadStats {
	int FPS = 0;
	int Camera = 0;
	int DataExport = 0;
	int DAQ_VB = 0;
};
struct MOVIE_Focus_stats {
	int imagenum = 0;
	int Sheetpos = 0;
};
struct Laser_LR_Vol_set {
	double L = 1;
	double R = -1;
}Laser_LR_Vol;

int DataChangeFlag = 0;
int PreScanImageNum = 1;
double deltaDCTVal[10000];
int MOVIE_focus_feedback_flag = 0;
double FWHM = 4;
int XY_stage_flag = 0;
int FocusVal = 0;
double DCTcompList1[24] = {};
double DCTcompList2[24] = {};
int Shutter_state = 0;

AxialSet AxialParam = {};
Thorlabs_Dev TSerials;
double deltatime_G = 0;

int MOVIE_skip_size = 0;
int x10_dualcolor_num = 0;
const char *deviceName = "VB8012-312E277";
const char *channel_camera = "VB8012-312E277/dig/0:1";
const char *channel_ETL_LR = "VB8012-312E277/dig/7";
const char *channel_ETL_right = "ps/+25V";
const char *channel_ETL_left = "ps/-25V";
const char *channel_ETL_PS = "ps/+6V";
const double voltageLevel = 5.0;
const double currentLimit_min = 0.005;

const bool high[2] = { true, true };
const bool low[2] = { false,false };
const size_t numChanne_Camera = 2;
struct MOVIEFocus {
	double FocusVal = 0;
	int recentimage = 0;
}MOVIEFocusVal;
//double MOVIEFocusVal = 0;
double Zdata[4] = { 0 };
int x10Filterpos = 0;
int Focus_Pos = 0;
int Save_PreScan = 0;
int PreScan_WINDOW = 0;
int PreScan_frag = 0;
int mouseevent_flag = 0;
int MOVIE_Thread_flag = 0;
int VB_flag = 0;
double FrameRate = 0;
int Axial_flag = 0;
int Axial_ETL_flag = 0;
int Axial_DAQ_flag = 0;
int Axial_GO = 0;
int g_LaserIndex = 0;
char illumination_flag = 'L';
double *AF_data;
float MAGx = 0.63;
int Sheet_mode = 0;
int Neo_flag = 0;
int ROI_center = 1024;
int ROI_width = 350;
int ROI_width_ori = 350;
int AS_flag = 0;
int Abort_flag = 0;
int AF_task_flag = 0;
int AF_R_flag = 0;
int AF_L_flag = 0;
int AF_exp_flag = 0;
int clicktime;
char SetNum_now;
int grobal_temp;
int Laser_Set_Power;
int Focus_Limit;
int Focus_Limit2;
int Focus_Flag = 0;
int Focus_BWFlag = 0;
int ROI_left = 0;
int ROI_right = 2304;
TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };
TaskHandle  taskHandle = 0;
TaskHandle  taskHandle2 = 0;
int multiLaser_flag = 1;
int L488_res = 0;
int L592_res = 0;
int L647_res = 0;
int LF_set_count;
string deffolder = "Z:\\";
int NumberOfBuffers = 500;
int LF_setnum = 0;
int total_image_num = 0;
int passed_image_num = 0;
int multi_mode = 0;
int tile_mode = 0;
int LIVE_window1 = 0;
int LIVE_window2 = 0;
int LIVE_window3 = 0;
char Laser_LR_now;
char Laser_LR_t;
int KOUZU_flag_m = 0;
int SIGMA_flag_m = 0;
int camera1_on_flag = 0;
int camera2_on_flag = 0;
int camera3_on_flag = 0;
int Timeout_time = TIMEOUT_TIME;
int g_Laser_Power = 0;
int Acq_flag = 0;
int Picnumber_c1 = 0;
int Picnumber_c2 = 0;
int Picnumber_c3 = 0;
int Picnumber_c1_w = 0;
int Picnumber_c2_w = 0;
int Picnumber_c3_w = 0;
int Camera_flag1 = 0;
int Camera_flag2 = 0;
int Camera_flag3 = 0;
int Export_flag1 = 0;
int Export_flag2 = 0;
int Export_flag3 = 0;
int Tile_number = 0;
int filter_num = 0;
int filter_abort_flag = 0;
int filter_flag = 0;
int Laser_FIlter_flag1 = 0;
int Laser_FIlter_flag2 = 0;
int Laser_FIlter_flag3 = 0;
int Laser_FIlter_flag4 = 0;
int LIVE_Acq_flag1 = 0;
int LIVE_Acq_flag2 = 0;

int Laser_FIlter_Laser1 = 0;
int Laser_FIlter_Laser2 = 0;
int Laser_FIlter_Laser3 = 0;
int Laser_FIlter_Laser4 = 0;
int Laser_FIlter_Filter1 = 0;
int Laser_FIlter_Filter2 = 0;
int Laser_FIlter_Filter3 = 0;
int Laser_FIlter_Filter4 = 0;
int iCallbackCount = 0;
int iCallbackContext = 0;

AT_64 ImageSizeBytes;
AT_64 Camera1_sensorHight = 2304;
AT_64 Camera1_sensorWidth = 2304;
AT_64 Camera1_ImageHight = 2304;
AT_64 Camera1_ImageWidth = 2304;
AT_64 Camera1_dataWidth = 2304;
int BufferSize;
unsigned char* pBuf1;
int pBuf1_flag = 0;
int reStart_Tile_num;
unsigned char** AcqBuffers1 = new unsigned char*[NumberOfBuffers];
unsigned char** AlignedBuffers1 = new unsigned char*[NumberOfBuffers];

AT_64 Camera2_sensorHight = 2304;
AT_64 Camera2_sensorWidth = 2304;
AT_64 Camera2_ImageHight = 2304;
AT_64 Camera2_ImageWidth = 2304;
AT_64 Camera2_dataWidth = 2304;
int BufferSize2;
unsigned char* pBuf2;
int pBuf2_flag = 0;
unsigned char** AcqBuffers2 = new unsigned char*[NumberOfBuffers];
unsigned char** AlignedBuffers2 = new unsigned char*[NumberOfBuffers];

int filter_wheel_SPD;
int LIVE_flag = 0;
int Piezo_flag = 0;

int x_com_rec_flag = 0;
int y_com_rec_flag = 0;
int xy_com_rec_flag = 0;
int Galvo_flag = 0;
int sheet_flag = 0;
int DAQ1_flag = 0;
int DAQ2_flag = 0;
int Zstage_Flag = 0;
int ETL_Flag = 0;
double maxint = 0;
double meanint = 0;
double maxint2 = 0;
double meanint2 = 0;
double maxint3 = 0;
double meanint3 = 0;
int Xstage_Flagw = 0;
int Ystage_Flagw = 0;
int Xstage_Flag = 0;
int Ystage_Flag = 0;
int XYstage_Flag = 0;

double Z_pos_global;
int sheet_task_flag = 0;
int camera_task_flag = 0;
int tile_total = 0;;

long  PictureCount = 0;

int ActiveWindow = 1;
int Image_window1 = 0;
int Image_window2 = 0;
cv::Mat Imageseq1;//raw image
cv::Mat Imageseq2;//raw image
string Imageseq_fname1;
string Imageseq_fname2;
vector <string> filelist_dat1;
vector <string> filelist_tif1;
vector <string> filelist_dat2;
vector <string> filelist_tif2;
string def_filepath = "Z:\\";

struct WINDOWParam {
	int x_live = 0;
	int y_live = 0;
	int x0_live = 0;
	int y0_live = 0;
	int x1_live = 0;
	int y1_live = 0;
	int x_m_live = 0;
	int y_m_live = 0;
	int x_d_live = 0;
	int y_d_live = 0;
	int x_on_live = 0;
	int y_on_live = 0;
	int mouse_zoom_flag = 0;
	int mouse_zoomout_flag = 0;
	int mouse_move_flag = 0;
	int width_live = 2304;
	int hight_live = 2304;
	double scale_live;
	double max_live = 65535;
	double min_live = 0;
};

WINDOWParam LIVE1;
WINDOWParam PreScan_XY;
WINDOWParam PreScan_XZ;
WINDOWParam PreScan_YZ;
ThreadStats Thread_Stats = {};

namespace MacroScopever1 {
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Threading;
	using namespace System::IO;
	using namespace std;
	using namespace msclr::interop;
	using namespace OptecHID_FilterWheelAPI;


	int DAQ_clear();
	int DAQ1_clear();
	int DAQ2_clear(int pos);


	struct  AF_struct {
		int bufNum;
		double score;

		bool operator<(const AF_struct &left) const
		{
			return score > left.score;//年齢を比較
		};

	};
	int CompFocus(const void* pElem1, const void* pElem2)
	{
		const AF_struct* pmem1 = (const AF_struct*)pElem1;
		const AF_struct* pmem2 = (const AF_struct*)pElem2;

		int nDiff;  // 両要素の差

		return pmem2->score - pmem1->score;
	}

	void Make_Dst(char *Dst) {
		auto k = 0;
		for (auto i = 0; i < 128; ++i) {
			for (auto j = 0; j < 128; ++j, ++k) {
				if (i + j < 42) {
					Dst[k] = 255;
				}
				else {
					Dst[k] = 0;

				}
			}
		}
	}
	void Make_Dst_size(char *Dst, int size) {
		auto k = 0;
		int size2 = size / 3;
		for (auto i = 0; i < size; ++i) {
			for (auto j = 0; j < size; ++j, ++k) {
				if (i + j < size2) {
					Dst[k] = 255;
				}
				else {
					Dst[k] = 0;

				}
			}
		}
	}
	void Make_Dst_Full(char *Dst) {
		auto k = 0;
		for (auto i = 0; i < 2304; ++i) {
			for (auto j = 0; j < 2304; ++j, ++k) {
				if (i + j < 512) {
					Dst[k] = 255;
				}
				else {
					Dst[k] = 0;

				}
			}
		}
	}


	///*******************************************

	///       Function list;

	///*******************************************

	bool checkExistenceOfFolder(const std::string folder_name) {
		if (_mkdir(folder_name.c_str()) == 0) {
			return true;
		}
		else {
			return false;
		}
	}

	static int CALLBACK SHBrowseProc(HWND hWnd, UINT uMsg, LPARAM lParam, LPARAM lpData)
	{
		if (uMsg == BFFM_INITIALIZED && lpData)
		{
			//  デフォルトで選択させるパスの指定
			SendMessage(hWnd, BFFM_SETSELECTION, TRUE, lpData);
		}
		return 0;
	}
	static void SHFree(ITEMIDLIST* pidl)
	{
		IMalloc*  pMalloc;
		SHGetMalloc(&pMalloc);
		if (pMalloc)
		{
			pMalloc->Free(pidl);
			pMalloc->Release();
		}
	}
	string GetOpenDirectoryName(HWND hwndOwner, LPCTSTR lpszDefaultFolder, LPTSTR lpszBuffer, DWORD dwBufferSize)
	{
		BROWSEINFO  bi;
		ZeroMemory(&bi, sizeof(BROWSEINFO));
		bi.hwndOwner = hwndOwner;
		bi.lpfn = SHBrowseProc;	//  コールバック関数を指定
		bi.lParam = (LPARAM)lpszDefaultFolder;	//  デフォルトで選択させておくフォルダを指定
		bi.lpszTitle = _T("フォルダを選択してください");	//  タイトルの指定

		ITEMIDLIST*  pidl = SHBrowseForFolder(&bi);	//  フォルダダイアログの起動
		if (pidl)
		{
			//  選択されたフォルダ名を取得
			TCHAR  szSelectedFolder[_MAX_PATH];
			SHGetPathFromIDList(pidl, szSelectedFolder);
			SHFree(pidl);
			if ((DWORD)_tcslen(szSelectedFolder) < dwBufferSize)
			{
				_tcsncpy_s(lpszBuffer, dwBufferSize, szSelectedFolder, _TRUNCATE);
				//  フォルダが選択された

				//return TRUE;
			}
			return szSelectedFolder;
		}
		//  フォルダは選択されなかった
		return FALSE;
	}
	int __stdcall BrowseCallbackProc(HWND hWnd, UINT uMsg, LPARAM lParam, LPARAM lpData) {
		TCHAR dir[MAX_PATH];
		ITEMIDLIST *lpid;
		HWND hEdit;

		switch (uMsg) {
		case BFFM_INITIALIZED:  //      ダイアログボックス初期化時
			SendMessage(hWnd, BFFM_SETSELECTION, (WPARAM)TRUE, lpData);     //      コモンダイアログの初期ディレクトリ
			break;
		case BFFM_VALIDATEFAILED:       //      無効なフォルダー名が入力された
			//MessageBox(hWnd, (TCHAR*)lParam, _TEXT("無効なフォルダー名が入力されました"), MB_OK);
			hEdit = FindWindowEx(hWnd, NULL, _TEXT("EDIT"), NULL);     //      エディットボックスのハンドルを取得する
			SetWindowText(hEdit, _TEXT(""));
			return 1;       //      ダイアログボックスを閉じない
			break;
		case BFFM_IUNKNOWN:
			break;
		case BFFM_SELCHANGED:   //      選択フォルダーが変化した場合
			lpid = (ITEMIDLIST *)lParam;
			SHGetPathFromIDList(lpid, dir);
			hEdit = FindWindowEx(hWnd, NULL, _TEXT("EDIT"), NULL);     //      エディットボックスのハンドルを取得する
			SetWindowText(hEdit, dir);
			break;
		}
		return 0;
	}
	bool GetDir(HWND hWnd, TCHAR* def_dir, TCHAR* path) {
		BROWSEINFO bInfo;
		LPITEMIDLIST pIDList;

		memset(&bInfo, 0, sizeof(bInfo));
		bInfo.hwndOwner = hWnd; // ダイアログの親ウインドウのハンドル 
		bInfo.pidlRoot = NULL; // ルートフォルダをデスクトップフォルダとする 
		bInfo.pszDisplayName = path; //フォルダ名を受け取るバッファへのポインタ 
		bInfo.lpszTitle = TEXT("Select directory"); // ツリービューの上部に表示される文字列 
		bInfo.ulFlags = BIF_RETURNONLYFSDIRS | BIF_EDITBOX | BIF_VALIDATE | BIF_NEWDIALOGSTYLE; // 表示されるフォルダの種類を示すフラグ 
		bInfo.lpfn = BrowseCallbackProc; // BrowseCallbackProc関数のポインタ 
		bInfo.lParam = (LPARAM)def_dir;
		pIDList = SHBrowseForFolder(&bInfo);
		if (pIDList == NULL) {
			path[0] = _TEXT('\0');
			return false; //何も選択されなかった場合 
		}
		else {
			if (!SHGetPathFromIDList(pIDList, path))
				return false;//変換に失敗 
			CoTaskMemFree(pIDList);// pIDListのメモリを開放 
			return true;
		}
	}


	bool AF_score_asc(const shared_ptr<AF_struct>& left, const shared_ptr<AF_struct>& right) {
		return left->score < right->score;
	}
	bool AF_score_desc(const shared_ptr<AF_struct>& left, const shared_ptr<AF_struct>& right) {
		return left->score > right->score;
	}


	//*************************************************
	//Mouse event
	//*************************************************
	void CallBackFunc(int eventType, int x, int y, int flags, void* userdata)
	{
		try {
			mouseParam *ptr = static_cast<mouseParam*> (userdata);
			ptr->x = x;
			ptr->y = y;
			ptr->event = eventType;
			ptr->flags = flags;
		}
		catch (exception &e) {
			cout << "Mouse CallBack error " << &e << endl;
		}
	}
	bool Area_check(int &x, int &y, int width) {
		try {
			if (x < 0) {
				x = 0;
			}
			if (y < 0) {
				y = 0;
			}

			if (width > 2304) {
				width = 2304;
			}
			if (x + width > 2304) {
				x = 2304 - width;
			}
			if (y + width > 2304) {
				y = 2304 - width;
			}
			return true;
		}
		catch (exception &e) {
			return false;
		}
	}
	int Imgae_Func(int &x, int &y, int &x0, int &y0, int &x1, int &y1, int &width) {

		if (abs(x1 - x0) > abs(y1 - y0)) {
			width = abs((x1 - x0));
		}
		if (abs(x1 - x0) < abs(y1 - y0)) {
			width = abs((y1 - y0));
		}
		if (width < 50) {
			return 0;
		}
		if (x0 < x1) {
			x = x + x0;
		}
		if (x0 > x1) {
			x = x + x1;
		}
		if (y0 < y1) {
			y = y + y0;
		}
		if (y0 > y1) {
			y = y + y1;
		}
		return 1;


	}
	int Image_ZoomOut_Func(int &x, int &x0, int &y, int &y0, int &width, int &hight) {
		double ratex = double(x0) / double(hight);
		double ratey = double(y0) / double(width);
		width *= ZOOMRATE;
		hight *= ZOOMRATE;
		x = x + x0 - (hight* ratex);
		y = y + y0 - (width* ratey);
		return 1;

	}
	int Image_Zoom_Func(int &x, int &x0, int &y, int &y0, int &width, int &hight) {
		double ratex = double(x0) / double(hight);
		double ratey = double(y0) / double(width);
		width /= ZOOMRATE;
		hight /= ZOOMRATE;
		x = x + x0 - (hight* ratex);
		y = y + y0 - (width* ratey);
		//cout << ratex << endl;
		//cout << ratey << endl;
		//cout << x << endl;
		//cout << y << endl;
		return 1;

	}

	//*************************************************
	//CRC16 check sum
	//*************************************************

	uint16_t crc16_update(uint16_t crc, uint8_t a)
	{
		int i;
		crc ^= a;
		for (i = 0; i < 8; ++i)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc = (crc >> 1);
		}
		return crc;
	}
	uint16_t calculatecrc(uint8_t data[])
	{
		uint16_t crc = 0, i;
		for (i = 0; i < sizeof(data) / sizeof(data[0]); i++)
		{
			crc = crc16_update(crc, data[i]);
		}
		return crc; //returns checksum over all elements
	}


	//*************************************************
	//ETL setting
	//*************************************************

	void ETL_set_current(cli::array<unsigned char>^ &buffer, float current) {
		int Xi;

		uint16_t Xn;
		uint8_t buffer1[4];
		Xi = (current / 293 * 4096);
		buffer1[0] = 'A';
		buffer1[1] = 'w';
		buffer1[2] = Xi / 256;
		buffer1[3] = Xi % 256;
		Xn = calculatecrc(buffer1);
		uint8_t buffer2[8];
		buffer2[0] = 'A';
		buffer2[1] = 'w';
		buffer2[2] = Xi / 256;
		buffer2[3] = Xi % 256;
		buffer2[4] = Xn % 256;
		buffer2[5] = Xn / 256;
		for (int i = 0; i < 8; i++) {
			buffer[i] = buffer2[i];
			//cout << buffernnn[i];
		}
		//cout << endl;


	}
	//*************************************************
	//DAQ control
	//*************************************************
	int ClearTaskHandle(TaskHandle& taskHandle) {
		int32 errorCode = 0;
		char errBuff[2048] = { 0 };
		cout << "ClearTaskHandle" << endl;
		//printf("TaskHandle before calling ClearTaskHandle: %p\n", taskHandle);
		if (taskHandle == 0) {
			//printf("TaskHandle is already cleared.\n");
			return 0; // 既にクリアされている場合は正常終了
		}
		// タスクが動作中かを確認
		bool32 isDone = 0;
		errorCode = DAQmxIsTaskDone(taskHandle, &isDone);
		if (errorCode != 0) {
			DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
			printf("Error checking task status: %s\n", errBuff);
			return errorCode; // エラー発生時に終了
		}

		if (!isDone) {
			// タスクが動作中の場合は停止
			errorCode = DAQmxStopTask(taskHandle);
			if (errorCode != 0) {
				DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
				printf("Error stopping task: %s\n", errBuff);
				return errorCode;
			}
		}
		// タスクのクリア
		errorCode = DAQmxClearTask(taskHandle);
		if (errorCode != 0) {
			DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
			printf("Error clearing task: %s\n", errBuff);
			return errorCode;
		}

		// 正常にクリアされた場合、taskHandleをリセット
		taskHandle = 0;
		//printf("Task successfully cleared.\n");
		return 0; // 成功
	}


	int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);
	void DAQ_reset();
	void DAQ_ERR(int i) {
		cout << "DAQ_ERR(): from " << i << " ;";
		int32   error = 0;
		char    errBuff[2304] = { '\0' };
		DAQmxGetExtendedErrorInfo(errBuff, 2304);
		printf("DAQmx Error: %s\n", errBuff);
		//DAQ1_clear();
		DAQ_reset();
		//MOVIE_stats.retry = true;

	}
	int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
	{
		int32   error = 0;
		char    errBuff[2304] = { '\0' };
		//cout << "DoneCallback" << endl;
		// Check to see if an error stopped the task.
		DAQmxErrChk(status);
		return 1;
	Error:
		if (DAQmxFailed(error)) {
			DAQ_ERR(0);
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			ClearTaskHandle(taskHandle);
			printf("DAQmx Error: %s\n", errBuff);
		}

		return 0;
	}
	int32 CVICALLBACK DoneCallback_Axial_DAQ(TaskHandle taskHandle, int32 status, void *callbackData)
	{
		int32   error = 0;
		char    errBuff[2048] = { '\0' };
		//cout << "DoneCallback_Axial_DAQ" << endl;
		// Check to see if an error stopped the task.
		DAQmxErrChk(status);
		return 1;
	Error:
		if (DAQmxFailed(error)) {
			DAQ_ERR(1);
		}
		return 0;
	}

	int DAQ1(double *data, double Sample, double rate)
	{
		DAQ1_clear();
		//cout << "DAQ1 start" << endl;
		DWORD dwWaitResult = WaitForSingleObject(Daq1Mutex, INFINITE);
		//cout << "DAQ1 start2" << endl;
		threads.Daq = true;
		int32       error = 0;
		DAQ1_flag = 1;
		char        errBuff[2048] = { '\0' };
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, DAQ1_ALL, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, Sample));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));
		DAQmxErrChk(DAQmxStartTask(taskHandle));
		threads.Daq = false;
		return 1;
	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(2);
		threads.Daq = false;
		ReleaseMutex(Daq1Mutex);
		DAQ1_clear();
		return 0;
	}

	int DAQ2(double *data, double Sample, double rate)
	{
		DAQ2_clear(0);
		DWORD dwWaitResult = WaitForSingleObject(Daq2Mutex, INFINITE);
		threads.Daq = true;
		//cout << "DAQ2 start" << endl;
		int32       error = 0;

		char        errBuff[2048] = { '\0' };
		/*********************************************/
		// DAQmx Configure Code
		/*********************************************/
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle2));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle2, DAQ2_ALL, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle2, "", rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, Sample));
		DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle2, 0, DoneCallback, NULL));

		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle2, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));
		DAQmxErrChk(DAQmxStartTask(taskHandle2));
		//ReleaseMutex(Daq2Mutex);
		return 1;
	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(4);
		threads.Daq = false;
		ReleaseMutex(Daq2Mutex);
		DAQ2_clear(1);
		return 0;
	}

	int DAQ2_single(double Sample, double rate, double *data)
	{
		DAQ2_clear(2);
		DWORD dwWaitResult = WaitForSingleObject(Daq2Mutex, INFINITE);
		threads.Daq = true;
		int32       error = 0;
		char        errBuff[2048] = { '\0' };
		/*********************************************/
		// DAQmx Configure Code
		/*********************************************/
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle2));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle2, DAQ2_ALL, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle2, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
		DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle2, 0, DoneCallback, NULL));

		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle2, Sample, 0, 10, DAQmx_Val_GroupByChannel, data, NULL, NULL));
		DAQmxErrChk(DAQmxStartTask(taskHandle2));
		DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle2, 10.0));
		DAQ2_clear(3);
		return 0;
	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(6);
		DAQ2_clear(4);

		return 0;
	}
	int DAQ1_single(double Sample, double rate, double *data)
	{
		DAQ1_clear();
		DWORD dwWaitResult = WaitForSingleObject(Daq1Mutex, INFINITE);
		threads.Daq = true;
		int32       error = 0;
		char        errBuff[2048] = { '\0' };
		/*********************************************/
		// DAQmx Configure Code
		/*********************************************/
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, DAQ1_ALL, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
		DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10, DAQmx_Val_GroupByChannel, data, NULL, NULL));
		DAQmxErrChk(DAQmxStartTask(taskHandle));
		DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle, 10.0));
		DAQ1_clear();
		return 0;
	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(6);
		DAQ1_clear();
		return 0;
	}
	void DAQ_sheet_endpos() {
		//cout << "DAQ_sheet_endpos" << endl;
		double data[8] = { 3, 3, 0, 0, 9, 9, -9, -9 };
		DAQ1_single(2, 2000, data);
	}
	int DAQ_sheet_Zeropos()
	{
		cout << "Goto Sheet Zero Pos" << endl;
		int32       error = 0;
		double data[8] = { 3, 3, 0, 0, 0, 0, 0, 0 };
		DAQ1_single(2, 2000, data);
		return 0;
	}

	void DAQ_reset() {
		DWORD dwWaitResult = WaitForSingleObject(Daq1Mutex, INFINITE);
		DWORD dwWaitResult2 = WaitForSingleObject(Daq2Mutex, INFINITE);
		DAQmxResetDevice("cDAQ1Mod1");
		DAQmxResetDevice("cDAQ2Mod1");
		ReleaseMutex(Daq1Mutex);
		ReleaseMutex(Daq2Mutex);
	}

	int DAQ_clear() {
		int ret1 = DAQ1_clear();
		int ret2 = DAQ2_clear(5);
		if (ret1 != 0 || ret2 != 0) {
			if (ret1 != 0) printf("Error clearing DAQ1 task.\n");
			if (ret2 != 0) printf("Error clearing DAQ2 task.\n");
			return -1; // 失敗時は一括してエラーを返す
		}
		return 0; // 正常終了
	}
	int DAQ1_clear() {
		//cout << "DAQ1_clear" << endl;
		ClearTaskHandle(taskHandle);
		ReleaseMutex(Daq1Mutex);
		threads.Daq = false;
		return 0;
	}
	int DAQ2_clear(int pos) {
		//cout << "DAQ2_clear from " << pos << endl;
		ClearTaskHandle(taskHandle2);
		ReleaseMutex(Daq2Mutex);
		threads.Daq = false;
		return 0;
	}


	//*************************************************
	void MarshalString(String ^ s, string& os) {
		using namespace Runtime::InteropServices;
		const char* chars =
			(const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
		os = chars;
		Marshal::FreeHGlobal(IntPtr((void*)chars));
	}
	bool check_int(std::string str)
	{
		if (str[0] == '-') {
			str = str.erase(0, 1);

			if (std::all_of(str.cbegin(), str.cend(), isdigit))
			{
				//std::cout << stoi(str) << std::endl;
				return true;
			}
		}
		else if (std::all_of(str.cbegin(), str.cend(), isdigit))
		{
			//std::cout << stoi(str) << std::endl;
			return true;
		}

		//std::cout << "not int" << std::endl;
		return false;
	}
	bool check_float(std::string str)
	{
		if (str[0] == '-') {
			str = str.erase(0, 1);

			if (std::all_of(str.cbegin(), str.cend(), isdigit))
			{
				//	std::cout << stof(str) << std::endl;
				return true;
			}
		}
		//std::cout << "not int" << std::endl;
		return false;
	}

	static bool* numberToBoolArray_true(size_t number, bool *data, size_t dataSize)
	{
		size_t i = 0;
		for (i = 0; i < dataSize; ++i)
		{
			data[i] = true;
			//data[i] = (number & ((size_t)1 << i)) ? true : false;
		}
		return data;
	}
	static bool* numberToBoolArray_false(size_t number, bool *data, size_t dataSize)
	{
		size_t i = 0;
		for (i = 0; i < dataSize; ++i)
		{
			data[i] = false;
			//data[i] = (number & ((size_t)1 << i)) ? true : false;
		}
		return data;
	}
	static void printDigitalData(const bool *data, size_t dataSize)
	{
		size_t i = 0;
		for (i = 0; i < dataSize; ++i)
		{
			if (data[i])
			{
				printf("T");
			}
			else
			{
				printf("F");
			}
		}
	}
	inline vector<string> split_comma(const string& src, const char* delim = ",") {
		vector<string> vec;
		string::size_type len = src.length();
		for (string::size_type i = 0, n; i < len; i = n + 1) {
			n = src.find_first_of(delim, i);
			if (n == string::npos) {
				n = len;
			}
			vec.push_back(src.substr(i, n - i));
		}
		return vec;
	}
	inline vector<int> split_comma_i(const string& src, const char* delim = ",") {
		vector<int> vec;
		string::size_type len = src.length();
		for (string::size_type i = 0, n; i < len; i = n + 1) {
			n = src.find_first_of(delim, i);
			if (n == string::npos) {
				n = len;
			}
			vec.push_back(stoi(src.substr(i, n - i)));
		}
		return vec;
	}
	inline vector<double> split_comma_d(const string& src, const char* delim = ",") {
		vector<double> vec;
		string::size_type len = src.length();
		for (string::size_type i = 0, n; i < len; i = n + 1) {
			n = src.find_first_of(delim, i);
			if (n == string::npos) {
				n = len;
			}
			vec.push_back(stod(src.substr(i, n - i)));
		}
		return vec;
	}
	void COUT_VEC_String(const vector<string> &vec) {
		for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
			cout << *itr << "\n";    // �ŃC�e���[�^�̎w���v�f���Q��
		}
	}
	void COUT_VEC_Int(const vector<int> &vec) {
		for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
			cout << *itr << "\n";    // �ŃC�e���[�^�̎w���v�f���Q��
		}
	}
	void COUT_VEC_Char(const vector<char*> &vec) {
		for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
			cout << *itr << "\n";    // �ŃC�e���[�^�̎w���v�f���Q��
		}
	}

	void Load_Axial_Param(string FP, int laser) {
		FILE *stream;
		char buf[1024];
		int count = 0;
		//vector<<int>*temp=new vector<int>[15];
		cout << "Open:" << FP << endl;
		fopen_s(&stream, FP.c_str(), "r");
		fgets(buf, 1024, stream);
		if (laser == 0) {
			cout << "ETL 488: " << buf;
			Axial_ETL_param.Laser488[0] = split_comma_d(buf)[0];
			Axial_ETL_param.Laser488[1] = split_comma_d(buf)[1];
			Axial_ETL_param.Laser488[2] = split_comma_d(buf)[2];
			Axial_ETL_param.Laser488[3] = split_comma_d(buf)[3];
		}
		else if (laser == 1) {
			cout << "ETL 592: " << buf;
			Axial_ETL_param.Laser592[0] = split_comma_d(buf)[0];
			Axial_ETL_param.Laser592[1] = split_comma_d(buf)[1];
			Axial_ETL_param.Laser592[2] = split_comma_d(buf)[2];
			Axial_ETL_param.Laser592[3] = split_comma_d(buf)[3];
		}
		else if (laser == 2) {
			cout << "ETL 647: " << buf;
			Axial_ETL_param.Laser647[0] = split_comma_d(buf)[0];
			Axial_ETL_param.Laser647[1] = split_comma_d(buf)[1];
			Axial_ETL_param.Laser647[2] = split_comma_d(buf)[2];
			Axial_ETL_param.Laser647[3] = split_comma_d(buf)[3];
		}
		fclose(stream);
	}
	void Initial_AxialSetting() {
		string FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_488.csv";
		Load_Axial_Param(FP, 0);
		FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_592.csv";
		Load_Axial_Param(FP, 1);
		FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_647.csv";
		Load_Axial_Param(FP, 2);
	}


	delegate void SerialDataRecievedDelegate_F(String^ recvData);
	delegate void SerialDataRecievedDelegate_z(String^ recvData);
	delegate void SerialDataRecievedDelegate_x(String^ recvData);
	delegate void SerialDataRecievedDelegate_x2(String^ recvData);
	delegate void SerialDataRecievedDelegate_y(String^ recvData);
	delegate void SerialDataRecievedDelegate_m(String^ recvData);
	delegate void SerialDataRecievedDelegate_488(String^ recvData);
	delegate void SerialDataRecievedDelegate_592(String^ recvData);
	delegate void SerialDataRecievedDelegate_647(String^ recvData);
	delegate void SerialDataRecievedDelegate_theta(String^ recvData);
	delegate void SerialDataRecievedDelegate_piezo(String^ recvData);
	delegate void SerialDataRecievedETL_L(String^ recvData);
	delegate void SerialDataRecievedETL_R(String^ recvData);
	delegate void DataexportDelegate(int cameraNum, int filenum, string FP, int &BufferSize, unsigned char* Buffer);
	delegate void DAQ_start(double Sample, double rate, double data[250000], char Dev[200]);
	delegate void single_exposure();
	delegate void CameraID();



	/// <summary>
	/// MyForm の概要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
		value class TASK_FOR_DAQ {
		public:
			double galvo_data;
		};
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: ここにコンストラクター コードを追加します
			//

			Console::WriteLine("Initialization Start");
			Zstage_port->Open();
			XYstage_port->Open();
			OpenComs();

			filter_wheel_SPD = int::Parse(Filter_wheel_SPD->Text);
			Zstage_Flag = 1;
			illumination_flag = 'L';
			BW_Q_Laser->RunWorkerAsync();
			BW_filter_check->RunWorkerAsync();
			Color_check->RunWorkerAsync();
			Color_check2->RunWorkerAsync();
			Zstage_Flag = 2;
			Xstage_Flagw = 0;
			Ystage_Flagw = 0;
			Open_ThorLabs(TSerials, x10Filterpos, Focus_Pos, Shutter_state);
			Flipper_Change(TSerials.FilterFlipper_Serial, x10Filterpos, 1);
			Camera_ini->RunWorkerAsync();
			startG = std::chrono::system_clock::now();
			Surveillance->RunWorkerAsync();
			filter_setting();
			Initial_AxialSetting();
			backgroundWorker_XYstage->RunWorkerAsync();
			backgroundWorker_Zstage->RunWorkerAsync();
			filter_ini->RunWorkerAsync();
			backgroundWorker1->RunWorkerAsync();
			Open_ETLParam();
			Laser_R_ON();
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}

	protected:
	private: System::Windows::Forms::TrackBar^  Amplitude_sheet_bar;
	private: System::Windows::Forms::TextBox^  Amplitude_sheet;
	private: System::Windows::Forms::Button^  L488_ON;
	private: System::Windows::Forms::Button^  L592_ON;
	private: System::Windows::Forms::Button^  L647_ON;
	private: System::Windows::Forms::Button^  Filter_1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TrackBar^  L488_bar;
	private: System::Windows::Forms::TrackBar^  L592_bar;
	private: System::Windows::Forms::TrackBar^  L647_bar;
	private: System::Windows::Forms::TextBox^  L488_box;
	private: System::Windows::Forms::TextBox^  L592_box;
	private: System::Windows::Forms::TextBox^  L647_box;
	private: System::IO::Ports::SerialPort^  Laser_488;
	private: System::IO::Ports::SerialPort^  Laser_647;
	private: System::IO::Ports::SerialPort^  Laser_592;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::TextBox^  X_pos_current;
	private: System::Windows::Forms::TextBox^  Y_pos_current;
	private: System::Windows::Forms::TextBox^  Z_pos_current;
	private: System::IO::Ports::SerialPort^  XYstage_port;
	private: System::IO::Ports::SerialPort^  Zstage_port;
	private: System::Windows::Forms::TextBox^  Z_target_pos;
	private: System::ComponentModel::IContainer^  components;
	private: String ^ STX = "\x02";
	private: String ^ Logfilename;
	private: String ^ XY_res;
	private: String ^ Z_res;
	private: String ^ L488_Res;
	private: String ^ L592_Res;
	private: String ^ L647_Res;
	private: String ^ LR_res;
	private: String ^ FW_res;
	private: String ^ L488_COM_NUM;
	private: String ^ L647_COM_NUM;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker_Zstage;
	private: System::Windows::Forms::TextBox^  CameraNo_1;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::TrackBar^  exposure_time_bar;
	private: System::Windows::Forms::TextBox^  exposure_time_box;
	private: System::Windows::Forms::Button^  up_1;
	private: System::Windows::Forms::Button^  down_1;
	private: System::Windows::Forms::Button^  right_1;
	private: System::Windows::Forms::Button^  Z_down;
	private: System::Windows::Forms::Button^  Z_up;
	private: System::Windows::Forms::Button^  Abort;
	private: System::Windows::Forms::Button^  Laser_L;
	private: System::Windows::Forms::Button^  Start_Z_but;
	private: System::Windows::Forms::Button^  End_Z_but;
	private: System::Windows::Forms::TextBox^  Start_Z_pos;
	private: System::Windows::Forms::TextBox^  End_Z_pos;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::Label^  label20;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::TextBox^  Step;
	private: System::Windows::Forms::Label^  label24;
	private: System::Windows::Forms::TrackBar^  ETL_bar;
	private: System::Windows::Forms::Label^  label29;
	private: System::Windows::Forms::TextBox^  ETL_box;
	private: System::IO::Ports::SerialPort^  ETL_L;
	private: System::IO::Ports::SerialPort^  ETL_R;
	private: System::Windows::Forms::Button^  LIVE_but;
	private: System::Windows::Forms::Button^  close;
	private: System::IO::Ports::SerialPort^  SIGMA_Theta;
	private: System::Windows::Forms::Button^  Laser_R;
	private: System::Windows::Forms::TextBox^  Stem_box1;
	private: System::Windows::Forms::TextBox^  Export_Box;
	private: System::Windows::Forms::Label^  label88;
	private: System::Windows::Forms::Label^  label89;
	private: System::Windows::Forms::Label^  label91;
	private: System::Windows::Forms::Button^  Export_dir_open;
	private: System::Windows::Forms::Label^  label92;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::TextBox^  LP4;
	private: System::Windows::Forms::TextBox^  LP3;
	private: System::Windows::Forms::TextBox^  LP2;
	private: System::Windows::Forms::ComboBox^  filter4_comb;
	private: System::Windows::Forms::ComboBox^  filter3_comb;
	private: System::Windows::Forms::ComboBox^  filter2_comb;
	private: System::Windows::Forms::ComboBox^  Laser4_comb;
	private: System::Windows::Forms::ComboBox^  Laser3_comb;
	private: System::Windows::Forms::ComboBox^  Laser2_comb;
	private: System::Windows::Forms::Label^  label94;
	private: System::Windows::Forms::Label^  label93;
	private: System::Windows::Forms::CheckBox^  LF_check4;
	private: System::Windows::Forms::CheckBox^  LF_check3;
	private: System::Windows::Forms::CheckBox^  LF_check2;
	private: System::Windows::Forms::CheckBox^  LF_check1;
	private: System::Windows::Forms::Button^  FIlter_2;
	private: System::Windows::Forms::Button^  FIlter_5;
	private: System::Windows::Forms::Button^  FIlter_6;
	private: System::Windows::Forms::GroupBox^  groupBox7;
	private: System::Windows::Forms::Button^  FIlter_8;
	private: System::Windows::Forms::Button^  FIlter_4;
	private: System::Windows::Forms::Button^  FIlter_7;
	private: System::Windows::Forms::Button^  FIlter_3;
	private: System::Windows::Forms::Button^  Single_Acquire;
	private: String ^ COMrec_buf;
	private: System::Windows::Forms::CheckBox^  multi_laser_check;
	private: System::ComponentModel::BackgroundWorker^  Color_check;
	private: System::ComponentModel::BackgroundWorker^  BW_XY_ini;
	private: System::Windows::Forms::TextBox^  LR_box;
	private: System::ComponentModel::BackgroundWorker^  Color_check2;
	private: System::ComponentModel::BackgroundWorker^  BW_Q_Laser;
	private: System::ComponentModel::BackgroundWorker^  BW_filter_check;
	private: System::Windows::Forms::TextBox^  current_filter_box;
	private: System::Windows::Forms::CheckBox^  MicroStep1;
	private: System::Windows::Forms::TextBox^  EXP_time4;
	private: System::Windows::Forms::TextBox^  EXP_time3;
	private: System::Windows::Forms::TextBox^  EXP_time2;
	private: System::Windows::Forms::Label^  label104;
	private: System::Windows::Forms::TextBox^  EXP_time1;
	private: System::Windows::Forms::TextBox^  LP1;
	private: System::Windows::Forms::ComboBox^  filter1_comb;
	private: System::Windows::Forms::ComboBox^  Laser1_comb;
	private: System::Windows::Forms::StatusStrip^  statusbar;
	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	public: System::Windows::Forms::ToolStripProgressBar^  toolStripProgressBar1;
	private: System::Windows::Forms::ToolStripStatusLabel^  z_left;
	private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel3;
	private: System::Windows::Forms::ToolStripStatusLabel^  z_right;
	private: System::Windows::Forms::ToolStripStatusLabel^  ThreadCondition;
	private: System::ComponentModel::BackgroundWorker^  BW_Cursor;
	public: System::ComponentModel::BackgroundWorker^  Progress_count;
	private: System::Windows::Forms::GroupBox^  groupBox8;
	private: System::Windows::Forms::ToolStripMenuItem^  saveFileToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  resetSettingToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem2;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem8;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator2;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem9;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem5;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem6;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem7;
	private: System::Windows::Forms::Button^  left_1;
	private: System::ComponentModel::BackgroundWorker^  filter_ini;
	private: System::ComponentModel::BackgroundWorker^  Laser_ini;
	private: System::ComponentModel::BackgroundWorker^  BW_ActiveWindow;
	private: System::Windows::Forms::ToolStripStatusLabel^  xpos;
	private: System::Windows::Forms::ToolStripStatusLabel^  ypos;
	private: System::ComponentModel::BackgroundWorker^  Camera_ini;
	private: System::Windows::Forms::TextBox^  set4_name;
	private: System::Windows::Forms::TextBox^  set3_name;
	private: System::Windows::Forms::TextBox^  set2_name;
	private: System::Windows::Forms::TextBox^  set1_name;
	private: System::Windows::Forms::Label^  label115;
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::Button^  LIVE_window_close2;
	private: System::Windows::Forms::Button^  LIVE_window_close1;
	private: System::Windows::Forms::TrackBar^  SH_low1_bar;
	private: System::Windows::Forms::TrackBar^  SH_low2_bar;
	private: System::Windows::Forms::TrackBar^  SH_high2_bar;
	private: System::Windows::Forms::TrackBar^  SH_high1_bar;
	private: System::Windows::Forms::TextBox^  SH_low2;
	private: System::Windows::Forms::TextBox^  SH_high2;
	private: System::Windows::Forms::TextBox^  SH_low1;
	private: System::Windows::Forms::TextBox^  SH_high1;
	private: System::Windows::Forms::CheckBox^  manual_scale2;
	private: System::Windows::Forms::CheckBox^  manual_scale1;
	private: System::Windows::Forms::Button^  ResizeWindow;
	private: System::Windows::Forms::TextBox^  ReStart_tile_num_box;
	private: System::Windows::Forms::Button^  MOVIE_strat;
	private: System::Windows::Forms::Label^  label73;
	private: System::Windows::Forms::TabControl^  aa;
	private: System::Windows::Forms::Button^  Laser_filter_setting_but;
	private: System::Windows::Forms::TabPage^  tabPage1;
	private: System::Windows::Forms::Label^  label109;
	private: System::Windows::Forms::TextBox^  Xpos_cursol;
	private: System::Windows::Forms::Label^  label112;
	private: System::Windows::Forms::TextBox^  Ypos_cursol;
	private: System::Windows::Forms::Label^  label116;
	private: System::Windows::Forms::TextBox^  Cursol_int;
	private: System::Windows::Forms::Label^  ma;
	private: System::Windows::Forms::Label^  qw;
	private: System::Windows::Forms::TextBox^  ImageMax;
	private: System::Windows::Forms::TextBox^  ImageMin;
	private: System::Windows::Forms::GroupBox^  groupBox9;
	private: System::Windows::Forms::CheckBox^  Axial_mode_check;
	private: System::Windows::Forms::TabPage^  Camera;
	private: System::Windows::Forms::CheckBox^  Down2Top;
	private: System::Windows::Forms::CheckBox^  Top2Down;
	private: System::Windows::Forms::CheckBox^  ExternalExposure;
	private: System::Windows::Forms::CheckBox^  OverlapMode;
	private: System::Windows::Forms::CheckBox^  External;
	private: System::Windows::Forms::CheckBox^  C2OUT;
	private: System::Windows::Forms::Label^  label141;
	private: System::Windows::Forms::Label^  label140;
	private: System::Windows::Forms::Label^  label138;
	private: System::Windows::Forms::TextBox^  Windowsize;
	private: System::Windows::Forms::Label^  label142;
	private: System::Windows::Forms::Label^  label143;
	private: System::Windows::Forms::TextBox^  ScanSpeed;
	private: System::Windows::Forms::Button^  camera_set;
	private: System::Windows::Forms::CheckBox^  Internal;
	private: System::Windows::Forms::Button^  Get_Camera_Condition;
	private: System::Windows::Forms::TextBox^  PixelReadoutRate_box;
	private: System::Windows::Forms::TextBox^  Scanspeedcontrol_box;
	private: System::Windows::Forms::TextBox^  exposuretime_box;
	private: System::Windows::Forms::TextBox^  overlapmode_box;
	private: System::Windows::Forms::TextBox^  ScanSpeed_box;
	private: System::Windows::Forms::TextBox^  Shuttermode_box;
	private: System::Windows::Forms::TextBox^  Triggermode_box;
	private: System::Windows::Forms::Label^  label148;
	private: System::Windows::Forms::Label^  label149;
	private: System::Windows::Forms::Label^  label147;
	private: System::Windows::Forms::Label^  label146;
	private: System::Windows::Forms::Label^  label145;
	private: System::Windows::Forms::Label^  label144;
	private: System::Windows::Forms::Label^  label150;
	private: System::Windows::Forms::CheckBox^  ScanSpeedControl_Check;
	private: System::Windows::Forms::CheckBox^  FastReadOut;
	private: System::Windows::Forms::Button^  Axial_647_load;
	private: System::Windows::Forms::Button^  Axial_592_load;
	private: System::Windows::Forms::Button^  Axial_488_load;
	private: System::Windows::Forms::Button^  Axial_Param_set;
	private: System::Windows::Forms::TextBox^  Axial_panel;
	private: System::Windows::Forms::Label^  label113;
	private: System::Windows::Forms::TextBox^  ExpoTime_Axial;
	private: System::Windows::Forms::Label^  label117;
	private: System::Windows::Forms::Button^  Axial_Param_set_R;
	private: System::Windows::Forms::CheckBox^  singlestackmode;
	private: System::Windows::Forms::GroupBox^  groupBox10;
	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem4;
	private: System::Windows::Forms::TextBox^  CameraNo_2;
	private: System::Windows::Forms::CheckBox^  Camera_2_use;
	private: System::Windows::Forms::TabPage^  tabPage3;
	private: System::Windows::Forms::TextBox^  Z_offset;
	private: System::Windows::Forms::TrackBar^  trackBar_Z;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label33;
	private: System::Windows::Forms::TextBox^  MFocus_right;
	private: System::Windows::Forms::TextBox^  MFocus_left;
	private: System::Windows::Forms::Label^  label35;
	private: System::Windows::Forms::TextBox^  DCTScoreBox;
	private: System::Windows::Forms::TabPage^  Calibration;
	private: System::Windows::Forms::GroupBox^  groupBox13;
	private: System::Windows::Forms::TextBox^  sheet_offset_R;
	private: System::Windows::Forms::TextBox^  sheet_offset_L;
	private: System::Windows::Forms::Label^  label83;
	private: System::Windows::Forms::Label^  label82;
	private: System::Windows::Forms::Label^  sheet;
	private: System::Windows::Forms::GroupBox^  groupBox11;
	private: System::Windows::Forms::TextBox^  filter_name8;
	private: System::Windows::Forms::TextBox^  filter_name4;
	private: System::Windows::Forms::TextBox^  filter_name7;
	private: System::Windows::Forms::TextBox^  filter_name3;
	private: System::Windows::Forms::TextBox^  Filter_wheel_SPD;
	private: System::Windows::Forms::TextBox^  filter_name6;
	private: System::Windows::Forms::TextBox^  Theta_SIGMA;
	private: System::Windows::Forms::TextBox^  filter_name2;
	private: System::Windows::Forms::TextBox^  filter_name5;
	private: System::Windows::Forms::TextBox^  filter_name1;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label81;
	private: System::Windows::Forms::Label^  label103;
	private: System::Windows::Forms::Button^  Theta_SIGMA_but;
	private: System::Windows::Forms::Label^  label99;
	private: System::Windows::Forms::Label^  label102;
	private: System::Windows::Forms::Label^  label98;
	private: System::Windows::Forms::Label^  label101;
	private: System::Windows::Forms::Label^  label97;
	private: System::Windows::Forms::Label^  label100;
	private: System::Windows::Forms::Label^  label96;
	private: System::Windows::Forms::TextBox^  sweep_max;
	private: System::Windows::Forms::TextBox^  sweep_min;
	private: System::Windows::Forms::Button^  z_zero;
	private: System::Windows::Forms::Button^  ETL_Sweep;
	private: System::Windows::Forms::Button^  Galvo_Zero;
	private: System::ComponentModel::BackgroundWorker^  Camera_ini2;
	private: System::Windows::Forms::TextBox^  ETL_swept_time;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::TextBox^  imagingtile;
	private: System::Windows::Forms::Label^  nama;
	private: System::Windows::Forms::GroupBox^  groupBox16;
	private: System::Windows::Forms::TextBox^  DCTFocusValBox;
	private: System::Windows::Forms::Label^  label84;
	private: System::Windows::Forms::TextBox^  bottom_limit_RV;
	private: System::Windows::Forms::TextBox^  left_limit_RV;
	private: System::Windows::Forms::TextBox^  right_limit_RV;
	private: System::Windows::Forms::TextBox^  top_limit_RV;
	private: System::Windows::Forms::TextBox^  theta_offset;
	private: System::Windows::Forms::Label^  label118;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker1;
	private: System::Windows::Forms::TextBox^  TargetX_box;
	private: System::Windows::Forms::TextBox^  TargetY_box;
	private: System::Windows::Forms::Label^  label152;
	private: System::Windows::Forms::Label^  label153;
	private: System::Windows::Forms::Label^  label31;
	private: System::Windows::Forms::Button^  Zweep;
	private: System::Windows::Forms::TextBox^  Sheet_Box;
	private: System::Windows::Forms::TrackBar^  Sheet_bar;
	private: System::Windows::Forms::Button^  Sheet_Step;
	private: System::Windows::Forms::TextBox^  sheet3;
	private: System::Windows::Forms::TextBox^  sheet2;
	private: System::Windows::Forms::TextBox^  sheet1;
	private: System::IO::Ports::SerialPort^  serialPort1;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::TextBox^  ETL_R_right;
	private: System::Windows::Forms::TextBox^  ETL_L_right;
	private: System::Windows::Forms::TextBox^  ETL_R_left;
	private: System::Windows::Forms::TextBox^  ETL_L_left;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label26;
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::ToolStripMenuItem^  eTLAxialToolStripMenuItem;
	private: System::Windows::Forms::Label^  label28;
	private: System::Windows::Forms::TextBox^  Z_galv_R;
	private: System::Windows::Forms::TextBox^  Z_galv_L;
	private: System::Windows::Forms::Label^  label34;
	private: System::Windows::Forms::Label^  label30;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  label39;
	private: System::Windows::Forms::TextBox^  bottom_limit;
	private: System::Windows::Forms::TextBox^  right_limit;
	private: System::Windows::Forms::TextBox^  left_limit;
	private: System::Windows::Forms::TextBox^  top_limit;
	private: System::Windows::Forms::Button^  left_limit_but;
	private: System::Windows::Forms::Button^  Y_limit_but;
	private: System::Windows::Forms::Button^  right_limit_but;
	private: System::Windows::Forms::Button^  top_limit_but;
	private: System::Windows::Forms::TextBox^  Overlap_um_y;
	private: System::Windows::Forms::TextBox^  Overlap_um;
	private: System::Windows::Forms::Label^  label41;
	private: System::Windows::Forms::Label^  label40;
	private: System::Windows::Forms::Label^  label43;
	private: System::Windows::Forms::Label^  label42;
	private: System::Windows::Forms::Label^  label38;
	private: System::Windows::Forms::TextBox^  Z_overlap_box;
	private: System::Windows::Forms::Label^  label90;
	private: System::Windows::Forms::Label^  label25;
	private: System::Windows::Forms::GroupBox^  group_focus;
	private: System::Windows::Forms::TextBox^  MOVIE_Retry_Box;
	private: System::Windows::Forms::TextBox^  MOVIE_Amplitude;
	private: System::Windows::Forms::Label^  label86;
	private: System::Windows::Forms::Label^  label85;
	private: System::Windows::Forms::Label^  label27;
	private: System::Windows::Forms::GroupBox^  groupBox15;
	private: System::Windows::Forms::TextBox^  MOVIE_skip_step_box;
	private: System::Windows::Forms::TextBox^  AF_area_box;
	private: System::Windows::Forms::TextBox^  BKarea_box;
	private: System::Windows::Forms::TextBox^  BK_box;
	private: System::Windows::Forms::Label^  label135;
	private: System::Windows::Forms::Label^  label156;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label71;
	private: System::Windows::Forms::Label^  label74;
	private: System::Windows::Forms::Button^  Reverse_but;
	private: System::Windows::Forms::Button^  gotocenter_but;
	private: System::Windows::Forms::Button^  Set_RV_but;
	private: System::Windows::Forms::Label^  label32;
	private: System::Windows::Forms::Label^  label19;
	private: System::Windows::Forms::TextBox^  s_pos_current;
	private: System::Windows::Forms::TextBox^  s_pos_target;
	private: System::Windows::Forms::Button^  s_left;
	private: System::Windows::Forms::Button^  s_right;
	private: System::Windows::Forms::Label^  label56;
	private: System::Windows::Forms::Button^  Theta_zero;
	private: System::Windows::Forms::TextBox^  delay_time_box;
	private: System::Windows::Forms::Label^  label23;
	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::TextBox^  windowsize_box;
	private: System::Windows::Forms::Label^  label17;
	private: System::ComponentModel::BackgroundWorker^  Theta_ini;
	private: System::Windows::Forms::Label^  label18;
	private: System::ComponentModel::BackgroundWorker^  BW_ETL_check;
	private: System::Windows::Forms::CheckBox^  MicroStep20;
	private: System::Windows::Forms::CheckBox^  MicroStep10;
	private: System::Windows::Forms::CheckBox^  MicroStep8;
	private: System::Windows::Forms::CheckBox^  MicroStep4;
	private: System::Windows::Forms::CheckBox^  MicroStep2;
	private: System::Windows::Forms::GroupBox^  groupBox14;
	private: System::Windows::Forms::CheckBox^  Camera_1_use;
	private: System::Windows::Forms::Label^  label44;
	private: System::Windows::Forms::Button^  Theta_Pos_Set;
	private: System::Windows::Forms::TextBox^  Theta_POS_Z;
	private: System::Windows::Forms::TextBox^  Theta_POS_X;
	private: System::Windows::Forms::Label^  label45;
	private: System::Windows::Forms::Label^  label46;
	private: System::Windows::Forms::Label^  exp;
	private: System::Windows::Forms::Button^  z_galv_set_rv;
	private: System::Windows::Forms::Label^  label48;
	private: System::Windows::Forms::TextBox^  Z_galv_L_RV;
	private: System::Windows::Forms::TextBox^  Z_galv_R_RV;
	private: System::Windows::Forms::Label^  label49;
	private: System::Windows::Forms::Button^  z_galv_set;
	private: System::Windows::Forms::CheckBox^  RV_Imaging_Check;
	private: System::Windows::Forms::TextBox^  End_Z_pos_RV;
	private: System::Windows::Forms::TextBox^  Start_Z_pos_RV;
	private: System::Windows::Forms::Button^  End_Z_but_RV;
	private: System::Windows::Forms::Button^  Start_Z_but_RV;
	private: System::Windows::Forms::TextBox^  ETL_box2;
	private: System::Windows::Forms::Label^  label50;
	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::TextBox^  Extra_time_box;
	private: System::Windows::Forms::Label^  label52;
	private: System::Windows::Forms::Label^  label51;
	private: System::Windows::Forms::TextBox^  Z_offset_current;
	private: System::Windows::Forms::Label^  label54;
	private: System::Windows::Forms::Label^  label53;
	private: System::Windows::Forms::TextBox^  DCT_calc_time;
	private: System::Windows::Forms::Label^  label55;
	private: System::Windows::Forms::TextBox^  ReverseSpos;
	private: System::Windows::Forms::CheckBox^  OneSideImaging;
	private: System::Windows::Forms::Label^  label57;
	private: System::Windows::Forms::TextBox^  SheetSPD;
	private: System::Windows::Forms::TrackBar^  SheetSPD_bar;
	private: System::Windows::Forms::TextBox^  LL;
	private: System::Windows::Forms::TextBox^  RR;
	private: System::Windows::Forms::Button^  X_LR;
	private: System::Windows::Forms::CheckBox^  SinglePlane;
	private: System::Windows::Forms::Button^  AxialAF_but;
	private: System::Windows::Forms::ToolStripStatusLabel^  ThreadCondition2;
	private: System::IO::Ports::SerialPort^  FW;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker_L592;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker_L488;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker_L647;
	private: System::Windows::Forms::Label^  L647_current_power;
	private: System::Windows::Forms::Label^  L592_current_power;
	private: System::Windows::Forms::Label^  L488_current_power;
	private: System::Windows::Forms::Label^  L647_temp;
	private: System::Windows::Forms::Label^  L592_temp;
	private: System::Windows::Forms::Label^  label59;
	private: System::Windows::Forms::Label^  L488_temp;
	private: System::Windows::Forms::Label^  Target_img;
	private: System::Windows::Forms::Label^  Acq_img;
	private: System::Windows::Forms::Label^  label61;
	private: System::Windows::Forms::Label^  label60;
	private: System::Windows::Forms::Label^  exp2;
	private: System::Windows::Forms::TextBox^  MOVIEfocus_calc_block;
	private: System::Windows::Forms::Label^  label47;
	private: System::Windows::Forms::Label^  label62;
	private: System::Windows::Forms::Button^  SkipTile;
	private: System::Windows::Forms::Button^  button5;
	private: System::Windows::Forms::Button^  button4;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Label^  label66;
	private: System::Windows::Forms::Label^  label65;
	private: System::Windows::Forms::Label^  label64;
	private: System::Windows::Forms::Label^  label63;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::Label^  label69;
	private: System::Windows::Forms::Label^  label68;
	private: System::Windows::Forms::TextBox^  AFnum;
	private: System::Windows::Forms::TextBox^  AFspan;
	private: System::Windows::Forms::Label^  label67;
	private: System::ComponentModel::BackgroundWorker^  Surveillance;
	private: System::Windows::Forms::CheckBox^  ManuaL_focus;
	private: System::Windows::Forms::TextBox^  manual_focus_box;
	private: System::Windows::Forms::TrackBar^  trackBar_manual_focus;
	private: System::Windows::Forms::GroupBox^  groupBox12;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker_XYstage;
			 /// <summary>
			 /// 必要なデザイナー変数です。
			 /// </summary>
#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナー サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディターで変更しないでください。
		/// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
				 this->Amplitude_sheet_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->Amplitude_sheet = (gcnew System::Windows::Forms::TextBox());
				 this->L488_ON = (gcnew System::Windows::Forms::Button());
				 this->L592_ON = (gcnew System::Windows::Forms::Button());
				 this->L647_ON = (gcnew System::Windows::Forms::Button());
				 this->Filter_1 = (gcnew System::Windows::Forms::Button());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->L488_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->L592_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->L647_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->L488_box = (gcnew System::Windows::Forms::TextBox());
				 this->L592_box = (gcnew System::Windows::Forms::TextBox());
				 this->L647_box = (gcnew System::Windows::Forms::TextBox());
				 this->Laser_488 = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Laser_647 = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Laser_592 = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->label5 = (gcnew System::Windows::Forms::Label());
				 this->label6 = (gcnew System::Windows::Forms::Label());
				 this->X_pos_current = (gcnew System::Windows::Forms::TextBox());
				 this->Y_pos_current = (gcnew System::Windows::Forms::TextBox());
				 this->Z_pos_current = (gcnew System::Windows::Forms::TextBox());
				 this->XYstage_port = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Zstage_port = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Z_target_pos = (gcnew System::Windows::Forms::TextBox());
				 this->backgroundWorker_Zstage = (gcnew System::ComponentModel::BackgroundWorker());
				 this->backgroundWorker_XYstage = (gcnew System::ComponentModel::BackgroundWorker());
				 this->CameraNo_1 = (gcnew System::Windows::Forms::TextBox());
				 this->label11 = (gcnew System::Windows::Forms::Label());
				 this->exposure_time_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->exposure_time_box = (gcnew System::Windows::Forms::TextBox());
				 this->up_1 = (gcnew System::Windows::Forms::Button());
				 this->down_1 = (gcnew System::Windows::Forms::Button());
				 this->right_1 = (gcnew System::Windows::Forms::Button());
				 this->Z_down = (gcnew System::Windows::Forms::Button());
				 this->Z_up = (gcnew System::Windows::Forms::Button());
				 this->Abort = (gcnew System::Windows::Forms::Button());
				 this->Laser_L = (gcnew System::Windows::Forms::Button());
				 this->Start_Z_but = (gcnew System::Windows::Forms::Button());
				 this->End_Z_but = (gcnew System::Windows::Forms::Button());
				 this->Start_Z_pos = (gcnew System::Windows::Forms::TextBox());
				 this->End_Z_pos = (gcnew System::Windows::Forms::TextBox());
				 this->label16 = (gcnew System::Windows::Forms::Label());
				 this->label20 = (gcnew System::Windows::Forms::Label());
				 this->label21 = (gcnew System::Windows::Forms::Label());
				 this->Step = (gcnew System::Windows::Forms::TextBox());
				 this->label24 = (gcnew System::Windows::Forms::Label());
				 this->ETL_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->label29 = (gcnew System::Windows::Forms::Label());
				 this->ETL_box = (gcnew System::Windows::Forms::TextBox());
				 this->ETL_L = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->ETL_R = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->LIVE_but = (gcnew System::Windows::Forms::Button());
				 this->close = (gcnew System::Windows::Forms::Button());
				 this->SIGMA_Theta = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Laser_R = (gcnew System::Windows::Forms::Button());
				 this->Stem_box1 = (gcnew System::Windows::Forms::TextBox());
				 this->Export_Box = (gcnew System::Windows::Forms::TextBox());
				 this->label88 = (gcnew System::Windows::Forms::Label());
				 this->label89 = (gcnew System::Windows::Forms::Label());
				 this->label91 = (gcnew System::Windows::Forms::Label());
				 this->Export_dir_open = (gcnew System::Windows::Forms::Button());
				 this->label92 = (gcnew System::Windows::Forms::Label());
				 this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
				 this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
				 this->label57 = (gcnew System::Windows::Forms::Label());
				 this->label18 = (gcnew System::Windows::Forms::Label());
				 this->SheetSPD = (gcnew System::Windows::Forms::TextBox());
				 this->Sheet_Box = (gcnew System::Windows::Forms::TextBox());
				 this->SheetSPD_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->Sheet_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
				 this->TargetX_box = (gcnew System::Windows::Forms::TextBox());
				 this->s_pos_current = (gcnew System::Windows::Forms::TextBox());
				 this->s_pos_target = (gcnew System::Windows::Forms::TextBox());
				 this->Theta_zero = (gcnew System::Windows::Forms::Button());
				 this->TargetY_box = (gcnew System::Windows::Forms::TextBox());
				 this->s_left = (gcnew System::Windows::Forms::Button());
				 this->left_1 = (gcnew System::Windows::Forms::Button());
				 this->s_right = (gcnew System::Windows::Forms::Button());
				 this->label152 = (gcnew System::Windows::Forms::Label());
				 this->label153 = (gcnew System::Windows::Forms::Label());
				 this->label56 = (gcnew System::Windows::Forms::Label());
				 this->label12 = (gcnew System::Windows::Forms::Label());
				 this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
				 this->L647_current_power = (gcnew System::Windows::Forms::Label());
				 this->L592_current_power = (gcnew System::Windows::Forms::Label());
				 this->L488_current_power = (gcnew System::Windows::Forms::Label());
				 this->LR_box = (gcnew System::Windows::Forms::TextBox());
				 this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
				 this->Laser_filter_setting_but = (gcnew System::Windows::Forms::Button());
				 this->EXP_time1 = (gcnew System::Windows::Forms::TextBox());
				 this->LP1 = (gcnew System::Windows::Forms::TextBox());
				 this->set4_name = (gcnew System::Windows::Forms::TextBox());
				 this->set3_name = (gcnew System::Windows::Forms::TextBox());
				 this->set2_name = (gcnew System::Windows::Forms::TextBox());
				 this->set1_name = (gcnew System::Windows::Forms::TextBox());
				 this->filter1_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->EXP_time4 = (gcnew System::Windows::Forms::TextBox());
				 this->Laser1_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->LP4 = (gcnew System::Windows::Forms::TextBox());
				 this->EXP_time3 = (gcnew System::Windows::Forms::TextBox());
				 this->LP3 = (gcnew System::Windows::Forms::TextBox());
				 this->EXP_time2 = (gcnew System::Windows::Forms::TextBox());
				 this->LP2 = (gcnew System::Windows::Forms::TextBox());
				 this->filter4_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->filter3_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->filter2_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->Laser4_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->Laser3_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->Laser2_comb = (gcnew System::Windows::Forms::ComboBox());
				 this->label115 = (gcnew System::Windows::Forms::Label());
				 this->label104 = (gcnew System::Windows::Forms::Label());
				 this->label94 = (gcnew System::Windows::Forms::Label());
				 this->label93 = (gcnew System::Windows::Forms::Label());
				 this->LF_check4 = (gcnew System::Windows::Forms::CheckBox());
				 this->LF_check3 = (gcnew System::Windows::Forms::CheckBox());
				 this->LF_check2 = (gcnew System::Windows::Forms::CheckBox());
				 this->multi_laser_check = (gcnew System::Windows::Forms::CheckBox());
				 this->LF_check1 = (gcnew System::Windows::Forms::CheckBox());
				 this->FIlter_2 = (gcnew System::Windows::Forms::Button());
				 this->FIlter_5 = (gcnew System::Windows::Forms::Button());
				 this->FIlter_6 = (gcnew System::Windows::Forms::Button());
				 this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
				 this->current_filter_box = (gcnew System::Windows::Forms::TextBox());
				 this->FIlter_8 = (gcnew System::Windows::Forms::Button());
				 this->FIlter_4 = (gcnew System::Windows::Forms::Button());
				 this->FIlter_7 = (gcnew System::Windows::Forms::Button());
				 this->FIlter_3 = (gcnew System::Windows::Forms::Button());
				 this->Single_Acquire = (gcnew System::Windows::Forms::Button());
				 this->Color_check = (gcnew System::ComponentModel::BackgroundWorker());
				 this->BW_XY_ini = (gcnew System::ComponentModel::BackgroundWorker());
				 this->Color_check2 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->BW_Q_Laser = (gcnew System::ComponentModel::BackgroundWorker());
				 this->BW_filter_check = (gcnew System::ComponentModel::BackgroundWorker());
				 this->MicroStep1 = (gcnew System::Windows::Forms::CheckBox());
				 this->statusbar = (gcnew System::Windows::Forms::StatusStrip());
				 this->toolStripProgressBar1 = (gcnew System::Windows::Forms::ToolStripProgressBar());
				 this->z_left = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->toolStripStatusLabel3 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->z_right = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->ThreadCondition = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->xpos = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->ypos = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->ThreadCondition2 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
				 this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
				 this->toolStripMenuItem8 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripSeparator2 = (gcnew System::Windows::Forms::ToolStripSeparator());
				 this->toolStripMenuItem9 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripMenuItem5 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripMenuItem6 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripMenuItem7 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->eTLAxialToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripMenuItem2 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->saveFileToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->toolStripMenuItem4 = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->resetSettingToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
				 this->BW_Cursor = (gcnew System::ComponentModel::BackgroundWorker());
				 this->Progress_count = (gcnew System::ComponentModel::BackgroundWorker());
				 this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
				 this->ETL_box2 = (gcnew System::Windows::Forms::TextBox());
				 this->label50 = (gcnew System::Windows::Forms::Label());
				 this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
				 this->label28 = (gcnew System::Windows::Forms::Label());
				 this->label54 = (gcnew System::Windows::Forms::Label());
				 this->label53 = (gcnew System::Windows::Forms::Label());
				 this->Z_offset_current = (gcnew System::Windows::Forms::TextBox());
				 this->Z_offset = (gcnew System::Windows::Forms::TextBox());
				 this->Axial_Param_set_R = (gcnew System::Windows::Forms::Button());
				 this->label33 = (gcnew System::Windows::Forms::Label());
				 this->label15 = (gcnew System::Windows::Forms::Label());
				 this->Axial_Param_set = (gcnew System::Windows::Forms::Button());
				 this->trackBar_Z = (gcnew System::Windows::Forms::TrackBar());
				 this->filter_ini = (gcnew System::ComponentModel::BackgroundWorker());
				 this->Laser_ini = (gcnew System::ComponentModel::BackgroundWorker());
				 this->BW_ActiveWindow = (gcnew System::ComponentModel::BackgroundWorker());
				 this->Camera_ini = (gcnew System::ComponentModel::BackgroundWorker());
				 this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
				 this->LIVE_window_close2 = (gcnew System::Windows::Forms::Button());
				 this->LIVE_window_close1 = (gcnew System::Windows::Forms::Button());
				 this->SH_low1_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->SH_low2_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->SH_high2_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->SH_high1_bar = (gcnew System::Windows::Forms::TrackBar());
				 this->SH_low2 = (gcnew System::Windows::Forms::TextBox());
				 this->SH_high2 = (gcnew System::Windows::Forms::TextBox());
				 this->SH_low1 = (gcnew System::Windows::Forms::TextBox());
				 this->SH_high1 = (gcnew System::Windows::Forms::TextBox());
				 this->manual_scale2 = (gcnew System::Windows::Forms::CheckBox());
				 this->manual_scale1 = (gcnew System::Windows::Forms::CheckBox());
				 this->ResizeWindow = (gcnew System::Windows::Forms::Button());
				 this->singlestackmode = (gcnew System::Windows::Forms::CheckBox());
				 this->imagingtile = (gcnew System::Windows::Forms::TextBox());
				 this->ReStart_tile_num_box = (gcnew System::Windows::Forms::TextBox());
				 this->MOVIE_strat = (gcnew System::Windows::Forms::Button());
				 this->nama = (gcnew System::Windows::Forms::Label());
				 this->label73 = (gcnew System::Windows::Forms::Label());
				 this->Axial_mode_check = (gcnew System::Windows::Forms::CheckBox());
				 this->aa = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage3 = (gcnew System::Windows::Forms::TabPage());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->z_galv_set_rv = (gcnew System::Windows::Forms::Button());
				 this->label48 = (gcnew System::Windows::Forms::Label());
				 this->Z_galv_L_RV = (gcnew System::Windows::Forms::TextBox());
				 this->Z_galv_R_RV = (gcnew System::Windows::Forms::TextBox());
				 this->label49 = (gcnew System::Windows::Forms::Label());
				 this->z_galv_set = (gcnew System::Windows::Forms::Button());
				 this->label30 = (gcnew System::Windows::Forms::Label());
				 this->Z_galv_L = (gcnew System::Windows::Forms::TextBox());
				 this->Z_galv_R = (gcnew System::Windows::Forms::TextBox());
				 this->label34 = (gcnew System::Windows::Forms::Label());
				 this->Axial_647_load = (gcnew System::Windows::Forms::Button());
				 this->groupBox16 = (gcnew System::Windows::Forms::GroupBox());
				 this->End_Z_pos_RV = (gcnew System::Windows::Forms::TextBox());
				 this->Start_Z_pos_RV = (gcnew System::Windows::Forms::TextBox());
				 this->End_Z_but_RV = (gcnew System::Windows::Forms::Button());
				 this->Start_Z_but_RV = (gcnew System::Windows::Forms::Button());
				 this->OneSideImaging = (gcnew System::Windows::Forms::CheckBox());
				 this->RV_Imaging_Check = (gcnew System::Windows::Forms::CheckBox());
				 this->SinglePlane = (gcnew System::Windows::Forms::CheckBox());
				 this->label39 = (gcnew System::Windows::Forms::Label());
				 this->bottom_limit = (gcnew System::Windows::Forms::TextBox());
				 this->right_limit = (gcnew System::Windows::Forms::TextBox());
				 this->left_limit = (gcnew System::Windows::Forms::TextBox());
				 this->top_limit = (gcnew System::Windows::Forms::TextBox());
				 this->left_limit_but = (gcnew System::Windows::Forms::Button());
				 this->Y_limit_but = (gcnew System::Windows::Forms::Button());
				 this->Theta_Pos_Set = (gcnew System::Windows::Forms::Button());
				 this->Reverse_but = (gcnew System::Windows::Forms::Button());
				 this->gotocenter_but = (gcnew System::Windows::Forms::Button());
				 this->Set_RV_but = (gcnew System::Windows::Forms::Button());
				 this->right_limit_but = (gcnew System::Windows::Forms::Button());
				 this->top_limit_but = (gcnew System::Windows::Forms::Button());
				 this->Theta_POS_Z = (gcnew System::Windows::Forms::TextBox());
				 this->ReverseSpos = (gcnew System::Windows::Forms::TextBox());
				 this->bottom_limit_RV = (gcnew System::Windows::Forms::TextBox());
				 this->left_limit_RV = (gcnew System::Windows::Forms::TextBox());
				 this->Theta_POS_X = (gcnew System::Windows::Forms::TextBox());
				 this->right_limit_RV = (gcnew System::Windows::Forms::TextBox());
				 this->Target_img = (gcnew System::Windows::Forms::Label());
				 this->Acq_img = (gcnew System::Windows::Forms::Label());
				 this->label61 = (gcnew System::Windows::Forms::Label());
				 this->label60 = (gcnew System::Windows::Forms::Label());
				 this->exp2 = (gcnew System::Windows::Forms::Label());
				 this->exp = (gcnew System::Windows::Forms::Label());
				 this->top_limit_RV = (gcnew System::Windows::Forms::TextBox());
				 this->label32 = (gcnew System::Windows::Forms::Label());
				 this->label45 = (gcnew System::Windows::Forms::Label());
				 this->label44 = (gcnew System::Windows::Forms::Label());
				 this->label19 = (gcnew System::Windows::Forms::Label());
				 this->label31 = (gcnew System::Windows::Forms::Label());
				 this->label46 = (gcnew System::Windows::Forms::Label());
				 this->AxialAF_but = (gcnew System::Windows::Forms::Button());
				 this->Axial_592_load = (gcnew System::Windows::Forms::Button());
				 this->ETL_R_right = (gcnew System::Windows::Forms::TextBox());
				 this->Axial_panel = (gcnew System::Windows::Forms::TextBox());
				 this->Axial_488_load = (gcnew System::Windows::Forms::Button());
				 this->ETL_L_right = (gcnew System::Windows::Forms::TextBox());
				 this->ETL_R_left = (gcnew System::Windows::Forms::TextBox());
				 this->ETL_L_left = (gcnew System::Windows::Forms::TextBox());
				 this->label10 = (gcnew System::Windows::Forms::Label());
				 this->label26 = (gcnew System::Windows::Forms::Label());
				 this->label13 = (gcnew System::Windows::Forms::Label());
				 this->label9 = (gcnew System::Windows::Forms::Label());
				 this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
				 this->ManuaL_focus = (gcnew System::Windows::Forms::CheckBox());
				 this->label69 = (gcnew System::Windows::Forms::Label());
				 this->label68 = (gcnew System::Windows::Forms::Label());
				 this->AFnum = (gcnew System::Windows::Forms::TextBox());
				 this->AFspan = (gcnew System::Windows::Forms::TextBox());
				 this->SkipTile = (gcnew System::Windows::Forms::Button());
				 this->windowsize_box = (gcnew System::Windows::Forms::TextBox());
				 this->label67 = (gcnew System::Windows::Forms::Label());
				 this->Extra_time_box = (gcnew System::Windows::Forms::TextBox());
				 this->manual_focus_box = (gcnew System::Windows::Forms::TextBox());
				 this->delay_time_box = (gcnew System::Windows::Forms::TextBox());
				 this->Overlap_um_y = (gcnew System::Windows::Forms::TextBox());
				 this->Overlap_um = (gcnew System::Windows::Forms::TextBox());
				 this->label41 = (gcnew System::Windows::Forms::Label());
				 this->label40 = (gcnew System::Windows::Forms::Label());
				 this->trackBar_manual_focus = (gcnew System::Windows::Forms::TrackBar());
				 this->label43 = (gcnew System::Windows::Forms::Label());
				 this->label42 = (gcnew System::Windows::Forms::Label());
				 this->label38 = (gcnew System::Windows::Forms::Label());
				 this->Z_overlap_box = (gcnew System::Windows::Forms::TextBox());
				 this->label17 = (gcnew System::Windows::Forms::Label());
				 this->label52 = (gcnew System::Windows::Forms::Label());
				 this->label51 = (gcnew System::Windows::Forms::Label());
				 this->label23 = (gcnew System::Windows::Forms::Label());
				 this->label22 = (gcnew System::Windows::Forms::Label());
				 this->label90 = (gcnew System::Windows::Forms::Label());
				 this->label25 = (gcnew System::Windows::Forms::Label());
				 this->group_focus = (gcnew System::Windows::Forms::GroupBox());
				 this->MOVIEfocus_calc_block = (gcnew System::Windows::Forms::TextBox());
				 this->MOVIE_Retry_Box = (gcnew System::Windows::Forms::TextBox());
				 this->MOVIE_Amplitude = (gcnew System::Windows::Forms::TextBox());
				 this->label47 = (gcnew System::Windows::Forms::Label());
				 this->label86 = (gcnew System::Windows::Forms::Label());
				 this->label85 = (gcnew System::Windows::Forms::Label());
				 this->label62 = (gcnew System::Windows::Forms::Label());
				 this->label27 = (gcnew System::Windows::Forms::Label());
				 this->groupBox15 = (gcnew System::Windows::Forms::GroupBox());
				 this->MOVIE_skip_step_box = (gcnew System::Windows::Forms::TextBox());
				 this->AF_area_box = (gcnew System::Windows::Forms::TextBox());
				 this->BKarea_box = (gcnew System::Windows::Forms::TextBox());
				 this->BK_box = (gcnew System::Windows::Forms::TextBox());
				 this->label135 = (gcnew System::Windows::Forms::Label());
				 this->label156 = (gcnew System::Windows::Forms::Label());
				 this->label8 = (gcnew System::Windows::Forms::Label());
				 this->label71 = (gcnew System::Windows::Forms::Label());
				 this->label74 = (gcnew System::Windows::Forms::Label());
				 this->Camera = (gcnew System::Windows::Forms::TabPage());
				 this->Down2Top = (gcnew System::Windows::Forms::CheckBox());
				 this->Top2Down = (gcnew System::Windows::Forms::CheckBox());
				 this->Internal = (gcnew System::Windows::Forms::CheckBox());
				 this->ExternalExposure = (gcnew System::Windows::Forms::CheckBox());
				 this->FastReadOut = (gcnew System::Windows::Forms::CheckBox());
				 this->ScanSpeedControl_Check = (gcnew System::Windows::Forms::CheckBox());
				 this->OverlapMode = (gcnew System::Windows::Forms::CheckBox());
				 this->External = (gcnew System::Windows::Forms::CheckBox());
				 this->C2OUT = (gcnew System::Windows::Forms::CheckBox());
				 this->label142 = (gcnew System::Windows::Forms::Label());
				 this->label141 = (gcnew System::Windows::Forms::Label());
				 this->label140 = (gcnew System::Windows::Forms::Label());
				 this->Get_Camera_Condition = (gcnew System::Windows::Forms::Button());
				 this->camera_set = (gcnew System::Windows::Forms::Button());
				 this->label143 = (gcnew System::Windows::Forms::Label());
				 this->label148 = (gcnew System::Windows::Forms::Label());
				 this->label150 = (gcnew System::Windows::Forms::Label());
				 this->label149 = (gcnew System::Windows::Forms::Label());
				 this->label147 = (gcnew System::Windows::Forms::Label());
				 this->label146 = (gcnew System::Windows::Forms::Label());
				 this->label145 = (gcnew System::Windows::Forms::Label());
				 this->label144 = (gcnew System::Windows::Forms::Label());
				 this->label138 = (gcnew System::Windows::Forms::Label());
				 this->ScanSpeed = (gcnew System::Windows::Forms::TextBox());
				 this->PixelReadoutRate_box = (gcnew System::Windows::Forms::TextBox());
				 this->Scanspeedcontrol_box = (gcnew System::Windows::Forms::TextBox());
				 this->exposuretime_box = (gcnew System::Windows::Forms::TextBox());
				 this->overlapmode_box = (gcnew System::Windows::Forms::TextBox());
				 this->ScanSpeed_box = (gcnew System::Windows::Forms::TextBox());
				 this->Shuttermode_box = (gcnew System::Windows::Forms::TextBox());
				 this->Triggermode_box = (gcnew System::Windows::Forms::TextBox());
				 this->Windowsize = (gcnew System::Windows::Forms::TextBox());
				 this->Calibration = (gcnew System::Windows::Forms::TabPage());
				 this->L647_temp = (gcnew System::Windows::Forms::Label());
				 this->groupBox12 = (gcnew System::Windows::Forms::GroupBox());
				 this->button2 = (gcnew System::Windows::Forms::Button());
				 this->textBox1 = (gcnew System::Windows::Forms::TextBox());
				 this->button5 = (gcnew System::Windows::Forms::Button());
				 this->textBox2 = (gcnew System::Windows::Forms::TextBox());
				 this->button4 = (gcnew System::Windows::Forms::Button());
				 this->textBox3 = (gcnew System::Windows::Forms::TextBox());
				 this->textBox4 = (gcnew System::Windows::Forms::TextBox());
				 this->button3 = (gcnew System::Windows::Forms::Button());
				 this->label63 = (gcnew System::Windows::Forms::Label());
				 this->label64 = (gcnew System::Windows::Forms::Label());
				 this->label65 = (gcnew System::Windows::Forms::Label());
				 this->label66 = (gcnew System::Windows::Forms::Label());
				 this->L592_temp = (gcnew System::Windows::Forms::Label());
				 this->groupBox13 = (gcnew System::Windows::Forms::GroupBox());
				 this->sheet_offset_R = (gcnew System::Windows::Forms::TextBox());
				 this->theta_offset = (gcnew System::Windows::Forms::TextBox());
				 this->sheet_offset_L = (gcnew System::Windows::Forms::TextBox());
				 this->label118 = (gcnew System::Windows::Forms::Label());
				 this->label83 = (gcnew System::Windows::Forms::Label());
				 this->label82 = (gcnew System::Windows::Forms::Label());
				 this->sheet = (gcnew System::Windows::Forms::Label());
				 this->LL = (gcnew System::Windows::Forms::TextBox());
				 this->label59 = (gcnew System::Windows::Forms::Label());
				 this->L488_temp = (gcnew System::Windows::Forms::Label());
				 this->RR = (gcnew System::Windows::Forms::TextBox());
				 this->groupBox11 = (gcnew System::Windows::Forms::GroupBox());
				 this->filter_name8 = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name4 = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name7 = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name3 = (gcnew System::Windows::Forms::TextBox());
				 this->Filter_wheel_SPD = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name6 = (gcnew System::Windows::Forms::TextBox());
				 this->Theta_SIGMA = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name2 = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name5 = (gcnew System::Windows::Forms::TextBox());
				 this->filter_name1 = (gcnew System::Windows::Forms::TextBox());
				 this->label4 = (gcnew System::Windows::Forms::Label());
				 this->label81 = (gcnew System::Windows::Forms::Label());
				 this->label103 = (gcnew System::Windows::Forms::Label());
				 this->Theta_SIGMA_but = (gcnew System::Windows::Forms::Button());
				 this->label99 = (gcnew System::Windows::Forms::Label());
				 this->label102 = (gcnew System::Windows::Forms::Label());
				 this->label98 = (gcnew System::Windows::Forms::Label());
				 this->label101 = (gcnew System::Windows::Forms::Label());
				 this->label97 = (gcnew System::Windows::Forms::Label());
				 this->label100 = (gcnew System::Windows::Forms::Label());
				 this->label96 = (gcnew System::Windows::Forms::Label());
				 this->Sheet_Step = (gcnew System::Windows::Forms::Button());
				 this->ETL_swept_time = (gcnew System::Windows::Forms::TextBox());
				 this->sweep_max = (gcnew System::Windows::Forms::TextBox());
				 this->sheet3 = (gcnew System::Windows::Forms::TextBox());
				 this->sheet2 = (gcnew System::Windows::Forms::TextBox());
				 this->sheet1 = (gcnew System::Windows::Forms::TextBox());
				 this->sweep_min = (gcnew System::Windows::Forms::TextBox());
				 this->X_LR = (gcnew System::Windows::Forms::Button());
				 this->groupBox14 = (gcnew System::Windows::Forms::GroupBox());
				 this->MicroStep2 = (gcnew System::Windows::Forms::CheckBox());
				 this->MicroStep4 = (gcnew System::Windows::Forms::CheckBox());
				 this->MicroStep8 = (gcnew System::Windows::Forms::CheckBox());
				 this->MicroStep10 = (gcnew System::Windows::Forms::CheckBox());
				 this->MicroStep20 = (gcnew System::Windows::Forms::CheckBox());
				 this->z_zero = (gcnew System::Windows::Forms::Button());
				 this->Zweep = (gcnew System::Windows::Forms::Button());
				 this->ETL_Sweep = (gcnew System::Windows::Forms::Button());
				 this->Galvo_Zero = (gcnew System::Windows::Forms::Button());
				 this->label14 = (gcnew System::Windows::Forms::Label());
				 this->label35 = (gcnew System::Windows::Forms::Label());
				 this->MFocus_right = (gcnew System::Windows::Forms::TextBox());
				 this->MFocus_left = (gcnew System::Windows::Forms::TextBox());
				 this->label109 = (gcnew System::Windows::Forms::Label());
				 this->Xpos_cursol = (gcnew System::Windows::Forms::TextBox());
				 this->label112 = (gcnew System::Windows::Forms::Label());
				 this->Ypos_cursol = (gcnew System::Windows::Forms::TextBox());
				 this->label116 = (gcnew System::Windows::Forms::Label());
				 this->Cursol_int = (gcnew System::Windows::Forms::TextBox());
				 this->ma = (gcnew System::Windows::Forms::Label());
				 this->qw = (gcnew System::Windows::Forms::Label());
				 this->ImageMax = (gcnew System::Windows::Forms::TextBox());
				 this->ImageMin = (gcnew System::Windows::Forms::TextBox());
				 this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
				 this->DCTFocusValBox = (gcnew System::Windows::Forms::TextBox());
				 this->DCTScoreBox = (gcnew System::Windows::Forms::TextBox());
				 this->DCT_calc_time = (gcnew System::Windows::Forms::TextBox());
				 this->label84 = (gcnew System::Windows::Forms::Label());
				 this->label55 = (gcnew System::Windows::Forms::Label());
				 this->label113 = (gcnew System::Windows::Forms::Label());
				 this->ExpoTime_Axial = (gcnew System::Windows::Forms::TextBox());
				 this->label117 = (gcnew System::Windows::Forms::Label());
				 this->groupBox10 = (gcnew System::Windows::Forms::GroupBox());
				 this->CameraNo_2 = (gcnew System::Windows::Forms::TextBox());
				 this->Camera_1_use = (gcnew System::Windows::Forms::CheckBox());
				 this->Camera_2_use = (gcnew System::Windows::Forms::CheckBox());
				 this->Camera_ini2 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->backgroundWorker1 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Theta_ini = (gcnew System::ComponentModel::BackgroundWorker());
				 this->BW_ETL_check = (gcnew System::ComponentModel::BackgroundWorker());
				 this->FW = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->backgroundWorker_L592 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->backgroundWorker_L488 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->backgroundWorker_L647 = (gcnew System::ComponentModel::BackgroundWorker());
				 this->Surveillance = (gcnew System::ComponentModel::BackgroundWorker());
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Amplitude_sheet_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L488_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L592_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L647_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->exposure_time_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ETL_bar))->BeginInit();
				 this->groupBox2->SuspendLayout();
				 this->groupBox3->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SheetSPD_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Sheet_bar))->BeginInit();
				 this->groupBox4->SuspendLayout();
				 this->groupBox5->SuspendLayout();
				 this->groupBox6->SuspendLayout();
				 this->groupBox7->SuspendLayout();
				 this->statusbar->SuspendLayout();
				 this->menuStrip1->SuspendLayout();
				 this->groupBox8->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar_Z))->BeginInit();
				 this->tabPage2->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_low1_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_low2_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_high2_bar))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_high1_bar))->BeginInit();
				 this->aa->SuspendLayout();
				 this->tabPage3->SuspendLayout();
				 this->groupBox1->SuspendLayout();
				 this->groupBox16->SuspendLayout();
				 this->tabPage1->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar_manual_focus))->BeginInit();
				 this->group_focus->SuspendLayout();
				 this->groupBox15->SuspendLayout();
				 this->Camera->SuspendLayout();
				 this->Calibration->SuspendLayout();
				 this->groupBox12->SuspendLayout();
				 this->groupBox13->SuspendLayout();
				 this->groupBox11->SuspendLayout();
				 this->groupBox14->SuspendLayout();
				 this->groupBox9->SuspendLayout();
				 this->groupBox10->SuspendLayout();
				 this->SuspendLayout();
				 // 
				 // Amplitude_sheet_bar
				 // 
				 this->Amplitude_sheet_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->Amplitude_sheet_bar->LargeChange = 1;
				 this->Amplitude_sheet_bar->Location = System::Drawing::Point(17, 32);
				 this->Amplitude_sheet_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Amplitude_sheet_bar->Maximum = 100;
				 this->Amplitude_sheet_bar->Name = L"Amplitude_sheet_bar";
				 this->Amplitude_sheet_bar->Size = System::Drawing::Size(165, 69);
				 this->Amplitude_sheet_bar->TabIndex = 2;
				 this->Amplitude_sheet_bar->TickFrequency = 10;
				 this->Amplitude_sheet_bar->Value = 20;
				 this->Amplitude_sheet_bar->Scroll += gcnew System::EventHandler(this, &MyForm::Amplitude_sheet_bar_Scroll);
				 // 
				 // Amplitude_sheet
				 // 
				 this->Amplitude_sheet->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Amplitude_sheet->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Amplitude_sheet->ForeColor = System::Drawing::Color::Black;
				 this->Amplitude_sheet->Location = System::Drawing::Point(191, 32);
				 this->Amplitude_sheet->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Amplitude_sheet->Name = L"Amplitude_sheet";
				 this->Amplitude_sheet->ReadOnly = true;
				 this->Amplitude_sheet->Size = System::Drawing::Size(40, 27);
				 this->Amplitude_sheet->TabIndex = 17;
				 this->Amplitude_sheet->Text = L"2";
				 this->Amplitude_sheet->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Amplitude_sheet->TextChanged += gcnew System::EventHandler(this, &MyForm::Amplitude_sheet_TextChanged);
				 // 
				 // L488_ON
				 // 
				 this->L488_ON->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L488_ON->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->L488_ON->ForeColor = System::Drawing::Color::Black;
				 this->L488_ON->Location = System::Drawing::Point(72, 17);
				 this->L488_ON->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L488_ON->Name = L"L488_ON";
				 this->L488_ON->Size = System::Drawing::Size(61, 25);
				 this->L488_ON->TabIndex = 1;
				 this->L488_ON->Text = L"OFF";
				 this->L488_ON->UseVisualStyleBackColor = false;
				 this->L488_ON->Click += gcnew System::EventHandler(this, &MyForm::L488_ON_Click);
				 // 
				 // L592_ON
				 // 
				 this->L592_ON->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L592_ON->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->L592_ON->ForeColor = System::Drawing::Color::Black;
				 this->L592_ON->Location = System::Drawing::Point(72, 96);
				 this->L592_ON->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L592_ON->Name = L"L592_ON";
				 this->L592_ON->Size = System::Drawing::Size(61, 25);
				 this->L592_ON->TabIndex = 1;
				 this->L592_ON->Text = L"OFF";
				 this->L592_ON->UseVisualStyleBackColor = false;
				 this->L592_ON->Click += gcnew System::EventHandler(this, &MyForm::L592_ON_Click);
				 // 
				 // L647_ON
				 // 
				 this->L647_ON->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L647_ON->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->L647_ON->ForeColor = System::Drawing::Color::Black;
				 this->L647_ON->Location = System::Drawing::Point(72, 180);
				 this->L647_ON->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L647_ON->Name = L"L647_ON";
				 this->L647_ON->Size = System::Drawing::Size(61, 25);
				 this->L647_ON->TabIndex = 1;
				 this->L647_ON->Text = L"OFF";
				 this->L647_ON->UseVisualStyleBackColor = false;
				 this->L647_ON->Click += gcnew System::EventHandler(this, &MyForm::L647_ON_Click);
				 // 
				 // Filter_1
				 // 
				 this->Filter_1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Filter_1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Filter_1->ForeColor = System::Drawing::Color::Black;
				 this->Filter_1->Location = System::Drawing::Point(11, 17);
				 this->Filter_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Filter_1->Name = L"Filter_1";
				 this->Filter_1->Size = System::Drawing::Size(87, 25);
				 this->Filter_1->TabIndex = 1;
				 this->Filter_1->UseVisualStyleBackColor = false;
				 this->Filter_1->Click += gcnew System::EventHandler(this, &MyForm::Filter_1_Click);
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 11, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label1->ForeColor = System::Drawing::Color::Blue;
				 this->label1->Location = System::Drawing::Point(11, 17);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(57, 28);
				 this->label1->TabIndex = 3;
				 this->label1->Text = L"488";
				 // 
				 // label2
				 // 
				 this->label2->AutoSize = true;
				 this->label2->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 11, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label2->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					 static_cast<System::Int32>(static_cast<System::Byte>(0)));
				 this->label2->Location = System::Drawing::Point(11, 96);
				 this->label2->Name = L"label2";
				 this->label2->Size = System::Drawing::Size(57, 28);
				 this->label2->TabIndex = 3;
				 this->label2->Text = L"592";
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 11, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label3->ForeColor = System::Drawing::Color::Red;
				 this->label3->Location = System::Drawing::Point(11, 180);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(57, 28);
				 this->label3->TabIndex = 3;
				 this->label3->Text = L"647";
				 // 
				 // L488_bar
				 // 
				 this->L488_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->L488_bar->Location = System::Drawing::Point(9, 46);
				 this->L488_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L488_bar->Maximum = 200;
				 this->L488_bar->Minimum = 20;
				 this->L488_bar->Name = L"L488_bar";
				 this->L488_bar->Size = System::Drawing::Size(228, 69);
				 this->L488_bar->TabIndex = 2;
				 this->L488_bar->TickFrequency = 20;
				 this->L488_bar->Value = 20;
				 this->L488_bar->Scroll += gcnew System::EventHandler(this, &MyForm::L488_bar_Scroll);
				 // 
				 // L592_bar
				 // 
				 this->L592_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->L592_bar->Location = System::Drawing::Point(14, 132);
				 this->L592_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L592_bar->Maximum = 500;
				 this->L592_bar->Minimum = 50;
				 this->L592_bar->Name = L"L592_bar";
				 this->L592_bar->Size = System::Drawing::Size(221, 69);
				 this->L592_bar->TabIndex = 2;
				 this->L592_bar->TickFrequency = 50;
				 this->L592_bar->Value = 50;
				 this->L592_bar->Scroll += gcnew System::EventHandler(this, &MyForm::L592_bar_Scroll);
				 this->L592_bar->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::L592_bar_MouseUp);
				 // 
				 // L647_bar
				 // 
				 this->L647_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->L647_bar->Location = System::Drawing::Point(16, 210);
				 this->L647_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L647_bar->Maximum = 140;
				 this->L647_bar->Minimum = 15;
				 this->L647_bar->Name = L"L647_bar";
				 this->L647_bar->Size = System::Drawing::Size(221, 69);
				 this->L647_bar->TabIndex = 2;
				 this->L647_bar->TickFrequency = 10;
				 this->L647_bar->Value = 15;
				 this->L647_bar->Scroll += gcnew System::EventHandler(this, &MyForm::L647_bar_Scroll);
				 // 
				 // L488_box
				 // 
				 this->L488_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L488_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L488_box->ForeColor = System::Drawing::Color::Black;
				 this->L488_box->Location = System::Drawing::Point(139, 20);
				 this->L488_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L488_box->Name = L"L488_box";
				 this->L488_box->ReadOnly = true;
				 this->L488_box->Size = System::Drawing::Size(40, 27);
				 this->L488_box->TabIndex = 8;
				 this->L488_box->Text = L"20";
				 this->L488_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 // 
				 // L592_box
				 // 
				 this->L592_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L592_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L592_box->ForeColor = System::Drawing::Color::Black;
				 this->L592_box->Location = System::Drawing::Point(139, 99);
				 this->L592_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L592_box->Name = L"L592_box";
				 this->L592_box->ReadOnly = true;
				 this->L592_box->Size = System::Drawing::Size(40, 27);
				 this->L592_box->TabIndex = 9;
				 this->L592_box->Text = L"50";
				 this->L592_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 this->L592_box->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::L592_box_MouseClick);
				 // 
				 // L647_box
				 // 
				 this->L647_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->L647_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L647_box->ForeColor = System::Drawing::Color::Black;
				 this->L647_box->Location = System::Drawing::Point(139, 183);
				 this->L647_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->L647_box->Name = L"L647_box";
				 this->L647_box->ReadOnly = true;
				 this->L647_box->Size = System::Drawing::Size(40, 27);
				 this->L647_box->TabIndex = 10;
				 this->L647_box->Text = L"15";
				 this->L647_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 // 
				 // Laser_488
				 // 
				 this->Laser_488->BaudRate = 115200;
				 this->Laser_488->PortName = L"COM9";
				 this->Laser_488->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::Laser_488_DataReceived);
				 // 
				 // Laser_647
				 // 
				 this->Laser_647->BaudRate = 115200;
				 this->Laser_647->PortName = L"COM4";
				 this->Laser_647->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::Laser_647_DataReceived);
				 // 
				 // Laser_592
				 // 
				 this->Laser_592->PortName = L"COM2";
				 this->Laser_592->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::Laser_592_DataReceived);
				 // 
				 // label5
				 // 
				 this->label5->AutoSize = true;
				 this->label5->BackColor = System::Drawing::Color::Transparent;
				 this->label5->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label5->ForeColor = System::Drawing::Color::White;
				 this->label5->Location = System::Drawing::Point(5, 43);
				 this->label5->Name = L"label5";
				 this->label5->Size = System::Drawing::Size(27, 24);
				 this->label5->TabIndex = 3;
				 this->label5->Text = L"X:";
				 this->label5->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label6
				 // 
				 this->label6->AutoSize = true;
				 this->label6->BackColor = System::Drawing::Color::Transparent;
				 this->label6->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label6->ForeColor = System::Drawing::Color::White;
				 this->label6->Location = System::Drawing::Point(5, 70);
				 this->label6->Name = L"label6";
				 this->label6->Size = System::Drawing::Size(25, 24);
				 this->label6->TabIndex = 3;
				 this->label6->Text = L"Y:";
				 // 
				 // X_pos_current
				 // 
				 this->X_pos_current->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->X_pos_current->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->X_pos_current->ForeColor = System::Drawing::Color::Black;
				 this->X_pos_current->Location = System::Drawing::Point(73, 40);
				 this->X_pos_current->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->X_pos_current->Name = L"X_pos_current";
				 this->X_pos_current->Size = System::Drawing::Size(48, 31);
				 this->X_pos_current->TabIndex = 1;
				 this->X_pos_current->Text = L"0";
				 this->X_pos_current->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Y_pos_current
				 // 
				 this->Y_pos_current->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Y_pos_current->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Y_pos_current->ForeColor = System::Drawing::Color::Black;
				 this->Y_pos_current->Location = System::Drawing::Point(73, 142);
				 this->Y_pos_current->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Y_pos_current->Name = L"Y_pos_current";
				 this->Y_pos_current->Size = System::Drawing::Size(48, 31);
				 this->Y_pos_current->TabIndex = 1;
				 this->Y_pos_current->Text = L"0";
				 this->Y_pos_current->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Z_pos_current
				 // 
				 this->Z_pos_current->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Z_pos_current->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_pos_current->ForeColor = System::Drawing::Color::Black;
				 this->Z_pos_current->Location = System::Drawing::Point(73, 65);
				 this->Z_pos_current->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_pos_current->Name = L"Z_pos_current";
				 this->Z_pos_current->ReadOnly = true;
				 this->Z_pos_current->Size = System::Drawing::Size(48, 31);
				 this->Z_pos_current->TabIndex = 2;
				 this->Z_pos_current->Text = L"0";
				 this->Z_pos_current->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // XYstage_port
				 // 
				 this->XYstage_port->BaudRate = 38400;
				 this->XYstage_port->PortName = L"COM7";
				 this->XYstage_port->ReadBufferSize = 1024;
				 this->XYstage_port->ReadTimeout = 20;
				 this->XYstage_port->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::XYstage_port_DataReceived);
				 // 
				 // Zstage_port
				 // 
				 this->Zstage_port->BaudRate = 19200;
				 this->Zstage_port->PortName = L"COM8";
				 this->Zstage_port->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::Zstage_port_DataReceived);
				 // 
				 // Z_target_pos
				 // 
				 this->Z_target_pos->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Z_target_pos->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_target_pos->ForeColor = System::Drawing::Color::Black;
				 this->Z_target_pos->Location = System::Drawing::Point(22, 65);
				 this->Z_target_pos->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_target_pos->Name = L"Z_target_pos";
				 this->Z_target_pos->Size = System::Drawing::Size(49, 31);
				 this->Z_target_pos->TabIndex = 3;
				 this->Z_target_pos->Text = L"0";
				 this->Z_target_pos->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_target_pos->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_target_pos_TextChanged);
				 this->Z_target_pos->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Z_target_pos_KeyDown);
				 this->Z_target_pos->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &MyForm::Z_target_pos_KeyPress);
				 // 
				 // backgroundWorker_Zstage
				 // 
				 this->backgroundWorker_Zstage->WorkerSupportsCancellation = true;
				 this->backgroundWorker_Zstage->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker_Zstage_DoWork);
				 // 
				 // backgroundWorker_XYstage
				 // 
				 this->backgroundWorker_XYstage->WorkerSupportsCancellation = true;
				 this->backgroundWorker_XYstage->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker_XYstage_DoWork);
				 // 
				 // CameraNo_1
				 // 
				 this->CameraNo_1->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->CameraNo_1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->CameraNo_1->ForeColor = System::Drawing::Color::Black;
				 this->CameraNo_1->Location = System::Drawing::Point(150, 10);
				 this->CameraNo_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->CameraNo_1->Name = L"CameraNo_1";
				 this->CameraNo_1->ReadOnly = true;
				 this->CameraNo_1->Size = System::Drawing::Size(116, 31);
				 this->CameraNo_1->TabIndex = 0;
				 // 
				 // label11
				 // 
				 this->label11->AutoSize = true;
				 this->label11->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label11->ForeColor = System::Drawing::Color::White;
				 this->label11->Location = System::Drawing::Point(6, 22);
				 this->label11->Name = L"label11";
				 this->label11->Size = System::Drawing::Size(154, 24);
				 this->label11->TabIndex = 3;
				 this->label11->Text = L"Sheet amplitude:";
				 // 
				 // exposure_time_bar
				 // 
				 this->exposure_time_bar->Location = System::Drawing::Point(764, 186);
				 this->exposure_time_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->exposure_time_bar->Maximum = 500;
				 this->exposure_time_bar->Minimum = 15;
				 this->exposure_time_bar->Name = L"exposure_time_bar";
				 this->exposure_time_bar->Size = System::Drawing::Size(254, 69);
				 this->exposure_time_bar->TabIndex = 2;
				 this->exposure_time_bar->TickFrequency = 100;
				 this->exposure_time_bar->Value = 50;
				 this->exposure_time_bar->Scroll += gcnew System::EventHandler(this, &MyForm::exposure_time_bar_Scroll);
				 // 
				 // exposure_time_box
				 // 
				 this->exposure_time_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->exposure_time_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->exposure_time_box->ForeColor = System::Drawing::Color::Black;
				 this->exposure_time_box->Location = System::Drawing::Point(153, 132);
				 this->exposure_time_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->exposure_time_box->Name = L"exposure_time_box";
				 this->exposure_time_box->Size = System::Drawing::Size(54, 27);
				 this->exposure_time_box->TabIndex = 13;
				 this->exposure_time_box->Text = L"50";
				 this->exposure_time_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->exposure_time_box->TextChanged += gcnew System::EventHandler(this, &MyForm::exposure_time_box_TextChanged);
				 this->exposure_time_box->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::exposure_time_box_KeyDown);
				 // 
				 // up_1
				 // 
				 this->up_1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->up_1->FlatAppearance->BorderSize = 0;
				 this->up_1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->up_1->ForeColor = System::Drawing::Color::Black;
				 this->up_1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"up_1.Image")));
				 this->up_1->Location = System::Drawing::Point(163, 12);
				 this->up_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->up_1->Name = L"up_1";
				 this->up_1->Size = System::Drawing::Size(43, 35);
				 this->up_1->TabIndex = 0;
				 this->up_1->TextAlign = System::Drawing::ContentAlignment::TopCenter;
				 this->up_1->UseVisualStyleBackColor = false;
				 this->up_1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::Z_up_MouseDown);
				 this->up_1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::Z_up_MouseUp);
				 // 
				 // down_1
				 // 
				 this->down_1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->down_1->Font = (gcnew System::Drawing::Font(L"Calibri", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->down_1->ForeColor = System::Drawing::Color::Black;
				 this->down_1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"down_1.Image")));
				 this->down_1->Location = System::Drawing::Point(162, 84);
				 this->down_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->down_1->Name = L"down_1";
				 this->down_1->Size = System::Drawing::Size(44, 36);
				 this->down_1->TabIndex = 0;
				 this->down_1->UseVisualStyleBackColor = false;
				 this->down_1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::Z_down_MouseDown);
				 this->down_1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::Z_down_MouseUp);
				 // 
				 // right_1
				 // 
				 this->right_1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->right_1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->right_1->ForeColor = System::Drawing::Color::Black;
				 this->right_1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"right_1.Image")));
				 this->right_1->Location = System::Drawing::Point(204, 47);
				 this->right_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->right_1->Name = L"right_1";
				 this->right_1->Size = System::Drawing::Size(39, 40);
				 this->right_1->TabIndex = 0;
				 this->right_1->UseVisualStyleBackColor = false;
				 this->right_1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::right_1_MouseDown_1);
				 this->right_1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::right_1_MouseUp_1);
				 // 
				 // Z_down
				 // 
				 this->Z_down->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Z_down->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_down->ForeColor = System::Drawing::Color::Black;
				 this->Z_down->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Z_down.Image")));
				 this->Z_down->Location = System::Drawing::Point(162, 126);
				 this->Z_down->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_down->Name = L"Z_down";
				 this->Z_down->Size = System::Drawing::Size(49, 30);
				 this->Z_down->TabIndex = 0;
				 this->Z_down->Text = L"<";
				 this->Z_down->TextAlign = System::Drawing::ContentAlignment::TopCenter;
				 this->Z_down->UseVisualStyleBackColor = false;
				 this->Z_down->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::up_1_MouseDown);
				 this->Z_down->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::up_1_MouseUp);
				 // 
				 // Z_up
				 // 
				 this->Z_up->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Z_up->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_up->ForeColor = System::Drawing::Color::Black;
				 this->Z_up->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Z_up.Image")));
				 this->Z_up->Location = System::Drawing::Point(162, 155);
				 this->Z_up->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_up->Name = L"Z_up";
				 this->Z_up->Size = System::Drawing::Size(49, 30);
				 this->Z_up->TabIndex = 0;
				 this->Z_up->Text = L">";
				 this->Z_up->UseVisualStyleBackColor = false;
				 this->Z_up->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::down_1_MouseDown);
				 this->Z_up->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::down_1_MouseUp);
				 // 
				 // Abort
				 // 
				 this->Abort->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					 static_cast<System::Int32>(static_cast<System::Byte>(128)));
				 this->Abort->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Abort->Location = System::Drawing::Point(149, 59);
				 this->Abort->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Abort->Name = L"Abort";
				 this->Abort->Size = System::Drawing::Size(117, 33);
				 this->Abort->TabIndex = 0;
				 this->Abort->Text = L"Abort";
				 this->Abort->UseVisualStyleBackColor = false;
				 this->Abort->Click += gcnew System::EventHandler(this, &MyForm::Abort_Click);
				 // 
				 // Laser_L
				 // 
				 this->Laser_L->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser_L->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser_L->ForeColor = System::Drawing::Color::Black;
				 this->Laser_L->Location = System::Drawing::Point(30, 247);
				 this->Laser_L->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser_L->Name = L"Laser_L";
				 this->Laser_L->Size = System::Drawing::Size(42, 30);
				 this->Laser_L->TabIndex = 4;
				 this->Laser_L->Text = L" L";
				 this->Laser_L->UseVisualStyleBackColor = false;
				 this->Laser_L->Click += gcnew System::EventHandler(this, &MyForm::Laser_L_Click);
				 // 
				 // Start_Z_but
				 // 
				 this->Start_Z_but->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Start_Z_but->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Start_Z_but->ForeColor = System::Drawing::Color::Black;
				 this->Start_Z_but->Location = System::Drawing::Point(189, 15);
				 this->Start_Z_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Start_Z_but->Name = L"Start_Z_but";
				 this->Start_Z_but->Size = System::Drawing::Size(50, 25);
				 this->Start_Z_but->TabIndex = 0;
				 this->Start_Z_but->Text = L"Start_Z";
				 this->Start_Z_but->UseVisualStyleBackColor = false;
				 this->Start_Z_but->Click += gcnew System::EventHandler(this, &MyForm::Start_Z_but_Click);
				 // 
				 // End_Z_but
				 // 
				 this->End_Z_but->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->End_Z_but->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->End_Z_but->ForeColor = System::Drawing::Color::Black;
				 this->End_Z_but->Location = System::Drawing::Point(189, 41);
				 this->End_Z_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->End_Z_but->Name = L"End_Z_but";
				 this->End_Z_but->Size = System::Drawing::Size(50, 25);
				 this->End_Z_but->TabIndex = 0;
				 this->End_Z_but->Text = L"End_Z";
				 this->End_Z_but->UseVisualStyleBackColor = false;
				 this->End_Z_but->Click += gcnew System::EventHandler(this, &MyForm::End_Z_but_Click);
				 // 
				 // Start_Z_pos
				 // 
				 this->Start_Z_pos->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Start_Z_pos->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Start_Z_pos->ForeColor = System::Drawing::Color::Black;
				 this->Start_Z_pos->Location = System::Drawing::Point(139, 16);
				 this->Start_Z_pos->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Start_Z_pos->Name = L"Start_Z_pos";
				 this->Start_Z_pos->Size = System::Drawing::Size(50, 31);
				 this->Start_Z_pos->TabIndex = 4;
				 this->Start_Z_pos->Text = L"0";
				 this->Start_Z_pos->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Start_Z_pos->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->Start_Z_pos->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // End_Z_pos
				 // 
				 this->End_Z_pos->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->End_Z_pos->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->End_Z_pos->ForeColor = System::Drawing::Color::Black;
				 this->End_Z_pos->Location = System::Drawing::Point(139, 41);
				 this->End_Z_pos->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->End_Z_pos->Name = L"End_Z_pos";
				 this->End_Z_pos->Size = System::Drawing::Size(50, 31);
				 this->End_Z_pos->TabIndex = 5;
				 this->End_Z_pos->Text = L"0";
				 this->End_Z_pos->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->End_Z_pos->TextChanged += gcnew System::EventHandler(this, &MyForm::End_Z_pos_TextChanged);
				 this->End_Z_pos->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::End_Z_pos_KeyDown);
				 // 
				 // label16
				 // 
				 this->label16->AutoSize = true;
				 this->label16->BackColor = System::Drawing::Color::Transparent;
				 this->label16->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label16->ForeColor = System::Drawing::Color::White;
				 this->label16->Location = System::Drawing::Point(61, 136);
				 this->label16->Name = L"label16";
				 this->label16->Size = System::Drawing::Size(86, 22);
				 this->label16->TabIndex = 3;
				 this->label16->Text = L"Cycle time";
				 this->label16->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label20
				 // 
				 this->label20->AutoSize = true;
				 this->label20->BackColor = System::Drawing::Color::Transparent;
				 this->label20->Font = (gcnew System::Drawing::Font(L"Calibri", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label20->ForeColor = System::Drawing::Color::White;
				 this->label20->Location = System::Drawing::Point(209, 133);
				 this->label20->Name = L"label20";
				 this->label20->Size = System::Drawing::Size(89, 24);
				 this->label20->TabIndex = 3;
				 this->label20->Text = L"ms/flame";
				 this->label20->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label21
				 // 
				 this->label21->AutoSize = true;
				 this->label21->Font = (gcnew System::Drawing::Font(L"Calibri", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label21->ForeColor = System::Drawing::Color::White;
				 this->label21->Location = System::Drawing::Point(233, 32);
				 this->label21->Name = L"label21";
				 this->label21->Size = System::Drawing::Size(21, 24);
				 this->label21->TabIndex = 3;
				 this->label21->Text = L"V";
				 this->label21->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // Step
				 // 
				 this->Step->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Step->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Step->ForeColor = System::Drawing::Color::Black;
				 this->Step->Location = System::Drawing::Point(198, 73);
				 this->Step->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Step->Name = L"Step";
				 this->Step->Size = System::Drawing::Size(32, 31);
				 this->Step->TabIndex = 6;
				 this->Step->Text = L"2.5";
				 this->Step->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Step->TextChanged += gcnew System::EventHandler(this, &MyForm::Step_TextChanged);
				 // 
				 // label24
				 // 
				 this->label24->AutoSize = true;
				 this->label24->BackColor = System::Drawing::Color::Transparent;
				 this->label24->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label24->ForeColor = System::Drawing::Color::White;
				 this->label24->Location = System::Drawing::Point(235, 78);
				 this->label24->Name = L"label24";
				 this->label24->Size = System::Drawing::Size(37, 24);
				 this->label24->TabIndex = 3;
				 this->label24->Text = L"um";
				 this->label24->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // ETL_bar
				 // 
				 this->ETL_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->ETL_bar->Location = System::Drawing::Point(19, 39);
				 this->ETL_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_bar->Maximum = 1000;
				 this->ETL_bar->Name = L"ETL_bar";
				 this->ETL_bar->Size = System::Drawing::Size(176, 69);
				 this->ETL_bar->TabIndex = 100;
				 this->ETL_bar->TickFrequency = 50;
				 this->ETL_bar->Value = 100;
				 this->ETL_bar->Scroll += gcnew System::EventHandler(this, &MyForm::ETL_bar_Scroll);
				 // 
				 // label29
				 // 
				 this->label29->AutoSize = true;
				 this->label29->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label29->ForeColor = System::Drawing::Color::White;
				 this->label29->Location = System::Drawing::Point(3, 17);
				 this->label29->Name = L"label29";
				 this->label29->Size = System::Drawing::Size(47, 29);
				 this->label29->TabIndex = 3;
				 this->label29->Text = L"ETL";
				 this->label29->Click += gcnew System::EventHandler(this, &MyForm::label29_Click);
				 // 
				 // ETL_box
				 // 
				 this->ETL_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_box->ForeColor = System::Drawing::Color::Black;
				 this->ETL_box->Location = System::Drawing::Point(198, 26);
				 this->ETL_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_box->Name = L"ETL_box";
				 this->ETL_box->ReadOnly = true;
				 this->ETL_box->Size = System::Drawing::Size(35, 31);
				 this->ETL_box->TabIndex = 11;
				 this->ETL_box->Text = L"1.00";
				 this->ETL_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_box->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_box_TextChanged);
				 // 
				 // ETL_L
				 // 
				 this->ETL_L->BaudRate = 115200;
				 this->ETL_L->PortName = L"COM12";
				 this->ETL_L->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::ETL_L_DataReceived);
				 // 
				 // ETL_R
				 // 
				 this->ETL_R->BaudRate = 115200;
				 this->ETL_R->PortName = L"COM11";
				 this->ETL_R->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::ETL_R_DataReceived);
				 // 
				 // LIVE_but
				 // 
				 this->LIVE_but->BackColor = System::Drawing::Color::LightGray;
				 this->LIVE_but->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LIVE_but->Location = System::Drawing::Point(20, 99);
				 this->LIVE_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LIVE_but->Name = L"LIVE_but";
				 this->LIVE_but->Size = System::Drawing::Size(124, 32);
				 this->LIVE_but->TabIndex = 0;
				 this->LIVE_but->Text = L"LIVE";
				 this->LIVE_but->UseVisualStyleBackColor = false;
				 this->LIVE_but->Click += gcnew System::EventHandler(this, &MyForm::LIVE_but_Click);
				 // 
				 // close
				 // 
				 this->close->BackColor = System::Drawing::Color::CornflowerBlue;
				 this->close->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->close->ForeColor = System::Drawing::Color::White;
				 this->close->Location = System::Drawing::Point(943, -3);
				 this->close->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->close->Name = L"close";
				 this->close->Size = System::Drawing::Size(87, 29);
				 this->close->TabIndex = 19;
				 this->close->Text = L"Close";
				 this->close->UseVisualStyleBackColor = false;
				 this->close->Click += gcnew System::EventHandler(this, &MyForm::close_Click);
				 // 
				 // SIGMA_Theta
				 // 
				 this->SIGMA_Theta->PortName = L"COM5";
				 this->SIGMA_Theta->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::SIGMA_Theta_DataReceived);
				 // 
				 // Laser_R
				 // 
				 this->Laser_R->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser_R->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser_R->ForeColor = System::Drawing::Color::Black;
				 this->Laser_R->Location = System::Drawing::Point(81, 247);
				 this->Laser_R->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser_R->Name = L"Laser_R";
				 this->Laser_R->Size = System::Drawing::Size(44, 30);
				 this->Laser_R->TabIndex = 4;
				 this->Laser_R->Text = L"R";
				 this->Laser_R->UseVisualStyleBackColor = false;
				 this->Laser_R->Click += gcnew System::EventHandler(this, &MyForm::Laser_R_Click);
				 // 
				 // Stem_box1
				 // 
				 this->Stem_box1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Stem_box1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Stem_box1->ForeColor = System::Drawing::Color::Black;
				 this->Stem_box1->Location = System::Drawing::Point(112, 17);
				 this->Stem_box1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Stem_box1->Name = L"Stem_box1";
				 this->Stem_box1->Size = System::Drawing::Size(118, 31);
				 this->Stem_box1->TabIndex = 14;
				 this->Stem_box1->Text = L"sample1";
				 this->Stem_box1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Export_Box
				 // 
				 this->Export_Box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Export_Box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Export_Box->ForeColor = System::Drawing::Color::Black;
				 this->Export_Box->Location = System::Drawing::Point(67, 43);
				 this->Export_Box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Export_Box->Name = L"Export_Box";
				 this->Export_Box->Size = System::Drawing::Size(163, 31);
				 this->Export_Box->TabIndex = 16;
				 this->Export_Box->Text = L"Z:\\\\";
				 this->Export_Box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label88
				 // 
				 this->label88->AutoSize = true;
				 this->label88->BackColor = System::Drawing::Color::Transparent;
				 this->label88->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label88->ForeColor = System::Drawing::Color::White;
				 this->label88->Location = System::Drawing::Point(166, 77);
				 this->label88->Name = L"label88";
				 this->label88->Size = System::Drawing::Size(47, 24);
				 this->label88->TabIndex = 3;
				 this->label88->Text = L"Step";
				 this->label88->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label89
				 // 
				 this->label89->AutoSize = true;
				 this->label89->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label89->ForeColor = System::Drawing::Color::White;
				 this->label89->Location = System::Drawing::Point(6, 21);
				 this->label89->Name = L"label89";
				 this->label89->Size = System::Drawing::Size(119, 18);
				 this->label89->TabIndex = 3;
				 this->label89->Text = L"Sample name";
				 // 
				 // label91
				 // 
				 this->label91->AutoSize = true;
				 this->label91->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label91->ForeColor = System::Drawing::Color::White;
				 this->label91->Location = System::Drawing::Point(4, 46);
				 this->label91->Name = L"label91";
				 this->label91->Size = System::Drawing::Size(87, 18);
				 this->label91->TabIndex = 3;
				 this->label91->Text = L"Directory";
				 // 
				 // Export_dir_open
				 // 
				 this->Export_dir_open->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Export_dir_open->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Export_dir_open->ForeColor = System::Drawing::Color::Black;
				 this->Export_dir_open->Location = System::Drawing::Point(236, 44);
				 this->Export_dir_open->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Export_dir_open->Name = L"Export_dir_open";
				 this->Export_dir_open->Size = System::Drawing::Size(29, 22);
				 this->Export_dir_open->TabIndex = 0;
				 this->Export_dir_open->Text = L"...";
				 this->Export_dir_open->UseVisualStyleBackColor = false;
				 this->Export_dir_open->Click += gcnew System::EventHandler(this, &MyForm::Export_dir_open_Click);
				 // 
				 // label92
				 // 
				 this->label92->AutoSize = true;
				 this->label92->Font = (gcnew System::Drawing::Font(L"Calibri", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label92->ForeColor = System::Drawing::Color::White;
				 this->label92->Location = System::Drawing::Point(235, 80);
				 this->label92->Name = L"label92";
				 this->label92->Size = System::Drawing::Size(37, 24);
				 this->label92->TabIndex = 3;
				 this->label92->Text = L"mV";
				 this->label92->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // groupBox2
				 // 
				 this->groupBox2->BackColor = System::Drawing::Color::Gray;
				 this->groupBox2->Controls->Add(this->Export_dir_open);
				 this->groupBox2->Controls->Add(this->label91);
				 this->groupBox2->Controls->Add(this->label89);
				 this->groupBox2->Controls->Add(this->Stem_box1);
				 this->groupBox2->Controls->Add(this->Export_Box);
				 this->groupBox2->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox2->ForeColor = System::Drawing::Color::White;
				 this->groupBox2->Location = System::Drawing::Point(744, 237);
				 this->groupBox2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox2->Name = L"groupBox2";
				 this->groupBox2->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox2->Size = System::Drawing::Size(276, 75);
				 this->groupBox2->TabIndex = 20;
				 this->groupBox2->TabStop = false;
				 this->groupBox2->Text = L"File setting";
				 // 
				 // groupBox3
				 // 
				 this->groupBox3->BackColor = System::Drawing::Color::Gray;
				 this->groupBox3->Controls->Add(this->label57);
				 this->groupBox3->Controls->Add(this->label18);
				 this->groupBox3->Controls->Add(this->label11);
				 this->groupBox3->Controls->Add(this->Amplitude_sheet_bar);
				 this->groupBox3->Controls->Add(this->SheetSPD);
				 this->groupBox3->Controls->Add(this->Sheet_Box);
				 this->groupBox3->Controls->Add(this->Amplitude_sheet);
				 this->groupBox3->Controls->Add(this->label21);
				 this->groupBox3->Controls->Add(this->label92);
				 this->groupBox3->Controls->Add(this->SheetSPD_bar);
				 this->groupBox3->Controls->Add(this->Sheet_bar);
				 this->groupBox3->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox3->ForeColor = System::Drawing::Color::White;
				 this->groupBox3->Location = System::Drawing::Point(744, 314);
				 this->groupBox3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox3->Name = L"groupBox3";
				 this->groupBox3->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox3->Size = System::Drawing::Size(275, 177);
				 this->groupBox3->TabIndex = 21;
				 this->groupBox3->TabStop = false;
				 this->groupBox3->Text = L"Galvano_control";
				 // 
				 // label57
				 // 
				 this->label57->AutoSize = true;
				 this->label57->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label57->ForeColor = System::Drawing::Color::White;
				 this->label57->Location = System::Drawing::Point(6, 109);
				 this->label57->Name = L"label57";
				 this->label57->Size = System::Drawing::Size(113, 24);
				 this->label57->TabIndex = 3;
				 this->label57->Text = L"Sheet spped";
				 // 
				 // label18
				 // 
				 this->label18->AutoSize = true;
				 this->label18->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label18->ForeColor = System::Drawing::Color::White;
				 this->label18->Location = System::Drawing::Point(6, 65);
				 this->label18->Name = L"label18";
				 this->label18->Size = System::Drawing::Size(116, 24);
				 this->label18->TabIndex = 3;
				 this->label18->Text = L"Sheet offset:";
				 // 
				 // SheetSPD
				 // 
				 this->SheetSPD->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->SheetSPD->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SheetSPD->ForeColor = System::Drawing::Color::Black;
				 this->SheetSPD->Location = System::Drawing::Point(188, 127);
				 this->SheetSPD->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SheetSPD->Name = L"SheetSPD";
				 this->SheetSPD->Size = System::Drawing::Size(61, 27);
				 this->SheetSPD->TabIndex = 17;
				 this->SheetSPD->Text = L"1";
				 this->SheetSPD->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Sheet_Box
				 // 
				 this->Sheet_Box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Sheet_Box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Sheet_Box->ForeColor = System::Drawing::Color::Black;
				 this->Sheet_Box->Location = System::Drawing::Point(188, 80);
				 this->Sheet_Box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Sheet_Box->Name = L"Sheet_Box";
				 this->Sheet_Box->Size = System::Drawing::Size(40, 27);
				 this->Sheet_Box->TabIndex = 17;
				 this->Sheet_Box->Text = L"0";
				 this->Sheet_Box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Sheet_Box->TextChanged += gcnew System::EventHandler(this, &MyForm::Sheet_Box_TextChanged);
				 // 
				 // SheetSPD_bar
				 // 
				 this->SheetSPD_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->SheetSPD_bar->LargeChange = 1;
				 this->SheetSPD_bar->Location = System::Drawing::Point(17, 124);
				 this->SheetSPD_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 0);
				 this->SheetSPD_bar->Maximum = 20000;
				 this->SheetSPD_bar->Name = L"SheetSPD_bar";
				 this->SheetSPD_bar->Size = System::Drawing::Size(165, 69);
				 this->SheetSPD_bar->TabIndex = 1;
				 this->SheetSPD_bar->TickFrequency = 1000;
				 this->SheetSPD_bar->Value = 10000;
				 this->SheetSPD_bar->Scroll += gcnew System::EventHandler(this, &MyForm::SheetSPD_bar_Scroll);
				 // 
				 // Sheet_bar
				 // 
				 this->Sheet_bar->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->Sheet_bar->LargeChange = 1;
				 this->Sheet_bar->Location = System::Drawing::Point(17, 79);
				 this->Sheet_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 0);
				 this->Sheet_bar->Maximum = 1000;
				 this->Sheet_bar->Name = L"Sheet_bar";
				 this->Sheet_bar->Size = System::Drawing::Size(165, 69);
				 this->Sheet_bar->TabIndex = 1;
				 this->Sheet_bar->TickFrequency = 100;
				 this->Sheet_bar->Value = 500;
				 this->Sheet_bar->Scroll += gcnew System::EventHandler(this, &MyForm::Sheet_bar_Scroll);
				 // 
				 // groupBox4
				 // 
				 this->groupBox4->BackColor = System::Drawing::Color::Gray;
				 this->groupBox4->Controls->Add(this->TargetX_box);
				 this->groupBox4->Controls->Add(this->X_pos_current);
				 this->groupBox4->Controls->Add(this->s_pos_current);
				 this->groupBox4->Controls->Add(this->s_pos_target);
				 this->groupBox4->Controls->Add(this->Theta_zero);
				 this->groupBox4->Controls->Add(this->TargetY_box);
				 this->groupBox4->Controls->Add(this->Y_pos_current);
				 this->groupBox4->Controls->Add(this->Z_pos_current);
				 this->groupBox4->Controls->Add(this->Z_target_pos);
				 this->groupBox4->Controls->Add(this->up_1);
				 this->groupBox4->Controls->Add(this->down_1);
				 this->groupBox4->Controls->Add(this->Z_down);
				 this->groupBox4->Controls->Add(this->Z_up);
				 this->groupBox4->Controls->Add(this->s_left);
				 this->groupBox4->Controls->Add(this->left_1);
				 this->groupBox4->Controls->Add(this->s_right);
				 this->groupBox4->Controls->Add(this->right_1);
				 this->groupBox4->Controls->Add(this->label5);
				 this->groupBox4->Controls->Add(this->label152);
				 this->groupBox4->Controls->Add(this->label153);
				 this->groupBox4->Controls->Add(this->label56);
				 this->groupBox4->Controls->Add(this->label12);
				 this->groupBox4->Controls->Add(this->label6);
				 this->groupBox4->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox4->ForeColor = System::Drawing::Color::White;
				 this->groupBox4->Location = System::Drawing::Point(6, 40);
				 this->groupBox4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox4->Name = L"groupBox4";
				 this->groupBox4->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox4->Size = System::Drawing::Size(253, 293);
				 this->groupBox4->TabIndex = 22;
				 this->groupBox4->TabStop = false;
				 this->groupBox4->Text = L"Stage";
				 // 
				 // TargetX_box
				 // 
				 this->TargetX_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->TargetX_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->TargetX_box->ForeColor = System::Drawing::Color::Black;
				 this->TargetX_box->Location = System::Drawing::Point(22, 40);
				 this->TargetX_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->TargetX_box->Name = L"TargetX_box";
				 this->TargetX_box->Size = System::Drawing::Size(48, 31);
				 this->TargetX_box->TabIndex = 1;
				 this->TargetX_box->Text = L"0";
				 this->TargetX_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->TargetX_box->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::X_pos_current_KeyDown);
				 // 
				 // s_pos_current
				 // 
				 this->s_pos_current->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->s_pos_current->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->s_pos_current->ForeColor = System::Drawing::Color::Black;
				 this->s_pos_current->Location = System::Drawing::Point(73, 207);
				 this->s_pos_current->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->s_pos_current->Name = L"s_pos_current";
				 this->s_pos_current->Size = System::Drawing::Size(48, 31);
				 this->s_pos_current->TabIndex = 1;
				 this->s_pos_current->Text = L"0";
				 this->s_pos_current->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // s_pos_target
				 // 
				 this->s_pos_target->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->s_pos_target->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->s_pos_target->ForeColor = System::Drawing::Color::Black;
				 this->s_pos_target->Location = System::Drawing::Point(22, 207);
				 this->s_pos_target->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->s_pos_target->Name = L"s_pos_target";
				 this->s_pos_target->Size = System::Drawing::Size(48, 31);
				 this->s_pos_target->TabIndex = 1;
				 this->s_pos_target->Text = L"0";
				 this->s_pos_target->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->s_pos_target->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::s_pos_target_KeyDown_1);
				 // 
				 // Theta_zero
				 // 
				 this->Theta_zero->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Theta_zero->ForeColor = System::Drawing::Color::Black;
				 this->Theta_zero->Location = System::Drawing::Point(141, 247);
				 this->Theta_zero->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_zero->Name = L"Theta_zero";
				 this->Theta_zero->Size = System::Drawing::Size(88, 25);
				 this->Theta_zero->TabIndex = 4;
				 this->Theta_zero->Text = L"Theta set zero";
				 this->Theta_zero->UseVisualStyleBackColor = true;
				 this->Theta_zero->Click += gcnew System::EventHandler(this, &MyForm::Theta_zero_Click);
				 // 
				 // TargetY_box
				 // 
				 this->TargetY_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->TargetY_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->TargetY_box->ForeColor = System::Drawing::Color::Black;
				 this->TargetY_box->Location = System::Drawing::Point(22, 142);
				 this->TargetY_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->TargetY_box->Name = L"TargetY_box";
				 this->TargetY_box->Size = System::Drawing::Size(48, 31);
				 this->TargetY_box->TabIndex = 1;
				 this->TargetY_box->Text = L"0";
				 this->TargetY_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->TargetY_box->TextChanged += gcnew System::EventHandler(this, &MyForm::TargetY_box_TextChanged);
				 this->TargetY_box->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Y_pos_current_KeyDown);
				 // 
				 // s_left
				 // 
				 this->s_left->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->s_left->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->s_left->ForeColor = System::Drawing::Color::Black;
				 this->s_left->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"s_left.Image")));
				 this->s_left->Location = System::Drawing::Point(141, 202);
				 this->s_left->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->s_left->Name = L"s_left";
				 this->s_left->Size = System::Drawing::Size(38, 40);
				 this->s_left->TabIndex = 0;
				 this->s_left->TextAlign = System::Drawing::ContentAlignment::TopCenter;
				 this->s_left->UseVisualStyleBackColor = false;
				 this->s_left->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::s_left_MouseDown);
				 this->s_left->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::s_left_MouseUp);
				 // 
				 // left_1
				 // 
				 this->left_1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->left_1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->left_1->ForeColor = System::Drawing::Color::Black;
				 this->left_1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"left_1.Image")));
				 this->left_1->Location = System::Drawing::Point(126, 46);
				 this->left_1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->left_1->Name = L"left_1";
				 this->left_1->Size = System::Drawing::Size(38, 40);
				 this->left_1->TabIndex = 0;
				 this->left_1->TextAlign = System::Drawing::ContentAlignment::TopCenter;
				 this->left_1->UseVisualStyleBackColor = false;
				 this->left_1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::left_1_MouseDown);
				 this->left_1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::left_1_MouseUp);
				 // 
				 // s_right
				 // 
				 this->s_right->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->s_right->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->s_right->ForeColor = System::Drawing::Color::Black;
				 this->s_right->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"s_right.Image")));
				 this->s_right->Location = System::Drawing::Point(190, 202);
				 this->s_right->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->s_right->Name = L"s_right";
				 this->s_right->Size = System::Drawing::Size(39, 40);
				 this->s_right->TabIndex = 0;
				 this->s_right->UseVisualStyleBackColor = false;
				 this->s_right->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::s_right_MouseDown_1);
				 this->s_right->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::s_right_MouseUp_1);
				 // 
				 // label152
				 // 
				 this->label152->AutoSize = true;
				 this->label152->BackColor = System::Drawing::Color::Transparent;
				 this->label152->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label152->ForeColor = System::Drawing::Color::White;
				 this->label152->Location = System::Drawing::Point(73, 22);
				 this->label152->Name = L"label152";
				 this->label152->Size = System::Drawing::Size(74, 24);
				 this->label152->TabIndex = 3;
				 this->label152->Text = L"Current";
				 this->label152->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label153
				 // 
				 this->label153->AutoSize = true;
				 this->label153->BackColor = System::Drawing::Color::Transparent;
				 this->label153->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label153->ForeColor = System::Drawing::Color::White;
				 this->label153->Location = System::Drawing::Point(31, 23);
				 this->label153->Name = L"label153";
				 this->label153->Size = System::Drawing::Size(61, 24);
				 this->label153->TabIndex = 3;
				 this->label153->Text = L"Target";
				 this->label153->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label56
				 // 
				 this->label56->AutoSize = true;
				 this->label56->BackColor = System::Drawing::Color::Transparent;
				 this->label56->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label56->ForeColor = System::Drawing::Color::White;
				 this->label56->Location = System::Drawing::Point(5, 209);
				 this->label56->Name = L"label56";
				 this->label56->Size = System::Drawing::Size(30, 24);
				 this->label56->TabIndex = 3;
				 this->label56->Text = L"Θ:";
				 // 
				 // label12
				 // 
				 this->label12->AutoSize = true;
				 this->label12->BackColor = System::Drawing::Color::Transparent;
				 this->label12->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label12->ForeColor = System::Drawing::Color::White;
				 this->label12->Location = System::Drawing::Point(5, 146);
				 this->label12->Name = L"label12";
				 this->label12->Size = System::Drawing::Size(26, 24);
				 this->label12->TabIndex = 3;
				 this->label12->Text = L"Z:";
				 // 
				 // groupBox5
				 // 
				 this->groupBox5->BackColor = System::Drawing::Color::Gray;
				 this->groupBox5->Controls->Add(this->L647_current_power);
				 this->groupBox5->Controls->Add(this->L592_current_power);
				 this->groupBox5->Controls->Add(this->L488_current_power);
				 this->groupBox5->Controls->Add(this->LR_box);
				 this->groupBox5->Controls->Add(this->label3);
				 this->groupBox5->Controls->Add(this->label2);
				 this->groupBox5->Controls->Add(this->Laser_L);
				 this->groupBox5->Controls->Add(this->Laser_R);
				 this->groupBox5->Controls->Add(this->L488_box);
				 this->groupBox5->Controls->Add(this->L592_box);
				 this->groupBox5->Controls->Add(this->L647_box);
				 this->groupBox5->Controls->Add(this->L488_ON);
				 this->groupBox5->Controls->Add(this->L592_ON);
				 this->groupBox5->Controls->Add(this->L647_ON);
				 this->groupBox5->Controls->Add(this->L488_bar);
				 this->groupBox5->Controls->Add(this->L592_bar);
				 this->groupBox5->Controls->Add(this->L647_bar);
				 this->groupBox5->Controls->Add(this->label1);
				 this->groupBox5->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox5->ForeColor = System::Drawing::Color::White;
				 this->groupBox5->Location = System::Drawing::Point(8, 342);
				 this->groupBox5->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox5->Name = L"groupBox5";
				 this->groupBox5->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox5->Size = System::Drawing::Size(249, 318);
				 this->groupBox5->TabIndex = 23;
				 this->groupBox5->TabStop = false;
				 this->groupBox5->Text = L"Laser";
				 this->groupBox5->Enter += gcnew System::EventHandler(this, &MyForm::groupBox5_Enter);
				 // 
				 // L647_current_power
				 // 
				 this->L647_current_power->AutoSize = true;
				 this->L647_current_power->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L647_current_power->Location = System::Drawing::Point(191, 190);
				 this->L647_current_power->Name = L"L647_current_power";
				 this->L647_current_power->Size = System::Drawing::Size(39, 20);
				 this->L647_current_power->TabIndex = 11;
				 this->L647_current_power->Text = L"0.0";
				 this->L647_current_power->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
				 // 
				 // L592_current_power
				 // 
				 this->L592_current_power->AutoSize = true;
				 this->L592_current_power->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L592_current_power->Location = System::Drawing::Point(191, 103);
				 this->L592_current_power->Name = L"L592_current_power";
				 this->L592_current_power->Size = System::Drawing::Size(39, 20);
				 this->L592_current_power->TabIndex = 11;
				 this->L592_current_power->Text = L"0.0";
				 this->L592_current_power->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
				 // 
				 // L488_current_power
				 // 
				 this->L488_current_power->AutoSize = true;
				 this->L488_current_power->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->L488_current_power->Location = System::Drawing::Point(191, 25);
				 this->L488_current_power->Name = L"L488_current_power";
				 this->L488_current_power->Size = System::Drawing::Size(39, 20);
				 this->L488_current_power->TabIndex = 11;
				 this->L488_current_power->Text = L"0.0";
				 this->L488_current_power->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
				 // 
				 // LR_box
				 // 
				 this->LR_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LR_box->ForeColor = System::Drawing::Color::Black;
				 this->LR_box->Location = System::Drawing::Point(134, 250);
				 this->LR_box->Name = L"LR_box";
				 this->LR_box->ReadOnly = true;
				 this->LR_box->Size = System::Drawing::Size(93, 34);
				 this->LR_box->TabIndex = 5;
				 this->LR_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 // 
				 // groupBox6
				 // 
				 this->groupBox6->BackColor = System::Drawing::Color::Gray;
				 this->groupBox6->Controls->Add(this->Laser_filter_setting_but);
				 this->groupBox6->Controls->Add(this->EXP_time1);
				 this->groupBox6->Controls->Add(this->LP1);
				 this->groupBox6->Controls->Add(this->set4_name);
				 this->groupBox6->Controls->Add(this->set3_name);
				 this->groupBox6->Controls->Add(this->set2_name);
				 this->groupBox6->Controls->Add(this->set1_name);
				 this->groupBox6->Controls->Add(this->filter1_comb);
				 this->groupBox6->Controls->Add(this->EXP_time4);
				 this->groupBox6->Controls->Add(this->Laser1_comb);
				 this->groupBox6->Controls->Add(this->LP4);
				 this->groupBox6->Controls->Add(this->EXP_time3);
				 this->groupBox6->Controls->Add(this->LP3);
				 this->groupBox6->Controls->Add(this->EXP_time2);
				 this->groupBox6->Controls->Add(this->LP2);
				 this->groupBox6->Controls->Add(this->filter4_comb);
				 this->groupBox6->Controls->Add(this->filter3_comb);
				 this->groupBox6->Controls->Add(this->filter2_comb);
				 this->groupBox6->Controls->Add(this->Laser4_comb);
				 this->groupBox6->Controls->Add(this->Laser3_comb);
				 this->groupBox6->Controls->Add(this->Laser2_comb);
				 this->groupBox6->Controls->Add(this->label115);
				 this->groupBox6->Controls->Add(this->label104);
				 this->groupBox6->Controls->Add(this->label94);
				 this->groupBox6->Controls->Add(this->label93);
				 this->groupBox6->Controls->Add(this->LF_check4);
				 this->groupBox6->Controls->Add(this->LF_check3);
				 this->groupBox6->Controls->Add(this->LF_check2);
				 this->groupBox6->Controls->Add(this->multi_laser_check);
				 this->groupBox6->Controls->Add(this->LF_check1);
				 this->groupBox6->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox6->ForeColor = System::Drawing::Color::White;
				 this->groupBox6->Location = System::Drawing::Point(259, 498);
				 this->groupBox6->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox6->Name = L"groupBox6";
				 this->groupBox6->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox6->Size = System::Drawing::Size(550, 162);
				 this->groupBox6->TabIndex = 24;
				 this->groupBox6->TabStop = false;
				 this->groupBox6->Text = L"Laser-Filter setting";
				 // 
				 // Laser_filter_setting_but
				 // 
				 this->Laser_filter_setting_but->BackColor = System::Drawing::Color::Gray;
				 this->Laser_filter_setting_but->Location = System::Drawing::Point(454, 19);
				 this->Laser_filter_setting_but->Name = L"Laser_filter_setting_but";
				 this->Laser_filter_setting_but->Size = System::Drawing::Size(88, 24);
				 this->Laser_filter_setting_but->TabIndex = 0;
				 this->Laser_filter_setting_but->Text = L"Set";
				 this->Laser_filter_setting_but->UseVisualStyleBackColor = false;
				 this->Laser_filter_setting_but->Click += gcnew System::EventHandler(this, &MyForm::Laser_filter_setting_but_Click);
				 // 
				 // EXP_time1
				 // 
				 this->EXP_time1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->EXP_time1->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->EXP_time1->ForeColor = System::Drawing::Color::Black;
				 this->EXP_time1->Location = System::Drawing::Point(320, 68);
				 this->EXP_time1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->EXP_time1->Name = L"EXP_time1";
				 this->EXP_time1->Size = System::Drawing::Size(53, 27);
				 this->EXP_time1->TabIndex = 24;
				 this->EXP_time1->Text = L"0";
				 this->EXP_time1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // LP1
				 // 
				 this->LP1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LP1->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LP1->ForeColor = System::Drawing::Color::Black;
				 this->LP1->Location = System::Drawing::Point(144, 66);
				 this->LP1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LP1->Name = L"LP1";
				 this->LP1->Size = System::Drawing::Size(54, 27);
				 this->LP1->TabIndex = 22;
				 this->LP1->Text = L"0";
				 this->LP1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // set4_name
				 // 
				 this->set4_name->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->set4_name->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->set4_name->ForeColor = System::Drawing::Color::Black;
				 this->set4_name->Location = System::Drawing::Point(330, 163);
				 this->set4_name->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->set4_name->Name = L"set4_name";
				 this->set4_name->Size = System::Drawing::Size(132, 31);
				 this->set4_name->TabIndex = 54;
				 this->set4_name->Text = L"set4";
				 this->set4_name->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // set3_name
				 // 
				 this->set3_name->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->set3_name->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->set3_name->ForeColor = System::Drawing::Color::Black;
				 this->set3_name->Location = System::Drawing::Point(398, 133);
				 this->set3_name->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->set3_name->Name = L"set3_name";
				 this->set3_name->Size = System::Drawing::Size(136, 31);
				 this->set3_name->TabIndex = 47;
				 this->set3_name->Text = L"set3";
				 this->set3_name->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // set2_name
				 // 
				 this->set2_name->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->set2_name->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->set2_name->ForeColor = System::Drawing::Color::Black;
				 this->set2_name->Location = System::Drawing::Point(398, 100);
				 this->set2_name->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->set2_name->Name = L"set2_name";
				 this->set2_name->Size = System::Drawing::Size(136, 31);
				 this->set2_name->TabIndex = 33;
				 this->set2_name->Text = L"set2";
				 this->set2_name->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // set1_name
				 // 
				 this->set1_name->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->set1_name->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->set1_name->ForeColor = System::Drawing::Color::Black;
				 this->set1_name->Location = System::Drawing::Point(397, 67);
				 this->set1_name->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->set1_name->Name = L"set1_name";
				 this->set1_name->Size = System::Drawing::Size(136, 31);
				 this->set1_name->TabIndex = 26;
				 this->set1_name->Text = L"set1";
				 this->set1_name->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter1_comb
				 // 
				 this->filter1_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->filter1_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->filter1_comb->ForeColor = System::Drawing::Color::Black;
				 this->filter1_comb->FormattingEnabled = true;
				 this->filter1_comb->Location = System::Drawing::Point(210, 67);
				 this->filter1_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter1_comb->Name = L"filter1_comb";
				 this->filter1_comb->Size = System::Drawing::Size(95, 28);
				 this->filter1_comb->TabIndex = 23;
				 // 
				 // EXP_time4
				 // 
				 this->EXP_time4->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->EXP_time4->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->EXP_time4->ForeColor = System::Drawing::Color::Black;
				 this->EXP_time4->Location = System::Drawing::Point(265, 164);
				 this->EXP_time4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->EXP_time4->Name = L"EXP_time4";
				 this->EXP_time4->Size = System::Drawing::Size(53, 27);
				 this->EXP_time4->TabIndex = 52;
				 this->EXP_time4->Text = L"0";
				 this->EXP_time4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Laser1_comb
				 // 
				 this->Laser1_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser1_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser1_comb->ForeColor = System::Drawing::Color::Black;
				 this->Laser1_comb->FormattingEnabled = true;
				 this->Laser1_comb->Items->AddRange(gcnew cli::array< System::Object^  >(4) { L"", L"488", L"592", L"647" });
				 this->Laser1_comb->Location = System::Drawing::Point(28, 67);
				 this->Laser1_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser1_comb->Name = L"Laser1_comb";
				 this->Laser1_comb->Size = System::Drawing::Size(110, 28);
				 this->Laser1_comb->TabIndex = 21;
				 // 
				 // LP4
				 // 
				 this->LP4->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LP4->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LP4->ForeColor = System::Drawing::Color::Black;
				 this->LP4->Location = System::Drawing::Point(113, 163);
				 this->LP4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LP4->Name = L"LP4";
				 this->LP4->Size = System::Drawing::Size(46, 27);
				 this->LP4->TabIndex = 50;
				 this->LP4->Text = L"0";
				 this->LP4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // EXP_time3
				 // 
				 this->EXP_time3->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->EXP_time3->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->EXP_time3->ForeColor = System::Drawing::Color::Black;
				 this->EXP_time3->Location = System::Drawing::Point(320, 132);
				 this->EXP_time3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->EXP_time3->Name = L"EXP_time3";
				 this->EXP_time3->Size = System::Drawing::Size(53, 27);
				 this->EXP_time3->TabIndex = 45;
				 this->EXP_time3->Text = L"0";
				 this->EXP_time3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // LP3
				 // 
				 this->LP3->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LP3->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LP3->ForeColor = System::Drawing::Color::Black;
				 this->LP3->Location = System::Drawing::Point(144, 130);
				 this->LP3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LP3->Name = L"LP3";
				 this->LP3->Size = System::Drawing::Size(54, 27);
				 this->LP3->TabIndex = 43;
				 this->LP3->Text = L"0";
				 this->LP3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // EXP_time2
				 // 
				 this->EXP_time2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->EXP_time2->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->EXP_time2->ForeColor = System::Drawing::Color::Black;
				 this->EXP_time2->Location = System::Drawing::Point(320, 101);
				 this->EXP_time2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->EXP_time2->Name = L"EXP_time2";
				 this->EXP_time2->Size = System::Drawing::Size(53, 27);
				 this->EXP_time2->TabIndex = 31;
				 this->EXP_time2->Text = L"0";
				 this->EXP_time2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // LP2
				 // 
				 this->LP2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LP2->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LP2->ForeColor = System::Drawing::Color::Black;
				 this->LP2->Location = System::Drawing::Point(144, 98);
				 this->LP2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LP2->Name = L"LP2";
				 this->LP2->Size = System::Drawing::Size(54, 27);
				 this->LP2->TabIndex = 29;
				 this->LP2->Text = L"0";
				 this->LP2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter4_comb
				 // 
				 this->filter4_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->filter4_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->filter4_comb->ForeColor = System::Drawing::Color::Black;
				 this->filter4_comb->FormattingEnabled = true;
				 this->filter4_comb->Location = System::Drawing::Point(177, 162);
				 this->filter4_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter4_comb->Name = L"filter4_comb";
				 this->filter4_comb->Size = System::Drawing::Size(72, 28);
				 this->filter4_comb->TabIndex = 51;
				 // 
				 // filter3_comb
				 // 
				 this->filter3_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->filter3_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->filter3_comb->ForeColor = System::Drawing::Color::Black;
				 this->filter3_comb->FormattingEnabled = true;
				 this->filter3_comb->Location = System::Drawing::Point(210, 131);
				 this->filter3_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter3_comb->Name = L"filter3_comb";
				 this->filter3_comb->Size = System::Drawing::Size(95, 28);
				 this->filter3_comb->TabIndex = 44;
				 // 
				 // filter2_comb
				 // 
				 this->filter2_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->filter2_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->filter2_comb->ForeColor = System::Drawing::Color::Black;
				 this->filter2_comb->FormattingEnabled = true;
				 this->filter2_comb->Location = System::Drawing::Point(210, 100);
				 this->filter2_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter2_comb->Name = L"filter2_comb";
				 this->filter2_comb->Size = System::Drawing::Size(95, 28);
				 this->filter2_comb->TabIndex = 30;
				 // 
				 // Laser4_comb
				 // 
				 this->Laser4_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser4_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser4_comb->ForeColor = System::Drawing::Color::Black;
				 this->Laser4_comb->FormattingEnabled = true;
				 this->Laser4_comb->Items->AddRange(gcnew cli::array< System::Object^  >(4) { L"", L"488", L"592", L"647" });
				 this->Laser4_comb->Location = System::Drawing::Point(28, 162);
				 this->Laser4_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser4_comb->Name = L"Laser4_comb";
				 this->Laser4_comb->Size = System::Drawing::Size(75, 28);
				 this->Laser4_comb->TabIndex = 49;
				 // 
				 // Laser3_comb
				 // 
				 this->Laser3_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser3_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser3_comb->ForeColor = System::Drawing::Color::Black;
				 this->Laser3_comb->FormattingEnabled = true;
				 this->Laser3_comb->Items->AddRange(gcnew cli::array< System::Object^  >(4) { L"", L"488", L"592", L"647" });
				 this->Laser3_comb->Location = System::Drawing::Point(28, 130);
				 this->Laser3_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser3_comb->Name = L"Laser3_comb";
				 this->Laser3_comb->Size = System::Drawing::Size(110, 28);
				 this->Laser3_comb->TabIndex = 42;
				 // 
				 // Laser2_comb
				 // 
				 this->Laser2_comb->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Laser2_comb->Font = (gcnew System::Drawing::Font(L"ＭＳ Ｐゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Laser2_comb->ForeColor = System::Drawing::Color::Black;
				 this->Laser2_comb->FormattingEnabled = true;
				 this->Laser2_comb->Items->AddRange(gcnew cli::array< System::Object^  >(4) { L"", L"488", L"592", L"647" });
				 this->Laser2_comb->Location = System::Drawing::Point(28, 99);
				 this->Laser2_comb->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Laser2_comb->Name = L"Laser2_comb";
				 this->Laser2_comb->Size = System::Drawing::Size(110, 28);
				 this->Laser2_comb->TabIndex = 28;
				 // 
				 // label115
				 // 
				 this->label115->AutoSize = true;
				 this->label115->BackColor = System::Drawing::Color::Transparent;
				 this->label115->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label115->ForeColor = System::Drawing::Color::White;
				 this->label115->Location = System::Drawing::Point(428, 43);
				 this->label115->Name = L"label115";
				 this->label115->Size = System::Drawing::Size(101, 29);
				 this->label115->TabIndex = 3;
				 this->label115->Text = L"filename";
				 // 
				 // label104
				 // 
				 this->label104->AutoSize = true;
				 this->label104->BackColor = System::Drawing::Color::Transparent;
				 this->label104->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label104->ForeColor = System::Drawing::Color::White;
				 this->label104->Location = System::Drawing::Point(316, 43);
				 this->label104->Name = L"label104";
				 this->label104->Size = System::Drawing::Size(93, 29);
				 this->label104->TabIndex = 3;
				 this->label104->Text = L"Exp(ms)";
				 // 
				 // label94
				 // 
				 this->label94->AutoSize = true;
				 this->label94->BackColor = System::Drawing::Color::Transparent;
				 this->label94->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label94->ForeColor = System::Drawing::Color::White;
				 this->label94->Location = System::Drawing::Point(235, 43);
				 this->label94->Name = L"label94";
				 this->label94->Size = System::Drawing::Size(65, 29);
				 this->label94->TabIndex = 3;
				 this->label94->Text = L"Filter";
				 // 
				 // label93
				 // 
				 this->label93->AutoSize = true;
				 this->label93->BackColor = System::Drawing::Color::Transparent;
				 this->label93->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label93->ForeColor = System::Drawing::Color::White;
				 this->label93->Location = System::Drawing::Point(41, 43);
				 this->label93->Name = L"label93";
				 this->label93->Size = System::Drawing::Size(123, 29);
				 this->label93->TabIndex = 3;
				 this->label93->Text = L"Laser (mw)";
				 // 
				 // LF_check4
				 // 
				 this->LF_check4->AutoSize = true;
				 this->LF_check4->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LF_check4->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->LF_check4->ForeColor = System::Drawing::Color::Black;
				 this->LF_check4->Location = System::Drawing::Point(8, 166);
				 this->LF_check4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LF_check4->Name = L"LF_check4";
				 this->LF_check4->Size = System::Drawing::Size(22, 21);
				 this->LF_check4->TabIndex = 48;
				 this->LF_check4->UseVisualStyleBackColor = false;
				 this->LF_check4->CheckedChanged += gcnew System::EventHandler(this, &MyForm::LF_check4_CheckedChanged);
				 // 
				 // LF_check3
				 // 
				 this->LF_check3->AutoSize = true;
				 this->LF_check3->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LF_check3->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->LF_check3->ForeColor = System::Drawing::Color::Black;
				 this->LF_check3->Location = System::Drawing::Point(8, 135);
				 this->LF_check3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LF_check3->Name = L"LF_check3";
				 this->LF_check3->Size = System::Drawing::Size(22, 21);
				 this->LF_check3->TabIndex = 41;
				 this->LF_check3->UseVisualStyleBackColor = false;
				 this->LF_check3->CheckedChanged += gcnew System::EventHandler(this, &MyForm::LF_check3_CheckedChanged);
				 // 
				 // LF_check2
				 // 
				 this->LF_check2->AutoSize = true;
				 this->LF_check2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LF_check2->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->LF_check2->ForeColor = System::Drawing::Color::Black;
				 this->LF_check2->Location = System::Drawing::Point(8, 102);
				 this->LF_check2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LF_check2->Name = L"LF_check2";
				 this->LF_check2->Size = System::Drawing::Size(22, 21);
				 this->LF_check2->TabIndex = 27;
				 this->LF_check2->UseVisualStyleBackColor = false;
				 this->LF_check2->CheckedChanged += gcnew System::EventHandler(this, &MyForm::LF_check2_CheckedChanged);
				 // 
				 // multi_laser_check
				 // 
				 this->multi_laser_check->AutoSize = true;
				 this->multi_laser_check->BackColor = System::Drawing::Color::Transparent;
				 this->multi_laser_check->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->multi_laser_check->ForeColor = System::Drawing::Color::White;
				 this->multi_laser_check->Location = System::Drawing::Point(8, 22);
				 this->multi_laser_check->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->multi_laser_check->Name = L"multi_laser_check";
				 this->multi_laser_check->Size = System::Drawing::Size(163, 27);
				 this->multi_laser_check->TabIndex = 17;
				 this->multi_laser_check->Text = L"Multi imaging";
				 this->multi_laser_check->UseVisualStyleBackColor = false;
				 this->multi_laser_check->CheckedChanged += gcnew System::EventHandler(this, &MyForm::multi_laser_check_CheckedChanged);
				 // 
				 // LF_check1
				 // 
				 this->LF_check1->AutoSize = true;
				 this->LF_check1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->LF_check1->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->LF_check1->ForeColor = System::Drawing::Color::Black;
				 this->LF_check1->Location = System::Drawing::Point(8, 71);
				 this->LF_check1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LF_check1->Name = L"LF_check1";
				 this->LF_check1->Size = System::Drawing::Size(22, 21);
				 this->LF_check1->TabIndex = 20;
				 this->LF_check1->UseVisualStyleBackColor = false;
				 this->LF_check1->CheckedChanged += gcnew System::EventHandler(this, &MyForm::LF_check1_CheckedChanged);
				 // 
				 // FIlter_2
				 // 
				 this->FIlter_2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_2->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_2->Location = System::Drawing::Point(11, 48);
				 this->FIlter_2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_2->Name = L"FIlter_2";
				 this->FIlter_2->Size = System::Drawing::Size(87, 25);
				 this->FIlter_2->TabIndex = 1;
				 this->FIlter_2->UseVisualStyleBackColor = false;
				 this->FIlter_2->Click += gcnew System::EventHandler(this, &MyForm::FIlter_2_Click);
				 // 
				 // FIlter_5
				 // 
				 this->FIlter_5->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_5->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_5->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_5->Location = System::Drawing::Point(104, 17);
				 this->FIlter_5->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_5->Name = L"FIlter_5";
				 this->FIlter_5->Size = System::Drawing::Size(84, 25);
				 this->FIlter_5->TabIndex = 1;
				 this->FIlter_5->UseVisualStyleBackColor = false;
				 this->FIlter_5->Click += gcnew System::EventHandler(this, &MyForm::FIlter_5_Click);
				 // 
				 // FIlter_6
				 // 
				 this->FIlter_6->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_6->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_6->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_6->Location = System::Drawing::Point(104, 46);
				 this->FIlter_6->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_6->Name = L"FIlter_6";
				 this->FIlter_6->Size = System::Drawing::Size(84, 25);
				 this->FIlter_6->TabIndex = 1;
				 this->FIlter_6->UseVisualStyleBackColor = false;
				 this->FIlter_6->Click += gcnew System::EventHandler(this, &MyForm::FIlter_6_Click);
				 // 
				 // groupBox7
				 // 
				 this->groupBox7->BackColor = System::Drawing::Color::Gray;
				 this->groupBox7->Controls->Add(this->current_filter_box);
				 this->groupBox7->Controls->Add(this->FIlter_6);
				 this->groupBox7->Controls->Add(this->FIlter_5);
				 this->groupBox7->Controls->Add(this->FIlter_8);
				 this->groupBox7->Controls->Add(this->FIlter_4);
				 this->groupBox7->Controls->Add(this->FIlter_7);
				 this->groupBox7->Controls->Add(this->FIlter_3);
				 this->groupBox7->Controls->Add(this->FIlter_2);
				 this->groupBox7->Controls->Add(this->Filter_1);
				 this->groupBox7->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox7->ForeColor = System::Drawing::Color::White;
				 this->groupBox7->Location = System::Drawing::Point(827, 498);
				 this->groupBox7->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox7->Name = L"groupBox7";
				 this->groupBox7->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->groupBox7->Size = System::Drawing::Size(192, 161);
				 this->groupBox7->TabIndex = 25;
				 this->groupBox7->TabStop = false;
				 this->groupBox7->Text = L"Filter";
				 this->groupBox7->Enter += gcnew System::EventHandler(this, &MyForm::groupBox7_Enter);
				 // 
				 // current_filter_box
				 // 
				 this->current_filter_box->Location = System::Drawing::Point(11, 138);
				 this->current_filter_box->Name = L"current_filter_box";
				 this->current_filter_box->ReadOnly = true;
				 this->current_filter_box->Size = System::Drawing::Size(177, 34);
				 this->current_filter_box->TabIndex = 5;
				 this->current_filter_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 // 
				 // FIlter_8
				 // 
				 this->FIlter_8->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_8->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_8->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_8->Location = System::Drawing::Point(104, 109);
				 this->FIlter_8->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_8->Name = L"FIlter_8";
				 this->FIlter_8->Size = System::Drawing::Size(84, 25);
				 this->FIlter_8->TabIndex = 1;
				 this->FIlter_8->UseVisualStyleBackColor = false;
				 this->FIlter_8->Click += gcnew System::EventHandler(this, &MyForm::FIlter_8_Click);
				 // 
				 // FIlter_4
				 // 
				 this->FIlter_4->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_4->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_4->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_4->Location = System::Drawing::Point(11, 108);
				 this->FIlter_4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_4->Name = L"FIlter_4";
				 this->FIlter_4->Size = System::Drawing::Size(87, 25);
				 this->FIlter_4->TabIndex = 1;
				 this->FIlter_4->UseVisualStyleBackColor = false;
				 this->FIlter_4->Click += gcnew System::EventHandler(this, &MyForm::FIlter_4_Click);
				 // 
				 // FIlter_7
				 // 
				 this->FIlter_7->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_7->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_7->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_7->Location = System::Drawing::Point(104, 78);
				 this->FIlter_7->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_7->Name = L"FIlter_7";
				 this->FIlter_7->Size = System::Drawing::Size(84, 25);
				 this->FIlter_7->TabIndex = 1;
				 this->FIlter_7->UseVisualStyleBackColor = false;
				 this->FIlter_7->Click += gcnew System::EventHandler(this, &MyForm::FIlter_7_Click);
				 // 
				 // FIlter_3
				 // 
				 this->FIlter_3->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->FIlter_3->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->FIlter_3->ForeColor = System::Drawing::Color::Black;
				 this->FIlter_3->Location = System::Drawing::Point(11, 78);
				 this->FIlter_3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->FIlter_3->Name = L"FIlter_3";
				 this->FIlter_3->Size = System::Drawing::Size(87, 25);
				 this->FIlter_3->TabIndex = 1;
				 this->FIlter_3->UseVisualStyleBackColor = false;
				 this->FIlter_3->Click += gcnew System::EventHandler(this, &MyForm::FIlter_3_Click);
				 // 
				 // Single_Acquire
				 // 
				 this->Single_Acquire->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(255)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 this->Single_Acquire->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Single_Acquire->Location = System::Drawing::Point(20, 59);
				 this->Single_Acquire->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Single_Acquire->Name = L"Single_Acquire";
				 this->Single_Acquire->Size = System::Drawing::Size(124, 32);
				 this->Single_Acquire->TabIndex = 0;
				 this->Single_Acquire->Text = L"Acquire";
				 this->Single_Acquire->UseVisualStyleBackColor = false;
				 this->Single_Acquire->Click += gcnew System::EventHandler(this, &MyForm::Single_Acquire_Click);
				 // 
				 // Color_check
				 // 
				 this->Color_check->WorkerSupportsCancellation = true;
				 this->Color_check->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Color_check_DoWork);
				 // 
				 // BW_XY_ini
				 // 
				 this->BW_XY_ini->WorkerSupportsCancellation = true;
				 this->BW_XY_ini->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_XY_ini_DoWork);
				 // 
				 // Color_check2
				 // 
				 this->Color_check2->WorkerSupportsCancellation = true;
				 this->Color_check2->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Color_check2_DoWork);
				 // 
				 // BW_Q_Laser
				 // 
				 this->BW_Q_Laser->WorkerSupportsCancellation = true;
				 this->BW_Q_Laser->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_Q_Laser_DoWork);
				 // 
				 // BW_filter_check
				 // 
				 this->BW_filter_check->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_filter_check_DoWork);
				 // 
				 // MicroStep1
				 // 
				 this->MicroStep1->AutoSize = true;
				 this->MicroStep1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep1->ForeColor = System::Drawing::Color::White;
				 this->MicroStep1->Location = System::Drawing::Point(15, 22);
				 this->MicroStep1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep1->Name = L"MicroStep1";
				 this->MicroStep1->Size = System::Drawing::Size(46, 28);
				 this->MicroStep1->TabIndex = 0;
				 this->MicroStep1->Text = L"1";
				 this->MicroStep1->UseVisualStyleBackColor = true;
				 this->MicroStep1->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep1_CheckedChanged);
				 // 
				 // statusbar
				 // 
				 this->statusbar->BackColor = System::Drawing::Color::DimGray;
				 this->statusbar->ImageScalingSize = System::Drawing::Size(24, 24);
				 this->statusbar->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(8) {
					 this->toolStripProgressBar1,
						 this->z_left, this->toolStripStatusLabel3, this->z_right, this->ThreadCondition, this->xpos, this->ypos, this->ThreadCondition2
				 });
				 this->statusbar->Location = System::Drawing::Point(0, 698);
				 this->statusbar->Name = L"statusbar";
				 this->statusbar->Size = System::Drawing::Size(1030, 32);
				 this->statusbar->TabIndex = 27;
				 this->statusbar->Text = L"statusStrip1";
				 // 
				 // toolStripProgressBar1
				 // 
				 this->toolStripProgressBar1->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)),
					 static_cast<System::Int32>(static_cast<System::Byte>(128)), static_cast<System::Int32>(static_cast<System::Byte>(255)));
				 this->toolStripProgressBar1->Name = L"toolStripProgressBar1";
				 this->toolStripProgressBar1->Size = System::Drawing::Size(600, 26);
				 this->toolStripProgressBar1->Step = 1;
				 this->toolStripProgressBar1->Style = System::Windows::Forms::ProgressBarStyle::Marquee;
				 // 
				 // z_left
				 // 
				 this->z_left->ForeColor = System::Drawing::Color::White;
				 this->z_left->ImageScaling = System::Windows::Forms::ToolStripItemImageScaling::None;
				 this->z_left->Name = L"z_left";
				 this->z_left->Size = System::Drawing::Size(77, 27);
				 this->z_left->Text = L" 000000";
				 // 
				 // toolStripStatusLabel3
				 // 
				 this->toolStripStatusLabel3->ForeColor = System::Drawing::Color::White;
				 this->toolStripStatusLabel3->Name = L"toolStripStatusLabel3";
				 this->toolStripStatusLabel3->Size = System::Drawing::Size(29, 27);
				 this->toolStripStatusLabel3->Text = L" / ";
				 // 
				 // z_right
				 // 
				 this->z_right->ForeColor = System::Drawing::Color::White;
				 this->z_right->ImageScaling = System::Windows::Forms::ToolStripItemImageScaling::None;
				 this->z_right->Name = L"z_right";
				 this->z_right->Size = System::Drawing::Size(77, 27);
				 this->z_right->Text = L" 000000";
				 this->z_right->Click += gcnew System::EventHandler(this, &MyForm::z_right_Click);
				 // 
				 // ThreadCondition
				 // 
				 this->ThreadCondition->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->ThreadCondition->ForeColor = System::Drawing::Color::White;
				 this->ThreadCondition->Name = L"ThreadCondition";
				 this->ThreadCondition->Size = System::Drawing::Size(132, 27);
				 this->ThreadCondition->Text = L"                    ";
				 // 
				 // xpos
				 // 
				 this->xpos->Name = L"xpos";
				 this->xpos->Size = System::Drawing::Size(0, 27);
				 // 
				 // ypos
				 // 
				 this->ypos->Name = L"ypos";
				 this->ypos->Size = System::Drawing::Size(0, 27);
				 // 
				 // ThreadCondition2
				 // 
				 this->ThreadCondition2->AutoSize = false;
				 this->ThreadCondition2->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Bold));
				 this->ThreadCondition2->ImageScaling = System::Windows::Forms::ToolStripItemImageScaling::None;
				 this->ThreadCondition2->Name = L"ThreadCondition2";
				 this->ThreadCondition2->Size = System::Drawing::Size(88, 27);
				 // 
				 // menuStrip1
				 // 
				 this->menuStrip1->BackColor = System::Drawing::Color::DarkGray;
				 this->menuStrip1->ImageScalingSize = System::Drawing::Size(24, 24);
				 this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {
					 this->toolStripMenuItem8,
						 this->toolStripMenuItem5, this->toolStripMenuItem2, this->saveFileToolStripMenuItem, this->toolStripMenuItem4, this->resetSettingToolStripMenuItem
				 });
				 this->menuStrip1->Location = System::Drawing::Point(0, 0);
				 this->menuStrip1->Margin = System::Windows::Forms::Padding(20, 0, 0, 0);
				 this->menuStrip1->Name = L"menuStrip1";
				 this->menuStrip1->Size = System::Drawing::Size(1030, 36);
				 this->menuStrip1->TabIndex = 26;
				 this->menuStrip1->Text = L"menuStrip1";
				 // 
				 // toolStripMenuItem8
				 // 
				 this->toolStripMenuItem8->BackColor = System::Drawing::Color::DarkGray;
				 this->toolStripMenuItem8->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
					 this->toolStripSeparator2,
						 this->toolStripMenuItem9
				 });
				 this->toolStripMenuItem8->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem8->ForeColor = System::Drawing::Color::Black;
				 this->toolStripMenuItem8->Name = L"toolStripMenuItem8";
				 this->toolStripMenuItem8->Padding = System::Windows::Forms::Padding(30, 0, 30, 0);
				 this->toolStripMenuItem8->Size = System::Drawing::Size(122, 32);
				 this->toolStripMenuItem8->Text = L"File";
				 this->toolStripMenuItem8->Click += gcnew System::EventHandler(this, &MyForm::toolStripMenuItem1_Click);
				 // 
				 // toolStripSeparator2
				 // 
				 this->toolStripSeparator2->Name = L"toolStripSeparator2";
				 this->toolStripSeparator2->Size = System::Drawing::Size(285, 6);
				 // 
				 // toolStripMenuItem9
				 // 
				 this->toolStripMenuItem9->BackColor = System::Drawing::SystemColors::MenuBar;
				 this->toolStripMenuItem9->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem9->Name = L"toolStripMenuItem9";
				 this->toolStripMenuItem9->Size = System::Drawing::Size(288, 30);
				 this->toolStripMenuItem9->Text = L"Open Image file";
				 this->toolStripMenuItem9->Click += gcnew System::EventHandler(this, &MyForm::openImageFileToolStripMenuItem_Click);
				 // 
				 // toolStripMenuItem5
				 // 
				 this->toolStripMenuItem5->BackColor = System::Drawing::Color::DarkGray;
				 this->toolStripMenuItem5->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {
					 this->toolStripMenuItem6,
						 this->toolStripMenuItem7, this->eTLAxialToolStripMenuItem
				 });
				 this->toolStripMenuItem5->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem5->ForeColor = System::Drawing::Color::Black;
				 this->toolStripMenuItem5->Name = L"toolStripMenuItem5";
				 this->toolStripMenuItem5->Padding = System::Windows::Forms::Padding(10, 0, 10, 0);
				 this->toolStripMenuItem5->Size = System::Drawing::Size(143, 32);
				 this->toolStripMenuItem5->Text = L"Load file";
				 // 
				 // toolStripMenuItem6
				 // 
				 this->toolStripMenuItem6->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem6->Name = L"toolStripMenuItem6";
				 this->toolStripMenuItem6->Size = System::Drawing::Size(232, 32);
				 this->toolStripMenuItem6->Text = L"Log file";
				 this->toolStripMenuItem6->Click += gcnew System::EventHandler(this, &MyForm::logFileToolStripMenuItem_Click);
				 // 
				 // toolStripMenuItem7
				 // 
				 this->toolStripMenuItem7->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem7->Name = L"toolStripMenuItem7";
				 this->toolStripMenuItem7->Size = System::Drawing::Size(232, 32);
				 this->toolStripMenuItem7->Text = L"Setting file";
				 this->toolStripMenuItem7->Click += gcnew System::EventHandler(this, &MyForm::settingFileToolStripMenuItem_Click);
				 // 
				 // eTLAxialToolStripMenuItem
				 // 
				 this->eTLAxialToolStripMenuItem->Name = L"eTLAxialToolStripMenuItem";
				 this->eTLAxialToolStripMenuItem->Size = System::Drawing::Size(232, 32);
				 this->eTLAxialToolStripMenuItem->Text = L"ETL-Axial";
				 this->eTLAxialToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::eTLAxialToolStripMenuItem_Click);
				 // 
				 // toolStripMenuItem2
				 // 
				 this->toolStripMenuItem2->BackColor = System::Drawing::Color::DarkGray;
				 this->toolStripMenuItem2->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem2->Name = L"toolStripMenuItem2";
				 this->toolStripMenuItem2->Size = System::Drawing::Size(31, 32);
				 this->toolStripMenuItem2->Text = L" ";
				 // 
				 // saveFileToolStripMenuItem
				 // 
				 this->saveFileToolStripMenuItem->BackColor = System::Drawing::Color::DarkGray;
				 this->saveFileToolStripMenuItem->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold,
					 System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(128)));
				 this->saveFileToolStripMenuItem->ForeColor = System::Drawing::Color::Black;
				 this->saveFileToolStripMenuItem->Name = L"saveFileToolStripMenuItem";
				 this->saveFileToolStripMenuItem->Padding = System::Windows::Forms::Padding(10, 0, 10, 0);
				 this->saveFileToolStripMenuItem->Size = System::Drawing::Size(142, 32);
				 this->saveFileToolStripMenuItem->Text = L"Save file";
				 this->saveFileToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::saveFileToolStripMenuItem_Click);
				 // 
				 // toolStripMenuItem4
				 // 
				 this->toolStripMenuItem4->BackColor = System::Drawing::Color::DarkGray;
				 this->toolStripMenuItem4->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->toolStripMenuItem4->Name = L"toolStripMenuItem4";
				 this->toolStripMenuItem4->Size = System::Drawing::Size(31, 32);
				 this->toolStripMenuItem4->Text = L" ";
				 // 
				 // resetSettingToolStripMenuItem
				 // 
				 this->resetSettingToolStripMenuItem->BackColor = System::Drawing::Color::DarkGray;
				 this->resetSettingToolStripMenuItem->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 12, System::Drawing::FontStyle::Bold,
					 System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(128)));
				 this->resetSettingToolStripMenuItem->ForeColor = System::Drawing::Color::Black;
				 this->resetSettingToolStripMenuItem->Name = L"resetSettingToolStripMenuItem";
				 this->resetSettingToolStripMenuItem->Padding = System::Windows::Forms::Padding(10, 0, 10, 0);
				 this->resetSettingToolStripMenuItem->Size = System::Drawing::Size(199, 32);
				 this->resetSettingToolStripMenuItem->Text = L"Reset setting";
				 this->resetSettingToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::resetSettingToolStripMenuItem_Click_1);
				 // 
				 // BW_Cursor
				 // 
				 this->BW_Cursor->WorkerSupportsCancellation = true;
				 this->BW_Cursor->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_Cursor_DoWork);
				 // 
				 // Progress_count
				 // 
				 this->Progress_count->WorkerReportsProgress = true;
				 this->Progress_count->WorkerSupportsCancellation = true;
				 this->Progress_count->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Progress_count_DoWork);
				 this->Progress_count->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &MyForm::Progress_count_ProgressChanged);
				 // 
				 // groupBox8
				 // 
				 this->groupBox8->BackColor = System::Drawing::Color::Gray;
				 this->groupBox8->Controls->Add(this->ETL_box2);
				 this->groupBox8->Controls->Add(this->label50);
				 this->groupBox8->Controls->Add(this->trackBar1);
				 this->groupBox8->Controls->Add(this->label28);
				 this->groupBox8->Controls->Add(this->label54);
				 this->groupBox8->Controls->Add(this->label53);
				 this->groupBox8->Controls->Add(this->label29);
				 this->groupBox8->Controls->Add(this->Z_offset_current);
				 this->groupBox8->Controls->Add(this->ETL_box);
				 this->groupBox8->Controls->Add(this->Z_offset);
				 this->groupBox8->Controls->Add(this->Axial_Param_set_R);
				 this->groupBox8->Controls->Add(this->label33);
				 this->groupBox8->Controls->Add(this->label15);
				 this->groupBox8->Controls->Add(this->Axial_Param_set);
				 this->groupBox8->Controls->Add(this->trackBar_Z);
				 this->groupBox8->Controls->Add(this->ETL_bar);
				 this->groupBox8->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 11.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox8->ForeColor = System::Drawing::Color::White;
				 this->groupBox8->Location = System::Drawing::Point(263, 359);
				 this->groupBox8->Name = L"groupBox8";
				 this->groupBox8->Size = System::Drawing::Size(479, 132);
				 this->groupBox8->TabIndex = 29;
				 this->groupBox8->TabStop = false;
				 this->groupBox8->Text = L"Focus setting";
				 // 
				 // ETL_box2
				 // 
				 this->ETL_box2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_box2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_box2->ForeColor = System::Drawing::Color::Black;
				 this->ETL_box2->Location = System::Drawing::Point(198, 79);
				 this->ETL_box2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_box2->Name = L"ETL_box2";
				 this->ETL_box2->ReadOnly = true;
				 this->ETL_box2->Size = System::Drawing::Size(35, 31);
				 this->ETL_box2->TabIndex = 102;
				 this->ETL_box2->Text = L"1.00";
				 this->ETL_box2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_box2->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_box2_TextChanged);
				 // 
				 // label50
				 // 
				 this->label50->AutoSize = true;
				 this->label50->BackColor = System::Drawing::Color::Transparent;
				 this->label50->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label50->ForeColor = System::Drawing::Color::White;
				 this->label50->Location = System::Drawing::Point(237, 87);
				 this->label50->Name = L"label50";
				 this->label50->Size = System::Drawing::Size(38, 24);
				 this->label50->TabIndex = 101;
				 this->label50->Text = L"mA";
				 // 
				 // trackBar1
				 // 
				 this->trackBar1->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->trackBar1->Location = System::Drawing::Point(18, 84);
				 this->trackBar1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->trackBar1->Maximum = 1000;
				 this->trackBar1->Name = L"trackBar1";
				 this->trackBar1->Size = System::Drawing::Size(176, 69);
				 this->trackBar1->TabIndex = 100;
				 this->trackBar1->TickFrequency = 50;
				 this->trackBar1->Value = 100;
				 this->trackBar1->Scroll += gcnew System::EventHandler(this, &MyForm::trackBar1_Scroll);
				 // 
				 // label28
				 // 
				 this->label28->AutoSize = true;
				 this->label28->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label28->ForeColor = System::Drawing::Color::White;
				 this->label28->Location = System::Drawing::Point(270, 16);
				 this->label28->Name = L"label28";
				 this->label28->Size = System::Drawing::Size(75, 29);
				 this->label28->TabIndex = 3;
				 this->label28->Text = L"Z-Galv";
				 this->label28->Click += gcnew System::EventHandler(this, &MyForm::label29_Click);
				 // 
				 // label54
				 // 
				 this->label54->AutoSize = true;
				 this->label54->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label54->ForeColor = System::Drawing::Color::White;
				 this->label54->Location = System::Drawing::Point(4, 39);
				 this->label54->Name = L"label54";
				 this->label54->Size = System::Drawing::Size(27, 29);
				 this->label54->TabIndex = 3;
				 this->label54->Text = L"R";
				 this->label54->Click += gcnew System::EventHandler(this, &MyForm::label29_Click);
				 // 
				 // label53
				 // 
				 this->label53->AutoSize = true;
				 this->label53->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label53->ForeColor = System::Drawing::Color::White;
				 this->label53->Location = System::Drawing::Point(6, 87);
				 this->label53->Name = L"label53";
				 this->label53->Size = System::Drawing::Size(23, 29);
				 this->label53->TabIndex = 3;
				 this->label53->Text = L"L";
				 this->label53->Click += gcnew System::EventHandler(this, &MyForm::label29_Click);
				 // 
				 // Z_offset_current
				 // 
				 this->Z_offset_current->BackColor = System::Drawing::SystemColors::Control;
				 this->Z_offset_current->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_offset_current->ForeColor = System::Drawing::Color::Black;
				 this->Z_offset_current->Location = System::Drawing::Point(333, 88);
				 this->Z_offset_current->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_offset_current->Name = L"Z_offset_current";
				 this->Z_offset_current->ReadOnly = true;
				 this->Z_offset_current->Size = System::Drawing::Size(78, 31);
				 this->Z_offset_current->TabIndex = 11;
				 this->Z_offset_current->Text = L"50";
				 this->Z_offset_current->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_offset_current->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_offset_current_TextChanged);
				 // 
				 // Z_offset
				 // 
				 this->Z_offset->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Z_offset->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_offset->ForeColor = System::Drawing::Color::Black;
				 this->Z_offset->Location = System::Drawing::Point(333, 26);
				 this->Z_offset->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_offset->Name = L"Z_offset";
				 this->Z_offset->Size = System::Drawing::Size(60, 31);
				 this->Z_offset->TabIndex = 4;
				 this->Z_offset->Text = L"0";
				 this->Z_offset->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_offset->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_offset_TextChanged);
				 // 
				 // Axial_Param_set_R
				 // 
				 this->Axial_Param_set_R->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Axial_Param_set_R->ForeColor = System::Drawing::Color::Black;
				 this->Axial_Param_set_R->Location = System::Drawing::Point(197, 102);
				 this->Axial_Param_set_R->Name = L"Axial_Param_set_R";
				 this->Axial_Param_set_R->Size = System::Drawing::Size(39, 24);
				 this->Axial_Param_set_R->TabIndex = 1;
				 this->Axial_Param_set_R->Text = L"SET";
				 this->Axial_Param_set_R->UseVisualStyleBackColor = true;
				 this->Axial_Param_set_R->Click += gcnew System::EventHandler(this, &MyForm::Axial_Param_set_R_Click);
				 // 
				 // label33
				 // 
				 this->label33->AutoSize = true;
				 this->label33->BackColor = System::Drawing::Color::Transparent;
				 this->label33->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label33->ForeColor = System::Drawing::Color::White;
				 this->label33->Location = System::Drawing::Point(238, 33);
				 this->label33->Name = L"label33";
				 this->label33->Size = System::Drawing::Size(38, 24);
				 this->label33->TabIndex = 3;
				 this->label33->Text = L"mA";
				 this->label33->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label15
				 // 
				 this->label15->AutoSize = true;
				 this->label15->BackColor = System::Drawing::Color::Transparent;
				 this->label15->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label15->ForeColor = System::Drawing::Color::White;
				 this->label15->Location = System::Drawing::Point(404, 34);
				 this->label15->Name = L"label15";
				 this->label15->Size = System::Drawing::Size(38, 24);
				 this->label15->TabIndex = 3;
				 this->label15->Text = L"mV";
				 this->label15->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // Axial_Param_set
				 // 
				 this->Axial_Param_set->Font = (gcnew System::Drawing::Font(L"UD デジタル 教科書体 NK-B", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Axial_Param_set->ForeColor = System::Drawing::Color::Black;
				 this->Axial_Param_set->Location = System::Drawing::Point(196, 51);
				 this->Axial_Param_set->Name = L"Axial_Param_set";
				 this->Axial_Param_set->Size = System::Drawing::Size(40, 24);
				 this->Axial_Param_set->TabIndex = 1;
				 this->Axial_Param_set->Text = L"SET";
				 this->Axial_Param_set->UseVisualStyleBackColor = true;
				 this->Axial_Param_set->Click += gcnew System::EventHandler(this, &MyForm::Axial_Param_set_Click);
				 // 
				 // trackBar_Z
				 // 
				 this->trackBar_Z->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->trackBar_Z->LargeChange = 50;
				 this->trackBar_Z->Location = System::Drawing::Point(261, 50);
				 this->trackBar_Z->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->trackBar_Z->Maximum = 20000;
				 this->trackBar_Z->Name = L"trackBar_Z";
				 this->trackBar_Z->Size = System::Drawing::Size(215, 69);
				 this->trackBar_Z->SmallChange = 10;
				 this->trackBar_Z->TabIndex = 2;
				 this->trackBar_Z->TickFrequency = 2000;
				 this->trackBar_Z->Value = 10000;
				 this->trackBar_Z->Scroll += gcnew System::EventHandler(this, &MyForm::trackBar_Z_Scroll);
				 // 
				 // filter_ini
				 // 
				 this->filter_ini->WorkerSupportsCancellation = true;
				 this->filter_ini->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::filter_ini_DoWork);
				 // 
				 // Laser_ini
				 // 
				 this->Laser_ini->WorkerSupportsCancellation = true;
				 this->Laser_ini->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Laser_ini_DoWork);
				 // 
				 // BW_ActiveWindow
				 // 
				 this->BW_ActiveWindow->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_ActiveWindow_DoWork);
				 // 
				 // Camera_ini
				 // 
				 this->Camera_ini->WorkerSupportsCancellation = true;
				 this->Camera_ini->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Camera_ini_DoWork);
				 // 
				 // tabPage2
				 // 
				 this->tabPage2->BackColor = System::Drawing::Color::Gray;
				 this->tabPage2->Controls->Add(this->LIVE_window_close2);
				 this->tabPage2->Controls->Add(this->LIVE_window_close1);
				 this->tabPage2->Controls->Add(this->SH_low1_bar);
				 this->tabPage2->Controls->Add(this->SH_low2_bar);
				 this->tabPage2->Controls->Add(this->SH_high2_bar);
				 this->tabPage2->Controls->Add(this->SH_high1_bar);
				 this->tabPage2->Controls->Add(this->SH_low2);
				 this->tabPage2->Controls->Add(this->SH_high2);
				 this->tabPage2->Controls->Add(this->SH_low1);
				 this->tabPage2->Controls->Add(this->SH_high1);
				 this->tabPage2->Controls->Add(this->manual_scale2);
				 this->tabPage2->Controls->Add(this->manual_scale1);
				 this->tabPage2->Controls->Add(this->ResizeWindow);
				 this->tabPage2->Location = System::Drawing::Point(4, 33);
				 this->tabPage2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->tabPage2->Name = L"tabPage2";
				 this->tabPage2->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->tabPage2->Size = System::Drawing::Size(471, 283);
				 this->tabPage2->TabIndex = 5;
				 this->tabPage2->Text = L"LIVE";
				 // 
				 // LIVE_window_close2
				 // 
				 this->LIVE_window_close2->BackColor = System::Drawing::Color::LightGray;
				 this->LIVE_window_close2->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LIVE_window_close2->Location = System::Drawing::Point(15, 212);
				 this->LIVE_window_close2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LIVE_window_close2->Name = L"LIVE_window_close2";
				 this->LIVE_window_close2->Size = System::Drawing::Size(94, 30);
				 this->LIVE_window_close2->TabIndex = 6;
				 this->LIVE_window_close2->Text = L"Close";
				 this->LIVE_window_close2->UseVisualStyleBackColor = false;
				 this->LIVE_window_close2->Click += gcnew System::EventHandler(this, &MyForm::LIVE_window_close2_Click);
				 // 
				 // LIVE_window_close1
				 // 
				 this->LIVE_window_close1->BackColor = System::Drawing::Color::LightGray;
				 this->LIVE_window_close1->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->LIVE_window_close1->Location = System::Drawing::Point(15, 95);
				 this->LIVE_window_close1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LIVE_window_close1->Name = L"LIVE_window_close1";
				 this->LIVE_window_close1->Size = System::Drawing::Size(94, 30);
				 this->LIVE_window_close1->TabIndex = 6;
				 this->LIVE_window_close1->Text = L"Close";
				 this->LIVE_window_close1->UseVisualStyleBackColor = false;
				 this->LIVE_window_close1->Click += gcnew System::EventHandler(this, &MyForm::LIVE_window_close1_Click);
				 // 
				 // SH_low1_bar
				 // 
				 this->SH_low1_bar->BackColor = System::Drawing::Color::Gray;
				 this->SH_low1_bar->LargeChange = 500;
				 this->SH_low1_bar->Location = System::Drawing::Point(6, 60);
				 this->SH_low1_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_low1_bar->Maximum = 65535;
				 this->SH_low1_bar->Minimum = 10;
				 this->SH_low1_bar->Name = L"SH_low1_bar";
				 this->SH_low1_bar->Size = System::Drawing::Size(281, 69);
				 this->SH_low1_bar->SmallChange = 100;
				 this->SH_low1_bar->TabIndex = 2;
				 this->SH_low1_bar->TickFrequency = 5000;
				 this->SH_low1_bar->Value = 50;
				 this->SH_low1_bar->Scroll += gcnew System::EventHandler(this, &MyForm::SH_low1_bar_Scroll);
				 // 
				 // SH_low2_bar
				 // 
				 this->SH_low2_bar->BackColor = System::Drawing::Color::Gray;
				 this->SH_low2_bar->LargeChange = 500;
				 this->SH_low2_bar->Location = System::Drawing::Point(7, 182);
				 this->SH_low2_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_low2_bar->Maximum = 65535;
				 this->SH_low2_bar->Minimum = 10;
				 this->SH_low2_bar->Name = L"SH_low2_bar";
				 this->SH_low2_bar->Size = System::Drawing::Size(280, 69);
				 this->SH_low2_bar->SmallChange = 100;
				 this->SH_low2_bar->TabIndex = 2;
				 this->SH_low2_bar->TickFrequency = 5000;
				 this->SH_low2_bar->Value = 50;
				 this->SH_low2_bar->Scroll += gcnew System::EventHandler(this, &MyForm::SH_low2_bar_Scroll);
				 // 
				 // SH_high2_bar
				 // 
				 this->SH_high2_bar->BackColor = System::Drawing::Color::Gray;
				 this->SH_high2_bar->LargeChange = 500;
				 this->SH_high2_bar->Location = System::Drawing::Point(7, 149);
				 this->SH_high2_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_high2_bar->Maximum = 65535;
				 this->SH_high2_bar->Minimum = 10;
				 this->SH_high2_bar->Name = L"SH_high2_bar";
				 this->SH_high2_bar->Size = System::Drawing::Size(280, 69);
				 this->SH_high2_bar->SmallChange = 100;
				 this->SH_high2_bar->TabIndex = 2;
				 this->SH_high2_bar->TickFrequency = 5000;
				 this->SH_high2_bar->Value = 65535;
				 this->SH_high2_bar->Scroll += gcnew System::EventHandler(this, &MyForm::SH_high2_bar_Scroll);
				 // 
				 // SH_high1_bar
				 // 
				 this->SH_high1_bar->BackColor = System::Drawing::Color::Gray;
				 this->SH_high1_bar->LargeChange = 500;
				 this->SH_high1_bar->Location = System::Drawing::Point(7, 32);
				 this->SH_high1_bar->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_high1_bar->Maximum = 65535;
				 this->SH_high1_bar->Minimum = 10;
				 this->SH_high1_bar->Name = L"SH_high1_bar";
				 this->SH_high1_bar->Size = System::Drawing::Size(281, 69);
				 this->SH_high1_bar->SmallChange = 100;
				 this->SH_high1_bar->TabIndex = 2;
				 this->SH_high1_bar->TickFrequency = 5000;
				 this->SH_high1_bar->Value = 65535;
				 this->SH_high1_bar->Scroll += gcnew System::EventHandler(this, &MyForm::SH_high1_bar_Scroll);
				 // 
				 // SH_low2
				 // 
				 this->SH_low2->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SH_low2->Location = System::Drawing::Point(313, 182);
				 this->SH_low2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_low2->Name = L"SH_low2";
				 this->SH_low2->Size = System::Drawing::Size(54, 27);
				 this->SH_low2->TabIndex = 0;
				 this->SH_low2->Text = L"0";
				 this->SH_low2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->SH_low2->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::SH_low2_KeyDown);
				 // 
				 // SH_high2
				 // 
				 this->SH_high2->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SH_high2->Location = System::Drawing::Point(313, 149);
				 this->SH_high2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_high2->Name = L"SH_high2";
				 this->SH_high2->Size = System::Drawing::Size(54, 27);
				 this->SH_high2->TabIndex = 0;
				 this->SH_high2->Text = L"65535";
				 this->SH_high2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->SH_high2->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::SH_high2_KeyDown);
				 // 
				 // SH_low1
				 // 
				 this->SH_low1->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SH_low1->Location = System::Drawing::Point(313, 65);
				 this->SH_low1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_low1->Name = L"SH_low1";
				 this->SH_low1->Size = System::Drawing::Size(54, 27);
				 this->SH_low1->TabIndex = 0;
				 this->SH_low1->Text = L"0";
				 this->SH_low1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->SH_low1->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::SH_low1_KeyDown);
				 // 
				 // SH_high1
				 // 
				 this->SH_high1->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SH_high1->Location = System::Drawing::Point(313, 34);
				 this->SH_high1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->SH_high1->Name = L"SH_high1";
				 this->SH_high1->Size = System::Drawing::Size(54, 27);
				 this->SH_high1->TabIndex = 0;
				 this->SH_high1->Text = L"65535";
				 this->SH_high1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->SH_high1->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::SH_high1_KeyDown);
				 // 
				 // manual_scale2
				 // 
				 this->manual_scale2->AutoSize = true;
				 this->manual_scale2->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->manual_scale2->ForeColor = System::Drawing::Color::White;
				 this->manual_scale2->Location = System::Drawing::Point(15, 132);
				 this->manual_scale2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->manual_scale2->Name = L"manual_scale2";
				 this->manual_scale2->Size = System::Drawing::Size(158, 27);
				 this->manual_scale2->TabIndex = 17;
				 this->manual_scale2->Text = L"manual scale";
				 this->manual_scale2->UseVisualStyleBackColor = true;
				 // 
				 // manual_scale1
				 // 
				 this->manual_scale1->AutoSize = true;
				 this->manual_scale1->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->manual_scale1->ForeColor = System::Drawing::Color::White;
				 this->manual_scale1->Location = System::Drawing::Point(15, 13);
				 this->manual_scale1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->manual_scale1->Name = L"manual_scale1";
				 this->manual_scale1->Size = System::Drawing::Size(158, 27);
				 this->manual_scale1->TabIndex = 17;
				 this->manual_scale1->Text = L"manual scale";
				 this->manual_scale1->UseVisualStyleBackColor = true;
				 // 
				 // ResizeWindow
				 // 
				 this->ResizeWindow->BackColor = System::Drawing::Color::LightGray;
				 this->ResizeWindow->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ResizeWindow->Location = System::Drawing::Point(290, 210);
				 this->ResizeWindow->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ResizeWindow->Name = L"ResizeWindow";
				 this->ResizeWindow->Size = System::Drawing::Size(94, 32);
				 this->ResizeWindow->TabIndex = 6;
				 this->ResizeWindow->Text = L"Reset";
				 this->ResizeWindow->UseVisualStyleBackColor = false;
				 this->ResizeWindow->Click += gcnew System::EventHandler(this, &MyForm::ResizeWindow_Click);
				 // 
				 // singlestackmode
				 // 
				 this->singlestackmode->AutoSize = true;
				 this->singlestackmode->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->singlestackmode->ForeColor = System::Drawing::Color::White;
				 this->singlestackmode->Location = System::Drawing::Point(12, 117);
				 this->singlestackmode->Name = L"singlestackmode";
				 this->singlestackmode->Size = System::Drawing::Size(146, 26);
				 this->singlestackmode->TabIndex = 49;
				 this->singlestackmode->Text = L"Single Stack";
				 this->singlestackmode->UseVisualStyleBackColor = true;
				 // 
				 // imagingtile
				 // 
				 this->imagingtile->BackColor = System::Drawing::Color::LightGray;
				 this->imagingtile->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->imagingtile->Location = System::Drawing::Point(198, 119);
				 this->imagingtile->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->imagingtile->Name = L"imagingtile";
				 this->imagingtile->ReadOnly = true;
				 this->imagingtile->Size = System::Drawing::Size(32, 31);
				 this->imagingtile->TabIndex = 20;
				 this->imagingtile->Text = L"0";
				 this->imagingtile->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // ReStart_tile_num_box
				 // 
				 this->ReStart_tile_num_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ReStart_tile_num_box->Location = System::Drawing::Point(198, 97);
				 this->ReStart_tile_num_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ReStart_tile_num_box->Name = L"ReStart_tile_num_box";
				 this->ReStart_tile_num_box->Size = System::Drawing::Size(32, 31);
				 this->ReStart_tile_num_box->TabIndex = 20;
				 this->ReStart_tile_num_box->Text = L"0";
				 this->ReStart_tile_num_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // MOVIE_strat
				 // 
				 this->MOVIE_strat->BackColor = System::Drawing::Color::DarkGreen;
				 this->MOVIE_strat->Font = (gcnew System::Drawing::Font(L"Calibri", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MOVIE_strat->Location = System::Drawing::Point(10, 178);
				 this->MOVIE_strat->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MOVIE_strat->Name = L"MOVIE_strat";
				 this->MOVIE_strat->Size = System::Drawing::Size(109, 38);
				 this->MOVIE_strat->TabIndex = 6;
				 this->MOVIE_strat->Text = L"Acquire";
				 this->MOVIE_strat->UseVisualStyleBackColor = false;
				 this->MOVIE_strat->Click += gcnew System::EventHandler(this, &MyForm::MOVIE_strat_Click);
				 // 
				 // nama
				 // 
				 this->nama->AutoSize = true;
				 this->nama->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->nama->ForeColor = System::Drawing::Color::White;
				 this->nama->Location = System::Drawing::Point(237, 123);
				 this->nama->Name = L"nama";
				 this->nama->Size = System::Drawing::Size(36, 24);
				 this->nama->TabIndex = 9;
				 this->nama->Text = L"tile";
				 // 
				 // label73
				 // 
				 this->label73->AutoSize = true;
				 this->label73->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label73->ForeColor = System::Drawing::Color::White;
				 this->label73->Location = System::Drawing::Point(143, 101);
				 this->label73->Name = L"label73";
				 this->label73->Size = System::Drawing::Size(81, 24);
				 this->label73->TabIndex = 9;
				 this->label73->Text = L"Start tile";
				 // 
				 // Axial_mode_check
				 // 
				 this->Axial_mode_check->AutoSize = true;
				 this->Axial_mode_check->Checked = true;
				 this->Axial_mode_check->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->Axial_mode_check->ForeColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Axial_mode_check->Location = System::Drawing::Point(153, 104);
				 this->Axial_mode_check->Name = L"Axial_mode_check";
				 this->Axial_mode_check->Size = System::Drawing::Size(133, 24);
				 this->Axial_mode_check->TabIndex = 105;
				 this->Axial_mode_check->Text = L"Axial sweep";
				 this->Axial_mode_check->UseVisualStyleBackColor = true;
				 this->Axial_mode_check->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Axial_mode_check_CheckedChanged);
				 // 
				 // aa
				 // 
				 this->aa->Controls->Add(this->tabPage3);
				 this->aa->Controls->Add(this->tabPage2);
				 this->aa->Controls->Add(this->tabPage1);
				 this->aa->Controls->Add(this->Camera);
				 this->aa->Controls->Add(this->Calibration);
				 this->aa->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->aa->Location = System::Drawing::Point(263, 32);
				 this->aa->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->aa->Name = L"aa";
				 this->aa->SelectedIndex = 0;
				 this->aa->Size = System::Drawing::Size(479, 320);
				 this->aa->SizeMode = System::Windows::Forms::TabSizeMode::FillToRight;
				 this->aa->TabIndex = 15;
				 // 
				 // tabPage3
				 // 
				 this->tabPage3->BackColor = System::Drawing::Color::DimGray;
				 this->tabPage3->Controls->Add(this->groupBox1);
				 this->tabPage3->Controls->Add(this->Axial_647_load);
				 this->tabPage3->Controls->Add(this->groupBox16);
				 this->tabPage3->Controls->Add(this->AxialAF_but);
				 this->tabPage3->Controls->Add(this->Axial_592_load);
				 this->tabPage3->Controls->Add(this->ETL_R_right);
				 this->tabPage3->Controls->Add(this->Axial_panel);
				 this->tabPage3->Controls->Add(this->Axial_488_load);
				 this->tabPage3->Controls->Add(this->ETL_L_right);
				 this->tabPage3->Controls->Add(this->ETL_R_left);
				 this->tabPage3->Controls->Add(this->ETL_L_left);
				 this->tabPage3->Controls->Add(this->label10);
				 this->tabPage3->Controls->Add(this->label26);
				 this->tabPage3->Controls->Add(this->label13);
				 this->tabPage3->Controls->Add(this->label9);
				 this->tabPage3->Location = System::Drawing::Point(4, 33);
				 this->tabPage3->Name = L"tabPage3";
				 this->tabPage3->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage3->Size = System::Drawing::Size(471, 283);
				 this->tabPage3->TabIndex = 10;
				 this->tabPage3->Text = L"x10";
				 this->tabPage3->Click += gcnew System::EventHandler(this, &MyForm::tabPage3_Click);
				 // 
				 // groupBox1
				 // 
				 this->groupBox1->Controls->Add(this->z_galv_set_rv);
				 this->groupBox1->Controls->Add(this->label48);
				 this->groupBox1->Controls->Add(this->Z_galv_L_RV);
				 this->groupBox1->Controls->Add(this->Z_galv_R_RV);
				 this->groupBox1->Controls->Add(this->label49);
				 this->groupBox1->Controls->Add(this->z_galv_set);
				 this->groupBox1->Controls->Add(this->label30);
				 this->groupBox1->Controls->Add(this->Z_galv_L);
				 this->groupBox1->Controls->Add(this->Z_galv_R);
				 this->groupBox1->Controls->Add(this->label34);
				 this->groupBox1->ForeColor = System::Drawing::Color::White;
				 this->groupBox1->Location = System::Drawing::Point(275, 217);
				 this->groupBox1->Name = L"groupBox1";
				 this->groupBox1->Size = System::Drawing::Size(182, 74);
				 this->groupBox1->TabIndex = 21;
				 this->groupBox1->TabStop = false;
				 this->groupBox1->Text = L"z-Galv setting";
				 // 
				 // z_galv_set_rv
				 // 
				 this->z_galv_set_rv->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->z_galv_set_rv->ForeColor = System::Drawing::Color::Black;
				 this->z_galv_set_rv->Location = System::Drawing::Point(125, 42);
				 this->z_galv_set_rv->Name = L"z_galv_set_rv";
				 this->z_galv_set_rv->Size = System::Drawing::Size(52, 24);
				 this->z_galv_set_rv->TabIndex = 5;
				 this->z_galv_set_rv->Text = L"RV: SET";
				 this->z_galv_set_rv->UseVisualStyleBackColor = true;
				 this->z_galv_set_rv->Click += gcnew System::EventHandler(this, &MyForm::z_galv_set_rv_Click);
				 // 
				 // label48
				 // 
				 this->label48->AutoSize = true;
				 this->label48->BackColor = System::Drawing::Color::Transparent;
				 this->label48->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label48->ForeColor = System::Drawing::Color::White;
				 this->label48->Location = System::Drawing::Point(7, 47);
				 this->label48->Name = L"label48";
				 this->label48->Size = System::Drawing::Size(18, 24);
				 this->label48->TabIndex = 6;
				 this->label48->Text = L"L";
				 // 
				 // Z_galv_L_RV
				 // 
				 this->Z_galv_L_RV->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Z_galv_L_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_galv_L_RV->ForeColor = System::Drawing::Color::Black;
				 this->Z_galv_L_RV->Location = System::Drawing::Point(23, 43);
				 this->Z_galv_L_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_galv_L_RV->Name = L"Z_galv_L_RV";
				 this->Z_galv_L_RV->ReadOnly = true;
				 this->Z_galv_L_RV->Size = System::Drawing::Size(39, 31);
				 this->Z_galv_L_RV->TabIndex = 8;
				 this->Z_galv_L_RV->Text = L"0";
				 this->Z_galv_L_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_galv_L_RV->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_galv_L_RV_TextChanged);
				 // 
				 // Z_galv_R_RV
				 // 
				 this->Z_galv_R_RV->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Z_galv_R_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_galv_R_RV->ForeColor = System::Drawing::Color::Black;
				 this->Z_galv_R_RV->Location = System::Drawing::Point(80, 43);
				 this->Z_galv_R_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_galv_R_RV->Name = L"Z_galv_R_RV";
				 this->Z_galv_R_RV->ReadOnly = true;
				 this->Z_galv_R_RV->Size = System::Drawing::Size(39, 31);
				 this->Z_galv_R_RV->TabIndex = 9;
				 this->Z_galv_R_RV->Text = L"0";
				 this->Z_galv_R_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_galv_R_RV->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_galv_R_RV_TextChanged);
				 // 
				 // label49
				 // 
				 this->label49->AutoSize = true;
				 this->label49->BackColor = System::Drawing::Color::Transparent;
				 this->label49->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label49->ForeColor = System::Drawing::Color::White;
				 this->label49->Location = System::Drawing::Point(64, 47);
				 this->label49->Name = L"label49";
				 this->label49->Size = System::Drawing::Size(21, 24);
				 this->label49->TabIndex = 7;
				 this->label49->Text = L"R";
				 // 
				 // z_galv_set
				 // 
				 this->z_galv_set->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->z_galv_set->ForeColor = System::Drawing::Color::Black;
				 this->z_galv_set->Location = System::Drawing::Point(125, 17);
				 this->z_galv_set->Name = L"z_galv_set";
				 this->z_galv_set->Size = System::Drawing::Size(52, 24);
				 this->z_galv_set->TabIndex = 1;
				 this->z_galv_set->Text = L"FW: SET";
				 this->z_galv_set->UseVisualStyleBackColor = true;
				 this->z_galv_set->Click += gcnew System::EventHandler(this, &MyForm::z_galv_set_Click);
				 // 
				 // label30
				 // 
				 this->label30->AutoSize = true;
				 this->label30->BackColor = System::Drawing::Color::Transparent;
				 this->label30->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label30->ForeColor = System::Drawing::Color::White;
				 this->label30->Location = System::Drawing::Point(7, 22);
				 this->label30->Name = L"label30";
				 this->label30->Size = System::Drawing::Size(18, 24);
				 this->label30->TabIndex = 3;
				 this->label30->Text = L"L";
				 this->label30->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // Z_galv_L
				 // 
				 this->Z_galv_L->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Z_galv_L->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_galv_L->ForeColor = System::Drawing::Color::Black;
				 this->Z_galv_L->Location = System::Drawing::Point(23, 18);
				 this->Z_galv_L->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_galv_L->Name = L"Z_galv_L";
				 this->Z_galv_L->ReadOnly = true;
				 this->Z_galv_L->Size = System::Drawing::Size(39, 31);
				 this->Z_galv_L->TabIndex = 4;
				 this->Z_galv_L->Text = L"0";
				 this->Z_galv_L->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_galv_L->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_galv_L_TextChanged);
				 // 
				 // Z_galv_R
				 // 
				 this->Z_galv_R->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Z_galv_R->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Z_galv_R->ForeColor = System::Drawing::Color::Black;
				 this->Z_galv_R->Location = System::Drawing::Point(80, 18);
				 this->Z_galv_R->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Z_galv_R->Name = L"Z_galv_R";
				 this->Z_galv_R->ReadOnly = true;
				 this->Z_galv_R->Size = System::Drawing::Size(39, 31);
				 this->Z_galv_R->TabIndex = 4;
				 this->Z_galv_R->Text = L"0";
				 this->Z_galv_R->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Z_galv_R->TextChanged += gcnew System::EventHandler(this, &MyForm::Z_galv_R_TextChanged);
				 // 
				 // label34
				 // 
				 this->label34->AutoSize = true;
				 this->label34->BackColor = System::Drawing::Color::Transparent;
				 this->label34->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label34->ForeColor = System::Drawing::Color::White;
				 this->label34->Location = System::Drawing::Point(64, 22);
				 this->label34->Name = L"label34";
				 this->label34->Size = System::Drawing::Size(21, 24);
				 this->label34->TabIndex = 3;
				 this->label34->Text = L"R";
				 this->label34->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // Axial_647_load
				 // 
				 this->Axial_647_load->BackColor = System::Drawing::Color::OrangeRed;
				 this->Axial_647_load->ForeColor = System::Drawing::Color::Black;
				 this->Axial_647_load->Location = System::Drawing::Point(220, 232);
				 this->Axial_647_load->Name = L"Axial_647_load";
				 this->Axial_647_load->Size = System::Drawing::Size(50, 26);
				 this->Axial_647_load->TabIndex = 8;
				 this->Axial_647_load->Text = L"647";
				 this->Axial_647_load->UseVisualStyleBackColor = false;
				 this->Axial_647_load->Click += gcnew System::EventHandler(this, &MyForm::Axial_647_load_Click);
				 // 
				 // groupBox16
				 // 
				 this->groupBox16->Controls->Add(this->End_Z_pos_RV);
				 this->groupBox16->Controls->Add(this->Start_Z_pos_RV);
				 this->groupBox16->Controls->Add(this->End_Z_but_RV);
				 this->groupBox16->Controls->Add(this->Start_Z_but_RV);
				 this->groupBox16->Controls->Add(this->OneSideImaging);
				 this->groupBox16->Controls->Add(this->RV_Imaging_Check);
				 this->groupBox16->Controls->Add(this->SinglePlane);
				 this->groupBox16->Controls->Add(this->singlestackmode);
				 this->groupBox16->Controls->Add(this->label39);
				 this->groupBox16->Controls->Add(this->bottom_limit);
				 this->groupBox16->Controls->Add(this->right_limit);
				 this->groupBox16->Controls->Add(this->End_Z_pos);
				 this->groupBox16->Controls->Add(this->Start_Z_pos);
				 this->groupBox16->Controls->Add(this->left_limit);
				 this->groupBox16->Controls->Add(this->top_limit);
				 this->groupBox16->Controls->Add(this->left_limit_but);
				 this->groupBox16->Controls->Add(this->Y_limit_but);
				 this->groupBox16->Controls->Add(this->imagingtile);
				 this->groupBox16->Controls->Add(this->Theta_Pos_Set);
				 this->groupBox16->Controls->Add(this->Reverse_but);
				 this->groupBox16->Controls->Add(this->gotocenter_but);
				 this->groupBox16->Controls->Add(this->Set_RV_but);
				 this->groupBox16->Controls->Add(this->right_limit_but);
				 this->groupBox16->Controls->Add(this->ReStart_tile_num_box);
				 this->groupBox16->Controls->Add(this->top_limit_but);
				 this->groupBox16->Controls->Add(this->End_Z_but);
				 this->groupBox16->Controls->Add(this->Theta_POS_Z);
				 this->groupBox16->Controls->Add(this->ReverseSpos);
				 this->groupBox16->Controls->Add(this->bottom_limit_RV);
				 this->groupBox16->Controls->Add(this->left_limit_RV);
				 this->groupBox16->Controls->Add(this->Theta_POS_X);
				 this->groupBox16->Controls->Add(this->right_limit_RV);
				 this->groupBox16->Controls->Add(this->nama);
				 this->groupBox16->Controls->Add(this->MOVIE_strat);
				 this->groupBox16->Controls->Add(this->Start_Z_but);
				 this->groupBox16->Controls->Add(this->Target_img);
				 this->groupBox16->Controls->Add(this->Acq_img);
				 this->groupBox16->Controls->Add(this->label61);
				 this->groupBox16->Controls->Add(this->label60);
				 this->groupBox16->Controls->Add(this->exp2);
				 this->groupBox16->Controls->Add(this->exp);
				 this->groupBox16->Controls->Add(this->label73);
				 this->groupBox16->Controls->Add(this->Step);
				 this->groupBox16->Controls->Add(this->top_limit_RV);
				 this->groupBox16->Controls->Add(this->label32);
				 this->groupBox16->Controls->Add(this->label45);
				 this->groupBox16->Controls->Add(this->label44);
				 this->groupBox16->Controls->Add(this->label19);
				 this->groupBox16->Controls->Add(this->label31);
				 this->groupBox16->Controls->Add(this->label24);
				 this->groupBox16->Controls->Add(this->label88);
				 this->groupBox16->Controls->Add(this->label46);
				 this->groupBox16->ForeColor = System::Drawing::Color::White;
				 this->groupBox16->Location = System::Drawing::Point(10, -4);
				 this->groupBox16->Name = L"groupBox16";
				 this->groupBox16->Size = System::Drawing::Size(458, 221);
				 this->groupBox16->TabIndex = 20;
				 this->groupBox16->TabStop = false;
				 this->groupBox16->Text = L"F-R";
				 this->groupBox16->Enter += gcnew System::EventHandler(this, &MyForm::groupBox16_Enter);
				 // 
				 // End_Z_pos_RV
				 // 
				 this->End_Z_pos_RV->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->End_Z_pos_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->End_Z_pos_RV->ForeColor = System::Drawing::Color::Black;
				 this->End_Z_pos_RV->Location = System::Drawing::Point(352, 41);
				 this->End_Z_pos_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->End_Z_pos_RV->Name = L"End_Z_pos_RV";
				 this->End_Z_pos_RV->Size = System::Drawing::Size(50, 31);
				 this->End_Z_pos_RV->TabIndex = 54;
				 this->End_Z_pos_RV->Text = L"0";
				 this->End_Z_pos_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Start_Z_pos_RV
				 // 
				 this->Start_Z_pos_RV->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Start_Z_pos_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Start_Z_pos_RV->ForeColor = System::Drawing::Color::Black;
				 this->Start_Z_pos_RV->Location = System::Drawing::Point(352, 16);
				 this->Start_Z_pos_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Start_Z_pos_RV->Name = L"Start_Z_pos_RV";
				 this->Start_Z_pos_RV->Size = System::Drawing::Size(50, 31);
				 this->Start_Z_pos_RV->TabIndex = 53;
				 this->Start_Z_pos_RV->Text = L"0";
				 this->Start_Z_pos_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // End_Z_but_RV
				 // 
				 this->End_Z_but_RV->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->End_Z_but_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->End_Z_but_RV->ForeColor = System::Drawing::Color::Black;
				 this->End_Z_but_RV->Location = System::Drawing::Point(402, 41);
				 this->End_Z_but_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->End_Z_but_RV->Name = L"End_Z_but_RV";
				 this->End_Z_but_RV->Size = System::Drawing::Size(50, 25);
				 this->End_Z_but_RV->TabIndex = 51;
				 this->End_Z_but_RV->Text = L"End_Z";
				 this->End_Z_but_RV->UseVisualStyleBackColor = false;
				 this->End_Z_but_RV->Click += gcnew System::EventHandler(this, &MyForm::End_Z_but_RV_Click);
				 // 
				 // Start_Z_but_RV
				 // 
				 this->Start_Z_but_RV->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Start_Z_but_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Start_Z_but_RV->ForeColor = System::Drawing::Color::Black;
				 this->Start_Z_but_RV->Location = System::Drawing::Point(402, 15);
				 this->Start_Z_but_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Start_Z_but_RV->Name = L"Start_Z_but_RV";
				 this->Start_Z_but_RV->Size = System::Drawing::Size(50, 25);
				 this->Start_Z_but_RV->TabIndex = 52;
				 this->Start_Z_but_RV->Text = L"Start_Z";
				 this->Start_Z_but_RV->UseVisualStyleBackColor = false;
				 this->Start_Z_but_RV->Click += gcnew System::EventHandler(this, &MyForm::Start_Z_but_RV_Click);
				 // 
				 // OneSideImaging
				 // 
				 this->OneSideImaging->AutoSize = true;
				 this->OneSideImaging->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->OneSideImaging->ForeColor = System::Drawing::Color::White;
				 this->OneSideImaging->Location = System::Drawing::Point(11, 135);
				 this->OneSideImaging->Name = L"OneSideImaging";
				 this->OneSideImaging->Size = System::Drawing::Size(193, 26);
				 this->OneSideImaging->TabIndex = 49;
				 this->OneSideImaging->Text = L"One side imaging";
				 this->OneSideImaging->UseVisualStyleBackColor = true;
				 // 
				 // RV_Imaging_Check
				 // 
				 this->RV_Imaging_Check->AutoSize = true;
				 this->RV_Imaging_Check->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->RV_Imaging_Check->ForeColor = System::Drawing::Color::White;
				 this->RV_Imaging_Check->Location = System::Drawing::Point(12, 153);
				 this->RV_Imaging_Check->Name = L"RV_Imaging_Check";
				 this->RV_Imaging_Check->Size = System::Drawing::Size(160, 26);
				 this->RV_Imaging_Check->TabIndex = 49;
				 this->RV_Imaging_Check->Text = L"Start from RV";
				 this->RV_Imaging_Check->UseVisualStyleBackColor = true;
				 // 
				 // SinglePlane
				 // 
				 this->SinglePlane->AutoSize = true;
				 this->SinglePlane->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->SinglePlane->ForeColor = System::Drawing::Color::White;
				 this->SinglePlane->Location = System::Drawing::Point(121, 190);
				 this->SinglePlane->Name = L"SinglePlane";
				 this->SinglePlane->Size = System::Drawing::Size(148, 26);
				 this->SinglePlane->TabIndex = 49;
				 this->SinglePlane->Text = L"Single Plane";
				 this->SinglePlane->UseVisualStyleBackColor = true;
				 // 
				 // label39
				 // 
				 this->label39->AutoSize = true;
				 this->label39->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label39->ForeColor = System::Drawing::Color::White;
				 this->label39->Location = System::Drawing::Point(236, 101);
				 this->label39->Name = L"label39";
				 this->label39->Size = System::Drawing::Size(36, 24);
				 this->label39->TabIndex = 39;
				 this->label39->Text = L"tile";
				 // 
				 // bottom_limit
				 // 
				 this->bottom_limit->BackColor = System::Drawing::SystemColors::HighlightText;
				 this->bottom_limit->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->bottom_limit->Location = System::Drawing::Point(46, 60);
				 this->bottom_limit->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->bottom_limit->Name = L"bottom_limit";
				 this->bottom_limit->Size = System::Drawing::Size(52, 31);
				 this->bottom_limit->TabIndex = 34;
				 this->bottom_limit->Text = L"0";
				 this->bottom_limit->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // right_limit
				 // 
				 this->right_limit->BackColor = System::Drawing::SystemColors::HighlightText;
				 this->right_limit->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->right_limit->Location = System::Drawing::Point(89, 38);
				 this->right_limit->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->right_limit->Name = L"right_limit";
				 this->right_limit->Size = System::Drawing::Size(49, 31);
				 this->right_limit->TabIndex = 31;
				 this->right_limit->Text = L"0";
				 this->right_limit->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // left_limit
				 // 
				 this->left_limit->BackColor = System::Drawing::SystemColors::HighlightText;
				 this->left_limit->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->left_limit->Location = System::Drawing::Point(5, 38);
				 this->left_limit->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->left_limit->Name = L"left_limit";
				 this->left_limit->Size = System::Drawing::Size(52, 31);
				 this->left_limit->TabIndex = 32;
				 this->left_limit->Text = L"0";
				 this->left_limit->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // top_limit
				 // 
				 this->top_limit->BackColor = System::Drawing::SystemColors::HighlightText;
				 this->top_limit->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->top_limit->Location = System::Drawing::Point(43, 15);
				 this->top_limit->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->top_limit->Name = L"top_limit";
				 this->top_limit->Size = System::Drawing::Size(54, 31);
				 this->top_limit->TabIndex = 35;
				 this->top_limit->Text = L"0";
				 this->top_limit->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // left_limit_but
				 // 
				 this->left_limit_but->BackColor = System::Drawing::Color::White;
				 this->left_limit_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->left_limit_but->ForeColor = System::Drawing::Color::Black;
				 this->left_limit_but->Location = System::Drawing::Point(60, 83);
				 this->left_limit_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->left_limit_but->Name = L"left_limit_but";
				 this->left_limit_but->Size = System::Drawing::Size(38, 22);
				 this->left_limit_but->TabIndex = 33;
				 this->left_limit_but->Text = L"set";
				 this->left_limit_but->UseVisualStyleBackColor = false;
				 this->left_limit_but->Click += gcnew System::EventHandler(this, &MyForm::left_limit_but_Click_1);
				 // 
				 // Y_limit_but
				 // 
				 this->Y_limit_but->BackColor = System::Drawing::Color::White;
				 this->Y_limit_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Y_limit_but->ForeColor = System::Drawing::Color::Black;
				 this->Y_limit_but->Location = System::Drawing::Point(5, 62);
				 this->Y_limit_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Y_limit_but->Name = L"Y_limit_but";
				 this->Y_limit_but->Size = System::Drawing::Size(37, 22);
				 this->Y_limit_but->TabIndex = 36;
				 this->Y_limit_but->Text = L"set";
				 this->Y_limit_but->UseVisualStyleBackColor = false;
				 this->Y_limit_but->Click += gcnew System::EventHandler(this, &MyForm::Y_limit_but_Click_1);
				 // 
				 // Theta_Pos_Set
				 // 
				 this->Theta_Pos_Set->BackColor = System::Drawing::Color::White;
				 this->Theta_Pos_Set->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Theta_Pos_Set->ForeColor = System::Drawing::Color::Black;
				 this->Theta_Pos_Set->Location = System::Drawing::Point(275, 133);
				 this->Theta_Pos_Set->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_Pos_Set->Name = L"Theta_Pos_Set";
				 this->Theta_Pos_Set->Size = System::Drawing::Size(55, 22);
				 this->Theta_Pos_Set->TabIndex = 37;
				 this->Theta_Pos_Set->Text = L"Set";
				 this->Theta_Pos_Set->UseVisualStyleBackColor = false;
				 this->Theta_Pos_Set->Click += gcnew System::EventHandler(this, &MyForm::Theta_Pos_Set_Click);
				 // 
				 // Reverse_but
				 // 
				 this->Reverse_but->BackColor = System::Drawing::Color::White;
				 this->Reverse_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Reverse_but->ForeColor = System::Drawing::Color::Black;
				 this->Reverse_but->Location = System::Drawing::Point(409, 96);
				 this->Reverse_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Reverse_but->Name = L"Reverse_but";
				 this->Reverse_but->Size = System::Drawing::Size(38, 22);
				 this->Reverse_but->TabIndex = 37;
				 this->Reverse_but->Text = L"Go";
				 this->Reverse_but->UseVisualStyleBackColor = false;
				 this->Reverse_but->Click += gcnew System::EventHandler(this, &MyForm::Reverse_but_Click);
				 // 
				 // gotocenter_but
				 // 
				 this->gotocenter_but->BackColor = System::Drawing::Color::White;
				 this->gotocenter_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->gotocenter_but->ForeColor = System::Drawing::Color::Black;
				 this->gotocenter_but->Location = System::Drawing::Point(409, 73);
				 this->gotocenter_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->gotocenter_but->Name = L"gotocenter_but";
				 this->gotocenter_but->Size = System::Drawing::Size(38, 22);
				 this->gotocenter_but->TabIndex = 37;
				 this->gotocenter_but->Text = L"Go";
				 this->gotocenter_but->UseVisualStyleBackColor = false;
				 this->gotocenter_but->Click += gcnew System::EventHandler(this, &MyForm::gotocenter_but_Click_1);
				 // 
				 // Set_RV_but
				 // 
				 this->Set_RV_but->BackColor = System::Drawing::Color::White;
				 this->Set_RV_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Set_RV_but->ForeColor = System::Drawing::Color::Black;
				 this->Set_RV_but->Location = System::Drawing::Point(409, 119);
				 this->Set_RV_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Set_RV_but->Name = L"Set_RV_but";
				 this->Set_RV_but->Size = System::Drawing::Size(39, 22);
				 this->Set_RV_but->TabIndex = 37;
				 this->Set_RV_but->Text = L"Set";
				 this->Set_RV_but->UseVisualStyleBackColor = false;
				 this->Set_RV_but->Click += gcnew System::EventHandler(this, &MyForm::Set_RV_but_Click);
				 // 
				 // right_limit_but
				 // 
				 this->right_limit_but->BackColor = System::Drawing::Color::White;
				 this->right_limit_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->right_limit_but->ForeColor = System::Drawing::Color::Black;
				 this->right_limit_but->Location = System::Drawing::Point(102, 61);
				 this->right_limit_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->right_limit_but->Name = L"right_limit_but";
				 this->right_limit_but->Size = System::Drawing::Size(37, 22);
				 this->right_limit_but->TabIndex = 37;
				 this->right_limit_but->Text = L"set";
				 this->right_limit_but->UseVisualStyleBackColor = false;
				 this->right_limit_but->Click += gcnew System::EventHandler(this, &MyForm::right_limit_but_Click_2);
				 // 
				 // top_limit_but
				 // 
				 this->top_limit_but->BackColor = System::Drawing::Color::White;
				 this->top_limit_but->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->top_limit_but->ForeColor = System::Drawing::Color::Black;
				 this->top_limit_but->Location = System::Drawing::Point(97, 15);
				 this->top_limit_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->top_limit_but->Name = L"top_limit_but";
				 this->top_limit_but->Size = System::Drawing::Size(37, 22);
				 this->top_limit_but->TabIndex = 38;
				 this->top_limit_but->Text = L"set";
				 this->top_limit_but->UseVisualStyleBackColor = false;
				 this->top_limit_but->Click += gcnew System::EventHandler(this, &MyForm::top_limit_but_Click_1);
				 // 
				 // Theta_POS_Z
				 // 
				 this->Theta_POS_Z->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->Theta_POS_Z->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Theta_POS_Z->ForeColor = System::Drawing::Color::Black;
				 this->Theta_POS_Z->Location = System::Drawing::Point(275, 180);
				 this->Theta_POS_Z->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_POS_Z->Name = L"Theta_POS_Z";
				 this->Theta_POS_Z->Size = System::Drawing::Size(55, 31);
				 this->Theta_POS_Z->TabIndex = 4;
				 this->Theta_POS_Z->Text = L"0";
				 this->Theta_POS_Z->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // ReverseSpos
				 // 
				 this->ReverseSpos->BackColor = System::Drawing::SystemColors::Control;
				 this->ReverseSpos->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ReverseSpos->ForeColor = System::Drawing::Color::Black;
				 this->ReverseSpos->Location = System::Drawing::Point(277, 87);
				 this->ReverseSpos->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ReverseSpos->Name = L"ReverseSpos";
				 this->ReverseSpos->Size = System::Drawing::Size(50, 31);
				 this->ReverseSpos->TabIndex = 4;
				 this->ReverseSpos->Text = L"180";
				 this->ReverseSpos->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // bottom_limit_RV
				 // 
				 this->bottom_limit_RV->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->bottom_limit_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->bottom_limit_RV->ForeColor = System::Drawing::Color::Black;
				 this->bottom_limit_RV->Location = System::Drawing::Point(277, 64);
				 this->bottom_limit_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->bottom_limit_RV->Name = L"bottom_limit_RV";
				 this->bottom_limit_RV->Size = System::Drawing::Size(50, 31);
				 this->bottom_limit_RV->TabIndex = 4;
				 this->bottom_limit_RV->Text = L"0";
				 this->bottom_limit_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // left_limit_RV
				 // 
				 this->left_limit_RV->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->left_limit_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->left_limit_RV->ForeColor = System::Drawing::Color::Black;
				 this->left_limit_RV->Location = System::Drawing::Point(252, 41);
				 this->left_limit_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->left_limit_RV->Name = L"left_limit_RV";
				 this->left_limit_RV->Size = System::Drawing::Size(50, 31);
				 this->left_limit_RV->TabIndex = 4;
				 this->left_limit_RV->Text = L"0";
				 this->left_limit_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Theta_POS_X
				 // 
				 this->Theta_POS_X->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->Theta_POS_X->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Theta_POS_X->ForeColor = System::Drawing::Color::Black;
				 this->Theta_POS_X->Location = System::Drawing::Point(275, 156);
				 this->Theta_POS_X->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_POS_X->Name = L"Theta_POS_X";
				 this->Theta_POS_X->Size = System::Drawing::Size(55, 31);
				 this->Theta_POS_X->TabIndex = 4;
				 this->Theta_POS_X->Text = L"0";
				 this->Theta_POS_X->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // right_limit_RV
				 // 
				 this->right_limit_RV->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->right_limit_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->right_limit_RV->ForeColor = System::Drawing::Color::Black;
				 this->right_limit_RV->Location = System::Drawing::Point(302, 41);
				 this->right_limit_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->right_limit_RV->Name = L"right_limit_RV";
				 this->right_limit_RV->Size = System::Drawing::Size(50, 31);
				 this->right_limit_RV->TabIndex = 4;
				 this->right_limit_RV->Text = L"0";
				 this->right_limit_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Target_img
				 // 
				 this->Target_img->AutoSize = true;
				 this->Target_img->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Target_img->ForeColor = System::Drawing::Color::White;
				 this->Target_img->Location = System::Drawing::Point(422, 178);
				 this->Target_img->Name = L"Target_img";
				 this->Target_img->Size = System::Drawing::Size(20, 24);
				 this->Target_img->TabIndex = 9;
				 this->Target_img->Text = L"0";
				 this->Target_img->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // Acq_img
				 // 
				 this->Acq_img->AutoSize = true;
				 this->Acq_img->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Acq_img->ForeColor = System::Drawing::Color::White;
				 this->Acq_img->Location = System::Drawing::Point(388, 178);
				 this->Acq_img->Name = L"Acq_img";
				 this->Acq_img->Size = System::Drawing::Size(20, 24);
				 this->Acq_img->TabIndex = 9;
				 this->Acq_img->Text = L"0";
				 this->Acq_img->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // label61
				 // 
				 this->label61->AutoSize = true;
				 this->label61->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label61->ForeColor = System::Drawing::Color::White;
				 this->label61->Location = System::Drawing::Point(407, 178);
				 this->label61->Name = L"label61";
				 this->label61->Size = System::Drawing::Size(18, 24);
				 this->label61->TabIndex = 9;
				 this->label61->Text = L"/";
				 this->label61->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // label60
				 // 
				 this->label60->AutoSize = true;
				 this->label60->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label60->ForeColor = System::Drawing::Color::White;
				 this->label60->Location = System::Drawing::Point(374, 152);
				 this->label60->Name = L"label60";
				 this->label60->Size = System::Drawing::Size(40, 24);
				 this->label60->TabIndex = 9;
				 this->label60->Text = L"Exp";
				 this->label60->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // exp2
				 // 
				 this->exp2->AutoSize = true;
				 this->exp2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->exp2->ForeColor = System::Drawing::Color::White;
				 this->exp2->Location = System::Drawing::Point(407, 160);
				 this->exp2->Name = L"exp2";
				 this->exp2->Size = System::Drawing::Size(20, 24);
				 this->exp2->TabIndex = 9;
				 this->exp2->Text = L"0";
				 this->exp2->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // exp
				 // 
				 this->exp->AutoSize = true;
				 this->exp->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->exp->ForeColor = System::Drawing::Color::White;
				 this->exp->Location = System::Drawing::Point(407, 145);
				 this->exp->Name = L"exp";
				 this->exp->Size = System::Drawing::Size(20, 24);
				 this->exp->TabIndex = 9;
				 this->exp->Text = L"0";
				 this->exp->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
				 // 
				 // top_limit_RV
				 // 
				 this->top_limit_RV->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->top_limit_RV->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->top_limit_RV->ForeColor = System::Drawing::Color::Black;
				 this->top_limit_RV->Location = System::Drawing::Point(278, 18);
				 this->top_limit_RV->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->top_limit_RV->Name = L"top_limit_RV";
				 this->top_limit_RV->Size = System::Drawing::Size(50, 31);
				 this->top_limit_RV->TabIndex = 4;
				 this->top_limit_RV->Text = L"0";
				 this->top_limit_RV->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label32
				 // 
				 this->label32->AutoSize = true;
				 this->label32->BackColor = System::Drawing::Color::Transparent;
				 this->label32->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label32->ForeColor = System::Drawing::Color::White;
				 this->label32->Location = System::Drawing::Point(354, 99);
				 this->label32->Name = L"label32";
				 this->label32->Size = System::Drawing::Size(81, 24);
				 this->label32->TabIndex = 3;
				 this->label32->Text = L"Reverse:";
				 this->label32->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label45
				 // 
				 this->label45->AutoSize = true;
				 this->label45->BackColor = System::Drawing::Color::Transparent;
				 this->label45->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label45->ForeColor = System::Drawing::Color::White;
				 this->label45->Location = System::Drawing::Point(225, 160);
				 this->label45->Name = L"label45";
				 this->label45->Size = System::Drawing::Size(80, 24);
				 this->label45->TabIndex = 3;
				 this->label45->Text = L"Θ-Pos x:";
				 this->label45->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label44
				 // 
				 this->label44->AutoSize = true;
				 this->label44->BackColor = System::Drawing::Color::Transparent;
				 this->label44->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label44->ForeColor = System::Drawing::Color::White;
				 this->label44->Location = System::Drawing::Point(238, 19);
				 this->label44->Name = L"label44";
				 this->label44->Size = System::Drawing::Size(62, 24);
				 this->label44->TabIndex = 3;
				 this->label44->Text = L"RV xy:";
				 this->label44->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label19
				 // 
				 this->label19->AutoSize = true;
				 this->label19->BackColor = System::Drawing::Color::Transparent;
				 this->label19->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label19->ForeColor = System::Drawing::Color::White;
				 this->label19->Location = System::Drawing::Point(333, 75);
				 this->label19->Name = L"label19";
				 this->label19->Size = System::Drawing::Size(115, 24);
				 this->label19->TabIndex = 3;
				 this->label19->Text = L"GoToCenter:";
				 this->label19->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label31
				 // 
				 this->label31->AutoSize = true;
				 this->label31->BackColor = System::Drawing::Color::Transparent;
				 this->label31->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label31->ForeColor = System::Drawing::Color::White;
				 this->label31->Location = System::Drawing::Point(360, 122);
				 this->label31->Name = L"label31";
				 this->label31->Size = System::Drawing::Size(72, 24);
				 this->label31->TabIndex = 3;
				 this->label31->Text = L"Center:";
				 this->label31->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label46
				 // 
				 this->label46->AutoSize = true;
				 this->label46->BackColor = System::Drawing::Color::Transparent;
				 this->label46->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label46->ForeColor = System::Drawing::Color::White;
				 this->label46->Location = System::Drawing::Point(225, 183);
				 this->label46->Name = L"label46";
				 this->label46->Size = System::Drawing::Size(79, 24);
				 this->label46->TabIndex = 50;
				 this->label46->Text = L"Θ-Pos z:";
				 // 
				 // AxialAF_but
				 // 
				 this->AxialAF_but->BackColor = System::Drawing::Color::Fuchsia;
				 this->AxialAF_but->ForeColor = System::Drawing::Color::Black;
				 this->AxialAF_but->Location = System::Drawing::Point(220, 258);
				 this->AxialAF_but->Name = L"AxialAF_but";
				 this->AxialAF_but->Size = System::Drawing::Size(50, 25);
				 this->AxialAF_but->TabIndex = 8;
				 this->AxialAF_but->Text = L"AF";
				 this->AxialAF_but->UseVisualStyleBackColor = false;
				 this->AxialAF_but->Click += gcnew System::EventHandler(this, &MyForm::AxialAF_but_Click);
				 // 
				 // Axial_592_load
				 // 
				 this->Axial_592_load->BackColor = System::Drawing::Color::Orange;
				 this->Axial_592_load->ForeColor = System::Drawing::Color::Black;
				 this->Axial_592_load->Location = System::Drawing::Point(171, 258);
				 this->Axial_592_load->Name = L"Axial_592_load";
				 this->Axial_592_load->Size = System::Drawing::Size(50, 25);
				 this->Axial_592_load->TabIndex = 8;
				 this->Axial_592_load->Text = L"592";
				 this->Axial_592_load->UseVisualStyleBackColor = false;
				 this->Axial_592_load->Click += gcnew System::EventHandler(this, &MyForm::Axial_592_load_Click);
				 // 
				 // ETL_R_right
				 // 
				 this->ETL_R_right->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_R_right->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_R_right->ForeColor = System::Drawing::Color::Black;
				 this->ETL_R_right->Location = System::Drawing::Point(120, 262);
				 this->ETL_R_right->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_R_right->Name = L"ETL_R_right";
				 this->ETL_R_right->Size = System::Drawing::Size(45, 31);
				 this->ETL_R_right->TabIndex = 4;
				 this->ETL_R_right->Text = L"0";
				 this->ETL_R_right->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_R_right->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_R_right_TextChanged);
				 // 
				 // Axial_panel
				 // 
				 this->Axial_panel->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(255)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 this->Axial_panel->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Axial_panel->ForeColor = System::Drawing::Color::Black;
				 this->Axial_panel->Location = System::Drawing::Point(221, 260);
				 this->Axial_panel->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Axial_panel->Name = L"Axial_panel";
				 this->Axial_panel->ReadOnly = true;
				 this->Axial_panel->Size = System::Drawing::Size(50, 31);
				 this->Axial_panel->TabIndex = 1;
				 this->Axial_panel->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Axial_488_load
				 // 
				 this->Axial_488_load->BackColor = System::Drawing::Color::Aqua;
				 this->Axial_488_load->ForeColor = System::Drawing::Color::Black;
				 this->Axial_488_load->Location = System::Drawing::Point(170, 232);
				 this->Axial_488_load->Name = L"Axial_488_load";
				 this->Axial_488_load->Size = System::Drawing::Size(50, 26);
				 this->Axial_488_load->TabIndex = 8;
				 this->Axial_488_load->Text = L"488";
				 this->Axial_488_load->UseVisualStyleBackColor = false;
				 this->Axial_488_load->Click += gcnew System::EventHandler(this, &MyForm::Axial_488_load_Click);
				 // 
				 // ETL_L_right
				 // 
				 this->ETL_L_right->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_L_right->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_L_right->ForeColor = System::Drawing::Color::Black;
				 this->ETL_L_right->Location = System::Drawing::Point(71, 262);
				 this->ETL_L_right->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_L_right->Name = L"ETL_L_right";
				 this->ETL_L_right->Size = System::Drawing::Size(45, 31);
				 this->ETL_L_right->TabIndex = 4;
				 this->ETL_L_right->Text = L"0";
				 this->ETL_L_right->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_L_right->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_L_right_TextChanged);
				 // 
				 // ETL_R_left
				 // 
				 this->ETL_R_left->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_R_left->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_R_left->ForeColor = System::Drawing::Color::Black;
				 this->ETL_R_left->Location = System::Drawing::Point(120, 238);
				 this->ETL_R_left->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_R_left->Name = L"ETL_R_left";
				 this->ETL_R_left->Size = System::Drawing::Size(45, 31);
				 this->ETL_R_left->TabIndex = 4;
				 this->ETL_R_left->Text = L"0";
				 this->ETL_R_left->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_R_left->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_R_left_TextChanged);
				 // 
				 // ETL_L_left
				 // 
				 this->ETL_L_left->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ETL_L_left->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_L_left->ForeColor = System::Drawing::Color::Black;
				 this->ETL_L_left->Location = System::Drawing::Point(71, 238);
				 this->ETL_L_left->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_L_left->Name = L"ETL_L_left";
				 this->ETL_L_left->Size = System::Drawing::Size(45, 31);
				 this->ETL_L_left->TabIndex = 4;
				 this->ETL_L_left->Text = L"0";
				 this->ETL_L_left->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ETL_L_left->TextChanged += gcnew System::EventHandler(this, &MyForm::ETL_L_left_TextChanged);
				 // 
				 // label10
				 // 
				 this->label10->AutoSize = true;
				 this->label10->BackColor = System::Drawing::Color::Transparent;
				 this->label10->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label10->ForeColor = System::Drawing::Color::White;
				 this->label10->Location = System::Drawing::Point(121, 220);
				 this->label10->Name = L"label10";
				 this->label10->Size = System::Drawing::Size(73, 24);
				 this->label10->TabIndex = 3;
				 this->label10->Text = L"R(laser)";
				 this->label10->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label26
				 // 
				 this->label26->AutoSize = true;
				 this->label26->BackColor = System::Drawing::Color::Transparent;
				 this->label26->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label26->ForeColor = System::Drawing::Color::White;
				 this->label26->Location = System::Drawing::Point(23, 267);
				 this->label26->Name = L"label26";
				 this->label26->Size = System::Drawing::Size(49, 24);
				 this->label26->TabIndex = 3;
				 this->label26->Text = L"right";
				 // 
				 // label13
				 // 
				 this->label13->AutoSize = true;
				 this->label13->BackColor = System::Drawing::Color::Transparent;
				 this->label13->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label13->ForeColor = System::Drawing::Color::White;
				 this->label13->Location = System::Drawing::Point(24, 241);
				 this->label13->Name = L"label13";
				 this->label13->Size = System::Drawing::Size(41, 24);
				 this->label13->TabIndex = 3;
				 this->label13->Text = L"Left";
				 // 
				 // label9
				 // 
				 this->label9->AutoSize = true;
				 this->label9->BackColor = System::Drawing::Color::Transparent;
				 this->label9->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label9->ForeColor = System::Drawing::Color::White;
				 this->label9->Location = System::Drawing::Point(67, 219);
				 this->label9->Name = L"label9";
				 this->label9->Size = System::Drawing::Size(70, 24);
				 this->label9->TabIndex = 3;
				 this->label9->Text = L"L(laser)";
				 this->label9->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // tabPage1
				 // 
				 this->tabPage1->BackColor = System::Drawing::Color::Gray;
				 this->tabPage1->Controls->Add(this->ManuaL_focus);
				 this->tabPage1->Controls->Add(this->label69);
				 this->tabPage1->Controls->Add(this->label68);
				 this->tabPage1->Controls->Add(this->AFnum);
				 this->tabPage1->Controls->Add(this->AFspan);
				 this->tabPage1->Controls->Add(this->SkipTile);
				 this->tabPage1->Controls->Add(this->windowsize_box);
				 this->tabPage1->Controls->Add(this->label67);
				 this->tabPage1->Controls->Add(this->Extra_time_box);
				 this->tabPage1->Controls->Add(this->manual_focus_box);
				 this->tabPage1->Controls->Add(this->delay_time_box);
				 this->tabPage1->Controls->Add(this->Overlap_um_y);
				 this->tabPage1->Controls->Add(this->Overlap_um);
				 this->tabPage1->Controls->Add(this->label41);
				 this->tabPage1->Controls->Add(this->label40);
				 this->tabPage1->Controls->Add(this->trackBar_manual_focus);
				 this->tabPage1->Controls->Add(this->label43);
				 this->tabPage1->Controls->Add(this->label42);
				 this->tabPage1->Controls->Add(this->label38);
				 this->tabPage1->Controls->Add(this->Z_overlap_box);
				 this->tabPage1->Controls->Add(this->label17);
				 this->tabPage1->Controls->Add(this->label52);
				 this->tabPage1->Controls->Add(this->label51);
				 this->tabPage1->Controls->Add(this->label23);
				 this->tabPage1->Controls->Add(this->label22);
				 this->tabPage1->Controls->Add(this->label90);
				 this->tabPage1->Controls->Add(this->label25);
				 this->tabPage1->Controls->Add(this->group_focus);
				 this->tabPage1->Controls->Add(this->groupBox15);
				 this->tabPage1->ForeColor = System::Drawing::Color::White;
				 this->tabPage1->Location = System::Drawing::Point(4, 33);
				 this->tabPage1->Name = L"tabPage1";
				 this->tabPage1->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage1->Size = System::Drawing::Size(471, 283);
				 this->tabPage1->TabIndex = 8;
				 this->tabPage1->Text = L"Setting";
				 // 
				 // ManuaL_focus
				 // 
				 this->ManuaL_focus->AutoSize = true;
				 this->ManuaL_focus->Font = (gcnew System::Drawing::Font(L"Calibri", 8, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ManuaL_focus->Location = System::Drawing::Point(284, 177);
				 this->ManuaL_focus->Name = L"ManuaL_focus";
				 this->ManuaL_focus->Size = System::Drawing::Size(127, 23);
				 this->ManuaL_focus->TabIndex = 58;
				 this->ManuaL_focus->Text = L"Manual focus";
				 this->ManuaL_focus->UseVisualStyleBackColor = true;
				 // 
				 // label69
				 // 
				 this->label69->AutoSize = true;
				 this->label69->BackColor = System::Drawing::Color::Transparent;
				 this->label69->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label69->ForeColor = System::Drawing::Color::White;
				 this->label69->Location = System::Drawing::Point(161, 238);
				 this->label69->Name = L"label69";
				 this->label69->Size = System::Drawing::Size(48, 24);
				 this->label69->TabIndex = 57;
				 this->label69->Text = L"num";
				 // 
				 // label68
				 // 
				 this->label68->AutoSize = true;
				 this->label68->BackColor = System::Drawing::Color::Transparent;
				 this->label68->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label68->ForeColor = System::Drawing::Color::White;
				 this->label68->Location = System::Drawing::Point(161, 215);
				 this->label68->Name = L"label68";
				 this->label68->Size = System::Drawing::Size(50, 24);
				 this->label68->TabIndex = 57;
				 this->label68->Text = L"span";
				 // 
				 // AFnum
				 // 
				 this->AFnum->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->AFnum->Location = System::Drawing::Point(199, 238);
				 this->AFnum->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->AFnum->Name = L"AFnum";
				 this->AFnum->Size = System::Drawing::Size(50, 31);
				 this->AFnum->TabIndex = 56;
				 this->AFnum->Text = L"21";
				 this->AFnum->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // AFspan
				 // 
				 this->AFspan->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->AFspan->Location = System::Drawing::Point(199, 212);
				 this->AFspan->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->AFspan->Name = L"AFspan";
				 this->AFspan->Size = System::Drawing::Size(50, 31);
				 this->AFspan->TabIndex = 56;
				 this->AFspan->Text = L"8";
				 this->AFspan->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // SkipTile
				 // 
				 this->SkipTile->Font = (gcnew System::Drawing::Font(L"Calibri", 8, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->SkipTile->Location = System::Drawing::Point(336, 126);
				 this->SkipTile->Name = L"SkipTile";
				 this->SkipTile->Size = System::Drawing::Size(75, 32);
				 this->SkipTile->TabIndex = 55;
				 this->SkipTile->Text = L"skip tile";
				 this->SkipTile->UseVisualStyleBackColor = true;
				 this->SkipTile->Click += gcnew System::EventHandler(this, &MyForm::SkipTile_Click);
				 // 
				 // windowsize_box
				 // 
				 this->windowsize_box->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->windowsize_box->Location = System::Drawing::Point(72, 195);
				 this->windowsize_box->Name = L"windowsize_box";
				 this->windowsize_box->Size = System::Drawing::Size(39, 34);
				 this->windowsize_box->TabIndex = 30;
				 this->windowsize_box->Text = L"8";
				 this->windowsize_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->windowsize_box->TextChanged += gcnew System::EventHandler(this, &MyForm::windowsize_box_TextChanged);
				 // 
				 // label67
				 // 
				 this->label67->AutoSize = true;
				 this->label67->BackColor = System::Drawing::Color::Transparent;
				 this->label67->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label67->ForeColor = System::Drawing::Color::White;
				 this->label67->Location = System::Drawing::Point(169, 193);
				 this->label67->Name = L"label67";
				 this->label67->Size = System::Drawing::Size(100, 24);
				 this->label67->TabIndex = 3;
				 this->label67->Text = L"Auto focus";
				 // 
				 // Extra_time_box
				 // 
				 this->Extra_time_box->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Extra_time_box->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Extra_time_box->Location = System::Drawing::Point(71, 259);
				 this->Extra_time_box->Name = L"Extra_time_box";
				 this->Extra_time_box->Size = System::Drawing::Size(39, 34);
				 this->Extra_time_box->TabIndex = 30;
				 this->Extra_time_box->Text = L"0";
				 this->Extra_time_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Extra_time_box->TextChanged += gcnew System::EventHandler(this, &MyForm::Extra_time_box_TextChanged);
				 // 
				 // manual_focus_box
				 // 
				 this->manual_focus_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->manual_focus_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->manual_focus_box->ForeColor = System::Drawing::Color::Black;
				 this->manual_focus_box->Location = System::Drawing::Point(397, 238);
				 this->manual_focus_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->manual_focus_box->Name = L"manual_focus_box";
				 this->manual_focus_box->Size = System::Drawing::Size(60, 31);
				 this->manual_focus_box->TabIndex = 4;
				 this->manual_focus_box->Text = L"0";
				 this->manual_focus_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->manual_focus_box->TextChanged += gcnew System::EventHandler(this, &MyForm::manual_focus_box_TextChanged);
				 // 
				 // delay_time_box
				 // 
				 this->delay_time_box->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->delay_time_box->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->delay_time_box->Location = System::Drawing::Point(71, 228);
				 this->delay_time_box->Name = L"delay_time_box";
				 this->delay_time_box->Size = System::Drawing::Size(39, 34);
				 this->delay_time_box->TabIndex = 30;
				 this->delay_time_box->Text = L"5";
				 this->delay_time_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->delay_time_box->TextChanged += gcnew System::EventHandler(this, &MyForm::delay_time_box_TextChanged);
				 // 
				 // Overlap_um_y
				 // 
				 this->Overlap_um_y->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Overlap_um_y->Location = System::Drawing::Point(336, 62);
				 this->Overlap_um_y->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Overlap_um_y->Name = L"Overlap_um_y";
				 this->Overlap_um_y->Size = System::Drawing::Size(50, 31);
				 this->Overlap_um_y->TabIndex = 39;
				 this->Overlap_um_y->Text = L"124";
				 this->Overlap_um_y->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Overlap_um
				 // 
				 this->Overlap_um->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Overlap_um->Location = System::Drawing::Point(336, 38);
				 this->Overlap_um->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Overlap_um->Name = L"Overlap_um";
				 this->Overlap_um->Size = System::Drawing::Size(50, 31);
				 this->Overlap_um->TabIndex = 40;
				 this->Overlap_um->Text = L"124";
				 this->Overlap_um->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label41
				 // 
				 this->label41->AutoSize = true;
				 this->label41->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label41->ForeColor = System::Drawing::Color::White;
				 this->label41->Location = System::Drawing::Point(392, 66);
				 this->label41->Name = L"label41";
				 this->label41->Size = System::Drawing::Size(49, 24);
				 this->label41->TabIndex = 34;
				 this->label41->Text = L"pixel";
				 // 
				 // label40
				 // 
				 this->label40->AutoSize = true;
				 this->label40->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label40->ForeColor = System::Drawing::Color::White;
				 this->label40->Location = System::Drawing::Point(392, 42);
				 this->label40->Name = L"label40";
				 this->label40->Size = System::Drawing::Size(49, 24);
				 this->label40->TabIndex = 35;
				 this->label40->Text = L"pixel";
				 // 
				 // trackBar_manual_focus
				 // 
				 this->trackBar_manual_focus->Cursor = System::Windows::Forms::Cursors::NoMoveHoriz;
				 this->trackBar_manual_focus->LargeChange = 50;
				 this->trackBar_manual_focus->Location = System::Drawing::Point(275, 200);
				 this->trackBar_manual_focus->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->trackBar_manual_focus->Maximum = 400;
				 this->trackBar_manual_focus->Name = L"trackBar_manual_focus";
				 this->trackBar_manual_focus->Size = System::Drawing::Size(193, 69);
				 this->trackBar_manual_focus->TabIndex = 2;
				 this->trackBar_manual_focus->TickFrequency = 20;
				 this->trackBar_manual_focus->Value = 200;
				 this->trackBar_manual_focus->Scroll += gcnew System::EventHandler(this, &MyForm::trackBar_manual_focus_Scroll);
				 // 
				 // label43
				 // 
				 this->label43->AutoSize = true;
				 this->label43->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label43->ForeColor = System::Drawing::Color::White;
				 this->label43->Location = System::Drawing::Point(319, 65);
				 this->label43->Name = L"label43";
				 this->label43->Size = System::Drawing::Size(24, 24);
				 this->label43->TabIndex = 36;
				 this->label43->Text = L"y:";
				 // 
				 // label42
				 // 
				 this->label42->AutoSize = true;
				 this->label42->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label42->ForeColor = System::Drawing::Color::White;
				 this->label42->Location = System::Drawing::Point(319, 41);
				 this->label42->Name = L"label42";
				 this->label42->Size = System::Drawing::Size(24, 24);
				 this->label42->TabIndex = 37;
				 this->label42->Text = L"x:";
				 // 
				 // label38
				 // 
				 this->label38->AutoSize = true;
				 this->label38->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label38->ForeColor = System::Drawing::Color::White;
				 this->label38->Location = System::Drawing::Point(314, 15);
				 this->label38->Name = L"label38";
				 this->label38->Size = System::Drawing::Size(75, 24);
				 this->label38->TabIndex = 38;
				 this->label38->Text = L"Overlap";
				 // 
				 // Z_overlap_box
				 // 
				 this->Z_overlap_box->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Z_overlap_box->Location = System::Drawing::Point(336, 87);
				 this->Z_overlap_box->Name = L"Z_overlap_box";
				 this->Z_overlap_box->Size = System::Drawing::Size(50, 34);
				 this->Z_overlap_box->TabIndex = 33;
				 this->Z_overlap_box->Text = L"200";
				 this->Z_overlap_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label17
				 // 
				 this->label17->AutoSize = true;
				 this->label17->BackColor = System::Drawing::Color::Transparent;
				 this->label17->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label17->ForeColor = System::Drawing::Color::White;
				 this->label17->Location = System::Drawing::Point(0, 199);
				 this->label17->Name = L"label17";
				 this->label17->Size = System::Drawing::Size(115, 24);
				 this->label17->TabIndex = 31;
				 this->label17->Text = L"windowsize:";
				 // 
				 // label52
				 // 
				 this->label52->AutoSize = true;
				 this->label52->BackColor = System::Drawing::Color::Transparent;
				 this->label52->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label52->ForeColor = System::Drawing::Color::White;
				 this->label52->Location = System::Drawing::Point(4, 263);
				 this->label52->Name = L"label52";
				 this->label52->Size = System::Drawing::Size(102, 24);
				 this->label52->TabIndex = 31;
				 this->label52->Text = L"Extra time:";
				 // 
				 // label51
				 // 
				 this->label51->AutoSize = true;
				 this->label51->BackColor = System::Drawing::Color::Transparent;
				 this->label51->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label51->ForeColor = System::Drawing::Color::White;
				 this->label51->Location = System::Drawing::Point(116, 263);
				 this->label51->Name = L"label51";
				 this->label51->Size = System::Drawing::Size(34, 24);
				 this->label51->TabIndex = 31;
				 this->label51->Text = L"ms";
				 // 
				 // label23
				 // 
				 this->label23->AutoSize = true;
				 this->label23->BackColor = System::Drawing::Color::Transparent;
				 this->label23->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label23->ForeColor = System::Drawing::Color::White;
				 this->label23->Location = System::Drawing::Point(4, 232);
				 this->label23->Name = L"label23";
				 this->label23->Size = System::Drawing::Size(104, 24);
				 this->label23->TabIndex = 31;
				 this->label23->Text = L"delay time:";
				 // 
				 // label22
				 // 
				 this->label22->AutoSize = true;
				 this->label22->BackColor = System::Drawing::Color::Transparent;
				 this->label22->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label22->ForeColor = System::Drawing::Color::White;
				 this->label22->Location = System::Drawing::Point(116, 232);
				 this->label22->Name = L"label22";
				 this->label22->Size = System::Drawing::Size(34, 24);
				 this->label22->TabIndex = 31;
				 this->label22->Text = L"ms";
				 // 
				 // label90
				 // 
				 this->label90->AutoSize = true;
				 this->label90->BackColor = System::Drawing::Color::Transparent;
				 this->label90->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label90->ForeColor = System::Drawing::Color::White;
				 this->label90->Location = System::Drawing::Point(393, 92);
				 this->label90->Name = L"label90";
				 this->label90->Size = System::Drawing::Size(37, 24);
				 this->label90->TabIndex = 31;
				 this->label90->Text = L"um";
				 // 
				 // label25
				 // 
				 this->label25->AutoSize = true;
				 this->label25->BackColor = System::Drawing::Color::Transparent;
				 this->label25->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label25->ForeColor = System::Drawing::Color::White;
				 this->label25->Location = System::Drawing::Point(319, 89);
				 this->label25->Name = L"label25";
				 this->label25->Size = System::Drawing::Size(24, 24);
				 this->label25->TabIndex = 32;
				 this->label25->Text = L"z:";
				 // 
				 // group_focus
				 // 
				 this->group_focus->Controls->Add(this->MOVIEfocus_calc_block);
				 this->group_focus->Controls->Add(this->MOVIE_Retry_Box);
				 this->group_focus->Controls->Add(this->MOVIE_Amplitude);
				 this->group_focus->Controls->Add(this->label47);
				 this->group_focus->Controls->Add(this->label86);
				 this->group_focus->Controls->Add(this->label85);
				 this->group_focus->Controls->Add(this->label62);
				 this->group_focus->Controls->Add(this->label27);
				 this->group_focus->ForeColor = System::Drawing::Color::White;
				 this->group_focus->Location = System::Drawing::Point(161, 21);
				 this->group_focus->Name = L"group_focus";
				 this->group_focus->Size = System::Drawing::Size(125, 138);
				 this->group_focus->TabIndex = 21;
				 this->group_focus->TabStop = false;
				 this->group_focus->Text = L"MOVIE-focus";
				 // 
				 // MOVIEfocus_calc_block
				 // 
				 this->MOVIEfocus_calc_block->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->MOVIEfocus_calc_block->Location = System::Drawing::Point(42, 93);
				 this->MOVIEfocus_calc_block->Name = L"MOVIEfocus_calc_block";
				 this->MOVIEfocus_calc_block->Size = System::Drawing::Size(39, 34);
				 this->MOVIEfocus_calc_block->TabIndex = 30;
				 this->MOVIEfocus_calc_block->Text = L"8";
				 this->MOVIEfocus_calc_block->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // MOVIE_Retry_Box
				 // 
				 this->MOVIE_Retry_Box->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->MOVIE_Retry_Box->Location = System::Drawing::Point(54, 62);
				 this->MOVIE_Retry_Box->Name = L"MOVIE_Retry_Box";
				 this->MOVIE_Retry_Box->Size = System::Drawing::Size(39, 34);
				 this->MOVIE_Retry_Box->TabIndex = 30;
				 this->MOVIE_Retry_Box->Text = L"300";
				 this->MOVIE_Retry_Box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // MOVIE_Amplitude
				 // 
				 this->MOVIE_Amplitude->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->MOVIE_Amplitude->Font = (gcnew System::Drawing::Font(L"メイリオ", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->MOVIE_Amplitude->ForeColor = System::Drawing::Color::Black;
				 this->MOVIE_Amplitude->Location = System::Drawing::Point(54, 33);
				 this->MOVIE_Amplitude->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MOVIE_Amplitude->Name = L"MOVIE_Amplitude";
				 this->MOVIE_Amplitude->Size = System::Drawing::Size(39, 34);
				 this->MOVIE_Amplitude->TabIndex = 4;
				 this->MOVIE_Amplitude->Text = L"0.8";
				 this->MOVIE_Amplitude->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label47
				 // 
				 this->label47->AutoSize = true;
				 this->label47->BackColor = System::Drawing::Color::Transparent;
				 this->label47->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label47->ForeColor = System::Drawing::Color::White;
				 this->label47->Location = System::Drawing::Point(8, 98);
				 this->label47->Name = L"label47";
				 this->label47->Size = System::Drawing::Size(44, 24);
				 this->label47->TabIndex = 3;
				 this->label47->Text = L"Calc";
				 // 
				 // label86
				 // 
				 this->label86->AutoSize = true;
				 this->label86->BackColor = System::Drawing::Color::Transparent;
				 this->label86->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label86->ForeColor = System::Drawing::Color::White;
				 this->label86->Location = System::Drawing::Point(8, 66);
				 this->label86->Name = L"label86";
				 this->label86->Size = System::Drawing::Size(54, 24);
				 this->label86->TabIndex = 3;
				 this->label86->Text = L"Retry";
				 // 
				 // label85
				 // 
				 this->label85->AutoSize = true;
				 this->label85->BackColor = System::Drawing::Color::Transparent;
				 this->label85->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label85->ForeColor = System::Drawing::Color::White;
				 this->label85->Location = System::Drawing::Point(8, 38);
				 this->label85->Name = L"label85";
				 this->label85->Size = System::Drawing::Size(59, 24);
				 this->label85->TabIndex = 3;
				 this->label85->Text = L"Ampl.";
				 // 
				 // label62
				 // 
				 this->label62->AutoSize = true;
				 this->label62->BackColor = System::Drawing::Color::Transparent;
				 this->label62->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label62->ForeColor = System::Drawing::Color::White;
				 this->label62->Location = System::Drawing::Point(85, 98);
				 this->label62->Name = L"label62";
				 this->label62->Size = System::Drawing::Size(63, 24);
				 this->label62->TabIndex = 3;
				 this->label62->Text = L"blocks";
				 // 
				 // label27
				 // 
				 this->label27->AutoSize = true;
				 this->label27->BackColor = System::Drawing::Color::Transparent;
				 this->label27->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label27->ForeColor = System::Drawing::Color::White;
				 this->label27->Location = System::Drawing::Point(97, 38);
				 this->label27->Name = L"label27";
				 this->label27->Size = System::Drawing::Size(35, 24);
				 this->label27->TabIndex = 3;
				 this->label27->Text = L"mv";
				 // 
				 // groupBox15
				 // 
				 this->groupBox15->Controls->Add(this->MOVIE_skip_step_box);
				 this->groupBox15->Controls->Add(this->AF_area_box);
				 this->groupBox15->Controls->Add(this->BKarea_box);
				 this->groupBox15->Controls->Add(this->BK_box);
				 this->groupBox15->Controls->Add(this->label135);
				 this->groupBox15->Controls->Add(this->label156);
				 this->groupBox15->Controls->Add(this->label8);
				 this->groupBox15->Controls->Add(this->label71);
				 this->groupBox15->Controls->Add(this->label74);
				 this->groupBox15->ForeColor = System::Drawing::Color::White;
				 this->groupBox15->Location = System::Drawing::Point(20, 21);
				 this->groupBox15->Name = L"groupBox15";
				 this->groupBox15->Size = System::Drawing::Size(125, 119);
				 this->groupBox15->TabIndex = 20;
				 this->groupBox15->TabStop = false;
				 this->groupBox15->Text = L"MOVIE-skip";
				 // 
				 // MOVIE_skip_step_box
				 // 
				 this->MOVIE_skip_step_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->MOVIE_skip_step_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MOVIE_skip_step_box->ForeColor = System::Drawing::Color::Black;
				 this->MOVIE_skip_step_box->Location = System::Drawing::Point(56, 16);
				 this->MOVIE_skip_step_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MOVIE_skip_step_box->Name = L"MOVIE_skip_step_box";
				 this->MOVIE_skip_step_box->Size = System::Drawing::Size(37, 31);
				 this->MOVIE_skip_step_box->TabIndex = 4;
				 this->MOVIE_skip_step_box->Text = L"50";
				 this->MOVIE_skip_step_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // AF_area_box
				 // 
				 this->AF_area_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->AF_area_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->AF_area_box->ForeColor = System::Drawing::Color::Black;
				 this->AF_area_box->Location = System::Drawing::Point(70, 90);
				 this->AF_area_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->AF_area_box->Name = L"AF_area_box";
				 this->AF_area_box->Size = System::Drawing::Size(48, 31);
				 this->AF_area_box->TabIndex = 4;
				 this->AF_area_box->Text = L"30";
				 this->AF_area_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // BKarea_box
				 // 
				 this->BKarea_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->BKarea_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->BKarea_box->ForeColor = System::Drawing::Color::Black;
				 this->BKarea_box->Location = System::Drawing::Point(70, 65);
				 this->BKarea_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->BKarea_box->Name = L"BKarea_box";
				 this->BKarea_box->Size = System::Drawing::Size(48, 31);
				 this->BKarea_box->TabIndex = 4;
				 this->BKarea_box->Text = L"2";
				 this->BKarea_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // BK_box
				 // 
				 this->BK_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->BK_box->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->BK_box->ForeColor = System::Drawing::Color::Black;
				 this->BK_box->Location = System::Drawing::Point(70, 40);
				 this->BK_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->BK_box->Name = L"BK_box";
				 this->BK_box->Size = System::Drawing::Size(48, 31);
				 this->BK_box->TabIndex = 4;
				 this->BK_box->Text = L"300";
				 this->BK_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label135
				 // 
				 this->label135->AutoSize = true;
				 this->label135->BackColor = System::Drawing::Color::Transparent;
				 this->label135->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label135->ForeColor = System::Drawing::Color::White;
				 this->label135->Location = System::Drawing::Point(3, 91);
				 this->label135->Name = L"label135";
				 this->label135->Size = System::Drawing::Size(73, 24);
				 this->label135->TabIndex = 3;
				 this->label135->Text = L"AF area";
				 // 
				 // label156
				 // 
				 this->label156->AutoSize = true;
				 this->label156->BackColor = System::Drawing::Color::Transparent;
				 this->label156->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label156->ForeColor = System::Drawing::Color::White;
				 this->label156->Location = System::Drawing::Point(1, 67);
				 this->label156->Name = L"label156";
				 this->label156->Size = System::Drawing::Size(93, 24);
				 this->label156->TabIndex = 3;
				 this->label156->Text = L"BKG(area)";
				 // 
				 // label8
				 // 
				 this->label8->AutoSize = true;
				 this->label8->BackColor = System::Drawing::Color::Transparent;
				 this->label8->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label8->ForeColor = System::Drawing::Color::White;
				 this->label8->Location = System::Drawing::Point(1, 45);
				 this->label8->Name = L"label8";
				 this->label8->Size = System::Drawing::Size(84, 24);
				 this->label8->TabIndex = 3;
				 this->label8->Text = L"BKG(int.)";
				 // 
				 // label71
				 // 
				 this->label71->AutoSize = true;
				 this->label71->BackColor = System::Drawing::Color::Transparent;
				 this->label71->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label71->ForeColor = System::Drawing::Color::White;
				 this->label71->Location = System::Drawing::Point(97, 22);
				 this->label71->Name = L"label71";
				 this->label71->Size = System::Drawing::Size(37, 24);
				 this->label71->TabIndex = 3;
				 this->label71->Text = L"um";
				 // 
				 // label74
				 // 
				 this->label74->AutoSize = true;
				 this->label74->BackColor = System::Drawing::Color::Transparent;
				 this->label74->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label74->ForeColor = System::Drawing::Color::White;
				 this->label74->Location = System::Drawing::Point(17, 20);
				 this->label74->Name = L"label74";
				 this->label74->Size = System::Drawing::Size(47, 24);
				 this->label74->TabIndex = 3;
				 this->label74->Text = L"Step";
				 // 
				 // Camera
				 // 
				 this->Camera->BackColor = System::Drawing::Color::Gray;
				 this->Camera->Controls->Add(this->Down2Top);
				 this->Camera->Controls->Add(this->Top2Down);
				 this->Camera->Controls->Add(this->Internal);
				 this->Camera->Controls->Add(this->ExternalExposure);
				 this->Camera->Controls->Add(this->FastReadOut);
				 this->Camera->Controls->Add(this->ScanSpeedControl_Check);
				 this->Camera->Controls->Add(this->OverlapMode);
				 this->Camera->Controls->Add(this->External);
				 this->Camera->Controls->Add(this->C2OUT);
				 this->Camera->Controls->Add(this->label142);
				 this->Camera->Controls->Add(this->label141);
				 this->Camera->Controls->Add(this->label140);
				 this->Camera->Controls->Add(this->Get_Camera_Condition);
				 this->Camera->Controls->Add(this->camera_set);
				 this->Camera->Controls->Add(this->label143);
				 this->Camera->Controls->Add(this->label148);
				 this->Camera->Controls->Add(this->label150);
				 this->Camera->Controls->Add(this->label149);
				 this->Camera->Controls->Add(this->label147);
				 this->Camera->Controls->Add(this->label146);
				 this->Camera->Controls->Add(this->label145);
				 this->Camera->Controls->Add(this->label144);
				 this->Camera->Controls->Add(this->label138);
				 this->Camera->Controls->Add(this->ScanSpeed);
				 this->Camera->Controls->Add(this->PixelReadoutRate_box);
				 this->Camera->Controls->Add(this->Scanspeedcontrol_box);
				 this->Camera->Controls->Add(this->exposuretime_box);
				 this->Camera->Controls->Add(this->overlapmode_box);
				 this->Camera->Controls->Add(this->ScanSpeed_box);
				 this->Camera->Controls->Add(this->Shuttermode_box);
				 this->Camera->Controls->Add(this->Triggermode_box);
				 this->Camera->Controls->Add(this->Windowsize);
				 this->Camera->Location = System::Drawing::Point(4, 33);
				 this->Camera->Name = L"Camera";
				 this->Camera->Padding = System::Windows::Forms::Padding(3);
				 this->Camera->Size = System::Drawing::Size(471, 283);
				 this->Camera->TabIndex = 9;
				 this->Camera->Text = L"Camera";
				 this->Camera->Click += gcnew System::EventHandler(this, &MyForm::Camera_Click);
				 // 
				 // Down2Top
				 // 
				 this->Down2Top->AutoSize = true;
				 this->Down2Top->Checked = true;
				 this->Down2Top->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->Down2Top->Location = System::Drawing::Point(30, 208);
				 this->Down2Top->Name = L"Down2Top";
				 this->Down2Top->Size = System::Drawing::Size(221, 28);
				 this->Down2Top->TabIndex = 0;
				 this->Down2Top->Text = L"Bottom Up Sequential";
				 this->Down2Top->UseVisualStyleBackColor = true;
				 this->Down2Top->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Down2Top_CheckedChanged);
				 // 
				 // Top2Down
				 // 
				 this->Top2Down->AutoSize = true;
				 this->Top2Down->Location = System::Drawing::Point(30, 186);
				 this->Top2Down->Name = L"Top2Down";
				 this->Top2Down->Size = System::Drawing::Size(215, 28);
				 this->Top2Down->TabIndex = 0;
				 this->Top2Down->Text = L"Top Down Sequential";
				 this->Top2Down->UseVisualStyleBackColor = true;
				 this->Top2Down->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Top2Down_CheckedChanged);
				 // 
				 // Internal
				 // 
				 this->Internal->AutoSize = true;
				 this->Internal->Location = System::Drawing::Point(30, 122);
				 this->Internal->Name = L"Internal";
				 this->Internal->Size = System::Drawing::Size(102, 28);
				 this->Internal->TabIndex = 0;
				 this->Internal->Text = L"Internal";
				 this->Internal->UseVisualStyleBackColor = true;
				 this->Internal->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Internal_CheckedChanged);
				 // 
				 // ExternalExposure
				 // 
				 this->ExternalExposure->AutoSize = true;
				 this->ExternalExposure->Location = System::Drawing::Point(30, 103);
				 this->ExternalExposure->Name = L"ExternalExposure";
				 this->ExternalExposure->Size = System::Drawing::Size(187, 28);
				 this->ExternalExposure->TabIndex = 0;
				 this->ExternalExposure->Text = L"External Exposure";
				 this->ExternalExposure->UseVisualStyleBackColor = true;
				 this->ExternalExposure->CheckedChanged += gcnew System::EventHandler(this, &MyForm::ExternalExposure_CheckedChanged);
				 // 
				 // FastReadOut
				 // 
				 this->FastReadOut->AutoSize = true;
				 this->FastReadOut->Location = System::Drawing::Point(247, 67);
				 this->FastReadOut->Name = L"FastReadOut";
				 this->FastReadOut->Size = System::Drawing::Size(153, 28);
				 this->FastReadOut->TabIndex = 0;
				 this->FastReadOut->Text = L"Fast Read Out";
				 this->FastReadOut->UseVisualStyleBackColor = true;
				 this->FastReadOut->CheckedChanged += gcnew System::EventHandler(this, &MyForm::FastReadOut_CheckedChanged);
				 // 
				 // ScanSpeedControl_Check
				 // 
				 this->ScanSpeedControl_Check->AutoSize = true;
				 this->ScanSpeedControl_Check->Checked = true;
				 this->ScanSpeedControl_Check->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->ScanSpeedControl_Check->Location = System::Drawing::Point(247, 48);
				 this->ScanSpeedControl_Check->Name = L"ScanSpeedControl_Check";
				 this->ScanSpeedControl_Check->Size = System::Drawing::Size(194, 28);
				 this->ScanSpeedControl_Check->TabIndex = 0;
				 this->ScanSpeedControl_Check->Text = L"Scan speed control";
				 this->ScanSpeedControl_Check->UseVisualStyleBackColor = true;
				 this->ScanSpeedControl_Check->CheckedChanged += gcnew System::EventHandler(this, &MyForm::ScanSpeedControl_Check_CheckedChanged);
				 // 
				 // OverlapMode
				 // 
				 this->OverlapMode->AutoSize = true;
				 this->OverlapMode->Location = System::Drawing::Point(247, 29);
				 this->OverlapMode->Name = L"OverlapMode";
				 this->OverlapMode->Size = System::Drawing::Size(155, 28);
				 this->OverlapMode->TabIndex = 0;
				 this->OverlapMode->Text = L"Overlap mode";
				 this->OverlapMode->UseVisualStyleBackColor = true;
				 this->OverlapMode->CheckedChanged += gcnew System::EventHandler(this, &MyForm::OverlapMode_CheckedChanged);
				 // 
				 // External
				 // 
				 this->External->AutoSize = true;
				 this->External->Checked = true;
				 this->External->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->External->Location = System::Drawing::Point(30, 86);
				 this->External->Name = L"External";
				 this->External->Size = System::Drawing::Size(105, 28);
				 this->External->TabIndex = 0;
				 this->External->Text = L"External";
				 this->External->UseVisualStyleBackColor = true;
				 this->External->CheckedChanged += gcnew System::EventHandler(this, &MyForm::External_CheckedChanged);
				 // 
				 // C2OUT
				 // 
				 this->C2OUT->AutoSize = true;
				 this->C2OUT->Location = System::Drawing::Point(30, 165);
				 this->C2OUT->Name = L"C2OUT";
				 this->C2OUT->Size = System::Drawing::Size(248, 28);
				 this->C2OUT->TabIndex = 0;
				 this->C2OUT->Text = L"Centre Out Simultaneous";
				 this->C2OUT->UseVisualStyleBackColor = true;
				 this->C2OUT->CheckedChanged += gcnew System::EventHandler(this, &MyForm::C2OUT_CheckedChanged);
				 // 
				 // label142
				 // 
				 this->label142->AutoSize = true;
				 this->label142->BackColor = System::Drawing::Color::Transparent;
				 this->label142->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label142->ForeColor = System::Drawing::Color::Black;
				 this->label142->Location = System::Drawing::Point(145, 33);
				 this->label142->Name = L"label142";
				 this->label142->Size = System::Drawing::Size(83, 24);
				 this->label142->TabIndex = 3;
				 this->label142->Text = L"pixel/ms";
				 this->label142->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label141
				 // 
				 this->label141->AutoSize = true;
				 this->label141->BackColor = System::Drawing::Color::Transparent;
				 this->label141->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label141->ForeColor = System::Drawing::Color::Black;
				 this->label141->Location = System::Drawing::Point(9, 147);
				 this->label141->Name = L"label141";
				 this->label141->Size = System::Drawing::Size(185, 24);
				 this->label141->TabIndex = 3;
				 this->label141->Text = L"Rolling shutter mode";
				 this->label141->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label140
				 // 
				 this->label140->AutoSize = true;
				 this->label140->BackColor = System::Drawing::Color::Transparent;
				 this->label140->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label140->ForeColor = System::Drawing::Color::Black;
				 this->label140->Location = System::Drawing::Point(9, 68);
				 this->label140->Name = L"label140";
				 this->label140->Size = System::Drawing::Size(119, 24);
				 this->label140->TabIndex = 3;
				 this->label140->Text = L"Trigger mode";
				 this->label140->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // Get_Camera_Condition
				 // 
				 this->Get_Camera_Condition->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Get_Camera_Condition->Font = (gcnew System::Drawing::Font(L"Calibri", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Get_Camera_Condition->ForeColor = System::Drawing::Color::Black;
				 this->Get_Camera_Condition->Location = System::Drawing::Point(93, 247);
				 this->Get_Camera_Condition->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Get_Camera_Condition->Name = L"Get_Camera_Condition";
				 this->Get_Camera_Condition->Size = System::Drawing::Size(63, 31);
				 this->Get_Camera_Condition->TabIndex = 1;
				 this->Get_Camera_Condition->Text = L"Get";
				 this->Get_Camera_Condition->UseVisualStyleBackColor = false;
				 this->Get_Camera_Condition->Click += gcnew System::EventHandler(this, &MyForm::Get_Camera_Condition_Click);
				 // 
				 // camera_set
				 // 
				 this->camera_set->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->camera_set->Font = (gcnew System::Drawing::Font(L"Calibri", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->camera_set->ForeColor = System::Drawing::Color::Black;
				 this->camera_set->Location = System::Drawing::Point(18, 247);
				 this->camera_set->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->camera_set->Name = L"camera_set";
				 this->camera_set->Size = System::Drawing::Size(63, 31);
				 this->camera_set->TabIndex = 1;
				 this->camera_set->Text = L"Set";
				 this->camera_set->UseVisualStyleBackColor = false;
				 this->camera_set->Click += gcnew System::EventHandler(this, &MyForm::camera_set_Click);
				 // 
				 // label143
				 // 
				 this->label143->AutoSize = true;
				 this->label143->BackColor = System::Drawing::Color::Transparent;
				 this->label143->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label143->ForeColor = System::Drawing::Color::Black;
				 this->label143->Location = System::Drawing::Point(14, 34);
				 this->label143->Name = L"label143";
				 this->label143->Size = System::Drawing::Size(103, 24);
				 this->label143->TabIndex = 3;
				 this->label143->Text = L"Scna speed";
				 this->label143->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label148
				 // 
				 this->label148->AutoSize = true;
				 this->label148->BackColor = System::Drawing::Color::Transparent;
				 this->label148->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label148->ForeColor = System::Drawing::Color::Black;
				 this->label148->Location = System::Drawing::Point(228, 189);
				 this->label148->Name = L"label148";
				 this->label148->Size = System::Drawing::Size(126, 24);
				 this->label148->TabIndex = 3;
				 this->label148->Text = L"overlap mode";
				 this->label148->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label150
				 // 
				 this->label150->AutoSize = true;
				 this->label150->BackColor = System::Drawing::Color::Transparent;
				 this->label150->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label150->ForeColor = System::Drawing::Color::Black;
				 this->label150->Location = System::Drawing::Point(205, 258);
				 this->label150->Name = L"label150";
				 this->label150->Size = System::Drawing::Size(161, 24);
				 this->label150->TabIndex = 3;
				 this->label150->Text = L"Pixel readout rate";
				 this->label150->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label149
				 // 
				 this->label149->AutoSize = true;
				 this->label149->BackColor = System::Drawing::Color::Transparent;
				 this->label149->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label149->ForeColor = System::Drawing::Color::Black;
				 this->label149->Location = System::Drawing::Point(202, 235);
				 this->label149->Name = L"label149";
				 this->label149->Size = System::Drawing::Size(168, 24);
				 this->label149->TabIndex = 3;
				 this->label149->Text = L"Scan speed control";
				 this->label149->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label147
				 // 
				 this->label147->AutoSize = true;
				 this->label147->BackColor = System::Drawing::Color::Transparent;
				 this->label147->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label147->ForeColor = System::Drawing::Color::Black;
				 this->label147->Location = System::Drawing::Point(224, 212);
				 this->label147->Name = L"label147";
				 this->label147->Size = System::Drawing::Size(130, 24);
				 this->label147->TabIndex = 3;
				 this->label147->Text = L"exposuer time";
				 this->label147->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label146
				 // 
				 this->label146->AutoSize = true;
				 this->label146->BackColor = System::Drawing::Color::Transparent;
				 this->label146->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label146->ForeColor = System::Drawing::Color::Black;
				 this->label146->Location = System::Drawing::Point(244, 166);
				 this->label146->Name = L"label146";
				 this->label146->Size = System::Drawing::Size(103, 24);
				 this->label146->TabIndex = 3;
				 this->label146->Text = L"Scan speed";
				 this->label146->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label145
				 // 
				 this->label145->AutoSize = true;
				 this->label145->BackColor = System::Drawing::Color::Transparent;
				 this->label145->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label145->ForeColor = System::Drawing::Color::Black;
				 this->label145->Location = System::Drawing::Point(263, 143);
				 this->label145->Name = L"label145";
				 this->label145->Size = System::Drawing::Size(71, 24);
				 this->label145->TabIndex = 3;
				 this->label145->Text = L"Shutter";
				 this->label145->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label144
				 // 
				 this->label144->AutoSize = true;
				 this->label144->BackColor = System::Drawing::Color::Transparent;
				 this->label144->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label144->ForeColor = System::Drawing::Color::Black;
				 this->label144->Location = System::Drawing::Point(267, 123);
				 this->label144->Name = L"label144";
				 this->label144->Size = System::Drawing::Size(66, 24);
				 this->label144->TabIndex = 3;
				 this->label144->Text = L"Trigger";
				 this->label144->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label138
				 // 
				 this->label138->AutoSize = true;
				 this->label138->BackColor = System::Drawing::Color::Transparent;
				 this->label138->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label138->ForeColor = System::Drawing::Color::Black;
				 this->label138->Location = System::Drawing::Point(10, 6);
				 this->label138->Name = L"label138";
				 this->label138->Size = System::Drawing::Size(155, 24);
				 this->label138->TabIndex = 3;
				 this->label138->Text = L"Window ratio  1/";
				 this->label138->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // ScanSpeed
				 // 
				 this->ScanSpeed->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ScanSpeed->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ScanSpeed->ForeColor = System::Drawing::Color::Black;
				 this->ScanSpeed->Location = System::Drawing::Point(85, 30);
				 this->ScanSpeed->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ScanSpeed->Name = L"ScanSpeed";
				 this->ScanSpeed->ReadOnly = true;
				 this->ScanSpeed->Size = System::Drawing::Size(54, 31);
				 this->ScanSpeed->TabIndex = 1;
				 this->ScanSpeed->Text = L"10.24";
				 this->ScanSpeed->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ScanSpeed->TextChanged += gcnew System::EventHandler(this, &MyForm::ScanSpeed_TextChanged);
				 // 
				 // PixelReadoutRate_box
				 // 
				 this->PixelReadoutRate_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->PixelReadoutRate_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->PixelReadoutRate_box->ForeColor = System::Drawing::Color::Black;
				 this->PixelReadoutRate_box->Location = System::Drawing::Point(314, 258);
				 this->PixelReadoutRate_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->PixelReadoutRate_box->Name = L"PixelReadoutRate_box";
				 this->PixelReadoutRate_box->ReadOnly = true;
				 this->PixelReadoutRate_box->Size = System::Drawing::Size(66, 27);
				 this->PixelReadoutRate_box->TabIndex = 13;
				 this->PixelReadoutRate_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Scanspeedcontrol_box
				 // 
				 this->Scanspeedcontrol_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Scanspeedcontrol_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Scanspeedcontrol_box->ForeColor = System::Drawing::Color::Black;
				 this->Scanspeedcontrol_box->Location = System::Drawing::Point(314, 235);
				 this->Scanspeedcontrol_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Scanspeedcontrol_box->Name = L"Scanspeedcontrol_box";
				 this->Scanspeedcontrol_box->ReadOnly = true;
				 this->Scanspeedcontrol_box->Size = System::Drawing::Size(66, 27);
				 this->Scanspeedcontrol_box->TabIndex = 13;
				 this->Scanspeedcontrol_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // exposuretime_box
				 // 
				 this->exposuretime_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->exposuretime_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->exposuretime_box->ForeColor = System::Drawing::Color::Black;
				 this->exposuretime_box->Location = System::Drawing::Point(314, 212);
				 this->exposuretime_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->exposuretime_box->Name = L"exposuretime_box";
				 this->exposuretime_box->ReadOnly = true;
				 this->exposuretime_box->Size = System::Drawing::Size(66, 27);
				 this->exposuretime_box->TabIndex = 13;
				 this->exposuretime_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // overlapmode_box
				 // 
				 this->overlapmode_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->overlapmode_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->overlapmode_box->ForeColor = System::Drawing::Color::Black;
				 this->overlapmode_box->Location = System::Drawing::Point(314, 189);
				 this->overlapmode_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->overlapmode_box->Name = L"overlapmode_box";
				 this->overlapmode_box->ReadOnly = true;
				 this->overlapmode_box->Size = System::Drawing::Size(66, 27);
				 this->overlapmode_box->TabIndex = 13;
				 this->overlapmode_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // ScanSpeed_box
				 // 
				 this->ScanSpeed_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->ScanSpeed_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->ScanSpeed_box->ForeColor = System::Drawing::Color::Black;
				 this->ScanSpeed_box->Location = System::Drawing::Point(314, 166);
				 this->ScanSpeed_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ScanSpeed_box->Name = L"ScanSpeed_box";
				 this->ScanSpeed_box->ReadOnly = true;
				 this->ScanSpeed_box->Size = System::Drawing::Size(66, 27);
				 this->ScanSpeed_box->TabIndex = 13;
				 this->ScanSpeed_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Shuttermode_box
				 // 
				 this->Shuttermode_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Shuttermode_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Shuttermode_box->ForeColor = System::Drawing::Color::Black;
				 this->Shuttermode_box->Location = System::Drawing::Point(314, 143);
				 this->Shuttermode_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Shuttermode_box->Name = L"Shuttermode_box";
				 this->Shuttermode_box->ReadOnly = true;
				 this->Shuttermode_box->Size = System::Drawing::Size(66, 27);
				 this->Shuttermode_box->TabIndex = 13;
				 this->Shuttermode_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Triggermode_box
				 // 
				 this->Triggermode_box->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->Triggermode_box->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Triggermode_box->ForeColor = System::Drawing::Color::Black;
				 this->Triggermode_box->Location = System::Drawing::Point(314, 120);
				 this->Triggermode_box->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Triggermode_box->Name = L"Triggermode_box";
				 this->Triggermode_box->ReadOnly = true;
				 this->Triggermode_box->Size = System::Drawing::Size(66, 27);
				 this->Triggermode_box->TabIndex = 13;
				 this->Triggermode_box->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Windowsize
				 // 
				 this->Windowsize->BackColor = System::Drawing::Color::Gray;
				 this->Windowsize->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->Windowsize->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Windowsize->ForeColor = System::Drawing::Color::Black;
				 this->Windowsize->Location = System::Drawing::Point(107, 6);
				 this->Windowsize->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Windowsize->Name = L"Windowsize";
				 this->Windowsize->Size = System::Drawing::Size(30, 24);
				 this->Windowsize->TabIndex = 1;
				 this->Windowsize->Text = L"6";
				 this->Windowsize->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
				 this->Windowsize->TextChanged += gcnew System::EventHandler(this, &MyForm::Windowsize_TextChanged);
				 // 
				 // Calibration
				 // 
				 this->Calibration->BackColor = System::Drawing::Color::Gray;
				 this->Calibration->Controls->Add(this->L647_temp);
				 this->Calibration->Controls->Add(this->groupBox12);
				 this->Calibration->Controls->Add(this->L592_temp);
				 this->Calibration->Controls->Add(this->groupBox13);
				 this->Calibration->Controls->Add(this->LL);
				 this->Calibration->Controls->Add(this->label59);
				 this->Calibration->Controls->Add(this->L488_temp);
				 this->Calibration->Controls->Add(this->RR);
				 this->Calibration->Controls->Add(this->groupBox11);
				 this->Calibration->Controls->Add(this->Sheet_Step);
				 this->Calibration->Controls->Add(this->ETL_swept_time);
				 this->Calibration->Controls->Add(this->sweep_max);
				 this->Calibration->Controls->Add(this->sheet3);
				 this->Calibration->Controls->Add(this->sheet2);
				 this->Calibration->Controls->Add(this->sheet1);
				 this->Calibration->Controls->Add(this->sweep_min);
				 this->Calibration->Controls->Add(this->X_LR);
				 this->Calibration->Controls->Add(this->groupBox14);
				 this->Calibration->Controls->Add(this->z_zero);
				 this->Calibration->Controls->Add(this->Zweep);
				 this->Calibration->Controls->Add(this->ETL_Sweep);
				 this->Calibration->Controls->Add(this->Galvo_Zero);
				 this->Calibration->Controls->Add(this->label14);
				 this->Calibration->Location = System::Drawing::Point(4, 33);
				 this->Calibration->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Calibration->Name = L"Calibration";
				 this->Calibration->Padding = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Calibration->Size = System::Drawing::Size(471, 283);
				 this->Calibration->TabIndex = 3;
				 this->Calibration->Text = L"Cal";
				 // 
				 // L647_temp
				 // 
				 this->L647_temp->AutoSize = true;
				 this->L647_temp->Location = System::Drawing::Point(283, 240);
				 this->L647_temp->Name = L"L647_temp";
				 this->L647_temp->Size = System::Drawing::Size(20, 24);
				 this->L647_temp->TabIndex = 35;
				 this->L647_temp->Text = L"0";
				 // 
				 // groupBox12
				 // 
				 this->groupBox12->Controls->Add(this->button2);
				 this->groupBox12->Controls->Add(this->textBox1);
				 this->groupBox12->Controls->Add(this->button5);
				 this->groupBox12->Controls->Add(this->textBox2);
				 this->groupBox12->Controls->Add(this->button4);
				 this->groupBox12->Controls->Add(this->textBox3);
				 this->groupBox12->Controls->Add(this->textBox4);
				 this->groupBox12->Controls->Add(this->button3);
				 this->groupBox12->Controls->Add(this->label63);
				 this->groupBox12->Controls->Add(this->label64);
				 this->groupBox12->Controls->Add(this->label65);
				 this->groupBox12->Controls->Add(this->label66);
				 this->groupBox12->Location = System::Drawing::Point(343, 151);
				 this->groupBox12->Name = L"groupBox12";
				 this->groupBox12->Size = System::Drawing::Size(125, 138);
				 this->groupBox12->TabIndex = 36;
				 this->groupBox12->TabStop = false;
				 this->groupBox12->Text = L"stage pos";
				 // 
				 // button2
				 // 
				 this->button2->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->button2->ForeColor = System::Drawing::Color::Black;
				 this->button2->Location = System::Drawing::Point(81, 19);
				 this->button2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->button2->Name = L"button2";
				 this->button2->Size = System::Drawing::Size(34, 25);
				 this->button2->TabIndex = 5;
				 this->button2->Text = L"set";
				 this->button2->UseVisualStyleBackColor = true;
				 this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click_1);
				 // 
				 // textBox1
				 // 
				 this->textBox1->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->textBox1->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->textBox1->ForeColor = System::Drawing::Color::Black;
				 this->textBox1->Location = System::Drawing::Point(27, 20);
				 this->textBox1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->textBox1->Name = L"textBox1";
				 this->textBox1->Size = System::Drawing::Size(49, 31);
				 this->textBox1->TabIndex = 4;
				 this->textBox1->Text = L"0";
				 this->textBox1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // button5
				 // 
				 this->button5->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->button5->ForeColor = System::Drawing::Color::Black;
				 this->button5->Location = System::Drawing::Point(81, 106);
				 this->button5->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->button5->Name = L"button5";
				 this->button5->Size = System::Drawing::Size(34, 25);
				 this->button5->TabIndex = 8;
				 this->button5->Text = L"set";
				 this->button5->UseVisualStyleBackColor = true;
				 // 
				 // textBox2
				 // 
				 this->textBox2->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->textBox2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->textBox2->ForeColor = System::Drawing::Color::Black;
				 this->textBox2->Location = System::Drawing::Point(27, 78);
				 this->textBox2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->textBox2->Name = L"textBox2";
				 this->textBox2->Size = System::Drawing::Size(49, 31);
				 this->textBox2->TabIndex = 4;
				 this->textBox2->Text = L"0";
				 this->textBox2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // button4
				 // 
				 this->button4->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->button4->ForeColor = System::Drawing::Color::Black;
				 this->button4->Location = System::Drawing::Point(81, 48);
				 this->button4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->button4->Name = L"button4";
				 this->button4->Size = System::Drawing::Size(34, 25);
				 this->button4->TabIndex = 7;
				 this->button4->Text = L"set";
				 this->button4->UseVisualStyleBackColor = true;
				 this->button4->Click += gcnew System::EventHandler(this, &MyForm::button4_Click);
				 // 
				 // textBox3
				 // 
				 this->textBox3->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->textBox3->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->textBox3->ForeColor = System::Drawing::Color::Black;
				 this->textBox3->Location = System::Drawing::Point(27, 49);
				 this->textBox3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->textBox3->Name = L"textBox3";
				 this->textBox3->Size = System::Drawing::Size(49, 31);
				 this->textBox3->TabIndex = 4;
				 this->textBox3->Text = L"0";
				 this->textBox3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // textBox4
				 // 
				 this->textBox4->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->textBox4->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->textBox4->ForeColor = System::Drawing::Color::Black;
				 this->textBox4->Location = System::Drawing::Point(27, 107);
				 this->textBox4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->textBox4->Name = L"textBox4";
				 this->textBox4->Size = System::Drawing::Size(49, 31);
				 this->textBox4->TabIndex = 4;
				 this->textBox4->Text = L"0";
				 this->textBox4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // button3
				 // 
				 this->button3->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->button3->ForeColor = System::Drawing::Color::Black;
				 this->button3->Location = System::Drawing::Point(81, 77);
				 this->button3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->button3->Name = L"button3";
				 this->button3->Size = System::Drawing::Size(34, 25);
				 this->button3->TabIndex = 6;
				 this->button3->Text = L"set";
				 this->button3->UseVisualStyleBackColor = true;
				 this->button3->Click += gcnew System::EventHandler(this, &MyForm::button3_Click_1);
				 // 
				 // label63
				 // 
				 this->label63->AutoSize = true;
				 this->label63->BackColor = System::Drawing::Color::Transparent;
				 this->label63->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label63->ForeColor = System::Drawing::Color::Black;
				 this->label63->Location = System::Drawing::Point(9, 24);
				 this->label63->Name = L"label63";
				 this->label63->Size = System::Drawing::Size(27, 24);
				 this->label63->TabIndex = 5;
				 this->label63->Text = L"X:";
				 // 
				 // label64
				 // 
				 this->label64->AutoSize = true;
				 this->label64->BackColor = System::Drawing::Color::Transparent;
				 this->label64->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label64->ForeColor = System::Drawing::Color::Black;
				 this->label64->Location = System::Drawing::Point(10, 53);
				 this->label64->Name = L"label64";
				 this->label64->Size = System::Drawing::Size(25, 24);
				 this->label64->TabIndex = 5;
				 this->label64->Text = L"Y:";
				 // 
				 // label65
				 // 
				 this->label65->AutoSize = true;
				 this->label65->BackColor = System::Drawing::Color::Transparent;
				 this->label65->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label65->ForeColor = System::Drawing::Color::Black;
				 this->label65->Location = System::Drawing::Point(10, 82);
				 this->label65->Name = L"label65";
				 this->label65->Size = System::Drawing::Size(26, 24);
				 this->label65->TabIndex = 5;
				 this->label65->Text = L"Z:";
				 // 
				 // label66
				 // 
				 this->label66->AutoSize = true;
				 this->label66->BackColor = System::Drawing::Color::Transparent;
				 this->label66->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label66->ForeColor = System::Drawing::Color::Black;
				 this->label66->Location = System::Drawing::Point(7, 111);
				 this->label66->Name = L"label66";
				 this->label66->Size = System::Drawing::Size(30, 24);
				 this->label66->TabIndex = 5;
				 this->label66->Text = L"Θ:";
				 // 
				 // L592_temp
				 // 
				 this->L592_temp->AutoSize = true;
				 this->L592_temp->Location = System::Drawing::Point(283, 225);
				 this->L592_temp->Name = L"L592_temp";
				 this->L592_temp->Size = System::Drawing::Size(20, 24);
				 this->L592_temp->TabIndex = 35;
				 this->L592_temp->Text = L"0";
				 // 
				 // groupBox13
				 // 
				 this->groupBox13->Controls->Add(this->sheet_offset_R);
				 this->groupBox13->Controls->Add(this->theta_offset);
				 this->groupBox13->Controls->Add(this->sheet_offset_L);
				 this->groupBox13->Controls->Add(this->label118);
				 this->groupBox13->Controls->Add(this->label83);
				 this->groupBox13->Controls->Add(this->label82);
				 this->groupBox13->Controls->Add(this->sheet);
				 this->groupBox13->Location = System::Drawing::Point(195, 69);
				 this->groupBox13->Name = L"groupBox13";
				 this->groupBox13->Size = System::Drawing::Size(123, 100);
				 this->groupBox13->TabIndex = 32;
				 this->groupBox13->TabStop = false;
				 this->groupBox13->Text = L"offset";
				 // 
				 // sheet_offset_R
				 // 
				 this->sheet_offset_R->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->sheet_offset_R->Location = System::Drawing::Point(81, 32);
				 this->sheet_offset_R->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->sheet_offset_R->Name = L"sheet_offset_R";
				 this->sheet_offset_R->Size = System::Drawing::Size(33, 28);
				 this->sheet_offset_R->TabIndex = 0;
				 this->sheet_offset_R->Text = L"0";
				 this->sheet_offset_R->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // theta_offset
				 // 
				 this->theta_offset->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->theta_offset->Location = System::Drawing::Point(49, 63);
				 this->theta_offset->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->theta_offset->Name = L"theta_offset";
				 this->theta_offset->Size = System::Drawing::Size(60, 28);
				 this->theta_offset->TabIndex = 0;
				 this->theta_offset->Text = L"-2803";
				 this->theta_offset->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->theta_offset->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::mspim_offset_L_KeyDown);
				 // 
				 // sheet_offset_L
				 // 
				 this->sheet_offset_L->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->sheet_offset_L->Location = System::Drawing::Point(43, 32);
				 this->sheet_offset_L->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->sheet_offset_L->Name = L"sheet_offset_L";
				 this->sheet_offset_L->Size = System::Drawing::Size(36, 28);
				 this->sheet_offset_L->TabIndex = 0;
				 this->sheet_offset_L->Text = L"0";
				 this->sheet_offset_L->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->sheet_offset_L->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::sheet_offset_L_KeyDown);
				 // 
				 // label118
				 // 
				 this->label118->AutoSize = true;
				 this->label118->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label118->ForeColor = System::Drawing::Color::Black;
				 this->label118->Location = System::Drawing::Point(12, 66);
				 this->label118->Name = L"label118";
				 this->label118->Size = System::Drawing::Size(21, 21);
				 this->label118->TabIndex = 3;
				 this->label118->Text = L"Θ";
				 this->label118->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label83
				 // 
				 this->label83->AutoSize = true;
				 this->label83->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label83->ForeColor = System::Drawing::Color::Black;
				 this->label83->Location = System::Drawing::Point(88, 20);
				 this->label83->Name = L"label83";
				 this->label83->Size = System::Drawing::Size(19, 21);
				 this->label83->TabIndex = 3;
				 this->label83->Text = L"R";
				 this->label83->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label82
				 // 
				 this->label82->AutoSize = true;
				 this->label82->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label82->ForeColor = System::Drawing::Color::Black;
				 this->label82->Location = System::Drawing::Point(57, 20);
				 this->label82->Name = L"label82";
				 this->label82->Size = System::Drawing::Size(17, 21);
				 this->label82->TabIndex = 3;
				 this->label82->Text = L"L";
				 this->label82->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // sheet
				 // 
				 this->sheet->AutoSize = true;
				 this->sheet->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->sheet->ForeColor = System::Drawing::Color::Black;
				 this->sheet->Location = System::Drawing::Point(1, 36);
				 this->sheet->Name = L"sheet";
				 this->sheet->Size = System::Drawing::Size(48, 21);
				 this->sheet->TabIndex = 3;
				 this->sheet->Text = L"sheet";
				 this->sheet->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // LL
				 // 
				 this->LL->Location = System::Drawing::Point(57, 257);
				 this->LL->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->LL->Name = L"LL";
				 this->LL->Size = System::Drawing::Size(54, 31);
				 this->LL->TabIndex = 25;
				 this->LL->Text = L"0";
				 this->LL->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label59
				 // 
				 this->label59->AutoSize = true;
				 this->label59->Location = System::Drawing::Point(235, 191);
				 this->label59->Name = L"label59";
				 this->label59->Size = System::Drawing::Size(102, 24);
				 this->label59->TabIndex = 35;
				 this->label59->Text = L"Laser temp";
				 // 
				 // L488_temp
				 // 
				 this->L488_temp->AutoSize = true;
				 this->L488_temp->Location = System::Drawing::Point(283, 210);
				 this->L488_temp->Name = L"L488_temp";
				 this->L488_temp->Size = System::Drawing::Size(20, 24);
				 this->L488_temp->TabIndex = 35;
				 this->L488_temp->Text = L"0";
				 // 
				 // RR
				 // 
				 this->RR->Location = System::Drawing::Point(3, 257);
				 this->RR->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->RR->Name = L"RR";
				 this->RR->Size = System::Drawing::Size(54, 31);
				 this->RR->TabIndex = 25;
				 this->RR->Text = L"0";
				 this->RR->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // groupBox11
				 // 
				 this->groupBox11->Controls->Add(this->filter_name8);
				 this->groupBox11->Controls->Add(this->filter_name4);
				 this->groupBox11->Controls->Add(this->filter_name7);
				 this->groupBox11->Controls->Add(this->filter_name3);
				 this->groupBox11->Controls->Add(this->Filter_wheel_SPD);
				 this->groupBox11->Controls->Add(this->filter_name6);
				 this->groupBox11->Controls->Add(this->Theta_SIGMA);
				 this->groupBox11->Controls->Add(this->filter_name2);
				 this->groupBox11->Controls->Add(this->filter_name5);
				 this->groupBox11->Controls->Add(this->filter_name1);
				 this->groupBox11->Controls->Add(this->label4);
				 this->groupBox11->Controls->Add(this->label81);
				 this->groupBox11->Controls->Add(this->label103);
				 this->groupBox11->Controls->Add(this->Theta_SIGMA_but);
				 this->groupBox11->Controls->Add(this->label99);
				 this->groupBox11->Controls->Add(this->label102);
				 this->groupBox11->Controls->Add(this->label98);
				 this->groupBox11->Controls->Add(this->label101);
				 this->groupBox11->Controls->Add(this->label97);
				 this->groupBox11->Controls->Add(this->label100);
				 this->groupBox11->Controls->Add(this->label96);
				 this->groupBox11->Location = System::Drawing::Point(14, 69);
				 this->groupBox11->Name = L"groupBox11";
				 this->groupBox11->Size = System::Drawing::Size(162, 173);
				 this->groupBox11->TabIndex = 30;
				 this->groupBox11->TabStop = false;
				 this->groupBox11->Text = L"Filter setting";
				 // 
				 // filter_name8
				 // 
				 this->filter_name8->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name8->Location = System::Drawing::Point(93, 89);
				 this->filter_name8->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name8->Name = L"filter_name8";
				 this->filter_name8->Size = System::Drawing::Size(52, 24);
				 this->filter_name8->TabIndex = 25;
				 this->filter_name8->Text = L"blank";
				 this->filter_name8->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name4
				 // 
				 this->filter_name4->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name4->Location = System::Drawing::Point(19, 89);
				 this->filter_name4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name4->Name = L"filter_name4";
				 this->filter_name4->Size = System::Drawing::Size(55, 24);
				 this->filter_name4->TabIndex = 25;
				 this->filter_name4->Text = L"676/29";
				 this->filter_name4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name7
				 // 
				 this->filter_name7->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name7->Location = System::Drawing::Point(93, 66);
				 this->filter_name7->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name7->Name = L"filter_name7";
				 this->filter_name7->Size = System::Drawing::Size(52, 24);
				 this->filter_name7->TabIndex = 25;
				 this->filter_name7->Text = L"blank";
				 this->filter_name7->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name3
				 // 
				 this->filter_name3->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name3->Location = System::Drawing::Point(19, 66);
				 this->filter_name3->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name3->Name = L"filter_name3";
				 this->filter_name3->Size = System::Drawing::Size(55, 24);
				 this->filter_name3->TabIndex = 25;
				 this->filter_name3->Text = L"625/25";
				 this->filter_name3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->filter_name3->TextChanged += gcnew System::EventHandler(this, &MyForm::filter_name3_TextChanged);
				 // 
				 // Filter_wheel_SPD
				 // 
				 this->Filter_wheel_SPD->Location = System::Drawing::Point(89, 128);
				 this->Filter_wheel_SPD->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Filter_wheel_SPD->Name = L"Filter_wheel_SPD";
				 this->Filter_wheel_SPD->Size = System::Drawing::Size(54, 31);
				 this->Filter_wheel_SPD->TabIndex = 25;
				 this->Filter_wheel_SPD->Tag = L"";
				 this->Filter_wheel_SPD->Text = L"2000";
				 this->Filter_wheel_SPD->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name6
				 // 
				 this->filter_name6->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name6->Location = System::Drawing::Point(93, 43);
				 this->filter_name6->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name6->Name = L"filter_name6";
				 this->filter_name6->Size = System::Drawing::Size(52, 24);
				 this->filter_name6->TabIndex = 25;
				 this->filter_name6->Text = L"blank";
				 this->filter_name6->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // Theta_SIGMA
				 // 
				 this->Theta_SIGMA->Location = System::Drawing::Point(23, 127);
				 this->Theta_SIGMA->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_SIGMA->Name = L"Theta_SIGMA";
				 this->Theta_SIGMA->Size = System::Drawing::Size(54, 31);
				 this->Theta_SIGMA->TabIndex = 25;
				 this->Theta_SIGMA->Text = L"5100";
				 this->Theta_SIGMA->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name2
				 // 
				 this->filter_name2->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name2->Location = System::Drawing::Point(19, 43);
				 this->filter_name2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name2->Name = L"filter_name2";
				 this->filter_name2->Size = System::Drawing::Size(55, 24);
				 this->filter_name2->TabIndex = 25;
				 this->filter_name2->Text = L"575/25";
				 this->filter_name2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->filter_name2->TextChanged += gcnew System::EventHandler(this, &MyForm::filter_name2_TextChanged);
				 // 
				 // filter_name5
				 // 
				 this->filter_name5->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name5->Location = System::Drawing::Point(93, 20);
				 this->filter_name5->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name5->Name = L"filter_name5";
				 this->filter_name5->Size = System::Drawing::Size(52, 24);
				 this->filter_name5->TabIndex = 25;
				 this->filter_name5->Text = L"700/LP";
				 this->filter_name5->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // filter_name1
				 // 
				 this->filter_name1->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->filter_name1->Location = System::Drawing::Point(19, 20);
				 this->filter_name1->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->filter_name1->Name = L"filter_name1";
				 this->filter_name1->Size = System::Drawing::Size(55, 24);
				 this->filter_name1->TabIndex = 25;
				 this->filter_name1->Text = L"525/50";
				 this->filter_name1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->filter_name1->TextChanged += gcnew System::EventHandler(this, &MyForm::filter_name1_TextChanged);
				 // 
				 // label4
				 // 
				 this->label4->AutoSize = true;
				 this->label4->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label4->ForeColor = System::Drawing::Color::Black;
				 this->label4->Location = System::Drawing::Point(31, 111);
				 this->label4->Name = L"label4";
				 this->label4->Size = System::Drawing::Size(56, 22);
				 this->label4->TabIndex = 23;
				 this->label4->Text = L"offset";
				 // 
				 // label81
				 // 
				 this->label81->AutoSize = true;
				 this->label81->Font = (gcnew System::Drawing::Font(L"Meiryo UI", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label81->ForeColor = System::Drawing::Color::Black;
				 this->label81->Location = System::Drawing::Point(90, 111);
				 this->label81->Name = L"label81";
				 this->label81->Size = System::Drawing::Size(66, 22);
				 this->label81->TabIndex = 23;
				 this->label81->Text = L"SPEED";
				 // 
				 // label103
				 // 
				 this->label103->AutoSize = true;
				 this->label103->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label103->ForeColor = System::Drawing::Color::Black;
				 this->label103->Location = System::Drawing::Point(5, 90);
				 this->label103->Name = L"label103";
				 this->label103->Size = System::Drawing::Size(24, 21);
				 this->label103->TabIndex = 23;
				 this->label103->Text = L"4:";
				 // 
				 // Theta_SIGMA_but
				 // 
				 this->Theta_SIGMA_but->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->Theta_SIGMA_but->Location = System::Drawing::Point(109, 150);
				 this->Theta_SIGMA_but->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Theta_SIGMA_but->Name = L"Theta_SIGMA_but";
				 this->Theta_SIGMA_but->Size = System::Drawing::Size(32, 21);
				 this->Theta_SIGMA_but->TabIndex = 4;
				 this->Theta_SIGMA_but->Text = L"Go";
				 this->Theta_SIGMA_but->UseVisualStyleBackColor = true;
				 this->Theta_SIGMA_but->Click += gcnew System::EventHandler(this, &MyForm::Theta_SIGMA_but_Click);
				 // 
				 // label99
				 // 
				 this->label99->AutoSize = true;
				 this->label99->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label99->ForeColor = System::Drawing::Color::Black;
				 this->label99->Location = System::Drawing::Point(80, 90);
				 this->label99->Name = L"label99";
				 this->label99->Size = System::Drawing::Size(24, 21);
				 this->label99->TabIndex = 23;
				 this->label99->Text = L"8:";
				 // 
				 // label102
				 // 
				 this->label102->AutoSize = true;
				 this->label102->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label102->ForeColor = System::Drawing::Color::Black;
				 this->label102->Location = System::Drawing::Point(5, 68);
				 this->label102->Name = L"label102";
				 this->label102->Size = System::Drawing::Size(24, 21);
				 this->label102->TabIndex = 23;
				 this->label102->Text = L"3:";
				 // 
				 // label98
				 // 
				 this->label98->AutoSize = true;
				 this->label98->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label98->ForeColor = System::Drawing::Color::Black;
				 this->label98->Location = System::Drawing::Point(80, 67);
				 this->label98->Name = L"label98";
				 this->label98->Size = System::Drawing::Size(24, 21);
				 this->label98->TabIndex = 23;
				 this->label98->Text = L"7:";
				 // 
				 // label101
				 // 
				 this->label101->AutoSize = true;
				 this->label101->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label101->ForeColor = System::Drawing::Color::Black;
				 this->label101->Location = System::Drawing::Point(5, 45);
				 this->label101->Name = L"label101";
				 this->label101->Size = System::Drawing::Size(24, 21);
				 this->label101->TabIndex = 23;
				 this->label101->Text = L"2:";
				 // 
				 // label97
				 // 
				 this->label97->AutoSize = true;
				 this->label97->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label97->ForeColor = System::Drawing::Color::Black;
				 this->label97->Location = System::Drawing::Point(80, 44);
				 this->label97->Name = L"label97";
				 this->label97->Size = System::Drawing::Size(24, 21);
				 this->label97->TabIndex = 23;
				 this->label97->Text = L"6:";
				 // 
				 // label100
				 // 
				 this->label100->AutoSize = true;
				 this->label100->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label100->ForeColor = System::Drawing::Color::Black;
				 this->label100->Location = System::Drawing::Point(5, 23);
				 this->label100->Name = L"label100";
				 this->label100->Size = System::Drawing::Size(24, 21);
				 this->label100->TabIndex = 23;
				 this->label100->Text = L"1:";
				 // 
				 // label96
				 // 
				 this->label96->AutoSize = true;
				 this->label96->Font = (gcnew System::Drawing::Font(L"Calibri", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label96->ForeColor = System::Drawing::Color::Black;
				 this->label96->Location = System::Drawing::Point(80, 22);
				 this->label96->Name = L"label96";
				 this->label96->Size = System::Drawing::Size(24, 21);
				 this->label96->TabIndex = 23;
				 this->label96->Text = L"5:";
				 // 
				 // Sheet_Step
				 // 
				 this->Sheet_Step->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Sheet_Step->Location = System::Drawing::Point(254, 37);
				 this->Sheet_Step->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Sheet_Step->Name = L"Sheet_Step";
				 this->Sheet_Step->Size = System::Drawing::Size(22, 25);
				 this->Sheet_Step->TabIndex = 4;
				 this->Sheet_Step->Text = L"S";
				 this->Sheet_Step->UseVisualStyleBackColor = true;
				 this->Sheet_Step->Click += gcnew System::EventHandler(this, &MyForm::Sheet_Step_Click);
				 // 
				 // ETL_swept_time
				 // 
				 this->ETL_swept_time->Location = System::Drawing::Point(219, 9);
				 this->ETL_swept_time->Name = L"ETL_swept_time";
				 this->ETL_swept_time->Size = System::Drawing::Size(33, 31);
				 this->ETL_swept_time->TabIndex = 29;
				 this->ETL_swept_time->Text = L"500";
				 this->ETL_swept_time->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // sweep_max
				 // 
				 this->sweep_max->Location = System::Drawing::Point(178, 8);
				 this->sweep_max->Name = L"sweep_max";
				 this->sweep_max->Size = System::Drawing::Size(33, 31);
				 this->sweep_max->TabIndex = 29;
				 this->sweep_max->Text = L"200";
				 this->sweep_max->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // sheet3
				 // 
				 this->sheet3->Location = System::Drawing::Point(219, 37);
				 this->sheet3->Name = L"sheet3";
				 this->sheet3->Size = System::Drawing::Size(33, 31);
				 this->sheet3->TabIndex = 29;
				 this->sheet3->Text = L"0";
				 this->sheet3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // sheet2
				 // 
				 this->sheet2->Location = System::Drawing::Point(180, 37);
				 this->sheet2->Name = L"sheet2";
				 this->sheet2->Size = System::Drawing::Size(33, 31);
				 this->sheet2->TabIndex = 29;
				 this->sheet2->Text = L"0";
				 this->sheet2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // sheet1
				 // 
				 this->sheet1->Location = System::Drawing::Point(142, 37);
				 this->sheet1->Name = L"sheet1";
				 this->sheet1->Size = System::Drawing::Size(33, 31);
				 this->sheet1->TabIndex = 29;
				 this->sheet1->Text = L"0";
				 this->sheet1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // sweep_min
				 // 
				 this->sweep_min->Location = System::Drawing::Point(143, 8);
				 this->sweep_min->Name = L"sweep_min";
				 this->sweep_min->Size = System::Drawing::Size(33, 31);
				 this->sweep_min->TabIndex = 29;
				 this->sweep_min->Text = L"100";
				 this->sweep_min->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // X_LR
				 // 
				 this->X_LR->BackColor = System::Drawing::Color::LightGray;
				 this->X_LR->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->X_LR->ForeColor = System::Drawing::Color::Black;
				 this->X_LR->Location = System::Drawing::Point(113, 256);
				 this->X_LR->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->X_LR->Name = L"X_LR";
				 this->X_LR->Size = System::Drawing::Size(44, 25);
				 this->X_LR->TabIndex = 0;
				 this->X_LR->Text = L"Start";
				 this->X_LR->UseVisualStyleBackColor = false;
				 this->X_LR->Click += gcnew System::EventHandler(this, &MyForm::X_LR_Click);
				 // 
				 // groupBox14
				 // 
				 this->groupBox14->Controls->Add(this->MicroStep2);
				 this->groupBox14->Controls->Add(this->MicroStep1);
				 this->groupBox14->Controls->Add(this->MicroStep4);
				 this->groupBox14->Controls->Add(this->MicroStep8);
				 this->groupBox14->Controls->Add(this->MicroStep10);
				 this->groupBox14->Controls->Add(this->MicroStep20);
				 this->groupBox14->Location = System::Drawing::Point(370, 5);
				 this->groupBox14->Name = L"groupBox14";
				 this->groupBox14->Size = System::Drawing::Size(95, 144);
				 this->groupBox14->TabIndex = 32;
				 this->groupBox14->TabStop = false;
				 this->groupBox14->Text = L"Microstep";
				 // 
				 // MicroStep2
				 // 
				 this->MicroStep2->AutoSize = true;
				 this->MicroStep2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep2->ForeColor = System::Drawing::Color::White;
				 this->MicroStep2->Location = System::Drawing::Point(15, 41);
				 this->MicroStep2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep2->Name = L"MicroStep2";
				 this->MicroStep2->Size = System::Drawing::Size(46, 28);
				 this->MicroStep2->TabIndex = 0;
				 this->MicroStep2->Text = L"2";
				 this->MicroStep2->UseVisualStyleBackColor = true;
				 this->MicroStep2->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep2_CheckedChanged);
				 // 
				 // MicroStep4
				 // 
				 this->MicroStep4->AutoSize = true;
				 this->MicroStep4->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep4->ForeColor = System::Drawing::Color::White;
				 this->MicroStep4->Location = System::Drawing::Point(15, 60);
				 this->MicroStep4->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep4->Name = L"MicroStep4";
				 this->MicroStep4->Size = System::Drawing::Size(46, 28);
				 this->MicroStep4->TabIndex = 0;
				 this->MicroStep4->Text = L"4";
				 this->MicroStep4->UseVisualStyleBackColor = true;
				 this->MicroStep4->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep4_CheckedChanged);
				 // 
				 // MicroStep8
				 // 
				 this->MicroStep8->AutoSize = true;
				 this->MicroStep8->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep8->ForeColor = System::Drawing::Color::White;
				 this->MicroStep8->Location = System::Drawing::Point(15, 79);
				 this->MicroStep8->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep8->Name = L"MicroStep8";
				 this->MicroStep8->Size = System::Drawing::Size(46, 28);
				 this->MicroStep8->TabIndex = 0;
				 this->MicroStep8->Text = L"8";
				 this->MicroStep8->UseVisualStyleBackColor = true;
				 this->MicroStep8->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep8_CheckedChanged);
				 // 
				 // MicroStep10
				 // 
				 this->MicroStep10->AutoSize = true;
				 this->MicroStep10->Checked = true;
				 this->MicroStep10->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->MicroStep10->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep10->ForeColor = System::Drawing::Color::White;
				 this->MicroStep10->Location = System::Drawing::Point(15, 98);
				 this->MicroStep10->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep10->Name = L"MicroStep10";
				 this->MicroStep10->Size = System::Drawing::Size(56, 28);
				 this->MicroStep10->TabIndex = 0;
				 this->MicroStep10->Text = L"10";
				 this->MicroStep10->UseVisualStyleBackColor = true;
				 this->MicroStep10->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep10_CheckedChanged);
				 // 
				 // MicroStep20
				 // 
				 this->MicroStep20->AutoSize = true;
				 this->MicroStep20->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MicroStep20->ForeColor = System::Drawing::Color::White;
				 this->MicroStep20->Location = System::Drawing::Point(15, 117);
				 this->MicroStep20->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MicroStep20->Name = L"MicroStep20";
				 this->MicroStep20->Size = System::Drawing::Size(56, 28);
				 this->MicroStep20->TabIndex = 0;
				 this->MicroStep20->Text = L"20";
				 this->MicroStep20->UseVisualStyleBackColor = true;
				 this->MicroStep20->CheckedChanged += gcnew System::EventHandler(this, &MyForm::MicroStep20_CheckedChanged);
				 // 
				 // z_zero
				 // 
				 this->z_zero->BackColor = System::Drawing::Color::LightGray;
				 this->z_zero->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->z_zero->ForeColor = System::Drawing::Color::Black;
				 this->z_zero->Location = System::Drawing::Point(0, 35);
				 this->z_zero->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->z_zero->Name = L"z_zero";
				 this->z_zero->Size = System::Drawing::Size(70, 25);
				 this->z_zero->TabIndex = 0;
				 this->z_zero->Text = L"Zpos-zero";
				 this->z_zero->UseVisualStyleBackColor = false;
				 this->z_zero->Click += gcnew System::EventHandler(this, &MyForm::z_zero_Click);
				 // 
				 // Zweep
				 // 
				 this->Zweep->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Zweep->Location = System::Drawing::Point(72, 35);
				 this->Zweep->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Zweep->Name = L"Zweep";
				 this->Zweep->Size = System::Drawing::Size(63, 25);
				 this->Zweep->TabIndex = 4;
				 this->Zweep->Text = L"ZSweep";
				 this->Zweep->UseVisualStyleBackColor = true;
				 this->Zweep->Click += gcnew System::EventHandler(this, &MyForm::Zweep_Click);
				 // 
				 // ETL_Sweep
				 // 
				 this->ETL_Sweep->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ETL_Sweep->Location = System::Drawing::Point(72, 7);
				 this->ETL_Sweep->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ETL_Sweep->Name = L"ETL_Sweep";
				 this->ETL_Sweep->Size = System::Drawing::Size(68, 25);
				 this->ETL_Sweep->TabIndex = 4;
				 this->ETL_Sweep->Text = L"Sweep";
				 this->ETL_Sweep->UseVisualStyleBackColor = true;
				 this->ETL_Sweep->Click += gcnew System::EventHandler(this, &MyForm::ETL_Sweep_Click_1);
				 // 
				 // Galvo_Zero
				 // 
				 this->Galvo_Zero->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Galvo_Zero->Location = System::Drawing::Point(1, 8);
				 this->Galvo_Zero->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Galvo_Zero->Name = L"Galvo_Zero";
				 this->Galvo_Zero->Size = System::Drawing::Size(69, 23);
				 this->Galvo_Zero->TabIndex = 4;
				 this->Galvo_Zero->Text = L"Galvo_Zero";
				 this->Galvo_Zero->UseVisualStyleBackColor = true;
				 this->Galvo_Zero->Click += gcnew System::EventHandler(this, &MyForm::Galvo_Zero_Click);
				 // 
				 // label14
				 // 
				 this->label14->AutoSize = true;
				 this->label14->BackColor = System::Drawing::Color::Transparent;
				 this->label14->Font = (gcnew System::Drawing::Font(L"Calibri", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label14->ForeColor = System::Drawing::Color::White;
				 this->label14->Location = System::Drawing::Point(251, 11);
				 this->label14->Name = L"label14";
				 this->label14->Size = System::Drawing::Size(34, 24);
				 this->label14->TabIndex = 3;
				 this->label14->Text = L"ms";
				 this->label14->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label35
				 // 
				 this->label35->AutoSize = true;
				 this->label35->BackColor = System::Drawing::Color::Transparent;
				 this->label35->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label35->ForeColor = System::Drawing::Color::White;
				 this->label35->Location = System::Drawing::Point(145, 14);
				 this->label35->Name = L"label35";
				 this->label35->Size = System::Drawing::Size(126, 24);
				 this->label35->TabIndex = 3;
				 this->label35->Text = L"MOVIE Focus:";
				 this->label35->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // MFocus_right
				 // 
				 this->MFocus_right->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->MFocus_right->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->MFocus_right->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MFocus_right->ForeColor = System::Drawing::Color::Black;
				 this->MFocus_right->Location = System::Drawing::Point(421, 13);
				 this->MFocus_right->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MFocus_right->Name = L"MFocus_right";
				 this->MFocus_right->Size = System::Drawing::Size(39, 24);
				 this->MFocus_right->TabIndex = 4;
				 this->MFocus_right->Text = L"0";
				 this->MFocus_right->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->MFocus_right->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->MFocus_right->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // MFocus_left
				 // 
				 this->MFocus_left->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->MFocus_left->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->MFocus_left->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->MFocus_left->ForeColor = System::Drawing::Color::Black;
				 this->MFocus_left->Location = System::Drawing::Point(368, 13);
				 this->MFocus_left->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->MFocus_left->Name = L"MFocus_left";
				 this->MFocus_left->Size = System::Drawing::Size(39, 24);
				 this->MFocus_left->TabIndex = 4;
				 this->MFocus_left->Text = L"0";
				 this->MFocus_left->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->MFocus_left->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->MFocus_left->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // label109
				 // 
				 this->label109->AutoSize = true;
				 this->label109->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label109->ForeColor = System::Drawing::Color::White;
				 this->label109->Location = System::Drawing::Point(495, 13);
				 this->label109->Name = L"label109";
				 this->label109->Size = System::Drawing::Size(24, 22);
				 this->label109->TabIndex = 37;
				 this->label109->Text = L"X:";
				 this->label109->Click += gcnew System::EventHandler(this, &MyForm::label43_Click);
				 // 
				 // Xpos_cursol
				 // 
				 this->Xpos_cursol->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Xpos_cursol->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->Xpos_cursol->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Xpos_cursol->ForeColor = System::Drawing::Color::Black;
				 this->Xpos_cursol->Location = System::Drawing::Point(517, 12);
				 this->Xpos_cursol->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Xpos_cursol->Name = L"Xpos_cursol";
				 this->Xpos_cursol->ReadOnly = true;
				 this->Xpos_cursol->Size = System::Drawing::Size(46, 24);
				 this->Xpos_cursol->TabIndex = 100;
				 this->Xpos_cursol->Text = L"0";
				 this->Xpos_cursol->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Xpos_cursol->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->Xpos_cursol->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // label112
				 // 
				 this->label112->AutoSize = true;
				 this->label112->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label112->ForeColor = System::Drawing::Color::White;
				 this->label112->Location = System::Drawing::Point(575, 13);
				 this->label112->Name = L"label112";
				 this->label112->Size = System::Drawing::Size(23, 22);
				 this->label112->TabIndex = 37;
				 this->label112->Text = L"Y:";
				 this->label112->Click += gcnew System::EventHandler(this, &MyForm::label43_Click);
				 // 
				 // Ypos_cursol
				 // 
				 this->Ypos_cursol->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Ypos_cursol->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->Ypos_cursol->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Ypos_cursol->ForeColor = System::Drawing::Color::Black;
				 this->Ypos_cursol->Location = System::Drawing::Point(601, 12);
				 this->Ypos_cursol->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Ypos_cursol->Name = L"Ypos_cursol";
				 this->Ypos_cursol->ReadOnly = true;
				 this->Ypos_cursol->Size = System::Drawing::Size(46, 24);
				 this->Ypos_cursol->TabIndex = 100;
				 this->Ypos_cursol->Text = L"0";
				 this->Ypos_cursol->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Ypos_cursol->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->Ypos_cursol->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // label116
				 // 
				 this->label116->AutoSize = true;
				 this->label116->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label116->ForeColor = System::Drawing::Color::White;
				 this->label116->Location = System::Drawing::Point(654, 13);
				 this->label116->Name = L"label116";
				 this->label116->Size = System::Drawing::Size(57, 22);
				 this->label116->TabIndex = 37;
				 this->label116->Text = L"Signal:";
				 this->label116->Click += gcnew System::EventHandler(this, &MyForm::label43_Click);
				 // 
				 // Cursol_int
				 // 
				 this->Cursol_int->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->Cursol_int->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->Cursol_int->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Cursol_int->ForeColor = System::Drawing::Color::Black;
				 this->Cursol_int->Location = System::Drawing::Point(707, 12);
				 this->Cursol_int->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Cursol_int->Name = L"Cursol_int";
				 this->Cursol_int->ReadOnly = true;
				 this->Cursol_int->Size = System::Drawing::Size(65, 24);
				 this->Cursol_int->TabIndex = 100;
				 this->Cursol_int->Text = L"0";
				 this->Cursol_int->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->Cursol_int->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->Cursol_int->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // ma
				 // 
				 this->ma->AutoSize = true;
				 this->ma->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ma->ForeColor = System::Drawing::Color::White;
				 this->ma->Location = System::Drawing::Point(781, 13);
				 this->ma->Name = L"ma";
				 this->ma->Size = System::Drawing::Size(46, 22);
				 this->ma->TabIndex = 37;
				 this->ma->Text = L"max:";
				 this->ma->Click += gcnew System::EventHandler(this, &MyForm::label43_Click);
				 // 
				 // qw
				 // 
				 this->qw->AutoSize = true;
				 this->qw->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->qw->ForeColor = System::Drawing::Color::White;
				 this->qw->Location = System::Drawing::Point(896, 13);
				 this->qw->Name = L"qw";
				 this->qw->Size = System::Drawing::Size(42, 22);
				 this->qw->TabIndex = 37;
				 this->qw->Text = L"min:";
				 this->qw->Click += gcnew System::EventHandler(this, &MyForm::label43_Click);
				 // 
				 // ImageMax
				 // 
				 this->ImageMax->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->ImageMax->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->ImageMax->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ImageMax->ForeColor = System::Drawing::Color::Black;
				 this->ImageMax->Location = System::Drawing::Point(822, 12);
				 this->ImageMax->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ImageMax->Name = L"ImageMax";
				 this->ImageMax->ReadOnly = true;
				 this->ImageMax->Size = System::Drawing::Size(65, 24);
				 this->ImageMax->TabIndex = 100;
				 this->ImageMax->Text = L"0";
				 this->ImageMax->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ImageMax->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->ImageMax->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // ImageMin
				 // 
				 this->ImageMin->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->ImageMin->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->ImageMin->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->ImageMin->ForeColor = System::Drawing::Color::Black;
				 this->ImageMin->Location = System::Drawing::Point(936, 12);
				 this->ImageMin->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ImageMin->Name = L"ImageMin";
				 this->ImageMin->ReadOnly = true;
				 this->ImageMin->Size = System::Drawing::Size(65, 24);
				 this->ImageMin->TabIndex = 100;
				 this->ImageMin->Text = L"0";
				 this->ImageMin->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ImageMin->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->ImageMin->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // groupBox9
				 // 
				 this->groupBox9->BackColor = System::Drawing::Color::Gray;
				 this->groupBox9->Controls->Add(this->ImageMin);
				 this->groupBox9->Controls->Add(this->qw);
				 this->groupBox9->Controls->Add(this->Xpos_cursol);
				 this->groupBox9->Controls->Add(this->ma);
				 this->groupBox9->Controls->Add(this->label116);
				 this->groupBox9->Controls->Add(this->Ypos_cursol);
				 this->groupBox9->Controls->Add(this->MFocus_right);
				 this->groupBox9->Controls->Add(this->DCTFocusValBox);
				 this->groupBox9->Controls->Add(this->DCTScoreBox);
				 this->groupBox9->Controls->Add(this->DCT_calc_time);
				 this->groupBox9->Controls->Add(this->MFocus_left);
				 this->groupBox9->Controls->Add(this->label84);
				 this->groupBox9->Controls->Add(this->label55);
				 this->groupBox9->Controls->Add(this->label35);
				 this->groupBox9->Controls->Add(this->label112);
				 this->groupBox9->Controls->Add(this->label109);
				 this->groupBox9->Controls->Add(this->Cursol_int);
				 this->groupBox9->Controls->Add(this->ImageMax);
				 this->groupBox9->Font = (gcnew System::Drawing::Font(L"メイリオ", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->groupBox9->ForeColor = System::Drawing::Color::White;
				 this->groupBox9->Location = System::Drawing::Point(6, 664);
				 this->groupBox9->Margin = System::Windows::Forms::Padding(0);
				 this->groupBox9->Name = L"groupBox9";
				 this->groupBox9->Size = System::Drawing::Size(1012, 35);
				 this->groupBox9->TabIndex = 101;
				 this->groupBox9->TabStop = false;
				 this->groupBox9->Text = L"Image status";
				 // 
				 // DCTFocusValBox
				 // 
				 this->DCTFocusValBox->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->DCTFocusValBox->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->DCTFocusValBox->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->DCTFocusValBox->ForeColor = System::Drawing::Color::Black;
				 this->DCTFocusValBox->Location = System::Drawing::Point(88, 16);
				 this->DCTFocusValBox->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->DCTFocusValBox->Name = L"DCTFocusValBox";
				 this->DCTFocusValBox->Size = System::Drawing::Size(44, 24);
				 this->DCTFocusValBox->TabIndex = 4;
				 this->DCTFocusValBox->Text = L"0";
				 this->DCTFocusValBox->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->DCTFocusValBox->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->DCTFocusValBox->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // DCTScoreBox
				 // 
				 this->DCTScoreBox->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->DCTScoreBox->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->DCTScoreBox->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->DCTScoreBox->ForeColor = System::Drawing::Color::Black;
				 this->DCTScoreBox->Location = System::Drawing::Point(229, 13);
				 this->DCTScoreBox->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->DCTScoreBox->Name = L"DCTScoreBox";
				 this->DCTScoreBox->Size = System::Drawing::Size(44, 24);
				 this->DCTScoreBox->TabIndex = 4;
				 this->DCTScoreBox->Text = L"0";
				 this->DCTScoreBox->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->DCTScoreBox->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->DCTScoreBox->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // DCT_calc_time
				 // 
				 this->DCT_calc_time->BackColor = System::Drawing::SystemColors::ButtonHighlight;
				 this->DCT_calc_time->BorderStyle = System::Windows::Forms::BorderStyle::None;
				 this->DCT_calc_time->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->DCT_calc_time->ForeColor = System::Drawing::Color::Black;
				 this->DCT_calc_time->Location = System::Drawing::Point(298, 13);
				 this->DCT_calc_time->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->DCT_calc_time->Name = L"DCT_calc_time";
				 this->DCT_calc_time->Size = System::Drawing::Size(39, 24);
				 this->DCT_calc_time->TabIndex = 4;
				 this->DCT_calc_time->Text = L"0";
				 this->DCT_calc_time->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->DCT_calc_time->TextChanged += gcnew System::EventHandler(this, &MyForm::Start_Z_pos_TextChanged_1);
				 this->DCT_calc_time->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Start_Z_pos_KeyDown);
				 // 
				 // label84
				 // 
				 this->label84->AutoSize = true;
				 this->label84->BackColor = System::Drawing::Color::Transparent;
				 this->label84->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label84->ForeColor = System::Drawing::Color::White;
				 this->label84->Location = System::Drawing::Point(6, 17);
				 this->label84->Name = L"label84";
				 this->label84->Size = System::Drawing::Size(110, 24);
				 this->label84->TabIndex = 3;
				 this->label84->Text = L"FocusValue:";
				 this->label84->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label55
				 // 
				 this->label55->AutoSize = true;
				 this->label55->BackColor = System::Drawing::Color::Transparent;
				 this->label55->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label55->ForeColor = System::Drawing::Color::White;
				 this->label55->Location = System::Drawing::Point(343, 12);
				 this->label55->Name = L"label55";
				 this->label55->Size = System::Drawing::Size(34, 24);
				 this->label55->TabIndex = 3;
				 this->label55->Text = L"ms";
				 this->label55->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // label113
				 // 
				 this->label113->AutoSize = true;
				 this->label113->BackColor = System::Drawing::Color::Transparent;
				 this->label113->Font = (gcnew System::Drawing::Font(L"Calibri", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label113->ForeColor = System::Drawing::Color::White;
				 this->label113->Location = System::Drawing::Point(61, 155);
				 this->label113->Name = L"label113";
				 this->label113->Size = System::Drawing::Size(114, 22);
				 this->label113->TabIndex = 3;
				 this->label113->Text = L"Exposure time";
				 this->label113->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // ExpoTime_Axial
				 // 
				 this->ExpoTime_Axial->BackColor = System::Drawing::SystemColors::ControlLight;
				 this->ExpoTime_Axial->Font = (gcnew System::Drawing::Font(L"ＭＳ ゴシック", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->ExpoTime_Axial->ForeColor = System::Drawing::Color::Black;
				 this->ExpoTime_Axial->Location = System::Drawing::Point(153, 154);
				 this->ExpoTime_Axial->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->ExpoTime_Axial->Name = L"ExpoTime_Axial";
				 this->ExpoTime_Axial->ReadOnly = true;
				 this->ExpoTime_Axial->Size = System::Drawing::Size(54, 27);
				 this->ExpoTime_Axial->TabIndex = 13;
				 this->ExpoTime_Axial->Text = L"5.00";
				 this->ExpoTime_Axial->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->ExpoTime_Axial->TextChanged += gcnew System::EventHandler(this, &MyForm::exposure_time_box_TextChanged);
				 this->ExpoTime_Axial->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::exposure_time_box_KeyDown);
				 // 
				 // label117
				 // 
				 this->label117->AutoSize = true;
				 this->label117->BackColor = System::Drawing::Color::Transparent;
				 this->label117->Font = (gcnew System::Drawing::Font(L"Calibri", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->label117->ForeColor = System::Drawing::Color::White;
				 this->label117->Location = System::Drawing::Point(209, 155);
				 this->label117->Name = L"label117";
				 this->label117->Size = System::Drawing::Size(81, 24);
				 this->label117->TabIndex = 3;
				 this->label117->Text = L"ms/pixel";
				 this->label117->Click += gcnew System::EventHandler(this, &MyForm::label5_Click);
				 // 
				 // groupBox10
				 // 
				 this->groupBox10->BackColor = System::Drawing::Color::Gray;
				 this->groupBox10->Controls->Add(this->Axial_mode_check);
				 this->groupBox10->Controls->Add(this->Single_Acquire);
				 this->groupBox10->Controls->Add(this->CameraNo_2);
				 this->groupBox10->Controls->Add(this->Camera_1_use);
				 this->groupBox10->Controls->Add(this->Camera_2_use);
				 this->groupBox10->Controls->Add(this->CameraNo_1);
				 this->groupBox10->Controls->Add(this->LIVE_but);
				 this->groupBox10->Controls->Add(this->Abort);
				 this->groupBox10->Controls->Add(this->label113);
				 this->groupBox10->Controls->Add(this->label16);
				 this->groupBox10->Controls->Add(this->label117);
				 this->groupBox10->Controls->Add(this->label20);
				 this->groupBox10->Controls->Add(this->ExpoTime_Axial);
				 this->groupBox10->Controls->Add(this->exposure_time_box);
				 this->groupBox10->Location = System::Drawing::Point(744, 31);
				 this->groupBox10->Name = L"groupBox10";
				 this->groupBox10->Size = System::Drawing::Size(276, 200);
				 this->groupBox10->TabIndex = 106;
				 this->groupBox10->TabStop = false;
				 // 
				 // CameraNo_2
				 // 
				 this->CameraNo_2->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->CameraNo_2->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->CameraNo_2->ForeColor = System::Drawing::Color::Black;
				 this->CameraNo_2->Location = System::Drawing::Point(150, 34);
				 this->CameraNo_2->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->CameraNo_2->Name = L"CameraNo_2";
				 this->CameraNo_2->ReadOnly = true;
				 this->CameraNo_2->Size = System::Drawing::Size(116, 31);
				 this->CameraNo_2->TabIndex = 0;
				 // 
				 // Camera_1_use
				 // 
				 this->Camera_1_use->AutoSize = true;
				 this->Camera_1_use->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Camera_1_use->ForeColor = System::Drawing::Color::White;
				 this->Camera_1_use->Location = System::Drawing::Point(39, 14);
				 this->Camera_1_use->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Camera_1_use->Name = L"Camera_1_use";
				 this->Camera_1_use->Size = System::Drawing::Size(115, 28);
				 this->Camera_1_use->TabIndex = 0;
				 this->Camera_1_use->Text = L"Camera 1";
				 this->Camera_1_use->UseVisualStyleBackColor = true;
				 this->Camera_1_use->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Camera_1_use_CheckedChanged);
				 // 
				 // Camera_2_use
				 // 
				 this->Camera_2_use->AutoSize = true;
				 this->Camera_2_use->Font = (gcnew System::Drawing::Font(L"Calibri", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 this->Camera_2_use->ForeColor = System::Drawing::Color::White;
				 this->Camera_2_use->Location = System::Drawing::Point(39, 36);
				 this->Camera_2_use->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Camera_2_use->Name = L"Camera_2_use";
				 this->Camera_2_use->Size = System::Drawing::Size(115, 28);
				 this->Camera_2_use->TabIndex = 0;
				 this->Camera_2_use->Text = L"Camera 2";
				 this->Camera_2_use->UseVisualStyleBackColor = true;
				 this->Camera_2_use->CheckedChanged += gcnew System::EventHandler(this, &MyForm::Camera_1_use_CheckedChanged);
				 // 
				 // Camera_ini2
				 // 
				 this->Camera_ini2->WorkerSupportsCancellation = true;
				 this->Camera_ini2->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Camera_ini2_DoWork);
				 // 
				 // backgroundWorker1
				 // 
				 this->backgroundWorker1->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker1_DoWork);
				 // 
				 // Theta_ini
				 // 
				 this->Theta_ini->WorkerSupportsCancellation = true;
				 this->Theta_ini->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Theta_ini_DoWork);
				 // 
				 // BW_ETL_check
				 // 
				 this->BW_ETL_check->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::BW_ETL_check_DoWork);
				 // 
				 // backgroundWorker_L592
				 // 
				 this->backgroundWorker_L592->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker_L592_DoWork);
				 // 
				 // backgroundWorker_L488
				 // 
				 this->backgroundWorker_L488->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker_L488_DoWork);
				 // 
				 // backgroundWorker_L647
				 // 
				 this->backgroundWorker_L647->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker_L647_DoWork);
				 // 
				 // Surveillance
				 // 
				 this->Surveillance->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::Surveillance_DoWork);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(11, 20);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->BackColor = System::Drawing::Color::DimGray;
				 this->ClientSize = System::Drawing::Size(1030, 730);
				 this->Controls->Add(this->groupBox10);
				 this->Controls->Add(this->groupBox9);
				 this->Controls->Add(this->groupBox8);
				 this->Controls->Add(this->statusbar);
				 this->Controls->Add(this->groupBox7);
				 this->Controls->Add(this->groupBox6);
				 this->Controls->Add(this->groupBox5);
				 this->Controls->Add(this->groupBox4);
				 this->Controls->Add(this->groupBox3);
				 this->Controls->Add(this->groupBox2);
				 this->Controls->Add(this->close);
				 this->Controls->Add(this->aa);
				 this->Controls->Add(this->exposure_time_bar);
				 this->Controls->Add(this->menuStrip1);
				 this->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->ForeColor = System::Drawing::Color::Black;
				 this->Icon = (cli::safe_cast<System::Drawing::Icon^>(resources->GetObject(L"$this.Icon")));
				 this->MainMenuStrip = this->menuStrip1;
				 this->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
				 this->Name = L"MyForm";
				 this->SizeGripStyle = System::Windows::Forms::SizeGripStyle::Show;
				 this->Text = L"GPS Imager";
				 this->TextChanged += gcnew System::EventHandler(this, &MyForm::MyForm_TextChanged);
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Amplitude_sheet_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L488_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L592_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->L647_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->exposure_time_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->ETL_bar))->EndInit();
				 this->groupBox2->ResumeLayout(false);
				 this->groupBox2->PerformLayout();
				 this->groupBox3->ResumeLayout(false);
				 this->groupBox3->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SheetSPD_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->Sheet_bar))->EndInit();
				 this->groupBox4->ResumeLayout(false);
				 this->groupBox4->PerformLayout();
				 this->groupBox5->ResumeLayout(false);
				 this->groupBox5->PerformLayout();
				 this->groupBox6->ResumeLayout(false);
				 this->groupBox6->PerformLayout();
				 this->groupBox7->ResumeLayout(false);
				 this->groupBox7->PerformLayout();
				 this->statusbar->ResumeLayout(false);
				 this->statusbar->PerformLayout();
				 this->menuStrip1->ResumeLayout(false);
				 this->menuStrip1->PerformLayout();
				 this->groupBox8->ResumeLayout(false);
				 this->groupBox8->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar_Z))->EndInit();
				 this->tabPage2->ResumeLayout(false);
				 this->tabPage2->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_low1_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_low2_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_high2_bar))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->SH_high1_bar))->EndInit();
				 this->aa->ResumeLayout(false);
				 this->tabPage3->ResumeLayout(false);
				 this->tabPage3->PerformLayout();
				 this->groupBox1->ResumeLayout(false);
				 this->groupBox1->PerformLayout();
				 this->groupBox16->ResumeLayout(false);
				 this->groupBox16->PerformLayout();
				 this->tabPage1->ResumeLayout(false);
				 this->tabPage1->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar_manual_focus))->EndInit();
				 this->group_focus->ResumeLayout(false);
				 this->group_focus->PerformLayout();
				 this->groupBox15->ResumeLayout(false);
				 this->groupBox15->PerformLayout();
				 this->Camera->ResumeLayout(false);
				 this->Camera->PerformLayout();
				 this->Calibration->ResumeLayout(false);
				 this->Calibration->PerformLayout();
				 this->groupBox12->ResumeLayout(false);
				 this->groupBox12->PerformLayout();
				 this->groupBox13->ResumeLayout(false);
				 this->groupBox13->PerformLayout();
				 this->groupBox11->ResumeLayout(false);
				 this->groupBox11->PerformLayout();
				 this->groupBox14->ResumeLayout(false);
				 this->groupBox14->PerformLayout();
				 this->groupBox9->ResumeLayout(false);
				 this->groupBox9->PerformLayout();
				 this->groupBox10->ResumeLayout(false);
				 this->groupBox10->PerformLayout();
				 this->ResumeLayout(false);
				 this->PerformLayout();

			 }
#pragma endregion

	private:void Orca_Stop() {
		cout << "Orca capture stop" << endl;
		Orca.buf_flag = 0;
		dcamwait_abort(Orca.hwait);
	}
	private:void Stage_Stop() {
		Fstage_STOP(TSerials.Fstage_Serial);
		Zstage_Flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIX:STOP\r");
		XYstage_port->Write("AXIY:STOP\r");
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP1/0\r");
		Sleep(100);
		Zstage_Flag = 0;

	}

	private: inline void Z_Galvo(double v) {
		double Sample = 2;
		double rate = 10000;
		double data[4] = {};
		char Dev[200];
		data[0] = v;
		data[1] = v;
		data[2] = v;
		data[3] = v;
		DAQ2_single(Sample, rate, data);
	}

			 //**********************************************
			 //Directory 
			 //***********************************************
	private: bool Dir_check(String^ Dir_name) {
		string dir_name;
		string dir_s;
		MarshalString(Dir_name, dir_name);
		int pos1 = 0;
		int pos2 = 4;
		while (1) {
			if (dir_name.find('\\', pos2) != std::string::npos) {
				pos2 = dir_name.find('\\', pos2);
				dir_s = dir_name.substr(0, pos2);
				checkExistenceOfFolder(dir_s);
				pos2++;
				//cout << "Created directory: " << dir_s << endl;
			}
			else {
				checkExistenceOfFolder(dir_name);
				break;
			}
		}
		return true;
	}

			 //**********************************************
			 //Camera control
			 //***********************************************

	private: void CameraSettings_Axial(AT_H Hndl) {

		//cout << "\n************************************************************" << endl;
		//cout << "             CameraSettings\n";
		//cout << "**************************************************************" << endl;
		int iret = AT_SetEnumeratedString(Hndl, L"PixelEncoding", L"Mono16");
		if (iret == AT_SUCCESS) {
			//Console::WriteLine("Camera " + Hndl + " Pixel Encoding set to Mono16 ");
		}
		double Exp = double::Parse(ExpoTime_Axial->Text);

		Exp = Exp / 1000.0;

		iret = AT_SetEnumeratedString(Hndl, L"TriggerMode", L"External");
		if (iret != AT_SUCCESS) {
			Console::WriteLine("Camera " + Hndl + ":Error setting trigger mode to External retcode = " + iret);
		}
		if (Axial_mode_check->Checked == true) {
			double Window = 2304 / (double::Parse(windowsize_box->Text));
			iret = AT_SetBool(Hndl, L"ScanSpeedControlEnable", AT_TRUE);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera " + Hndl + "ScanSpeedControl  Enable");
			}
			else {
				Console::WriteLine("Camera " + Hndl + "ScanSpeedControl set error: " + iret);
			}
			if (Exp > 0.200) {
				int iret = AT_SetEnumeratedString(Hndl, L"PixelReadoutRate", L"100 MHz");
				if (iret == AT_SUCCESS) {
					Console::WriteLine("Camera " + Hndl + "PixelReadoutRate: 100 MHz");
				}
				else {
					Console::WriteLine("PixelReadoutRate set error: 100 MHz  ");
				}
			}
			else {
				int iret = AT_SetEnumeratedString(Hndl, L"PixelReadoutRate", L"280 MHz");
				if (iret == AT_SUCCESS) {
					Console::WriteLine("Camera " + Hndl + "PixelReadoutRate: 280 MHz");
				}
				else {
					Console::WriteLine("PixelReadoutRate set error: 100 MHz  ");
				}
			}
			if (Down2Top->Checked == true) {
				iret = AT_SetEnumeratedString(Hndl, L"SensorReadoutMode", L"Bottom Up Sequential");
				if (iret == AT_SUCCESS) {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Bottom Up Sequential");
				}
				else {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Bottom Up Sequential set error iret = " + iret);
				}
			}
			else if (Top2Down->Checked == true) {
				iret = AT_SetEnumeratedString(Hndl, L"SensorReadoutMode", L"Top Down Sequential");
				if (iret == AT_SUCCESS) {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Top Down Sequential");
				}
				else {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Top Down Sequential set error iret = " + iret);
				}
			}
			else if (C2OUT->Checked == true) {

				iret = AT_SetEnumeratedString(Hndl, L"SensorReadoutMode", L"Centre Out Simultaneous");
				if (iret == AT_SUCCESS) {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Centre Out Simultaneous");
				}
				else {
					Console::WriteLine("Camera " + Hndl + "SensorReadoutMode: Centre Out Simultaneous set error iret = " + iret);
				}
			}
			double speed = double::Parse(ScanSpeed->Text);
			speed = speed * 1000;
			iret = AT_SetFloat(Hndl, L"LineScanSpeed", speed);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera " + Hndl + "LineScanSpeed set: " + speed);
			}
			else {
				Console::WriteLine("Camera " + Hndl + "LineScanSpeed set :" + speed + " error: " + iret);
			}
			iret = AT_SetInt(Hndl, L"ExposedPixelHeight", Window);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera " + Hndl + "ExposedPixelHeight set " + Window + " pixels");
			}
			else {
				Console::WriteLine("Camera " + Hndl + "ExposedPixelHeight set error " + Window + "pixels iret = " + iret);
			}

			iret = AT_SetFloat(Hndl, L"ExposureTime", Exp);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera " + Hndl + "Exposure Time set: " + Exp + " s");
				Console::WriteLine("Camera " + Hndl + "Window size: " + Window + " pixel");
			}
			else {
				//	Console::WriteLine("Exposure Time set: " + Exp * Window / 2304 + " s");
				//	Console::WriteLine("Exposure Time set error: " + iret);
				AT_BOOL Implemented; //To determine if Exposure time is implemented by the camera 
				AT_IsImplemented(Hndl, L"ExposureTime", &Implemented);
				AT_BOOL ReadOnly; //To determine if Exposure time a Read Only Feature
				AT_IsReadOnly(Hndl, L"ExposureTime", &ReadOnly);
				double Min = 0;
				double Max = 0;
				if (Implemented == AT_TRUE) {
					//Get the Limits for Exposure Time double Min, Max;
					AT_GetFloatMin(Hndl, L"ExposureTime", &Min);
					AT_GetFloatMax(Hndl, L"ExposureTime", &Max);
					cout << "ExposureTime MAX: " << Max << " s/ Min:" << Min << " s" << endl;
					AT_BOOL Writable, Readable;
					AT_IsWritable(Hndl, L"ExposureTime", &Writable);
					AT_IsReadable(Hndl, L"ExposureTime", &Readable);
					cout << "Writable: " << Writable << " / Readable:" << Readable << endl;
					double ExposureTime_current;
					if (Readable == AT_TRUE) { //To get the current value of Exposure time in //microseconds double ExposureTime;
						AT_GetFloat(Hndl, L"ExposureTime", &ExposureTime_current);
						cout << "Current ExposureTime: " << ExposureTime_current << endl;
					}
				}
			}
		}

		else {
			iret = AT_SetFloat(Hndl, L"ExposureTime", Exp);

			iret = AT_SetBool(Hndl, L"ScanSpeedControlEnable", AT_FALSE);
			if (iret == AT_SUCCESS) {
				//	Console::WriteLine("ScanSpeedControl  Enable");
			}
			else {
				Console::WriteLine("ScanSpeedControl set error: " + iret);
			}
			if (iret == AT_SUCCESS) {
				//	Console::WriteLine("Exposure Time set: " + Exp * 1000 + " ms");
			}
			else {
				Console::WriteLine("Exposure Time set error: " + iret);
			}
		}
		iret = AT_SetEnumeratedString(Hndl, L"CycleMode", L"Continuous");
		if (iret != AT_SUCCESS) {
			Console::WriteLine("Camera " + Hndl + ":Error setting Cycle Mode to Continuous, retcode=" + iret);
		}
		else {
			//Console::WriteLine("Camera #1:CycleMode set to Continuous");
		}
		//cout << "************************************************************" << endl << endl;
		//AlignedBuffers_Prep(Hndl);
		//getUserSettings(Hndl);
		GetCameraSettings(Hndl);
	}

	private: void ScanSpeed_Calc() {
		double Exp = double::Parse(exposure_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		//delay_time_box->Text = (Exp / (WINDOWSIZE + 3)).ToString();
		double del = double::Parse(delay_time_box->Text);
		double Window = 2304 / (double::Parse(windowsize_box->Text));
		EXTRA_TIME = double::Parse(Extra_time_box->Text);
		double sspeed = (2304 + Window) / (Exp - del - EXTRA_TIME);
		//double sspeed = (2304) / (Exp - del - EXTRA_TIME);
		ScanSpeed->Text = sspeed.ToString();
		Orca.scanspeed = sspeed;
		if (Axial_mode_check->Checked == true) {
			ExpoTime_Axial->Text = (Window / sspeed).ToString("F2");
		}
		else {
			ExpoTime_Axial->Text = (Exp - del - EXTRA_TIME).ToString("F2");
		}
	}

	private: delegate System::Void Dele_imagenumd();
	private: delegate System::Void Dele_intd(String ^str);
	private: delegate System::Void Dele_maxd(double max);
	private: delegate System::Void Dele_mind(double min);
	private: delegate System::Void Dele_xposd(String ^str);
	private: delegate System::Void Dele_AFleft(String ^str);
	private: delegate System::Void Dele_AFright(String ^str);
	private: delegate System::Void Dele_AFcenter(String ^str);
	private: delegate System::Void Dele_MouseIvent();
	private: delegate System::Void Dele_FocusSet(String^ Focus);
	private: delegate System::Void Dele_LFocus(String^ Focus);
	private: delegate System::Void Dele_RFocus(String^ Focus);


	private:  System::Void mouseivent() {

		if (ActiveWindow == 1) {
			if (mparam.event == cv::EVENT_LBUTTONDOWN &&mparam.flags == 9) {
				LIVE1.mouse_move_flag = 0;
				LIVE1.x0_live = mparam.x; LIVE1.y0_live = mparam.y;//R+Ctrl
				Image_Zoom_Func(LIVE1.x_live, LIVE1.x0_live, LIVE1.y_live, LIVE1.y0_live, LIVE1.width_live, LIVE1.hight_live);
				Sleep(100);
			}
			if (mparam.event == cv::EVENT_RBUTTONDOWN) {
				LIVE1.mouse_move_flag = 0;
				LIVE1.x0_live = mparam.x; LIVE1.y0_live = mparam.y;
				Image_ZoomOut_Func(LIVE1.x_live, LIVE1.x0_live, LIVE1.y_live, LIVE1.y0_live, LIVE1.width_live, LIVE1.hight_live);
				Sleep(100);
			}
			if (mparam.event == cv::EVENT_MBUTTONDOWN) {
				if (LIVE1.mouse_move_flag == 0) {
					LIVE1.mouse_move_flag = 1;
					LIVE1.x_m_live = mparam.x; LIVE1.y_m_live = mparam.y;
				}
			}
			if (mparam.event == cv::EVENT_MBUTTONUP) {
				LIVE1.mouse_move_flag = 0;
			}
			if (mparam.event == cv::EVENT_MOUSEMOVE) {
				if (LIVE1.mouse_move_flag == 1) {
					LIVE1.x_d_live = mparam.x; LIVE1.y_d_live = mparam.y;
					LIVE1.x_live = LIVE1.x_live - LIVE1.x_d_live + LIVE1.x_m_live;
					LIVE1.y_live = LIVE1.y_live - LIVE1.y_d_live + LIVE1.y_m_live;
					LIVE1.x_m_live = LIVE1.x_d_live;
					LIVE1.y_m_live = LIVE1.y_d_live;
				}
				LIVE1.x_on_live = mparam.x;
				LIVE1.y_on_live = mparam.y;
			}
		}


		else {};

		mouseevent_flag = 0;
	}
	private: void DAQ_sheet_TASK_prep(double *data, const double &SampleNum, char *Dev) {
		double Ampli_sheet = double::Parse(Amplitude_sheet->Text);
		char Dev1[] = SHEET_GALV_R;//Right-sheet
		char Dev2[] = SHEET_GALV_L;//Left-sheet
		double sheet_offset = 0;
		cout << "Sheet SampleNum: " << SampleNum << endl;
		if (illumination_flag == 'L') {
			sheet_offset = double::Parse(sheet_offset_L->Text);
			double a = 0;
			int i = 0;
			for (i = 0; Dev2[i] != '\0'; i++)
				Dev[i] = Dev2[i];
			Dev[i] = '\0';
		}
		if (illumination_flag == 'R') {
			sheet_offset = double::Parse(sheet_offset_R->Text);
			double a = 0;
			int i = 0;
			for (i = 0; Dev1[i] != '\0'; i++)
				Dev[i] = Dev1[i];
			Dev[i] = '\0';
		}
		double max = Ampli_sheet + sheet_offset;
		double min = sheet_offset - Ampli_sheet;
		double delta = 4.0 * Ampli_sheet / SampleNum;
		double v = max;
		//double v2 = max;
		int di = 1;
		for (int i = 0; i < SampleNum; ++i) {
			v = v - delta * di;
			if (v < min) {
				//cout << v << "/" << min;
				double del = min - v;
				v = min + del;
				if (v < -10) {
					v = -10;
				}
				//cout << " - "<<v << "/" << min;
				di = -1;
			}
			if (v > max) {
				double del = abs(v) - abs(max);
				v = max - del;
				if (v > 10) {
					v = 10;
				}
				di = 1;
			}
			data[i] = v;
			//v = v2;
		}
		/*
		try {
			String^ filename = ".\\DAQ_sheet.txt";
			StreamWriter^ sw = gcnew StreamWriter(filename);
			for (int i = 0; i < SampleNum; ++i)
				sw->WriteLine(i + "," + data[i]);
			sw->Close();
		}
		catch (exception &e) {

		}*/

	}

	private: void DAQ_ETL_TASK_prep(double *data, const int &SampleNum, char *Dev) {
		double Ampli_sheet = double::Parse(Amplitude_sheet->Text);
		//Dev = "cDAQ1Mod1/ao0,cDAQ1Mod1/ao1\0";
		double Exposuretime = double::Parse(exposure_time_box->Text) * 20;
		double delay = double::Parse(delay_time_box->Text) * 20;
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		double exp = Exposuretime - EXTRA_TIME * 20 - delay;
		double start = 0;
		double end = 0;
		int a = 0;
		char Dev1[] = SHEET_GALV_R;//Right-sheet
		char Dev2[] = SHEET_GALV_L;//Left-sheet
		for (int i = 0; Dev1[i] != '\0'; i++, a++)
			Dev[a] = Dev1[i];
		//a++;
		Dev[a] = ',';
		a++;
		for (int i = 0; Dev2[i] != '\0'; i++, a++)
			Dev[a] = Dev2[i];
		Dev[a] = ',';
		a++;
		Dev[a] = '\0';
		double delta = ((end - start)) / exp;
		//cout << "Sheet SampleNum: " << SampleNum << endl;
		if (Axial_mode_check->Checked == true) {
			if (illumination_flag == 'L') {
				start = double::Parse(ETL_L_left->Text);
				end = double::Parse(ETL_L_right->Text);

			}
			if (illumination_flag == 'R') {
				start = double::Parse(ETL_R_left->Text);
				end = double::Parse(ETL_R_right->Text);
			}
			delta = ((end - start)) / exp;
		}
		else {
			if (DataChangeFlag == 2) {
				start = double::Parse(ETL_box2->Text);
				end = double::Parse(ETL_box2->Text);
			}
			else {
				start = double::Parse(ETL_box->Text);
				end = double::Parse(ETL_box->Text);
			}
			delta = 0;
		}

		for (int i = 0; i < SampleNum; ++i) {
			if (i < delay)
				data[i] = start;
			else if (i > (Exposuretime - EXTRA_TIME * 20))
				data[i] = end;
			else {
				data[i] = (start + delta * (i - delay));
			}
		}

		for (int i = 0; i < SampleNum; ++i) {
			if (i < delay)
				data[i + SampleNum] = 0;
			else
				data[i + SampleNum] = 5;
		}

	}
	private: void DAQ_ETL_TASK_prep2(double *data, const int &SampleNum) {
		double Ampli_sheet = double::Parse(Amplitude_sheet->Text);
		double sheetofset_L = double::Parse(sheet_offset_L->Text);
		double sheetofset_R = double::Parse(sheet_offset_R->Text);
		double adj = double::Parse(SheetSPD->Text);
		//Dev = "cDAQ1Mod1/ao0,cDAQ1Mod1/ao1\0";
		double Exposuretime = double::Parse(exposure_time_box->Text) * 20;
		double delay = double::Parse(delay_time_box->Text) * 20;
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		double extra = double::Parse(Extra_time_box->Text) * 20;
		double exp = Exposuretime - extra - delay;
		double start = 0;
		double end = 0;
		int a = 0;
		double delta = (end - start) / exp;
		//cout << "Sheet SampleNum: " << SampleNum << endl;
		if (Axial_mode_check->Checked == true) {
			if (illumination_flag == 'L') {
				start = double::Parse(ETL_L_left->Text);
				end = double::Parse(ETL_L_right->Text);

			}
			if (illumination_flag == 'R') {
				start = double::Parse(ETL_R_left->Text);
				end = double::Parse(ETL_R_right->Text);
			}
			delta = ((end - start)) / exp;
		}
		else {
			if (DataChangeFlag == 2) {
				start = double::Parse(ETL_box2->Text);
				end = double::Parse(ETL_box2->Text);
			}
			else {
				start = double::Parse(ETL_box->Text);
				end = double::Parse(ETL_box->Text);
			}
			delta = 0;
		}

		for (int i = 0; i < SampleNum; ++i) {
			if (i < delay)
				data[i] = start;
			else if (i > (Exposuretime - extra))
				data[i] = end;
			else {
				data[i] = (start + delta * (i - delay - extra));
			}
		}
		for (int i = 0; i < SampleNum; ++i) {
			if (i < delay)
				data[i + SampleNum] = 0;
			else if (i < delay + 20)
				data[i + SampleNum] = 5;
			else
				data[i + SampleNum] = 0;
		}
		double deltasheet = Ampli_sheet / exp * (WINDOWSIZE + 1) * 2 * adj;
		int direction = -1;
		double sheet = Ampli_sheet / 2 + sheetofset_L;
		for (int i = 0; i < SampleNum; ++i) {
			if (illumination_flag == 'R') {
				sheet = sheet + direction * deltasheet;
				if (sheet >= Ampli_sheet / 2) {
					sheet = Ampli_sheet - sheet;
					direction = -1;
				}
				else if (sheet < (-1 * Ampli_sheet / 2)) {
					sheet = -1 * Ampli_sheet - sheet;
					direction = 1;
				}
				//if (i> SampleNum - 20 * 3) {
				//	sheet = Ampli_sheet / 2 + sheetofset_L;
				//}
				data[i + SampleNum * 2] = sheet + sheetofset_L;
			}
			else {
				data[i + SampleNum * 2] = -9;
				//cout << sheet << endl;
			}
		}

		deltasheet = Ampli_sheet / (exp)*(WINDOWSIZE + 1) * 2 * adj;
		direction = -1;
		sheet = Ampli_sheet / 2 + sheetofset_R;
		for (int i = 0; i < SampleNum; ++i) {
			if (illumination_flag == 'L') {
				sheet = sheet + direction * deltasheet;
				if (sheet > Ampli_sheet / 2) {
					sheet = Ampli_sheet - sheet;
					direction = -1;
				}
				else if (sheet < (-1 * Ampli_sheet / 2)) {
					sheet = -1 * Ampli_sheet - sheet;
					direction = 1;
				}
				//if (i> SampleNum-20*3) {
				//	sheet = Ampli_sheet / 2 + sheetofset_R;
				//}
				data[i + SampleNum * 3] = sheet + sheetofset_R;

			}
			//}
			else {
				data[i + SampleNum * 3] = -9;
				//cout << sheet << endl;
			}
		}

		try {
			String^ filename = ".\\DAQ_sheet.txt";
			StreamWriter^ sw = gcnew StreamWriter(filename);
			for (int i = 0; i < SampleNum * 4; ++i)
				sw->WriteLine(i + "," + data[i]);
			sw->Close();
		}
		catch (exception &e) {

		}
	}
	private:void Set_ETL_param() {
		if (Laser_Info.L488.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser488[0];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser488[1];
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser488[2];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser488[3];
			}

		}
		if (Laser_Info.L592.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser592[0];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser592[1];
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser592[2];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser592[3];
			}
		}
		if (Laser_Info.L647.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser647[0];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser647[1];
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser647[2];
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser647[3];
			}
		}
		//ETL_box3->Text = MOVIE_stats.Axial_param.ETL_Start.ToString();
		//ETL_box4->Text = MOVIE_stats.Axial_param.ETL_End.ToString();
	}
	private:void Set_ETL_param_AF(double v) {
		if (Laser_Info.L488.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param_v.ETL_Start = Axial_ETL_param.Laser488[0];
				MOVIE_stats.Axial_param_v.ETL_End = Axial_ETL_param.Laser488[1];
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param_v.ETL_Start = Axial_ETL_param.Laser488[2];
				MOVIE_stats.Axial_param_v.ETL_End = Axial_ETL_param.Laser488[3];
			}

		}
		if (Laser_Info.L592.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param_v.ETL_Start = Axial_ETL_param.Laser592[0] + v;
				MOVIE_stats.Axial_param_v.ETL_End = Axial_ETL_param.Laser592[1] + v;
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param.ETL_Start = Axial_ETL_param.Laser592[2] + v;
				MOVIE_stats.Axial_param.ETL_End = Axial_ETL_param.Laser592[3] + v;
			}
		}
		if (Laser_Info.L647.stat == 1) {
			if (illumination_flag == 'L') {
				MOVIE_stats.Axial_param_v.ETL_Start = Axial_ETL_param.Laser647[0] + v;
				MOVIE_stats.Axial_param_v.ETL_End = Axial_ETL_param.Laser647[1] + v;
			}
			else if (illumination_flag == 'R') {
				MOVIE_stats.Axial_param_v.ETL_Start = Axial_ETL_param.Laser647[2] + v;
				MOVIE_stats.Axial_param_v.ETL_End = Axial_ETL_param.Laser647[3] + v;
			}
		}
		//ETL_box3->Text = MOVIE_stats.Axial_param.ETL_Start.ToString();
		//ETL_box4->Text = MOVIE_stats.Axial_param.ETL_End.ToString();
	}

	private:void DAQ_sheet_LIVE_func()
	{
		LIVE_Axial_DAQ2();//Sheet control by DAQ

	}
	private:void DAQ_sheet_func() {

		if (Axial_mode_check->Checked == true) {
			Axial_MOVIE_VB_wMOVIEfocus();
		}
		else {
			Axial_MOVIE_woMOVIEfocus();
			//MOVIE_stats.Abort = 1;
		}

	}
	private: void DAQ_sheet_TASK_Axial_AF_func()
	{
		Thread_Stats.DAQ_VB = 1;
		Axial_MOVIE_woMOVIEfocus_AF();
		//Axial_MOVIE_woMOVIEfocus();
		Thread_Stats.DAQ_VB = 0;
	}

	private: void DAQ_sheet_TASK_Axial_skip_func()
	{
		Axial_MOVIE_woMOVIEfocus();
	}


	private: void Axial_MOVIE_woMOVIEfocus()//For skip
	{
		//cout << "Axial_MOVIE_woMOVIEfocus" << endl;
		chrono::system_clock::time_point start_L, end_L;
		threads.Daq = true;
		int retrycount = 0;
		int err = 0;
	RetryDAQ:
		update_z_galv();
		err = 1;
		double z_val[2] = {};
		z_val[1] = z_val[0];
		Sleep(200);

		double Exposuretime = double::Parse(exposure_time_box->Text);
		double delay = double::Parse(delay_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		double SampleRate = 20000;//0.1ms
		int SampleNum_sheet = Exposuretime * SampleRate / 1000;

		double *data_sheet = new double[SampleNum_sheet * 4];
		char Dev_sheet[100] = DAQ1_ALL;
		Set_ETL_param();
		DAQ_ETL_TASK_prep2(data_sheet, SampleNum_sheet);//ETL control by DAQ
														   //		cout << Dev_sheet << endl;
		int32       error = 0;
		DAQ1_flag = 1;
		char        errBuff[2304] = { '\0' };

		Set_ETL_param();

		MOVIE_stats.DAQ = 0;
		DAQ1_flag = 1;

		//Start Sheet
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev_sheet, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", SampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, SampleNum_sheet));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, SampleNum_sheet, 0, 10.0, DAQmx_Val_GroupByChannel, data_sheet, NULL, NULL));

		deltatime_G = 0;
		double imagingtime = 0;
		double deltatime = 0;
		int imagenum = 0;
		MOVIE_stats.DAQ = 2;
		MOVIE_stats.External = 2;
		double d;
		double ImagingTime = Exposuretime - delay - EXTRA_TIME;
		Running_LOG(" Axial_MOVIE_VB_R: MOVIE_stats.External = 2");
		//SheetGen();
		while (MOVIE_stats.ExposureFlag != 1) {
			Sleep(10);
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.retry == true) {
				break;
			}
		}

		MOVIE_stats.DAQ = 2;
		MOVIE_focus_feedback_flag = 0;
		MOVIE_stats.TTLs = 0;
		startG = chrono::system_clock::now();
		DAQmxErrChk(DAQmxStartTask(taskHandle));
		for (int n = 0; n < MOVIE_stats.TargetImageNum; ++n) {
			while (Picnumber_c1 <= n) {
				if (MOVIE_stats.cancel_flag == 1)
					break;
				Sleep(1);
			}
			if (MOVIE_stats.retry == true) {
				break;
			}
			//cout << "Axial_MOVIE_woMOVIEfocus: " << n << endl;
		}
		Sleep(200);
		MOVIE_stats.ExposureFlag = 0;
		threads.Daq = false;
		DAQ_sheet_endpos();
		MOVIE_stats.DAQ = 0;
		delete[] data_sheet;
		err = 0;
	Error:
		if (err != 0) {
			if (DAQmxFailed(error)) {
				DAQ_ERR(17);
			}
			if (retrycount < 5) {
				retrycount++;
				DAQ_reset();
				Sleep(100);
				if (MOVIE_stats.cancel_flag != 1)
					goto RetryDAQ;
			}
		}
		DAQ_clear();
		threads.Daq = false;
	}


	private:void Focus_List_export(String^ Dir, int xpos, int ypos, int file_num) {
		Dir_check(Dir);
		errno_t error;
		Running_LOG("Focus_List_export Start");
		string FP;
		String^ X = (xpos + 100000).ToString("D8");
		String^ Y = (ypos + 100000).ToString("D8");
		String^ Fname = Dir + "\\SamplePos" + Y + "_" + X + ".csv";
		MarshalString(Fname, FP);
		FILE *fpw;
		error = fopen_s(&fpw, FP.c_str(), "w");
		if (error == 0) {
			for (int i = 0; i < file_num; ++i) {
				fprintf(fpw, "%lf,%lf\n", MOVIE_stats.Stage_pos_list[i], MOVIE_stats.MOVIE_focus_list[i]);
			}
			fclose(fpw);
			Running_LOG("Focus_List_export End");
		}

	}
	private:void DCTs_List_export(String^ Dir, int xpos, int ypos, int file_num) {
		Running_LOG("deltaDCT_List_export Start");
		errno_t error;
		string FP;
		String^ X = (xpos + 100000).ToString("D8");
		String^ Y = (ypos + 100000).ToString("D8");
		String^ Fname = Dir + "\\DeltaDCT" + Y + "_" + X + ".csv";
		MarshalString(Fname, FP);
		FILE *fpw;
		error = fopen_s(&fpw, FP.c_str(), "w");
		if (error == 0) {
			for (int i = 0; i < file_num; ++i) {
				fprintf(fpw, "%lf,%lf\n", MOVIE_stats.Stage_pos_list[i], deltaDCTVal[i]);
			}
			fclose(fpw);
		}
		Running_LOG("DCT_List_export End");

	}
	private: void ExportCUBICInformatics(String^ Dir) {
		Running_LOG("DataExport_func ExportCUBICInformatics Start");
		string FP;
		int spdOK = 0;
		MarshalString(Dir, FP);
		errno_t error;
		Running_LOG("DataExport_ Ready ExportCUBICInformatics");
		//int maximagenum= int::Parse()
		//cout << "DataExport_ Ready ExportCUBICInformatics" << endl;
		MOVIE_stats.data = 2;
		while (MOVIE_stats.ExposureFlag != 1) {
			Sleep(10);
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.Skiptile == true)
				break;
		}
		MOVIE_stats.data = 1;
		Running_LOG(" DataExport_ Start");
		Running_LOG(" TargetImageNum: " + MOVIE_stats.TargetImageNum);

		if (Camera_1_use->Checked == true) {
			for (int n = 0; n < MOVIE_stats.TargetImageNum; ++n) {
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				if (MOVIE_stats.Skiptile == true)
					break;
				while (Picnumber_c1 <= n) {
					Sleep(1);
					if (MOVIE_stats.Skiptile == true)
						break;
					if (MOVIE_stats.cancel_flag == 1) {
						break;
					}
				}
				FILE *fpw;
				char z[9] = { '\0' };
				sprintf(z, "%08d", n + MOVIE_stats.ImagingStartNum);
				//sprintf(z, "%08d", MOVIE_stats.MaxImageNum-(n + MOVIE_stats.ImagingStartNum)-1);
				error = fopen_s(&fpw, (FP + "\\" + z + ".bin").c_str(), "wb");
				if (error != 0) {
					//cout << "Cannnot open file: " << FP + "\\" + z + ".bin" << endl;
					cout << "Error: " << error << endl;
				}
				else {
					fwrite(Orca.bufAddr[n%Orca.number_of_buffer], sizeof(unsigned short), Orca.height * Orca.width, fpw);
					cout << "\rImage Export: " << n << "/" << Picnumber_c1 << " / total: " << MOVIE_stats.TargetImageNum;
					// cout << "\rImage Export: " << Picnumber_c1_w << " / " << Picnumber_c1;
					fclose(fpw);
				}
				if ((Picnumber_c1 - n) >= Orca.number_of_buffer) {
					spdOK = 1;
					MOVIE_stats.dataspeed += 1;
					cout << "Data Export speed error !!!" << MOVIE_stats.dataspeed << endl;
					if (MOVIE_stats.dataspeed > 4) {
						Abort_ALL();
						Laser_ALL_OFF();
						break;
					} //getchar();
				}
				if (acqdata_flag == false)
					break;
			}

		}
		cout << endl;
		Console::WriteLine("DataExport ExportCUBICInformatics: End");
		Running_LOG("DataExport ExportCUBICInformatics: End");
		if (spdOK == 0)
			MOVIE_stats.dataspeed = 0;
	}
	private: void ExportData_SinglePlane() {
		MOVIE_stats.ExposureFlag = 0;
		Running_LOG(" DataExport_func  Start");
		int file_num = int::Parse(imagingtile->Text);
		char buf_s[7] = { '\0' };
		char buf_s2[15] = { '\0' };
		String^ Dir = Export_Box->Text;
		String^ Stem = Stem_box1->Text;
		String^ Path;
		string pathw;
		pathw = DirList[MOVIE_stats.tilenum].dir_FW;
		Path = gcnew String(pathw.c_str());
		Dir_check(Path);
		FILE* fpw;
		char z[9] = { '\0' };
		sprintf(z, "%08d", 0);
		errno_t error = fopen_s(&fpw, (pathw + "\\" + z + ".bin").c_str(), "wb");
		if (error != 0) {
			cout << "Cannnot open file: " << pathw + "\\" + z + ".bin" << endl;
			cout << "Error: " << error << endl;
		}
		else {
			fwrite(BestImage, sizeof(unsigned short), BufferSize, fpw);
			cout << "\rExport: " << pathw + "\\" + z + ".bin" << endl;
			fclose(fpw);
		}

	}
	private:  void DataExport_func() {
		threads.DataExport = true;
		MOVIE_stats.ExposureFlag = 0;
		Running_LOG(" DataExport_func  Start");
		int file_num = int::Parse(imagingtile->Text);
		char buf_s[7] = { '\0' };
		char buf_s2[15] = { '\0' };
		String^ Dir = Export_Box->Text;
		String^ Stem = Stem_box1->Text;
		String^ Path;
		string pathw;

		if (MOVIE_stats.FR == 0)
			pathw = DirList[MOVIE_stats.tilenum].dir_FW;
		else
			pathw = DirList[MOVIE_stats.tilenum].dir_RV;
		Path = gcnew String(pathw.c_str());

		Dir_check(Path);
		ExportCUBICInformatics(Path);
		cout << endl;

		//Focus_List_export(Dir + "\\Focus_pos\\" + Stem, DirList[MOVIE_stats.tilenum].FW_xpos, DirList[MOVIE_stats.tilenum].FW_ypos, file_num);

		Running_LOG(" DataExport_ End");
		MOVIE_stats.data = 0;
		Thread_Stats.DataExport = 0;
		threads.DataExport = false;
		// AT_Flush(Camera1.Hndl);
	}


	private: void DataExport_blank() {
		String^ Path;
		string FP;
		if (MOVIE_stats.FR == 0)
			FP = DirList[MOVIE_stats.tilenum].dir_FW;
		else {
			FP = DirList[MOVIE_stats.tilenum].dir_RV;
		}
		//Running_LOG(" BlankDataExport");
		FILE *fpw;
		char z[9] = { '\0' };
		sprintf(z, "%08d", 0);
		BIT* blank = new BIT[BufferSize2];
		errno_t error;
		memset(blank, 0, BufferSize2);
		error = fopen_s(&fpw, (FP + "\\" + z + ".bin").c_str(), "wb");
		if (error == 0) {
			//cout << "Blank Export:" << FP + "\\" + z + ".bin" << endl;
			fwrite(blank, sizeof(BIT), BufferSize2, fpw);
			fclose(fpw);
		}

	}
	private: void FocusRSet(String^ R) {
		MFocus_right->Text = R;
		DCTScoreBox->Text = MOVIEFocusVal.FocusVal.ToString();
		DCTFocusValBox->Text = FocusVal.ToString();
	}
	private: void FocusLSet(String^ L) {
		MFocus_left->Text = L;
	}

	private:inline bool  BlankCheck(cv::Mat &Image1, cv::Mat &Image2) {

		double BK = double::Parse(BK_box->Text);
		cv::Mat Mask = cv::Mat::zeros(2304, 2304, CV_8U);
		cv::threshold(Image1, Mask, BK, 1, cv::THRESH_BINARY);
		cv::Scalar sum1 = cv::sum(Mask);
		cv::threshold(Image2, Mask, BK, 1, cv::THRESH_BINARY);
		cv::Scalar sum2 = cv::sum(Mask);

		if (sum1[0] > 50 && sum2[0] > 50) {
			return true;
		}
		//cout << "  BlankCheck: false\n";
		return false;

	}

	private:inline bool BrankBlockCkeck(cv::Mat &Image, cv::Mat &dst, double &BK) {
		//cout << Image << endl;
		cv::threshold(Image, dst, BK, 1, cv::THRESH_BINARY);
		cv::Scalar sum = cv::sum(dst);
		//cout << sum << endl;
		if (sum[0] > 30) {
			return true;
		}
		return false;
	}
	private:inline int BrankBlockCkeck_ROI(cv::Mat &Image, cv::Mat &dst, double &BK) {
		//cout << Image << endl;
		cv::threshold(Image, dst, BK, 1, cv::THRESH_BINARY);
		cv::Scalar sum = cv::sum(dst);
		return sum[0];
	}
	private:inline int Sobel_Calc_full(cv::Mat &Image1, cv::Mat &Image2, double BK) {
		if (BlankCheck(Image1, Image2) == true) {
			cv::Mat Input_F;
			cv::Mat Mask = cv::Mat(Camera2_sensorWidth, Camera2_sensorHight, CV_64FC1);
			cv::Mat sobel_x;
			cv::Mat sobel_y;
			cv::Mat sobel;
			cv::Mat sobel_th = cv::Mat(Camera2_sensorWidth, Camera2_sensorHight, CV_64FC1);

			Image1.convertTo(Input_F, CV_64FC1);
			cv::threshold(Image1, Mask, BK, 1, cv::THRESH_BINARY);
			cv::Sobel(Input_F, sobel_x, CV_64FC1, 1, 0);
			//cv::Sobel(Input_F, sobel_y, CV_64FC1, 0, 1);
			//sobel = (abs(sobel_x) + abs(sobel_y)) / 2;
			//cv::threshold(sobel, sobel, BK, 65535, cv::THRESH_TOZERO);
			//cv::Scalar Score1 = cv::sum(sobel_th);
			cv::Scalar Score1 = cv::sum(abs(sobel_x));

			Image2.convertTo(Input_F, CV_64FC1);
			cv::Sobel(Input_F, sobel_x, CV_64FC1, 1, 0);
			//cv::Sobel(Input_F, sobel_y, CV_64FC1, 0, 1);
			//sobel = (abs(sobel_x) + abs(sobel_y)) / 2;
			//cv::threshold(sobel, sobel, BK, 65535, cv::THRESH_TOZERO);
			//cv::Scalar Score2 = cv::sum(sobel_th);
			cv::Scalar Score2 = cv::sum(abs(sobel_x));

			if (Score2[0] > Score1[0]) {
				return 1; //odd = good;
			}
			if (Score2[0] < Score1[0]) {
				return -1;//even = good;
			}
			return 0;

		}
	}
	private:inline int Sobel_Calc(cv::Mat &Image1, cv::Mat &Image2, double BK) {

		if (BlankCheck(Image1, Image2) == true) {
			cv::Mat Input_F;
			cv::Mat sobel_x;
			cv::Mat sobel_y;
			cv::Mat sobel;
			cv::Mat Mask = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_8U);
			//	vector<BlockInfo> L2list(256);//total 256 blocks
			cv::Mat nDCTS_Value1 = cv::Mat();
			cv::Mat nDCTS_Value2 = cv::Mat();
			//	cv::Scalar nDCTS_Value1;
			//	cv::Scalar nDCTS_Value2;
			double DCTScore1 = 0;
			double DCTScore2 = 0;
			int m = 0;
			int n = 0;
			int w = 16;//16 block x 128 pixel=2304;
			int h = 16;
			for (m = 0; m < w; ++m) {
				for (n = 0; n < h; ++n) {
					cv::Rect roi(m * 128, n * 128, 128, 128);
					cv::Mat R1 = Image1(roi);
					cv::Mat R2 = Image2(roi);
					L2list[m*h + n].L2 = cv::norm(R1, R2);
					L2list[m*h + n].m = m;
					L2list[m*h + n].n = n;
					L2list[m*h + n].blankcheck = BrankBlockCkeck(Image1(roi), Mask, BK);
					if (L2list[m*h + n].blankcheck == false) {
						L2list[m*h + n].L2 = 0;
					}
				}
			}
			//sort(L2list.begin(), L2list.end(), asc);
			sort(L2list.begin(), L2list.end(), asc);
			cv::Scalar Score1 = 0;
			cv::Scalar Score2 = 0;
			for (int i = 0; i < 24; ++i) {
				if (L2list[i].blankcheck == false)
					continue;
				cv::Rect roi(L2list[i].m * 128, L2list[i].n * 128, 128, 128);
				Image1(roi).convertTo(Input_F, CV_64FC1);
				cv::Sobel(Input_F, sobel_x, CV_64FC1, 1, 0);
				cv::Sobel(Input_F, sobel_y, CV_64FC1, 0, 1);
				sobel = (abs(sobel_x) + abs(sobel_y)) / 2;
				cv::threshold(sobel, sobel, 100, 65535, cv::THRESH_TOZERO);
				Score1 += cv::sum(sobel);

				Image2(roi).convertTo(Input_F, CV_64FC1);
				//Image2.convertTo(Input_F, CV_64FC1);
				cv::Sobel(Input_F, sobel_x, CV_64FC1, 1, 0);
				cv::Sobel(Input_F, sobel_y, CV_64FC1, 0, 1);
				sobel = (abs(sobel_x) + abs(sobel_y)) / 2;
				cv::threshold(sobel, sobel, 100, 65535, cv::THRESH_TOZERO);
				//cv::multiply(sobel, Mask, sobel_th);
				Score2 += cv::sum(sobel);
			}
			if (Score2[0] > Score1[0]) {
				return 1; //odd = good;
			}
			if (Score2[0] < Score1[0]) {
				return -1;//even = good;
			}
			return 0;

		}
	}
	private:inline double DCTScore_Calc_block(cv::Mat &Image, double BK) {
		double DCT_List = 0;
		//vector<BlockInfo> L2list(256);//total 256 blocks
		char Dst[128 * 128];
		Make_Dst(Dst);
		cv::Mat Dct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Input_F;
		cv::Mat MatZERO = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat nDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat nDct_2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat lDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Ent = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Ent2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Mask = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_8U);
		cv::Mat Mask_dst = cv::Mat(MASKSIZE, MASKSIZE, CV_8U, Dst);
		cv::Mat nDCTS_Value1 = cv::Mat();
		cv::Mat nDCTS_Value2 = cv::Mat();

		double DCTScore = 0;
		double DCTcomp1 = 0;
		double DCTcomp2 = 0;
		int m = 0;
		int n = 0;
		int w = 16;//16 block x 128 pixel=2304;
		int h = 16;
		for (m = 0; m < w; ++m) {
			for (n = 0; n < h; ++n) {
				cv::Rect roi(m * 128, n * 128, 128, 128);
				cv::Mat Im = Image(roi);
				L2list[m*h + n].L2 = cv::mean(Im)[0];
				L2list[m*h + n].m = m;
				L2list[m*h + n].n = n;
				L2list[m*h + n].blankcheck = BrankBlockCkeck(Image(roi), Mask, BK);
				if (L2list[m*h + n].blankcheck == false) {
					L2list[m*h + n].L2 = 0;
				}
			}
		}
		sort(L2list.begin(), L2list.end(), asc);

		for (int i = 0; i < 24; ++i) {
			if (L2list[i].blankcheck == false) {
				DCT_List += 0;
				continue;
			}
			cv::Rect roi(L2list[i].m * 128, L2list[i].n * 128, 128, 128);
			Image(roi).convertTo(Input_F, CV_64FC1);
			dct(Input_F, Dct, CV_DXT_FORWARD);
			nDct = abs((Dct / norm(Dct)));
			threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
			add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
			log(nDct_2, lDct);
			absdiff(lDct, MatZERO, lDct);
			multiply(nDct, lDct, Ent);
			Ent.copyTo(Ent2, Mask_dst);
			reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
			reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
			DCT_List += nDCTS_Value2.at<double>(0);
		}
		return DCT_List;
	}
	private:inline void DCTBlockCalc_num_counter() {
		// Win32 の interlocked 関数による同期
		InterlockedIncrement(&DCTBlockCalc_num);

		// Win32 の critical section による同期
		//	EnterCriticalSection(&cs);
		//DCTBlockCalc_num++;
		//	LeaveCriticalSection(&cs);

		// Win32 の mutex による同期
		//	WaitForSingleObject(mtx_MOVIEFocus, INFINITE);
		//DCTBlockCalc_num++;
		//		ReleaseMutex(mtx_MOVIEFocus);
	}
	private:void DCTBlockCalc_func(Object^ V) {

		int n = (int)V;
		cout << "DCTBlockCalc_func start : " << n << endl;
		int threadflag = 0;
		char Dst[128 * 128];
		Make_Dst(Dst);
		cv::Mat Dct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Input_F;
		cv::Mat MatZERO = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat nDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat nDct_2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat lDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Ent = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Ent2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
		cv::Mat Mask = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_8U);
		cv::Mat Mask_dst = cv::Mat(MASKSIZE, MASKSIZE, CV_8U, Dst);
		cv::Mat nDCTS_Value1 = cv::Mat();
		cv::Mat nDCTS_Value2 = cv::Mat();


		double DCTScore = 0;
		double DCTcomp1 = 0;
		double DCTcomp2 = 0;
		while (1) {
			while (DCTBlockCalc_flag != 1) {
				Sleep(1);
				if (DCTBlockCalc_flag == 999)
					break;
			}
			if (DCTBlockCalc_flag == 999)
				break;
			int i = n;
			for (; i < 24; i += 8) {
				if (L2list[i].blankcheck == false) {
					//cout << "\rBlankBlock detected";

				}
				else {
					cv::Rect roi(L2list[i].m * 128, L2list[i].n * 128, 128, 128);
					MOVIEFocusImages.Image1(roi).convertTo(Input_F, CV_64FC1);
					dct(Input_F, Dct, CV_DXT_FORWARD);
					nDct = abs((Dct / norm(Dct)));
					threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
					add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
					log(nDct_2, lDct);
					absdiff(lDct, MatZERO, lDct);
					multiply(nDct, lDct, Ent);
					Ent.copyTo(Ent2, Mask_dst);
					reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
					reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
					DCTcomp1 = nDCTS_Value2.at<double>(0);

					//Calc Image2
					MOVIEFocusImages.Image2(roi).convertTo(Input_F, CV_64FC1);
					dct(Input_F, Dct, CV_DXT_FORWARD);
					nDct = abs((Dct / norm(Dct)));
					threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
					add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
					log(nDct_2, lDct);
					absdiff(lDct, MatZERO, lDct);
					multiply(nDct, lDct, Ent);
					Ent.copyTo(Ent2, Mask_dst);
					reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
					reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
					//nDCTS_Value2 = cv::sum(Ent);
					DCTcomp2 = nDCTS_Value2.at<double>(0);
					//DCTScore2 += nDCTS_Value2.at<double>(0);
					//cout << DCTcomp1 << " : " << DCTcomp2 << "\n";
					if (DCTcomp2 > DCTcomp1) {
						DCTBlockCalc_scores[i] = 1;
					}
					else {
						DCTBlockCalc_scores[i] = -1;
					}
				}
			}
			DCTBlockCalc_scores[i] = 0;
			DCTBlockCalc_num_counter();
			while (DCTBlockCalc_flag != 0) {
				Sleep(1);
				if (DCTBlockCalc_flag == 999)
					break;
			}
			if (DCTBlockCalc_flag == 999)
				break;
		}

		cout << " DCTBlockCalc_func end" << endl;
	}
	private:inline double DCTScore_Roi(cv::Mat &Image, int row, int col, int im) {
		int Masksize = 128;
		char* Dst = new char[Masksize * Masksize];
		Make_Dst_size(Dst, Masksize);
		cv::Mat Input_F;
		cv::Mat Output;
		cv::Mat Dct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat MatZERO = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat nDct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat nDct_2 = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat lDct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat Ent = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat Ent2 = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
		cv::Mat Mask = cv::Mat::zeros(Masksize, Masksize, CV_8U);
		cv::Mat Mask_dst = cv::Mat(Masksize, Masksize, CV_8U, Dst);
		cv::Mat nDCTS_Value1 = cv::Mat();
		cv::Mat nDCTS_Value2 = cv::Mat();
		//cout << Mask_dst << endl;
		double result = 0;
		int m = 0;
		int n = 0;
		int w = 16;//16 block x 128 pixel=2304;
		int h = 1;//128 pixel
		for (n = 0; n < h; ++n) {
			for (m = 0; m < w; ++m) {
				cout << n << ":" << m << endl;
				cv::Rect roi(m * 128, n * 128, 128, 128);
				cv::Mat R1 = Image(roi);
				Image(roi).convertTo(Input_F, CV_64FC1);
				dct(Input_F, Dct, CV_DXT_FORWARD);
				nDct = abs((Dct / norm(Dct)));
				threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
				add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct, lDct, Ent);
				Ent.copyTo(Ent2, Mask_dst);
				reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
				reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
				result += nDCTS_Value2.at<double>(0);
				Ent.convertTo(Output, CV_16UC1);
				string fname = "AxialAF_data\\DCT\\" + to_string(im) + "_" + to_string(m) + "_" + to_string(n) + ".tif";
				cv::imwrite(fname, Output);
			}
		}
		delete[] Dst;
		return result;
	}
	private:inline int DCT_Calc(cv::Mat &Image1, cv::Mat &Image2, double BK, double &deltaDCT) {
		//cout << "DCT_Calc\n";
		deltaDCT = 0;
		if (BlankCheck(Image1, Image2) == true) {
			//BK = BK / 2;
			vector<BlockInfo> L2list(256);//total 256 blocks
			char Dst[128 * 128];
			Make_Dst(Dst);
			cv::Mat Dct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat Input_F;
			cv::Mat MatZERO = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat nDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat nDct_2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat lDct = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat Ent = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat Ent2 = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_64FC1);
			cv::Mat Mask = cv::Mat::zeros(MASKSIZE, MASKSIZE, CV_8U);
			cv::Mat Mask_dst = cv::Mat(MASKSIZE, MASKSIZE, CV_8U, Dst);
			cv::Mat nDCTS_Value1 = cv::Mat();
			cv::Mat nDCTS_Value2 = cv::Mat();
			cv::Mat BIN;

			int calcblock = int::Parse(MOVIEfocus_calc_block->Text);

			double DCTScore = 0;
			double DCTcomp1 = 0;
			double DCTcomp2 = 0;
			int m = 0;
			int n = 0;
			int w = 16;//16 block x 128 pixel=2304;
			int h = 16;
			for (n = 0; n < h; ++n) {
				for (m = 0; m < w; ++m) {
					cv::Rect roi(m * 128, n * 128, 128, 128);
					cv::Mat R1 = Image1(roi);
					cv::Mat R2 = Image2(roi);
					L2list[m*h + n].L2 = cv::norm(R1, R2);
					L2list[m*h + n].m = m;
					L2list[m*h + n].n = n;
					//cv::minMaxLoc(R1, &L2list[m*h + n].min1, &L2list[m*h + n].max1,0,0);
					//cv::minMaxLoc(R2, &L2list[m*h + n].min2, &L2list[m*h + n].max2, 0, 0);
					L2list[m*h + n].blankcheck = BrankBlockCkeck(R1, BIN, BK);
					if (L2list[m*h + n].blankcheck == false) {
						L2list[m*h + n].L2 = 0;
					}
				}
			}
			sort(L2list.begin(), L2list.end(), asc);
			//sort(L2list.begin(), L2list.end(), des);
			for (int i = 0; i < calcblock; ++i) {
				//for (int i = 0; i < 24; ++i) {
				//cout << " \n" <<L2list[i].L2<<": " << L2list[i].m << ": " << L2list[i].n << ": " << nn << "," << i;
				if (L2list[i].blankcheck == false) {
					//cout << "\rBlankBlock detected";
					continue;
				}
				cv::Rect roi(L2list[i].m * 128, L2list[i].n * 128, 128, 128);
				Image1(roi).convertTo(Input_F, CV_64FC1);
				//cv::normalize(Input_F, Input_F, 0, 65535, cv::NORM_MINMAX);
				dct(Input_F, Dct, CV_DXT_FORWARD);
				nDct = abs((Dct / norm(Dct)));
				threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
				add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct, lDct, Ent);
				Ent.copyTo(Ent2, Mask_dst);
				reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
				reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
				DCTcomp1 = nDCTS_Value2.at<double>(0);

				/*
				Image1(roi).convertTo(Input_F, CV_64F);
				cv::dct(Input_F, Dct);//, CV_DXT_FORWARD
				Dct.convertTo(nDct_2, CV_64F, 1 / cv::norm(Dct));
				absdiff(nDct_2, MatZERO, nDct_2);
				threshold(nDct_2, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 0は 1 に、それ以外はそのまま。
				add(nDct_2, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct_2, lDct, Ent);
				DCTcomp1 = sum(mean(Ent))[0];
				*/
				//Calc Image2
				//Image2(roi).convertTo(Input_F, CV_64FC1);
				//cv::normalize(Input_F, Input_F, 0, 65535, cv::NORM_MINMAX);

				Image2(roi).convertTo(Input_F, CV_64FC1);
				//cv::normalize(Input_F, Input_F, 0, 65535, cv::NORM_MINMAX);
				dct(Input_F, Dct, CV_DXT_FORWARD);
				nDct = abs((Dct / norm(Dct)));
				threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
				add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct, lDct, Ent);
				Ent.copyTo(Ent2, Mask_dst);
				reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
				reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
				DCTcomp2 = nDCTS_Value2.at<double>(0);
				/*
				Image2(roi).convertTo(Input_F, CV_64F);
				cv::dct(Input_F, Dct);//, CV_DXT_FORWARD
				Dct.convertTo(nDct_2, CV_64F, 1 / cv::norm(Dct));
				absdiff(nDct_2, MatZERO, nDct_2);
				threshold(nDct_2, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 0は 1 に、それ以外はそのまま。
				add(nDct_2, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct_2, lDct, Ent);
				DCTcomp2 = sum(mean(Ent))[0];

				*/

				if (DCTcomp2 > DCTcomp1) {
					DCTScore += 1;
				}
				else {
					DCTScore -= 1;
				}
				deltaDCT += (DCTcomp2 - DCTcomp1);
			}
			//cout << deltaDCT << "\n";
			if (DCTScore > 0) {
				return +1;
			}
			else if (DCTScore <= 0) {
				return -1;
			}
			if (deltaDCT > 0) {
				return +1;//DCT2(odd = better)
			}
			else if (deltaDCT <= 0) {
				return -1;//DCT2(even = better)
			}
		}
		else {
			//cout << "BlankImage detected\n" << endl;
		}
		return 0;
	}
			/*
			private:inline	double nDCTS(cv::Mat &Image1, int nDCTS_height, int nDCTS_Width)
			{
				cv::Mat Dct;
				cv::Mat Dct_2;
				cv::Mat Input_F;
				cv::Mat MatZERO = cv::Mat::zeros(nDCTS_height, nDCTS_Width, CV_32F);
				cv::Mat lDct;
				cv::Mat Ent;
				cv::Mat Mask;
				Image1.convertTo(Input_F, CV_32F);
				cv::dct(Input_F, Dct);//, CV_DXT_FORWARD
				Dct.convertTo(Dct_2, CV_32F, 1 / cv::norm(Dct));
				absdiff(Dct_2, MatZERO, Dct_2);
				threshold(Dct_2, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 0は 1 に、それ以外はそのまま。
				add(Dct_2, Mask, Dct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(Dct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(Dct_2, lDct, Ent);
				double ib = sum(mean(Ent, global_Mask_dst_128))[0];
				Dct.release();
				Dct_2.release();
				Input_F.release();
				MatZERO.release();
				lDct.release();
				Ent.release();
				Mask.release();

				return ib;
			}*/



	private:inline int DCT_Calc_large(cv::Mat &Image1, cv::Mat &Image2, double BK) {
		if (BlankCheck(Image1, Image2) == true) {
			int Masksize = 256;
			vector<BlockInfo> L2list(64);//total 256 blocks
			char* Dst = new char[Masksize * Masksize];
			Make_Dst_size(Dst, Masksize);
			cv::Mat Dct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat Input_F;
			cv::Mat MatZERO = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat nDct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat nDct_2 = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat lDct = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat Ent = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat Ent2 = cv::Mat::zeros(Masksize, Masksize, CV_64FC1);
			cv::Mat Mask = cv::Mat::zeros(Masksize, Masksize, CV_8U);
			cv::Mat Mask_dst = cv::Mat(Masksize, Masksize, CV_8U, Dst);
			cv::Mat nDCTS_Value1 = cv::Mat();
			cv::Mat nDCTS_Value2 = cv::Mat();


			//	cv::Scalar nDCTS_Value1;
			//	cv::Scalar nDCTS_Value2;
			double DCTScore = 0;
			double DCTcomp1 = 0;
			double DCTcomp2 = 0;
			int m = 0;
			int n = 0;
			int w = 16;//16 block x 128 pixel=2304;
			int h = 16;
			for (m = 0; m < w; ++m) {
				for (n = 0; n < h; ++n) {
					cv::Rect roi(m * 128, n * 128, 128, 128);
					cv::Mat R1 = Image1(roi);
					cv::Mat R2 = Image2(roi);
					L2list[m*h + n].L2 = cv::norm(R1, R2);
					L2list[m*h + n].m = m;
					L2list[m*h + n].n = n;
					L2list[m*h + n].blankcheck = BrankBlockCkeck(Image1(roi), Mask, BK);
					if (L2list[m*h + n].blankcheck == false) {
						L2list[m*h + n].L2 = 0;
					}
				}
			}
			sort(L2list.begin(), L2list.end(), asc);
			//sort(L2list.begin(), L2list.end(), des);

			for (int i = 0; i < 12; ++i) {
				if (L2list[i].blankcheck == false)
					continue;
				cv::Rect roi(L2list[i].m * 128, L2list[i].n * 128, 128, 128);
				Image1(roi).convertTo(Input_F, CV_64FC1);
				dct(Input_F, Dct, CV_DXT_FORWARD);
				nDct = abs((Dct / norm(Dct)));
				threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
				add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct, lDct, Ent);
				Ent.copyTo(Ent2, Mask_dst);
				reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
				reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
				DCTcomp1 = nDCTS_Value2.at<double>(0);

				//Calc Image2
				Image2(roi).convertTo(Input_F, CV_64FC1);
				dct(Input_F, Dct, CV_DXT_FORWARD);
				nDct = abs((Dct / norm(Dct)));
				threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
				add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
				log(nDct_2, lDct);
				absdiff(lDct, MatZERO, lDct);
				multiply(nDct, lDct, Ent);
				Ent.copyTo(Ent2, Mask_dst);
				reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
				reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
				//nDCTS_Value2 = cv::sum(Ent);
				DCTcomp2 = nDCTS_Value2.at<double>(0);
				//DCTScore2 += nDCTS_Value2.at<double>(0);
				if (DCTcomp2 > DCTcomp1) {
					DCTScore += 1;
				}
				else {
					DCTScore -= 1;
				}
			}
			delete[] Dst;
			if (DCTScore > 0) {
				return +1;
			}
			else if (DCTScore < 0) {
				return -1;
			}/*
			 if (DCTScore2 > DCTScore1) {
			 return 1; //odd = good;
			 }
			 if (DCTScore2 < DCTScore1) {
			 return -1;//even = good;
			 }*/
		}
		return 0;
	}
	private:int  BlankCheck_Skip(cv::Mat Image, double BK) {
		cv::Mat Mask = cv::Mat::zeros(2304, 2304, CV_8U);
		cv::threshold(Image, Mask, BK, 1, cv::THRESH_BINARY);
		cv::Scalar sum1 = cv::sum(Mask);
		//cout << "\n\nMOVIE skip val =: " << sum1[0] << "\n\n";
		if (sum1[0] > 700) {
			return sum1[0];
		}
		return 0;

	}
	private:int MOVIE_IniAF_func() {

		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		//cout << "MOVIE_IniAF_func start" << endl;
		while (MOVIE_stats.cancel_flag != 1) {
			if (MOVIE_stats.camera == 2 && MOVIE_stats.DAQ == 2) {
				break;
			}
			Sleep(1);
			end = std::chrono::system_clock::now();
			if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() > 60) {
				MOVIE_stats.cancel_flag = 1;
				break;
			}
		}
		//	cout << "MOVIE_IniAF Exp start" << endl;
		MOVIE_stats.ExposureFlag = 1;
		while (MOVIE_stats.cancel_flag != 1) {
			//cout << "\r Waiting AF ";
			if (MOVIE_stats.ExposureFlag == 0) {
				break;
			}
			Sleep(50);
		}
		if (MOVIE_stats.cancel_flag != 1) {
			Sleep(200);
			//cout << "Initial Focus: " << BestAF.Zpos << endl;
			if (SinglePlane->Checked == false) {
				AF_param BestAF = InitialAF_Check_func();
				if (MOVIE_stats.AF_sample == true) {
					if (illumination_flag == 'R') {
						MOVIE_stats.InitialFocus_R = BestAF.Zpos;
					}
					if (illumination_flag == 'L') {
						MOVIE_stats.InitialFocus_L = BestAF.Zpos;
					}
					return 0;
				}
			}
			else {
				InitialAF_Check_func_SinglePlane();

				return -1;
			}
		}

		return -1;
	}
	private:inline double DCTScore_Calc_Full(cv::Mat &Image, double BK) {
		char *Dst = new char[2304 * 2304];
		Make_Dst_Full(Dst);
		cv::Mat Dct = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat Input_F;
		cv::Mat MatZERO = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat nDct = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat nDct_2 = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat lDct = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat Ent = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat Ent2 = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_64FC1);
		cv::Mat Mask = cv::Mat::zeros(PIXELHIGHT, PIXELWIDTH, CV_8U);
		cv::Mat Mask_dst = cv::Mat(PIXELHIGHT, PIXELWIDTH, CV_8U, Dst);
		cv::Mat nDCTS_Value1 = cv::Mat();
		cv::Mat nDCTS_Value2 = cv::Mat();

		double DCTScore = 0;
		double DCTcomp1 = 0;
		double DCTcomp2 = 0;
		int m = 0;
		int n = 0;
		int w = 16;//16 block x 128 pixel=2304;
		int h = 16;
		cv::Rect roi(0, 0, PIXELHIGHT, PIXELWIDTH);
		Image(roi).convertTo(Input_F, CV_64FC1);
		dct(Input_F, Dct, CV_DXT_FORWARD);
		nDct = abs((Dct / norm(Dct)));
		threshold(nDct, Mask, 1E-30, 1, cv::THRESH_BINARY_INV);  // 閾値を超えるピクセルは 0 に、それ以外は maxVal
		add(nDct, Mask, nDct_2);// DCTの係数が0だったピクセルを、1にします。あとで、logをとると0になるので、こうしています。
		log(nDct_2, lDct);
		absdiff(lDct, MatZERO, lDct);
		multiply(nDct, lDct, Ent);
		Ent.copyTo(Ent2, Mask_dst);
		reduce(Ent2, nDCTS_Value1, 0, CV_REDUCE_SUM, -1);
		reduce(nDCTS_Value1, nDCTS_Value2, 1, CV_REDUCE_SUM, -1);
		delete[] Dst;
		return nDCTS_Value2.at<double>(0);
	}

	private: double* InitialAF_Check_func_Axial(char LR) {

		double *result = new double[2];
		cv::Rect roi1(0, 0, 2304, 128);
		cv::Rect roi2(0, 2303 - 128, 2304, 128);
		vector<double>res1;
		vector<double>res2;
		for (int i = 0; i < MOVIE_stats.TargetImageNum; ++i) {
			cv::Mat Image = cv::Mat(2304, 2304, CV_16UC1, Orca.bufAddr[i]);
			//cv::Mat Image = cv::Mat(Camera1_sensorHight, 2304, CV_16UC1, AlignedBuffers1[i]);
			cv::Mat Im1 = Image(roi1);
			cv::Mat Im2 = Image(roi2);
			//string  f = "AxialAF_data\\Full\\" + to_string(i) + ".tif";
			//string  f1 = "AxialAF_data\\Axial_AF_im1\\Axial_AF_im1_" + to_string(i) + ".tif";
			//string  f2 = "AxialAF_data\\Axial_AF_im2\\Axial_AF_im2_" + to_string(i) + ".tif";
			//cv::imwrite(f, Image);
			//cv::imwrite(f1, Im1);
			//cv::imwrite(f2, Im2);
			res1.push_back(DCTScore_Roi(Im1, 2304, 128, i));
			res2.push_back(DCTScore_Roi(Im2, 2304, 128, i));
			//cout << res1[i] << " :   " << res2[i] << endl;
		}
		double max1 = -9999;
		double max2 = -9999;
		int max_index1 = 0;
		int max_index2 = 0;
		for (int i = 0; i < res1.size(); ++i) {
			cout << i << " \t " << res1[i] << "\t\t" << res2[i] << endl;
			if (max1 < res1[i]) {
				max1 = res1[i];
				max_index1 = i;
			}
			if (max2 < res2[i]) {
				max2 = res2[i];
				max_index2 = i;
			}
		}
		result[0] = max_index1;
		result[1] = max_index2;

		return result;
		//return (std::max_element(AF_Score_List.begin(), AF_Score_List.end()) - AF_Score_List.begin());
	}
	private: AF_param InitialAF_Check_func() {

		//cout << "InitialAF_Check_func start" << endl;
		double BK = double::Parse(BK_box->Text);

		cv::Mat Image1 = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[0]);
		cv::Mat Image2 = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[int::Parse(AFnum->Text) - 1]);

		MOVIE_stats.AF_sample = BlankCheck(Image1, Image2);

		if (MOVIE_stats.AF_sample == true) {
			vector<double> AF_Score_List;
			for (int i = 0; i <int::Parse(AFnum->Text); ++i) {
				cv::Mat Image = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[i]);
				AF_stats[i].Focus_score = DCTScore_Calc_Full(Image, BK);
			}
			double max = 0;
			int max_index = 0;
			for (int i = 0; i < MOVIE_stats.TargetImageNum && i < AF_stats.size(); ++i) {
				cout << AF_stats[i].Zpos << " :   " << AF_stats[i].Focus_score << endl;
				if (max < AF_stats[i].Focus_score) {
					max = AF_stats[i].Focus_score;
					max_index = i;
				}
			}
			return (AF_stats[max_index]);
		}
		return { 0,0 };
		//return (std::max_element(AF_Score_List.begin(), AF_Score_List.end()) - AF_Score_List.begin());
	}
	private: void InitialAF_Check_func_SinglePlane() {

		//cout << "InitialAF_Check_func start" << endl;
		double BK = double::Parse(BK_box->Text);
		cv::Mat Image1 = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[0]);
		cv::Mat Image2 = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[int::Parse(AFnum->Text) - 1]);
		vector<double> AF_Score_List;
		for (int i = 0; i < int::Parse(AFnum->Text); ++i) {
			cv::Mat Image = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[i]);
			AF_stats[i].Focus_score = DCTScore_Calc_Full(Image, BK);
			//AF_stats[i].Focus_score = DCTScore_Calc_block(Image, BK);
		}
		double max = 0;
		int max_index = 0;
		for (int i = 0; i < AF_stats.size(); ++i) {
			cout << AF_stats[i].Zpos << " :   " << AF_stats[i].Focus_score << endl;
			if (max < AF_stats[i].Focus_score) {
				max = AF_stats[i].Focus_score;
				max_index = i;
			}
		}
		cout << "Best image is: " << max_index << endl;
		memcpy_s(BestImage, 2304 * 2304 * sizeof(BIT), Orca.bufAddr[max_index], 2304 * 2304 * sizeof(BIT));
		ExportData_SinglePlane();
	}

	private: void MOVIE_Skip_Check_func() {
		cout << "MOVIE_Skip_Check_func Start " << endl;
		double BK = double::Parse(BK_box->Text);
		double BKarea = double::Parse(BKarea_box->Text);
		double Exp = double::Parse(exposure_time_box->Text);
		cv::Mat Image;
		cv::Rect roi(0, 0, Orca.width, Orca.height);
		MOVIE_stats.MOVIE_skip_thread = 2;
		MOVIE_skip_size = 0;
		int skipval = 0;

		while (Picnumber_c1 < (MOVIE_stats.TargetImageNum - 1)) {
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.retry == true) {
				break;
			}
			Sleep(1);
		}
		while (MOVIE_stats.ExposureFlag != 1) {
			//cout << "MOVIE_stats.MOVIE_skip_thread 2 " << endl;
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.retry == true) {
				break;
			}
			Sleep(1);
		}
		MOVIE_stats.MOVIE_skip_thread = 1;
		//cout << "MOVIE skip_func start" << endl;
		Sleep(Exp * 10);
		//Sleep(100);
		if (MOVIE_stats.retry == false) {
			for (int n = 0; n < MOVIE_stats.TargetImageNum; ++n) {
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				while (Picnumber_c1 <= n) {
					Sleep(1);
				}
				//	cout << "\rMOVIE_Skip_Check_func: Image :" << MOVIE_skip_size % NumberOfBuffers;
				Image = cv::Mat(Orca.width, Orca.height, CV_16UC1, Orca.bufAddr[MOVIE_skip_size%Orca.number_of_buffer]);
				//cout << "BlankCheck_Skip(Image, BK): " << BlankCheck_Skip(Image, BK) << endl;
				MOVIE_stats.MOVIE_skip_list[MOVIE_skip_size] = BlankCheck_Skip(Image, BK);
				//cout << "MOVIE_skip_list[MOVIE_skip_size: " << MOVIE_skip_list[MOVIE_skip_size] << endl;

				FILE *fpw;
				char z[9] = { '\0' };
				sprintf(z, "%04d", n);
				string fp = "skipimg";
				//sprintf(z, "%08d", MOVIE_stats.MaxImageNum-(n + MOVIE_stats.ImagingStartNum)-1);
				errno_t error;
				error = fopen_s(&fpw, (fp + "\\" + z + ".bin").c_str(), "wb");
				if (error != 0) {
					cout << "Error export skip image: " << error << endl;
				}
				else {
					fwrite(Orca.bufAddr[n%Orca.number_of_buffer], sizeof(unsigned short), Orca.height * Orca.width, fpw);
					cout << "\rImage Export: " << n << "/" << Picnumber_c1 << " / total: " << MOVIE_stats.TargetImageNum;
					// cout << "\rImage Export: " << Picnumber_c1_w << " / " << Picnumber_c1;
					fclose(fpw);
				}
				MOVIE_skip_size = n + 1;
			}
			for (int i = 0; i < MOVIE_stats.TargetImageNum; ++i) {
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				if (MOVIE_stats.MOVIE_skip_list[i] > BKarea) {
					skipval += 1;
				}
			}
			if (skipval == 0) {
				MOVIE_stats.skip = true;
				if (MOVIE_stats.cancel_flag != 1) {
					DataExport_blank();
				}
			}
			else {
				MOVIE_stats.skip = false;
			}
			MOVIE_stats.MOVIE_skip_thread = 0;
		}
		else {
			MOVIE_stats.skip = false;
		}
		MOVIE_stats.MOVIE_skip_thread = 0;
		cout << "MOVIE_Skip_Check_func End " << endl;
	}
	private:void MOVIEFocus_func() {
		threads.MOVIE_focus = true;
		Running_LOG("MOVIEFocus_func() :Start");
		cout << "MOVIEFocus_func() :Start" << endl;
		DCTBlockCalc_flag = 0;

		double BK = double::Parse(BK_box->Text);
		int countup = 0;
		cv::Rect roi(0, 0, Orca.height, Orca.width);
		int calced = 1;
		chrono::system_clock::time_point start, end;
		MOVIE_stats.MOVIE_Focus = 2;
		while (MOVIE_stats.ExposureFlag != 1) {
			Sleep(10);
			if (MOVIE_stats.Skiptile == true)
				break;
			if (MOVIE_stats.cancel_flag == 1)
				break;
		}
		MOVIE_stats.MOVIE_Focus = 1;
		while (Picnumber_c1 < MOVIE_stats.TargetImageNum - 1) {
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.Skiptile == true)
				break;
			if (acqdata_flag == false)
				break;
			if (Picnumber_c1 > calced) {
				start = chrono::system_clock::now();
				if (calced == 1) {
					MOVIEFocusImages.Image1 = cv::Mat(Orca.height, Orca.width, CV_16UC1, Orca.bufAddr[(calced - 1) % Orca.number_of_buffer]);
					MOVIEFocusImages.Image2 = cv::Mat(Orca.height, Orca.width, CV_16UC1, Orca.bufAddr[calced%Orca.number_of_buffer]);
					countup += DCT_Calc(MOVIEFocusImages.Image1, MOVIEFocusImages.Image2, BK, deltaDCTVal[calced]);
					//countup += DCT_Calc_large(Image1, Image2, BK);
				}
				else {
					if (calced % 2 == 0) {
						MOVIEFocusImages.Image1 = cv::Mat(Orca.height, Orca.width, CV_16UC1, Orca.bufAddr[calced% Orca.number_of_buffer]);
						countup += DCT_Calc(MOVIEFocusImages.Image1, MOVIEFocusImages.Image2, BK, deltaDCTVal[calced]);
						//countup += DCT_Calc_large(Image1, Image2, BK);
						//cout << "Image1: " << calced << endl;
					}
					else {
						MOVIEFocusImages.Image2 = cv::Mat(Orca.height, Orca.width, CV_16UC1, Orca.bufAddr[calced % Orca.number_of_buffer]);
						countup += DCT_Calc(MOVIEFocusImages.Image1, MOVIEFocusImages.Image2, BK, deltaDCTVal[calced]);
						//countup += DCT_Calc_large(Image1, Image2, BK);
						//cout << "Image2: " << calced << endl;
					}
				}
				//cout << "FocusVal: " <<  FocusVal << endl;

				++calced;

				if (countup > SHEET_AF_SPAN) {
					MOVIEFocusVal.FocusVal -= 1;
					//MOVIE_focus_feedback_flag = 1;
					countup = 0;

				}
				else if (countup < -SHEET_AF_SPAN) {

					MOVIEFocusVal.FocusVal += 1;
					//MOVIE_focus_feedback_flag = 1;
					countup = 0;

				}
				end = chrono::system_clock::now();
				//cout << countup << " \tDCT Calc time: " << calced << " : " << static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0) << " ms\n";
				DCT_calc_time->Text = (static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0)).ToString("F2");

			}
			if (multicolorcount == 0) {
				if (abs(MOVIEFocusVal.FocusVal) > MOVIE_stats.MOVIE_Retry_int) {
					MOVIE_stats.retry = true;
				}
			}
			else {
				MOVIE_stats.retry = false;
			}

		}
		DCTBlockCalc_flag = 999;
		MOVIE_stats.MOVIE_Focus = 0;
		Running_LOG("MOVIEFocus_func() :End");
		threads.MOVIE_focus = false;
		//cout << "MOVIEFocus_func finished" << endl << endl;
	}

	private:int check_values() {
		double v;
		if (double::TryParse(exposure_time_box->Text, v) == false)
			return -1;
		if (double::TryParse(windowsize_box->Text, v) == false)
			return -1;
		return 0;
	}
	private:int SetTrigger(Orca_Param & P) {
		DCAMERR err;

		err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERSOURCE__EXTERNAL);
		if (!failed(err)) {
			cout << "Set trigger to EXTERNAL OK" << endl;
			return 0;
		}
		else {
			cout << "Error! : Set trigger to EXTERNAL" << endl;
			return -1;
		}

		err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERACTIVE__EDGE);
		if (failed(err))
			cout << "Error! : Set DCAMPROP_TRIGGERACTIVE__EDG" << endl;

		err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERENABLE_POLARITY__NEGATIVE);
		if (failed(err))
			cout << "Error! : Set  DCAMPROP_TRIGGERENABLE_POLARITY__NEGATIVE" << endl;


	}
	private:int ShutterMode(Orca_Param & P) {
		DCAMERR err;
		if (Axial_mode_check->Checked == true) {
			err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_SENSORMODE, DCAMPROP_SENSORMODE__PROGRESSIVE);
			if (!failed(err)) {
				cout << "Set light-sheet mode OK" << endl;
			}
			else {
				cout << "Error! : Set light-sheet mode" << endl;
				//return -1;
			}
			double cyc; //ms
			double wnd;
			double spd; //m/s

			if (double::TryParse(exposure_time_box->Text, cyc)) {
				if (double::TryParse(windowsize_box->Text, wnd)) {
					double travel = ((double)P.pixelheight * (double)P.height / 1000.0 / 1000.0) * (((double)wnd + 1) / (double)wnd);// um -> m ;
					cout << P.pixelheight << ": " << P.height << ": " << wnd << endl;
					cout << travel << endl;
					double speed = 1000.0 / (cyc - 5) * travel;
					cout << travel << endl;
					err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_INTERNALLINESPEED, speed);


					if (!failed(err)) {
						cout << "Set light-sheet line speed set to " << speed << " m/s" << endl;
					}
					else {
						cout << "Error!!  Set light-sheet line speed set to " << speed << " m/s" << endl;
						//return -1;
					}
					return 1;
				}
			}

		}
		else {
			err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_SENSORMODE, DCAMPROP_SENSORMODE__AREA);
			if (!failed(err)) {
				cout << "Set default mode (not light-sheet mode)" << endl;
				return 0;

			}
			else {
				cout << "Error! : Set default mode (not light-sheet mode)" << endl;
				return -1;
			}

		}
	}	private: int SetExposure(Orca_Param& P) {
		double v = 0.03;
		double wnd = 1;
		DCAMERR err;
		/*
		if (!double::TryParse(exposure_time_box->Text, v)) {
		cout << "Exposuretime value error" << endl;
		return -1;
		}
		if (Axial_mode_check->Checked == true) {
		double::TryParse(windowsize_box->Text, wnd);
		}

		if (v < 1) {
		cout << "Exposuretime value error" << endl;
		return -1;
		}
		*/
		double::TryParse(ExpoTime_Axial->Text, v);
		v = v / 1000; //exposuretime (/s)
		err = dcamprop_setvalue(P.hdcam, DCAM_IDPROP_EXPOSURETIME, v);

		if (!failed(err)) {
			cout << "Exposuretime set to " << v * 1000 << " ms" << endl;
			return 0;
		}
		else {
			cout << "Exposuretime set error" << endl;
			return -1;
		}
		return 0;
	}

	private: int SetHotPixelCorrection(Orca_Param & P) {
		dcamprop_setvalue(P.hdcam, DCAM_IDPROP_DEFECTCORRECT_MODE, DCAMPROP_MODE__ON);
		dcamprop_setvalue(P.hdcam, DCAM_IDPROP_HOTPIXELCORRECT_LEVEL, DCAMPROP_HOTPIXELCORRECT_LEVEL__STANDARD);
		return 1;
	}
	private:void CameraSettings_Orca(Orca_Param & P) {
		int ret = 0;
		threads.CameraSetting = true;
		cout << "**********************************************" << endl;
		cout << "Orca Set setting " << endl;
		cout << "**********************************************" << endl;
		ret = ShutterMode(P);
		ret = SetTrigger(P);
		ret = SetExposure(P);
		//ret = SetDelay(P);
		ret = SetHotPixelCorrection(P);
		//ret = SetWindowsize(P);
		//if (Axial_mode_check->Checked == true)
		//ret = SetLineSpeed(P);

		cout << "**********************************************" << endl;
		threads.CameraSetting = false;
	}

	private:void CameraSettingUpdate() {
		cout << "CameraSettingUpdate" << endl;
		ScanSpeed_Calc();
		CameraSettings_Orca(Orca);

	}

	private: void Acquisition_LIVE(Orca_Param& P) {
		chrono::system_clock::time_point start, end;
		cout << "Acquisition_LIVE_Orca:" << Orca.cam << endl;
		threads.Camera = true;
		//Orca_Free(P);
		wait = 0;
		DCAMERR err;
		int ret;
		double dur;
		P.buf_flag = 0;
		pBuf1_flag = 0;
		DCAMWAIT_OPEN	waitopen;
		memset(&waitopen, 0, sizeof(waitopen));
		waitopen.size = sizeof(waitopen);
		waitopen.hdcam = P.hdcam;

		err = dcamwait_open(&waitopen);
		if (failed(err))
		{
			dcamcon_show_dcamerr(P.hdcam, err, "dcamwait_open()");
			ret = 1;
		}
		else
		{
			//cout << "a" << endl;
			P.hwait = waitopen.hwait;
			P.bufAddr = new void*[P.number_of_buffer];
			P.buf = new char[P.bufframebytes * P.number_of_buffer];
			memset(P.buf, 0, P.bufframebytes * P.number_of_buffer);
			int		i;
			for (i = 0; i < P.number_of_buffer; i++)
			{
				P.bufAddr[i] = P.buf + P.bufframebytes * i;
			}
			//cout << "b" << endl;
			DCAMBUF_ATTACH bufattach;
			memset(&bufattach, 0, sizeof(bufattach));
			bufattach.size = sizeof(bufattach);
			bufattach.iKind = DCAMBUF_ATTACHKIND_FRAME;
			bufattach.buffer = P.bufAddr;
			bufattach.buffercount = P.number_of_buffer;
			//cout << "c" << endl;
			// attach user buffer
			err = dcambuf_attach(P.hdcam, &bufattach);
			if (failed(err))
			{
				dcamcon_show_dcamerr(P.hdcam, err, "dcambuf_attach()");
				ret = 1;
			}
			else
			{
				// start_capture
				cout << "start_capture" << endl;
				err = dcamcap_start(P.hdcam, DCAMCAP_START_SEQUENCE);
				if (failed(err))
				{
					dcamcon_show_dcamerr(P.hdcam, err, "dcamcap_start()");
					ret = 1;
				}
				else
				{
					printf("\nStart Capture\n");
					DCAMWAIT_START waitstart;
					memset(&waitstart, 0, sizeof(waitstart));
					waitstart.size = sizeof(waitstart);
					waitstart.eventmask = DCAMWAIT_CAPEVENT_FRAMEREADY;
					waitstart.timeout = DCAMWAIT_TIMEOUT_INFINITE;// 10000;
					DCAMCAP_TRANSFERINFO	captransferinfo;
					memset(&captransferinfo, 0, sizeof(captransferinfo));
					captransferinfo.size = sizeof(captransferinfo);
					P.bufflag = 1;
					start = chrono::system_clock::now();
					end = chrono::system_clock::now();
					while (Abort_flag != 1)
					{
						DCAMERR err;
						//cout << "wait image" << endl;
						wait = 1;
						err = dcamwait_start(P.hwait, &waitstart);
						wait = 0;
						dur = static_cast<double>(chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now() - end).count() / 1000);
						//fps_box->Text = (1000.0 / dur).ToString("F2");
						//fps_box->Text = dur.ToString("F2");
						end = chrono::system_clock::now();
						if (failed(err))
						{
							dcamcon_show_dcamerr(P.hdcam, err, "dcamwait_start()");
							break;
						}
						err = dcamcap_transferinfo(P.hdcam, &captransferinfo);
						if (failed(err))
						{
							dcamcon_show_dcamerr(P.hdcam, err, "dcamcap_transferinfo()");
							continue;
						}
						else {

							P.buf_flag = 1;
							pBuf1_flag = 1;
							P.bufpos = captransferinfo.nNewestFrameIndex;
							//cout << "got image: findex=" << P.bufpos << endl;
						}
					}
				}
			}
			Orca_Free(P);
		}
		wait = 0;
		threads.Camera = false;
	}

	private: void Orca_Free(Orca_Param& P) {
		cout << "Orca_Free" << endl;
		//RunningLog("Orca buff Free");
		//WaitForSingleObject(mtx_camera, INFINITE);
		try {
			if (P.bufflag == 1) {
				P.buf_flag = 0;
				dcamwait_abort(P.hwait);
				dcamcap_stop(P.hdcam);
				printf("Stop Capture\n");
				dcambuf_release(P.hdcam);
				delete P.buf;
				delete P.bufAddr;
				dcamwait_close(P.hwait);
				P.bufflag = 0;
				cout << "Orca buffer released" << endl;
				//RunningLog("Orca buffer released");
				//LIVE_flag = 0;
			}
		}
		catch (exception& e) {
			cout << "Orca_STOP error" << endl;
		}
		//ReleaseMutex(mtx_camera);
	}

	private: void Acqisition_Camera_func() {
		cout << "Thread Acquisition Start" << endl;
		Thread_Stats.Camera = 1;
		threads.Camera = true;
		MOVIE_stats.camera = 9;
		chrono::system_clock::time_point start, end;
		//GetShutterInfomation(Orca);
		Picnumber_c1 = 0;
		MOVIE_stats.passed_image_num1 = 0;
		wait = 0;
		Orca_Free(Orca);
		DCAMERR err;
		int ret;
		double dur = 0.0;
		Orca.buf_flag = 0;
		DCAMWAIT_OPEN waitopen;
		memset(&waitopen, 0, sizeof(waitopen));
		waitopen.size = sizeof(waitopen);
		waitopen.hdcam = Orca.hdcam;
		err = dcamwait_open(&waitopen);
		if (failed(err)) {
			dcamcon_show_dcamerr(Orca.hdcam, err, "dcamwait_open()");
			cout << " dcamwait_open error" << endl;
			MOVIE_stats.camera = -1;
			goto errorpoin;
		}
		else {
			Orca.hwait = waitopen.hwait;
			Orca.bufAddr = new void*[Orca.number_of_buffer];
			Orca.buf = new char[Orca.bufframebytes * Orca.number_of_buffer];
			memset(Orca.buf, 0, Orca.bufframebytes * Orca.number_of_buffer);
			int i;
			for (i = 0; i < Orca.number_of_buffer; i++) {
				Orca.bufAddr[i] = Orca.buf + Orca.bufframebytes * i;
			}
			DCAMBUF_ATTACH bufattach;
			memset(&bufattach, 0, sizeof(bufattach));
			bufattach.size = sizeof(bufattach);
			bufattach.iKind = DCAMBUF_ATTACHKIND_FRAME;
			bufattach.buffer = Orca.bufAddr;
			bufattach.buffercount = Orca.number_of_buffer;
			Orca.bufflag = 1;
			// attach user buffer
			err = dcambuf_attach(Orca.hdcam, &bufattach);
			if (failed(err)) {
				dcamcon_show_dcamerr(Orca.hdcam, err, "dcambuf_attach()");
				cout << "dcambuf_attach error" << endl;
				MOVIE_stats.camera = -1;
				goto errorpoin;
				// ret = 1;
			}
			else {
				// start_capture
				err = dcamcap_start(Orca.hdcam, DCAMCAP_START_SEQUENCE);
				if (failed(err)) {
					dcamcon_show_dcamerr(Orca.hdcam, err, "dcamcap_start()");
					cout << " dcamcap_start error" << endl;
					MOVIE_stats.camera = -1;
					goto errorpoin;
				}
				else {
					printf("\nStart Capture\n");
					DCAMWAIT_START waitstart;
					memset(&waitstart, 0, sizeof(waitstart));
					waitstart.size = sizeof(waitstart);
					waitstart.eventmask = DCAMWAIT_CAPEVENT_FRAMEREADY;
					waitstart.timeout = DCAMWAIT_TIMEOUT_INFINITE; // 10000;
																   // waitstart.timeout = 2000;
					DCAMCAP_TRANSFERINFO captransferinfo;
					memset(&captransferinfo, 0, sizeof(captransferinfo));
					captransferinfo.size = sizeof(captransferinfo);
					DCAMERR err;
					MOVIE_stats.camera = 2;
					while (MOVIE_stats.ExposureFlag != 1) {
						if (MOVIE_stats.cancel_flag == 1) {
							break;
						}
						if (MOVIE_stats.Skiptile == true)
							break;
						Sleep(1);
					}
					printf("\nMOVIE_stats.ExposureFlag @ Camera\n");
					MOVIE_stats.camera = 1;
					while (1) {
						start = std::chrono::system_clock::now();
						err = dcamwait_start(Orca.hwait, &waitstart);
						MOVIE_stats.camera = 1;
						if (failed(err)) {
							dcamcon_show_dcamerr(Orca.hdcam, err, "dcamwait_start()");
							if (err == DCAMERR_ABORT) {
								break;
							}
							break;
						}
						err = dcamcap_transferinfo(Orca.hdcam, &captransferinfo);
						if (failed(err)) {
							dcamcon_show_dcamerr(Orca.hdcam, err, "dcamcap_transferinfo()");
							continue;
						}
						else {
							Orca.bufpos = captransferinfo.nNewestFrameIndex;
							Orca.buf_flag = 1;
							pBuf1_flag = 1;
							Picnumber_c1++;
							MOVIE_stats.passed_image_num1++;
						}
						if (Abort_flag == 1) {
							break;
						}
						if (MOVIE_stats.cancel_flag == 1) {
							break;
						}
						if (MOVIE_stats.Skiptile == true)
							break;
						if (Picnumber_c1 >= MOVIE_stats.TargetImageNum) {
							cout << "Acquired TargetImageNum" << endl;
							break;
						}
						dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start).count();
						BeginInvoke(gcnew Dele_LFocus(this, &MyForm::FocusLSet), (dur / 1000).ToString("F2"));
					}
					cout << dec << Picnumber_c1 << "images are acquired" << endl << endl;
					Orca.buf_flag = 0;
					dcamcap_stop(Orca.hdcam);
					// printf("Stop Capture\n");
					while (MOVIE_stats.data != 0) {
						Sleep(1);
						// printf("Stop Capture\n");
						if (MOVIE_stats.cancel_flag == 1) {
							break;
						}
						if (Abort_flag == 1) {
							break;
						}
						if (MOVIE_stats.Skiptile == true)
							break;
					}
				}
			}
		}
		//Orca_Free(Orca);
		MOVIE_stats.camera = 0;
		Thread_Stats.Camera = 0;
		threads.Camera = false;
		cout << "Acqisition_Camera_func finished Picnumber_c1:" << Picnumber_c1 << "images" << endl;
	errorpoin:

		return;
	}

	private:void Acquisition_LIVE_Thread(Object ^ n) {
		Acquisition_LIVE(Orca);
	}
	private:void doLoopOfAcquisition_func()
	{
		CameraSettingUpdate();
		pBuf1_flag = 0;
		if (check_values() == 0) {
			LIVE_flag = 1;
			ParameterizedThreadStart^ Start = gcnew ParameterizedThreadStart(this, &MyForm::Acquisition_LIVE_Thread);
			Thread^ thread2 = gcnew Thread(Start);
			thread2->Start(1);
		}
		else {
			std::cout << "error volues... " << std::endl;
		}

	}
	private:void doLoopOfAcquisition_func2()
	{
		LIVE_flag == 1;
		CameraSettings_Axial(Camera2.Hndl);
		CameraSettings_Axial(Camera2.Hndl);
		int iret;
		int BufSize;
		Picnumber_c1 = 0;
		try {
			AT_Command(Camera2.Hndl, L"AcquisitionStart");
			LIVE_flag = 1;
			Sleep(100);
			MOVIE_stats.camera = 2;
			while (LIVE_flag == 1) {
				iret = AT_WaitBuffer(Camera2.Hndl, &pBuf2, &BufSize, AT_INFINITE); //AT_INFINITE
				if (iret == AT_SUCCESS) {
					AT_QueueBuffer(Camera2.Hndl, AlignedBuffers2[Picnumber_c1% NumberOfBuffers], BufferSize2);
					pBuf2_flag = 1;
					Picnumber_c1++;
				}
				else {
					cout << "AT_WaitBuffer error" << endl;
					pBuf2_flag = 0;
					break;
				}
			}


		}
		catch (exception &e) {
			cout << "AcquisitionStart error" << endl;
		}
		AT_Command(Camera2.Hndl, L"AcquisitionStop");
		//cout << "doLoopOfAcquisition_func2() Finish" << endl;
		pBuf2_flag = 0;
		MOVIE_stats.camera = 0;
	}
	private: void LIVE_func() {
		while (Orca.buf_flag == 0) {
			Sleep(100);
			if (Abort_flag == 1)
				break;
		}
		LIVE_window1 = true;
		WORD* im = new WORD[Orca.width * Orca.height];

		cv::Mat Image; // raw image
		cv::Mat image;
		cv::Mat imageRoi;
		cv::Mat Output;
		string min_value;
		string max_value;
		int L_flag = 0;
		int R_flag = 0;
		int Rimagepos = 0;
		int Limagepos = 0;
		mparam.x = 0;
		mparam.y = 0;
		mparam.event = 0;
		mparam.flags = 0;
		LIVE1.x_live = Orca.width;
		LIVE1.y_live = Orca.height;
		int imagesize = Orca.width;
		cout << "image w" << Orca.width << endl;
		cout << "image h" << Orca.height << endl;

		cv::Mat ori = cv::Mat::ones(Orca.height / 3, Orca.width / 3, CV_8U);
		//cv::Mat blank = cv::Mat::ones(Orca.height, Orca.width, CV_16U);
		cv::namedWindow("Camera #1", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
		//cv::namedWindow("Camera 1x ROI", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
		int key = 0;
		int focus_pos = 0;
		cv::setMouseCallback("Camera #1", CallBackFunc, &mparam);
		int i = 0;
		while (LIVE_window1 == true) {
			// cout << "0:" << Live_Flag << Orca.buf_flag << endl;
			try {
				if (LIVE_flag == 1 && Orca.buf_flag == 1) {
					int ret = memcpy_s(im, Orca.height * Orca.width * 2, Orca.bufAddr[Orca.bufpos], Orca.height * Orca.width * 2);
					if (ret != 0) {
						Sleep(100);
						continue;
					}
					Image = cv::Mat(Orca.height, Orca.width, CV_16UC1, im);
				}
				if (LIVE_Acq_flag1 == true) {
					//cout << "# LIVE_Acq_flag == 1" << endl;
					Acquire_single(2, Image);
					// ReleaseMutex(mtx_data);
				}
				Output = Image.clone();

				cv::transpose(Output, Output);
				cv::flip(Output, Output, 1);
				//cv::flip(Output, Output, 0);
				cv::minMaxLoc(Output, &LIVE1.min_live, &LIVE1.max_live, NULL, NULL);
				int v = LIVE1.max_live;

				// cv::rotate(Output, Output, cv::ROTATE_90_CLOCKWISE);
				if (mouseevent_flag == 0) {
					mouseevent_flag = 1;
					BeginInvoke(gcnew Dele_MouseIvent(this, &MyForm::mouseivent));
				}

				if (LIVE1.x_live < 0) {
					LIVE1.x_live = 0;
				}
				if (LIVE1.y_live < 0) {
					LIVE1.y_live = 0;
				}
				//cout << "5" << endl;
				if (LIVE1.width_live > Camera1_sensorWidth) {
					LIVE1.width_live = Camera1_sensorWidth;
				}
				if (LIVE1.hight_live > Camera1_sensorHight) {
					LIVE1.hight_live = Camera1_sensorHight;
				}
				//cout << "6" << endl;
				if (LIVE1.y_live + LIVE1.width_live > Camera1_sensorWidth) {
					LIVE1.y_live = Camera1_sensorWidth - LIVE1.width_live;
				}
				if (LIVE1.x_live + LIVE1.hight_live > Camera1_sensorHight) {
					LIVE1.x_live = Camera1_sensorHight - LIVE1.hight_live;
				}
				//cout << "7" << endl;
				if (LIVE1.x_live < 0) {
					LIVE1.x_live = 0;
				}
				if (LIVE1.y_live < 0) {
					LIVE1.y_live = 0;
				}
				//cout << LIVE1.x_live << ":" << LIVE1.y_live << ":" << LIVE1.hight_live << ":" << LIVE1.width_live << endl;
				cv::Rect roi(LIVE1.x_live, LIVE1.y_live, LIVE1.hight_live, LIVE1.width_live);
				Output = Output(roi);
				try {
					if (ActiveWindow == 1) {
						if (LIVE1.y_on_live <int(Output.rows) && LIVE1.x_on_live < int(Output.cols)) {
							Xpos_cursol->Text = (LIVE1.x_on_live + LIVE1.x_live).ToString();
							Ypos_cursol->Text = (LIVE1.y_on_live + LIVE1.y_live).ToString();
							Cursol_int->Text = (Output.at<unsigned short>(LIVE1.y_on_live, LIVE1.x_on_live)).ToString();
							cv::minMaxLoc(Output, &LIVE1.min_live, &LIVE1.max_live, NULL, NULL);

						}
					}
					if (Image_window1 == 1) {
						ImageMax->Text = LIVE1.max_live.ToString();
						ImageMin->Text = LIVE1.min_live.ToString();
					}
				}
				catch (exception &e) {

				}
				if (manual_scale1->Checked == true) {
					Double::TryParse(SH_high1->Text, LIVE1.max_live);
					Double::TryParse(SH_low1->Text, LIVE1.min_live);
					if (LIVE1.min_live >= LIVE1.max_live) {
						LIVE1.min_live = 0;
						LIVE1.max_live = 65534;
					}
					Output = (Output - int(LIVE1.min_live))*int((65535 / (LIVE1.max_live - LIVE1.min_live)));
				}
				else {
					cv::normalize(Output, Output, 0, 65535, cv::NORM_MINMAX);
				}
				cv::imshow("Camera #1", Output);
				key = cv::waitKey(1);
				//++i;
			}
			catch (exception& e) {
				//Console::Write("\rCamera 1x: error");
				// cout << Orca.width << ":" << Orca.height << "/" << LIVE1.x_live
				// << "/" << LIVE1.y_live << "/" << LIVE1.width_live << "/" <<
				// LIVE1.hight_live << endl; getchar();
				LIVE1.x_live = 0;
				LIVE1.y_live = 0;
				LIVE1.width_live = Orca.height;
				LIVE1.hight_live = Orca.width;
				// LIVE_Acq_flag = 0;
				// Live_Flag = 0;
			}
		}
		Orca.buf_flag = 0;
		cv::destroyWindow("Camera #1");
		LIVE_window2 = false;
		//Live_Flag = 0;
		//Abort_ALL();
		delete[] im;
	}
	private: size_t Get_File_Size(const string &FP) {
		size_t size = 0;
		fpos_t fpos = 0;
		errno_t error;
		FILE* stream;
		error = fopen_s(&stream, FP.c_str(), "rb");
		if (error != 0) {
			cout << "Cannot opne file: " << FP << endl;
			return 0;
		}
		else {
			_fseeki64(stream, 18, SEEK_SET);
			fgetpos(stream, &fpos);
			size_t fstart = fpos;
			_fseeki64(stream, 18, SEEK_END);
			fgetpos(stream, &fpos);
			size_t fend = fpos;
			fclose(stream);
			return (fend - fstart);
		}
	}
	private:void DAQ_sheet_Axial_Thread() {
		Thread_Stats.DAQ_VB = 1;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::DAQ_sheet_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void DAQ_sheet_Axial_Thread_AF() {
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::DAQ_sheet_TASK_Axial_AF_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void DAQ_sheet_Axial_Thread_skip() {
		Thread_Stats.DAQ_VB = 1;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::DAQ_sheet_TASK_Axial_skip_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void DAQ_sheetLIVE_Thread() {
		cout << "DAQ_sheetLIVE_Thread()" << endl;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::DAQ_sheet_LIVE_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void DataExportThread()
	{
		Thread_Stats.DataExport = 1;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::DataExport_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void Acqisition_CameraThread()
	{

		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Acqisition_Camera_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void LIVE_Thread()
	{
		cout << " LIVE_Thread start \n";
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::LIVE_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();

	}
	private:void MOVIE_Focus_thread() {

		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::MOVIEFocus_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void doLoopOfAcquisitionThread()
	{
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::doLoopOfAcquisition_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void MOVIE_stage()
	{
		//cout << " LIVE_ExtarnalThread start \n";
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::MOVE_stage_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:void Acquire_single(int X, cv::Mat image) {
		cout << "Acquire !" << endl;

		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		String^ Dir = Export_Box->Text;
		Dir_check(Dir);
		cv::Rect roi(cv::Point(0, 0), cv::Size(2304, 2304));

		cv::Mat Img;
		if (MOVIE_stats.SingleExposure == 1) {
			if (X == 2) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\Center_" + date + "_" + Stem + "_2.tif";
				Console::WriteLine("Export: " + Path);
				cv::Mat Img = image(roi);
				cv::transpose(Img, Img);
				cv::flip(Img, Img, -1);

				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), Img);
				MOVIE_stats.SingleExposure = 0;
			}
		}
		else {
			if (X == 1) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\" + date + "_" + Stem + "_1.tif";
				Console::WriteLine("Export: " + Path);
				cv::Mat Img = image(roi);
				cv::transpose(Img, Img);
				cv::flip(Img, Img, -1);

				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), Img);
			}
			if (X == 2) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\" + date + "_" + Stem + "_2.tif";
				Console::WriteLine("Export: " + Path);
				cv::Mat Img = image(roi);
				cv::transpose(Img, Img);
				cv::flip(Img, Img, -1);

				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), Img);
			}
			if (X == 10) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\PreScan_XY.tif";
				Console::WriteLine("Export: " + Path);
				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), image);
			}
			if (X == 11) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\PreScan_YZ.tif";
				Console::WriteLine("Export: " + Path);
				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), image);
			}
			if (X == 12) {
				String^ Stem = Stem_box1->Text;
				String^ Path = Dir + "\\PreScan_XZ.tif";
				Console::WriteLine("Export: " + Path);
				string path;
				MarshalString(Path, path);
				cv::imwrite(path.c_str(), image);
			}
		}
		LIVE_Acq_flag1 = 0;
		LIVE_Acq_flag2 = 0;
	}
	private:void LIVE_Axial_DAQ2() {
		cout << "Start LIVE_Axial_DAQ" << endl;
		Abort_flag = 0;
		LIVE_flag = 1;
		chrono::system_clock::time_point start_L, end_L;

		double Exposuretime = double::Parse(exposure_time_box->Text);
		double Exp = double::Parse(ExpoTime_Axial->Text);
		double delay = double::Parse(delay_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);

		int SampleNum_sheet = Exposuretime * 20;
		double SampleRate = 20000;//0.05ms
		double *data_sheet = new double[SampleNum_sheet * 4];
		char Dev_sheet[100] = DAQ1_ALL;
		Set_ETL_param();
		DAQ_ETL_TASK_prep2(data_sheet, SampleNum_sheet);
		//cout << Dev_sheet << endl;
		deltatime_G = 0;
		double imagingtime = 0;
		double deltatime = 0;
		int imagenum = 0;
		int32 error = 0;
		char errBuff[2048] = { '\0' };
		update_z_galv();
		DataChangeFlag = 0;
		//Start Sheet
		DAQ1(data_sheet, SampleNum_sheet, SampleRate);
		Axial_flag = 1;
		MOVIE_stats.ExposureFlag = 1;
		while (Abort_flag == 0) {
			if (DataChangeFlag == 1) {
				Set_ETL_param();
				//update_z_galv();
				DAQ_ETL_TASK_prep2(data_sheet, SampleNum_sheet);
				DAQ1(data_sheet, SampleNum_sheet, SampleRate);
				DataChangeFlag = 0;
			}
		}
		MOVIE_stats.ExposureFlag = 0;
		Axial_flag = 0;
		DAQ_clear();
		delete[] data_sheet;
	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(13);
		DAQ1_clear();
	}

	private:void Axial_MOVIE_VB_wMOVIEfocus()//For MOVIE
	{
		chrono::system_clock::time_point start_L, end_L;
		DAQ1_clear();
		DAQ2_clear(7);
		threads.Daq = true;
		string DEV = Z_GALV_DAQ_L;
		double z_val = 0;
		if (illumination_flag == 'L') {
			z_val = MOVIE_stats.InitialFocus_L;
			DEV = Z_GALV_DAQ_L;
		}
		else {
			z_val = MOVIE_stats.InitialFocus_R;
			DEV = Z_GALV_DAQ_R;
		}
		double d = z_val;
		cout << " Axial_MOVIE_VB_wMOVIEfocus Z ini: " << d << endl;
		if (d > 10) {
			d = 9;
			z_val = 9;
		}
		else if (d < -10) {
			d = -9;
			z_val = -9;
		}
		double Exposuretime = double::Parse(exposure_time_box->Text);
		double delay = double::Parse(delay_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		int SampleNum_sheet = Exposuretime * 20;
		double SampleRate = 20000;//0.1ms
		double *data_sheet = new double[SampleNum_sheet * 4];
		char Dev_sheet[100] = DAQ1_ALL;

		Set_ETL_param();
		DAQ_ETL_TASK_prep2(data_sheet, SampleNum_sheet);//ETL control by DAQ
		MOVIE_stats.DAQ = 0;
		//DAQ_sheet_endpos();
		int32       error = 0;
		DAQ1_flag = 1;
		char        errBuff[2048] = { '\0' };
		Sleep(100);
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev_sheet, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", SampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, SampleNum_sheet));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, SampleNum_sheet, false, 10.0, DAQmx_Val_GroupByChannel, data_sheet, NULL, NULL));

		DAQmxErrChk2(DAQmxCreateTask("", &taskHandle2));
		DAQmxErrChk2(DAQmxCreateAOVoltageChan(taskHandle2, DEV.c_str(), "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk2(DAQmxWriteAnalogScalarF64(taskHandle2, 1, 0, z_val, NULL));

		double Ampli = 0.8;
		if (double::TryParse(MOVIE_Amplitude->Text, Ampli)) {
			if (Ampli < 0 || Ampli > 10) {
				Ampli = 0.8;
				Console::WriteLine("Input out of range. Amplitude reset to default: 0.8");
			}
		}
		else {
			Ampli = 0.8;
			Console::WriteLine("Invalid input. Amplitude set to default: 0.8");
		}
		if (MOVIE_stats.retrycount == 1)
			Ampli = 0.4;
		if (MOVIE_stats.retrycount == 2)
			Ampli = 1.6;
		if (MOVIE_stats.retrycount > 2)
			Ampli = 0;
		cout << " Axial_MOVIE_VB_wMOVIEfocus Amplitude: " << Ampli << " mV" << endl;
		Ampli = Ampli / 2000;
		MOVIEFocusVal.FocusVal = 0;
		MOVIE_stats.DAQ = 2;
		MOVIE_stats.External = 2;
		int MOVIE_Focus_pos = 0;
		int flag = 0;
		Running_LOG(" Axial_MOVIE_VB_R: MOVIE_stats.External = 2");
		while (MOVIE_stats.ExposureFlag != 1) {
			Sleep(10);
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (acqdata_flag == false)
				break;
			if (MOVIE_stats.Skiptile == true)
				break;
		}
		DAQmxErrChk(DAQmxStartTask(taskHandle));
		MOVIE_stats.DAQ = 1;
		MOVIE_focus_feedback_flag = 0;

		for (; Picnumber_c1 < MOVIE_stats.TargetImageNum;) {
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.Skiptile == true)
				break;
			MOVIE_stats.Stage_pos_list[Picnumber_c1] = Stagepos_current.z;
			if (multicolorcount == 0 || multi_laser_check->Checked == false) {
				if (Picnumber_c1 % 2 == 0) {
					d = z_val - Ampli * 2 + MOVIEFocusVal.FocusVal  * Ampli;
					MOVIE_Focus_pos = -1;
				}
				else {
					d = z_val + Ampli * 2 + MOVIEFocusVal.FocusVal  * Ampli;
					MOVIE_Focus_pos = +1;
				}
				MOVIE_stats.MOVIE_focus_list[Picnumber_c1] = d;
			}
			else {
				d = MOVIE_stats.MOVIE_focus_list[Picnumber_c1];
			}
			if (d > 10)
				d = 10;
			else if (d < -10)
				d = -10;
			BeginInvoke(gcnew Dele_RFocus(this, &MyForm::FocusRSet), (d * 1000).ToString("F1"));
			MOVIE_stats.MOVIE_Focus_pos = MOVIE_Focus_pos;
			if (MOVIE_stats.camera == 0) {
				cout << "MOVIE_stats.camera == 0" << endl;
				break;
			}
			DAQmxErrChk2(DAQmxWriteAnalogScalarF64(taskHandle2, 1, 0, d, NULL));
			Sleep(2);
		}
		MOVIE_stats.DAQ = 0;
		Sleep(200);
		delete[] data_sheet;
		MOVIE_stats.ExposureFlag = 0;
		threads.Daq = false;
		DAQ_sheet_endpos();
		DAQ2_clear(9);

	Error:
		if (DAQmxFailed(error))
			DAQ_ERR(14);
		DAQ1_clear();

	Error2:
		if (DAQmxFailed(error))
			DAQ_ERR(142);

		DAQ2_clear(10);
	}
	private: void Axial_MOVIE_woMOVIEfocus_AF()//For skip
	{
		int retrycount = 0;
		int err = 0;

		Running_LOG("Axial_MOVIE_woMOVIEfocus_AF()");
		//cout << "Axial_MOVIE_woMOVIEfocus_AF" << endl;
		chrono::system_clock::time_point start_L, end_L;
		string DEV = Z_GALV_DAQ_L;
		//double z_val = 0;
		double z_val = 0;
		double spos = 0;
		if (double::TryParse(s_pos_current->Text, spos)) {
			if (abs(spos < 150)) {
				if (illumination_flag == 'L') {
					z_val = double::Parse(Z_galv_L->Text) / 1000;
					//z_val = double::Parse(Z_offset_current->Text) / 1000;
				}
				else {
					z_val = double::Parse(Z_galv_R->Text) / 1000;
					//z_val = double::Parse(Z_offset_current->Text) / 1000;
					DEV = Z_GALV_DAQ_R;
				}
			}
			else {
				if (illumination_flag == 'L') {
					z_val = double::Parse(Z_galv_L_RV->Text) / 1000;
					//z_val = double::Parse(Z_offset_current->Text) / 1000;
				}
				else {
					z_val = double::Parse(Z_galv_R_RV->Text) / 1000;
					//z_val = double::Parse(Z_offset_current->Text) / 1000;
					DEV = Z_GALV_DAQ_R;
				}
			}
		}

		err = 1;
		double Exposuretime = double::Parse(exposure_time_box->Text);
		double delay = double::Parse(delay_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		int SampleNum_sheet = Exposuretime * 20;
		double SampleRate = 20000;//0.1ms
		double *data_sheet = new double[SampleNum_sheet * 4];

		char Dev_sheet[100] = DAQ1_ALL;
		Set_ETL_param();
		DAQ_ETL_TASK_prep2(data_sheet, SampleNum_sheet);//ETL control by DAQ
		//double Ampli = INITIAL_AF_SPAN;
		double Ampli = int::Parse(AFspan->Text);
		cout << "AF stats: span " << Ampli << " ,num" << MOVIE_stats.TargetImageNum << endl;
		Ampli = Ampli / 1000;
		/*
		for (int i = 0; i < INITIAL_AF_NUM; ++i) {
			AF_stats[i].Zpos = z_val - (Ampli*INITIAL_AF_NUM / 2) + i * Ampli;
		}*/
		for (int i = 0; i < MOVIE_stats.TargetImageNum; ++i) {
			AF_stats[i].Zpos = z_val - (Ampli*MOVIE_stats.TargetImageNum / 2) + i * Ampli;
		}
		MOVIE_stats.DAQ = 999;
		DAQ_sheet_endpos();
		DAQ1_clear();
		DAQ2_clear(11);
		int32       error = 0;
		DAQ1_flag = 1;
		char        errBuff[2048] = { '\0' };
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev_sheet, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", SampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, SampleNum_sheet));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, SampleNum_sheet, 0, 10.0, DAQmx_Val_GroupByChannel, data_sheet, NULL, NULL));

		DAQmxErrChk(DAQmxCreateTask("", &taskHandle2));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle2, DEV.c_str(), "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxWriteAnalogScalarF64(taskHandle2, 1, 0, AF_stats[0].Zpos, NULL));
		deltatime_G = 0;
		double imagingtime = 0;
		double deltatime = 0;
		int imagenum = 0;
		MOVIE_stats.DAQ = 2;
		MOVIE_stats.External = 2;
		double d;
		double ImagingTime = Exposuretime - delay - EXTRA_TIME;
		Running_LOG(" Axial_MOVIE_woMOVIEfocus_AF: MOVIE_stats.External = 2");
		//cout << " Axial_MOVIE_woMOVIEfocus_AF: MOVIE_stats.External = 2" << endl;
		//SheetGen();
		while (MOVIE_stats.ExposureFlag != 1) {
			Sleep(10);
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			if (MOVIE_stats.retry == true) {
				break;
			}
		}
		Running_LOG(" Axial_MOVIE_woMOVIEfocus_AF:Start");

		//cout << "Axial_MOVIE_woMOVIEfocus_AF:DAQmxStartTask" << endl;
		DAQmxErrChk(DAQmxStartTask(taskHandle));
		MOVIE_stats.DAQ = 1;
		for (int n = 0; n < MOVIE_stats.TargetImageNum; ++n) {
			DAQmxErrChk(DAQmxWriteAnalogScalarF64(taskHandle2, 1, 0, AF_stats[n].Zpos, NULL));
			if (MOVIE_stats.cancel_flag == 1)
				break;
			while (Picnumber_c1 <= n) {
				Sleep(1);
				if (MOVIE_stats.cancel_flag == 1)
					break;
			}
		}
		Sleep(100);
		MOVIE_stats.DAQ = 0;
		MOVIE_stats.ExposureFlag = 0;
		Running_LOG("Axial_MOVIE_woMOVIEfocus_AF finish");
		err = 0;
		DAQ1_clear();
		DAQ2_clear(12);
	Error:
		if (err != 0) {
			if (DAQmxFailed(error)) {
				DAQ_ERR(15);
			}
			DAQ_clear();
		}
	Error2:
		if (err != 0) {
			if (DAQmxFailed(error)) {
				DAQ_ERR(155);
			}
			DAQ2_clear(13);
		}

		DAQ_sheet_endpos();
		delete[] data_sheet;
	}
	private: void exportconditions() {

		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		Logfilename = date;
		Console::WriteLine(date);
		String^ Exposuretime = exposure_time_box->Text;
		String^ ScanStep = Step->Text;
		String^ ZS = Start_Z_pos->Text;
		String^ ZE = End_Z_pos->Text;
		String^ dir = Export_Box->Text;
		String^ stem1 = Stem_box1->Text;
		String^ X = X_pos_current->Text;
		X = X->Trim('\r');
		String^ Y = Y_pos_current->Text;
		Y = Y->Trim('\r');

		String^ filename;
		SaveFileDialog^ dlg = gcnew SaveFileDialog;
		dlg->InitialDirectory = "initial_path\\Setting";
		if (dlg->ShowDialog() != System::Windows::Forms::DialogResult::OK)return;
		filename = dlg->FileName;

		try {
			if (filename->IndexOf(".gps") == -1) {
				filename = (filename + ".gps");
			}
			StreamWriter^ sw = gcnew StreamWriter(filename);
			sw->WriteLine("Directory: " + dir);
			sw->WriteLine("Stem1: " + stem1);
			if (Axial_mode_check->Checked == true) {
				sw->WriteLine("Axial mode-ON");

			}
			else {
				sw->WriteLine("Axial mode-OFF");

			}
			if (singlestackmode->Checked == true) {
				sw->WriteLine("singlestackmode: on");
			}
			if (multi_laser_check->Checked == true) {
				sw->WriteLine("multi_laser_check ==true");
				if (Laser_Info.L488.stat == 1) {
					sw->WriteLine("Laser488 / " + L488_box->Text + "mW");
				}
				if (Laser_Info.L592.stat == 1) {
					sw->WriteLine("Laser592 / " + L592_box->Text + "mW");
				}
				if (Laser_Info.L647.stat == 1) {
					sw->WriteLine("Laser647 / " + L647_box->Text + "mW");
				}
			}

			sw->WriteLine("Laser/Filter");
			string list;
			int LF_num = Laser_Filter_count(list);

			sw->WriteLine("Laser_Filter_set_count: " + LF_num);
			sw->WriteLine("Laser/Filter/Laser_power(mw)/exposure_time(ms)/FocusPosition");

			for (int i = 0; i < LF_num; ++i) {
				int L = 0;
				int P = 0;
				int F = 0;
				int E = 0;
				int Fo = 0;
				String^ setname = "";
				Laser_Filter_check(list[i], L, F, P, E, Fo, setname);
				if (L != 0) {
					sw->WriteLine(L + "/" + F + "/" + P + "/" + E + "/" + Fo + "/" + setname);

				}

			}

			sw->WriteLine("Exposuretime: " + Exposuretime);
			sw->WriteLine("Amplitude_sheet: " + Amplitude_sheet->Text);
			sw->WriteLine("Magnification: " + MAG);
			sw->WriteLine("Z_ScanStep: " + ScanStep);
			sw->WriteLine("X/Y: " + X + "/" + Y);
			sw->WriteLine("Start_Z/End_Z: " + ZS + "/" + ZE);
			sw->WriteLine("Tiling conditions:");
			sw->WriteLine("Top: " + top_limit->Text);
			sw->WriteLine("left: " + left_limit->Text);
			sw->WriteLine("bottom: " + bottom_limit->Text);
			sw->WriteLine("right: " + right_limit->Text);
			sw->WriteLine("ROI_Hight: " + Camera1_sensorHight.ToString());
			sw->WriteLine("ROI_Width: " + Camera1_ImageWidth.ToString());
			sw->WriteLine("ROI_data: " + Camera1_dataWidth.ToString());
			sw->WriteLine("Camera_Hight: " + Camera1_sensorHight.ToString());
			sw->WriteLine("Camera_Width: " + Camera1_sensorWidth.ToString());
			sw->WriteLine("MOVIE_Amplitude: " + MOVIE_Amplitude->Text);
			sw->WriteLine("filter_setting1: " + filter_name1->Text);
			sw->WriteLine("filter_setting2: " + filter_name2->Text);
			sw->WriteLine("filter_setting3: " + filter_name3->Text);
			sw->WriteLine("filter_setting4: " + filter_name4->Text);
			sw->WriteLine("filter_setting5: " + filter_name5->Text);
			sw->WriteLine("filter_setting6: " + filter_name6->Text);
			sw->WriteLine("filter_setting7: " + filter_name7->Text);
			sw->WriteLine("filter_setting8: " + filter_name8->Text);
			sw->WriteLine("Background: " + BK_box->Text);
			sw->WriteLine("sheet_offset_L: " + sheet_offset_L->Text);
			sw->WriteLine("sheet_offset_R: " + sheet_offset_R->Text);
			sw->WriteLine("MOVIE_Retry_Box: " + MOVIE_Retry_Box->Text);
			sw->WriteLine("Z_overlap: " + Z_overlap_box->Text);


			sw->WriteLine("top_limit_RV: " + top_limit_RV->Text);
			sw->WriteLine("bottom_limit_RV: " + bottom_limit_RV->Text);
			sw->WriteLine("left_limit_RV: " + left_limit_RV->Text);
			sw->WriteLine("right_limit_RV: " + right_limit_RV->Text);

			sw->WriteLine("Start_Z_pos_RV: " + Start_Z_pos_RV->Text);
			sw->WriteLine("End_Z_pos_RV: " + End_Z_pos_RV->Text);
			sw->WriteLine("ReverseSpos: " + ReverseSpos->Text);
			sw->WriteLine("Theta_POS_X: " + Theta_POS_X->Text);
			sw->WriteLine("Theta_POS_Z: " + Theta_POS_Z->Text);
			sw->WriteLine("windowsize_box: " + windowsize_box->Text);
			sw->WriteLine("delay_time_box: " + delay_time_box->Text);
			sw->WriteLine("Extra_time_box: " + Extra_time_box->Text);
			sw->WriteLine("Z_galv_L: " + Z_galv_L->Text);
			sw->WriteLine("Z_galv_R: " + Z_galv_R->Text);
			sw->WriteLine("Z_galv_L_RV: " + Z_galv_L_RV->Text);
			sw->WriteLine("Z_galv_R_RV: " + Z_galv_R_RV->Text);

			sw->Close();
			Console::WriteLine("a setting file ('{0}') has been written", filename);

		}
		catch (System::Exception^ e) {
			MessageBox::Show(e->ToString());
		}
	}

	private: void export_tiling() {
		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		Logfilename = date;
		Console::WriteLine(date);
		String^ Exposuretime = exposure_time_box->Text;
		String^ ScanStep = Step->Text;
		String^ ZS = Start_Z_pos->Text;
		String^ ZE = End_Z_pos->Text;
		String^ X = X_pos_current->Text;
		X = X->Trim('\r');
		String^ Y = Y_pos_current->Text;
		Y = Y->Trim('\r');
		String^ filename = "initial_path\\LOG\\LOG_" + date + ".log";


		StreamWriter^ sw = gcnew StreamWriter(filename);
		String^ dir = Export_Box->Text;
		Dir_check(dir);
		string dir_s;
		MarshalString(dir, dir_s);
		String^ Path = dir + "\\LOG_" + date + ".log";
		StreamWriter^ sw2 = gcnew StreamWriter(Path);
		String^ stem1 = Stem_box1->Text;


		sw->WriteLine("mode: tiling");
		sw->WriteLine(DateTime::Now);
		sw2->WriteLine("mode: tiling");
		sw2->WriteLine(DateTime::Now);
		if (Axial_mode_check->Checked == true) {
			sw->WriteLine("Axial mode-ON");
			sw2->WriteLine("Axial mode-ON");
		}
		else {
			sw->WriteLine("Axial mode-OFF");
			sw2->WriteLine("Axial mode-OFF");
		}
		if (singlestackmode->Checked == true) {
			sw->WriteLine("singlestackmode: off");
			sw2->WriteLine("singlestackmode: off");
		}


		sw->WriteLine("Directory: " + dir);
		sw->WriteLine("Stem1: " + stem1);
		sw2->WriteLine("Directory: " + dir);
		sw2->WriteLine("Stem1: " + stem1);

		sw->WriteLine("Z_left_offset: " + Z_galv_L->Text);
		sw->WriteLine("Z_right_offset: " + Z_galv_R->Text);
		sw2->WriteLine("Z_left_offset: " + Z_galv_L->Text);
		sw2->WriteLine("Z_right_offset: " + Z_galv_R->Text);
		sw->WriteLine("Background: " + BK_box->Text);
		sw2->WriteLine("Background: " + BK_box->Text);
		sw->WriteLine("sheet_offset_L: " + sheet_offset_L->Text);
		sw->WriteLine("sheet_offset_R: " + sheet_offset_R->Text);
		sw2->WriteLine("sheet_offset_L: " + sheet_offset_L->Text);
		sw2->WriteLine("sheet_offset_R: " + sheet_offset_R->Text);
		if (multi_laser_check->Checked == true) {
			sw->WriteLine("multi_laser_check ==true");
			sw2->WriteLine("multi_laser_check ==true");
			if (Laser_Info.L488.stat == 1) {
				sw->WriteLine("Laser488 / " + L488_box->Text + "mW");
				sw2->WriteLine("Laser488 / " + L488_box->Text + "mW");
			}
			if (Laser_Info.L592.stat == 1) {
				sw->WriteLine("Laser592 / " + L592_box->Text + "mW");
				sw2->WriteLine("Laser592 / " + L592_box->Text + "mW");
			}
			if (Laser_Info.L647.stat == 1) {
				sw->WriteLine("Laser647 / " + L647_box->Text + "mW");
				sw2->WriteLine("Laser647 / " + L647_box->Text + "mW");
			}
			if (Laser_Info.L488.stat == 1) {
				sw->WriteLine("Laser488 / " + L488_box->Text + "mW");
				sw2->WriteLine("Laser488 / " + L488_box->Text + "mW");
			}
			if (Laser_Info.L592.stat == 1) {
				sw2->WriteLine("Laser592 / " + L592_box->Text + "mW");
				sw->WriteLine("Laser592 / " + L592_box->Text + "mW");
			}
			if (Laser_Info.L647.stat == 1) {
				sw->WriteLine("Laser647 / " + L647_box->Text + "mW");
				sw2->WriteLine("Laser647 / " + L647_box->Text + "mW");
			}
			sw->WriteLine("Laser/Filter");
			sw2->WriteLine("Laser/Filter");
			string list;
			int LF_num = Laser_Filter_count(list);
			sw->WriteLine("Laser_Filter_set_count: " + LF_num);
			sw->WriteLine("Laser/Filter/Laser_power(mw)/exposure_time(ms)/FocusPosition");
			sw2->WriteLine("Laser_Filter_set_count: " + LF_num);
			sw2->WriteLine("Laser/Filter/Laser_power(mw)/exposure_time(ms)/FocusPosition");
			for (int i = 0; i < LF_num; ++i) {
				int L = 0;
				int P = 0;
				int F = 0;
				int E = 0;
				int Fo = 0;
				String^ setname = "";
				Laser_Filter_check(list[i], L, F, P, E, Fo, setname);
				if (L != 0) {
					sw->WriteLine(L + "/" + F + "/" + P + "/" + E + "/" + Fo + "/" + setname);
					sw2->WriteLine(L + "/" + F + "/" + P + "/" + E + "/" + Fo + "/" + setname);
				}
			}
		}

		if (Axial_mode_check->Checked == true) {
			sw->WriteLine("Axial mode-ON");
			sw2->WriteLine("Axial mode-ON");
		}
		else {
			sw->WriteLine("Axial mode-OFF");
			sw2->WriteLine("Axial mode-OFF");
		}
		if (singlestackmode->Checked == true) {
			sw->WriteLine("singlestackmode: on");
			sw2->WriteLine("singlestackmode: on");
		}
		sw->WriteLine("Exposuretime: " + Exposuretime);
		sw->WriteLine("Amplitude_sheet: " + Amplitude_sheet->Text);
		sw->WriteLine("Magnification: " + MAG);
		sw->WriteLine("Z_ScanStep: " + ScanStep);
		sw->WriteLine("X/Y: " + X + "/" + Y);
		sw->WriteLine("Start_Z/End_Z: " + ZS + "/" + ZE);
		sw->WriteLine("Tiling conditions:");
		sw->WriteLine("Top: " + top_limit->Text);
		sw->WriteLine("left: " + left_limit->Text);
		sw->WriteLine("bottom: " + bottom_limit->Text);
		sw->WriteLine("right: " + right_limit->Text);
		sw->WriteLine("Camera_Hight: " + Camera1_sensorHight.ToString());
		sw->WriteLine("Camera_Width: " + Camera1_sensorWidth.ToString());
		sw->WriteLine("Data_Width: " + Camera1_dataWidth.ToString());
		sw2->WriteLine("Exposuretime: " + Exposuretime);
		sw2->WriteLine("Amplitude_sheet: " + Amplitude_sheet->Text);
		sw2->WriteLine("Magnification: " + MAG);
		sw2->WriteLine("ScanStep: " + ScanStep);
		sw2->WriteLine("X/Y: " + X + "/" + Y);
		sw2->WriteLine("Start_Z/End_Z: " + ZS + "/" + ZE);
		sw2->WriteLine("Tiling conditions:");
		sw2->WriteLine("Top: " + top_limit->Text);
		sw2->WriteLine("left: " + left_limit->Text);
		sw2->WriteLine("bottom: " + bottom_limit->Text);
		sw2->WriteLine("right: " + right_limit->Text);
		sw2->WriteLine("Camera_Hight: " + Camera1_sensorHight.ToString());
		sw2->WriteLine("Camera_Width: " + Camera1_sensorWidth.ToString());
		sw2->WriteLine("Data_Width: " + Camera1_dataWidth.ToString());
		sw->WriteLine("set1_name: " + set1_name->Text);
		sw->WriteLine("set2_name: " + set2_name->Text);
		sw->WriteLine("set3_name: " + set3_name->Text);
		sw->WriteLine("set4_name: " + set4_name->Text);
		sw2->WriteLine("set1_name: " + set1_name->Text);
		sw2->WriteLine("set2_name: " + set2_name->Text);
		sw2->WriteLine("set3_name: " + set3_name->Text);
		sw2->WriteLine("set4_name: " + set4_name->Text);
		sw->WriteLine("MOVIE_Amplitude: " + MOVIE_Amplitude->Text);
		sw2->WriteLine("MOVIE_Amplitude: " + MOVIE_Amplitude->Text);
		sw->WriteLine("MOVIE_Retry_Box: " + MOVIE_Retry_Box->Text);
		sw2->WriteLine("MOVIE_Retry_Box: " + MOVIE_Retry_Box->Text);
		sw->WriteLine("Z_overlap: " + Z_overlap_box->Text);
		sw2->WriteLine("Z_overlap: " + Z_overlap_box->Text);

		sw->WriteLine("top_limit_RV: " + top_limit_RV->Text);
		sw->WriteLine("bottom_limit_RV: " + bottom_limit_RV->Text);
		sw->WriteLine("left_limit_RV: " + left_limit_RV->Text);
		sw->WriteLine("right_limit_RV: " + right_limit_RV->Text);

		sw2->WriteLine("top_limit_RV: " + top_limit_RV->Text);
		sw2->WriteLine("bottom_limit_RV: " + bottom_limit_RV->Text);
		sw2->WriteLine("left_limit_RV: " + left_limit_RV->Text);
		sw2->WriteLine("right_limit_RV: " + right_limit_RV->Text);

		sw->WriteLine("Start_Z_pos_RV: " + Start_Z_pos_RV->Text);
		sw->WriteLine("End_Z_pos_RV: " + End_Z_pos_RV->Text);
		sw->WriteLine("ReverseSpos: " + ReverseSpos->Text);
		sw->WriteLine("Theta_POS_X: " + Theta_POS_X->Text);
		sw->WriteLine("Theta_POS_Z: " + Theta_POS_Z->Text);
		sw->WriteLine("windowsize_box: " + windowsize_box->Text);
		sw->WriteLine("delay_time_box: " + delay_time_box->Text);
		sw->WriteLine("Extra_time_box: " + Extra_time_box->Text);
		sw2->WriteLine("Start_Z_pos_RV: " + Start_Z_pos_RV->Text);
		sw2->WriteLine("End_Z_pos_RV: " + End_Z_pos_RV->Text);
		sw2->WriteLine("ReverseSpos: " + ReverseSpos->Text);
		sw2->WriteLine("Theta_POS_X: " + Theta_POS_X->Text);
		sw2->WriteLine("Theta_POS_Z: " + Theta_POS_Z->Text);
		sw2->WriteLine("windowsize_box: " + windowsize_box->Text);
		sw2->WriteLine("delay_time_box: " + delay_time_box->Text);
		sw2->WriteLine("Extra_time_box: " + Extra_time_box->Text);
		sw->WriteLine("Z_galv_L: " + Z_galv_L->Text);
		sw->WriteLine("Z_galv_R: " + Z_galv_R->Text);
		sw->WriteLine("Z_galv_L_RV: " + Z_galv_L_RV->Text);
		sw->WriteLine("Z_galv_R_RV: " + Z_galv_R_RV->Text);
		sw2->WriteLine("Z_galv_L: " + Z_galv_L->Text);
		sw2->WriteLine("Z_galv_R: " + Z_galv_R->Text);
		sw2->WriteLine("Z_galv_L_RV: " + Z_galv_L_RV->Text);
		sw2->WriteLine("Z_galv_R_RV: " + Z_galv_R_RV->Text);

		sw->Close();
		sw2->Close();
		Console::WriteLine("a new file ('{0}') has been written", filename);
	}
	private: void parameterfilePrep() {
		cout << "Start Param file prep" << endl;
		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		Logfilename = date;
		Console::WriteLine(date);
		String^ ScanStep = Step->Text;
		String^ dir = Export_Box->Text;
		String^ stem1 = Stem_box1->Text;
		//String^ Xstep1 = (double::Parse(Overlap_um->Text) / 2 * mag).ToString();
		String^ Xstep1 = (74 * PIXELSIZE / MAG).ToString();
		String^ Xstep2 = (86 * PIXELSIZE / MAG).ToString();
		String^ Ystep = (74 * PIXELSIZE / MAG).ToString();
		//String^ Xstep2 = (double::Parse(Overlap_um->Text) / 2 * mag + 12 * 6.5 / mag).ToString();
		//String^ Ystep = (double::Parse(Overlap_um_y->Text) / 2 * mag).ToString();
		Dir_check(dir);
		String^ filename = dir + "\\" + stem1 + "_ScaleMerge_command.txt";
		StreamWriter^ sw = gcnew StreamWriter(filename);
		String^ Dir = dir->Replace("\\\\", "/");
		Dir = Dir->Substring(3);
		sw->WriteLine("ssh GPSi@DS2");
		sw->WriteLine("Pass :uedamacro");
		sw->WriteLine("command: ");
		sw->WriteLine("cd ../../export2/GPSi");
		sw->Write("sudo ./ScaleMerge.out ");
		sw->Write(Dir + "/" + stem1 + " ");
		sw->Write(Dir + "/ScaleMerge/" + stem1 + " --skip 10 --scale 0.3 ");
		sw->Write("--vflip ");
		//sw->Write("--hflip ");

		sw->Write("--width 2304 --height 2304 --overlap ");
		sw->WriteLine(Xstep1 + " " + Xstep2 + " " + Ystep + " " + Ystep);
		sw->WriteLine("XY step; 58 58 70 70");

		sw->Close();


	}

	private: void export_tilepos() {
		DateTime dt = DateTime::Now;
		String^ ZS = Start_Z_pos->Text;
		String^ ZE = End_Z_pos->Text;

		String^ X = X_pos_current->Text;
		X = X->Trim('\r');
		String^ Y = Y_pos_current->Text;
		Y = Y->Trim('\r');
		String^ filename = "initial_path\\LOG\\LOG_" + Logfilename + ".log";

		try {

			String^ dir = Export_Box->Text;
			Dir_check(dir);
			string dir_s;
			MarshalString(dir, dir_s);
			String^ Path = dir + "\\Tile_Position.LOG";
			StreamWriter^ sw = gcnew StreamWriter(filename, "append");
			StreamWriter^ sw2 = gcnew StreamWriter(Path, "append");

			DateTime dt = DateTime::Now;
			String^ time = dt.ToString("H-mm-ss");
			sw->WriteLine("Tile: " + MOVIE_stats.tilenum + "\t: " + X + "," + Y + "," + ZS + "->" + ZE + " finished@ " + time);
			sw2->WriteLine("Tile: " + MOVIE_stats.tilenum + "\t: " + X + "," + Y + "," + ZS + "->" + ZE + " finished@ " + time);

			sw->Close();
			sw2->Close();
			//Console::WriteLine("a new file ('{0}') has been written", filename);
		}
		catch (System::Exception^ e) {
			MessageBox::Show(e->ToString());
		}
	}
	private: void export_multifocus_column(float E) {

		DateTime dt = DateTime::Now;
		String^ ZS = Start_Z_pos->Text;
		String^ ZE = End_Z_pos->Text;

		String^ X = X_pos_current->Text;
		X = X->Trim('\r');
		String^ Y = Y_pos_current->Text;
		Y = Y->Trim('\r');
		String^ filename = "initial_path\\LOG\\LOG_" + Logfilename + ".log";
		String^ dir = Export_Box->Text;
		Dir_check(dir);
		string dir_s;
		MarshalString(dir, dir_s);
		String^ Path = dir + "\\Tile_Position.log";

		StreamWriter^ sw = gcnew StreamWriter(filename, "append");
		StreamWriter^ sw2 = gcnew StreamWriter(Path, "append");

		String^ time = dt.ToString("H-mm-ss");
		sw->WriteLine("ETL: " + E.ToString() + ", Position: " + X + "," + Y + "," + ZS + "->" + ZE + " finished@ " + time);
		sw2->WriteLine("ETL: " + E.ToString() + ", Position: " + X + "," + Y + "," + ZS + "->" + ZE + " finished@ " + time);

		sw->Close();
		sw2->Close();
	}
	private: void setting_reset() {
		Laser1_comb->SelectedIndex = -1;
		Laser2_comb->SelectedIndex = -1;
		Laser3_comb->SelectedIndex = -1;
		Laser4_comb->SelectedIndex = -1;
		filter1_comb->SelectedIndex = -1;
		filter2_comb->SelectedIndex = -1;
		filter3_comb->SelectedIndex = -1;
		filter4_comb->SelectedIndex = -1;
		set1_name->Text = "";
		set2_name->Text = "";
		set3_name->Text = "";
		set4_name->Text = "";
		LP1->Text = "0";
		LP2->Text = "0";
		LP3->Text = "0";
		LP4->Text = "0";
		EXP_time1->Text = "0";
		EXP_time2->Text = "0";
		EXP_time3->Text = "0";
		EXP_time4->Text = "0";
		top_limit->Text = "0";
		bottom_limit->Text = "0";
		left_limit->Text = "0";
		right_limit->Text = "0";
		LF_check1->Checked = false;
		LF_check2->Checked = false;
		LF_check3->Checked = false;
		LF_check4->Checked = false;
		LF_check4->Checked = false;
		multi_laser_check->Checked = false;
	}
	private: void Error_LOG(int X, String^ estr) {
		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		Console::WriteLine(date);

		String^ filename = "initial_path\\LOG\\errorlog.log";
		StreamWriter^ sw = gcnew StreamWriter(filename, "append");
		sw->WriteLine(date + ": error #" + X + ":" + estr);
		sw->Close();
	}
	private: void Running_LOG(String^ estr) {
		DateTime dt = DateTime::Now;
		String^ date = dt.ToString("yyyy-MM-dd-H-mm-ss");
		//Console::WriteLine(date);
		WaitForSingleObject(hMutex, INFINITE);
		String^ filename = "initial_path\\LOG\\runninglog.log";
		StreamWriter^ sw = gcnew StreamWriter(filename, "append");
		//Console::WriteLine(date + ": " + estr);
		sw->WriteLine(date + ": " + estr);
		sw->Close();
		ReleaseMutex(hMutex);
	}
	private: void file_export_DAQ(double data[2000]) {
		FILE* fp;
		char filename[] = "initial_path\\Sheet_log\\sheet.txt";
		cout << filename << endl;
		fopen_s(&fp, filename, "w");
		try {
			for (int i = 0; i < 2000; i++) {
				// cout << data[i] << endl;
				fprintf_s(fp, "%f\n", data[i]);
			}
		}
		catch (exception &e) {

		}
		fclose(fp); //ファイルを閉じる
	}
			 //**********************************************;
			 //Laser control;
			 //***********************************************;

	private:void Laser_ALL_OFF() {
		cout << "Laser_ALL_OFF" << endl;
		Laser_Info.L488.stat_Q = 0;
		Laser_Info.L592.stat_Q = 0;
		Laser_Info.L647.stat_Q = 0;
	}
	private: delegate System::Void Dele_LP488(String ^str);
	private: delegate System::Void Dele_LP592(String ^str);
	private: delegate System::Void Dele_LP647(String ^str);
	private: System::Void Dele_LP488_d(String ^str) {
		L488_box->Text = str;
	}
	private: System::Void Dele_LP592_d(String ^str) {
		L592_box->Text = str;
	}
	private: System::Void Dele_LP647_d(String ^str) {
		L647_box->Text = str;
	}
	private: System::Void L592_ON_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser_Info.L592.stat_Q == 0) {
			Laser_Info.L592.stat_Q = 1;
		}
		else {
			Laser_Info.L592.stat_Q = 0;
		}
	}
	private: System::Void L488_ON_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser_Info.L488.stat_Q == 0) {
			Laser_Info.L488.stat_Q = 1;
		}
		else {
			Laser_Info.L488.stat_Q = 0;
		}
	}
	private: System::Void L647_ON_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser_Info.L647.stat_Q == 0) {
			Laser_Info.L647.stat_Q = 1;
		}
		else {
			Laser_Info.L647.stat_Q = 0;
		}
	}

	private: System::Void L488_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {

		L488_box->Text = String::Concat("", L488_bar->Value);

	}
	private: System::Void L592_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {

		L592_box->Text = String::Concat("", L592_bar->Value);

	}
	private: System::Void L647_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {

		L647_box->Text = String::Concat("", L647_bar->Value);

	}
	private: System::Void L488_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void L647_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void L592_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void L592_bar_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {

	}
	private: System::Void L592_box_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {

	}
	private: System::Void BW_Laser_LR_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}
	private: void Laser_R_ON() {
		illumination_flag = 'R';
		Flipper_Change(TSerials.FilterFlipper_Serial, x10Filterpos, 2);
		ETLParam_vis();
		DataChangeFlag = 1;
	}
	private:void Laser_L_ON() {
		illumination_flag = 'L';
		Flipper_Change(TSerials.FilterFlipper_Serial, x10Filterpos, 1);
		ETLParam_vis();
		DataChangeFlag = 1;
	}
			//**********************************************;
			//Filter control;
			//***********************************************;

	private:void filter_setting() {
		Filter_1->Text = filter_name1->Text;
		FIlter_2->Text = filter_name2->Text;
		FIlter_3->Text = filter_name3->Text;
		FIlter_4->Text = filter_name4->Text;
		FIlter_5->Text = filter_name5->Text;
		FIlter_6->Text = filter_name6->Text;
		FIlter_7->Text = filter_name7->Text;
		FIlter_8->Text = filter_name8->Text;

		filter1_comb->Items->Add(filter_name1->Text);
		filter2_comb->Items->Add(filter_name1->Text);
		filter3_comb->Items->Add(filter_name1->Text);
		filter4_comb->Items->Add(filter_name1->Text);

		filter1_comb->Items->Add(filter_name2->Text);
		filter2_comb->Items->Add(filter_name2->Text);
		filter3_comb->Items->Add(filter_name2->Text);
		filter4_comb->Items->Add(filter_name2->Text);

		filter1_comb->Items->Add(filter_name3->Text);
		filter2_comb->Items->Add(filter_name3->Text);
		filter3_comb->Items->Add(filter_name3->Text);
		filter4_comb->Items->Add(filter_name3->Text);

		filter1_comb->Items->Add(filter_name4->Text);
		filter2_comb->Items->Add(filter_name4->Text);
		filter3_comb->Items->Add(filter_name4->Text);
		filter4_comb->Items->Add(filter_name4->Text);
		filter1_comb->Items->Add(filter_name5->Text);
		filter2_comb->Items->Add(filter_name5->Text);
		filter3_comb->Items->Add(filter_name5->Text);
		filter4_comb->Items->Add(filter_name5->Text);

		filter1_comb->Items->Add(filter_name6->Text);
		filter2_comb->Items->Add(filter_name6->Text);
		filter3_comb->Items->Add(filter_name6->Text);
		filter4_comb->Items->Add(filter_name6->Text);

		filter1_comb->Items->Add(filter_name7->Text);
		filter2_comb->Items->Add(filter_name7->Text);
		filter3_comb->Items->Add(filter_name7->Text);
		filter4_comb->Items->Add(filter_name7->Text);

		filter1_comb->Items->Add(filter_name8->Text);
		filter2_comb->Items->Add(filter_name8->Text);
		filter3_comb->Items->Add(filter_name8->Text);
		filter4_comb->Items->Add(filter_name8->Text);
	}
	private:void filter_move() {
		/*
		filter_flag = 1;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::filter_move_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
		*/
	}

	private:void filter_move_func() {
		FilterWheels^ myFilterWheelManager;
		try {
			myFilterWheelManager = gcnew FilterWheels();
		}
		catch (...) {
			std::cout << "myFilterWheelManager = gcnew FilterWheels(); \n";
		}
		FilterWheel^ HSFW;
		for each (FilterWheel ^ FW in myFilterWheelManager->FilterWheelList) {
			HSFW = FW;
			break;
		}
		//Check for no filterwheels
		if (!HSFW) {
			std::cout << "There are no filter wheels currently attached." << "\n";
		}
		else {
			std::cout << myFilterWheelManager->AttachedDeviceCount << endl;
		}
		//Clear any errors
		HSFW->ClearErrorState();
		//Check for more then one HSFW
		try
		{
			if (myFilterWheelManager->AttachedDeviceCount == 0)
			{
				std::cout << "There are no filter wheels currently attached." << "\n";
			}
			else {
				std::cout << "There number of filter wheels currently attached is: " << myFilterWheelManager->AttachedDeviceCount << "\n";
			}
		}
		catch (...) {
			std::cout << "There are no filter wheels currently attached." << "\n";

		}

		if (HSFW->IsAttached)
		{
			if (filter_num == 0 | filter_num == 1) {
				HSFW->HomeDevice();
				while (HSFW->IsHoming) {
					Sleep(10);
					//wait for home to finish
				}
			}
			else if (filterpos != filter_num) {
				HSFW->CurrentPosition = filter_num;
				while (HSFW->IsMoving)
				{
					// wait for move to complete...
					Sleep(50);
				}
				filterpos = filter_num;
				std::cout << "The current position is: " << HSFW->CurrentPosition << "\n";
			}

		}
		filterpos = filter_num;
		filter_flag = 0;
	}

	private: System::Void Filter_1_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 0;
			filter_move();
		}
	}
	private: System::Void FIlter_2_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 2;
			filter_move();
		}
	}
	private: System::Void FIlter_3_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 3;
			filter_move();
		}
	}
	private: System::Void FIlter_4_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 4;
			filter_move();
		}
	}
	private: System::Void FIlter_5_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 5;
			filter_move();
		}
	}
	private: System::Void FIlter_6_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 6;
			filter_move();
		}
	}
	private: System::Void FIlter_7_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 7;
			filter_move();
		}
	}
	private: System::Void FIlter_8_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 8;
			filter_move();
		}
	}
	private: System::Void Theta_SIGMA_but_Click(System::Object^  sender, System::EventArgs^  e) {
		if (filter_flag == 0) {
			filter_num = 9;
			filter_move();
		}
	}

			 //**********************************************
			 //Focus control
			 //***********************************************
	private: System::Void Focus_up_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_up_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_down_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_down_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_up_F_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_up_F_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_down_F_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void Focus_down_F_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}

	private: System::Void focus_but_set1_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void focus_but_set2_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void focus_but_set3_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void focus_but_set4_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void focus_Zero_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Focus_set_BW_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}
	private: System::Void button1_Click_3(System::Object^  sender, System::EventArgs^  e) {


	}

			 //**********************************************
			 //Laser & Filter check box check
			 //***********************************************
	private: int Laser_Color(int X) {
		if (X == 1) {
			return 488;
		}
		else if (X == 2) {
			return 592;
		}
		else if (X == 3) {
			return 647;
		}
		else {
			return 0;
		}
	}
	private: int Filter_Color(int X) {
		if (X == 1) {
			return 525;
		}
		else if (X == 2) {
			return 561;
		}
		else if (X == 3) {
			return 600;
		}
		else if (X == 4) {
			return 0;
		}
		else if (X == 5) {
			return 625;
		}
		else if (X == 6) {
			return 660;
		}
		else if (X == 7) {
			return 700;
		}
		else if (X == 8) {
			return 0;
		}
	}
	private: int Laser_Filter_count(string &ary) {
		int count = 0;
		if (LF_check1->Checked == true) {
			ary.append("1");
			count++;
		}
		if (LF_check2->Checked == true) {
			ary.append("2");
			count++;
		}
		if (LF_check3->Checked == true) {
			ary.append("3");
			count++;
		}
		if (LF_check4->Checked == true) {
			ary.append("4");
			count++;
		}
		return count;
	}
	private: int Get_Laser_index() {
		if (Laser_Info.L488.stat == 1) {
			cout << "Laser stats: 488 nm ON" << endl;
			return 1;
		}
		if (Laser_Info.L592.stat == 1) {
			cout << "Laser stats: 592 nm ON" << endl;
			return 2;
		}
		if (Laser_Info.L647.stat == 1) {
			cout << "Laser stats: 647 nm ON" << endl;
			return 3;
		}
		else {
			return 0;
		}
	}
	private: void Laser_Filter_check(char X, int &L, int &F, int &P, int &E, int &Fo, String^ &Setname) {
		cout << "Laser_Filter_check  index " << X << endl;
		if (X == '1') {
			L = Laser1_comb->SelectedIndex;
			F = filter1_comb->SelectedIndex;
			P = int::Parse(LP1->Text);
			E = int::Parse(EXP_time1->Text);
			Setname = set1_name->Text;
		}
		else if (X == '2') {
			L = Laser2_comb->SelectedIndex;
			F = filter2_comb->SelectedIndex;
			P = int::Parse(LP2->Text);
			E = int::Parse(EXP_time2->Text);
			Setname = set2_name->Text;
		}

		else if (X == '3') {
			L = Laser3_comb->SelectedIndex;
			F = filter3_comb->SelectedIndex;
			P = int::Parse(LP3->Text);
			E = int::Parse(EXP_time3->Text);
			Setname = set3_name->Text;
		}
		else if (X == '4') {
			L = Laser4_comb->SelectedIndex;
			F = filter4_comb->SelectedIndex;
			P = int::Parse(LP4->Text);
			E = int::Parse(EXP_time4->Text);
			Setname = set4_name->Text;
		}
		else {
			L = 0;
			F = 0;
			P = 0;
			cout << "Laser_Filter_check error!!" << endl;
			cout << "Please push any keys..." << endl;
			getchar();
		}
	}

	private: System::Void MyForm_Scroll(System::Object^  sender, System::Windows::Forms::ScrollEventArgs^  e) {
	}
	private: System::Void Galvano_ON_Click(System::Object^  sender, System::EventArgs^  e) {

	}

	private: System::Void Amplitude_mSPIM_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Amplitude_sheet_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		double ampli = double(Amplitude_sheet_bar->Value) / 10;
		Amplitude_sheet->Text = ampli.ToString();
	}
	private:void Read_S_Pos() {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "RDP1\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void backgroundWorker_Zstage_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
		Sleep(100);
		//Zstage_port->WriteLine(STX + "WSY1/66/2\r");
		Zstage_port->WriteLine(STX + "WSY1/66/2\r");

		Sleep(100);
		Zstage_port->WriteLine(STX + "WSY2/66/7\r");
		MicroStepZ = 5;
		//Zstage_port->WriteLine(STX + "WSY2/66/2\r");
		Sleep(100);
		Zstage_Flag = 0;
		/*stage_stat.s = 1;
		Zstage_port->WriteLine(STX + "ORG1/4/0\r");
		while (stage_stat.s == 1) {
			//Zstage_port->WriteLine(STX + "STR1\r");
			Sleep(50);
		}
		double v = -90 / Theta_deg;
		Theta_MOVE(v);
		*/
		while (1) {

			while (Zstage_Flag == 0) {
				Zstage_port->WriteLine(STX + "RDP2\r");
				Sleep(50);
			}
			while (Zstage_Flag == 10) {
				Zstage_port->WriteLine(STX + "RDP1\r");
				Sleep(50);
			}
			Sleep(500);

		}
	}
	private: void SerialDataRecieved_z(String^ recvData) {
		if (recvData->Length > 5) {
			//Console::WriteLine(recvData->Substring(2));
			String^ Data = recvData->Substring(2);
			if (Data->IndexOf("RDP2") != -1) {
				//String^ zposCurrent = recvData->Substring(6);
				double Z = 0.0;
				if (double::TryParse(Data->Substring(5), Z) == true) {
					if (Zstage_Flag == 0) {
						//Stagepos_current.y = Z / MicroStepZ*2;
						Stagepos_current.y = Z / MicroStepZ;
						Z_pos_current->Text = Stagepos_current.y.ToString("F0");
						Zstage_Flag = 10;
					}
				}
			}
			if (Data->IndexOf("RDP1") != -1) {
				//String^ zposCurrent = recvData->Substring(6);
				double Z = 0.0;
				if (double::TryParse(Data->Substring(5), Z) == true) {
					Stagepos_current.s = Z * Theta_deg;
					s_pos_current->Text = Stagepos_current.s.ToString("F2");
					Zstage_Flag = 0;
				}
			}
			if (Data->IndexOf("STR1") != -1) {
				recvData = recvData->Trim('\r', '\n');
				Console::Write("\r" + recvData);
				int stat;
				if (int::TryParse(Data->Substring(5, 1), stat) == true) {
					stage_stat.s = stat;
					if (stat == 0) {
						Zstage_Flag = 0;
						Sleep(50);
					}
				}
			}
			if (Data->IndexOf("STR2") != -1) {
				recvData = recvData->Trim('\r', '\n');
				Console::Write("\r" + recvData);
				int stat;
				if (int::TryParse(Data->Substring(5, 1), stat) == true) {
					stage_stat.y = stat;
					if (stat == 0) {
						Zstage_Flag = 0;
						Sleep(50);
					}
				}
			}
			if (Data->IndexOf("ORG1") != -1) {
				Console::WriteLine(recvData);
				//recvData = recvData->Trim('\r', '\n');
				//Console::Write("\r" + recvData);
				int stat;
				if (int::TryParse(Data->Substring(3, 1), stat) == true) {
					stage_stat.s = 0;
					Zstage_Flag = 0;
				}
				else {
					Console::Write("Theta Stage ORG error: " + recvData);
				}
				Zstage_Flag = 0;
			}
		}

		Z_res = "";
	}
	private: void SerialDataRecieved_x(String^ recvData) {
		if (recvData == "") {}
		else {
			String^ Sep = recvData->Trim('\r');
			string sh;
			MarshalString(Sep, sh);
			if (check_int(sh) == true) {
				if (Xstage_Flag == 1) {
					X_pos_current->Text = Sep;
					Stagepos_current.x = stod(sh);
					Xstage_Flag = 0;
				}
				else if (Ystage_Flag == 1) {
					Y_pos_current->Text = Sep;
					Stagepos_current.z = stod(sh);
					Ystage_Flag = 0;
				}
			}
		}
		XY_res = "";
	}
	private: void SerialDataRecieved_488(String^ recvData) {
		//Console::WriteLine(recvData);
		if (recvData->IndexOf("13034") != -1) {

			L488_COM_NUM = int::Parse(recvData).ToString();
			Console::WriteLine(L488_COM_NUM);
		}
		else if (recvData->IndexOf("13026") != -1) {
			L488_COM_NUM = int::Parse(recvData).ToString();
			Console::WriteLine(L488_COM_NUM);
		}
		if (Laser_Info.L488.Q == 1) {
			//if (recvData->IndexOf("1") != -1) {
			if (recvData->Contains("1")) {
				Laser_Info.L488.stat = true;
			}
			else if (recvData->Contains("0")) {
				//Console::WriteLine("Laser 488 OFF");
				Laser_Info.L488.stat = false;
			}
		}
		if (Laser_Info.L488.Q == 2) {
			double pw = -1;
			if (double::TryParse(recvData, pw)) {
				pw = pw * 1000;
				Laser_Info.L488.LP_current = pw;
			}
		}
		if (Laser_Info.L488.Q == 8) {
			double t;
			//Console::WriteLine(recvData);
			if (double::TryParse(recvData, t))
				Laser_Info.L488.temp = t;
		}
		Laser_Info.L488.Q = 0;
	}
	private: void SerialDataRecieved_592(String^ recvData2) {
		String^ recvData = "00";
		if (Buf592->Length > 3)
			recvData = Buf592->Substring(3, Buf592->Length - 4);
		//Console::WriteLine(recvData);
		if (Laser_Info.L592.Q == 1) {
			if (recvData->Contains("1")) {
				//Console::WriteLine("Laser 592 ON");
				Laser_Info.L592.stat = true;

			}
			else if (recvData->Contains("0")) {
				//Console::WriteLine("Laser 592 OFF");
				Laser_Info.L592.stat = false;

			}
		}
		if (Laser_Info.L592.Q == 2) {
			double val; if (double::TryParse(recvData, val)) {
				//Console::WriteLine("Power: " + val);
				Laser_Info.L592.LP_current = val;
			}
		}
		if (Laser_Info.L592.Q == 8) {
			double t;
			if (double::TryParse(recvData, t)) {
				Laser_Info.L592.temp = t;
				//Console::WriteLine("temp: " + t);
			}
		}
		Buf592 = "";
		Laser_Info.L592.Q = 0;
	}
	private: void SerialDataRecieved_647(String^ recvData) {

		if (Laser_Info.L647.Q == 1) {
			if (recvData->IndexOf("1") != -1) {
				Laser_Info.L647.stat = true;
			}
			else if (recvData->IndexOf("0") != -1) {
				Laser_Info.L647.stat = false;
			}
			if (recvData->IndexOf("13026") != -1) {
				Laser_Info.L647.COMOpen = true;
				Laser_Info.L647.Serial = to_string(int ::Parse(recvData));
				cout << "COM L647 opened" << endl;
			}

		}
		if (Laser_Info.L647.Q == 2) {
			double pw = -1;
			if (double::TryParse(recvData, pw)) {
				pw = pw * 1000;
				Laser_Info.L647.LP_current = pw;
			}
		}
		if (Laser_Info.L647.Q == 8) {
			double t;
			//Console::WriteLine(recvData);
			if (double::TryParse(recvData, t))
				Laser_Info.L647.temp = t;
		}
		Laser_Info.L647.Q = 0;
	}
	private: void SerialDataRecieved_theta(String^ recvData) {
		//Console::WriteLine(recvData);


		int i;
		if (recvData->IndexOf("A") != -1) {
			filter_flag = 0;
		}
		if (recvData->IndexOf("*") != -1) {
			filter_flag = 1;
		}
		else if (int ::TryParse(recvData, i)) {
			filterpos = i;
			filter_flag = 0;
		}
	}
	private: void SerialDataRecieved_piezo(String^ recvData) {
		Console::WriteLine(recvData);
		if (recvData == "") {
		}
		else if (Piezo_flag == 0) {
			//Piezo_x_current->Text = recvData;
		}
		else if (Piezo_flag == 100) {
			//Piezo_y_current->Text = recvData;
		}

	}
			 /*private: void SerialDataRecieved_F(String^ recvData) {
				 String^ F;
				 int f;
				 if (int::TryParse(recvData, f) == true) {
					 Focus_pos->Text = f.ToString();
					 Focus_Flag = 0;
					 grobal_temp = f;
					 if (f > Focus_Limit2 || f < Focus_Limit) {
						 if (Stage_Cal_Mode->Checked == false) {
							 Focus_BWFlag = 0;
							 Sleep(50);
							 Focus_unit_com->Write("L:\r\n");
							 Console::WriteLine("Focus Limit !!!");
						 }
					 }
				 }
				 COMrec_buf = "";
			 }
			 private: System::Void Focus_unit_com_DataReceived_1(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
				 SerialDataRecievedDelegate_F^ dlgteF = gcnew SerialDataRecievedDelegate_F(this, &MacroScopever1::MyForm::SerialDataRecieved_F);
				 String^ RecievedData = Focus_unit_com->ReadExisting();
				 COMrec_buf = COMrec_buf + RecievedData;
				 if (COMrec_buf->IndexOf("\r\n") != -1) {
					 this->Invoke(dlgteF, COMrec_buf);
				 }
			 }*/
	private: System::Void Zstage_port_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		try {
			SerialDataRecievedDelegate_z^ dlgtez = gcnew SerialDataRecievedDelegate_z(this, &MacroScopever1::MyForm::SerialDataRecieved_z);
			String^ RecievedData = Zstage_port->ReadExisting();
			Z_res = Z_res + RecievedData;
			if (Z_res->IndexOf("\r\n") != -1) {
				//this->Invoke(dlgtez, Z_res);
				this->Invoke(dlgtez, Z_res);
			}
		}
		catch (exception &e) {
			Running_LOG("ZportRecieved:" + Z_res);
		}
		//Console::WriteLine( RecievedData);
		//this->Invoke(dlgtez, RecievedData);

	}
	private: System::Void XYstage_port_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		SerialDataRecievedDelegate_x^ dlgtex = gcnew SerialDataRecievedDelegate_x(this, &MacroScopever1::MyForm::SerialDataRecieved_x);
		String^ RecievedData = XYstage_port->ReadExisting();
		XY_res = XY_res + RecievedData;
		if (XY_res->IndexOf("\r") != -1) {
			this->Invoke(dlgtex, XY_res);
		}

	}
	private: System::Void Laser_488_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		SerialDataRecievedDelegate_488^ dlgte488 = gcnew SerialDataRecievedDelegate_488(this, &MacroScopever1::MyForm::SerialDataRecieved_488);
		String^ RecievedData = Laser_488->ReadExisting();
		this->Invoke(dlgte488, RecievedData);
	}
	private: String ^ Buf592 = "";
	private: System::Void Laser_592_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		String^ RecievedData = Laser_592->ReadExisting();
		Buf592 = Buf592 + RecievedData;
		if (Buf592->Contains("\r")) {
			SerialDataRecievedDelegate_592^ dlgte592 = gcnew SerialDataRecievedDelegate_592(this, &MacroScopever1::MyForm::SerialDataRecieved_592);
			this->Invoke(dlgte592, Buf592);
		}
	}
	private: System::Void Laser_647_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		SerialDataRecievedDelegate_647^ dlgte647 = gcnew SerialDataRecievedDelegate_647(this, &MacroScopever1::MyForm::SerialDataRecieved_647);
		String^ RecievedData = Laser_647->ReadExisting();
		this->Invoke(dlgte647, RecievedData);
	}
	private: System::Void SIGMA_Theta_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		SerialDataRecievedDelegate_theta^ dlgteTheta = gcnew SerialDataRecievedDelegate_theta(this, &MacroScopever1::MyForm::SerialDataRecieved_theta);
		String^ RecievedData = SIGMA_Theta->ReadExisting();
		this->Invoke(dlgteTheta, RecievedData);
	}
	private: System::Void Piezo_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
	}
	private: void SerialDataRecieved_m(String^ recvData) {
		if (recvData == "") {}

	}
	private: System::Void Z_target_pos_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ z = Z_target_pos->Text;
			string Z;
			MarshalString(z, Z);
			if (check_int(Z) == true) {
				float z_Target = float::Parse(Z_target_pos->Text);

				Zstage_Flag = 1;
				Sleep(50);
				int z_Target2 = z_Target * MicroStepZ;
				Zstage_port->WriteLine(STX + "APS2/5/" + z_Target2.ToString() + "/1\r");
				Sleep(500);
				Zstage_Flag = 0;
			}
		}
	}
	private: System::Void Single_Acquire_Click(System::Object^  sender, System::EventArgs^  e) {
		if (camera1_on_flag == 1) {
			LIVE_Acq_flag1 = 1;
		}
		if (camera2_on_flag == 1) {
			LIVE_Acq_flag2 = 1;
		}
	}
	private: System::Void Z_target_pos_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e) {

	}
	private: System::Void Z_target_pos_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void exposure_time_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		exposure_time_box->Text = String::Concat("", exposure_time_bar->Value);
	}
	private: System::Void Z_move_but_Click(System::Object^  sender, System::EventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		float z_Target = float::Parse(Z_target_pos->Text);
		int z_Target2 = z_Target * MicroStepZ;
		Zstage_port->WriteLine(STX + "APS2/5/" + z_Target2.ToString() + "/1\r");
		Sleep(100);
		Zstage_Flag = 0;

	}
	private: System::Void Z_up_Click(System::Object^  sender, System::EventArgs^  e) {
		int pos_z = int::Parse(Z_target_pos->Text);
		pos_z += 5;
		Z_target_pos->Text = pos_z.ToString();
		Zstage_Flag = 1;
		Sleep(50);
		String^ STX = "\x02";
		float z_Target = float::Parse(Z_target_pos->Text);
		int z_Target2 = z_Target * MicroStepZ;
		Zstage_port->WriteLine(STX + "APS2/5/" + z_Target2.ToString() + "/1\r");
		Sleep(500);
		Zstage_Flag = 0;

	}
	private: System::Void Z_down_Click(System::Object^  sender, System::EventArgs^  e) {
		int pos_z = int::Parse(Z_target_pos->Text);
		pos_z -= 5;
		Z_target_pos->Text = pos_z.ToString();
		Zstage_Flag = 1;
		Sleep(50);
		String^ STX = "\x02";
		float z_Target = float::Parse(Z_target_pos->Text);
		int z_Target2 = z_Target * MicroStepZ;
		Zstage_port->WriteLine(STX + "APS2/5/" + z_Target2.ToString() + "/1\r");
		Sleep(500);
		Zstage_Flag = 0;
	}
	private: System::Void saveFileDialog1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) {
	}
	private: System::Void open_file_Click(System::Object^  sender, System::EventArgs^  e) {
		String^ folderName;// 選択されたフォルダのフルパス  
		FolderBrowserDialog ^dlg = gcnew FolderBrowserDialog();
		if (dlg->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			folderName = dlg->SelectedPath;//選択されたフォルダを設定  
			//savefilepath->Text = folderName;
	}


	private:void OpenComs() {
		//Load_COMS();
		cout << "OpenCOMS_" << endl;
		cli::array<String^>^ ports = IO::Ports::SerialPort::GetPortNames();
		for (int i = 0; i < ports->Length; ++i)
			Console::WriteLine(ports[i]);
		/*
		try {
			Laser_488->Open();
		}
		catch (exception& e) {
			try {
				String^ s = Laser_647->PortName;
				String^ s2 = Laser_488->PortName;
				Laser_488->PortName = s;
				Laser_647->PortName = s2;
				Laser_488->Open();
			}
			catch (exception& e) {
				cout << "Error connection Laser 488 " << endl;
			}
		}*/
		try {
			Laser_592->Open();
		}

		catch (exception& e) {
			cout << "Error connection Laser 592 " << endl;
		}/*
		try {
			Laser_647->Open();
		}
		catch (exception& e) {
			cout << "Error connection Laser 647 " << endl;
		}*/
	}
	private:void Abort_ALL() {
		Orca_Stop();

		LIVE_Acq_flag1 = 0;
		Abort_flag = 1;
		LIVE_flag = 0;
		Galvo_flag = 0;
		MOVIE_stats.cancel_flag = 1;
		MOVIE_stats.ExposureFlag = 0;
		Stage_Stop();
		Export_flag1 = 0;
		Export_flag2 = 0;
		Acq_flag = 0;
		filter_flag = 0;
		DAQ_clear();
		DAQ_sheet_endpos();
	}
	private: System::Void Abort_Click(System::Object^  sender, System::EventArgs^  e) {
		Abort_ALL();
	}
	private:void Finalization() {
		AS_flag = 0;
		Laser_ALL_OFF();
		//Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);
		PreScan_frag = 0;
		passed_image_num = total_image_num;
		Abort_flag = 1;
		LIVE_flag = 0;
		multi_mode = 0;
		tile_mode = 0;
		Galvo_flag = 0;
		MOVIE_stats.cancel_flag = 1;
		MOVIE_stats.ExposureFlag = 0;
		DAQ_clear();
		DAQ_sheet_endpos();
		DAQ_reset();
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP1/0\r");
		Sleep(200);
		Export_flag1 = 0;
		Export_flag2 = 0;
		Zstage_Flag = 0;
		delete_MOVIE_param();

		Abort_ALL();
		Console::WriteLine("MOVIE finished !!");
		Running_LOG("MOVIE finished !!");
	}
	private: System::Void Expose_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void End_Z_but_Click(System::Object^  sender, System::EventArgs^  e) {
		float F = float::Parse(Y_pos_current->Text);
		int S = (int)F;
		End_Z_pos->Text = S.ToString();
	}
	private: System::Void Start_Z_but_Click(System::Object^  sender, System::EventArgs^  e) {
		float F = float::Parse(Y_pos_current->Text);
		int S = (int)F;
		Start_Z_pos->Text = S.ToString();
	}
	private: System::Void MyForm_Load(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void top_limit_but_Click(System::Object^  sender, System::EventArgs^  e) {
		top_limit->Text = Y_pos_current->Text;
	}
	private: System::Void left_limit_but_Click(System::Object^  sender, System::EventArgs^  e) {
		bottom_limit->Text = Y_pos_current->Text;
	}
	private: System::Void right_limit_but_Click(System::Object^  sender, System::EventArgs^  e) {
		right_limit->Text = X_pos_current->Text;
	}
	private: System::Void bottom_limit_but_Click(System::Object^  sender, System::EventArgs^  e) {
		bottom_limit->Text = X_pos_current->Text;
	}


	private: void Z_stage_Ini() {
		MOVIE_stats.stage = 1;
		Running_LOG("Z_stage_Initialization: " + MOVIE_stats.ExpStartPos);
		cout << "Z_stage_Initialization: " << MOVIE_stats.ExpStartPos << endl;
		TargetY_box->Text = MOVIE_stats.ExpStartPos.ToString();
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIY:SELSP 5:GOABS " + MOVIE_stats.ExpStartPos.ToString() + "\r");
		Sleep(50);
		XY_stage_flag = 0;
		while (abs(Stagepos_current.z - MOVIE_stats.ExpStartPos) > 1) {
			Sleep(10);
			Zstage_Flag = 0;
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			cout << "\r" << Stagepos_current.z << " / " << MOVIE_stats.ExpStartPos;
		}

		cout << "\r" << "Z_stage_Initialization finished: " << Stagepos_current.z;
		Running_LOG("Z_stage_Initialization finished" + Stagepos_current.z);
		cout << endl;
		MOVIE_stats.stage = 2;
	}


	private: void Z_stage_Ini_forSkip() {
		std::chrono::system_clock::time_point  start, end;
		MOVIE_stats.stage = 1;
		Sleep(100);
		start = std::chrono::system_clock::now();
		while (MOVIE_stats.cancel_flag != 1) {
			if (MOVIE_stats.MOVIE_skip_thread == 2 && MOVIE_stats.camera == 2 && MOVIE_stats.DAQ == 2) {
				cout << "\r" << MOVIE_stats.MOVIE_skip_thread << " , " << MOVIE_stats.camera << " , " << MOVIE_stats.DAQ;
				break;
			}
			Sleep(1);
			end = std::chrono::system_clock::now();
			if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() > 60) {
				MOVIE_stats.cancel_flag = 1;
				Running_LOG("Z_stage_Ini_forSkip timeout!!");
				cout << "Z_stage_Ini_forSkip timeout!!" << endl;
				DAQ_sheet_endpos();
				break;
			}
		}

		cout << endl;
		//cout << "MOVIE Skip Start" << endl;
		XY_stage_flag = 1;
		Sleep(100);
		XYstage_port->Write("AXIY:SELSP 1:GOABS " + MOVIE_stats.SkipEndPos.ToString() + "\r");
		MOVIE_stats.ExposureFlag = 1;
		Sleep(50);
		XY_stage_flag = 0;
		while (abs(Stagepos_current.z - MOVIE_stats.SkipEndPos) > 50) {
			Sleep(1);
			Zstage_Flag = 0;
			if (MOVIE_stats.cancel_flag == 1) {
				MOVIE_stats.ExposureFlag = 0;
				break;
			}
			//cout << "\r" << Stagepos_current.z << " / " << MOVIE_stats.SkipEndPos;
		}
		//cout << endl << "MOVIE SKIP : stage end" << endl;
		while (MOVIE_stats.cancel_flag != 1) {
			//	cout << "MOVIE skip Check: " << MOVIE_stats.MOVIE_skip_thread << " , " << MOVIE_stats.camera << " , " << MOVIE_stats.DAQ << "\r";
			if (MOVIE_stats.MOVIE_skip_thread == 0 && MOVIE_stats.camera == 0 && MOVIE_stats.DAQ == 0)
				break;
			Sleep(10);
		}
		if (MOVIE_stats.skip == true) {
			cout << " MOVIE skip: skip tile !" << endl;
			Running_LOG(" MOVIE skip: skip tile");
		}
		else {
			cout << " MOVIE skip: take tile !" << endl;
			Running_LOG(" MOVIE skip: take tile");
		}
		//Running_LOG("Z_stage_Initialization finished" + Z_start);
		cout << endl;
		MOVIE_stats.stage = 2;
		skipcalc_flag = 0;
	}
	private: void MOVIE_Skip() {
		threads.MOVIE_skip = true;
		MOVIE_stats.MOVIE_skip_thread = 1;
		skipcalc_flag = 1;
		cout << "\nStart MOVIE-skip" << endl;
		Running_LOG("DAQ_sheet_endpos");
		DAQ_sheet_endpos();
		if (MOVIE_stats.FR == 0) {
			MOVIE_stats.SkipEndPos = double::Parse(Start_Z_pos->Text);
			MOVIE_stats.SkipStartPos = double::Parse(End_Z_pos->Text);
		}
		else {
			MOVIE_stats.SkipEndPos = double::Parse(Start_Z_pos_RV->Text);
			MOVIE_stats.SkipStartPos = double::Parse(End_Z_pos_RV->Text);
		}
		Running_LOG("Z_stage_gotoEndpos");
		Z_stage_gotoEndpos();
		double Ztravel = abs(MOVIE_stats.SkipEndPos - MOVIE_stats.SkipStartPos);
		double Zskip = double::Parse(MOVIE_skip_step_box->Text);
		MOVIE_stats.TargetImageNum = Ztravel / Zskip;
		//cout << "MOVIE skip :TargetImageNum = " << MOVIE_stats.TargetImageNum << endl;
		MOVIE_stats.ExposureFlag = 0;
		MOVIE_stats.camera = 0;
		MOVIE_stats.DAQ = 0;
		MOVIE_stats.skip = false;
		Acqisition_CameraThread();
		DAQ_sheet_Axial_Thread_skip();

		Running_LOG("MOVIE_Skip: Thread start");
		Console::WriteLine("MOVIE_Skip: Thread start");
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Z_stage_Ini_forSkip);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
		ThreadStart^ start2 = gcnew ThreadStart(this, &MyForm::MOVIE_Skip_Check_func);
		Thread^ thread2 = gcnew Thread(start2);
		thread2->Start();
		while (MOVIE_stats.MOVIE_skip_thread != 0) {
			Sleep(1);
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
		}
		while (MOVIE_stats.camera != 0) {
			Sleep(1);
			cout << "\rWaiting Camera Thread finish";
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
		}
		Console::WriteLine("MOVIE_Skip: Thread end");
		threads.MOVIE_skip = false;
	}
	private: void InitialAF_SinglePlane() {
		cout << "Initial AF _Start" << endl;
		Running_LOG("Initial AF _Start");
		MOVIE_stats.TargetImageNum = int::Parse(AFnum->Text);

		MOVIE_stats.stage = 2;
		MOVIE_stats.ExposureFlag = 0;
		MOVIE_stats.camera = 0;
		MOVIE_stats.DAQ = 0;
		MOVIE_stats.skip = false;
		Acqisition_CameraThread();
		DAQ_sheet_Axial_Thread_AF();
		MOVIE_IniAF_func();

		MOVIE_stats.ExposureFlag = 0;

	}
	private: void InitialAF() {
		cout << "Initial AF _Start" << endl;
		threads.AF = true;
		MOVIE_stats.cancel_flag = 0;
		Running_LOG("Initial AF _Start");
		int count = 0;
		int direction = 1;
		if ((float::Parse(Start_Z_pos->Text) - float::Parse(End_Z_pos->Text)) > 0) {
			direction = -1;
		}

		if (illumination_flag == 'R') {
			MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R->Text) / 1000;
		}
		if (illumination_flag == 'L') {
			MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L->Text) / 1000;
		}
		TargetY_box->Text = MOVIE_stats.AFPOS_F.ToString();
		MOVIE_stats.TargetImageNum = int::Parse(AFnum->Text);
		MOVIE_stats.AF_sample = false;
		int ret = -1;
		while (1) {
			int Z_pos;
			Sleep(100);
			Z_pos = (MOVIE_stats.AFPOS_F + count * 500 * direction);
			TargetY_box->Text = Z_pos.ToString();
			MOVIE_stats.stage = 1;
			XY_stage_flag = 1;
			Sleep(50);
			XYstage_port->Write("AXIY:SELSP 1:GOABS " + Z_pos.ToString() + "\r");
			//MOVIE_stats.ExposureFlag = 1;
			Sleep(50);
			XY_stage_flag = 0;
			while (abs(Stagepos_current.z - Z_pos) > 1) {
				Sleep(1);
				if (MOVIE_stats.cancel_flag == 1) {
					MOVIE_stats.ExposureFlag = 0;
					break;
				}
				//cout << "\r" << Stagepos_current.z << " / " << Z_pos;
			}
			//MOVIE_stats.TargetImageNum = INITIAL_AF_NUM;
			MOVIE_stats.TargetImageNum = int::Parse(AFnum->Text);
			if (MOVIE_stats.cancel_flag != 1) {
				MOVIE_stats.stage = 2;
				MOVIE_stats.ExposureFlag = 0;
				MOVIE_stats.camera = 0;
				MOVIE_stats.DAQ = 0;
				MOVIE_stats.skip = false;
				Acqisition_CameraThread();
				DAQ_sheet_Axial_Thread_AF();
				ret = MOVIE_IniAF_func();//ret =0 OKn, -1: no sample
			}
			Running_LOG("Initial AF @pos " + count.ToString() + " End");
			count++;
			MOVIE_stats.ExposureFlag = 0;
			if (ret == 0)break;
			if (count > 3)break;//for 1st marmoset
		}
		if (illumination_flag == 'R') {
			cout << "Best Focus Pos R :" << MOVIE_stats.InitialFocus_R << endl;
			Z_offset_current->Text = (MOVIE_stats.InitialFocus_R * 1000).ToString();
		}
		if (illumination_flag == 'L') {
			cout << "Best Focus Pos L :" << MOVIE_stats.InitialFocus_L << endl;
			Z_offset_current->Text = (MOVIE_stats.InitialFocus_L * 1000).ToString();
		}
		Running_LOG("Initial AF _End");
		threads.AF = false;
		cout << "Initial AF _End" << endl;

	}
	private: void Calc_Exp_region(float Start, float End) {
		cout << "Calc_Exp_region Start" << endl;
		float Step_skip = float::Parse(MOVIE_skip_step_box->Text);
		float Steps = float::Parse(Step->Text);
		int TH_area = int::Parse(AF_area_box->Text);
		int AFpos = 0;
		MOVIE_stats.ImagingStartNum = 0;
		MOVIE_stats.ImagingEndNum = MOVIE_stats.TargetImageNum;
		MOVIE_stats.ExpStartPos = Start;
		MOVIE_stats.ExpEndPos = End;

		int direction;
		if (End - Start > 0) {
			direction = 1;
		}
		else {
			direction = -1;
		}

		for (int i = 0; i < MOVIE_skip_size; ++i) {
			if (MOVIE_stats.MOVIE_skip_list[i] > 0) {
				MOVIE_stats.ExpEndPos = End - (i * Step_skip*direction) + Step_skip * direction * 3;
				break;
			}
		}
		for (int i = 0; i < MOVIE_skip_size; ++i) {
			if (MOVIE_stats.MOVIE_skip_list[MOVIE_skip_size - i - 1] > 0) {
				MOVIE_stats.ExpStartPos = Start + i * Step_skip*direction - Step_skip * direction * 3;
				if (direction == 1) {
					if (MOVIE_stats.ExpStartPos < Start) {
						MOVIE_stats.ExpStartPos = Start;
					}
					if (MOVIE_stats.ExpEndPos > End) {
						MOVIE_stats.ExpEndPos = End;
					}
				}
				else {
					if (MOVIE_stats.ExpStartPos > Start) {
						MOVIE_stats.ExpStartPos = Start;
					}
					if (MOVIE_stats.ExpEndPos < End) {
						MOVIE_stats.ExpEndPos = End;
					}
				}
				break;
			}
		}
		MOVIE_stats.AFPOS_F = MOVIE_stats.ExpStartPos;
		for (int i = 0; i < MOVIE_skip_size; ++i) {
			if (MOVIE_stats.MOVIE_skip_list[MOVIE_skip_size - i - 1] > TH_area) {
				MOVIE_stats.AFPOS_F = Start + i * Step_skip*direction;
				if (direction == 1) {
					if (MOVIE_stats.AFPOS_F < Start) {
						MOVIE_stats.AFPOS_F = Start;
					}
					if (MOVIE_stats.AFPOS_F > End) {
						MOVIE_stats.AFPOS_F = End;
					}
				}
				else {
					if (MOVIE_stats.AFPOS_F > Start) {
						MOVIE_stats.AFPOS_F = Start;
					}
					if (MOVIE_stats.AFPOS_F < End) {
						MOVIE_stats.AFPOS_F = End;
					}
				}
				break;
			}
		}
		MOVIE_stats.TargetImageNum = (int)(abs((MOVIE_stats.ExpStartPos - MOVIE_stats.ExpEndPos) / Steps));
		MOVIE_stats.ImagingStartNum = (int)(abs(MOVIE_stats.ExpStartPos - Start) / Steps);
		MOVIE_stats.ImagingEndNum = (int)(abs(MOVIE_stats.ExpEndPos - Start) / Steps);
		cout << "MOVIE_skip_size: " << MOVIE_skip_size << endl;
		cout << "Skip step :" << Step_skip << " um" << endl;
		cout << "Imaging region num: " << MOVIE_stats.ImagingStartNum << " - " << MOVIE_stats.ImagingEndNum << endl;
		cout << "Imaging region pos: " << MOVIE_stats.ExpStartPos << " - " << MOVIE_stats.ExpEndPos << endl;
		cout << "Imaging num : " << MOVIE_stats.TargetImageNum << endl;
		if (MOVIE_stats.TargetImageNum <= 0) {
			MOVIE_stats.TargetImageNum = 0;
			cout << "TargetImageNum error:" << MOVIE_stats.TargetImageNum << endl;
		}
	}

	private: void Calc_Imaging_Region(int t, double zstart, double zend) {
		MOVIE_stats.skip = true;
		double zstep = abs(zstart - zend) / (double)PreScanImageNum;
		double imagezstep = double::Parse(Step->Text);
		double imageratio = zstep / imagezstep;
		int z = PreScanImageNum;
		MOVIE_stats.AFPOS_F = 999999;
		MOVIE_stats.AFPOS_R = 999999;
		//cout << "PreScanImageNum;" << PreScanImageNum << endl;
		for (int i = 0; i < z; ++i) {
			//	cout << "Start pos: "<<i << " : " << MOVIESkipdata[t][i].sample << endl;
			if (MOVIESkipdata[t][i].sample == true) {
				if (zend > zstart) {
					MOVIE_stats.ExpStartPos = zstart + (double)i * zstep - 300;
				}
				else {
					MOVIE_stats.ExpStartPos = zstart - (double)i * zstep + 300;
				}
				MOVIE_stats.skip = false;

				break;
			}
		}
		for (int i = 0; i < z; ++i) {
			//cout << "Star AFt: " << i << ":" << MOVIESkipdata[t][i].AF << endl;
			if (MOVIESkipdata[t][i].AF == true) {
				if (zend > zstart) {
					MOVIE_stats.AFPOS_F = zstart + (double)i * zstep;
				}
				else {
					MOVIE_stats.AFPOS_F = zstart - (double)i * zstep;
				}
				break;
			}
			//MOVIE_stats.AFPOS_F = MOVIE_stats.ExpStartPos;
		}
		for (int i = z; i > 0; --i) {
			//cout << i << ":" << MOVIESkipdata[t][i - 1].sample << endl;
			if (MOVIESkipdata[t][i - 1].sample == true) {
				if (zend > zstart) {
					MOVIE_stats.ExpEndPos = zstart + (double)i * zstep + 300;
				}
				else {
					MOVIE_stats.ExpEndPos = zstart - (double)i * zstep - 300;
				}

				break;
			}

		}
		for (int i = z; i > 0; --i) {
			//cout << i << ":" << MOVIESkipdata[t][i - 1].AF << endl;
			if (MOVIESkipdata[t][i - 1].AF) {
				if (zend > zstart) {
					MOVIE_stats.AFPOS_R = zstart + (double)i * zstep;
				}
				else {
					MOVIE_stats.AFPOS_R = zstart - (double)i * zstep;
				}
				break;
			}
			//MOVIE_stats.AFPOS_R = MOVIE_stats.ExpEndPos;
		}
		if (zend > zstart) {

			if (MOVIE_stats.ExpEndPos > zend)
				MOVIE_stats.ExpEndPos = zend;
			if (MOVIE_stats.ExpStartPos < zstart)
				MOVIE_stats.ExpStartPos = zstart;
		}
		else {
			if (MOVIE_stats.ExpEndPos < zend)
				MOVIE_stats.ExpEndPos = zend;
			if (MOVIE_stats.ExpStartPos > zstart)
				MOVIE_stats.ExpStartPos = zstart;
		}

		if (MOVIE_stats.AFPOS_F == 999999 || MOVIE_stats.AFPOS_R == 999999) {
			int max = 0;
			for (int i = 0; i < z; ++i) {
				if (MOVIESkipdata[t][i].sample > max) {
					max = MOVIESkipdata[t][i].sample;
				}
			}
			MOVIE_stats.AFPOS_F = zstart + (double)max * zstep;
			MOVIE_stats.AFPOS_R = zstart + (double)max * zstep;

		}
		if (zend > zstart) {
			if (MOVIE_stats.AFPOS_F < MOVIE_stats.ExpStartPos)
				MOVIE_stats.AFPOS_F = MOVIE_stats.ExpStartPos;
			if (MOVIE_stats.AFPOS_R > MOVIE_stats.ExpEndPos)
				MOVIE_stats.AFPOS_R = MOVIE_stats.ExpEndPos;
		}
		else {
			if (MOVIE_stats.AFPOS_F > MOVIE_stats.ExpStartPos)
				MOVIE_stats.AFPOS_F = MOVIE_stats.ExpStartPos;
			if (MOVIE_stats.AFPOS_R < MOVIE_stats.ExpEndPos)
				MOVIE_stats.AFPOS_R = MOVIE_stats.ExpEndPos;
		}


		MOVIE_stats.ImagingStartNum = abs(MOVIE_stats.ExpStartPos - zstart) / imagezstep;
		MOVIE_stats.ImagingEndNum = abs(MOVIE_stats.ExpEndPos - zstart) / imagezstep - 1;

		MOVIE_stats.TargetImageNum = abs(MOVIE_stats.ImagingEndNum - MOVIE_stats.ImagingStartNum);
		if (MOVIE_stats.TargetImageNum < 10)
			MOVIE_stats.skip = true;
		//cout << "FR" << MOVIE_stats.MOVIE_FR << endl;
		//cout << "imagen area:" << MOVIE_stats.ExpStartPos << "  -  " << MOVIE_stats.ExpEndPos << endl;
		//cout << "imagen num:" << MOVIE_stats.ImagingStartNum << "  -  " << MOVIE_stats.ImagingEndNum << endl;
		//cout << "AFpos:" << MOVIE_stats.AFPOS_F << "||" << MOVIE_stats.AFPOS_R << endl;
		//cout << "TargetImageNum: " << TargetImageNum << endl << endl;
	}
	private: int Z_stage_Initialization() {
		MOVIE_stats.stage = 1;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Z_stage_Ini);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
		return 0;
	}
	private: void Z_stage_gotoEndpos() {
		MOVIE_stats.stage = 1;
		//BeginInvoke(gcnew Dele_FocusSet(this, &MyForm::Z_stage_Ini),Z_start.ToString());
		TargetY_box->Text = MOVIE_stats.SkipStartPos.ToString();
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIY:SELSP 5:GOABS " + MOVIE_stats.SkipStartPos.ToString() + "\r");
		Console::WriteLine("AXIY:SELSP 5:GOABS " + MOVIE_stats.SkipStartPos.ToString() + "\r");
		Sleep(50);
		XY_stage_flag = 0;
		while (abs(Stagepos_current.z - MOVIE_stats.SkipStartPos) > 1) {
			//while (Z_start != Z_pos_global * 20) {
			cout << "Z stage inititalization :" << Z_pos_global << "/" << MOVIE_stats.SkipStartPos << "\r";
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
		}
		cout << endl;
		Sleep(500);
		cout << "Z stage inititalization end" << endl;
		MOVIE_stats.stage = 0;
	}
	private: void Z_stage_Speed_Setting() {
		float MOVIE_step = float::Parse(Step->Text);
		float MOVIE_step_skip = float::Parse(MOVIE_skip_step_box->Text);
		float Exp = float::Parse(exposure_time_box->Text);
		int stage_speed = MOVIE_step / Exp * (double)1000;
		int stage_speed2 = MOVIE_step_skip / Exp * (double)1000;//for skip
		XY_stage_flag = 1;
		Sleep(50);
		Console::WriteLine("Z Stage speed setting: " + stage_speed.ToString() + " / " + stage_speed2.ToString());

		XYstage_port->Write("AXIY:L0 " + stage_speed.ToString() + "\r");
		Sleep(100);
		XYstage_port->Write("AXIY:F0 " + stage_speed.ToString() + "\r");
		Sleep(100);
		XYstage_port->Write("AXIY:L1 " + stage_speed2.ToString() + "\r");
		Sleep(100);
		XYstage_port->Write("AXIY:F1 " + stage_speed2.ToString() + "\r");
		Sleep(100);
		XY_stage_flag = 0;
	}
			 /*
	private: void Z_stage_Speed_Setting_forSkip() {

		float MOVIE_step = float::Parse(MOVIE_skip_step_box->Text);
		float Exp = float::Parse(exposure_time_box->Text);
		double WINDOWSIZE = double::Parse(windowsize_box->Text);
		int stage_speed = MOVIE_step / ((Exp + Exp / WINDOWSIZE + EXTRA_TIME)) * (double)20000;
		cout << "MOVIE_step:" << MOVIE_step << endl;
		cout << "MOVIE_exp:" << (Exp + Exp / WINDOWSIZE + EXTRA_TIME) << endl;

		Zstage_Flag = 1;
		Sleep(100);
		Running_LOG("Z_stage_Speed_Setting: " + stage_speed.ToString());
		Console::WriteLine("Stage_speed setting@MOVIE-skip:" + stage_speed.ToString());
		Zstage_port->WriteLine(STX + "WTB2/0/" + stage_speed.ToString() + "/" + stage_speed.ToString() + "/1/1\r");
		Sleep(100);
		Zstage_Flag = 0;
	}*/

	private: void MOVIE_Setting() {
		cout << " MOVIE_Setting" << endl;
		acqdata_flag = true;
		//CameraSettingUpdate();
		MOVIE_stats.ExposureFlag = 0;
		MOVIE_stats.camera = 0;
		//Orca_Stop();
		MOVIE_Focus_thread();
		DataExportThread();
		DAQ_sheet_Axial_Thread();
		Acqisition_CameraThread();
		cout << " MOVIE_Setting end" << endl;
	}

	private: int MOVIE(int Z_end) {

		std::chrono::system_clock::time_point  start, end;
		//cout << "Start +MOVIE func" << endl;
		Running_LOG("MOVIE function Start");
		TargetY_box->Text = Z_end.ToString();
		start = std::chrono::system_clock::now();
		while (MOVIE_stats.stage != 2 || MOVIE_stats.camera != 2 || MOVIE_stats.data != 2 || MOVIE_stats.DAQ != 2) {
			//Running_LOG("MOVIE setting waiting @ MOVIE:" + MOVIE_stats.stage + ", " + MOVIE_stats.camera.ToString() + ", " + MOVIE_stats.data.ToString() + ", " + MOVIE_stats.DAQ.ToString() + ", " + MOVIE_stats.External.ToString());
			if (MOVIE_stats.stage == 2 && MOVIE_stats.camera == 2 && MOVIE_stats.data == 2 && MOVIE_stats.DAQ == 2)
				break;
			Sleep(100);
			cout << "\rMOVIE_stats @ MOVIE:" << MOVIE_stats.stage << ", " << MOVIE_stats.camera << ", " << MOVIE_stats.data << ", " << MOVIE_stats.DAQ;
			end = std::chrono::system_clock::now();
			if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() > 20) {
				MOVIE_stats.cancel_flag = 1;
				DAQ_sheet_endpos();
				break;
			}
			if (MOVIE_stats.Skiptile == true)
				break;
		}
		cout << endl;
		if (MOVIE_stats.cancel_flag != 1) {
			XYstage_Flag = 1;
			Sleep(200);
			Console::WriteLine("MOVIE START");
			Running_LOG("MOVIE START@ MOVIE func");
			LIVE_flag = 1;
			MOVIE_stats.ExposureFlag = 1;
			Running_LOG("AXIY:SELSP 0 : GOABS " + Z_end.ToString() + "\r");
			XYstage_port->Write("AXIY:SELSP 0:GOABS " + Z_end.ToString() + "\r");
			//Zstage_port->WriteLine(STX + "APS2/0/" + Z_end.ToString() + "/1\r");
			Sleep(100);
			XYstage_Flag = 0;
			//update_z_galv();
			__try {
				while (abs(Stagepos_current.z - MOVIE_stats.ExpEndPos) > 1) {
					if (MOVIE_stats.cancel_flag == 1) {
						LIVE_flag = 0;
						MOVIE_stats.ExposureFlag = 0;
						Console::WriteLine("MOVIE canceled !");
						return 1;
					}
					if (MOVIE_stats.Skiptile == true) {
						Stage_Stop();
						break;
					}
					//cout << "remaine" << abs(Stagepos_current.z - MOVIE_stats.ExpEndPos) << endl;
				}
				Running_LOG("Zstage Move End");
				MOVIE_stats.stage = 0;
				//MOVIE_stats.ExposureFlag = 0;
				//Sleep(500);
				//Console::WriteLine("Total image #: " + Picnumber_c1);
				return 0;
			}
			__except (1) {
				printf("MOVIE error!!");
				//cout << Picnumber_c2 << "/" << Picnumber_c2_w << endl;
				return 1;
			}
		}
		return 1;
	}




	private: String ^ ExportNamePrep(int flag, int num, int index, int c_num) {
		String^ Dir = Export_Box->Text;
		String^ Stem1 = Stem_box1->Text;
		String^ Stem2 = Stem_box1->Text;
		String^ FP;
		if (flag == 0) {
			if (c_num == 0) {
				FP = Dir + "\\" + Stem1;
			}
			else if (c_num == 0) {
				FP = Dir + "\\" + Stem2;
			}
		}
		else if (flag == 1) {
			if (c_num == 0) {
				if (index == 0)
					FP = Dir + "\\" + set1_name + "_" + Stem1;
				else if (index == 1)
					FP = Dir + "\\" + set2_name + "_" + Stem1;
				else if (index == 1)
					FP = Dir + "\\" + set3_name + "_" + Stem1;
				else if (index == 1)
					FP = Dir + "\\" + set4_name + "_" + Stem1;
			}
			else if (c_num == 1) {
				if (index == 0)
					FP = Dir + "\\" + set1_name + "_" + Stem2;
				else if (index == 1)
					FP = Dir + "\\" + set2_name + "_" + Stem2;
				else if (index == 1)
					FP = Dir + "\\" + set3_name + "_" + Stem2;
				else if (index == 1)
					FP = Dir + "\\" + set4_name + "_" + Stem2;
			}
		}
		return FP;
	}
	private:void MOVIE_param_list_prep() {
		delete_MOVIE_param();
		cout << "MOVIE_param_list_prep: " << MOVIE_stats.MaxImageNum + 100 << endl;
		MOVIE_stats.MOVIE_focus_list = new double[MOVIE_stats.MaxImageNum + 100];
		MOVIE_stats.Stage_pos_list = new double[MOVIE_stats.MaxImageNum + 100];
		MOVIE_stats.MOVIE_skip_list = new double[MOVIE_stats.MaxImageNum + 100];
	}
	private: void delete_MOVIE_param() {
		delete[] MOVIE_stats.MOVIE_focus_list;
		delete[] MOVIE_stats.Stage_pos_list;
		delete[] MOVIE_stats.MOVIE_skip_list;
	}

	private: void Axial_tiling() {
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		MOVIE_stats.dataspeed = 0;
		Acq_flag = 1;
		vector<FW_RV_PosList> DL(1);
		MOVIE_stats.cancel_flag = 0;
		MOVIE_stats.StartTile = int::Parse(ReStart_tile_num_box->Text);
		MOVIE_stats.ExpStartPos = double::Parse(Start_Z_pos->Text);
		MOVIE_stats.ExpEndPos = double::Parse(End_Z_pos->Text);
		MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R->Text) / 1000;
		MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L->Text) / 1000;
		MOVIE_stats.Step = float::Parse(Step->Text);
		MOVIE_stats.TargetImageNum = abs(MOVIE_stats.ExpStartPos - MOVIE_stats.ExpEndPos) / MOVIE_stats.Step;
		MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum;
		cout << "TargetImageNum : " << MOVIE_stats.TargetImageNum << endl;
		MOVIE_param_list_prep();
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "Start");
		MOVIE_stats.ExposureFlag = 0;
		Z_stage_Speed_Setting();
		float Xstep = (2304.0 - 2 * float::Parse(Overlap_um->Text)) * PIXELSIZE / 10;
		float Ystep = (2304.0 - 2 * float::Parse(Overlap_um_y->Text)) * PIXELSIZE / 10;

		int Xstart = float::Parse(left_limit->Text);
		//float Ystart = float::Parse(top_limit->Text);
		float Ystart = float::Parse(bottom_limit->Text);
		int Xend = float::Parse(right_limit->Text);
		//float Yend = float::Parse(bottom_limit->Text);
		float Yend = float::Parse(top_limit->Text);
		int XTile_num = ceil(abs(Xend - Xstart) / Xstep) + 1;
		int YTile_num = ceil(abs(Yend - Ystart) / Ystep) + 1;
		int deltaX = (XTile_num - 1) * Xstep - (Xend - Xstart);
		if (abs(Xend - Xstart) > 0) {
			Xstart = Xstart - deltaX / 2;
			Xend = Xend + deltaX / 2;
		}
		else {
			Xstart = Xstart + deltaX / 2;
			Xend = Xend - deltaX / 2;
			Xstep = Xstep * (-1);
		}
		int deltaY = (YTile_num - 1) * Ystep - (Yend - Ystart);
		//deltaY = 0;//Yを上に合わせるとき

		if (abs(Yend - Ystart) > 0) {
			Ystart = Ystart - deltaY / 2;
			Yend = Yend + deltaY / 2;
		}
		else {
			Ystart = Ystart + deltaY / 2;
			Yend = Yend - deltaY / 2;
			Ystep = Ystep * (-1);
		}

		cout << "deltaX/deltaY: " << deltaX << " / " << deltaY << endl;
		MOVIE_stats.MOVIE_Retry_int = int::Parse(MOVIE_Retry_Box->Text);
		DirList = Dir_Prep(Xstart, Ystart, Xstep, Ystep, XTile_num, YTile_num, deltaX / 2, deltaY / 2);
		parameterfilePrep();
		tile_total = XTile_num * YTile_num;
		Console::WriteLine("Tile total = " + tile_total + " tiles");
		Console::WriteLine("Start tile# " + MOVIE_stats.StartTile);
		MOVIE_stats.AFPOS_F = double::Parse(Start_Z_pos->Text) / 1000;
		MOVIE_stats.AFPOS_R = double::Parse(Start_Z_pos_RV->Text) / 1000;
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		MOVIE_stats.FR = 0;
		if (RV_Imaging_Check->Checked == true)
			MOVIE_stats.FR = 1;

		if (MOVIE_stats.StartTile >= tile_total) {
			MOVIE_stats.FR = 1;
			MOVIE_stats.StartTile = MOVIE_stats.StartTile - tile_total;
			RV_Imaging_Check->Checked = true;
			cout << "Start Reverse @" << MOVIE_stats.StartTile << " tile" << endl;
		}

		for (; MOVIE_stats.FR < 2; ++MOVIE_stats.FR) {
			if (MOVIE_stats.FR == 1) {
				//if (RV_Imaging_Check->Checked == true) {
				//	MOVIE_stats.StartTile = int::Parse(ReStart_tile_num_box->Text);
			//	}
				//else {
				//	break;//only upper half
				//}
				MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R_RV->Text) / 1000;
				MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L_RV->Text) / 1000;

				MOVIE_stats.ExpStartPos = double::Parse(Start_Z_pos_RV->Text);
				MOVIE_stats.ExpEndPos = double::Parse(End_Z_pos_RV->Text);
				MOVIE_stats.TargetImageNum = abs(MOVIE_stats.ExpStartPos - MOVIE_stats.ExpEndPos) / MOVIE_stats.Step;
				MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum;
				Z_stage_Speed_Setting();

			}
			MOVIE_param_list_prep();
			for (int tilenum = MOVIE_stats.StartTile; tilenum < tile_total; tilenum++) {
				DAQ1_clear();
				Sleep(100);
				if (Abort_flag == 1)
					break;
				if (MOVIE_stats.FR == 0) {
					if (DirList[tilenum].FW_LR == 'L') {
						Laser_L_ON();
						Z_offset_current->Text = Z_galv_L->Text;
					}
					else {
						Laser_R_ON();
						Z_offset_current->Text = Z_galv_R->Text;
					}
				}
				else {
					if (DirList[tilenum].RV_LR == 'L') {
						Laser_L_ON();
						Z_offset_current->Text = Z_galv_L_RV->Text;
					}
					else {
						Laser_R_ON();
						Z_offset_current->Text = Z_galv_R_RV->Text;
					}
				}
				MOVIE_stats.tilenum = tilenum;
				if (MOVIE_stats.FR == 0)
					imagingtile->Text = tilenum.ToString();
				if (MOVIE_stats.FR == 1)
					imagingtile->Text = (tilenum + tile_total).ToString();
				MOVIE_stats.retrycount = 0;
			ReTry_Point:
				double cz = double::Parse(Z_pos_current->Text);
				MOVIE_stats.skip = false;
				cout << "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
				Console::WriteLine("tile num ;  " + tilenum + " / " + tile_total * 2 + " start");
				cout << "imagen area:" << MOVIE_stats.ExpStartPos << "  -  " << MOVIE_stats.ExpEndPos << endl;
				Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "/ tile num ;  " + tilenum + " / " + tile_total + " Start");
				if (MOVIE_stats.FR == 0) {
					Stagepos_target.x = DirList[tilenum].FW_xpos;
					Stagepos_target.y = DirList[tilenum].FW_ypos;
				}
				else {
					Stagepos_target.x = DirList[tilenum].RV_xpos;
					Stagepos_target.y = DirList[tilenum].RV_ypos;
				}
				cout << "targetX: " << Stagepos_target.x << endl;
				cout << "targetY: " << Stagepos_target.y << endl;
				Stagepos_target.move_x = true;
				Stagepos_target.move_y = true;
				Stagepos_target.move_z = false;
				Stagepos_target.move_s = false;
				Running_LOG("MOVE_stage_func");
				MOVE_stage_func();
				while (1) {
					if (abs(Stagepos_target.x - double::Parse(X_pos_current->Text)) < 2)
					{
						break;
					}
					Sleep(1);
				}
				Running_LOG("MOVE_stage X OK");
				while (1) {
					if (abs(Stagepos_target.y - double::Parse(Z_pos_current->Text)) < 2)
					{
						break;
					}
					Sleep(1);
				}
				Running_LOG("MOVE_stage Y OK");
				MOVIE_stats.MOVIE_Retry_int = int::Parse(MOVIE_Retry_Box->Text);
				MOVIE_stats.retry = false;
				MOVIE_stats.stage = 0;
				if (MOVIE_stats.retrycount > 2) {
					Running_LOG("MOVIE_stats.retrycount > 2");
					goto SKIP_Point;
				}
				if (MOVIE_stats.FR == 0) {
					MOVIE_stats.InitialFocus_R = double::Parse(Start_Z_pos->Text) / 1000;
					MOVIE_stats.InitialFocus_L = double::Parse(Start_Z_pos_RV->Text) / 1000;
				}
				else if (MOVIE_stats.FR == 1) {
					MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R_RV->Text) / 1000;
					MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L_RV->Text) / 1000;
				}
				int mskip = 0;
				MOVIE_Skip();
				cout << "MOVIE Skip end" << endl;
				Running_LOG("MOVIE Skip end");
				if (MOVIE_stats.FR == 0) {
					Calc_Exp_region(float::Parse(Start_Z_pos->Text), float::Parse(End_Z_pos->Text));
				}
				else {
					Calc_Exp_region(float::Parse(Start_Z_pos_RV->Text), float::Parse(End_Z_pos_RV->Text));
				}
				//while (skipcalc_flag == 1) {
				//	Sleep(100);
				//}
				if (MOVIE_stats.skip == true) {
					DataExport_blank();
					cout << "Skip tile !\n" << endl;
					Running_LOG("MOVIE_stats.skip == true: goto SKIP_Point");
					goto SKIP_Point;
				}
				cout << "imagen num:" << MOVIE_stats.ImagingStartNum << "  -  " << MOVIE_stats.ImagingEndNum << endl;
				cout << "AFpos:" << MOVIE_stats.AFPOS_F << " / " << MOVIE_stats.AFPOS_R << endl;
				//Calc_Exp_region_wiSkip();
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				//(Sleep(1000));

				int targetnum = MOVIE_stats.TargetImageNum;

				InitialAF();
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				Z_stage_Initialization();
				MOVIE_stats.TargetImageNum = targetnum;
				MOVIE_Setting();
				Running_LOG("MOVIE start, MOVIE_stats.stage :" + MOVIE_stats.stage.ToString());
				MOVIE_stats.stage = MOVIE(MOVIE_stats.ExpEndPos);
				Running_LOG("MOVIE End check start");
				while (MOVIE_stats.cancel_flag != 1) {
					if (MOVIE_stats.camera == 0 && MOVIE_stats.data == 0 && MOVIE_stats.DAQ == 0) {
						MOVIE_stats.ExposureFlag = 0;
						Running_LOG("MOVIE End check finish");
						break;
					}
					Sleep(50);
					cout << "\rMOVIE End check" << MOVIE_stats.camera << MOVIE_stats.data << MOVIE_stats.DAQ;
				}
				cout << endl;
				acqdata_flag = false;
				if (MOVIE_stats.Abort == 1) {
					MOVIE_stats.cancel_flag = 0;
					goto ReTry_Point;
				}
				if (MOVIE_stats.dataspeed != 0) {
					//MOVIE_stats.cancel_flag = 0;
					start = std::chrono::system_clock::now();
					end = std::chrono::system_clock::now();
					double duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
					while (1) {
						Sleep(1000);
						end = std::chrono::system_clock::now();
						duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
						cout << "\r  ReTry dataspeed error waiting : " << duration << "s";
						Running_LOG("ReTry dataspeed error waiting : " + duration.ToString() + " s");
						if (duration > 180)
							break;
					}
					goto ReTry_Point;
				}
				if (MOVIE_stats.cancel_flag == 1) {
					break;
				}
				if (MOVIE_stats.retry == true) {
					MOVIE_stats.retrycount++;
					cout << "\n\n  ReTry !! count: " << MOVIE_stats.retrycount << "/ 4  \n\n";
					goto ReTry_Point;
				}
				if (singlestackmode->Checked == true) {
					MOVIE_stats.cancel_flag = 1;
					break;
				}
			SKIP_Point:

				MOVIE_stats.skip == false;
				//Running_LOG("Next tile");
				//Console::WriteLine("next tile");
				if (MOVIE_stats.cancel_flag == 1) {
					//Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);
					break;
				}
				export_tilepos();
			}
			if (MOVIE_stats.cancel_flag == 0) {
				if (MOVIE_stats.FR == 0) {
					if (OneSideImaging->Checked == false) {
						double s1 = Stagepos_current.s;
						MOVIE_stats.StartTile = 0;
						Reverse_func();
					}
					else {
						break;
					}
				}
			}
			if (MOVIE_stats.cancel_flag == 1) {
				//Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);
				break;
			}

		}
		Finalization();
		Console::WriteLine("Imaging Finished");
		DirList.clear();
		DirList.resize(0);
		MOVIE_Thread_flag = 0;
		end = std::chrono::system_clock::now();
		cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() / 60 << "min\n";
		threads.MOVIE_Thread = false;
	}

	private: void Laser_setting(int laserID, String^ LP)
	{
		threads.Laser = true;
		if (laserID == 1) {
			L488_box->Text = LP;
			Laser_Info.L488.LP = float::Parse(LP);
			Laser_Info.L488.stat_Q = 1;
			Laser_Info.L592.stat_Q = 0;
			Laser_Info.L647.stat_Q = 0;
			while (1) {
				if (abs(Laser_Info.L488.LP_current - Laser_Info.L488.LP) < 2)
					break;
				Sleep(500);
			}
		}
		else if (laserID == 2) {
			L592_box->Text = LP;
			Laser_Info.L592.LP = float::Parse(LP);
			Laser_Info.L488.stat_Q = 0;
			Laser_Info.L592.stat_Q = 1;
			Laser_Info.L647.stat_Q = 0;
			while (1) {
				if (abs(Laser_Info.L592.LP_current - Laser_Info.L592.LP) < 2)
					break;
				Sleep(500);
			}

		}
		else if (laserID == 3) {
			L647_box->Text = LP;
			Laser_Info.L647.LP = float::Parse(LP);
			Laser_Info.L488.stat_Q = 0;
			Laser_Info.L592.stat_Q = 0;
			Laser_Info.L647.stat_Q = 1;
			while (1) {
				if (abs(Laser_Info.L647.LP_current - Laser_Info.L647.LP) < 2)
					break;
				Sleep(500);
			}
		}
		threads.Laser = false;
	}

	private: void Laser_filter_check(int mc) {
		if (mc == 0) {

			filter_num = filter1_comb->SelectedIndex + 1;
			filter_move();
			exposure_time_box->Text = EXP_time1->Text;

			Laser_setting(Laser1_comb->SelectedIndex, LP1->Text);

		}
		if (mc == 1) {
			filter_num = filter2_comb->SelectedIndex + 1;
			filter_move();
			exposure_time_box->Text = EXP_time2->Text;
			Laser_setting(Laser2_comb->SelectedIndex, LP2->Text);
		}
		if (mc == 2) {
			filter_num = filter3_comb->SelectedIndex + 1;
			filter_move();
			exposure_time_box->Text = EXP_time3->Text;
			Laser_setting(Laser3_comb->SelectedIndex, LP3->Text);
		}

	}
	private: System::Void BW_Cursor_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		Sleep(100);
		while (1) {
			exp->Text = MOVIE_stats.ExpStartPos.ToString();
			exp2->Text = MOVIE_stats.ExpEndPos.ToString();
			Target_img->Text = MOVIE_stats.TargetImageNum.ToString();
			Acq_img->Text = Picnumber_c1.ToString();
			Sleep(100);
			if (Abort_flag == 1)
				break;
		}
	}
	private: void Axial_tiling_multi() {
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		MOVIE_stats.dataspeed = 0;
		MOVIE_stats.cancel_flag = 0;
		vector<FW_RV_PosList> DL(1);
		MOVIE_stats.StartTile = int::Parse(ReStart_tile_num_box->Text);
		MOVIE_stats.ExpStartPos = double::Parse(Start_Z_pos->Text);
		MOVIE_stats.ExpEndPos = double::Parse(End_Z_pos->Text);
		MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R->Text) / 1000;
		MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L->Text) / 1000;
		MOVIE_stats.Step = float::Parse(Step->Text);
		MOVIE_stats.TargetImageNum_RV = abs(double::Parse(Start_Z_pos_RV->Text) - double::Parse(End_Z_pos_RV->Text)) / MOVIE_stats.Step;
		MOVIE_stats.TargetImageNum_FR = abs(MOVIE_stats.ExpStartPos - MOVIE_stats.ExpEndPos) / MOVIE_stats.Step;
		BW_Cursor->RunWorkerAsync();
		if (MOVIE_stats.TargetImageNum_RV > MOVIE_stats.TargetImageNum_FR)
			MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum_RV;
		else
			MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum_FR;

		cout << "TargetImageNum max : " << MOVIE_stats.MaxImageNum << endl;
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "Start");
		MOVIE_stats.ExposureFlag = 0;


		float Xstep = (2304.0 - 2 * float::Parse(Overlap_um->Text)) * PIXELSIZE / 10;
		float Ystep = (2304.0 - 2 * float::Parse(Overlap_um_y->Text)) * PIXELSIZE / 10;

		int Xstart = float::Parse(left_limit->Text);
		float Ystart = float::Parse(bottom_limit->Text);
		int Xend = float::Parse(right_limit->Text);
		float Yend = float::Parse(top_limit->Text);
		int XTile_num = ceil(abs(Xend - Xstart) / Xstep) + 1;
		int YTile_num = ceil(abs(Yend - Ystart) / Ystep) + 1;
		int deltaX = (XTile_num - 1) * Xstep - (Xend - Xstart);
		if (abs(Xend - Xstart) > 0) {
			Xstart = Xstart - deltaX / 2;
			Xend = Xend + deltaX / 2;
		}
		else {
			Xstart = Xstart + deltaX / 2;
			Xend = Xend - deltaX / 2;
			Xstep = Xstep * (-1);
		}
		int deltaY = (YTile_num - 1) * Ystep - (Yend - Ystart);
		//deltaY = 0;//Yを上に合わせるとき

		if (abs(Yend - Ystart) > 0) {
			Ystart = Ystart - deltaY / 2;
			Yend = Yend + deltaY / 2;
		}
		else {
			Ystart = Ystart + deltaY / 2;
			Yend = Yend - deltaY / 2;
			Ystep = Ystep * (-1);
		}

		cout << "deltaX/deltaY: " << deltaX << " / " << deltaY << endl;
		MOVIE_stats.MOVIE_Retry_int = int::Parse(MOVIE_Retry_Box->Text);
		//DirList = Dir_Prep(Xstart, Ystart, Xstep, Ystep, XTile_num, YTile_num, deltaX / 2, deltaY / 2);
		parameterfilePrep();
		tile_total = XTile_num * YTile_num;
		Console::WriteLine("Tile total = " + tile_total + " tiles");
		Console::WriteLine("Start tile# " + MOVIE_stats.StartTile);
		MOVIE_stats.AFPOS_F = double::Parse(Start_Z_pos->Text) / 1000;
		MOVIE_stats.AFPOS_R = double::Parse(Start_Z_pos_RV->Text) / 1000;
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		MOVIE_stats.FR = 0;
		if (RV_Imaging_Check->Checked == true)
			MOVIE_stats.FR = 1;

		double spos = double::Parse(s_pos_current->Text);
		if (abs(spos) > 1 && MOVIE_stats.FR == 0) {
			s_pos_target->Text = "0";
			Rotation_func(0);
		}
		else if (1 > abs(spos - float::Parse(ReverseSpos->Text)) && MOVIE_stats.FR == 1) {
			Rotation_func(float::Parse(ReverseSpos->Text));
		}

		if (MOVIE_stats.StartTile >= tile_total) {
			MOVIE_stats.FR = 1;
			MOVIE_stats.StartTile = MOVIE_stats.StartTile - tile_total;
			RV_Imaging_Check->Checked = true;
			cout << "Start Reverse @" << MOVIE_stats.StartTile << " tile" << endl;
			if (1 > abs(spos - float::Parse(ReverseSpos->Text)) && MOVIE_stats.FR == 1) {
				Rotation_func(float::Parse(ReverseSpos->Text));
			}
		}
		Z_stage_Speed_Setting();
		for (; MOVIE_stats.FR < 2; ++MOVIE_stats.FR) {
			if (MOVIE_stats.FR == 0) {
				MOVIE_stats.TargetImageNum = MOVIE_stats.TargetImageNum_FR;
				MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum_FR;
			}
			else if (MOVIE_stats.FR == 1) {
				MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R_RV->Text) / 1000;
				MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L_RV->Text) / 1000;
				MOVIE_stats.ExpStartPos = double::Parse(Start_Z_pos_RV->Text);
				MOVIE_stats.ExpEndPos = double::Parse(End_Z_pos_RV->Text);
				MOVIE_stats.TargetImageNum = MOVIE_stats.TargetImageNum_RV;
				MOVIE_stats.MaxImageNum = MOVIE_stats.TargetImageNum_RV;
			}
			MOVIE_param_list_prep();
			for (int tilenum = MOVIE_stats.StartTile; tilenum < tile_total; tilenum++) {
				MOVIE_stats.tilenum = tilenum;
				MOVIE_stats.skip = false;
				MOVIE_stats.Skiptile = false;
				int mskip = 0;
				for (int mc = 0; mc < 3; ++mc) {

					if (MOVIE_stats.skip == true) {
						DataExport_blank();
						break;
					}
					if (Abort_flag == 1)
						break;
					multicolorcount = mc;
					cout << "******************************************************************" << endl;
					cout << "Start tile :" << tilenum << " , color " << mc << endl;
					if (MOVIE_stats.cancel_flag == 1) {
						break;
					}
					if (mc == 0)
						if (LF_check1->Checked == false)
							continue;
					if (mc == 1)
						if (LF_check2->Checked == false)
							continue;
					if (mc == 2)
						if (LF_check3->Checked == false)
							continue;
					Laser_filter_check(mc);
					DAQ_clear();

					DirList = Dir_Prep_mc(mc, Xstart, Ystart, Xstep, Ystep, XTile_num, YTile_num, deltaX / 2, deltaY / 2);

					if (Abort_flag == 1)
						break;
					if (MOVIE_stats.FR == 0) {
						imagingtile->Text = tilenum.ToString();
						if (singlestackmode->Checked == false) {
							if (DirList[tilenum].FW_LR == 'L') {
								Laser_L_ON();
								Z_offset_current->Text = Z_galv_L->Text;
							}
							else {
								Laser_R_ON();
								Z_offset_current->Text = Z_galv_R->Text;
							}
						}
					}
					else {
						imagingtile->Text = (tilenum + tile_total).ToString();
						if (singlestackmode->Checked == false) {
							if (DirList[tilenum].RV_LR == 'L') {
								Laser_L_ON();
								Z_offset_current->Text = Z_galv_L_RV->Text;
							}
							else {
								Laser_R_ON();
								Z_offset_current->Text = Z_galv_R_RV->Text;
							}
						}
						else {
							if (illumination_flag == 'L') {
								if (MOVIE_stats.FR == 1)
									Z_offset_current->Text = Z_galv_L_RV->Text;
								else
									Z_offset_current->Text = Z_galv_L->Text;
							}
							else if (illumination_flag == 'R') {
								if (MOVIE_stats.FR == 1)
									Z_offset_current->Text = Z_galv_R_RV->Text;
								else
									Z_offset_current->Text = Z_galv_R->Text;
							}

						}
					}
					Z_stage_Speed_Setting();
					CameraSettingUpdate();
					MOVIE_stats.retrycount = 0;
				ReTry_Point:
					double cz = double::Parse(Z_pos_current->Text);
					cout << "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
					Console::WriteLine("tile num ;  " + tilenum + " / " + tile_total * 2 + " start");
					cout << "imagen area:" << MOVIE_stats.ExpStartPos << "  -  " << MOVIE_stats.ExpEndPos << endl;
					Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "/ tile num ;  " + tilenum + " / " + tile_total + " Start");
					if (MOVIE_stats.FR == 0) {
						Stagepos_target.x = DirList[tilenum].FW_xpos;
						Stagepos_target.y = DirList[tilenum].FW_ypos;

						if (mc != 0)
							Stagepos_target.z = double::Parse(Start_Z_pos->Text) / 1000;
					}
					else {
						Stagepos_target.x = DirList[tilenum].RV_xpos;
						Stagepos_target.y = DirList[tilenum].RV_ypos;
						if (mc != 0)
							Stagepos_target.z = double::Parse(Start_Z_pos_RV->Text) / 1000;
					}
					cout << "targetX: " << Stagepos_target.x << endl;
					cout << "targetY: " << Stagepos_target.y << endl;

					Stagepos_target.move_x = true;
					Stagepos_target.move_y = true;
					Stagepos_target.move_z = false;
					Stagepos_target.move_s = false;
					if (mc != 0) {
						Stagepos_target.z = MOVIE_stats.ExpStartPos;
						Stagepos_target.move_z = true;
					}
					Running_LOG("MOVE_stage_func");
					MOVE_stage_func();
					while (1) {
						if (abs(Stagepos_target.x - double::Parse(X_pos_current->Text)) < 2)
						{
							break;
						}
						Sleep(5);
					}
					Running_LOG("MOVE_stage X OK");
					while (1) {
						if (abs(Stagepos_target.y - double::Parse(Z_pos_current->Text)) < 2)
						{
							break;
						}
						Sleep(5);
					}
					if (mc != 0) {
						while (1) {
							if (abs(Stagepos_target.z - double::Parse(Y_pos_current->Text)) < 2)
							{
								break;
							}
							Sleep(5);
						}
					}
					Running_LOG("MOVE_stages OK");
					MOVIE_stats.MOVIE_Retry_int = int::Parse(MOVIE_Retry_Box->Text);
					MOVIE_stats.retry = false;
					MOVIE_stats.stage = 0;
					if (MOVIE_stats.retrycount > 2) {
						Running_LOG("MOVIE_stats.retrycount > 2");
						goto SKIP_Point;
					}
					if (MOVIE_stats.FR == 0) {
						MOVIE_stats.InitialFocus_R = double::Parse(Start_Z_pos->Text) / 1000;
						MOVIE_stats.InitialFocus_L = double::Parse(Start_Z_pos_RV->Text) / 1000;
					}
					else if (MOVIE_stats.FR == 1) {
						MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R_RV->Text) / 1000;
						MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L_RV->Text) / 1000;
					}

					if (mc == 0) {
						MOVIE_Skip();
						cout << "MOVIE Skip end" << endl;
						Running_LOG("MOVIE Skip end");
						if (MOVIE_stats.FR == 0) {
							Calc_Exp_region(float::Parse(Start_Z_pos->Text), float::Parse(End_Z_pos->Text));
						}
						else {
							Calc_Exp_region(float::Parse(Start_Z_pos_RV->Text), float::Parse(End_Z_pos_RV->Text));
						}
					}
					if (MOVIE_stats.skip == true) {
						DataExport_blank();
						cout << "Skip tile !\n" << endl;
						Running_LOG("MOVIE_stats.skip == true: goto SKIP_Point");
						goto SKIP_Point;
					}
					cout << "imagen num:" << MOVIE_stats.ImagingStartNum << "  -  " << MOVIE_stats.ImagingEndNum << endl;
					cout << "AFpos:" << MOVIE_stats.AFPOS_F << " / " << MOVIE_stats.AFPOS_R << endl;
					//Calc_Exp_region_wiSkip();
					if (MOVIE_stats.cancel_flag == 1) {
						break;
					}
					//(Sleep(1000));

					int targetnum = MOVIE_stats.TargetImageNum;
					if (mc == 0) {
						InitialAF();
					}
					Z_stage_Initialization();
					if (MOVIE_stats.cancel_flag == 1) {
						break;
					}

					MOVIE_stats.TargetImageNum = targetnum;
					MOVIE_Setting();
					Running_LOG("MOVIE start, MOVIE_stats.stage :" + MOVIE_stats.stage.ToString());

					MOVIE_stats.stage = MOVIE(MOVIE_stats.ExpEndPos);
					Running_LOG("MOVIE End check start");
					while (MOVIE_stats.cancel_flag != 1) {
						if (MOVIE_stats.camera == 0 && MOVIE_stats.data == 0 && MOVIE_stats.DAQ == 0) {
							MOVIE_stats.ExposureFlag = 0;
							Running_LOG("MOVIE End check finish");
							break;
						}
						if (MOVIE_stats.Skiptile == true)
							break;
						Sleep(50);
						cout << "\rMOVIE End check: camera,data,DAQ : " << MOVIE_stats.camera << MOVIE_stats.data << MOVIE_stats.DAQ;
					}
					cout << endl;
					acqdata_flag = false;
					if (MOVIE_stats.Abort == 1) {
						MOVIE_stats.cancel_flag = 0;
						goto ReTry_Point;
					}
					if (MOVIE_stats.dataspeed != 0) {
						//MOVIE_stats.cancel_flag = 0;
						start = std::chrono::system_clock::now();
						end = std::chrono::system_clock::now();
						double duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
						while (1) {
							Sleep(1000);
							end = std::chrono::system_clock::now();
							duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
							cout << "\r  ReTry dataspeed error waiting : " << duration << "s";
							Running_LOG("ReTry dataspeed error waiting : " + duration.ToString() + " s");
							if (duration > 180)
								break;
							if (MOVIE_stats.Skiptile == true)
								break;
						}
						goto ReTry_Point;
					}
					if (MOVIE_stats.cancel_flag == 1) {
						break;
					}
					if (MOVIE_stats.retry == true) {
						MOVIE_stats.retrycount++;
						cout << "\n\n  ReTry !! count: " << MOVIE_stats.retrycount << "/ 4  \n\n";
						goto ReTry_Point;
					SKIP_Point:
						if (MOVIE_stats.cancel_flag == 1) {
							break;
						}
					}
					export_tilepos();
				}
			}
			if (singlestackmode->Checked == true) {
				MOVIE_stats.cancel_flag = 1;
				break;
			}
			if (MOVIE_stats.cancel_flag == 0) {
				if (MOVIE_stats.FR == 0) {
					if (OneSideImaging->Checked == false) {
						double s1 = Stagepos_current.s;
						MOVIE_stats.StartTile = 0;
						Rotation_func(float::Parse(ReverseSpos->Text));
					}
					else {
						break;
					}
				}
			}
			if (MOVIE_stats.cancel_flag == 1) {
				//Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);
				break;
			}
		}
		Finalization();
		Console::WriteLine("Imaging Finished");
		BW_Cursor->CancelAsync();
		DirList.clear();
		DirList.resize(0);
		MOVIE_Thread_flag = 0;
		end = std::chrono::system_clock::now();
		cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() / 60 << "min\n";
		threads.MOVIE_Thread = false;
	}
	private: System::Void LIVE_but_Click(System::Object^  sender, System::EventArgs^  e) {
		Abort_flag = 0;
		if (Laser_Info.L488.stat == 0 && Laser_Info.L592.stat == 0 && Laser_Info.L647.stat == 0) {
			cout << "Laser OFF" << endl;
		}
		if (LIVE_flag == 0) {
			ScanSpeed_Calc();
			if (Acq_flag == 0) {
				LIVE_flag = 1;
				cout << "LIVE 1" << endl;
				if (Camera_1_use->Checked == true) {
					if (LIVE_window1 == 0) {
						LIVE_Thread();
						LIVE_window1 = 1;
					}
				}
				DAQ_sheetLIVE_Thread();
				doLoopOfAcquisitionThread();
			}
			else {
				cout << "LIVE 2" << endl;
				LIVE_flag = 1;
				if (Camera_1_use->Checked == true) {
					if (LIVE_window1 == 0) {
						LIVE_Thread();
						LIVE_window1 = 1;
					}
				}
			}
		}

	}


	private: void SinglePlane_Axial()
	{
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now();
		MOVIE_stats.dataspeed = 0;
		Acq_flag = 1;
		vector<FW_RV_PosList> DL(1);
		Console::WriteLine("Start Single stack  @axial mode");
		MOVIE_stats.cancel_flag = 0;
		MOVIE_stats.StartTile = int::Parse(ReStart_tile_num_box->Text);
		MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R->Text) / 1000 + 3;
		MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L->Text) / 1000 + 3;
		MOVIE_stats.TargetImageNum = 1;
		MOVIE_param_list_prep();
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "Start");
		MOVIE_stats.ExposureFlag = 0;
		//MOVIE_stats.stage = Z_stage_Initialization();
		if (Camera_1_use->Checked == true) {
			CameraSettings_Axial(Camera1.Hndl);
			CameraSettings_Axial(Camera1.Hndl);
		}
		if (Camera_2_use->Checked == true) {
			CameraSettings_Axial(Camera2.Hndl);
			CameraSettings_Axial(Camera2.Hndl);
		}
		//MOVIE_stats.tilenum = MOVIE_stats.StartTile;
		float Xstep = (2304.0 - 2 * float::Parse(Overlap_um->Text)) * PIXELSIZE / 10;
		float Ystep = (2304.0 - 2 * float::Parse(Overlap_um_y->Text)) * PIXELSIZE / 10;

		int Xstart = float::Parse(left_limit->Text);
		//float Ystart = float::Parse(top_limit->Text);
		float Ystart = float::Parse(bottom_limit->Text);
		int Xend = float::Parse(right_limit->Text);
		//float Yend = float::Parse(bottom_limit->Text);
		float Yend = float::Parse(top_limit->Text);
		int XTile_num = ceil(abs(Xend - Xstart) / Xstep) + 1;
		int YTile_num = ceil(abs(Yend - Ystart) / Ystep) + 1;
		int deltaX = (XTile_num - 1) * Xstep - (Xend - Xstart);
		if (abs(Xend - Xstart) > 0) {
			Xstart = Xstart - deltaX / 2;
			Xend = Xend + deltaX / 2;
		}
		else {
			Xstart = Xstart + deltaX / 2;
			Xend = Xend - deltaX / 2;
			Xstep = Xstep * (-1);
		}
		int deltaY = (YTile_num - 1) * Ystep - (Yend - Ystart);
		//deltaY = 0;//Yを上に合わせるとき

		if (abs(Yend - Ystart) > 0) {
			Ystart = Ystart - deltaY / 2;
			Yend = Yend + deltaY / 2;
		}
		else {
			Ystart = Ystart + deltaY / 2;
			Yend = Yend - deltaY / 2;
			Ystep = Ystep * (-1);
		}

		cout << "deltaX/deltaY: " << deltaX << " / " << deltaY << endl;
		DirList = Dir_Prep(Xstart, Ystart, Xstep, Ystep, XTile_num, YTile_num, deltaX / 2, deltaY / 2);
		parameterfilePrep();
		tile_total = XTile_num * YTile_num;

		Console::WriteLine("Tile total = " + tile_total + " tiles");
		Console::WriteLine("Start tile# " + MOVIE_stats.StartTile);
		MOVIE_stats.AFPOS_F = double::Parse(Start_Z_pos->Text) / 1000;
		MOVIE_stats.AFPOS_R = double::Parse(Start_Z_pos_RV->Text) / 1000;
		MOVIE_stats.passed_image_num1 = 0;
		MOVIE_stats.passed_image_num2 = 0;
		for (int tilenum = MOVIE_stats.StartTile; tilenum < tile_total; tilenum++) {
			DAQ_clear();
			Sleep(100);
			if (Abort_flag == 1)
				break;
			if (MOVIE_stats.FR == 0) {
				if (DirList[tilenum].FW_LR == 'L') {
					Laser_L_ON();
					Z_offset_current->Text = Z_galv_L->Text;
				}
				else {
					Laser_R_ON();
					Z_offset_current->Text = Z_galv_R->Text;
				}
			}
			else {
				if (DirList[tilenum].RV_LR == 'L') {
					Laser_L_ON();
					Z_offset_current->Text = Z_galv_L_RV->Text;
				}
				else {
					Laser_R_ON();
					Z_offset_current->Text = Z_galv_R_RV->Text;
				}
			}
			MOVIE_stats.tilenum = tilenum;
			imagingtile->Text = tilenum.ToString();
			MOVIE_stats.retrycount = 0;
		ReTry_Point:
			cout << "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
			Console::WriteLine("tile num ;  " + tilenum + " / " + tile_total + " start");
			cout << "imagen area:" << MOVIE_stats.ExpStartPos << "  -  " << MOVIE_stats.ExpEndPos << endl;
			Running_LOG(LF_set_count + MOVIE_stats.LF_Set_count.ToString() + "/ tile num ;  " + tilenum + " / " + tile_total + " Start");

			Stagepos_target.x = DirList[tilenum].FW_xpos;
			Stagepos_target.y = DirList[tilenum].FW_ypos;


			cout << "targetX: " << Stagepos_target.x << endl;
			cout << "targetY: " << Stagepos_target.y << endl;
			Stagepos_target.move_x = true;
			Stagepos_target.move_y = true;
			Stagepos_target.move_z = false;
			Stagepos_target.move_s = false;
			Running_LOG("MOVE_stage_func");
			MOVE_stage_func();
			while (1) {
				if (abs(Stagepos_target.x - double::Parse(X_pos_current->Text)) < 2)
				{
					break;
				}
				Sleep(1);
			}
			Running_LOG("MOVE_stage X OK");
			while (1) {
				if (abs(Stagepos_target.y - double::Parse(Z_pos_current->Text)) < 2)
				{
					break;
				}
				Sleep(1);
			}
			Running_LOG("MOVE_stage Y OK");
			MOVIE_stats.MOVIE_Retry_int = int::Parse(MOVIE_Retry_Box->Text);
			MOVIE_stats.retry = false;
			MOVIE_stats.stage = 0;

			if (MOVIE_stats.FR == 0) {
				MOVIE_stats.InitialFocus_R = double::Parse(Start_Z_pos->Text) / 1000;
				MOVIE_stats.InitialFocus_L = double::Parse(Start_Z_pos_RV->Text) / 1000;
			}
			else if (MOVIE_stats.FR == 1) {
				MOVIE_stats.InitialFocus_R = double::Parse(Z_galv_R_RV->Text) / 1000;
				MOVIE_stats.InitialFocus_L = double::Parse(Z_galv_L_RV->Text) / 1000;
			}
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}
			//(Sleep(1000));
			InitialAF_SinglePlane();
			if (MOVIE_stats.cancel_flag == 1) {
				break;
			}

		}

		Finalization();
		Console::WriteLine("Imaging Finished");
		DirList.clear();
		DirList.resize(0);
		MOVIE_Thread_flag = 0;
		end = std::chrono::system_clock::now();
		cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() / 60 << "min\n";
	}
			 //*****************************************************;
			 //multi focus;
			 //*****************************************************;
	private: System::Void Column_start_but_Click(System::Object^  sender, System::EventArgs^  e) {

	}
			 //*****************************************************;
			 //Tiling;
			 //*****************************************************;
	private: vector<FW_RV_PosList> Dir_Prep(double Xstart, double Ystart, double Xstep, double Ystep, int XTile_num, int YTile_num, double deltax, double deltay) {
		cout << "Start Dir_Prep" << endl;
		vector<FW_RV_PosList> list;
		double Xstart_RV = double::Parse(right_limit_RV->Text) + deltax;
		double Ystart_RV = double::Parse(bottom_limit_RV->Text) - deltay;

		char buf_s[7] = { '\0' };
		char buf_s2[15] = { '\0' };
		String^ Dir = Export_Box->Text;
		String^ Stem = Stem_box1->Text;
		String^ Path;
		string pathw;
		char buf1[9] = { '\0' };
		char buf2[9] = { '\0' };
		//cout << "ZeroBuffer prep start" << endl;
		unsigned char* ZeroBuffer = new unsigned char[2304 * 2304];
		for (int i = 0; i < 2304 * 2304; ++i) {
			ZeroBuffer[i] = 0;
		}
		for (int x = 0; x < XTile_num; ++x) {
			for (int y = 0; y < YTile_num; ++y) {
				FW_RV_PosList L;
				int xpos = Xstart + Xstep * x;
				int ypos = Ystart + Ystep * y;
				int xpos_rv = Xstart_RV - Xstep * x;
				int ypos_rv = Ystart_RV + Ystep * y;
				L.FW_xpos = xpos;
				L.FW_ypos = ypos;
				L.RV_xpos = xpos_rv;
				L.RV_ypos = ypos_rv;
				if (x < (XTile_num / 2)) {
					L.FW_LR = 'R';
					L.RV_LR = 'L';
				}
				else {
					L.FW_LR = 'L';
					L.RV_LR = 'R';
				}

				String^ X = (xpos * 10 + 1000000).ToString("D8");
				String^ Y = (ypos * 10 + 1000000).ToString("D8");
				//String^ Y = (ypos * 10 + 1000000+162770).ToString("D8");
				String^ Dir_FW = Dir + "\\" + Stem + "\\FW\\" + Y + "\\" + Y + "_" + X;
				String^ Dir_RV = Dir + "\\" + Stem + "\\RV\\" + Y + "\\" + Y + "_" + X;
				Console::Write(Dir_FW + ":");
				Console::WriteLine(L.FW_xpos + "/" + L.FW_ypos + ":" + L.FW_LR + "//" + L.RV_xpos + "/" + L.RV_ypos + ":" + L.RV_LR);



				Dir_check(Dir_FW);
				Dir_check(Dir_RV);
				string s;
				MarshalString(Dir_FW, s);
				string srv;
				MarshalString(Dir_RV, srv);
				L.dir_FW = s;
				L.dir_RV = srv;

				list.push_back(L);
			}
		}
		delete[] ZeroBuffer;

		return list;
	}
	private: vector<FW_RV_PosList> Dir_Prep_mc(int mc, double Xstart, double Ystart, double Xstep, double Ystep, int XTile_num, int YTile_num, double deltax, double deltay) {
		//cout << "Start Dir_Prep" << endl;
		vector<FW_RV_PosList> list;
		double Xstart_RV = double::Parse(right_limit_RV->Text) + deltax;
		double Ystart_RV = double::Parse(bottom_limit_RV->Text) - deltay;

		char buf_s[7] = { '\0' };
		char buf_s2[15] = { '\0' };
		String^ Dir = Export_Box->Text;
		String^ Stem = Stem_box1->Text;
		String^ Path;
		string pathw;
		char buf1[9] = { '\0' };
		char buf2[9] = { '\0' };
		//cout << "ZeroBuffer prep start" << endl;
		unsigned char* ZeroBuffer = new unsigned char[2304 * 2304];
		for (int i = 0; i < 2304 * 2304; ++i) {
			ZeroBuffer[i] = 0;
		}
		for (int x = 0; x < XTile_num; ++x) {
			for (int y = 0; y < YTile_num; ++y) {
				FW_RV_PosList L;
				int xpos = Xstart + Xstep * x;
				int ypos = Ystart + Ystep * y;
				int xpos_rv = Xstart_RV - Xstep * x;
				int ypos_rv = Ystart_RV + Ystep * y;
				L.FW_xpos = xpos;
				L.FW_ypos = ypos;
				L.RV_xpos = xpos_rv;
				L.RV_ypos = ypos_rv;
				if (x < (XTile_num / 2)) {
					L.FW_LR = 'R';
					L.RV_LR = 'L';
				}
				else {
					L.FW_LR = 'L';
					L.RV_LR = 'R';
				}

				String^ X = (xpos * 10 + 1000000).ToString("D8");
				String^ Y = (ypos * 10 + 1000000).ToString("D8");
				//String^ Y = (ypos * 10 + 1000000+162770).ToString("D8");
				String^ MC;
				if (mc == 0)
					MC = set1_name->Text;
				if (mc == 1)
					MC = set2_name->Text;
				if (mc == 2)
					MC = set3_name->Text;

				String^ Dir_FW = Dir + "\\" + Stem + "\\" + MC + "\\FW\\" + Y + "\\" + Y + "_" + X;
				String^ Dir_RV = Dir + "\\" + Stem + "\\" + MC + "\\RV\\" + Y + "\\" + Y + "_" + X;
				//Console::Write(Dir_FW + ":");
				//Console::WriteLine(L.FW_xpos + "/" + L.FW_ypos + ":" + L.FW_LR + "//" + L.RV_xpos + "/" + L.RV_ypos + ":" + L.RV_LR);

				Dir_check(Dir_FW);
				Dir_check(Dir_RV);
				string s;
				MarshalString(Dir_FW, s);
				string srv;
				MarshalString(Dir_RV, srv);
				L.dir_FW = s;
				L.dir_RV = srv;
				list.push_back(L);
			}
		}
		delete[] ZeroBuffer;

		return list;
	}



	private: System::Void MOVIE_strat_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Acq_flag == 0) {
			Abort_ALL();
			Acq_flag = 1;
			threads.MOVIE_Thread = true;
			if (MOVIE_Thread_flag == 1) {
				cout << "MOVIE Thread is running!" << endl;
			}
			else {
				ScanSpeed_Calc();
				tile_mode = 1;
				multi_mode = 0;
				Abort_flag = 0;
				export_tiling();
				LIVE_flag = 1;
				MOVIE_stats = {};
				CameraSettingUpdate();
				if (LIVE_window1 == 0) {
					LIVE_Thread();
					LIVE_window1 = 1;
				}
				if (SinglePlane->Checked == true) {
					ThreadStart^ start = gcnew ThreadStart(this, &MyForm::SinglePlane_Axial);
					Thread^ thread = gcnew Thread(start);
					thread->Start();
				}
				else if (multi_laser_check->Checked == true) {
					ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Axial_tiling_multi);
					Thread^ thread = gcnew Thread(start);
					thread->Start();
				}
				else {
					ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Axial_tiling);
					Thread^ thread = gcnew Thread(start);
					thread->Start();
				}
			}
		}
	}
	private: System::Void Galvo_Zero_Click(System::Object^  sender, System::EventArgs^  e) {
		DAQ_sheet_Zeropos();
	}
	private: System::Void Check_pos(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void label29_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void ETL_left_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void ETL_right_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void ETL_left_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void ETL_right_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {

	}
	private: void SerialDataRecieved_ETL_L(String^ recvData) {
		string recvData2;
		MarshalString(recvData, recvData2);
		cout << recvData2 << endl;
		if (recvData == "") {}
		if (recvData2.find("TCA") != std::string::npos) {
			string data = recvData2.substr(3, 2);
			//	double Temp = std::stod(data)*0.0625;
			//	ETL_L_temp->Text = Temp.ToString();
		}
	}
	private: void SerialDataRecieved_ETL_R(String^ recvData) {
		string recvData2;
		MarshalString(recvData, recvData2);
		cout << recvData2 << endl;
		if (recvData == "") {}
		if (recvData2.find("TCA") != std::string::npos) {
			const char* data = (recvData2.substr(3, 2)).c_str();
			//double Temp = std::stod(data)*0.0625;
		//	ETL_R_temp->Text = Temp.ToString();
		}
	}
	private: System::Void ETL_L_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {

	}
	private: System::Void ETL_R_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {

	}
	private: System::Void ETL_OFF_Click(System::Object^  sender, System::EventArgs^  e) {
		ETL_R->Close();
		ETL_L->Close();

	}
	private: System::Void ETL_ON_Click(System::Object^  sender, System::EventArgs^  e) {
		ETL_R->Close();
		ETL_L->Close();
		Sleep(200);
		ETL_R->Open();
		ETL_L->Open();

	}
	private: System::Void Start_Z_pos_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {

	}
	private: System::Void End_Z_pos_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {

	}
	private: System::Void A_Positon_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void TEST_R_Click(System::Object^  sender, System::EventArgs^  e) {
		DAQ_clear();
		DAQ_sheet_Zeropos();
	}
	private: System::Void TEST_but_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Load_LOG_Click(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void TEST_but_Click_1(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void com_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void button1_Click_1(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Save_condition_but_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void TEST2_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void right_limit_but_Click_1(System::Object^  sender, System::EventArgs^  e) {
		right_limit->Text = X_pos_current->Text;

	}
	private: System::Void Y_limit_but_Click(System::Object^  sender, System::EventArgs^  e) {
		left_limit->Text = X_pos_current->Text;
	}


	private: System::Void top_limit_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void X_target_pos_TextChanged(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void left_limit_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void bottom_limit_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void right_limit_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x063_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x08_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void x1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void x125_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x16_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void x2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x25_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x32_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x4_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x5_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void x63_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void x100_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Tiling_overlap_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void label32_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void MyForm_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void exposure_time_box_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (Acq_flag == 0) {
			DataChangeFlag = 1;
		}
	}
	private: System::Void Amplitude_sheet_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (LIVE_flag == 1) {
			DataChangeFlag = 1;
		}

	}
	private: System::Void Amplitude_mSPIM_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Galvo_L_check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Galvo_R_check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void close_Click(System::Object^  sender, System::EventArgs^  e) {
		//Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);

		Laser_ALL_OFF();
		Sleep(1000);
		Close_Flag = 0;
		LIVE_window1 = 0;
		LIVE_window2 = 0;
		DAQ_clear();
		DAQ_sheet_endpos();
		cv::destroyWindow("Camera #1");
		Environment::Exit(0);

	}

	private: System::Void Laser_B_Click(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void label5_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void sheet_offset_L_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
		}
	}
	private: System::Void mspim_offset_L_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {

	}
	private: System::Void label71_Click(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void Camera_connect_Click_1(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void label12_Click(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void single_Expo_Start_Click(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void Step_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Start_Z_pos_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Start_Z_pos_TextChanged_1(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void End_Z_pos_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private:void updateETLVal(double v, int n) {
		if (Laser_Info.L488.stat == 1) {
			Axial_ETL_param.Laser488[n] = v;
		}

		if (Laser_Info.L592.stat == 1) {
			Axial_ETL_param.Laser592[n] = v;

		}

		if (Laser_Info.L647.stat == 1) {
			Axial_ETL_param.Laser647[n] = v;

		}
	}
	private: System::Void Color_check_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
		Sleep(100);
		DAQ_sheet_endpos();
		while (1) {
			Sleep(500);
			if (Laser_Info.L488.stat == true) {
				L488_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
				Axial_panel->Text = "488";
				L488_ON->Text = "ON";
				ETL_L_left->Text = Axial_ETL_param.Laser488[0].ToString();
				ETL_L_right->Text = Axial_ETL_param.Laser488[1].ToString();
				ETL_R_left->Text = Axial_ETL_param.Laser488[2].ToString();
				ETL_R_right->Text = Axial_ETL_param.Laser488[3].ToString();

			}
			else {
				L488_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(224)), static_cast<System::Int32>(static_cast<System::Byte>(224)),
					static_cast<System::Int32>(static_cast<System::Byte>(224)));
				L488_ON->Text = "OFF";
			}

			if (Laser_Info.L592.stat == true) {
				Axial_panel->Text = "592";
				L592_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
				L592_ON->Text = "ON";
				ETL_L_left->Text = Axial_ETL_param.Laser592[0].ToString();
				ETL_L_right->Text = Axial_ETL_param.Laser592[1].ToString();
				ETL_R_left->Text = Axial_ETL_param.Laser592[2].ToString();
				ETL_R_right->Text = Axial_ETL_param.Laser592[3].ToString();
			}
			else {
				L592_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(224)), static_cast<System::Int32>(static_cast<System::Byte>(224)),
					static_cast<System::Int32>(static_cast<System::Byte>(224)));
				L592_ON->Text = "OFF";

			}
			if (Laser_Info.L647.stat == true) {
				Axial_panel->Text = "647";
				L647_ON->Text = "ON";
				L647_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
				ETL_L_left->Text = Axial_ETL_param.Laser647[0].ToString();
				ETL_L_right->Text = Axial_ETL_param.Laser647[1].ToString();
				ETL_R_left->Text = Axial_ETL_param.Laser647[2].ToString();
				ETL_R_right->Text = Axial_ETL_param.Laser647[3].ToString();
			}
			else {
				L647_ON->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(224)), static_cast<System::Int32>(static_cast<System::Byte>(224)),
					static_cast<System::Int32>(static_cast<System::Byte>(224)));
				L647_ON->Text = "OFF";
			}
			L488_current_power->Text = Laser_Info.L488.LP_current.ToString("F1");
			L592_current_power->Text = Laser_Info.L592.LP_current.ToString("F1");
			L647_current_power->Text = Laser_Info.L647.LP_current.ToString("F1");
			L488_temp->Text = Laser_Info.L488.temp.ToString("F1");
			L592_temp->Text = Laser_Info.L592.temp.ToString("F1");
			L647_temp->Text = Laser_Info.L647.temp.ToString("F1");
		}
		Sleep(50);
	}
	private: System::Void z_galv_set_Click(System::Object^  sender, System::EventArgs^  e) {
		if (illumination_flag == 'L') {
			Z_galv_L->Text = Z_offset->Text;
		}
		else if (illumination_flag == 'R') {
			Z_galv_R->Text = Z_offset->Text;
		}

	}
	private: System::Void Color_check2_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
		while (1) {
			Sleep(500);
			if (LIVE_flag == 1) {
				LIVE_but->BackColor = System::Drawing::Color::Aqua;
			}
			else {
				LIVE_but->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(224)), static_cast<System::Int32>(static_cast<System::Byte>(224)),
					static_cast<System::Int32>(static_cast<System::Byte>(224)));
			}
			if (Acq_flag == 1) {
				MOVIE_strat->BackColor = System::Drawing::Color::Aqua;
			}
			else {
				MOVIE_strat->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(224)), static_cast<System::Int32>(static_cast<System::Byte>(224)),
					static_cast<System::Int32>(static_cast<System::Byte>(224)));
			}
			Sleep(200);
		}
	}

	private: System::Void SH_high1_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		SH_high1->Text = String::Concat("", SH_high1_bar->Value);
	}
	private: System::Void SH_low1_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		SH_low1->Text = String::Concat("", SH_low1_bar->Value);
	}
	private: System::Void SH_high2_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		SH_high2->Text = String::Concat("", SH_high2_bar->Value);
	}
	private: System::Void SH_low2_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		SH_low2->Text = String::Concat("", SH_low2_bar->Value);
	}

	private: System::Void BW_Q_Laser_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(1000);
		}
		Laser_488->Write("gsn?\r");
		Laser_647->Write("gsn?\r");
		Sleep(100);
		if (L488_COM_NUM->IndexOf("13026") != -1) {
			Console::WriteLine("L488_COM_NUM:" + L488_COM_NUM);
			Laser_488->Close();
			Laser_647->Close();
			String^ C488 = Laser_647->PortName;
			String^ C647 = Laser_488->PortName;
			Laser_488->PortName = C488;
			Laser_647->PortName = C647;
			Laser_488->Open();
			Laser_647->Open();
			Sleep(100);
		}
		else { Console::WriteLine("L488_COM_NUM:" + L488_COM_NUM); }

		//cout << "wait BW_Q_Laser" << endl;
		backgroundWorker_L488->RunWorkerAsync();
		backgroundWorker_L592->RunWorkerAsync();
		backgroundWorker_L647->RunWorkerAsync();

		while (1) {
			if (illumination_flag == 'R') {
				LR_box->Text = "Right";
				this->LR_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (illumination_flag == 'L') {
				LR_box->Text = "Left";
				this->LR_box->BackColor = System::Drawing::Color::Aqua;
			}
			else {
				LR_box->Text = "----";
				this->LR_box->BackColor = System::Drawing::Color::Gray;
			}
		}

	}
	private: System::Void BW_filter_check_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
		while (1) {
			if (filter_num == 0 && filter_flag == 0) {
				current_filter_box->Text = Filter_1->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 0 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + Filter_1->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}

			else if (filter_num == 1 && filter_flag == 0) {
				current_filter_box->Text = Filter_1->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 1 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + Filter_1->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}

			else if (filter_num == 2 && filter_flag == 0) {
				current_filter_box->Text = FIlter_2->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 2 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_2->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}
			else if (filter_num == 3 && filter_flag == 0) {
				current_filter_box->Text = FIlter_3->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 3 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_3->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}
			else if (filter_num == 4 && filter_flag == 0) {
				current_filter_box->Text = FIlter_4->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 4 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_4->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}
			else if (filter_num == 5 && filter_flag == 0) {
				current_filter_box->Text = FIlter_5->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 5 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_5->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}
			else if (filter_num == 6 && filter_flag == 0) {
				current_filter_box->Text = FIlter_6->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 6 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_6->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}

			else if (filter_num == 7 && filter_flag == 0) {
				current_filter_box->Text = FIlter_7->Text;
				this->current_filter_box->BackColor = System::Drawing::Color::Aqua;
			}
			else if (filter_num == 7 && filter_flag == 1) {
				String^ Buf = "move to ";
				current_filter_box->Text = (Buf + FIlter_7->Text);
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}
			else {
				current_filter_box->Text = "------------";
				this->current_filter_box->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)),
					static_cast<System::Int32>(static_cast<System::Byte>(128)));
			}


			Sleep(500);
		}
	}
	private: System::Void TEST_Click(System::Object^  sender, System::EventArgs^  e) {


		cout << Laser_Color(Laser1_comb->SelectedIndex) << endl;
		cout << Filter_Color(filter1_comb->SelectedIndex) << endl;

	}
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Camera_1_use_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Camera_1_use->Checked == true) {
			camera1_on_flag = 1;
		}
		else {
			camera1_on_flag = 0;
		}
	}
	private: System::Void Export_dir_open_Click(System::Object^  sender, System::EventArgs^  e) {
		TCHAR dir[MAX_PATH];            //      選択されたフォルダー名
		TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };        //      初期フォルダー

		/*


		if (GetDir(0, def_dir, dir) == TRUE) {
			cout << dir << endl;
			cout << lstrlen(dir) << endl;
			char Dir[255];
			string a;

			for (int i = 0; i < lstrlen(dir); ++i) {
				Dir[i] = dir[i];
				if (dir[i] == '\\') {
					a.push_back('\\');
				}
				a.push_back(Dir[i]);
			}
			String^ DIR = gcnew String(a.c_str());
			Console::WriteLine(DIR);
			Export_Box->Text = DIR;
		}
		*/
		//フォルダ選択ダイアログの表示
		IFileDialog *pifd = NULL;
		//IFileDialog *pifd = NULL;
		IShellItem *psi = NULL;
		DWORD dwOptions = 0;
		TCHAR pszPath = NULL;
		LPWSTR P = L"Z:\\";
		CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pifd));
		pifd->GetOptions(&dwOptions);

		// この一行をなくすとファイル選択になります。この行があるとフォルダ選択になります。
		pifd->SetOptions(dwOptions | FOS_PICKFOLDERS);

		pifd->Show(NULL);

		pifd->GetResult(&psi);

		// pszPathに選択したファイル、あるいはフォルダへのパスが入ります。
		//psi->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &pszPath);
		psi->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &P);
		String^ SS = marshal_as<System::String^>(P);
		Export_Box->Text = SS;
		psi->Release();
		pifd->Release();

	}

	private: System::Void EXP_time1_but_Click(System::Object^  sender, System::EventArgs^  e) {
		EXP_time1->Text = exposure_time_box->Text;
	}
	private: System::Void EXP_time2_but_Click(System::Object^  sender, System::EventArgs^  e) {
		EXP_time2->Text = exposure_time_box->Text;
	}
	private: System::Void EXP_time3_but_Click(System::Object^  sender, System::EventArgs^  e) {
		EXP_time3->Text = exposure_time_box->Text;
	}
	private: System::Void EXP_time4_but_Click(System::Object^  sender, System::EventArgs^  e) {
		EXP_time4->Text = exposure_time_box->Text;
	}
	private: System::Void LP_set_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser1_comb->SelectedIndex == 1) {
			LP1->Text = L488_box->Text;
		}
		else if (Laser1_comb->SelectedIndex == 2) {
			LP1->Text = L592_box->Text;
		}
		else if (Laser1_comb->SelectedIndex == 3) {
			LP1->Text = L647_box->Text;
		}
	}
	private: System::Void LP_set2_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser2_comb->SelectedIndex == 1) {
			LP2->Text = L488_box->Text;
		}
		else if (Laser2_comb->SelectedIndex == 2) {
			LP2->Text = L592_box->Text;
		}
		else if (Laser2_comb->SelectedIndex == 3) {
			LP2->Text = L647_box->Text;
		}
	}
	private: System::Void LP_set3_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser3_comb->SelectedIndex == 1) {
			LP3->Text = L488_box->Text;
		}
		else if (Laser3_comb->SelectedIndex == 2) {
			LP3->Text = L592_box->Text;
		}
		else if (Laser3_comb->SelectedIndex == 3) {
			LP3->Text = L647_box->Text;
		}
	}
	private: System::Void LP_set4_Click(System::Object^  sender, System::EventArgs^  e) {
		if (Laser4_comb->SelectedIndex == 1) {
			LP4->Text = L488_box->Text;
		}
		else if (Laser4_comb->SelectedIndex == 2) {
			LP4->Text = L592_box->Text;
		}
		else if (Laser4_comb->SelectedIndex == 3) {
			LP4->Text = L647_box->Text;
		}
	}
	private: System::Void LF_check1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void LF_check2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void LF_check3_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void LF_check4_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void multi_laser_check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}


	private: System::Void button1_Click_2(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void BW_piezo_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}


	private: System::Void resetSettingToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
		Progress_count->RunWorkerAsync();
	}

	private: System::Void Progress_count_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}
	private:void loadsetting(String^ fileName) {
		try
		{
			Console::WriteLine("trying to open file {0}...", fileName);
			StreamReader^ din = File::OpenText(fileName);

			String^ str;

			int count = 0;
			int flag_mode = 0;
			int flag_L = 0;
			int flag_R = 0;
			int FL_count;
			int FR_count;
			int FL_pos = 0;
			int FR_pos = 0;
			int dif = 0;
			int FFL_count;
			int FFR_count;
			int FFL_pos = 0;
			int FFR_pos = 0;
			int Fdif = 0;
			int set_count = 0;
			int set_count2 = 0;
			int FL_set_pos = 0;

			setting_reset();

			while ((str = din->ReadLine()) != nullptr) {
				Console::WriteLine(str);
				count++;
				if (str == "") {
					continue;
				}

				if (str->IndexOf("Directory: ") != -1) {
					Export_Box->Text = str->Substring(11);
				}
				if (str->IndexOf("Stem1: ") != -1) {
					Stem_box1->Text = str->Substring(7);
				}
				if (str->IndexOf("singlestackmode: on ") != -1) {
					singlestackmode->Checked = true;

				}
				if (str->IndexOf("Amplitude_sheet: ") != -1) {
					Amplitude_sheet->Text = str->Substring(17);
				}

				if (str->IndexOf("filter_setting1: ") != -1) {
					filter_name1->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting2: ") != -1) {
					filter_name2->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting3: ") != -1) {
					filter_name3->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting4: ") != -1) {
					filter_name4->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting5: ") != -1) {
					filter_name5->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting6: ") != -1) {
					filter_name6->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting7: ") != -1) {
					filter_name7->Text = str->Substring(17);
				}
				if (str->IndexOf("filter_setting8: ") != -1) {
					filter_name8->Text = str->Substring(17);
				}

				if (str->IndexOf("Singlestackmode") != -1) {
					singlestackmode->Checked = true;
				}
				if (str->IndexOf("Tilemode") != -1) {
					singlestackmode->Checked = false;
				}

				if (str->IndexOf("top_limit_RV: ") != -1) {
					top_limit_RV->Text = str->Substring(14);
				}
				if (str->IndexOf("bottom_limit_RV: ") != -1) {
					bottom_limit_RV->Text = str->Substring(17);
				}
				if (str->IndexOf("left_limit_RV: ") != -1) {
					left_limit_RV->Text = str->Substring(15);
				}
				if (str->IndexOf("right_limit_RV: ") != -1) {
					right_limit_RV->Text = str->Substring(16);
				}

				if (str->IndexOf("sheet_offset_L: ") != -1) {
					sheet_offset_L->Text = str->Substring(16);
				}
				if (str->IndexOf("sheet_offset_R: ") != -1) {
					sheet_offset_R->Text = str->Substring(16);
				}


				if (str->IndexOf("MOVIE_Retry_Box: ") != -1) {
					MOVIE_Retry_Box->Text = str->Substring(17);
				}
				if (str->IndexOf("Z_overlap: ") != -1) {
					Z_overlap_box->Text = str->Substring(11);
				}

				if (str->IndexOf("Z_galv_L: ") != -1) {
					Z_galv_L->Text = str->Substring(10);
				}

				if (str->IndexOf("Z_galv_R: ") != -1) {
					Z_galv_R->Text = str->Substring(10);
				}

				if (str->IndexOf("Z_galv_L_RV: ") != -1) {
					Z_galv_L_RV->Text = str->Substring(13);
				}

				if (str->IndexOf("Z_galv_R_RV: ") != -1) {
					Z_galv_R_RV->Text = str->Substring(13);
				}


				String^ SS = "Start_Z_pos_RV: ";

				if (str->IndexOf(SS) != -1) {
					Start_Z_pos_RV->Text = str->Substring(SS->Length);
				}
				SS = "End_Z_pos_RV: ";
				if (str->IndexOf(SS) != -1) {
					End_Z_pos_RV->Text = str->Substring(SS->Length);
				}
				SS = "ReverseSpos: ";
				if (str->IndexOf(SS) != -1) {
					ReverseSpos->Text = str->Substring(SS->Length);
				}
				SS = "Theta_POS_X: ";
				if (str->IndexOf(SS) != -1) {
					Theta_POS_X->Text = str->Substring(SS->Length);
				}
				SS = "Theta_POS_Z: ";
				if (str->IndexOf(SS) != -1) {
					Theta_POS_Z->Text = str->Substring(SS->Length);
				}
				SS = "windowsize_box: ";
				if (str->IndexOf(SS) != -1) {
					windowsize_box->Text = str->Substring(SS->Length);
				}
				SS = "delay_time_box: ";
				if (str->IndexOf(SS) != -1) {
					delay_time_box->Text = str->Substring(SS->Length);
				}
				SS = "Extra_time_box: ";
				if (str->IndexOf(SS) != -1) {
					Extra_time_box->Text = str->Substring(SS->Length);
				}




				String^ mag;
				if (str->IndexOf("Magnification_ROI: ") != -1) {
					mag = str->Substring(19);
				}

				if (str->IndexOf("Laser_Filter_set_count: ") != -1) {
					set_count = int::Parse(str->Substring(24));
					set_count2 = count;
					cout << set_count << endl;
				}
				if (count > set_count2 + 1 && count < set_count2 + 2 + set_count) {
					if (FL_set_pos == 0) {
						cout << set_count << endl;
						LF_check1->Checked = true;
						Laser1_comb->SelectedIndex = int::Parse(str->Substring(0, 1));
						int a = str->Substring(4)->IndexOf("/");
						filter1_comb->SelectedIndex = int::Parse(str->Substring(2, 1));
						LP1->Text = str->Substring(4, str->Substring(4)->IndexOf("/"));
						String^ str2 = str->Substring(4 + a + 1);
						int b = str2->IndexOf("/");
						Console::WriteLine("str2; " + str2);
						EXP_time1->Text = str2->Substring(0, b);

						String^ str3 = str2->Substring(b + 1);
						Console::WriteLine("str3; " + str3);
						int c = str3->IndexOf("/");
						String^ str4 = str3->Substring(c + 1);
						Console::WriteLine("str4: " + str4);
						set1_name->Text = str4->Substring(0);

					}
					if (FL_set_pos == 1) {
						LF_check2->Checked = true;
						Laser2_comb->SelectedIndex = int::Parse(str->Substring(0, 1));
						int a = str->Substring(4)->IndexOf("/");
						filter2_comb->SelectedIndex = int::Parse(str->Substring(2, 1));
						LP2->Text = str->Substring(4, str->Substring(4)->IndexOf("/"));
						String^ str2 = str->Substring(4 + a + 1);
						int b = str2->IndexOf("/");
						EXP_time2->Text = str2->Substring(0, b);
						String^ str3 = str2->Substring(b + 1);
						Console::WriteLine("str3; " + str3);
						int c = str3->IndexOf("/");
						String^ str4 = str3->Substring(c + 1);
						Console::WriteLine("str4: " + str4);
						set2_name->Text = str4->Substring(0);
					}
					if (FL_set_pos == 2) {
						LF_check3->Checked = true;
						Laser3_comb->SelectedIndex = int::Parse(str->Substring(0, 1));
						int a = str->Substring(4)->IndexOf("/");
						filter3_comb->SelectedIndex = int::Parse(str->Substring(2, 1));
						LP3->Text = str->Substring(4, str->Substring(4)->IndexOf("/"));
						String^ str2 = str->Substring(4 + a + 1);
						int b = str2->IndexOf("/");
						EXP_time3->Text = str2->Substring(0, b);
						String^ str3 = str2->Substring(b + 1);
						Console::WriteLine("str3; " + str3);
						int c = str3->IndexOf("/");
						String^ str4 = str3->Substring(c + 1);
						Console::WriteLine("str4: " + str4);
						set3_name->Text = str4->Substring(0);
					}
					if (FL_set_pos == 3) {
						LF_check4->Checked = true;
						Laser4_comb->SelectedIndex = int::Parse(str->Substring(0, 1));
						int a = str->Substring(4)->IndexOf("/");
						filter4_comb->SelectedIndex = int::Parse(str->Substring(2, 1));
						LP4->Text = str->Substring(4, str->Substring(4)->IndexOf("/"));
						String^ str2 = str->Substring(4 + a + 1);
						int b = str2->IndexOf("/");
						EXP_time4->Text = str2->Substring(0, b);
						String^ str3 = str2->Substring(b + 1);
						Console::WriteLine("str3; " + str3);
						int c = str3->IndexOf("/");
						String^ str4 = str3->Substring(c + 1);
						Console::WriteLine("str4: " + str4);
						set4_name->Text = str4->Substring(0);
					}
					++FL_set_pos;
				}
			Laser_Filter_set_count:

				if (str->IndexOf("mode: multifocus") != -1) {
					flag_mode = 1;
				}
				if (str->IndexOf("mode: axial-sweep") != -1) {
					flag_mode = 2;
				}
				if (str->IndexOf("mode: tiling") != -1) {
					flag_mode = 3;
				}
				if (str->IndexOf("Axial mode-OFF") != -1) {
					Axial_mode_check->Checked == false;
				}
				if (str->IndexOf("Axial mode-ON") != -1) {
					Axial_mode_check->Checked == true;
				}

				if (str->IndexOf("MOVIE_Amplitude: ") != -1) {
					double I = 0;
					double::TryParse(str->Substring(17), I);
					MOVIE_Amplitude->Text = I.ToString();
				}

				if (str->IndexOf("Exposuretime:") != -1) {
					double I = 0;
					double::TryParse(str->Substring(14), I);
					exposure_time_box->Text = I.ToString();
				}
				else if (str->IndexOf("Z_ScanStep:") != -1) {
					double I = 5;
					double::TryParse(str->Substring(12), I);
					Step->Text = I.ToString();
				}
				else if (str->IndexOf("Start_Z/End_Z:") != -1) {
					int i = str->IndexOf("Start_Z/End_Z:");//15
					int j = str->IndexOf("/", 15);
					Start_Z_pos->Text = str->Substring(15, j - 15);
					End_Z_pos->Text = str->Substring(j + 1);
				}

				else if (str->IndexOf("Top:") != -1) {
					int i = str->IndexOf("Top:");
					Console::WriteLine(str->Substring(5));
					top_limit->Text = str->Substring(5);
				}
				else if (str->IndexOf("left:") != -1) {
					int i = str->IndexOf("left:");
					left_limit->Text = str->Substring(6);

				}
				else if (str->IndexOf("bottom:") != -1) {
					int i = str->IndexOf("bottom:");
					bottom_limit->Text = str->Substring(8);
				}
				else if (str->IndexOf("right:") != -1) {
					int i = str->IndexOf("right:");
					right_limit->Text = str->Substring(7);
				}
				else if (str->IndexOf("Background: ") != -1) {
					BK_box->Text = str->Substring(12);
				}
				else if (str->IndexOf("left focus Count: ") != -1) {
					FL_pos = count + 2;
					dif = 0;
					FL_count = int::Parse(str->Substring(18));
				}
				else if (str->IndexOf("right focus Count: ") != -1) {
					FR_pos = count + 2;
					dif = 0;
					FR_count = int::Parse(str->Substring(19));
				}



			}
			din->Close();
		}
		catch (Exception^ e)
		{
			if (dynamic_cast<FileNotFoundException^>(e))
				Console::WriteLine("file '{0}' not found", fileName);
			else {
				Console::WriteLine("problem reading file '{0}'", fileName);
				Console::WriteLine(e);
			}
		}
	}
	private: System::Void loadsettingToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		String^ fileName;// 選択されたフォルダのフルパス  
		OpenFileDialog^ dlg = gcnew OpenFileDialog;
		dlg->InitialDirectory = "initial_path\\Setting";
		dlg->Filter = "system file (*.gps)|*.gps";
		if (dlg->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			fileName = dlg->FileName;
		loadsetting(fileName);
	}
	private: System::Void loadLogFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		String^ fileName;// 選択されたフォルダのフルパス  

		OpenFileDialog^ dlg = gcnew OpenFileDialog;
		dlg->InitialDirectory = "initial_path\\LOG";
		dlg->Filter = "Log(*.log) |*.log;|default(*.gps)|*.gps|Text(*.txt)|*.txt";
		if (dlg->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			fileName = dlg->FileName;
		loadsetting(fileName);
	}
	private: System::Void saveFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		exportconditions();
	}
	private: System::Void logFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		String^ fileName;// 選択されたフォルダのフルパス  
		OpenFileDialog^ dlg = gcnew OpenFileDialog;
		dlg->InitialDirectory = "initial_path\\LOG";
		dlg->Filter = "Log(*.log) |*.log;|default(*.gps)|*.gps|Text(*.txt)|*.txt";
		if (dlg->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			fileName = dlg->FileName;
		loadsetting(fileName);
	}
	private: System::Void settingFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {

		String^ fileName;// 選択されたフォルダのフルパス  
		OpenFileDialog^ dlg = gcnew OpenFileDialog;
		dlg->InitialDirectory = "initial_path\\Setting";
		dlg->Filter = "system file (*.gps)|*.gps";
		if (dlg->ShowDialog() == System::Windows::Forms::DialogResult::OK)
			fileName = dlg->FileName;
		loadsetting(fileName);
	}
	private: System::Void resetSettingToolStripMenuItem_Click_1(System::Object^  sender, System::EventArgs^  e) {
		setting_reset();
	}
	private: System::Void toolStripMenuItem1_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void BW_viewer1_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {


	}

	private: System::Void openImageFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void LIVE_window_close1_Click(System::Object^  sender, System::EventArgs^  e) {
		LIVE_window1 = 0;
	}

	private: System::Void LIVE_window_close2_Click(System::Object^  sender, System::EventArgs^  e) {
		LIVE_window2 = 0;
	}


	private: System::Void bitToolStripMenuItem1_Click(System::Object^  sender, System::EventArgs^  e) {
		TCHAR dir[MAX_PATH];            //      選択されたフォルダー名
		TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };        //      初期フォルダー


		string dir_name;
		if (GetDir(0, def_dir, dir) == TRUE) {
			cout << dir << endl;
			cout << lstrlen(dir) << endl;
			char Dir[255];


			for (int i = 0; i < lstrlen(dir); ++i) {
				Dir[i] = dir[i];
				if (dir[i] == '\\') {
					dir_name.push_back('\\');
				}
				dir_name.push_back(Dir[i]);
			}
		}
		if (strcmp(dir_name.c_str(), deffolder.c_str()) != 0) {

			String^ DIR = gcnew String(dir_name.c_str());

			Diagnostics::ProcessStartInfo^ startInfo2 = gcnew Diagnostics::ProcessStartInfo("C:\\Users\\ueda_macro\\source\\repos\\dat_to_tif\\dat_to_tiff_8bit\\x64\\Release\\dat_to_tiff_8bit.exe");
			startInfo2->WindowStyle = Diagnostics::ProcessWindowStyle::Normal;
			startInfo2->Arguments = DIR;
			Diagnostics::Process::Start(startInfo2);

		}

	}
	private: System::Void stitchingToolStripMenuItem_Click_2(System::Object^  sender, System::EventArgs^  e) {

		TCHAR dir[MAX_PATH];
		TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };

		string dir_name;
		if (GetDir(0, def_dir, dir) == TRUE) {
			cout << dir << endl;
			cout << lstrlen(dir) << endl;
			char Dir[255];


			for (int i = 0; i < lstrlen(dir); ++i) {
				Dir[i] = dir[i];
				if (dir[i] == '\\') {
					dir_name.push_back('\\');
				}
				dir_name.push_back(Dir[i]);
			}
		}

		if (strcmp(dir_name.c_str(), deffolder.c_str()) != 0) {

			String^ DIR = gcnew String(dir_name.c_str());
			Diagnostics::ProcessStartInfo^ startInfo2 = gcnew Diagnostics::ProcessStartInfo("C:\\Users\\ueda_macro\\source\\repos\\Tile_reconstitution\\x64\\Release\\Stitcher.exe");
			startInfo2->WindowStyle = Diagnostics::ProcessWindowStyle::Normal;
			startInfo2->Arguments = DIR;
			Diagnostics::Process::Start(startInfo2);
		}

	}
	private: System::Void bitmaxminToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		TCHAR dir[MAX_PATH];            //      選択されたフォルダー名
		TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };        //      初期フォルダー

		string dir_name;
		if (GetDir(0, def_dir, dir) == TRUE) {
			cout << dir << endl;
			cout << lstrlen(dir) << endl;
			char Dir[255];


			for (int i = 0; i < lstrlen(dir); ++i) {
				Dir[i] = dir[i];
				if (dir[i] == '\\') {
					dir_name.push_back('\\');
				}
				dir_name.push_back(Dir[i]);
			}
		}
		String^ DIR = gcnew String(dir_name.c_str());
		if (strcmp(dir_name.c_str(), deffolder.c_str()) != 0) {
			Diagnostics::ProcessStartInfo^ startInfo3 = gcnew Diagnostics::ProcessStartInfo("C:\\Users\\ueda_macro\\source\\repos\\dat_to_tif\\dat_to_tiff_8bit_norm\\x64\\Release\\dat_to_tiff_8it_norm.exe");
			startInfo3->WindowStyle = Diagnostics::ProcessWindowStyle::Normal;
			startInfo3->Arguments = DIR;
			Diagnostics::Process::Start(startInfo3);
		}

	}
	private: System::Void stitchingToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		TCHAR dir[MAX_PATH];            //      選択されたフォルダー名
		TCHAR def_dir[MAX_PATH] = { _T("Z:\\") };        //      初期フォルダー			
		string dir_name;
		if (GetDir(0, def_dir, dir) == TRUE) {
			cout << dir << endl;
			cout << lstrlen(dir) << endl;
			char Dir[255];


			for (int i = 0; i < lstrlen(dir); ++i) {
				Dir[i] = dir[i];
				if (dir[i] == '\\') {
					dir_name.push_back('\\');
				}
				dir_name.push_back(Dir[i]);
			}
		}
		String^ DIR = gcnew String(dir_name.c_str());
		if (strcmp(dir_name.c_str(), deffolder.c_str()) != 0) {

			Diagnostics::ProcessStartInfo^ startInfo3 = gcnew Diagnostics::ProcessStartInfo("C:\\Users\\ueda_macro\\source\\repos\\Image_Integration\\x64\\Release\\Image_Integration.exe");
			startInfo3->WindowStyle = Diagnostics::ProcessWindowStyle::Normal;
			startInfo3->Arguments = DIR;
			Diagnostics::Process::Start(startInfo3);
		}

	}
	private: System::Void ResizeWindow_Click(System::Object^  sender, System::EventArgs^  e) {
		cv::resizeWindow("Camera #1", 800, 800);
		cv::resizeWindow("Camera #2", 800, 800);
	}

	private: System::Void z_ori_Click(System::Object^  sender, System::EventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "ORG2/7/0\r");
		Zstage_Flag = 0;

	}
	private: System::Void right_1_MouseDown_1(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		startm = std::chrono::system_clock::now();
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIX:SELSP 4:Go CWJ\r");
		XY_stage_flag = 0;
	}
	private: System::Void right_1_MouseUp_1(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		endm = std::chrono::system_clock::now();
		double elapsed = chrono::duration_cast<std::chrono::milliseconds>(endm - startm).count();
		XY_stage_flag = 1;
		if (elapsed < 60) {
			int XY = int::Parse(X_pos_current->Text);
			XYstage_port->Write("AXIX:STOP\r");
			XYstage_port->Write("AXIX:GOABSolute " + (XY + 10).ToString() + "\r");
			//XYstage_port->Write("AXIX:Go CWJ\r");
			//Sleep(50);
		}
		else {
			XYstage_port->Write("AXIX:STOP\r");
		}
		XY_stage_flag = 0;
	}


	private: System::Void statusStrip2_ItemClicked(System::Object^  sender, System::Windows::Forms::ToolStripItemClickedEventArgs^  e) {
	}
	private: System::Void left_1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		startm = std::chrono::system_clock::now();
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIX:SELSP 4:Go CCWJ\r");
		XY_stage_flag = 0;
	}
	private: System::Void left_1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		XY_stage_flag = 1;
		endm = std::chrono::system_clock::now();
		Sleep(50);
		double elapsed = chrono::duration_cast<std::chrono::milliseconds>(endm - startm).count();
		if (elapsed < 60) {
			int XY = int::Parse(X_pos_current->Text);
			XYstage_port->Write("AXIX:STOP\r");
			XYstage_port->Write("AXIX:GOABSolute " + (XY - 10).ToString() + "\r");
		}
		else {
			XYstage_port->Write("AXIX:STOP\r");
		}
		XY_stage_flag = 0;

	}
	private: System::Void down_1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		XY_stage_flag = 1;
		Sleep(50);
		startm = std::chrono::system_clock::now();
		XYstage_port->Write("AXIY:SELSP 4:Go CCWJ\r");
		XY_stage_flag = 0;

	}
	private: System::Void down_1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		endm = std::chrono::system_clock::now();
		double elapsed = chrono::duration_cast<std::chrono::milliseconds>(endm - startm).count();

		XY_stage_flag = 1;
		Sleep(50);
		if (elapsed < 60) {
			int XY = int::Parse(Y_pos_current->Text);
			XYstage_port->Write("AXIY:STOP\r");
			XYstage_port->Write("AXIY:GOABSolute " + (XY - 10).ToString() + "\r");
			//XYstage_port->Write("AXIX:Go CWJ\r");
			//Sleep(50);
		}
		else {
			XYstage_port->Write("AXIY:STOP\r");
		}
		XY_stage_flag = 0;
	}
	private: System::Void up_1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		XY_stage_flag = 1;
		Sleep(50);
		//XYstage_port->Write("AXIY:SELSP 3:GOABS " + Stagepos_target.z.ToString() + "\r");
		startm = std::chrono::system_clock::now();
		XYstage_port->Write("AXIY:SELSP 4:Go CWJ\r");
		XY_stage_flag = 0;

	}
	private: System::Void up_1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		endm = std::chrono::system_clock::now();
		double elapsed = chrono::duration_cast<std::chrono::milliseconds>(endm - startm).count();
		XY_stage_flag = 1;
		Sleep(50);
		if (elapsed < 60) {
			int XY = int::Parse(Y_pos_current->Text);
			XYstage_port->Write("AXIY:STOP\r");
			XYstage_port->Write("AXIY:GOABSolute " + (XY + 10).ToString() + "\r");
		}
		else {
			XYstage_port->Write("AXIY:STOP\r");
		}
		XY_stage_flag = 0;
	}

	private: System::Void backgroundWorker_XYstage_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		Sleep(3000);
		while (1) {
			try {
				if (XY_stage_flag == 0) {
					Xstage_Flag = 1;
					Ystage_Flag = 0;
					XYstage_port->Write("AXIX:POS?\r");
					while (Xstage_Flag == 1) {
						//cout << Xstage_Flag << ":" << Ystage_Flag << endl;
						Sleep(10);
					}
					Xstage_Flag = 0;
					Ystage_Flag = 1;
					XYstage_port->Write("AXIY:POS?\r");
					while (Ystage_Flag == 1) {
						//cout << Xstage_Flag << ":" << Ystage_Flag << endl;
						Sleep(10);
					}
					Sleep(10);
				}
			}
			catch (exception &e) {
				cout << "XY stage pos error" << endl;
			}
		}

	}
	private: System::Void BW_XY_ini_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}

		BW_XY_ini->CancelAsync();
	}
	private: System::Void Z_down_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP2/4/1\r");
		Sleep(50);
		Zstage_Flag = 0;

	}
	private: System::Void Z_down_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void Z_up_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP2/4/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void Z_up_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void s_left_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP1/3/0\r");
		Sleep(50);
		Zstage_Flag = 10;
	}
	private: System::Void s_left_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP1/0\r");
		Read_S_Pos();
		Zstage_Flag = 0;
	}
	private: System::Void s_right_MouseDown_1(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP1/3/1\r");
		Sleep(50);
		Zstage_Flag = 10;
	}
	private: System::Void s_right_MouseUp_1(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP1/0\r");
		Sleep(50);
		Read_S_Pos();
		Zstage_Flag = 0;
	}
	private: System::Void Z_up_FF_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP2/8/1\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void Z_up_FF_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 2;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void Z_down_FF_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "FRP2/8/0\r");
		Sleep(50);
		//Zstage_Flag = 0;
	}
	private: System::Void Z_down_FF_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void XY_home_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void filter_ini_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		Sleep(1000);
		cout << "filter_ini_DoWork" << endl;
		if (filter_flag == 0) {
			filter_num = 1;
			filter_move();
		}
	}
	private: System::Void Laser_ini_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		/*
		Zstage_Flag = 1;
		if (KOUZU_flag_m != 1) {
			KOUZU_flag_m = 1;
			Laser_LR_t = 'R';
			if (BW_Laser_LR->IsBusy == false) {
				BW_Laser_LR->RunWorkerAsync();
			}
		}
		Zstage_Flag = 0;*/
	}
	private: System::Void ETL_correction_647_488_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}



	private: System::Void filter_name3_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void Set_home_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void SH_high1_TextChanged(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void SH_high1_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ SH = SH_high1->Text;
			string sh;
			MarshalString(SH, sh);
			if (check_int(sh) == true) {
				SH_high1_bar->Value = int::Parse(SH_high1->Text);
			}
		}
	}
	private: System::Void SH_low1_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ SH = SH_low1->Text;
			string sh;
			MarshalString(SH, sh);
			if (check_int(sh) == true) {
				SH_low1_bar->Value = int::Parse(SH_low1->Text);
			}
		}
	}
	private: System::Void SH_high2_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ SH = SH_high2->Text;
			string sh;
			MarshalString(SH, sh);
			if (check_int(sh) == true) {
				SH_high2_bar->Value = int::Parse(SH_high2->Text);
			}
		}
	}
	private: System::Void SH_low2_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ SH = SH_low2->Text;
			string sh;
			MarshalString(SH, sh);
			if (check_int(sh) == true) {
				SH_low2_bar->Value = int::Parse(SH_low2->Text);
			}
		}


	}
	private: System::Void BW_ActiveWindow_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		Sleep(5000);
		char str0[] = "GPS imager";
		char str1[] = "Camera #1";
		char str2[] = "Camera #2";
		char str3[] = "Image 1";
		char str4[] = "Image 2";
		char str5[] = "PreScan_XY";
		char str6[] = "PreScan_YZ";
		char str7[] = "PreScan_XZ";
		while (1) {
			char buf[1000];
			HWND hWnd = GetForegroundWindow();
			GetWindowText(hWnd, buf, 1000);
			if (strcmp(buf, str0) == 0) {
				ActiveWindow = 0;
			}
			else if (strcmp(buf, str1) == 0) {
				ActiveWindow = 1;
				Image_window1 = 1;
			}
			else if (strcmp(buf, str2) == 0) {
				ActiveWindow = 2;
				Image_window1 = 2;
			}
			else if (strcmp(buf, str3) == 0) {
				ActiveWindow = 3;
			}
			else if (strcmp(buf, str4) == 0) {
				ActiveWindow = 4;
			}
			else if (strcmp(buf, str5) == 0) {
				ActiveWindow = 5;
			}
			else if (strcmp(buf, str6) == 0) {
				ActiveWindow = 6;
			}
			else if (strcmp(buf, str7) == 0) {
				ActiveWindow = 7;
			}
			else {
				ActiveWindow = 0;
			}
		}
	}

	private: System::Void button5_Click(System::Object^  sender, System::EventArgs^  e) {
		cv::resizeWindow("Image 1", 1024, 1024);
		cv::resizeWindow("Image 2", 1024, 1024);
	}
	private: System::Void VW1_open_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void VW2_open_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void BW_viewer2_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {


	}
	private: System::Void Open_seq_1_Click(System::Object^  sender, System::EventArgs^  e) {

	}

	private: System::Void VW_seq1_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {



	}

	private: System::Void label43_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private:void Load_COMS() {
		try {
			cout << "\nLoad_COMs" << endl;
			String^ filename = "COMport.txt";
			StreamReader^ din = File::OpenText(filename);
			int v;
			String^ str;
			while ((str = din->ReadLine()) != nullptr) {
				Console::WriteLine(str);
				if (str == "") {
					continue;
				}

				if (str->IndexOf("L1:") != -1) {
					string s;
					MarshalString(str->Substring(3), s);
					Laser_Info.L488.COM = s;
					Laser_488->PortName = str->Substring(3, str->Length - 3);
				}
				if (str->IndexOf("L2:") != -1) {
					string s;
					MarshalString(str->Substring(3), s);
					Laser_Info.L592.COM = s;
					Laser_592->PortName = str->Substring(3, str->Length - 3);
				}
				if (str->IndexOf("L3:") != -1) {
					string s;
					MarshalString(str->Substring(3), s);
					Laser_Info.L647.COM = s;
					Laser_647->PortName = str->Substring(3, str->Length - 3);
				}
				if (str->IndexOf("L4:") != -1) {
					string s;
					MarshalString(str->Substring(3), s);
					Laser_Info.L647.COM = s;
					Laser_647->PortName = str->Substring(3, str->Length - 3);
				}
				if (str->IndexOf("add:") != -1) {
					string s;
					MarshalString(str->Substring(4), s);
					CoboltCOMlist.push_back(s);
				}
			}
			din->Close();
		}
		catch (System::Exception^ e) {
			MessageBox::Show(e->ToString());
		}
	}
	private:void GetShutterInfomation(Orca_Param& P) {
		DCAMERR err;
		double v;

		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, &v);
		P.Pixeltype = v;
		cout << "Pixeltype: " << v << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGE_WIDTH, &v);
		P.width = v;
		cout << "Pixel width: " << v << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGE_HEIGHT, &v);
		P.height = v;
		cout << "Pixel height: " << v << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGE_ROWBYTES, &v);
		P.rowbyte = v;
		cout << "Pixel rowbytes: " << v << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_BUFFER_FRAMEBYTES, &v);
		P.bufframebytes = v;
		cout << "FRAMEBYTES: " << v << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGEDETECTOR_PIXELWIDTH, &v);
		P.pixelwidth = v;
		cout << "Pixel width: " << v << " um" << endl;
		err = dcamprop_getvalue(P.hdcam, DCAM_IDPROP_IMAGEDETECTOR_PIXELHEIGHT, &v);
		P.pixelheight = v;
		cout << "Pixel height: " << v << " um" << endl;
	}
	private: System::Void Surveillance_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
		Sleep(100);
		Diagnostics::ProcessStartInfo^ startInfo = gcnew Diagnostics::ProcessStartInfo("C:\\Users\\ueda_macro\\source\\repos\\Surveillance\\x64\\Release\\Surveillance.exe");
		startInfo->WindowStyle = Diagnostics::ProcessWindowStyle::Normal;
		Diagnostics::Process::Start(startInfo);
	}
	private:int Orca_Connection(Orca_Param & P) {
		printf("Orca_Connection START\n");
		int	ret = 0;
		DCAMAPI_INIT apiinit;
		memset(&apiinit, 0, sizeof(apiinit));
		apiinit.size = sizeof(apiinit);
		DCAMERR err;
		err = dcamapi_init(&apiinit);
		if (failed(err)) {
			cout << "Orca_Connection error" << endl;
			return NULL;
		}

		int32 nDev = apiinit.iDeviceCount;
		ASSERT(nDev > 0);
		int32 iDev;
		for (iDev = 0; iDev < nDev; ++iDev) {
			dcamcon_show_dcamdev_info((HDCAM)(intptr_t)iDev);
		}
		cout << "nDev" << nDev << endl;
		for (iDev = 0; iDev < nDev; ++iDev) {
			P.hdcam = dcamcon_init_open(iDev);
			if (P.hdcam != NULL)
			{
				// show device information
				dcamcon_show_dcamdev_info(P.hdcam);
				// open wait handle
				char	buf[256];
				my_dcamdev_string(err, P.hdcam, DCAM_IDSTR_CAMERAID, buf, sizeof(buf));
				Orca = P;
				string s = buf;
				CameraNo_1->Text = marshal_as<System::String^>(s);
				GetShutterInfomation(Orca);
				Camera_1_use->Checked = true;
				Orca.cam = 10;
				CameraSettings_Orca(Orca);
				printf("\nOrca_Flush For 10x connected\n\n");
			}

			else {
				cout << "Camera connection failed" << endl;
			}

		}

		return 0;
	}


	private: bool Camera_initialization(int X) {

		Orca_Connection(Orca);
		return true;
	}
	private: System::Void Camera_ini_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		Camera_initialization(0);
		//Camera_initialization(1);
		//Camera_ini->CancelAsync();
	}
	private: System::Void Camera_ini2_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		//Camera_initialization(1);
		//Camera_ini2->CancelAsync();
	}

	private: System::Void Column_Click(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void Progress_count_ProgressChanged(System::Object^  sender, System::ComponentModel::ProgressChangedEventArgs^  e) {

	}
	private: System::Void stitchingToolStripMenuItem_Click_1(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void button1_Click_4(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void groupBox5_Enter(System::Object^  sender, System::EventArgs^  e) {
	}


	private: System::Void Focus_limit_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Focus_limit2_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}


	private: System::Void manual_focus_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: delegate System::Void Dele_ROI_BOX(String ^str);
	private: delegate System::Void Dele_AF_left_pos(String ^str);
	private: delegate System::Void Dele_AF_right_pos(String ^str);
	private: delegate System::Void Dele_AF_left(String ^str);
	private: delegate System::Void Dele_AF_right(String ^str);

	private: System::Void AF_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}


	private: System::Void AF_Camera_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

	}

	private: void Laser_filter_setting_func(int x) {

		if (Laser_Info.L488.stat == 1) {
			if (x == 1) {
				Laser1_comb->SelectedIndex = 1;
				LP1->Text = L488_box->Text;
			}
			if (x == 2) {
				Laser2_comb->SelectedIndex = 1;
				LP2->Text = L488_box->Text;
			}
			if (x == 3) {
				Laser3_comb->SelectedIndex = 1;
				LP3->Text = L488_box->Text;
			}
			if (x == 4) {
				Laser4_comb->SelectedIndex = 1;
				LP4->Text = L488_box->Text;
			}
		}
		else if (Laser_Info.L592.stat == 1) {
			if (x == 1) {
				Laser1_comb->SelectedIndex = 2;
				LP1->Text = L592_box->Text;
			}
			if (x == 2) {
				Laser2_comb->SelectedIndex = 2;
				LP2->Text = L592_box->Text;
			}
			if (x == 3) {
				Laser3_comb->SelectedIndex = 2;
				LP3->Text = L592_box->Text;
			}
			if (x == 4) {
				Laser4_comb->SelectedIndex = 2;
				LP4->Text = L592_box->Text;
			}
		}
		else if (Laser_Info.L647.stat == 1) {
			if (x == 1) {
				Laser1_comb->SelectedIndex = 3;
				LP1->Text = L647_box->Text;
			}
			if (x == 2) {
				Laser2_comb->SelectedIndex = 3;
				LP2->Text = L647_box->Text;
			}
			if (x == 3) {
				Laser3_comb->SelectedIndex = 3;
				LP3->Text = L647_box->Text;
			}
			if (x == 4) {
				Laser4_comb->SelectedIndex = 3;
				LP4->Text = L647_box->Text;
			}
		}
		if (filter_num == 0) {
			if (x == 1) {
				filter1_comb->SelectedIndex = 0;
			}
			else if (x == 2) {
				filter2_comb->SelectedIndex = 0;
			}
			else if (x == 3) {
				filter3_comb->SelectedIndex = 0;
			}
			else if (x == 4) {
				filter4_comb->SelectedIndex = 0;
			}
		}
		if (filter_num == 1) {
			if (x == 1) {
				filter1_comb->SelectedIndex = 0;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = 0;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = 0;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = 0;
			}
		}
		if (filter_num == 2) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}

		if (filter_num == 3) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}
		if (filter_num == 4) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}
		if (filter_num == 5) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}
		if (filter_num == 6) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}
		if (filter_num == 7) {
			if (x == 1) {
				filter1_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 2) {
				filter2_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 3) {
				filter3_comb->SelectedIndex = filter_num - 1;
			}
			if (x == 4) {
				filter4_comb->SelectedIndex = filter_num - 1;
			}
		}
	}
	private: System::Void Laser_filter_setting_but_Click(System::Object^  sender, System::EventArgs^  e) {

		if (LF_check1->Checked == false) {
			Laser_filter_setting_func(1);
			LF_check1->Checked = true;
			EXP_time1->Text = exposure_time_box->Text;
		}
		else if (LF_check2->Checked == false) {
			Laser_filter_setting_func(2);
			LF_check2->Checked = true;
			EXP_time2->Text = exposure_time_box->Text;
		}
		else if (LF_check3->Checked == false) {

			Laser_filter_setting_func(3);
			LF_check3->Checked = true;
			EXP_time3->Text = exposure_time_box->Text;
		}
		else if (LF_check4->Checked == false) {

			Laser_filter_setting_func(4);
			LF_check4->Checked = true;
			EXP_time4->Text = exposure_time_box->Text;
		}
	}
	private: System::Void groupBox7_Enter(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Camera_temp_Click(System::Object^  sender, System::EventArgs^  e) {
		double temperature = 0;
		AT_GetFloat(Camera1.Hndl, L"SensorTemperature", &temperature);
		cout << "Camera1: " << temperature << endl;
		AT_GetFloat(Camera2.Hndl, L"SensorTemperature", &temperature);
		cout << "Camera2: " << temperature << endl;
	}

	private: System::Void Sheet_Mode_box_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Camera_setting_but_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Camera_disconnection_but_Click(System::Object^  sender, System::EventArgs^  e) {
		AT_Close(Camera1.Hndl);
		AT_Close(Camera2.Hndl);
		AT_FinaliseLibrary();
	}

	private: System::Void button1_Click_5(System::Object^  sender, System::EventArgs^  e) {
		MOVIE_stats.restart_flag = 1;
	}
	private: System::Void exposure_time_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		double Exp = 200;
		exposure_time_box->Text = exposure_time_box->Text;
		if (double::TryParse(exposure_time_box->Text, Exp) == true) {
			ScanSpeed_Calc();
		}
	}
	private: System::Void camera_set_Click(System::Object^  sender, System::EventArgs^  e) {
		CameraSettings_Axial(Camera1.Hndl);
	}
	private: System::Void External_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (External->Checked == true) {

			ExternalExposure->Checked = false;

			Internal->Checked = false;

			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"TriggerMode", L"External");
			if (iret != AT_SUCCESS) {
				Console::WriteLine("Camera :  External setting error:  retcode = " + iret);
			}
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera :  External set");
			}
		}

	}
	private: System::Void ExternalExposure_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (ExternalExposure->Checked == true) {
			External->Checked = false;
			Internal->Checked = false;

			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"TriggerMode", L"External Exposure");
			if (iret != AT_SUCCESS) {
				Console::WriteLine("Camera :  External Exposure setting error:  retcode = " + iret);
			}
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera :  External Exposure set");
			}
		}

	}
	private: System::Void Internal_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Internal->Checked == true) {

			External->Checked = false;


			ExternalExposure->Checked = false;

			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"TriggerMode", L"Internal");
			if (iret != AT_SUCCESS) {
				Console::WriteLine("Camera :  Internal setting error:  retcode = " + iret);
			}
			if (iret == AT_SUCCESS) {
				Console::WriteLine("Camera :  Internal set");
			}
		}
	}
	private: System::Void C2OUT_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (C2OUT->Checked == true) {
			Top2Down->Checked = false;
			Down2Top->Checked = false;
			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"SensorReadoutMode", L"Centre Out Simultaneous");
			if (iret == AT_SUCCESS) {
				Console::WriteLine("SensorReadoutMode: Centre Out Simultaneous");
			}
			else {
				Console::WriteLine("SensorReadoutMode: Centre Out Simultaneous set error iret = " + iret);
			}
		}
	}
	private: System::Void Top2Down_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

		if (Top2Down->Checked == true) {
			C2OUT->Checked = false;
			Down2Top->Checked = false;
			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"SensorReadoutMode", L"Top Down Sequential");
			if (iret == AT_SUCCESS) {
				Console::WriteLine("SensorReadoutMode: Top Down Sequential");
			}
			else {
				Console::WriteLine("SensorReadoutMode: Top Down Sequential set error iret = " + iret);
			}
		}
	}
	private: System::Void Down2Top_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Down2Top->Checked == true) {
			C2OUT->Checked = false;
			Top2Down->Checked = false;
			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"SensorReadoutMode", L"Bottom Up Sequential");
			if (iret == AT_SUCCESS) {
				Console::WriteLine("SensorReadoutMode: Bottom Up Sequential");
			}
			else {
				Console::WriteLine("SensorReadoutMode: Bottom Up Sequential set error iret = " + iret);
			}
		}
	}
	private: System::Void ScanSpeed_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		double Exp = double::Parse(exposure_time_box->Text);
		double speed = double::Parse(ScanSpeed->Text);
		speed = speed * 1000;
		int iret = 0;

		if (Axial_mode_check->Checked == true) {
			int iret = AT_SetBool(Camera1.Hndl, L"ScanSpeedControlEnable", AT_TRUE);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("ScanSpeedControl  Enable");
			}
			else {
				Console::WriteLine("ScanSpeedControl set error: " + iret);
			}
			iret = AT_SetFloat(Camera1.Hndl, L"LineScanSpeed", speed);
			if (iret == AT_SUCCESS) {
				Console::WriteLine("LineScanSpeed set: " + speed);
			}
			else {
				Console::WriteLine("LineScanSpeed set :" + speed + "error: " + iret);
			}
		}
	}
	private: System::Void OverlapMode_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (OverlapMode->Checked == true) {
			int iret = AT_SetBool(Camera1.Hndl, L"Overlap", AT_TRUE);
			if (iret != AT_SUCCESS) {
				Console::WriteLine("Camera: Error setting Overlap mode, retcode = " + iret);
				OverlapMode->Checked = false;
			}
		}
		else {
			int iret = AT_SetBool(Camera1.Hndl, L"Overlap", AT_FALSE);
			if (iret != AT_SUCCESS) {
				Console::WriteLine("Camera Error setting Non Overlap mode, retcode = " + iret);

			}
		}
	}
	private: void GetCameraSettings(AT_H Hndl) {
		//cout << endl << "**********************************************************" << endl;
		//cout << "　　　　 GetCameraSettings";
		//cout << endl << "**********************************************************" << endl;
		int i_index;
		int iret = AT_GetEnumerated(Hndl, L"PixelReadoutRate", &i_index);
		PixelReadoutRate_box->Text = i_index.ToString();
		if (iret != AT_SUCCESS) {
			Console::WriteLine("Camera " + Hndl + ":Error getting PixelReadoutRate index " + iret);
		}
		else {
			//		Console::WriteLine("Camera " + Hndl + "PixelReadoutRate : " + i_index);
		}
		iret = AT_GetEnumIndex(Hndl, L"TriggerMode", &i_index);
		//cout << " TriggerMode: " << i_index << endl;
		Triggermode_box->Text = i_index.ToString();
		if (iret != AT_SUCCESS) {
			Console::WriteLine("Camera" + Hndl + " :Error getting TriggerMode index " + iret);
		}
		else {
			//		Console::WriteLine("Camera" + Hndl + " :TriggerMode : " + i_index);
		}
		double speed;
		iret = AT_GetFloat(Hndl, L"LineScanSpeed", &speed);
		//	Console::WriteLine("Camera" + Hndl + " :LineScanSpeed: " + speed.ToString("F3"));
		ScanSpeed_box->Text = speed.ToString("F3");
		AT_BOOL TF = 0;
		iret = AT_GetBool(Hndl, L"Overlap", &TF);
		//cout << " Overlap: " << TF << endl;
		if (TF == AT_TRUE) {
			overlapmode_box->Text = "ON";
		}
		else {
			overlapmode_box->Text = "OFF";
		}
		iret = AT_GetBool(Hndl, L"ScanSpeedControlEnable", &TF);
		if (TF == AT_TRUE) {
			Scanspeedcontrol_box->Text = "ON";
			//		cout << "ScanSpeedControlEnable:  true" << endl;
		}
		else {
			Scanspeedcontrol_box->Text = "OFF";
			//		cout << "ScanSpeedControlEnable:  false" << endl;
		}

		double Exp;
		iret = AT_GetFloat(Hndl, L"ExposureTime", &Exp);
		exposuretime_box->Text = Exp.ToString("F6");
		AT_WC Buf;
		iret = AT_GetEnumerated(Hndl, L"SensorReadoutMode", &i_index);
		//cout << "SensorReadoutMode: " << i_index << endl;
		iret = AT_GetEnumeratedString(Hndl, L"SensorReadoutMode", i_index, &Buf, 20);
		//cout << "SensorReadoutMode: " << Buf << endl;
		Shuttermode_box->Text = Buf.ToString();
		//	cout << endl << "**********************************************************" << endl;



	}
	private: System::Void Get_Camera_Condition_Click(System::Object^  sender, System::EventArgs^  e) {
		GetCameraSettings(Camera2.Hndl);
	}

	private: System::Void ScanSpeedControl_Check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (ScanSpeedControl_Check->Checked == true) {
			int iret = AT_SetBool(Camera1.Hndl, L"ScanSpeedControlEnable", AT_TRUE);
			if (iret == AT_SUCCESS) {
				cout << "ScanSpeedControlEnable" << endl;
			}
			else {
				cout << "ScanSpeedControlEnable error!!" << endl;
				ScanSpeedControl_Check->Checked = false;
			}
		}
		else {
			int iret = AT_SetBool(Camera1.Hndl, L"ScanSpeedControlEnable", AT_FALSE);
			if (iret == AT_SUCCESS) {
				cout << "ScanSpeedControl DisEnable" << endl;
			}
			else {
				cout << "ScanSpeedControl DisEnable error!!" << endl;
				ScanSpeedControl_Check->Checked = false;
			}
		}
	}
	private: System::Void Axial_mode_check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		double Exp;
		if (double::TryParse(exposure_time_box->Text, Exp) == true) {
			//double::TryParse(exposure_time_box->Text, Exp)
			Exp = Exp / 1000;
			int i_ret = AT_SetFloat(Camera1.Hndl, L"ExposureTime", Exp);
			ScanSpeed_Calc();
		}
		else {

		}
	}
	private: System::Void FastReadOut_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (FastReadOut->Checked == false) {
			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"PixelReadoutRate", L"100 MHz");
			if (iret == AT_SUCCESS) {
				Console::WriteLine("PixelReadoutRate: 100 MHz");
			}
			else {
				Console::WriteLine("PixelReadoutRate set error: 100 MHz  ");
			}
		}
		else {
			int iret = AT_SetEnumeratedString(Camera1.Hndl, L"PixelReadoutRate", L"280 MHz");
			if (iret == AT_SUCCESS) {
				Console::WriteLine("PixelReadoutRate: 280 MHz");
			}
			else {
				Console::WriteLine("PixelReadoutRate set error: 280 MHz  ");
			}
		}



	}
	private: System::Void eTLAxialToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		//string FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_488.csv";
		//Load_Axial_Param(FP,0);
		//string FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_592.csv";
		//Load_Axial_Param(FP,1);
		//string FP = "initial_path\\Axial_ETL_setfile\\Axial_Setting_647.csv";
		//Load_Axial_Param(FP,2);
	}
	private: System::Void Axial_488_load_Click(System::Object^  sender, System::EventArgs^  e) {


	}
	private: System::Void Axial_592_load_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Axial_647_load_Click(System::Object^  sender, System::EventArgs^  e) {

	}
	private: vector<double> ETL_Cals(int min, int max, int minpos, int maxpos) {
		return vector<double>{};
	}
	private: System::Void Axial_Param_set_Click(System::Object^  sender, System::EventArgs^  e) {
		if (illumination_flag == 'L')
		{
			if (Axial_panel->Text == "488") {
				Axial_ETL_param.Laser488[0] = double::Parse(ETL_box->Text);
			}
			if (Axial_panel->Text == "592") {
				Axial_ETL_param.Laser592[0] = double::Parse(ETL_box->Text);
			}
			if (Axial_panel->Text == "647") {
				Axial_ETL_param.Laser647[0] = double::Parse(ETL_box->Text);
			}
			ETL_L_left->Text = ETL_box->Text;
		}
		if (illumination_flag == 'R')
		{
			if (Axial_panel->Text == "488") {
				Axial_ETL_param.Laser488[2] = double::Parse(ETL_box->Text);
			}
			if (Axial_panel->Text == "592") {
				Axial_ETL_param.Laser592[2] = double::Parse(ETL_box->Text);
			}
			if (Axial_panel->Text == "647") {
				Axial_ETL_param.Laser647[2] = double::Parse(ETL_box->Text);
			}
			ETL_R_left->Text = ETL_box->Text;


		}
		DataChangeFlag = 1;
		ETLParam_save();
	}
	private:void ETLParam_vis() {
		if (illumination_flag == 'L')
		{
			if (Axial_panel->Text == "488") {
				ETL_bar->Value = Axial_ETL_param.Laser488[0] * 100;
				ETL_box->Text = Axial_ETL_param.Laser488[0].ToString();
				trackBar1->Value = Axial_ETL_param.Laser488[1] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser488[1].ToString();
			}
			if (Axial_panel->Text == "592") {
				ETL_bar->Value = Axial_ETL_param.Laser592[0] * 100;
				ETL_box->Text = Axial_ETL_param.Laser592[0].ToString();
				trackBar1->Value = Axial_ETL_param.Laser592[1] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser592[1].ToString();
			}
			if (Axial_panel->Text == "647") {
				ETL_bar->Value = Axial_ETL_param.Laser647[0] * 100;
				ETL_box->Text = Axial_ETL_param.Laser647[0].ToString();
				trackBar1->Value = Axial_ETL_param.Laser647[1] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser647[1].ToString();
			}
		}
		if (illumination_flag == 'R')
		{
			if (Axial_panel->Text == "488") {
				ETL_bar->Value = Axial_ETL_param.Laser488[2] * 100;
				ETL_box->Text = Axial_ETL_param.Laser488[2].ToString();
				trackBar1->Value = Axial_ETL_param.Laser488[3] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser488[3].ToString();
			}
			if (Axial_panel->Text == "592") {
				ETL_bar->Value = Axial_ETL_param.Laser592[2] * 100;
				ETL_box->Text = Axial_ETL_param.Laser592[2].ToString();
				trackBar1->Value = Axial_ETL_param.Laser592[3] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser592[3].ToString();
			}
			if (Axial_panel->Text == "647") {
				ETL_bar->Value = Axial_ETL_param.Laser647[2] * 100;
				ETL_box->Text = Axial_ETL_param.Laser647[2].ToString();
				trackBar1->Value = Axial_ETL_param.Laser647[3] * 100;
				ETL_box2->Text = Axial_ETL_param.Laser647[3].ToString();
			}
		}

	}
	private:void ETLParam_save() {
		String^ filename = "initial_path\\Axial_ETL_setfile\\ETLParam.txt";
		StreamWriter^ sw = gcnew StreamWriter(filename);
		sw->WriteLine(Axial_ETL_param.Laser488[0]);
		sw->WriteLine(Axial_ETL_param.Laser488[1]);
		sw->WriteLine(Axial_ETL_param.Laser488[2]);
		sw->WriteLine(Axial_ETL_param.Laser488[3]);
		sw->WriteLine(Axial_ETL_param.Laser592[0]);
		sw->WriteLine(Axial_ETL_param.Laser592[1]);
		sw->WriteLine(Axial_ETL_param.Laser592[2]);
		sw->WriteLine(Axial_ETL_param.Laser592[3]);
		sw->WriteLine(Axial_ETL_param.Laser647[0]);
		sw->WriteLine(Axial_ETL_param.Laser647[1]);
		sw->WriteLine(Axial_ETL_param.Laser647[2]);
		sw->WriteLine(Axial_ETL_param.Laser647[3]);
		sw->Close();
	}
	private:void Open_ETLParam() {
		String^ filename = "initial_path\\Axial_ETL_setfile\\ETLParam.txt";
		StreamReader^ din = File::OpenText(filename);
		String^ str;
		int i = 0;
		while ((str = din->ReadLine()) != nullptr) {
			Console::WriteLine(str);

			if (str == "") {
				continue;
			}
			if (i < 4) {
				Axial_ETL_param.Laser488[i] = double::Parse(str);
			}
			else if (i < 8) {
				Axial_ETL_param.Laser592[i - 4] = double::Parse(str);
			}
			else if (i < 12) {
				Axial_ETL_param.Laser647[i - 8] = double::Parse(str);
			}

			i++;
		}
		din->Close();
	}
	private: System::Void Axial_Param_set_R_Click(System::Object^  sender, System::EventArgs^  e) {
		if (illumination_flag == 'L')
		{
			if (Axial_panel->Text == "488") {
				Axial_ETL_param.Laser488[1] = double::Parse(ETL_box2->Text);
			}
			if (Axial_panel->Text == "592") {
				Axial_ETL_param.Laser592[1] = double::Parse(ETL_box2->Text);
			}
			if (Axial_panel->Text == "647") {
				Axial_ETL_param.Laser647[1] = double::Parse(ETL_box2->Text);
			}
			ETL_L_right->Text = ETL_box->Text;
		}
		if (illumination_flag == 'R')
		{
			if (Axial_panel->Text == "488") {
				Axial_ETL_param.Laser488[3] = double::Parse(ETL_box2->Text);
			}
			if (Axial_panel->Text == "592") {
				Axial_ETL_param.Laser592[3] = double::Parse(ETL_box2->Text);
			}
			if (Axial_panel->Text == "647") {
				Axial_ETL_param.Laser647[3] = double::Parse(ETL_box2->Text);
			}
			ETL_R_right->Text = ETL_box->Text;
		}

		DataChangeFlag = 1;
	}

	private: System::Void textBox2_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void label121_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void tabPage3_Click(System::Object^  sender, System::EventArgs^  e) {
	}


	private: System::Void AF_Click(System::Object^  sender, System::EventArgs^  e) {

		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::MOVIEFocus_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();


	}
	private: System::Void trackBar_Z_left_Scroll(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void trackBar_Z_right_Scroll(System::Object^  sender, System::EventArgs^  e) {

	}

	private: void Z_galvo_initialization() {
		cout << "Z_galvo_initialization " << endl;
		double v;
		bool ret;
		char errBuff[2304] = { '\0' };
		if (illumination_flag == 'R')
			v = MOVIE_stats.InitialFocus_R;
		else
			v = MOVIE_stats.InitialFocus_L;

		Z_Galvo(v);

	}

	private: System::Void Z_left_offset_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Z_right_offset_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void Camera_Click(System::Object^  sender, System::EventArgs^  e) {
	}



	private: System::Void Windowsize_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		double Window = 10;
		if (double::TryParse(windowsize_box->Text, Window) == true) {
			double Window = 2304 / (double::Parse(windowsize_box->Text));
			double Exp = double::Parse(exposure_time_box->Text);
			Exp = Exp / 1000.0;
			if (Camera_1_use->Checked == true) {
				if (Axial_mode_check->Checked == true) {
					int iret = 0;
					if (iret == AT_SUCCESS) {
						Console::WriteLine("Camera " + Camera1.Hndl + "ExposedPixelHeight set " + Window + "pixels");
					}
					else {
						Console::WriteLine("Camera " + Camera1.Hndl + "ExposedPixelHeight set error " + Window + "pixels iret = " + iret);
					}

					iret = AT_SetFloat(Camera1.Hndl, L"ExposureTime", Exp *Window / 2304);
					if (iret == AT_SUCCESS) {
						Console::WriteLine("Camera " + Camera1.Hndl + "Exposure Time set: " + Exp * Window / 2304 + " s");
						Console::WriteLine("Camera " + Camera1.Hndl + "Window size: " + Window + " pixel");
					}
				}
			}
			if (Camera_2_use->Checked == true) {

				if (Axial_mode_check->Checked == true) {
					int iret = 0;

					if (iret == AT_SUCCESS) {
						Console::WriteLine("Camera " + Camera2.Hndl + "ExposedPixelHeight set " + Window + "pixels");
					}
					else {
						Console::WriteLine("Camera " + Camera2.Hndl + "ExposedPixelHeight set error " + Window + "pixels iret = " + iret);
					}

					iret = AT_SetFloat(Camera2.Hndl, L"ExposureTime", Exp *Window / 2304);
					if (iret == AT_SUCCESS) {
						Console::WriteLine("Camera " + Camera1.Hndl + "Exposure Time set: " + Exp * Window / 2304 + " s");
						Console::WriteLine("Camera " + Camera1.Hndl + "Window size: " + Window + " pixel");
					}
				}
			}
		}
	}

	private: System::Void filter1_but_Click(System::Object^  sender, System::EventArgs^  e) {
		Flipper_Change(TSerials.FilterFlipper_Serial, x10Filterpos, 1);
	}
	private: System::Void filter2_but_Click(System::Object^  sender, System::EventArgs^  e) {
		Flipper_Change(TSerials.FilterFlipper_Serial, x10Filterpos, 2);
	}
	private: System::Void GetFilterPos_Click(System::Object^  sender, System::EventArgs^  e) {
		cout << Flipper_GetPos(TSerials.FilterFlipper_Serial) << endl;
	}
	private: System::Void MF_Sobel_check_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void X_tile_num_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Y_tile_num_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void GotoCenter_but_Click(System::Object^  sender, System::EventArgs^  e) {
		double x_target = (int::Parse(left_limit->Text) + int::Parse(right_limit->Text)) / 2;
		double y_target = (int::Parse(top_limit->Text) + int::Parse(bottom_limit->Text)) / 2;

		double z_overlap = double::Parse(Z_overlap_box->Text);
		double z_target = double::Parse(Start_Z_pos->Text) - z_overlap;

		z_target = z_target * MicroStepZ;
		String^ x_Target = x_target.ToString();
		String^ y_Target = y_target.ToString();
		String^ z_Target = z_target.ToString();

		XY_stage_flag = 1;
		XYstage_port->Write("AXIX:GOABS " + x_Target + "\r");
		Sleep(100);
		XYstage_port->Write("AXIY:GOABS " + y_Target + "\r");
		Sleep(50);
		XY_stage_flag = 0;
		Zstage_Flag = 1;
		Sleep(50);
		//cout << "AF: Z_stage_GoTo " << z_target / 20 << endl;
		MOVIE_stats.stage = 1;
		Zstage_port->WriteLine(STX + "APS2/5/" + z_Target + "/1\r");
		Sleep(50);
		Zstage_Flag = 0;



	}

	private: System::Void MOVIE_skip_exp_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		//float exp = 50;
		//float step = 5;
		//float::TryParse(MOVIE_skip_step_box->Text, step);
		//float::TryParse(MOVIE_skip_exp_box->Text, exp);
		//step = 1250 / exp;
		//MOVIE_skip_step_box->Text = (exp / 2.4).ToString("F0");
	}
	private: System::Void Shutter_Close_Click(System::Object^  sender, System::EventArgs^  e) {
		Shutter_Close_func(TSerials.Shutter_Serial, Shutter_state);

	}
	private: System::Void Shutter_Open_Click(System::Object^  sender, System::EventArgs^  e) {
		Shutter_Open_func(TSerials.Shutter_Serial, Shutter_state);
	}

	private: System::Void HOME_Z_Click(System::Object^  sender, System::EventArgs^  e) {
		CheckWindow^ frm = gcnew CheckWindow();
		// モーダル表示
		frm->ShowDialog(this);
		frm->Close();

		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "ORG2/0/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void HOME_XY_Click(System::Object^  sender, System::EventArgs^  e) {
		CheckWindow^ frm = gcnew CheckWindow();
		// モーダル表示
		frm->ShowDialog(this);
		frm->Close();
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIY:GO HOME\r");
		Sleep(50);
		XYstage_port->Write("AXIX:GO HOME\r");
		XY_stage_flag = 0;
	}

	private: System::Void dual_main1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void dual_main2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	}

	private: System::Void Go_Rev_Center_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void GetCenter_Click(System::Object^  sender, System::EventArgs^  e) {
		//x_center_box->Text = ((int::Parse(left_limit->Text) + int::Parse(right_limit->Text)) / 2 + offset).ToString();
		//y_center_box->Text = ((int::Parse(top_limit->Text) + int::Parse(bottom_limit->Text)) / 2).ToString();
		/*
		String^ Dir = Export_Box->Text;
		x_center_box->Text = X_pos_current->Text;
		y_center_box->Text = Y_pos_current->Text;
		StreamReader^ din = File::OpenText(Dir + "\\PreScan\\PreScan_FW_log.txt");
		String^ str;
		int count = 0;
		double x, y, zstart, zend, zstep;
		while ((str = din->ReadLine()) != nullptr) {
			if (count == 0) {
				x = double::Parse(str);
			}
			if (count == 1) {
				y = double::Parse(str);
			}
			if (count == 2) {
				zstart = double::Parse(str);
				Start_Z_pos->Text = zstart.ToString();
			}
			if (count == 3) {
				zend = double::Parse(str);
				End_Z_pos->Text = zend.ToString();
			}
			if (count == 4) {
				zstep = double::Parse(str);
			}
			count++;
		}
		din->Close();
		double macro_pixel = PIXELSIZE / 0.59194 / 0.63 * 4;
		double micro_pixel = PIXELSIZE / 9.88;
		zstep = abs(zstart - zend) / (double)PreScanImageNum;
		double imagewindow = (2304.0 - 200)* PIXELSIZE / MAG / 4;
		int Xoffset = int::Parse(xoffset->Text) * macro_pixel / 4;
		int Yoffset = int::Parse(yoffset->Text) * macro_pixel / 4;
		x = double::Parse(X_pos_current->Text);
		y = double::Parse(Y_pos_current->Text);

		double delta_y = double::Parse(y_center_box_fw->Text) - double::Parse(y_center_box->Text);
		z_limit_RV->Text = Z_pos_current->Text;
		double x_FW_dis = (int::Parse(left_limit->Text) - int::Parse(right_limit->Text)) / 2;
		//double offset = double::Parse(theta_offset->Text);
		Start_Z_pos->Text = (double::Parse(z_limit_RV->Text) + double::Parse(Z_overlap_box->Text)).ToString();
		left_limit_RV->Text = ((((int)(x + Xoffset))) + 500).ToString();
		right_limit_RV->Text = (((int)(x - macro_pixel * 511 + imagewindow + Xoffset) - 500)).ToString();
		top_limit_RV->Text = ((int)(y - Yoffset) - 500).ToString();
		bottom_limit_RV->Text = (((int)(y + macro_pixel * 511 - imagewindow - Yoffset) + 500)).ToString();
		*/
	}
	private: System::Void X_pos_current_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {

		if (e->KeyCode == Keys::Enter) {
			String^ z = TargetX_box->Text;
			string Z;
			MarshalString(z, Z);
			if (check_int(Z) == true) {
				XY_stage_flag = 1;
				Sleep(50);
				XYstage_port->Write("AXIX:GOABS " + TargetX_box->Text + "\r");
				Sleep(50);
				XY_stage_flag = 0;
				//Sleep(1000);
				//XYstage_port->Write("AXIX:POS?\r");
			}
		}


	}
	private: System::Void Y_pos_current_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			String^ z = TargetY_box->Text;
			string Z;
			MarshalString(z, Z);
			if (check_int(Z) == true) {
				XY_stage_flag = 1;
				Sleep(50);
				//Running_LOG("X stagte moving Start");
				XYstage_port->Write("AXIY:GOABS " + TargetY_box->Text + "\r");
				Sleep(50);
				XY_stage_flag = 0;
			}
		}
	}
	private: System::Void XY_QPOS_Click(System::Object^  sender, System::EventArgs^  e) {
		XYstage_port->Write("AXIX:POS?\r:AXIY:POS?\r");
		//cout << "AXIX:POS?:AXIY:POS?\r" << endl;
		//XYstage_port->Write("AXIY:POS?\r");

	}

	private: System::Void YHOME_Click(System::Object^  sender, System::EventArgs^  e) {
		XY_stage_flag = 1;
		Sleep(50);
		//Running_LOG("X stagte moving Start");
		XYstage_port->Write("AXIY:GOABS 0\r");
		Sleep(50);
		XY_stage_flag = 0;

	}
	private: System::Void Laser_R_Click(System::Object^  sender, System::EventArgs^  e) {
		Laser_R_ON();
	}
	private: System::Void Laser_L_Click(System::Object^  sender, System::EventArgs^  e) {
		Laser_L_ON();

	}
	private: void  Zsweep_thread() {
		bool ret;
		int FW = 0;
		double pos = 0;
		char		errBuff[2048] = { '\0' };
		while (Abort_flag != 1) {
			Z_Galvo(pos);
			Sleep(10);
			if (FW == 0) {
				pos += 0.1;
			}
			else {
				pos -= 0.1;
			}
			if (pos > 1)
				FW = -1;
			if (pos < -1) {
				FW = 0;
			}

		}
		pos = 0;
		Z_Galvo(pos);
	}
	private: System::Void Zweep_Click(System::Object^  sender, System::EventArgs^  e) {
		Abort_flag = 0;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Zsweep_thread);
		Thread^ thread = gcnew Thread(start);
		thread->Start();

	}
	private: System::Void L_gar_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Sheet_Box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (illumination_flag == 'R')
			sheet_offset_R->Text = Sheet_Box->Text;
		else if (illumination_flag == 'L')
			sheet_offset_L->Text = Sheet_Box->Text;
		DataChangeFlag = 1;
	}
	private: System::Void Sheet_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		int v = Sheet_bar->Value;

		Sheet_Box->Text = ((float)v / 100.0 - 5.0).ToString();
	}
	private: System::Void Sheet_Step_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void down_1_MouseDownn(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	}
	private: System::Void ETL_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		ETL_box->Text = ((float)ETL_bar->Value / 100).ToString("F2");
	}
	private: System::Void trackBar_Z_Scroll(System::Object^  sender, System::EventArgs^  e) {
		double Value = double(trackBar_Z->Value);
		Z_offset->Text = ((Value - 10000) / 10).ToString("F1");
	}
	private: System::Void trackBar1_Scroll(System::Object^  sender, System::EventArgs^  e) {
		ETL_box2->Text = ((float)trackBar1->Value / 100).ToString("F2");
	}
	private: System::Void ETL_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		DataChangeFlag = 1;
	}
	private: System::Void ETL_box2_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		DataChangeFlag = 2;
	}

	private:void update_z_galv() {
		double v = 0;
		bool ret;
		string dev;
		double  spos;
		if (Acq_flag != 1) {
			if (double ::TryParse(s_pos_current->Text, spos)) {
				if (abs(spos) < 150) {
					if (illumination_flag == 'L') {
						dev = Z_GALV_DAQ_L;
						Z_offset_current->Text = Z_galv_L->Text;
					}
					else {
						dev = Z_GALV_DAQ_R;
						Z_offset_current->Text = Z_galv_R->Text;
					}
				}
				else {
					if (illumination_flag == 'L') {
						dev = Z_GALV_DAQ_L;
						Z_offset_current->Text = Z_galv_L_RV->Text;
					}
					else {
						dev = Z_GALV_DAQ_R;
						Z_offset_current->Text = Z_galv_R_RV->Text;
					}
				}
			}
		}
		if (double::TryParse(Z_offset_current->Text, v)) {
			cout << "update z galv to: " << v << endl;
			Z_Galvo(v / 1000);
		}
	}
	private: System::Void Z_offset_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		//if (Acq_flag == 0 && LIVE_flag == 0) {
		if (Acq_flag == 0) {
			double v[2] = { 0 };
			bool ret;
			string dev;
			double  spos;
			if (double::TryParse(Z_offset->Text, v[0])) {
				v[0] = v[0] / 1000;
				v[1] = v[0];
				//v = v + 3;
				if (double ::TryParse(s_pos_current->Text, spos)) {
					if (abs(spos) < 150) {
						if (illumination_flag == 'L') {
							dev = Z_GALV_DAQ_L;
							Z_galv_L->Text = Z_offset->Text;
						}
						else {
							dev = Z_GALV_DAQ_R;
							Z_galv_R->Text = Z_offset->Text;
						}
					}
					else {
						if (illumination_flag == 'L') {
							dev = Z_GALV_DAQ_L;
							Z_galv_L_RV->Text = Z_offset->Text;
						}
						else {
							dev = Z_GALV_DAQ_R;
							Z_galv_R_RV->Text = Z_offset->Text;
						}
					}
				}
				else {
					cout << "Error s_pos_current text" << endl;
				}
				update_z_galv();
			}
		}
		else {
			cout << "Acq_flag == 1" << endl;
		}

	}



	private:void MOVE_stage_func() {
		TargetX_box->Text = Stagepos_target.x.ToString();
		Z_target_pos->Text = Stagepos_target.y.ToString();
		TargetY_box->Text = Stagepos_target.z.ToString();
		if (Stagepos_target.move_x == true) {
			XY_stage_flag = 1;
			Sleep(50);
			cout << "X stage goto :" << Stagepos_target.x << endl;
			XYstage_port->Write("AXIX:SELSP 5:GOABS " + Stagepos_target.x.ToString() + "\r");
			Sleep(50);
			XY_stage_flag = 0;
			Stagepos_target.move_x = false;
		}
		if (Stagepos_target.move_z == true) {
			XY_stage_flag = 1;
			Sleep(50);
			cout << "Z stage goto :" << Stagepos_target.z << endl;
			XYstage_port->Write("AXIY:SELSP 5:GOABS " + Stagepos_target.z.ToString() + "\r");
			Sleep(50);
			XY_stage_flag = 0;
			Stagepos_target.move_z = false;
		}
		if (Stagepos_target.move_y == true) {
			Zstage_Flag = 1;
			Sleep(50);
			cout << "Y stage goto :" << Stagepos_target.y << endl;
			int y = Stagepos_target.y * MicroStepZ;
			Zstage_port->WriteLine(STX + "APS2/3/" + (y).ToString() + "/1\r");
			Sleep(50);
			Zstage_Flag = 0;
			Stagepos_target.move_y = false;
		}
		if (Stagepos_target.move_s == true) {
			Zstage_Flag = 2;
			Sleep(50);
			cout << "S stage goto :" << Stagepos_target.s << endl;
			int s = Stagepos_target.y * MicroStepZ;
			Zstage_port->WriteLine(STX + "APS1/3/" + (s).ToString() + "/1\r");
			Sleep(50);
			while (stage_stat.s == 1) {
				Zstage_port->WriteLine(STX + "STR1\r");
				Sleep(50);
			}
			Zstage_Flag = 0;
			Stagepos_target.move_s = false;
		}
		Sleep(200);
	}

	private: System::Void gotocenter_but_Click_1(System::Object^  sender, System::EventArgs^  e) {
		double x = (double::Parse(left_limit->Text) + double::Parse(right_limit->Text)) / 2;
		double y = (double::Parse(top_limit->Text) + double::Parse(bottom_limit->Text)) / 2;
		double z;
		if (double::Parse(Start_Z_pos->Text) > double::Parse(End_Z_pos->Text))
			z = double::Parse(Start_Z_pos->Text) - double::Parse(Z_overlap_box->Text);
		else {
			z = double::Parse(Start_Z_pos->Text) + double::Parse(Z_overlap_box->Text);
		}
		Stagepos_target.x = x;
		Stagepos_target.y = y;
		Stagepos_target.z = z;
		Stagepos_target.move_s = false;
		Stagepos_target.move_y = true;
		Stagepos_target.move_x = true;
		Stagepos_target.move_z = true;
		//MOVE_stage_func();
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::MOVE_stage_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private:int CheckXYpos() {
		if ((int)Stagepos_current.x != (int)Stagepos_target.x)
			return -1;
		if ((int)Stagepos_current.z != (int)Stagepos_target.z)
			return -1;
		return 1;
	}
	private:int CheckSpos(float t) {
		float s = float::Parse(s_pos_target->Text);
		if ((int)Stagepos_current.s == (int)t)
			return 1;
		return -1;
	}

	private:void Rotation_func(double target) {
		cout << "Theta stage go to " << target << endl;
		double x = (double::Parse(Theta_POS_X->Text));
		double z = (double::Parse(Theta_POS_Z->Text));
		Stagepos_target.x = x;
		Stagepos_target.z = z;
		Stagepos_target.move_s = false;
		Stagepos_target.move_y = false;
		Stagepos_target.move_x = true;
		Stagepos_target.move_z = true;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::MOVE_stage_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
		Sleep(2000);
		int ret = 0;
		while (1) {
			ret = CheckXYpos();
			if (ret == 1)
				break;
		}
		ThetaMove(target);
		Sleep(2000);
		while (1) {
			ret = CheckSpos(target);
			if (ret == 1)
				break;
		}

	}
	private:void Reverse_func() {
		Rotation_func(float::Parse(ReverseSpos->Text));
	}

	private: System::Void Reverse_but_Click(System::Object^  sender, System::EventArgs^  e) {
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::Reverse_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}
	private: System::Void Set_RV_but_Click(System::Object^  sender, System::EventArgs^  e) {
		double x_center = double::Parse(X_pos_current->Text);
		double y_center = double::Parse(Z_pos_current->Text);
		double x_left_fw = double::Parse(left_limit->Text);
		double x_right_fw = double::Parse(right_limit->Text);
		double y_top_fw = double::Parse(top_limit->Text);
		double y_bottom_fw = double::Parse(bottom_limit->Text);

		double deltax = x_center - (x_left_fw + x_right_fw) / 2;
		double deltay = y_center - (y_top_fw + y_bottom_fw) / 2;


		top_limit_RV->Text = ((int)(y_top_fw + deltay)).ToString();
		bottom_limit_RV->Text = ((int)(y_bottom_fw + deltay)).ToString();
		left_limit_RV->Text = ((int)(x_left_fw + deltax)).ToString();
		right_limit_RV->Text = ((int)(x_right_fw + deltax)).ToString();

		ReverseSpos->Text = s_pos_current->Text;

	}
	private:void ThetaMove(float S) {
		Zstage_Flag = 1;
		int s_Target2 = S * MicroStepS;
		Sleep(50);
		Running_LOG("S stagte moving Start");
		cout << "S stagte moving Start to " << s_Target2 << endl;
		Zstage_port->WriteLine(STX + "APS1/4/" + s_Target2.ToString() + "/1\r");
		Sleep(50);
		Zstage_Flag = 0;
	}

	private: System::Void s_pos_target_KeyDown_1(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
		if (e->KeyCode == Keys::Enter) {
			//String^ s = s_pos_target->Text;
			//string S;
			float S;
			if (float::TryParse(s_pos_target->Text, S)) {
				ThetaMove(S);
			}
		}
	}

	private: System::Void Theta_zero_Click(System::Object^  sender, System::EventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "WRP1/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void z_zero_Click(System::Object^  sender, System::EventArgs^  e) {

		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "WRP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private:void SaveStage() {
		Zstage_port->WriteLine(STX + "SAV\r");
	}
	private: System::Void ETL_L_left_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Acq_flag == 0)
			DataChangeFlag = 1;
	}
	private: System::Void ETL_R_left_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Acq_flag == 0)
			DataChangeFlag = 1;
	}
	private: System::Void ETL_L_right_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Acq_flag == 0)
			DataChangeFlag = 1;
	}
	private: System::Void ETL_R_right_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (Acq_flag == 0)
			DataChangeFlag = 1;
	}

	private:void Theta_func(Object^ V) {
		double v = (double)V;
		Zstage_Flag = 2;
		stage_stat.s = 1;
		Sleep(50);
		Console::WriteLine("RPS1/4/" + v.ToString() + "/0");
		Zstage_port->WriteLine(STX + "RPS1/4/" + v.ToString() + "/0\r");
		Sleep(50);
		Zstage_Flag = 10;
		/*while (stage_stat.s == 1) {
			Zstage_port->WriteLine(STX + "STR1\r");
			Sleep(50);
		}
		Read_S_Pos();*/
	}
	private:void Theta_MOVETheta_MOVE(double v) {
		ParameterizedThreadStart^ start = gcnew ParameterizedThreadStart(this, &MyForm::Theta_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start(v);
	}
	private: System::Void Theta_ini_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		/*Sleep(2000);
		Zstage_Flag = 0;
		Zstage_port->WriteLine(STX + "ORG1/4/0\r");
		while (stage_stat.s == 1) {
			Zstage_port->WriteLine(STX + "STR1\r");
			Sleep(50);
		}
		double v = -90 / Theta_deg;
		Theta_MOVE(v);
		*/
	}
	private: System::Void windowsize_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		double v;
		if (double::TryParse(windowsize_box->Text, v) == true)
			ScanSpeed_Calc();
	}
	private: System::Void delay_time_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

		double v;
		if (double::TryParse(delay_time_box->Text, v) == true)
			ScanSpeed_Calc();
	}
	private: System::Void BW_ETL_check_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
			Sleep(100);
		}
	}
	private: void STOP_Stage() {

		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "STP2/0\r");
		Sleep(50);
		Zstage_Flag = 0;
		XY_stage_flag = 1;
		Sleep(50);
		XYstage_port->Write("AXIX:STOP\r");
		XYstage_port->Write("AXIY:STOP\r");
		Sleep(50);
		XY_stage_flag = 0;
	}


	private: System::Void top_limit_but_Click_1(System::Object^  sender, System::EventArgs^  e) {

		top_limit->Text = Z_pos_current->Text;
	}
	private: System::Void left_limit_but_Click_1(System::Object^  sender, System::EventArgs^  e) {
		bottom_limit->Text = Z_pos_current->Text;
	}
	private: System::Void Y_limit_but_Click_1(System::Object^  sender, System::EventArgs^  e) {
		left_limit->Text = X_pos_current->Text;
	}
	private: System::Void right_limit_but_Click_2(System::Object^  sender, System::EventArgs^  e) {
		right_limit->Text = X_pos_current->Text;
	}
	private:void CheckOFF(int n) {
		for (int i = 0; i < 6; ++i) {
			cout << "Check off" << i << "/" << n << endl;
			if (i != n) {
				if (i == 0)
					MicroStep1->Checked == false;
				if (i == 1)
					MicroStep2->Checked == false;
				if (i == 2)
					MicroStep4->Checked == false;
				if (i == 3)
					MicroStep8->Checked == false;
				if (i == 4)
					MicroStep10->Checked == false;
				if (i == 5)
					MicroStep20->Checked == false;
			}
		}
	}
	private:void StepChange(int n) {
		Console::WriteLine(STX + "WSY2/66/" + n.ToString() + "\r");
		Zstage_port->WriteLine(STX + "WSY2/66/" + n.ToString() + "\r");
	}
	private: System::Void MicroStep1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep1->Checked == true) {
			CheckOFF(0);
			StepChange(0);
			MicroStepZ = 1;
		}
	}
	private: System::Void MicroStep2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep2->Checked == true) {
			CheckOFF(1);
			StepChange(1);
			MicroStepZ = 2;
		}
	}
	private: System::Void MicroStep4_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep4->Checked == true) {
			CheckOFF(2);
			StepChange(2);
			MicroStepZ = 4;
		}
	}
	private: System::Void MicroStep8_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep8->Checked == true) {
			CheckOFF(3);
			StepChange(3);
			MicroStepZ = 8;
		}
	}
	private: System::Void MicroStep10_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep10->Checked == true) {
			CheckOFF(4);
			StepChange(4);
			MicroStepZ = 10;
		}
	}
	private: System::Void MicroStep20_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (MicroStep20->Checked == true) {
			CheckOFF(5);
			StepChange(5);
			MicroStepZ = 20;
		}
	}
	private: System::Void Theta_Pos_Set_Click(System::Object^  sender, System::EventArgs^  e) {
		Theta_POS_X->Text = X_pos_current->Text;
		Theta_POS_Z->Text = Y_pos_current->Text;

	}
	private: System::Void z_galv_set_rv_Click(System::Object^  sender, System::EventArgs^  e) {
		if (illumination_flag == 'L') {
			Z_galv_L_RV->Text = Z_offset->Text;
		}
		else if (illumination_flag == 'R') {
			Z_galv_R_RV->Text = Z_offset->Text;
		}
	}
	private: System::Void groupBox16_Enter(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Start_Z_but_RV_Click(System::Object^  sender, System::EventArgs^  e) {
		float F = float::Parse(Y_pos_current->Text);
		int S = (int)F;
		Start_Z_pos_RV->Text = S.ToString();
	}
	private: System::Void End_Z_but_RV_Click(System::Object^  sender, System::EventArgs^  e) {
		float F = float::Parse(Y_pos_current->Text);
		int S = (int)F;
		End_Z_pos_RV->Text = S.ToString();
	}

	private: System::Void Extra_time_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {

		double v;
		if (double::TryParse(Extra_time_box->Text, v) == true) {
			EXTRA_TIME = v;
			ScanSpeed_Calc();

		}

	}
	private: System::Void TargetY_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
			 cli::array<String^>^ AA;
	private: int Ser() {
		int port;
		cout << "input com" << endl;
		cin >> port;
		//以下ループ
		if (port < AA->Length)
			Console::WriteLine(AA[port]);

		return 1;
	}

	private: System::Void SheetSPD_bar_Scroll(System::Object^  sender, System::EventArgs^  e) {
		SheetSPD->Text = (1 + ((10000.0 - (double)SheetSPD_bar->Value) / 100000.0)).ToString("F6");
		cout << 1 + (10000.0 - (double)SheetSPD_bar->Value) / 100000.0 << endl;
	}

	private:void XMOVE_func() {
		double L = double::Parse(LL->Text->ToString());
		double R = double::Parse(RR->Text->ToString());
		while (Abort_flag == 0) {
			Stagepos_target.x = L;
			Stagepos_target.move_x = true;
			Stagepos_target.move_y = false;
			Stagepos_target.move_z = false;
			Stagepos_target.move_s = false;
			MOVE_stage_func();
			while (1) {
				if (abs(Stagepos_target.x - double::Parse(X_pos_current->Text)) < 10)
				{
					cout << "\r" << abs(Stagepos_target.x - double::Parse(X_pos_current->Text));
					break;
				}
				Sleep(100);
			}
			Sleep(100);
			Stagepos_target.x = R;
			Stagepos_target.move_x = true;
			Stagepos_target.move_y = false;
			Stagepos_target.move_z = false;
			Stagepos_target.move_s = false;
			Sleep(1000);
			MOVE_stage_func();
			while (1) {
				if (abs(Stagepos_target.x - double::Parse(X_pos_current->Text)) < 10)
				{
					cout << "\r" << abs(Stagepos_target.x - double::Parse(X_pos_current->Text));
					break;
				}
				Sleep(100);
			}
			Sleep(1000);

		}
	}
	private: System::Void X_LR_Click(System::Object^  sender, System::EventArgs^  e) {
		Abort_flag = 0;
		ThreadStart^ start = gcnew ThreadStart(this, &MyForm::XMOVE_func);
		Thread^ thread = gcnew Thread(start);
		thread->Start();
	}


	private: System::Void AxialAF_but_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void z_right_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void filter_name2_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void filter_name1_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void backgroundWorker_L488_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		while (Close_Flag == 0) {
			Laser_Info.L488.Q = 1;
			Laser_488->Write("l?\r");
			Sleep(100);
			Laser_Info.L488.Q = 2;
			Laser_488->Write("pa?\r");
			Sleep(100);
			while (Laser_Info.L488.stat_Q == 1 && Laser_Info.L488.stat == false) {
				Laser_Info.L488.Q = 1;
				Laser_488->Write("l1\r");
				Sleep(100);
				Laser_Info.L488.Q = 1;
				Laser_488->Write("l?\r");
				if (Close_Flag == 1)
					break;
				Sleep(100);
			}
			while (Laser_Info.L488.stat_Q == 0 && Laser_Info.L488.stat == true) {
				Laser_Info.L488.Q = 1;
				Laser_488->Write("l0\r");
				Sleep(100);
				Laser_Info.L488.Q = 1;
				Laser_488->Write("l?\r");
				if (Close_Flag == 1)
					break;
				Sleep(100);
			}
			float LP_target;
			if (float ::TryParse(L488_box->Text, LP_target) == true) {
				LP_target = LP_target / 1000;
				Laser_488->Write("p " + LP_target.ToString() + "\r");
			}
			Sleep(100);
			Laser_Info.L488.Q = 8;
			Laser_488->Write("rbpt?\r");
			Sleep(100);
		}
		Laser_Info.L488.stat = false;
		Sleep(50);
		Laser_488->Write("l0\r");
		Sleep(50);
		Laser_488->Write("l0\r");
	}
	private: System::Void backgroundWorker_L592_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		float LP;
		Sleep(100);
		while (Close_Flag == 0) {
			Laser_Info.L592.Q = 2;
			Sleep(100);
			Laser_592->Write("Power 0\r");
			Sleep(200);
			Laser_Info.L592.Q = 1;
			Sleep(100);
			Laser_592->Write("getLDenable\r");
			Sleep(200);
			// cout << Laser_Info.L592.Q << "/" << Laser_Info.L592.stat << endl;
			if (Laser_Info.L592.stat_Q == 1 && Laser_Info.L592.stat == false) {
				Laser_Info.L592.Q = 1;
				Sleep(100);
				Laser_592->Write("setLDenable 1\r");
			}
			if (Laser_Info.L592.stat_Q == 0 && Laser_Info.L592.stat == true) {
				Laser_Info.L592.Q = 1;
				Sleep(100);
				Laser_592->Write("setLDenable 0\r");
			}
			Laser_Info.L592.Q = 2;
			Sleep(100);
			if (float ::TryParse(L592_box->Text, LP) == true) {
				if (LP > 500)
					LP = 500;
				if (LP < 50)
					LP = 50;
				LP = LP;
				Laser_592->Write("Setpower 0 " + int(LP).ToString() + "\r");
			}
			Sleep(200);
			Laser_Info.L592.Q = 8;
			Sleep(100);
			Laser_592->Write("LDtemp 1\r");
			Sleep(200);
		}
		Laser_Info.L592.stat = false;
		Sleep(200);
		Laser_592->Write("setLDenable 0\r");
		Sleep(200);
		Laser_592->Write("setLDenable 0\r");
		Sleep(200);
		Laser_592->Write("setLDenable 0\r");
		Sleep(200);
		Laser_592->Write("setLDenable 0\r");
	}
	private: System::Void backgroundWorker_L647_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		while (Close_Flag == 0) {
			Laser_Info.L647.Q = 1;
			Laser_647->Write("l?\r");
			Sleep(100);
			Laser_Info.L647.Q = 2;
			Laser_647->Write("pa?\r");
			Sleep(100);
			while (Laser_Info.L647.stat_Q == 1 && Laser_Info.L647.stat == false) {
				Laser_Info.L647.Q = 1;
				Laser_647->Write("l1\r");
				Sleep(100);
				Laser_Info.L647.Q = 1;
				Laser_647->Write("l?\r");
				if (Close_Flag == 1)
					break;
				Sleep(100);
			}
			while (Laser_Info.L647.stat_Q == 0 && Laser_Info.L647.stat == true) {
				Laser_Info.L647.Q = 1;
				Laser_647->Write("l0\r");
				Sleep(100);
				Laser_Info.L647.Q = 1;
				Laser_647->Write("l?\r");
				if (Close_Flag == 1)
					break;
				Sleep(100);
			}
			float LP_target;
			if (float ::TryParse(L647_box->Text, LP_target) == true) {
				LP_target = LP_target / 1000;
				Laser_647->Write("p " + LP_target.ToString() + "\r");
			}
			Sleep(100);
			Laser_Info.L647.Q = 8;
			Laser_647->Write("rbpt?\r");
			Sleep(100);
		}
		Sleep(200);
		Laser_Info.L647.stat = false;
		Sleep(200);
		Laser_647->Write("l0\r");
		Sleep(200);
		Laser_647->Write("l0\r");
	}
	private: System::Void backgroundWorker1_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		while (1) {
			HWND hWnd = ::FindWindow(NULL, "GPS Imager");
			if (hWnd) {
				break;
			}
		}
		Sleep(1000);

		while (Close_Flag == 0) {
			unsigned long long data = 1000000000000;
			data = data + threads.MOVIE_Thread * 100000000000;
			data = data + threads.Camera * 10000000000;
			data = data + threads.CameraSetting * 1000000000;
			data = data + threads.Acquisition * 100000000;
			data = data + threads.DataExport * 10000000;
			data = data + threads.MOVIE_focus * 1000000;
			data = data + threads.MOVIE_skip * 100000;
			data = data + threads.MOVIE_skip_calc * 10000;
			data = data + threads.MOVIE_scan * 1000;
			data = data + threads.AF * 100;
			data = data + threads.DaqPrep * 10;
			data = data + threads.Daq * 1;
			ThreadCondition->Text = data.ToString();
			Sleep(200);
		}

	}
	private: System::Void SkipTile_Click(System::Object^  sender, System::EventArgs^  e) {
		MOVIE_stats.Skiptile = true;
		Orca_Stop();
	}
	private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
		Zstage_Flag = 1;
		Sleep(50);
		Zstage_port->WriteLine(STX + "WRP2/" + (5 * double::Parse(textBox3->Text)).ToString() + "\r");
		Sleep(50);
		Zstage_Flag = 0;
	}
	private: System::Void button2_Click_1(System::Object^  sender, System::EventArgs^  e) {

		XYstage_port->Write("AXIX:POSition " + textBox1->Text + "\r");

	}
	private: System::Void button3_Click_1(System::Object^  sender, System::EventArgs^  e) {
		XYstage_port->Write("AXIY:POSition " + textBox2->Text + "\r");
	}

	private: System::Void Z_galv_L_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Z_offset_current_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Z_galv_L_RV_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Z_galv_R_RV_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Z_galv_R_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void trackBar_manual_focus_Scroll(System::Object^  sender, System::EventArgs^  e) {
		manual_focus_box->Text = (trackBar_manual_focus->Value - (trackBar_manual_focus->Maximum / 2)).ToString("F0");
	}
	private: System::Void manual_focus_box_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		if (ManuaL_focus->Checked) {
			MOVIEFocusVal.FocusVal = int::Parse(manual_focus_box->Text);
		}

	}
	private: System::Void ETL_Sweep_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void ETL_Sweep_Click_1(System::Object^  sender, System::EventArgs^  e) {
	}
	};

}

