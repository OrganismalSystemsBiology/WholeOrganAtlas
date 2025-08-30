#pragma once

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32   error = 0;
	char    errBuff[2048] = { '\0' };

	// Check to see if an error stopped the task.
	DAQmxErrChk(status);

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}

	return 0;
}
int DAQ(double Sample, double rate, double data[250000], char Dev[200])
{
	int32       error = 0;
	taskHandle = 0;
	char        errBuff[2048] = { '\0' };

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, Sample));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));



	/*********************************************/
	// DAQmx Write Code
	/*********************************************/

	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));
	//wave‚ð•Û‘¶ sheet

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));

	printf("Generating voltage continuously.\n");

	//getchar();

	return 0;
Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program\n");
	//getchar();
	return 0;
}
int DAQ_single(double Sample, double rate, double data[250000], char Dev[200])
{
	int32       error = 0;
	taskHandle = 0;
	char        errBuff[2048] = { '\0' };


	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	//DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle, "cDAQ1Mod1/ao1,cDAQ1Mod2/ao1","",-10.0,10.0,DAQmx_Val_Volts,NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
	//DAQmxErrChk (DAQmxCfgAnlgEdgeStartTrig(taskHandle,"",DAQmx_Val_RisingSlope,0));
	//DAQmxErrChk (DAQmxSetAnlgEdgeStartTrigHyst(taskHandle, 1.0));

	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));

	DAQmxErrChk(DAQmxStartTask(taskHandle));

	DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle, 10.0));
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);

	return 0;
Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	return 0;
}
int DAQTTL(char Dev[100])
{
	int32       error = 0;
	taskHandle = 0;
	int         i = 0;
	char        errBuff[2048] = { '\0' };
	double rate = 1000;
	double Sample = 1000;
	double TTL[1000];
	for (; i < 1000; i++) {
		if (i < 100)
			TTL[i] = 5.00000;
		else
			TTL[i] = 0.00;
	}
	cout << "TTL" << endl;
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	//DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle, "cDAQ1Mod1/ao1,cDAQ1Mod2/ao1","",-10.0,10.0,DAQmx_Val_Volts,NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
	//DAQmxErrChk (DAQmxCfgAnlgEdgeStartTrig(taskHandle,"",DAQmx_Val_RisingSlope,0));
	//DAQmxErrChk (DAQmxSetAnlgEdgeStartTrigHyst(taskHandle, 1.0));

	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, TTL, NULL, NULL));

	DAQmxErrChk(DAQmxStartTask(taskHandle));

	DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle, 10.0));
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);

	return 0;
Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	return 0;
}
int DAQ_sheet_endpos()
{
	int32       error = 0;
	taskHandle = 0;

	char Dev[100] = "cDAQ2Mod1/ao0,cDAQ2Mod2/ao0,cDAQ2Mod1/ao1,cDAQ2Mod2/ao1";
	int         i = 0;
	char        errBuff[2048] = { '\0' };
	double rate = 2000;
	double Sample = 200;
	double data[1000];
	for (; i < 800; i++) {
		if (i < 400)
			data[i] = 9.00000;
		else
			data[i] = 0.00000;

	}
	cout << "Goto Sheet End Pos" << endl;
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	//DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle, "cDAQ1Mod1/ao1,cDAQ1Mod2/ao1","",-10.0,10.0,DAQmx_Val_Volts,NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
	//DAQmxErrChk (DAQmxCfgAnlgEdgeStartTrig(taskHandle,"",DAQmx_Val_RisingSlope,0));
	//DAQmxErrChk (DAQmxSetAnlgEdgeStartTrigHyst(taskHandle, 1.0));

	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));

	DAQmxErrChk(DAQmxStartTask(taskHandle));

	DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle, 10.0));
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);

	return 0;
Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	return 0;
}
int DAQ_sheet_Zeropos()
{
	int32       error = 0;
	taskHandle = 0;

	char Dev[] = "cDAQ2Mod1/ao0,cDAQ2Mod2/ao0,cDAQ2Mod1/ao1,cDAQ2Mod2/ao1";
	int         i = 0;
	char        errBuff[2048] = { '\0' };
	double rate = 1000;
	double Sample = 200;
	double data[1000];
	for (; i < 1000; i++) {
		data[i] = 0.00000;
	}
	cout << "Goto Sheet Zero Pos" << endl;
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	//DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle, "cDAQ1Mod1/ao1,cDAQ1Mod2/ao1","",-10.0,10.0,DAQmx_Val_Volts,NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, Dev, "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, Sample));
	//DAQmxErrChk (DAQmxCfgAnlgEdgeStartTrig(taskHandle,"",DAQmx_Val_RisingSlope,0));
	//DAQmxErrChk (DAQmxSetAnlgEdgeStartTrigHyst(taskHandle, 1.0));

	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, Sample, 0, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));

	DAQmxErrChk(DAQmxStartTask(taskHandle));

	DAQmxErrChk(DAQmxWaitUntilTaskDone(taskHandle, 10.0));
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);

	return 0;
Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	return 0;
}
void DAQ_TASKmaking_MOVIE2(double Ampli_sheet, double Ampli_mSPIM, double Exposuretime) {

	cout << Ampli_sheet << Ampli_mSPIM << Exposuretime << endl;

}
void DAQ_TASKmaking_MOVIE(double Ampli_sheet, double Ampli_mSPIM, double Exposuretime) {
	double SampleRate = 10000;
	double SampleNum = 10 * Exposuretime;
	cout << SampleNum << endl;
	char Dev[200];
	char Dev1[] = "cDAQ2Mod1/ao0";//Right-sheet
	char Dev2[] = "cDAQ2Mod2/ao0";//Left-sheet
	char Dev3[] = "cDAQ2Mod1/ao1";//Right-mSPIM
	char Dev4[] = "cDAQ2Mod2/ao1";//Left-mSPIM
	char Dev5[] = "cDAQ2Mod2/ao2";//Camera

	int i = 0;
	int j = 0;

	int Datalength; //total sample number
	int galvoNum = 0; //# of Galvo
	galvoNum = galvoNum + 2;
	for (i = 0; Dev1[i] != '\0'; i++)
		Dev[i] = Dev1[i];
	Dev[i] = ',';
	i++;
	for (j = 0; Dev2[j] != '\0'; i++, j++)
		Dev[i] = Dev2[j];
	Dev[i] = ',';
	i++;
	for (j = 0; Dev3[j] != '\0'; i++, j++)
		Dev[i] = Dev3[j];
	Dev[i] = ',';
	i++;
	for (j = 0; Dev4[j] != '\0'; i++, j++)
		Dev[i] = Dev4[j];
	Dev[i] = ',';
	i++;
	for (j = 0; Dev5[j] != '\0'; i++, j++)
		Dev[i] = Dev5[j];
	i++;
	Dev[i] = '\0';

	//cout << Dev << endl;

	std::vector<double> data_sheet(SampleNum);
	std::vector<double>data_mSPIM(SampleNum);
	//double data_galvo[25000] = { 0 };

	double *data_galvo;
	data_galvo = new double[250000];

	int n = 0;
	double x = 0;
	for (int i = 0; i < SampleNum; i++) {
		if (i < SampleNum / 12)
			x = x + 12 * Ampli_sheet / SampleNum;
		else if (i < 3 * SampleNum / 12)
			x = x - 12 * Ampli_sheet / SampleNum;
		else if (i < 5 * SampleNum / 12)
			x = x + 12 * Ampli_sheet / SampleNum;
		else if (i < 7 * SampleNum / 12)
			x = x - 12 * Ampli_sheet / SampleNum;
		else if (i < 9 * SampleNum / 12)
			x = x + 12 * Ampli_sheet / SampleNum;
		else if (i < 11 * SampleNum / 12)
			x = x - 12 * Ampli_sheet / SampleNum;
		else
			x = x + 12 * Ampli_sheet / SampleNum;
		if (x > Ampli_sheet)
			x = Ampli_sheet;
		if (x < -1 * Ampli_sheet)
			x = -1 * Ampli_sheet;
		data_sheet[i] = x;
	}

	for (int p = 0; p < SampleNum; p++) {
		double a = p / SampleNum;
		if (a > 0.666666) {
			data_mSPIM[p] = Ampli_mSPIM;
		}
		else if (a > 0.3333333) {
			data_mSPIM[p] = 0;
		}
		else {
			data_mSPIM[p] = Ampli_mSPIM * (-1);
		}
	}

	//Exposuretime
	std::vector<double>externalexposure(SampleNum);
	for (int i = 0; i < SampleNum; i++) {
		if (i < 10)
			externalexposure[i] = 0; // 1ms break
		else if (i < SampleNum - 100)
			externalexposure[i] = 5;
		else
			externalexposure[i] = 0;
	}
	int t = 0;
	for (int k = 0; k < SampleNum; t++, k++)
		data_galvo[t] = data_sheet[k];
	for (int k = 0; k < SampleNum; t++, k++)
		data_galvo[t] = data_sheet[k];
	for (int k = 0; k < SampleNum; t++, k++)
		data_galvo[t] = data_mSPIM[k];
	for (int k = 0; k < SampleNum; t++, k++)
		data_galvo[t] = data_mSPIM[k];
	for (int k = 0; k < SampleNum; t++, k++) {
		data_galvo[t] = externalexposure[k];
	}
	/*
	ofstream fout3("Wave.txt");
	for (int m = 0; m< SampleNum * 5; m++) {
	fout3 << data_galvo[m] << "\n";
	}
	fout3.close();
	ofstream fout("Dev.txt");
	for (int h = 0; h < SampleNum; h++) {
	fout << Dev[h] << "\n";
	}
	*/

	DAQ(SampleNum, SampleRate, data_galvo, Dev);
	delete data_galvo;
}

Void DAQ_clear() {

	if (taskHandle != 0) {
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
}

void single_exposure2(int cameraNum, float exp_time, int PicNum, string FP2) {
	Acqisition(0, exp_time, PicNum, FP2);
}



