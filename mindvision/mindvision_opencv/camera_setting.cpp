#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <cstdio>
#include <chrono>


#include "CameraApi.h"

unsigned char* g_pRgbBuffer;
int            hCamera;
tSdkFrameHead  sFrameInfo;
BYTE*          pbyBuffer;

bool cameraInitConfig()
{
   int                iCameraCounts = 1;
	 int                iStatus = -1;
	 tSdkCameraDevInfo  tCameraEnumList; 
	 tSdkCamerCapbility tCapability;
	 int                channel = 3;

	CameraSdkInit(1);
  	
