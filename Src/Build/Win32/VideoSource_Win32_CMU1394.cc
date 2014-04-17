// Copyright 2008 Isis Innovation Limited
// This VideoSource for Win32 uses CMU's 1394 driver
// available at 
// http://www.cs.cmu.edu/~iwan/1394/

#define WIN32_LEAN_AND_MEAN
#include "VideoSource.h"
#include <Windows.h>
#include <1394Camera.h>
#include <cvd/utility.h>

using namespace CVD;
using namespace std;

VideoSource::VideoSource()
{
	int nRet;

	C1394Camera *pCamera = new C1394Camera;
	mptr = pCamera;
	cout << "  CMU 1394 driver camera interface.  " << endl;
	
	int nNum = pCamera->RefreshCameraList();
	cout << "  Found " << nNum << " cameras." << endl;
	if(nNum <= 0)
		{cerr << "! Not enough cameras found - exit." << endl; exit(1);}

    nRet = pCamera->SelectCamera(0);
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (SelectCamera(0)) " << endl; exit(1);}
    
	pCamera->InitCamera();
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (InitCamera) " << endl; exit(1);}

	unsigned long nFormat=999;
	unsigned long nMode=999;
	if(pCamera->HasVideoMode(1,5))
	{	
			nFormat = 1; nMode = 5;
	}
	else if(pCamera->HasVideoMode(0,5))
	{
			nFormat = 0; nMode = 5;
	}
	
	nRet = pCamera->SetVideoFormat(nFormat);
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (SetVideoFormat) " << endl; exit(1);}

	nRet = pCamera->SetVideoMode(nMode);
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (SetVideoMode) " << endl; exit(1);}

	int nFrameRate = 999;
	if(pCamera->HasVideoFrameRate(nFormat, nMode, 4))
		nFrameRate = 4;
	else if(pCamera->HasVideoFrameRate(nFormat, nMode, 3))
		nFrameRate = 3;

	pCamera->SetVideoFrameRate(nFrameRate);
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (SetVideoFrameRate) " << endl; exit(1);}

	{
		unsigned long x,y;
		pCamera->GetVideoFrameDimensions(&x,&y);
		mirSize.x = x;
		mirSize.y = y;
	}

    pCamera->StartImageAcquisitionEx(3,10000,ACQ_START_VIDEO_STREAM);
	if(nRet != CAM_SUCCESS) { cerr << "!! Error on CMU1394 init (StartImageAcquisitionEx) " << endl; exit(1);}
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
   C1394Camera *pCamera = (C1394Camera*) mptr;
   pCamera->AcquireImage();
   unsigned long nLength;
   unsigned char *pImage = pCamera->GetRawData(&nLength);
   BasicImage<byte> imCaptured(pImage, mirSize);
   copy(imCaptured, imBW);
   copy(imCaptured, imRGB);
}

ImageRef VideoSource::Size()
{
	return mirSize;
}
