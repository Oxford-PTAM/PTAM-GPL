// Copyright 2008 Isis Innovation Limited
#include "VideoSource.h"
#include <cvd/OSX/qtbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>

using namespace CVD;
using namespace std;

VideoSource::VideoSource()
{
  cout << "  VideoSource_OSX: Creating QTBuffer...." << endl;
  cout << "  IMPORTANT " << endl;
  cout << "  This will open a quicktime settings planel. " << endl
       << "  You should use this settings dialog to turn the camera's " << endl
       << "  sharpness to a minimum, or at least so small that no sharpening " << endl
       << "  artefacts appear! In-camera sharpening will seriously degrade the " << endl
       << "  performance of both the camera calibrator and the tracking system. " << endl;
  QTBuffer<yuv422>* pvb;
  try 
    {
      pvb= new QTBuffer<yuv422>(ImageRef(640,480), 0, true);
    }
  catch (CVD::Exceptions::All a)
    {
      cerr << "  Error creating QTBuffer; expection: " << a.what << endl;
      exit(1);    
    }
  mptr = pvb;
  mirSize = pvb->size();
  cout << "  .. created QTBuffer of size " << mirSize << endl;
};

ImageRef VideoSource::Size()
{ 
  return mirSize;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  QTBuffer<yuv422>* pvb = (QTBuffer<yuv422>*) mptr;

  while(!pvb->frame_pending())
    usleep(2000);

  VideoFrame<yuv422> *pVidFrame = pvb->get_frame();

  if(pvb->get_frame_format_string().find("yuyv") != string::npos)
  {
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
  } 
  else if(pvb->get_frame_format_string().find("uyvy") != string::npos)
  {
    convert_image(*((VideoFrame<vuy422>*)pVidFrame), imBW);
    convert_image(*((VideoFrame<vuy422>*)pVidFrame), imRGB);
  } 
  else
  {
    cout << "! Code for converting from format \"" << pvb->get_frame_format_string() 
      << "\"" << endl << "  not implemented yet, check VideoSource_OSX.cc. " << endl;
    exit(1);  
  }
  
  pvb->put_frame(pVidFrame);
}





