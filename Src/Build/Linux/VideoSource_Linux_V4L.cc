// Copyright 2008 Isis Innovation Limited
#include "VideoSource.h"
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource::VideoSource()
{
  cout << "  VideoSource_Linux: Opening video source..." << endl;
  string QuickCamFile = GV3::get<string>("VideoSource.V4LDevice", "/dev/video0");
  ImageRef irSize = GV3::get<ImageRef>("VideoSource.Resolution", ImageRef(640,480));
  int nFrameRate = GV3::get<int>("VideoSource.Framerate", 30);
  V4LBuffer<yuv422>* pvb = new V4LBuffer<yuv422>(QuickCamFile, irSize, -1, false, nFrameRate);
  mirSize = pvb->size();
  mptr = pvb;
  cout << "  ... got video source." << endl;
};

ImageRef VideoSource::Size()
{ 
  return mirSize;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  V4LBuffer<yuv422>* pvb = (V4LBuffer<yuv422>*) mptr;
  VideoFrame<yuv422> *pVidFrame = pvb->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  pvb->put_frame(pVidFrame);
}
