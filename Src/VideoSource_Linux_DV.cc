// Copyright 2008 Isis Innovation Limited
#include "VideoSource.h"
#include <cvd/Linux/dvbuffer3.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>

using namespace CVD;
using namespace std;

VideoSource::VideoSource()
{
  cout << "  VideoSource_Linux: Opening video source..." << endl;
  DVBuffer3<yuv411>* pvb= new DVBuffer3<yuv411>();
  mptr = pvb;
  mirSize = pvb->size();
  cout << "  ... got video source." << endl;
};

ImageRef VideoSource::Size()
{ 
  return mirSize;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  DVBuffer3<yuv411>* pvb = (DVBuffer3<yuv411>*) mptr;
  VideoFrame<yuv411> *pVidFrame = pvb->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  pvb->put_frame(pVidFrame);
}
