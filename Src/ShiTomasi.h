// Copyright 2008 Isis Innovation Limited
#ifndef __SHI_TOMASI__H
#define __SHI_TOMASI__H

#include <cvd/image.h>
#include <cvd/byte.h>


double FindShiTomasiScoreAtPoint(CVD::BasicImage<CVD::byte> &image,
				 int nHalfBoxSize,
				 CVD::ImageRef irCenter);


#endif
