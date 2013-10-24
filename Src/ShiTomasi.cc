// Copyright 2008 Isis Innovation Limited
#include "ShiTomasi.h"
#include <math.h>

using namespace CVD;

double FindShiTomasiScoreAtPoint(BasicImage<byte> &image,
				 int nHalfBoxSize,
				 ImageRef irCenter)
{
  double dXX = 0;
  double dYY = 0;
  double dXY = 0;
  
  ImageRef irStart = irCenter - ImageRef(nHalfBoxSize, nHalfBoxSize);
  ImageRef irEnd = irCenter + ImageRef(nHalfBoxSize, nHalfBoxSize);
  
  ImageRef ir;
  for(ir.y = irStart.y; ir.y<=irEnd.y; ir.y++)
    for(ir.x = irStart.x; ir.x<=irEnd.x; ir.x++)
      {
	double dx = image[ir + ImageRef(1,0)] - image[ir - ImageRef(1,0)];
	double dy = image[ir + ImageRef(0,1)] - image[ir - ImageRef(0,1)];
	dXX += dx*dx;
	dYY += dy*dy;
	dXY += dx*dy;
      }
  
  int nPixels = (irEnd - irStart + ImageRef(1,1)).area();
  dXX = dXX / (2.0 * nPixels);
  dYY = dYY / (2.0 * nPixels);
  dXY = dXY / (2.0 * nPixels);
  
  // Find and return smaller eigenvalue:
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
};

