// Copyright 2008 Isis Innovation Limited
#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

Relocaliser::Relocaliser(Map &map, ATANCamera &camera)
  : mMap(map),
    mCamera(camera)
{
};

SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}

bool Relocaliser::AttemptRecovery(KeyFrame &kCurrent)
{
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!kCurrent.pSBI)
    kCurrent.pSBI = new SmallBlurryImage(kCurrent);
  else
    kCurrent.pSBI->MakeFromKF(kCurrent);
  
  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kCurrent);

  // And estimate a camera rotation from a 3DOF image alignment
  pair<SE2<>, double> result_pair = kCurrent.pSBI->IteratePosRelToTarget(*mMap.vpKeyFrames[mnBest]->pSBI, 6);
  mse2 = result_pair.first;
  double dScore =result_pair.second;
  
  SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;
  
  if(dScore < GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT))
    return true;
  else 
    return false;
};

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame &kCurrent)
{
  mdBestScore = 99999999999999.9;
  mnBest = -1;
  
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      double dSSD = kCurrent.pSBI->ZMSSD(*mMap.vpKeyFrames[i]->pSBI);
      if(dSSD < mdBestScore)
	{
	  mdBestScore = dSSD;
	  mnBest = i;
	}
    }
}

