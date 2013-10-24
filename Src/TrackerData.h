// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H

#include "PatchFinder.h"
#include "ATANCamera.h"

// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

struct TrackerData
{
TrackerData(MapPoint *pMapPoint) 
: Point(*pMapPoint)
  {};
  
  MapPoint &Point;
  PatchFinder Finder;
  
  // Projection itermediates:
  Vector<3> v3Cam;        // Coords in current cam frame
  Vector<2> v2ImPlane;    // Coords in current cam z=1 plane
  Vector<2> v2Image;      // Pixel coords in LEVEL0
  Matrix<2> m2CamDerivs;  // Camera projection derivs
  bool bInImage;        
  bool bPotentiallyVisible;
  
  int nSearchLevel;
  bool bSearched;
  bool bFound;
  bool bDidSubPix;
  Vector<2> v2Found;      // Pixel coords of found patch (L0)
  double dSqrtInvNoise;   // Only depends on search level..
  
  
  // Stuff for pose update:
  Vector<2> v2Error_CovScaled;
  Matrix<2,6> m26Jacobian;   // Jacobian wrt camera position
  
  // Project point into image given certain pose and camera.
  // This can bail out at several stages if the point
  // will not be properly in the image.
  inline void Project(const SE3<> &se3CFromW, ATANCamera &Cam)
  {
    bInImage = bPotentiallyVisible = false;
    v3Cam = se3CFromW * Point.v3WorldPos;
    if(v3Cam[2] < 0.001)
      return;
    v2ImPlane = project(v3Cam);
    if(v2ImPlane*v2ImPlane > Cam.LargestRadiusInImage() * Cam.LargestRadiusInImage())
      return;
    v2Image = Cam.Project(v2ImPlane);
    if(Cam.Invalid())
      return;
    
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
      return;
    bInImage = true;
  }
  
  // Get the projection derivatives (depend only on the camera.)
  // This is called Unsafe because it depends on the camera caching 
  // results from the previous projection:
  // Only do this right after the same point has been projected!
  inline void GetDerivsUnsafe(ATANCamera &Cam) 
  {
    m2CamDerivs = Cam.GetProjectionDerivs();
  }
  
  // Does projection and gets camera derivs all in one.
  inline void ProjectAndDerivs(SE3<> &se3, ATANCamera &Cam)
  {
    Project(se3, Cam);
    if(bFound)
      GetDerivsUnsafe(Cam);
  }
  
  // Jacobian of projection W.R.T. the camera position
  // I.e. if  p_cam = SE3Old * p_world, 
  //         SE3New = SE3Motion * SE3Old
  inline void CalcJacobian()
  {
    double dOneOverCameraZ = 1.0 / v3Cam[2];
    for(int m=0; m<6; m++)
      {
	const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
	Vector<2> v2CamFrameMotion;
	v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	m26Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
      };
  }
  
  // Sometimes in tracker instead of reprojecting, just update the error linearly!
  inline void LinearUpdate(const Vector<6> &v6)
  {
    v2Image += m26Jacobian * v6;
  }
  
  // This static member is filled in by the tracker and allows in-image checks in this class above.
  static CVD::ImageRef irImageSize;
};






#endif




