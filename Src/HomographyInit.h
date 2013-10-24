// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited

// HomographyInit.h 
// Declares the HomographyInit class and a few helper functions. 
//
// This class is used by MapMaker to bootstrap the map, and implements
// the homography decomposition of Faugeras and Lustman's 1988 tech
// report.
//
// Implementation according to Faugeras and Lustman

#ifndef __HOMOGRAPHY_INIT_H
#define __HOMOGRAPHY_INIT_H
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <vector>

// Homography matches are 2D-2D matches in a stereo pair, unprojected
// to the Z=1 plane.
struct HomographyMatch
{
  // To be filled in by MapMaker:
  Vector<2> v2CamPlaneFirst;
  Vector<2> v2CamPlaneSecond;
  Matrix<2> m2PixelProjectionJac;
};

// Storage for each homography decomposition
struct HomographyDecomposition
{
  Vector<3> v3Tp;
  Matrix<3> m3Rp;
  double d;
  Vector<3> v3n;
  
  // The resolved composition..
  SE3<> se3SecondFromFirst;
  int nScore;
};

class HomographyInit
{
public:
  bool Compute(std::vector<HomographyMatch> vMatches, double dMaxPixelError, SE3<> &se3SecondCameraPose);
protected:
  Matrix<3> HomographyFromMatches(std::vector<HomographyMatch> vMatches);
  void BestHomographyFromMatches_MLESAC();
  void DecomposeHomography();
  void ChooseBestDecomposition();
  void RefineHomographyWithInliers();
  
  bool IsHomographyInlier(Matrix<3> m3Homography, HomographyMatch match);
  double MLESACScore(Matrix<3> m3Homography, HomographyMatch match);
  
  double mdMaxPixelErrorSquared;
  Matrix<3> mm3BestHomography;
  std::vector<HomographyMatch> mvMatches;
  std::vector<HomographyMatch> mvHomographyInliers;
  std::vector<HomographyDecomposition> mvDecompositions;
};



#endif
