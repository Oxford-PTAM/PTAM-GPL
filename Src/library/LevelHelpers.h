// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

// LevelHelpers.h - a few handy tools to ease using levels.
// The important thing is the XXXPos functions, which convert
// image positions from one level to another. Use these whenever
// transforming positions to ensure consistent operation!!

#ifndef __LEVEL_HELPERS_H
#define __LEVEL_HELPERS_H
#include <TooN/TooN.h>
using namespace TooN;
#include <cvd/image_ref.h>

// Set of global colours useful for drawing stuff:
extern Vector<3> gavLevelColors[];
// (These are filled in in KeyFrame.cc)

// What is the scale of a level?
inline int LevelScale(int nLevel)
{
  return 1 << nLevel;
}

// 1-D transform to level zero:
inline double LevelZeroPos(double dLevelPos, int nLevel)
{
  return (dLevelPos + 0.5) * LevelScale(nLevel) - 0.5;
}

// 2-D transforms to level zero:
inline Vector<2> LevelZeroPos(Vector<2> v2LevelPos, int nLevel)
{
  Vector<2> v2Ans;
  v2Ans[0] = LevelZeroPos(v2LevelPos[0], nLevel);
  v2Ans[1] = LevelZeroPos(v2LevelPos[1], nLevel);
  return v2Ans;
}
inline Vector<2> LevelZeroPos(CVD::ImageRef irLevelPos, int nLevel) 
{
  Vector<2> v2Ans;
  v2Ans[0] = LevelZeroPos(irLevelPos.x, nLevel);
  v2Ans[1] = LevelZeroPos(irLevelPos.y, nLevel);
  return v2Ans;
}

// 1-D transform from level zero to level N:
inline double LevelNPos(double dRootPos, int nLevel)
{
  return (dRootPos + 0.5) / LevelScale(nLevel) - 0.5;
}

// 2-D transform from level zero to level N:
inline Vector<2> LevelNPos(Vector<2> v2RootPos, int nLevel)
{
  Vector<2> v2Ans;
  v2Ans[0] = LevelNPos(v2RootPos[0], nLevel);
  v2Ans[1] = LevelNPos(v2RootPos[1], nLevel);
  return v2Ans;
}

#endif
