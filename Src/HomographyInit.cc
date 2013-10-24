// Copyright 2008 Isis Innovation Limited
#include "HomographyInit.h"
#include "SmallMatrixOpts.h"
#include <utility>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
#include <TooN/wls.h>
#include "MEstimator.h"

using namespace std;
bool HomographyInit::IsHomographyInlier(Matrix<3> m3Homography, HomographyMatch match)
{
  Vector<2> v2Projected = project(m3Homography * unproject(match.v2CamPlaneFirst));
  Vector<2> v2Error = match.v2CamPlaneSecond - v2Projected;
  Vector<2> v2PixelError = match.m2PixelProjectionJac * v2Error;
  double dSquaredError = v2PixelError * v2PixelError;
  return (dSquaredError < mdMaxPixelErrorSquared);
}

double HomographyInit::MLESACScore(Matrix<3> m3Homography, HomographyMatch match)
{
  Vector<2> v2Projected = project(m3Homography * unproject(match.v2CamPlaneFirst));
  Vector<2> v2Error = match.v2CamPlaneSecond - v2Projected;
  Vector<2> v2PixelError = match.m2PixelProjectionJac * v2Error;
  double dSquaredError = v2PixelError * v2PixelError;
  if(dSquaredError > mdMaxPixelErrorSquared)
    return mdMaxPixelErrorSquared;
  else 
    return dSquaredError;
}

bool HomographyInit::Compute(vector<HomographyMatch> vMatches, double dMaxPixelError, SE3<> &se3SecondFromFirst)
{
  mdMaxPixelErrorSquared = dMaxPixelError * dMaxPixelError;
  mvMatches = vMatches;
  
  // Find best homography from minimal sets of image matches
  BestHomographyFromMatches_MLESAC();
  
  // Generate the inlier set, and refine the best estimate using this
  mvHomographyInliers.clear();
  for(unsigned int i=0; i<mvMatches.size(); i++)
    if(IsHomographyInlier(mm3BestHomography, mvMatches[i]))
      mvHomographyInliers.push_back(mvMatches[i]);
  for(int iteration = 0; iteration < 5; iteration++)
    RefineHomographyWithInliers();
  
  // Decompose the best homography into a set of possible decompositions
  DecomposeHomography();

  // At this stage should have eight decomposition options, if all went according to plan
  if(mvDecompositions.size() != 8)
    return false;
  
  // And choose the best one based on visibility constraints
  ChooseBestDecomposition();
  
  se3SecondFromFirst = mvDecompositions[0].se3SecondFromFirst;
  return true;
}

Matrix<3> HomographyInit::HomographyFromMatches(vector<HomographyMatch> vMatches)
{
  unsigned int nPoints = vMatches.size();
  assert(nPoints >= 4);
  int nRows = 2*nPoints;
  if(nRows < 9)
    nRows = 9;
  Matrix<> m2Nx9(nRows, 9);
  for(unsigned int n=0; n<nPoints; n++)
    {
      double u = vMatches[n].v2CamPlaneSecond[0];
      double v = vMatches[n].v2CamPlaneSecond[1];
      
      double x = vMatches[n].v2CamPlaneFirst[0];
      double y = vMatches[n].v2CamPlaneFirst[1];
      
      // [u v]T = H [x y]T
      m2Nx9[n*2+0][0] = x;
      m2Nx9[n*2+0][1] = y;
      m2Nx9[n*2+0][2] = 1;
      m2Nx9[n*2+0][3] = 0;
      m2Nx9[n*2+0][4] = 0;
      m2Nx9[n*2+0][5] = 0;
      m2Nx9[n*2+0][6] = -x*u;
      m2Nx9[n*2+0][7] = -y*u;
      m2Nx9[n*2+0][8] = -u;

      m2Nx9[n*2+1][0] = 0;
      m2Nx9[n*2+1][1] = 0;
      m2Nx9[n*2+1][2] = 0;
      m2Nx9[n*2+1][3] = x;
      m2Nx9[n*2+1][4] = y;
      m2Nx9[n*2+1][5] = 1;
      m2Nx9[n*2+1][6] = -x*v;
      m2Nx9[n*2+1][7] = -y*v;
      m2Nx9[n*2+1][8] = -v;
    }

  if(nRows == 9)  
   for(int i=0; i<9; i++)  // Zero the last row of the matrix, 
     m2Nx9[8][i] = 0.0;  // TooN SVD leaves out the null-space otherwise
  
  // The right null-space of the matrix gives the homography...
  SVD<> svdHomography(m2Nx9);
  Vector<9> vH = svdHomography.get_VT()[8];
  Matrix<3> m3Homography;
  m3Homography[0] = vH.slice<0,3>();
  m3Homography[1] = vH.slice<3,3>();
  m3Homography[2] = vH.slice<6,3>();
  return m3Homography;
};

// Throughout the whole thing,
// SecondView = Homography * FirstView

void HomographyInit::RefineHomographyWithInliers()
{
  WLS<9> wls;
  wls.add_prior(1.0);
  
  vector<Matrix<2,9> > vmJacobians;
  vector<Vector<2> > vvErrors;
  vector<double> vdErrorSquared;
  
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++)
    {
      // First, find error.
      Vector<2> v2First = mvHomographyInliers[i].v2CamPlaneFirst;
      Vector<3> v3Second = mm3BestHomography * unproject(v2First);
      Vector<2> v2Second = project(v3Second);
      Vector<2> v2Second_real = mvHomographyInliers[i].v2CamPlaneSecond;
      Vector<2> v2Error = mvHomographyInliers[i].m2PixelProjectionJac * (v2Second_real - v2Second);
      
      vdErrorSquared.push_back(v2Error* v2Error);
      vvErrors.push_back(v2Error);
      
      Matrix<2,9> m29Jacobian;
      double dDenominator = v3Second[2];
      
      // Jacobians wrt to the elements of the homography:
      // For x:
      m29Jacobian[0].slice<0,3>() = unproject(v2First) / dDenominator;
      m29Jacobian[0].slice<3,3>() = Zeros;
      double dNumerator = v3Second[0];
      m29Jacobian[0].slice<6,3>() = -unproject(v2First) * dNumerator / (dDenominator * dDenominator);
      // For y:
      m29Jacobian[1].slice<0,3>() = Zeros;
      m29Jacobian[1].slice<3,3>()  = unproject(v2First) / dDenominator;;
      dNumerator = v3Second[1];
      m29Jacobian[1].slice<6,3>() = -unproject(v2First) * dNumerator / (dDenominator * dDenominator);
      
      vmJacobians.push_back(mvHomographyInliers[i].m2PixelProjectionJac * m29Jacobian);
    }
  
  // Calculate robust sigma:
  vector<double> vdd = vdErrorSquared;
  double dSigmaSquared = Tukey::FindSigmaSquared(vdd);
  
  // Add re-weighted measurements to WLS:
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++)
    {
      double dWeight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
      wls.add_mJ(vvErrors[i][0], vmJacobians[i][0], dWeight);
      wls.add_mJ(vvErrors[i][1], vmJacobians[i][1], dWeight);
    }
  wls.compute();
  Vector<9> v9Update = wls.get_mu();
  Matrix<3> m3Update;
  m3Update[0] = v9Update.slice<0,3>();
  m3Update[1] = v9Update.slice<3,3>();
  m3Update[2] = v9Update.slice<6,3>();
  mm3BestHomography += m3Update;
}

void HomographyInit::BestHomographyFromMatches_MLESAC()
{
  // Not many matches? Don't do ransac, throw them all in a pot and see what comes out.
  if(mvMatches.size() < 10)
    {
      mm3BestHomography = HomographyFromMatches(mvMatches);
      return;
    }
  
  // Enough matches? Run MLESAC.
  int anIndices[4];
  
  mm3BestHomography = Identity;
  double dBestError = 999999999999999999.9;
  
  // Do 300 MLESAC trials.
  for(int nR = 0; nR < 300 ; nR++)
    { 
      // Find set of four unique matches
      for(int i=0; i<4; i++)
	{
	  bool isUnique = false;
	  int n;
	  while(!isUnique)
	    {
	      n = rand() % mvMatches.size();
	      isUnique =true;
	      for(int j=0; j<i && isUnique; j++)
		if(anIndices[j] == n)
		  isUnique = false;
	    };
	  anIndices[i] = n;
	}
      vector<HomographyMatch> vMinimalMatches;
      for(int i=0; i<4; i++)
	vMinimalMatches.push_back(mvMatches[anIndices[i]]);
      
      // Find a homography from the minimal set..
      Matrix<3> m3Homography = HomographyFromMatches(vMinimalMatches);
      
      //..and sum resulting MLESAC score
      double dError = 0.0;
      for(unsigned int i=0; i<mvMatches.size(); i++)
 	dError += MLESACScore(m3Homography, mvMatches[i]);
      
      if(dError < dBestError)
	{
	  mm3BestHomography = m3Homography;
	  dBestError = dError;
	}
    };
}

void HomographyInit::DecomposeHomography()
{
  mvDecompositions.clear();
  SVD<3> svd(mm3BestHomography);
  Vector<3> v3Diag = svd.get_diagonal();
  double d1 = fabs(v3Diag[0]); // The paper suggests the square of these (e.g. the evalues of AAT)
  double d2 = fabs(v3Diag[1]); // should be used, but this is wrong. c.f. Faugeras' book.
  double d3 = fabs(v3Diag[2]);
  
  Matrix<3> U = svd.get_U();
  Matrix<3> V = svd.get_VT().T();
  
  double s = M3Det(U) * M3Det(V);
  
  double dPrime_PM = d2;
  
  int nCase;
  if(d1 != d2 && d2 != d3)
    nCase = 1;
  else if( d1 == d2 && d2 == d3)
    nCase = 3;
  else
    nCase = 2;
  
  if(nCase != 1)
    {
      cout << "  Homographyinit: This motion case is not implemented or is degenerate. Try again. " << endl;
      return;
    }
  
  double x1_PM;
  double x2;
  double x3_PM;

  // All below deals with the case = 1 case.
  // Case 1 implies (d1 != d3) 
  { // Eq. 12
    x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
    x2    = 0;
    x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
  };
  
  double e1[4] = {1.0,-1.0,1.0,-1.0};
  double e3[4] = {1.0, 1.0, -1.0,-1.0};
    
  Vector<3> v3np;
  HomographyDecomposition decomposition;

  // Case 1, d' > 0:
  decomposition.d = s * dPrime_PM;
  for(int signs=0; signs<4; signs++)
    {
      // Eq 13
      decomposition.m3Rp = Identity;
      double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
      double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
      decomposition.m3Rp[0][0] = dCosTheta;                
      decomposition.m3Rp[0][2] = -dSinTheta;
      decomposition.m3Rp[2][0] = dSinTheta;                
      decomposition.m3Rp[2][2] = dCosTheta;
      
      // Eq 14
      decomposition.v3Tp[0] = (d1 - d3) * x1_PM * e1[signs];
      decomposition.v3Tp[1] = 0.0;
      decomposition.v3Tp[2] = (d1 - d3) * -x3_PM * e3[signs];
  
      v3np[0] = x1_PM * e1[signs];
      v3np[1] = x2;
      v3np[2] = x3_PM * e3[signs];
      decomposition.v3n = V * v3np;
      
      mvDecompositions.push_back(decomposition);
    }
  // Case 1, d' < 0:
  decomposition.d = s * -dPrime_PM;
  for(int signs=0; signs<4; signs++)
    { 
      // Eq 15
      decomposition.m3Rp = -1 * Identity;
      double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
      double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
      decomposition.m3Rp[0][0] = dCosPhi;                
      decomposition.m3Rp[0][2] = dSinPhi;
      decomposition.m3Rp[2][0] = dSinPhi;                
      decomposition.m3Rp[2][2] = -dCosPhi;
      
      // Eq 16
      decomposition.v3Tp[0] = (d1 + d3) * x1_PM * e1[signs];
      decomposition.v3Tp[1] = 0.0;
      decomposition.v3Tp[2] = (d1 + d3) * x3_PM * e3[signs];

      v3np[0] = x1_PM * e1[signs];
      v3np[1] = x2;
      v3np[2] = x3_PM * e3[signs];
      decomposition.v3n = V * v3np;
      
      mvDecompositions.push_back(decomposition);
    }
  
  // While we have the SVD results calculated here, store the decomposition R and t results as well..
  for(unsigned int i=0; i<mvDecompositions.size(); i++)
    {
      mvDecompositions[i].se3SecondFromFirst.get_rotation() = 
	s * U * mvDecompositions[i].m3Rp * V.T();
      mvDecompositions[i].se3SecondFromFirst.get_translation() = 
	U * mvDecompositions[i].v3Tp;
    }
}

bool operator<(const HomographyDecomposition lhs, const HomographyDecomposition rhs)
{
  return lhs.nScore < rhs.nScore;
}

static double SampsonusError(Vector<2> &v2Dash, const Matrix<3> &m3Essential, Vector<2> &v2)
{
  Vector<3> v3Dash = unproject(v2Dash);
  Vector<3> v3 = unproject(v2);  
  
  double dError = v3Dash * m3Essential * v3;
  
  Vector<3> fv3 = m3Essential * v3;
  Vector<3> fTv3Dash = m3Essential.T() * v3Dash;
  
  Vector<2> fv3Slice = fv3.slice<0,2>();
  Vector<2> fTv3DashSlice = fTv3Dash.slice<0,2>();
  
  return (dError * dError / (fv3Slice * fv3Slice + fTv3DashSlice * fTv3DashSlice));
}


void HomographyInit::ChooseBestDecomposition()
{
  assert(mvDecompositions.size() == 8);
  for(unsigned int i=0; i<mvDecompositions.size(); i++)
    {
      HomographyDecomposition &decom = mvDecompositions[i];
      int nPositive = 0;
      for(unsigned int m=0; m<mvHomographyInliers.size(); m++)
	{
	  Vector<2> &v2 = mvHomographyInliers[m].v2CamPlaneFirst;
	  double dVisibilityTest = (mm3BestHomography[2][0] * v2[0] + mm3BestHomography[2][1] * v2[1] + mm3BestHomography[2][2]) / decom.d;
	  if(dVisibilityTest > 0.0)
	    nPositive++;
	};
      decom.nScore = -nPositive;
    }
  
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(4);
  
  for(unsigned int i=0; i<mvDecompositions.size(); i++)
    {
      HomographyDecomposition &decom = mvDecompositions[i];
      int nPositive = 0;
      for(unsigned int m=0; m<mvHomographyInliers.size(); m++)
	{
	  Vector<3> v3 = unproject(mvHomographyInliers[m].v2CamPlaneFirst);
	  double dVisibilityTest = v3 * decom.v3n / decom.d;
	  if(dVisibilityTest > 0.0)
	    nPositive++;
	};
      decom.nScore = -nPositive;
    }
  
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(2);
  
  // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
  // but in practive, better to look at the ratio!
  double dRatio = (double) mvDecompositions[1].nScore / (double) mvDecompositions[0].nScore;

  if(dRatio < 0.9) // no ambiguity!
    mvDecompositions.erase(mvDecompositions.begin() + 1);
  else             // two-way ambiguity. Resolve by sampsonus score of all points.
    {
      double dErrorSquaredLimit  = mdMaxPixelErrorSquared * 4;
      double adSampsonusScores[2];
      for(int i=0; i<2; i++)
	{
	  SE3<> se3 = mvDecompositions[i].se3SecondFromFirst;
	  Matrix<3> m3Essential;
	  for(int j=0; j<3; j++)
	    m3Essential.T()[j] = se3.get_translation() ^ se3.get_rotation().get_matrix().T()[j];
	  
	  double dSumError = 0;
	  for(unsigned int m=0; m < mvMatches.size(); m++ )
	    {
	      double d = SampsonusError(mvMatches[m].v2CamPlaneSecond, m3Essential, mvMatches[m].v2CamPlaneFirst);
	      if(d > dErrorSquaredLimit)
		d = dErrorSquaredLimit;
	      dSumError += d;
	    }
	  
	  adSampsonusScores[i] = dSumError;
	}

      if(adSampsonusScores[0] <= adSampsonusScores[1])
	mvDecompositions.erase(mvDecompositions.begin() + 1);
      else
	mvDecompositions.erase(mvDecompositions.begin());
    }
  
}


