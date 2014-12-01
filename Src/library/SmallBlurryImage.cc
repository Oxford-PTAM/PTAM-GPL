// Copyright 2008 Isis Innovation Limited
#include "SmallBlurryImage.h"
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/vision.h>
#include <TooN/se2.h>
#include <TooN/Cholesky.h>
#include <TooN/wls.h>

using namespace CVD;
using namespace std;

ImageRef SmallBlurryImage::mirSize(-1,-1);

SmallBlurryImage::SmallBlurryImage(KeyFrame &kf, double dBlur)
{
  mbMadeJacs = false;
  MakeFromKF(kf, dBlur);
}

SmallBlurryImage::SmallBlurryImage()
{
  mbMadeJacs = false;
}

// Make a SmallBlurryImage from a KeyFrame This fills in the mimSmall
// image (Which is just a small un-blurred version of the KF) and
// mimTemplate (which is a floating-point, zero-mean blurred version
// of the above)
void SmallBlurryImage::MakeFromKF(KeyFrame &kf, double dBlur)
{
  if(mirSize[0] == -1)
    mirSize = kf.aLevels[3].im.size() / 2;
  mbMadeJacs = false;
  
  mimSmall.resize(mirSize);
  mimTemplate.resize(mirSize);
  
  mbMadeJacs = false;
  halfSample(kf.aLevels[3].im, mimSmall);
  ImageRef ir;
  unsigned int nSum = 0;
  do
    nSum += mimSmall[ir];
  while(ir.next(mirSize));
  
  float fMean = ((float) nSum) / mirSize.area();
  
  ir.home();
  do
    mimTemplate[ir] = mimSmall[ir] - fMean;
  while(ir.next(mirSize));
  
  convolveGaussian(mimTemplate, dBlur);
}

// Make the jacobians (actually, no more than a gradient image)
// of the blurred template
void SmallBlurryImage::MakeJacs()
{
  mimImageJacs.resize(mirSize);
  // Fill in the gradient image
  ImageRef ir;
  do
    {
      Vector<2> &v2Grad = mimImageJacs[ir];
      if(mimTemplate.in_image_with_border(ir,1))
	{
	  v2Grad[0] = mimTemplate[ir + ImageRef(1,0)] - mimTemplate[ir - ImageRef(1,0)];
	  v2Grad[1] = mimTemplate[ir + ImageRef(0,1)] - mimTemplate[ir - ImageRef(0,1)];
	  // N.b. missing 0.5 factor in above, this will be added later.
	}
      else
	v2Grad = Zeros;
    }
  while(ir.next(mirSize));
  mbMadeJacs = true;
};

// Calculate the zero-mean SSD between one image and the next.
// Since both are zero mean already, just calculate the SSD...
double SmallBlurryImage::ZMSSD(SmallBlurryImage &other)
{
  double dSSD = 0.0;
  ImageRef ir;
  do
    {
      double dDiff = mimTemplate[ir] - other.mimTemplate[ir];
      dSSD += dDiff * dDiff;
    }
  while(ir.next(mirSize));
  return dSSD;
}


// Find an SE2 which best aligns an SBI to a target
// Do this by ESM-tracking a la Benhimane & Malis
pair<SE2<>,double> SmallBlurryImage::IteratePosRelToTarget(SmallBlurryImage &other, int nIterations)
{
  SE2<> se2CtoC;
  SE2<> se2WfromC;
  ImageRef irCenter = mirSize / 2;
  se2WfromC.get_translation() = vec(irCenter);

  pair<SE2<>, double> result_pair;
  if(!other.mbMadeJacs)
    {
      cerr << "You spanner, you didn't make the jacs for the target." << endl;
      assert(other.mbMadeJacs);
    };
  
  double dMeanOffset = 0.0;
  Vector<4> v4Accum;
  
  Vector<10> v10Triangle;
  Image<float> imWarped(mirSize);

  double dFinalScore = 0.0;
  for(int it = 0; it<nIterations; it++)
    {
      dFinalScore = 0.0;
      v4Accum = Zeros;
      v10Triangle = Zeros; // Holds the bottom-left triangle of JTJ
      Vector<4> v4Jac;
      v4Jac[3] = 1.0;
      
      SE2<> se2XForm = se2WfromC * se2CtoC * se2WfromC.inverse();
      
      // Make the warped current image template:
      Vector<2> v2Zero = Zeros;
      CVD::transform(mimTemplate, imWarped, se2XForm.get_rotation().get_matrix(), se2XForm.get_translation(), v2Zero, -9e20f);

      // Now compare images, calc differences, and current image jacobian:
      ImageRef ir;
      do
	{
	  if(!imWarped.in_image_with_border(ir,1))
	    continue;
	  float l,r,u,d,here;
	  l = imWarped[ir - ImageRef(1,0)];
	  r = imWarped[ir + ImageRef(1,0)];
	  u = imWarped[ir - ImageRef(0,1)];
	  d = imWarped[ir + ImageRef(0,1)];
	  here = imWarped[ir];
	  if(l + r + u + d + here < -9999.9)   // This means it's out of the image; c.f. the -9e20f param to transform.
	    continue;

	  Vector<2> v2CurrentGrad;
	  v2CurrentGrad[0] = r - l; // Missing 0.5 factor
	  v2CurrentGrad[1] = d - u;
	
	  Vector<2> v2SumGrad = 0.25 * (v2CurrentGrad  + other.mimImageJacs[ir]); 
	  // Why 0.25? This is from missing 0.5 factors: One for 
	  // the fact we average two gradients, the other from 
	  // each gradient missing a 0.5 factor.

	  v4Jac[0] = v2SumGrad[0];
	  v4Jac[1] = v2SumGrad[1];
	  v4Jac[2] = -(ir.y - irCenter.y) * v2SumGrad[0] + (ir.x - irCenter.x) * v2SumGrad[1];
	  //	  v4Jac[3] = 1.0;
	  
	  double dDiff = imWarped[ir] - other.mimTemplate[ir] + dMeanOffset;
	  dFinalScore += dDiff * dDiff;
	  
	  v4Accum += dDiff * v4Jac;
	  
	  // Speedy fill of the LL triangle of JTJ:
	  double *p = &v10Triangle[0];
	  *p++ += v4Jac[0] * v4Jac[0];
	  *p++ += v4Jac[1] * v4Jac[0];
	  *p++ += v4Jac[1] * v4Jac[1];
	  *p++ += v4Jac[2] * v4Jac[0];
	  *p++ += v4Jac[2] * v4Jac[1];
	  *p++ += v4Jac[2] * v4Jac[2];
	  *p++ += v4Jac[0];
	  *p++ += v4Jac[1];
	  *p++ += v4Jac[2];
	  *p++ += 1.0;
	}
      while(ir.next(mirSize));
      
      Vector<4> v4Update;
      
      // Solve for JTJ-1JTv;
      {
	Matrix<4> m4;
	int v=0;
	for(int j=0; j<4; j++)
	  for(int i=0; i<=j; i++)
	    m4[j][i] = m4[i][j] = v10Triangle[v++];
	Cholesky<4> chol(m4);
	v4Update = chol.backsub(v4Accum);
      }
      
      SE2<> se2Update;
      se2Update.get_translation() = -v4Update.slice<0,2>();
      se2Update.get_rotation() = SO2<>::exp(-v4Update[2]);
      se2CtoC = se2CtoC * se2Update;
      dMeanOffset -= v4Update[3];
    }

  result_pair.first = se2CtoC;
  result_pair.second = dFinalScore;
  return result_pair;
}


// What is the 3D camera rotation (zero trans) SE3<> which causes an
// input image SO2 rotation?
SE3<> SmallBlurryImage::SE3fromSE2(SE2<> se2, ATANCamera camera) 
{
  // Do this by projecting two points, and then iterating the SE3<> (SO3
  // actually) until convergence. It might seem stupid doing this so
  // precisely when the whole SE2-finding is one big hack, but hey.
  
  camera.SetImageSize(mirSize);
  
  Vector<2> av2Turned[2];   // Our two warped points in pixels
  av2Turned[0] = vec(mirSize / 2) + se2 * vec(ImageRef(5,0));
  av2Turned[1] = vec(mirSize / 2) + se2 * vec(ImageRef(-5,0));
  
  Vector<3> av3OrigPoints[2];   // 3D versions of these points.
  av3OrigPoints[0] = unproject(camera.UnProject(vec(mirSize / 2) + vec(ImageRef(5,0))));
  av3OrigPoints[1] = unproject(camera.UnProject(vec(mirSize / 2) + vec(ImageRef(-5,0))));
  
  SO3<> so3;
  for(int it = 0; it<3; it++)
    {
      WLS<3> wls;  // lazy; no need for the 'W'
      wls.add_prior(10.0);
      for(int i=0; i<2; i++)
	{
	  // Project into the image to find error
	  Vector<3> v3Cam = so3 * av3OrigPoints[i];
	  Vector<2> v2Implane = project(v3Cam);
	  Vector<2> v2Pixels = camera.Project(v2Implane);
	  Vector<2> v2Error = av2Turned[i] - v2Pixels;
	  
	  Matrix<2> m2CamDerivs = camera.GetProjectionDerivs();
	  Matrix<2,3> m23Jacobian;
	  double dOneOverCameraZ = 1.0 / v3Cam[2];
	  for(int m=0; m<3; m++)
	    {
	      const Vector<3> v3Motion = SO3<>::generator_field(m, v3Cam);
	      Vector<2> v2CamFrameMotion;
	      v2CamFrameMotion[0] = (v3Motion[0] - v3Cam[0] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	      v2CamFrameMotion[1] = (v3Motion[1] - v3Cam[1] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	      m23Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
	    };
	  wls.add_mJ(v2Error[0], m23Jacobian[0], 1.0);
	  wls.add_mJ(v2Error[1], m23Jacobian[1], 1.0);
	};
      
      wls.compute();
      Vector<3> v3Res = wls.get_mu();
      so3 = SO3<>::exp(v3Res) * so3;
    };
  
  SE3<> se3Result;
  se3Result.get_rotation() = so3;
  return se3Result;
}










