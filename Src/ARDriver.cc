// Copyright 2008 Isis Innovation Limited
#define GL_GLEXT_PROTOTYPES 1
#include "ARDriver.h"

using namespace GVars3;
using namespace CVD;
using namespace std;

static bool CheckFramebufferStatus();

ARDriver::ARDriver(const ATANCamera &cam, ImageRef irFrameSize, GLWindow2 &glw)
  :mCamera(cam), mGLWindow(glw)
{
  mirFrameSize = irFrameSize;
  mCamera.SetImageSize(mirFrameSize);
  mbInitialised = false;
}

void ARDriver::Init()
{
  mbInitialised = true;
  mirFBSize = GV3::get<ImageRef>("ARDriver.FrameBufferSize", ImageRef(1200,900), SILENT);
  glGenTextures(1, &mnFrameTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 
	       GL_RGBA, mirFrameSize.x, mirFrameSize.y, 0, 
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL); 
  MakeFrameBuffer();
  mGame.Init();
};

void ARDriver::Reset()
{
  mGame.Reset();
  mnCounter = 0;
}

void ARDriver::Render(Image<Rgb<byte> > &imFrame, SE3<> se3CfromW)
{
  if(!mbInitialised)
    {
      Init();
      Reset();
    };
  
  mnCounter++;
  
  // Upload the image to our frame texture
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
		  0, 0, 0,
		  mirFrameSize.x, mirFrameSize.y,
		  GL_RGB,
		  GL_UNSIGNED_BYTE,
		  imFrame.data());
  
  // Set up rendering to go the FBO, draw undistorted video frame into BG
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,mnFrameBuffer);
  CheckFramebufferStatus();
  glViewport(0,0,mirFBSize.x,mirFBSize.y);
  DrawFBBackGround();
  glClearDepth(1);
  glClear(GL_DEPTH_BUFFER_BIT);
  
  // Set up 3D projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrix(mCamera.MakeUFBLinearFrustumMatrix(0.005, 100));
  glMultMatrix(se3CfromW);
  
  DrawFadingGrid();
  
  mGame.DrawStuff(se3CfromW.inverse().get_translation());
  
  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_BLEND);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  // Set up for drawing 2D stuff:
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
  
  DrawDistortedFB();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();
}



void ARDriver::MakeFrameBuffer()
{
  // Needs nvidia drivers >= 97.46
  cout << "  ARDriver: Creating FBO... ";
  
  glGenTextures(1, &mnFrameBufferTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameBufferTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 
	       GL_RGBA, mirFBSize.x, mirFBSize.y, 0, 
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL); 
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  GLuint DepthBuffer;
  glGenRenderbuffersEXT(1, &DepthBuffer);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, DepthBuffer);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, mirFBSize.x, mirFBSize.y);

  glGenFramebuffersEXT(1, &mnFrameBuffer);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mnFrameBuffer); 
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, 
			    GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, 
  			       GL_RENDERBUFFER_EXT, DepthBuffer);
  
  CheckFramebufferStatus();
  cout << " .. created FBO." << endl;
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

static bool CheckFramebufferStatus()         
{                                            
  
  GLenum n;                                            
  n = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if(n == GL_FRAMEBUFFER_COMPLETE_EXT)
    return true; // All good
  
  cout << "glCheckFrameBufferStatusExt returned an error." << endl;
  return false;
};

void ARDriver::DrawFBBackGround()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupUnitOrtho();
  
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);  
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  // Cache the cpu-intesive projections in a display list..
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      glColor3f(1,1,1);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      for(int ystep = 0; ystep< nStepsY; ystep++)
	{  
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep <= nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep+1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		// If this is a border quad, draw a little beyond the
		// outside of the frame, this avoids strange jaggies
		// at the edge of the reconstructed frame later:
		if(xstep == 0 || yystep == 0 || xstep == nStepsX || yystep == nStepsY)
		  for(int i=0; i<2; i++)
		    v2Iter[i] = v2Iter[i] * 1.02 - 0.01; 
		Vector<2> v2UFBDistorted = v2Iter; 
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBDistorted[0] * mirFrameSize.x, v2UFBDistorted[1] * mirFrameSize.y);
		glVertex(v2UFBUnDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}


void ARDriver::DrawDistortedFB()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupViewport();
  mGLWindow.SetupUnitOrtho();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex);  
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      glColor3f(1,1,1);
      for(int ystep = 0; ystep<nStepsY; ystep++)
	{  
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep<=nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep + 1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		Vector<2> v2UFBDistorted = v2Iter; 
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBUnDistorted[0] * mirFBSize.x, (1.0 - v2UFBUnDistorted[1]) * mirFBSize.y);
		glVertex(v2UFBDistorted);
	      }	 
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

void ARDriver::DrawFadingGrid()
{
  double dStrength;
  if(mnCounter >= 60)
    return;
  if(mnCounter < 30)
    dStrength = 1.0;
  dStrength = (60 - mnCounter) / 30.0;
  
  glColor4f(1,1,1,dStrength);
  int nHalfCells = 8;
  if(mnCounter < 8)
    nHalfCells = mnCounter + 1;
  int nTot = nHalfCells * 2 + 1;
  Vector<3>  aaVertex[17][17];
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	Vector<3> v3;
	v3[0] = (i - nHalfCells) * 0.1;
	v3[1] = (j - nHalfCells) * 0.1;
	v3[2] = 0.0;
	aaVertex[i][j] = v3;
      }

  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[i][j]);
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[j][i]);
      glEnd();
    };
};







