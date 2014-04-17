Parallel Tracking and Mapping for Small AR Workspaces
-----------------------------------------------------
Source Code Release v1.0-r114
Package compiled on Fri, 29 Jan 2010 02:06:18 +0000
Copyright 2008 Isis Innovation Limited

This software is an implementation of the method described in the
paper `Parallel Tracking and Mapping for Small AR Workspaces' by 
Georg Klein and David Murray, which appeared in the proceedings 
of the IEEE/ACM International Symposium on Mixed and Augmented
Reality (ISMAR) 2007.

This release is aimed at experienced software developers and/or
researchers familiar with implementing real-time vision algorithms in
C++ on the platform of their choice.

Questions? E-mail
ptam@robots.ox.ac.uk

REQUIREMENTS
------------
Supported Operating Systems: 
----------------------------
The code was developed on x86/x86-64 Linux. It has also been ported to
Intel-based MacOS X (using X11 for the display). These two operating
systems are supported.

The software can also be compiled on Win32, but this is not our target
platform and so this port is not very clean; see the Windows section later
in this document.

Processor Requirements: 
-----------------------
The software runs at least two processing-intensive threads at the
same time and so needs at least a dual-core machine. Intel Core 2 Duo
processors 2.4GHz+ are fine.

Graphics:
---------
The software requires accelerated OpenGL graphics output. It has been
written and tested only with nVidia cards: this is primarily of
concern under Linux, where the use of an nVidia card and the
proprietary nVidia display drivers are highly recommended. Since the
Linux code compiles directly against the nVidia driver's GL headers,
use of a different GL driver may require some modifications to the
code.

Video Input:
------------
The software requires a video camera with a wide-angle lens, capable
of 640x480x30Hz video capture and an appropriate driver installation
(which is supported by libCVD.) Only monochrome capture is needed for
tracking, colour images are used only for the AR display. A futher
discussion of video input follows later in this file.

Libraries:
----------
The software has three principal dependencies: 

1. TooN - a header library for linear algebra 
2. libCVD - a library for image handling, video capture and computer
vision
3. Gvars3 - a run-time configuration/scripting library, this is a
sub-project of libCVD.

All three above are written by member of the Cambridge Machine
Intelligence lab and are licensed under the LGPL.

Current versions are available from Savannah via CVS:
http://savannah.nongnu.org/projects/toon (for TooN)
http://savannah.nongnu.org/projects/libcvd (for libCVD and GVars)

The latest version of these libraries can be obtained via CVS and ssh:

# export CVS_RSH=ssh
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/toon co TooN
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co libcvd
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co gvars3

It should be noted, however, that the libraries change rapidly. To
ensure compatibility, it may be useful to download the libraries
corresponding to a time at which they were known to be compatible. To
do so, use the following commands:

# export CVS_RSH=ssh
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/toon co -D "Mon May 11 16:29:26 BST 2009" TooN
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co -D "Mon May 11 16:29:26 BST 2009" libcvd
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co -D "Mon May 11 16:29:26 BST 2009" gvars3

The installation of these libraries is described below.

INSTALLATION
------------
Installation of the dependencies
--------------------------------

The three dependent libraries are all compiled and installed using the
familiar ./configure; make; make install system, however the following
points are worth noting.

On Linux, the following libraries (and their -devel versions) are
required: blas, lapack, perhaps libgfortran, ncurses and libreadline
(optional, for GVars3), libdc1394 (and maybe libraw1394)
for firewire capture, optionally libtiff, libjpeg, libpng.

(On OSX, these or their equivalents should all be available with the
system.)

The order of installation should be 
1. TooN, 2. libCVD, 3. GVars3;

TooN installation is trivial, since it's only a bunch of headers.

For libCVD, I recommend the following configure options:
# export CXXFLAGS=-D_REENTRANT
# ./configure --without-ffmpeg

if the compiler hangs on one of the FAST detectors these can be
disabled with configure --disable-fast7 (for example.)
Documentation can be generated (if doxygen is installed) by running
# make docs

For GVars3, I recommend the following configure options:
# ./configure --disable-widgets

Compiling the Software
----------------------
The source code is in the PTAM directory.

The first step is to copy the appropriate platform build files to the
PTAM source directory. Eg. for linux, copy all the files from
PTAM/Build/Linux to PTAM. The Makefile can then be edited to
reference any custom include or linker paths which might be necessary
(depending on where the dependencies were installed.)

The second step, for Linux, is to set up the correct video source. Two
files are provided, VideoSource_Linux_DV.cc and
VideoSource_Linux_V4L.cc, which work with the Unibrain Fire-i and the
Logitech Quickcam Pro 5000 respectively. The DV version is compiled by
default; edit the Makefile to switch to the V4L version instead, if
needed. Other cameras may require manual editing of the video input
files, e.g. to change the videobuffer's colourspace.

Other video source classes are available with libCVD. Finally, if a
custom video source not supported by libCVD is required, the code for
it will have to be put into some VideoSource_XYZ.cc file (the
interface for this file is very simple.)

The software can then be compiled with the command 
# make

This builds two target executables: PTAM and CameraCalibrator.

RUNNING THE SOFTWARE
--------------------
Calibrating the Camera
----------------------
CameraCalibrator should be run first to obtain a camera calibration
(and to verify that video input is in fact working.) This requires the
user to point the camera at a checker-board calibration pattern; any
checkerboard of any size will do, a sample is included as
calib_pattern.pdf.

The camera calibrator attempts to find square corners in the image,
and then to link these together. This is indicated by fragments of
squares appearing in the image. The calibrator is not very robust; If
no squares are detected, some things to try would be:
- Modify camera settings to remove any sharpening; Sharpening
  artefacts break the calibrator, and also reduce performance of the
  tracker.
- Adjust the calibrator's settings, for example increase the value of
  CameraCalibrator.BlurSigma

When the camera is in a pose in which some portions of the grid are
detected, the user should press the `GrabFrame' button to add a
snapshot of that frame to the optimiser. After a few frames from
different poses have been added, pressing `Optimize' button
iteratively calculates the camera parameters. When the user is
satisfied with convergence (the RMS error should be no more than
around 0.3 pixels) pressing `Save' stores the camera calibration in a
file camera.cfg.

Running the Tracker
-------------------
Once a calibration has been stored, invoking PTAM runs the
tracker.

At the start, the tracker requires the user to provide a stereo pair
to initialise the map. The user does this by pointing the camera at an
angle to a planar (or near-planar) surface to be augmented, pressing
space-bar, slowly translating the camera to provide a baseline, and
pressing space-bar again to complete the stereo pair. At this point a
map is created and the tracker runs. Simple augmented graphics can be
shown once the tracker is running by pressing the "Draw AR" toggle
button.

If there appear to be problems with the map not being expanded or not
being tracked well, the most likely culprit is a lack of baseline,
i.e. the camera was not translated enough. A stereo initialisation
with only rotation does not provide a baseline and cannot ever work.

N.b. you don't need the calibration grid for the tracker! Any planar
or near-planar textured scene will do.

General Use of GVars
--------------------
Both programs rely on GVars for a console user-interface. In the
terminal window, a user may inspect a list of all tweakable variables
by typing 
> gvarlist 
(or gvarlist -a for a more complete list) and then modify variables by
typing
> Variable_Name = new_value
This allows the user to tweak a number of the program's features; for
example, if the quality of video input is dubious, typing
DrawFASTCorners=1 in the tracker allows the user to inspect the
response of the FAST corner detector.

VIDEO SOURCES
-------------
The software was developed with a Unibrain Fire-i colour camera, using
a 2.1mm M12 (board-mount) wide-angle lens. It also runs well with a
Logitech Quickcam Pro 5000 camera, modified to use the same 2.1mm M12
lens.

Wide-angle lenses are important for natural feature trackers and SLAM
systems, and the software's performance using a zoomier lens is likely
to be compromised. It will require more key-frames, robustness to
rotation will decrease, and relocaliser performance will drop.

How wide is wide-angle? The first number in camera.cfg is a normalized
horizontal focal length (fx): for our wide-angle lenses fx=0.57. Up to
fx<=1.0 the system would be expected to run fine, beyond that
performance will likely be degraded.

Independent of OS, it is important to turn down in-camera sharpening to the
point that no sharpening artifacts appear! Sharpening artifacts produce
spurious corners from image noise, move the location of real corners, and
break scale and rotation invariance. Expect poor performance if running on
an over-sharpened image. For example, on the unibrain fire-i, turn sharpness
down from the default 80 to 25.

Linux fire-i notes: Firewire with libCVD has been tested against the
old-style (pre-juju) firewire stack, using either the libDC-1 and
libDC-2 series. If your distribution ships with firewire support in
the form of the experimental juju stack (e.g. Fedora 8 and 9) you may
experience video input lockups, you should install packages for the
old firewire stack instead.

Linux Logitech Quickcam notes: the Logitech Quickcam pro 5000 is
supported by the linux-uvc driver and can be used with a
CVD::V4LBuffer<yuv422>.

MacOS X notes: Video input properties are reset every time the software
is run, and so settings will have to be adjusted every time
(e.g. turning down in-camera sharpening.) 

Windows notes: If attempting to port to windows, note that we have
experienced poor performance in conjunction with DSVL: this seems to be a
threading issue, as the video source also uses a thread, which can get
starved.  Using the unibrain or point grey APIs as appropriate would be
preferable. Note also that YUV->RGB->greyscale produces notable artefacts as
opposed to direct YUV->greyscale. At the moment, only a CMU1394 interface is
included, this works fine.

THE SOURCE CODE 
--------------- 
The main documentation of the code is in the code's comments. What
follows here is an overview of the tracking system.

The tracker runs from main.cc, which spawns a System-class which
starts two main threads:

1. The Tracker-class thread, driven by the System class event loop,
which performs real-time tracking and video I/O and graphical display;

2. The MapMaker-class thread, which updates the map in the
background.

Both threads access a common Map-class data structure. The Map-class
contains two main arrays: a vector of MapPoint-structs, and a vector
of KeyFrame-structs. Together these make up the map.

The bulk of the functionality is contained within the classes Tracker
and MapMaker, which also make use of auxiliary classes. The tracker
uses the Relocaliser-class to recover from failure, MapMaker uses the
Bundle-class to perform bundle adjustment and the HomographyInit-class
to bootstrap the map. Many bits of code use ATANCamera-class, which is
a model of the camera's intrinsic parameters.

Further, ARDriver-class provides distorted rendering and compositing
to run the AR simulation called EyeGame-class.

COMPILING AND RUNNING ON WINDOWS
--------------------------------
The software compiles fine on Windows, but the software remains a console
application (gvars interface is in a terminal window) and we experience some
frame-drop issues which don't seem to arise on other platforms.

To compile (and maybe also to run) the software on windows, the following
libraries are needed:

Lapack and BLAS - available pre-compiled e.g. from 
http://www.fi.muni.cz/~xsvobod2/misc/lapack/

pthreads from 
http://sourceware.org/pthreads-win32/

GLEW from 
http://glew.sourceforge.net/

CMU1394 camera driver from 
http://www.cs.cmu.edu/~iwan/1394/

libjpeg for win32 e.g. from 
http://gnuwin32.sourceforge.net/packages/jpeg.htm

Obtain LibCVD, TooN, and GVars3 from CVS as above. To compile and install,
libCVD and GVars3 come with msdev project files, TooN is just a bunch of
headers (copy it to your include directory.)

Copy the contents of PTAM/Build/Win32 to PTAM; then open
PTAM.sln, which contains projects to build the Camera Calibrator and
the Tracker. Edit include and lib directories as appropriate. Good luck!
