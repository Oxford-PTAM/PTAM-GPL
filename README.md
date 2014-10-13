PTAM-GPL
========

PTAM (Parallel Tracking and Mapping) re-released under GPLv3.

What is PTAM
------------

PTAM is a monocular SLAM (Simultaneous Localization and Mapping) system useful for real-time
6-DOF camera tracking in small scenes. It was originally developed as a research system in the Active 
Vision Laboratory of the University of Oxford, as described in the following papers:

- Georg Klein and David Murray, "Parallel Tracking and Mapping for Small AR Workspaces", Proc. ISMAR 2007
- Georg Klein and David Murray, "Improving the Agility of Keyframe-based SLAM", Proc. ECCV 2008

Building PTAM
------------
This library comes with a cmake script for Ubuntu Linux (cmake required).  It can be modified to work on other systems.

This system depends on libCVD, TooN, GVars, BLAS, LAPACK, Opencv and libGL.  These libraries can be found at the links below:

TooN (should be installed first)
https://github.com/edrosten/TooN

LibCVD
https://github.com/edrosten/libcvd

GVars
https://github.com/edrosten/gvars

BLAS
http://www.netlib.org/blas/
(also available on Ubuntu Linux as libblas-dev)

LAPACK
http://www.netlib.org/lapack/
(also available on Ubuntu Linux as liblapack-dev)

OpenCV
http://opencv.org/downloads.html
(also available on Ubuntu Linux as libopencv-core-dev)

libGL
Depends on operating system
(Available on Ubuntu Linux as libgl1-mesa-dev)

If opencv dependency is undesirable, other video source files can be found here:
http://ewokrampage.wordpress.com/video-sources/

Previous PTAM release
---------------------

PTAM was initially released in 2008 by Isis Innovation under a license suitable for
academic use. That version is also available for commercial use subject to a license
agreement with Isis Innovation. That version remains available here:
http://www.robots.ox.ac.uk/~gk/PTAM/

This code represents a re-licensed fork of Isis Innovation's PTAM Source Code Release v1.0-r114.


Bug fixing, new features, and branches
--------------------------------------

If you have a version of PTAM with changes that you would like to merge into this master version, please send a pull or patch request.

Requests will be placed in their own branch for review and testing. Therefore, branches may or may not work. Branches will eventually be merged into the master once we have had time to review and test.
