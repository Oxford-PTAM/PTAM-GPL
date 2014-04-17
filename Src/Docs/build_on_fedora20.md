For detailed instructions, refer to [Build PTAM on Fedora 20 the Easy Way - 10 Minutes Tutorial](http://hustcalm.me/blog/2014/04/17/build-ptam-on-fedora-20-the-easy-way-10-minutes-tutorial/).

Dependencies:

*   [TooN](http://www.edwardrosten.com/cvd/toon.html)

A numerics library used by libCVD.

    sudo yum install liblapack-devel
    sudo yum install libblas-devel

    ./configure
    make
    sudo make install

*   [libCVD](http://www.edwardrosten.com/cvd/index.html)

libCVD is a very portable and high performance C++ library for computer vision, image, and video processing.

    sudo yum install freeglut-devel

    ./configure
    make
    sudo make install

*   [GVars3](http://www.edwardrosten.com/cvd/gvars3.html)

A configuration library which integrates well with TooN.

    ./configure
    make
    sudo make install

*   [OpenGL](http://web.eecs.umich.edu/~sugih/courses/eecs487/glut-howto/)

    sudo yum install freeglut-devel

PTAM:

    add **include <unistd.h>** to Tracker.cc.

    add -llapack -lGL -lGLU -lglut to linker commands in Makefile.

    make

V4L2:
    
    sudo yum install v4l-utils

    v4l2-ctl

Trouble-shooting:

*   Segmentation Fault as soon as ... got video source.

    Check if you got a Nvidia card and driver combo.
