// Copyright 2008 Isis Innovation Limited
#ifndef __OPENGL_INCLUDES_H
#define __OPENGL_INCLUDES_H

#ifdef _LINUX
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#ifdef _OSX
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#endif

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <GL/glew.h>
#endif

#include <cvd/gl_helpers.h>
#endif
