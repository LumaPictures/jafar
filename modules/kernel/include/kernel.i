/* $Id$ */

/** swig/tcl interface file for module kernel.
 *
 * \file kernel.i
 * \ingroup kernel
 */      

%module kernel

%{

#include <string>

#include "kernel/jafarDebug.hpp"
#include "kernel/dataLog.hpp"
#include "kernel/timingTools.hpp"
#include "kernel/IdFactory.hpp"

%}

%include "jafar.i"

/*
 * wrapped headers 
 */       

%include "kernelException.i"
%include "kernelTools.i"

/* More Ruby-like constants */
#ifdef SWIGRUBY
%rename(ON_TIME)  jafar::kernel::FrameRate::ontime;
%rename(ON_FRAME) jafar::kernel::FrameRate::onframe;
#endif

%include "kernel/dataLog.hpp"
%include "kernel/jafarDebug.hpp"
%include "kernel/timingTools.hpp"
%include "kernel/IdMaker.hpp"
