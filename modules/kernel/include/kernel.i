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

%inline %{

  namespace jafar {namespace kernel {

void srand(unsigned int seed)
{ std::srand(seed); }

int *new_int(int ivalue) {
  int *i = new int(ivalue);
  return i;
}
int get_int(int *i) {
  return *i;
}


}}
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
%include "kernel/IdFactory.hpp"

