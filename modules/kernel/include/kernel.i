/* $Id$ */

/** swig/tcl interface file for module kernel.
 *
 * \file kernel.i
 * \ingroup kernel
 */      

%module kernel

%{

#include <string>

#include "kernel/jafarException.hpp"
#include "kernel/jafarDebug.hpp"
#include "kernel/dataLog.hpp"
#include "kernel/timingTools.hpp"

%}

%include "std_string.i"

%include "kernelException.i"

/*
 * wrapped headers 
 */       

%include "kernelTools.i"
%template(print) jafar::kernel::print<jafar::kernel::DataLog>;

%include "kernel/dataLog.hpp"

%include "kernel/jafarDebug.hpp"

%include "kernel/timingTools.hpp"
