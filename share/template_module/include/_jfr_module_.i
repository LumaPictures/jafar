/* $Id:$ */

/** swig interface file for module _jfr_module_.
 *
 * \file _jfr_module_.i
 * \ingroup _jfr_module_
 */      

%module _jfr_module_

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"

// using namespace jafar::_jfr_module_;

%}

%include "jafar.i"
%include "_jfr_module_Exception.i"

/*
 * headers to be wrapped goes here
 */

// %include "_jfr_module_Tools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::_jfr_module_::print<jafar::_jfr_module_::Type>;

