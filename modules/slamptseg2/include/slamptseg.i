/* $Id$ */

/** swig interface file for module slamptseg.
 *
 * \file slamptseg.i
 * \ingroup slamptseg
 */      

%module slamptseg

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"

// using namespace jafar::slamptseg;

%}

%include "jafar.i"
%include "slamptsegException.i"

/*
 * headers to be wrapped goes here
 */

// %include "slamptsegTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::slamptseg::print<jafar::slamptseg::Type>;


