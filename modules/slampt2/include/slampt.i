/* $Id$ */

/** swig interface file for module slampt.
 *
 * \file slampt.i
 * \ingroup slampt
 */      

%module slampt

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"

#include "slampt/imagePointManager.hpp"
#include "slampt/omniImagePointManager.hpp"

// using namespace jafar::slampt;

%}

%include "jafar.i"
%include "slamptException.i"

%import "slam/observation.hpp"
%import "kernel/dataLog.hpp"
%import "hpm/export.hpp"

%include "slampt/imagePointManager.hpp"
%include "slampt/omniImagePointManager.hpp"

/*
 * headers to be wrapped goes here
 */

%include "slamptTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::slampt::print<jafar::slampt::Type>;


