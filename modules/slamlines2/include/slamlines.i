/* $Id$ */

/** swig interface file for module slamlines.
 *
 * \file slamlines.i
 * \ingroup slamlines
 */      

%module slamlines

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"

// using namespace jafar::slamlines;
#include "slamlines/imageSegmentManager.hpp"

%}

%include "jafar.i"
%include "slamlinesException.i"

%import "slam/bearingOnlySlam.hpp"

%include "slamlines/imageSegmentManager.hpp"

/*
 * headers to be wrapped goes here
 */

%include "slamlinesTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::slamlines::print<jafar::slamlines::Type>;
%template(print) jafar::slamlines::print<jafar::slamlines::ImageSegment>;


