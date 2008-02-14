/* $Id$ */

/** swig interface file for module lines.
 *
 * \file lines.i
 * \ingroup lines
 */      

%module lines

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"
//#include "image/Image.hpp"
#include "lines/lineSegment.hpp"
#include "lines/lineSegmentSet.hpp"
#include "lines/lsTracker.hpp"
#include "lines/matchingSet.hpp"
#include "lines/ctdef.hpp"
#include "lines/lsMisc.hpp"
#include "lines/histogram.hpp"
#include "lines/lsExtractor.hpp"
#include "lines/lsPredictor2D.hpp"
#include "lines/constPositionKF.hpp"
#include "lines/constVelocityKF.hpp"

// using namespace jafar::lines;

%}

%include "jafar.i"
%include "linesException.i"

/*
 * headers to be wrapped goes here
 */

%include "jafarConfig.h"
%include "lines/lineSegment.hpp"
%include "lines/lineSegmentSet.hpp"
%include "lines/lsTracker.hpp"
%include "lines/matchingSet.hpp"
%include "lines/ctdef.hpp"
%include "lines/lsMisc.hpp"
%include "lines/histogram.hpp"
%include "lines/lsExtractor.hpp"
%include "lines/lsPredictor2D.hpp"
%include "lines/constPositionKF.hpp"
%include "lines/constVelocityKF.hpp"
    
// %include "linesTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::lines::print<jafar::lines::Type>;


