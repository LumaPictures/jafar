/* $Id$ */

/** swig interface file for module slamseg.
 *
 * \file slamseg.i
 * \ingroup slamseg
 */      

%module slamseg

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"

#include "slam/segmentFeature.hpp"

#include "slamseg/MonoImageSegmentManager.hpp"
#include "slamseg/ImageSegmentManager.hpp"
#include "slamseg/SlamPredictor.hpp"

// using namespace jafar::slamseg;

%}

%include "jafarConfig.h"
%include "jafar.i"
%include "slamsegException.i"

%import "dseg/Predictor.hpp"
%import "dseg/DirectSegmentsDetector.hpp"
%import "slam/observation.hpp"

%include "slamseg/MonoImageSegmentManager.hpp"
%include "slamseg/ImageSegmentManager.hpp"
%include "slamseg/SlamPredictor.hpp"

%template(RhoThetaImagePluckerFeatureObserveModelSlamPredictor) jafar::slamseg::SlamPredictor<jafar::slam::RhoThetaImagePluckerFeatureObserveModel>;

/*
 * headers to be wrapped goes here
 */

%include "slamsegTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::slamseg::print<jafar::slamseg::Type>;


