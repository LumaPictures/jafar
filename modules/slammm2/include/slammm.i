/* $Id$ */

/** swig interface file for module slammm.
 *
 * \file slammm.i
 * \ingroup slammm
 */      

%module slammm

%{
/* ruby defines ALLOC which conflicts with boost */ 
#undef ALLOC
  
/* 
   * headers necessary to compile the wrapper
   */

#include "jafarConfig.h"
#include "slammm/ListMapsPoseManager.hpp"
#include "slammm/MultiMapManager.hpp"
#include "slammm/MapsPoseManager.hpp"

// using namespace jafar::slammm;

%}

%include "jafar.i"
%include "slammmException.i"

%import "slam/baseSlam.hpp"
%import "slam/slamEkf.hpp"
%import "jmath/jblas.i"
%import "geom/geomVec.i"
%import "geom/t3dEuler.hpp"
%import "kernel/dataLog.hpp"

%include "slammm/StartNewMapStrategy.hpp"
%include "slammm/MapsPoseManager.hpp"
%include "slammm/ListMapsPoseManager.hpp"
%include "slammm/MergeMapsStrategy.hpp"
%include "slammm/MultiMapManager.hpp"

/*
 * headers to be wrapped goes here
 */

%template(processObservations) jafar::slammm::MultiMapsSlam::processObservations<std::list<jafar::slam::Observation*> >;
%template(processObservations) jafar::slammm::MultiMapsSlam::processObservations<std::vector<jafar::slam::Observation*> >;

// %include "slammmTools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::slammm::print<jafar::slammm::Type>;


