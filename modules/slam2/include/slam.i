/* $Id$ */

/** swig/tcl interface file for module slam.
 *
 * \file slam.i
 * \ingroup slam
 */      

/*#if defined(SWIGRUBY)
    
RUBY_ARRAY_TO_PTR_VECTOR(std::list<jafar::slam::Observation *>, jafar::slam::Observation)

#endif*/

%module(directors="1") slam

%{

/* ruby defines ALLOC which conflicts with boost */
#undef ALLOC

// to wrap std::string
#include <string> 

#include "geom/t3dEuler.hpp"

#include "slam/observation.hpp"
#include "slam/feature.hpp"
#include "slam/featureModel.hpp"
#include "slam/bearingOnlyFeature.hpp"
#include "slam/pointFeature.hpp"
#include "slam/basisFeature.hpp"
#include "slam/pointInfFeature.hpp"
#include "slam/pointInvDepthFeature.hpp"
#include "slam/segmentFeature.hpp"
#include "slam/abstractMapManager.hpp"
#include "slam/mapManager.hpp"
#include "slam/slamEkf.hpp"
#include "slam/bearingOnlySlam.hpp"
#include "slam/eulerTools.hpp"
#include "slam/odo3dPredictModel.hpp"
#include "slam/robot.hpp"
#include "slam/lineTools.hpp"
#include "slam/MapManagerFactory.hpp"
#include "slam/segmentInvDepthFeature.hpp"
#include "slam/segmentInvDepthInovFeature.hpp"

#include "slam/karma.hpp"
#include "slam/map3d.hpp"

#include "slam/dala.hpp"

#include "slam/full3dPredictModel.hpp"
#include "jmath/jblas.hpp"
  using namespace jafar::slam;
  using namespace jafar;

%}

%include "std_common.i"

// to wrap std::string
%include "std_string.i" 

%import "jafarConfig.h"

%include "jafar.i"
%include "slamException.i"

%import "geom/geomVec.i"

/*
 * wrapped headers 
 */ 

%import "jmath/jblas.i"

%import "kernel/dataLog.hpp"

%include "jmath/gaussianVector.hpp"

%include "filter/observeModel.hpp"

%include "filter/predictModel.hpp"
%include "slam/odo3dPredictModel.hpp"

%include "slam/abstractMapObject.hpp"
%include "slam/observation.hpp"
%include "slam/abstractMapManager.hpp"
%include "slam/feature.hpp"
%include "slam/featureModel.hpp"
%include "slam/bearingOnlyFeature.hpp"
%include "slam/baseSlam.hpp"
%include "slam/pointFeature.hpp"
%include "slam/basisFeature.hpp"
%include "slam/pointInfFeature.hpp"
%include "slam/pointInvDepthFeature.hpp"
%include "slam/segmentFeature.hpp"
%include "slam/robot.hpp"
%include "slam/MapManagerFactory.hpp"
%feature("director") jafar::slam::SlamEventListener;
%include "slam/slamEvents.hpp"
%include "slam/lineTools.hpp"
%include "slam/segmentInvDepthFeature.hpp"
%include "slam/segmentInvDepthInovFeature.hpp"

%include "slam/mapManager.hpp"

#if defined(SWIGRUBY)
    %apply SWIGTYPE *DISOWN { jafar::slam::FeatureObserveModel* model };
#endif
%include "slam/slamEkf.hpp"

%template(computeExtObs) jafar::slam::SlamEkf::computeExtObs<jafar::slam::ImagePluckerFeatureObserveModel>;
%template(computeExtObs) jafar::slam::SlamEkf::computeExtObs<jafar::slam::StereoImagePluckerFeatureObserveModel>;
%template(computeExtObs) jafar::slam::SlamEkf::computeExtObs<jafar::slam::ImageEuclideanPluckerFeatureObserveModel>;
%template(computeExtObs) jafar::slam::SlamEkf::computeExtObs<jafar::slam::ImageSegInvDepthFeatureObserveModel>;
%template(computeExtObs) jafar::slam::SlamEkf::computeExtObs<jafar::slam::ImageSegInvDepthInovFeatureObserveModel>;

%include "slam/bearingOnlySlam.hpp"

%include "slam/eulerTools.hpp"


#ifdef HAVE_TTL
%include "slam/map3d.hpp"
%include "slam/karma.hpp"
#endif

%include "slam/dala.hpp"

%include "slam/full3dPredictModel.hpp"


%include "slamTools.i"
%template(print) jafar::slam::print<jafar::slam::Observation>;
%template(print) jafar::slam::print<jafar::slam::SegmentObservation>;
%template(print) jafar::slam::print<jafar::slam::StereoSegmentObservation>;
%template(print) jafar::slam::print<jafar::slam::BaseFeature>;
%template(print) jafar::slam::print<jafar::slam::InitFeature>;
%template(print) jafar::slam::print<jafar::slam::SegmentFeature>;
%template(print) jafar::slam::print<jafar::slam::SlamEkf>;
%template(print) jafar::slam::print<jafar::slam::BearingOnlySlam>;

/* %template(processObservationsImpl) jafar::slam::SlamEkf::processObservationsImpl<std::list<jafar::slam::Observation*>, std::list<jafar::slam::Observation*> >; */


#ifdef HAVE_BOOST_SANDBOX
#ifdef HAVE_LAPACK
%template(covToEllipsoid) jafar::slam::covToEllipsoid<jblas::vec, jblas::sym_mat>;
%template(covToEllipsoid) jafar::slam::covToEllipsoid<jblas::vec3, jblas::sym_mat>;
%template(covToEllipsoid) jafar::slam::covToEllipsoid<jblas::vec_range, jblas::sym_mat_range>;
#endif
#endif

%extend jafar::slam::SegmentFeature {
  int id() {
    return self->id();
  }
};

%extend jafar::slam::SegmentIDFeature {
  int id() {
    return self->id();
  }
};

%include "std_map.i"
