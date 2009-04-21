/* $Id$ */

#include "kernel/jafarException.hpp"

#include "slam/eulerTools.hpp"
#include "slam/pointInfFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::slam;

/*
 * class PointInfFeatureModel
 */

PointInfFeatureModel::PointInfFeatureModel(): 
  FeatureModel(6, 3, 3) {}

PointInfFeatureModel::~PointInfFeatureModel() {}


void PointInfFeatureModel::toFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) 
{
  EulerTools::toFrame(frame_, x_, EulerTools::h_0, xRes);
}

void PointInfFeatureModel::toFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  EulerTools::toFrameJac(frame_, x_, EulerTools::h_0, Jframe, Jx);
}

void PointInfFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes) {
  EulerTools::fromFrame(frame_, x_, EulerTools::h_0, xRes);
}

void PointInfFeatureModel::fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) {
  EulerTools::fromFrame(frame_, x_, EulerTools::h_0, xRes);
}
      
void PointInfFeatureModel::fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  EulerTools::fromFrameJac(frame_, x_, EulerTools::h_0, Jframe, Jx);
}

void PointInfFeatureModel::computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac )
{
  mergeX_ = x_;
  jac = jblas::identity_mat( sizeMergeState() );
  JFR_RUN_TIME("Not sure");
}

/*
 * BearingPointInfFeatureObserveModel
 */

BearingPointInfFeatureObserveModel::BearingPointInfFeatureObserveModel(PointInfFeatureModel& featureModel_) :
  InfFeatureObserveModel(featureModel_, 2)
{}

BearingPointInfFeatureObserveModel::~BearingPointInfFeatureObserveModel() {}

jblas::vec const& BearingPointInfFeatureObserveModel::predictObservationInSensorFrame(const jblas::vec& feature_)
{
  EulerTools::vectorToBearings(feature_,z);
  return z;
}

void BearingPointInfFeatureObserveModel::predictObservationInSensorFrameJac(const jblas::vec& feature_)
{
  EulerTools::vectorToBearingsJac(feature_, JobsSensor);
}

jblas::vec const& BearingPointInfFeatureObserveModel::computeInnovation(jblas::vec const& z_,
									jblas::vec const& zPred_)
{
  z = z_-zPred_;
  if (z(0) > M_PI)
    z(0) -= 2*M_PI;
  if (z(0) < -M_PI)
    z(0) += 2*M_PI;
  return z;
}

jblas::vec BearingPointInfFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_)
{
  vec x(3);
  EulerTools::bearingsToUnitVector(obs_.z, x);
  return x;
}

void BearingPointInfFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_)
{
  EulerTools::bearingsToUnitVectorJac(obs_.z, JinvObs);
}

double BearingPointInfFeatureObserveModel::computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const {
  vec3 dRef;
  dRef(0) = cos(obsRef.z(1))*cos(obsRef.z(0));
  dRef(1) = cos(obsRef.z(1))*sin(obsRef.z(0));
  dRef(2) = sin(obsRef.z(1));

  return sqrt(pow(norm_2(deltaPose),2) - pow(inner_prod(deltaPose, dRef),2) );
}

/*
 * class OmniImagePointInfFeatureObserveModel
 */

OmniImagePointInfFeatureObserveModel::OmniImagePointInfFeatureObserveModel(PointInfFeatureModel& featureModel_,
									   camera::CameraParabolicBarreto const& camera_) :
  InfFeatureObserveModel(featureModel_, 2),
  camera(camera_)
{}

OmniImagePointInfFeatureObserveModel::~OmniImagePointInfFeatureObserveModel() {}

vec const& OmniImagePointInfFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{  
  camera.project(f_,z);
  return z;
}

void OmniImagePointInfFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{
  camera.projectJac(f_, JobsSensor);
}

vec OmniImagePointInfFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
  vec x(3);
  camera.imageToCameraFrameDirection(obs_.z, x);
  return x;
}

void OmniImagePointInfFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  camera.imageToCameraFrameDirectionJac(obs_.z, JinvObs);
}

double OmniImagePointInfFeatureObserveModel::computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, 
									  Observation const& obsRef) const
{
  vec3 dRef;
  camera.imageToCameraFrameDirection(obsRef.z, dRef);

  return sqrt(pow(norm_2(deltaPose),2) - pow(inner_prod(deltaPose, dRef),2) );
}
