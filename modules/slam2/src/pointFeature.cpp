/* $Id$*/

#include "kernel/jafarException.hpp"

#include "jmath/angle.hpp"

#include "slam/pointFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::jmath;
using namespace jafar::slam;

/*
 * class PointFeatureModel
 */

PointFeatureModel::PointFeatureModel(): 
  FeatureModel(6, 3, 3) {}

PointFeatureModel::~PointFeatureModel() {}

void PointFeatureModel::toFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) 
{
  EulerTools::toFrame(frame_, x_, EulerTools::h_1, xRes);
}

void PointFeatureModel::toFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  EulerTools::toFrameJac(frame_, x_, EulerTools::h_1, Jframe, Jx);
}

void PointFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes) {
  EulerTools::fromFrame(frame_, x_, EulerTools::h_1, xRes);
}
      
void PointFeatureModel::fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) {
  EulerTools::fromFrame(frame_, x_, EulerTools::h_1, xRes);
}
      
void PointFeatureModel::fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  EulerTools::fromFrameJac(frame_, x_, EulerTools::h_1, Jframe, Jx);
}

void PointFeatureModel::computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac  )
{
  mergeX_ = x_;
  jac = jblas::identity_mat( sizeMergeState() );
}

/*
 * class CartesianPointFeatureObserveModel
 */

CartesianPointFeatureObserveModel::CartesianPointFeatureObserveModel(PointFeatureModel& featureModel_) :
  FeatureObserveModel(featureModel_, 3)
{
  JobsSensor.assign(identity_mat(3));
  JinvObs.assign(identity_mat(3));
}

CartesianPointFeatureObserveModel::~CartesianPointFeatureObserveModel() {}

vec const& CartesianPointFeatureObserveModel::predictObservationInSensorFrame(const vec& feature_)
{
  z.assign(feature_);
  return z;
}

void CartesianPointFeatureObserveModel::predictObservationInSensorFrameJac(const vec& feature_) 
{
  // jacobian is the constant (3x3) identity matrix
  // initialized in the constructor
}

vec CartesianPointFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
  JFR_PRECOND(obs_.size() == sizeObs(),
              "CartesianPointFeatureObserveModel::inverseObservation: size of z_ does not match");
  vec x(3);
  x(0) = obs_.z(0);
  x(1) = obs_.z(1);
  x(2) = obs_.z(2);
  return x;
}

void  CartesianPointFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  // jacobian is the constant (3x3) identity matrix
  // initialized in the constructor
}

/*
 * class PolarPointFeatureObserveModel
 */

PolarPointFeatureObserveModel::PolarPointFeatureObserveModel(PointFeatureModel& featureModel_) :
  FeatureObserveModel(featureModel_, 3)
{}

PolarPointFeatureObserveModel::~PolarPointFeatureObserveModel() {}

void PolarPointFeatureObserveModel::setNoiseValues(double rhoUnitStdDev_, double thetaStdDev_, double phiStdDev_)
{
  p_isCorrelated = false;
  rhoUnitStdDev = rhoUnitStdDev_;
  R.clear();
  R(1,1) = pow(thetaStdDev_, 2);
  R(2,2) = pow(phiStdDev_, 2);
}

void PolarPointFeatureObserveModel::computeSensorR(Observation const& obs)
{
  R(0,0) = pow(obs.z(0)*obs.z(0)*rhoUnitStdDev, 2);
}


vec const& PolarPointFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{ 
  z(0) = norm_2(f_);
  z(1) = atan2(f_(1), f_(0));
  z(2) = -atan2(f_(2), sqrt(f_(0)*f_(0) + f_(1)*f_(1)));
  return z;
}

void PolarPointFeatureObserveModel::predictObservationInSensorFrameJac(const vec& feature_) 
{
  double f_x = feature_(0);
  double f_y = feature_(1);
  double f_z = feature_(2);

  /* copy/paste from maple */
  double t1 = fabs(f_x);
  double t2 = t1 * t1;
  double t3 = fabs(f_y);
  double t4 = t3 * t3;
  double t5 = fabs(f_z);
  double t6 = t5 * t5;
  double t8 = sqrt(t2 + t4 + t6);
  double t9 = 0.1e1 / t8;
  double t11 = fabs(f_x) /  f_x;
  double t14 = fabs(f_y) / f_y;
  double t17 = fabs(f_z) / f_z;
  double t19 = f_x * f_x;
  double t20 = 1.0 / t19;
  double t22 = f_y * f_y;
  double t25 = 1.0 / (1.0 + t22 * t20);
  double t29 = t19 + t22;
  double t30 = sqrt(t29);
  double t33 =  f_z / t30 / t29;
  double t34 = f_z * f_z;
  double t38 = 1.0 / (1.0 + t34 / t29);
  JobsSensor(0,0) = t9 * t1 * t11;
  JobsSensor(0,1) = t9 * t3 * t14;
  JobsSensor(0,2) = t9 * t5 * t17;
  JobsSensor(1,0) = -(f_y * t20 * t25);
  JobsSensor(1,1) = (1.0 / f_x * t25);
  JobsSensor(1,2) = 0.0e0;
  JobsSensor(2,0) = t33 * f_x * t38;
  JobsSensor(2,1) = t33 * f_y * t38;
  JobsSensor(2,2) = -0.1e1 / t30 * t38;

  /* end copy/paste from maple */
}

jblas::vec const& PolarPointFeatureObserveModel::computeInnovation(jblas::vec const& z_,
								   jblas::vec const& zPred_)
{
  z = z_-zPred_;
  if (z(0) > M_PI)
    z(0) -= 2*M_PI;
  if (z(0) < -M_PI)
    z(0) += 2*M_PI;
  return z;
}

vec PolarPointFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
  vec x(3);
  x(0) = obs_.z(0) * cos(obs_.z(2))*cos(obs_.z(1));
  x(1) = obs_.z(0) * cos(obs_.z(2))*sin(obs_.z(1));
  x(2) = -obs_.z(0) * sin(obs_.z(2));

  return x;
}

void PolarPointFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {

  double rho = obs_.z(0);
  double theta = obs_.z(1);
  double phi = obs_.z(2);

  /* copy/paste from maple */

  double t1 = cos(phi);
  double t2 = cos(theta);
  double t4 = rho * t1;
  double t5 = sin(theta);
  double t7 = sin(phi);
  double t8 = rho * t7;
  JinvObs(0,0) = t1 * t2;
  JinvObs(0,1) = -t4 * t5;
  JinvObs(0,2) = -t8 * t2;
  JinvObs(1,0) = t1 * t5;
  JinvObs(1,1) = t4 * t2;
  JinvObs(1,2) = -t8 * t5;
  JinvObs(2,0) = -t7;
  JinvObs(2,1) = 0.0e0;
  JinvObs(2,2) = -t4;

  /* end copy/paste from maple */
}

/*
 * class BearingPointFeatureObserveModel
 */

BearingPointFeatureObserveModel::BearingPointFeatureObserveModel(PointFeatureModel& featureModel_, double alpha_, double kSigma_) :
  BearingOnlyFeatureObserveModel(featureModel_, 2),
  alpha(alpha_), beta(kSigmaToBeta(alpha_, kSigma_))
{}

BearingPointFeatureObserveModel::~BearingPointFeatureObserveModel() {}

vec const& BearingPointFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{  
  z(0) = atan2(f_(1), f_(0));
  z(1) = -atan2(f_(2), sqrt(f_(0)*f_(0) + f_(1)*f_(1)));
  return z;
}

void BearingPointFeatureObserveModel::predictObservationInSensorFrameJac(const vec& feature_) 
{
  double f_x = feature_(0);
  double f_y = feature_(1);
  double f_z = feature_(2);

  /* copy/paste from maple */
  double t1 = f_x * f_x;
  double t2 = 1.0 / t1;
  double t4 = f_y * f_y;
  double t7 = 1.0 / (1.0 + t4 * t2);
  double t11 = t1 + t4;
  double t12 = sqrt(t11);
  double t15 = f_z / t12 / t11;
  double t16 = f_z * f_z;
  double t20 = 0.1e1 / (0.1e1 + t16 / t11);

  JobsSensor(0,0) = -f_y * t2 * t7;
  JobsSensor(0,1) = 1 / f_x * t7;
  JobsSensor(0,2) = 0;
  JobsSensor(1,0) = t15 * f_x * t20;
  JobsSensor(1,1) = t15 * f_y * t20;
  JobsSensor(1,2) = -0.1e1 / t12 * t20;
  /* end copy/paste from maple */
}

jblas::vec const& BearingPointFeatureObserveModel::computeInnovation(jblas::vec const& z_,
								     jblas::vec const& zPred_)
{
  z = z_-zPred_;
  if (z(0) > M_PI)
    z(0) -= 2*M_PI;
  if (z(0) < -M_PI)
    z(0) += 2*M_PI;
  return z;
}

void BearingPointFeatureObserveModel::initStateInSensorFrame(InitFeature& feature_,
							     Observation const& obs_,
                                                             double sMin_, double sMax_)
{
  //  JFR_DEBUG("BearingPointFeatureObserveModel::initState:");
  feature_.initState.clear();
  // the number of gaussians
  //  int n = 1 + int(ceil( log( ((1-alpha)/(1+alpha)) * sMax_/sMin_ )/log(beta) ));
  //JFR_DEBUG("n: " << n);
  //JFR_DEBUG("w: " << w);

  computeSensorR(obs_);

  jblas::vec3 d;
  jblas::sym_mat dCov(3,3);
  jblas::mat J(3,2);

  EulerTools::bearingsToUnitVector(obs_.z, d);
  EulerTools::bearingsToUnitVectorJac(obs_.z, J);
  dCov.assign(prod(J, mat(prod(R, trans(J))) ));

  boPointsGeometricGaussianSum(d, dCov, alpha, beta, sMin_, sMax_, feature_.initState);

//   JFR_DEBUG("number of gaussians: " << feature_.initState.size());
}

bool BearingPointFeatureObserveModel::doUpdateInitStateInSensorFrame(jblas::vec const& closestMemberState, Observation const& obsRef)
{
  vec dz = predictObservationInSensorFrame(closestMemberState) - obsRef.z;
  return (fabs(dz(0)) > sqrt(getR()(0,0))) || (fabs(dz(1)) > sqrt(getR()(1,1)));
}

/*
 * class ImagePointFeatureObserveModel
 */

ImagePointFeatureObserveModel::ImagePointFeatureObserveModel(PointFeatureModel& featureModel_, 
							     camera::CameraPinhole const& camera_,
							     double alpha_, double kSigma_) :
  BearingOnlyFeatureObserveModel(featureModel_, 2),
  camera(camera_),
  alpha(alpha_), beta(kSigmaToBeta(alpha_, kSigma_))
{}

ImagePointFeatureObserveModel::~ImagePointFeatureObserveModel() {}

vec const& ImagePointFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{  
  camera.project(f_,z);
  return z;
}

void ImagePointFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{
  camera.projectJac(f_, JobsSensor);
}

void ImagePointFeatureObserveModel::initStateInSensorFrame(InitFeature& feature_,
							   Observation const& obs_,
							   double sMin_, double sMax_)
{
  //  JFR_DEBUG("BearingPointFeatureObserveModel::initState:");
  feature_.initState.clear();
  // the number of gaussians
  //  int n = 1 + int(ceil( log( ((1-alpha)/(1+alpha)) * sMax_/sMin_ )/log(beta) ));
  //JFR_DEBUG("n: " << n);
  //JFR_DEBUG("w: " << w);

  jblas::vec3 d;
  jblas::sym_mat dCov(3,3);
  jblas::mat J(3,2);

  camera.imageToCameraFrameDirection(obs_.z, d);
  camera.imageToCameraFrameDirectionJac(obs_.z, J);

  computeSensorR(obs_);
  dCov.assign(prod( J, mat(prod(R, trans(J)))) );

  boPointsGeometricGaussianSum(d, dCov, alpha, beta, sMin_, sMax_, feature_.initState);

//   JFR_DEBUG("number of gaussians: " << feature_.initState.size());
}

/*
 * class StereoImagePointFeatureObserveModel
 */

StereoImagePointFeatureObserveModel::StereoImagePointFeatureObserveModel(PointFeatureModel& featureModel_,
									 jafar::camera::StereoBench const& stereoBench_) :
  FeatureObserveModel(featureModel_, 3),
  stereoBench(stereoBench_)
{}


StereoImagePointFeatureObserveModel::~StereoImagePointFeatureObserveModel() {}

jblas::vec const& StereoImagePointFeatureObserveModel::predictObservationInSensorFrame(vec const& f_)
{
  vec_range pix = project(z, range(0,2));
  JFR_TRACE_BEGIN;
  stereoBench.computePixDisp(f_, pix, z(2));
  JFR_TRACE_END("StereoImagePointFeatureObserveModel::predictObservationInSensorFrame:");
  return z;
}

void StereoImagePointFeatureObserveModel::predictObservationInSensorFrameJac(vec const& f_)
{
  JFR_TRACE_BEGIN;
  stereoBench.computePixDispJac(f_, JobsSensor);
  JFR_TRACE_END("StereoImagePointFeatureObserveModel::predictObservationInSensorFrameJac:");
}

jblas::vec StereoImagePointFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) 
{
  vec x(3);
  JFR_TRACE_BEGIN;
  stereoBench.compute3dPoint(project(obs_.z, range(0,2)), obs_.z(2), x);
  JFR_TRACE_END("StereoImagePointFeatureObserveModel::inverseObservationInSensorFrame:");
  return x; 
}

void StereoImagePointFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) 
{
  JFR_TRACE_BEGIN;
  stereoBench.compute3dPointJac(project(obs_.z, range(0,2)), obs_.z(2), JinvObs);
  JFR_TRACE_END("StereoImagePointFeatureObserveModel::inverseObservationInSensorFrameJac:");
}


/*
 * class OmniImagePointFeatureObserveModel
 */

OmniImagePointFeatureObserveModel::OmniImagePointFeatureObserveModel(PointFeatureModel& featureModel_, 
								     camera::CameraParabolicBarreto const& camera_) :
  BearingOnlyFeatureObserveModel(featureModel_, 2),
  camera(camera_)
{
  setBoParam();
  // default noise to 1 pixel
  R.clear();
  R(0,0) = 1.0;
  R(1,1) = 1.0;
}

OmniImagePointFeatureObserveModel::~OmniImagePointFeatureObserveModel() {}

void OmniImagePointFeatureObserveModel::setBoParam(double alpha_, double kSigma_)
{
  alpha = alpha_; 
  beta = kSigmaToBeta(alpha_, kSigma_);
}

vec const& OmniImagePointFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{  
  camera.project(f_,z);
  return z;
}

void OmniImagePointFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{
  camera.projectJac(f_, JobsSensor);
}

void OmniImagePointFeatureObserveModel::initStateInSensorFrame(InitFeature& feature_,
							       Observation const& obs_,
							       double sMin_, double sMax_)
{
  //  JFR_DEBUG("BearingPointFeatureObserveModel::initState:");
  feature_.initState.clear();

  jblas::vec3 d;
  jblas::sym_mat dCov(3,3);
  jblas::mat J(3,2);

  JFR_TRACE_BEGIN;

  camera.imageToCameraFrameDirection(obs_.z, d);
  camera.imageToCameraFrameDirectionJac(obs_.z, J);
  computeSensorR(obs_);
  dCov.assign(prod( J, mat(prod(R, trans(J)))) );

  boPointsGeometricGaussianSum(d, dCov, alpha, beta, sMin_, sMax_, feature_.initState);

  JFR_TRACE_END("OmniImagePointFeatureObserveModel::initStateInSensorFrame");
}

double OmniImagePointFeatureObserveModel::computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, 
								       Observation const& obsRef) const
{
  vec3 dRef;
  camera.imageToCameraFrameDirection(obsRef.z, dRef);

  return sqrt(pow(norm_2(deltaPose),2) - pow(inner_prod(deltaPose, dRef),2) );
}
