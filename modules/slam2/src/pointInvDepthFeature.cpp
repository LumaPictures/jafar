/* $Id$ */

#include "slam/eulerTools.hpp"
#include "slam/pointInvDepthFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::slam;


/*
 * class PointInvDepthFeatureModel
 */

PointInvDepthFeatureModel::PointInvDepthFeatureModel(): 
  FeatureModel(6, 7, 3),
  J3dPoint(3,7)
{
  // constant Jacobian values
  Jframe.clear();
  Jx.clear();
  Jx(6,6) = 1.0;

  J3dPoint.clear();
  J3dPoint(0,0) = 1.0;
  J3dPoint(1,1) = 1.0;
  J3dPoint(2,2) = 1.0;
}

PointInvDepthFeatureModel::~PointInvDepthFeatureModel() {}


void PointInvDepthFeatureModel::toFrame(const jblas::vec& frame_, jblas::vec const& x_, 
					jblas::vec& xRes) 
{
  vec_range xRes1(xRes, range(0,3));
  EulerTools::toFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, 
		      xRes1);
  vec_range xRes2(xRes, range(3,6));
  EulerTools::toFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, 
		      xRes2);
  xRes(6) = x_(6);
}

void PointInvDepthFeatureModel::toFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  mat_range Jframe1(Jframe, range(0,3), range(0,6));
  mat_range Jx1(Jx, range(0,3), range(0,3));
  EulerTools::toFrameJac(frame_, project(x_, range(0,3)), EulerTools::h_1, 
			 Jframe1, Jx1 );

  mat_range Jframe2(Jframe, range(3,6), range(0,6));
  mat_range Jx2(Jx, range(3,6), range(3,6));
  EulerTools::toFrameJac(frame_, project(x_, range(3,6)), EulerTools::h_0, 
			 Jframe2, Jx2);
}

void PointInvDepthFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes) {
  typedef ublas::vector_range<vec_range> vec_range_range;

  vec_range_range xRes1(xRes, range(0,3));
  EulerTools::fromFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, xRes1);
  vec_range_range xRes2(xRes, range(3,6));
  EulerTools::fromFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, xRes2);
  xRes(6) = x_(6);
}

void PointInvDepthFeatureModel::fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) {
  vec_range xRes1(xRes, range(0,3));
  EulerTools::fromFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, xRes1);

  vec_range xRes2(xRes, range(3,6));
  EulerTools::fromFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, xRes2);

  xRes(6) = x_(6);
}
      
void PointInvDepthFeatureModel::fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  mat_range Jframe1(Jframe, range(0,3), range(0,6));
  mat_range Jx1(Jx, range(0,3), range(0,3));
  EulerTools::fromFrameJac(frame_, project(x_, range(0,3)), EulerTools::h_1, 
			   Jframe1,
			   Jx1 );

  mat_range Jframe2(Jframe, range(3,6), range(0,6));
  mat_range Jx2(Jx, range(3,6), range(3,6));
  EulerTools::fromFrameJac(frame_, project(x_, range(3,6)), EulerTools::h_0, 
			   Jframe2,
			   Jx2 );
}

void PointInvDepthFeatureModel::compute3dPoint(jblas::vec const& x, jblas::vec& pt)
{
  JFR_PRECOND(x.size() == 7,
	      "PointInvDepthFeatureModel::compute3dPoint: invalid size");
  JFR_PRECOND(pt.size() == 3,
	      "PointInvDepthFeatureModel::compute3dPoint: invalid size");

	double id = x(6);
	if (id <= 0) 
	{ 
		JFR_WARNING("Negative estimated inverse depth " << id << " set to 1e-10 for 3dPoint computation");
		id = 1e-10;
	}
  pt.assign(project(x, range(0,3)) +  project(x, range(3,6))/id);
}

void PointInvDepthFeatureModel::compute3dPointJac(jblas::vec const& x)
{
	double id = x(6);
	if (id <= 0) 
	{ 
		JFR_WARNING("Negative estimated inverse depth " << id << " set to 1e-10 for 3dPointJac computation");
		id = 1e-10;
	}
  double d = 1.0/id;
  J3dPoint(0,3) = d;
  J3dPoint(1,4) = d;
  J3dPoint(2,5) = d;

  double dd = -1.0 / (id*id);
  J3dPoint(0,6) = dd * x(3);
  J3dPoint(1,6) = dd * x(4);
  J3dPoint(2,6) = dd * x(5);
}

void PointInvDepthFeatureModel::computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac )
{
  compute3dPoint( x_, mergeX_ );
  compute3dPointJac( x_ );
  jac.assign( J3dPoint );
}

/*
 * class BearingPointInvDepthFeatureObserveModel
 */

BearingPointInvDepthFeatureObserveModel::BearingPointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_) :
  FeatureObserveModel(featureModel_, 2),
  pointInvDepthFeatureModel(featureModel_)  
{
  setup();
}

BearingPointInvDepthFeatureObserveModel::~BearingPointInvDepthFeatureObserveModel() {}

vec const& BearingPointInvDepthFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);

  z(0) = atan2(pt(1), pt(0));
  z(1) = -atan2(pt(2), sqrt(pt(0)*pt(0) + pt(1)*pt(1)));
  return z;
}

void BearingPointInvDepthFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_)
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);
  pointInvDepthFeatureModel.compute3dPointJac(f_);

  jblas::mat Jproj(2,3);

  double pt_x = pt(0);
  double pt_y = pt(1);
  double pt_z = pt(2);

  /* copy/paste from maple */
  double t1 = pt_x * pt_x;
  double t2 = 1.0 / t1;
  double t4 = pt_y * pt_y;
  double t7 = 1.0 / (1.0 + t4 * t2);
  double t11 = t1 + t4;
  double t12 = sqrt(t11);
  double t15 = pt_z / t12 / t11;
  double t16 = pt_z * pt_z;
  double t20 = 0.1e1 / (0.1e1 + t16 / t11);

  Jproj(0,0) = -pt_y * t2 * t7;
  Jproj(0,1) = 1 / pt_x * t7;
  Jproj(0,2) = 0;
  Jproj(1,0) = t15 * pt_x * t20;
  Jproj(1,1) = t15 * pt_y * t20;
  Jproj(1,2) = -0.1e1 / t12 * t20;
  /* end copy/paste from maple */

  JobsSensor.assign(prod(Jproj, pointInvDepthFeatureModel.J3dPoint));
}

jblas::vec const& BearingPointInvDepthFeatureObserveModel::computeInnovation(jblas::vec const& z_,
									     jblas::vec const& zPred_)
{
  z = z_-zPred_;
  if (z(0) > M_PI)
    z(0) -= 2*M_PI;
  if (z(0) < -M_PI)
    z(0) += 2*M_PI;
  return z;
}

vec BearingPointInvDepthFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {

  vec x(7);
  x.clear();
  x(3) = cos(obs_.z(1))*cos(obs_.z(0));
  x(4) = cos(obs_.z(1))*sin(obs_.z(0));
  x(5) = -sin(obs_.z(1));
  x(6) = m_id0;

  return x;
}

void BearingPointInvDepthFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  JinvObs.clear();
  jblas::mat_range Jdir(JinvObs, range(3,6), range(0,2));
  double theta = obs_.z(0);
  double phi = obs_.z(1);

  /* copy/paste from maple */
  double t1 = cos(phi);
  double t2 = sin(theta);
  double t4 = sin(phi);
  double t5 = cos(theta);
  Jdir(0,0) = -t1 * t2;
  Jdir(0,1) = -t4 * t5;
  Jdir(1,0) = t1 * t5;
  Jdir(1,1) = -t4 * t2;
  Jdir(2,0) = 0.0e0;
  Jdir(2,1) = -t1;
  /* end copy/paste from maple */
}

void BearingPointInvDepthFeatureObserveModel::postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature_) {
  JFR_PRECOND(P.size1() == 7,
	      "BearingPointInvDepthFeatureObserveModel::postInitCovariance: invalid size");
  P(6,6) = m_sigmaId0 * m_sigmaId0;
}

void BearingPointInvDepthFeatureObserveModel::setup(double id0, double sigmaId0) {
  JFR_PRECOND(id0 > 0 && sigmaId0 > 0,
	      "BearingPointInvDepthFeatureObserveModel::setup: invalid values");
  m_id0 = id0;
  m_sigmaId0 = sigmaId0;

}



/*
 * class OmniImagePointInvDepthFeatureObserveModel
 */

OmniImagePointInvDepthFeatureObserveModel::OmniImagePointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_,
										     camera::CameraParabolicBarreto const& camera_) :
  FeatureObserveModel(featureModel_, 2),
  camera(camera_),
  pointInvDepthFeatureModel(featureModel_)  
{
  setup();
}

OmniImagePointInvDepthFeatureObserveModel::~OmniImagePointInvDepthFeatureObserveModel() {}

vec const& OmniImagePointInvDepthFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);
  camera.project(pt,z);
JFR_DEBUG(__FILE__ << ":" << __LINE__ << "|OmniImagePointInvDepthFeatureObserveModel::predictObservationInSensorFrame " <<
 "pointInvDepthFeatureModel.compute3dPoint(" << f_ << ") = " << pt << " ; camera.project z = " << z);
  return z;
}

void OmniImagePointInvDepthFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_)
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);
  pointInvDepthFeatureModel.compute3dPointJac(f_);
  
  jblas::mat Jproj(2,3);
  camera.projectJac(pt, Jproj);
  
  JobsSensor.assign(prod(Jproj, pointInvDepthFeatureModel.J3dPoint));
}

vec OmniImagePointInvDepthFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
   vec x(7);
   x.clear();
   vec_range u(x, range(3,6));
   camera.imageToCameraFrameDirection(obs_.z, u);
   x(6) = m_id0;
   return x;
}

void OmniImagePointInvDepthFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  JinvObs.clear();
  jblas::mat_range Jdir(JinvObs, range(3,6), range(0,2));
  camera.imageToCameraFrameDirectionJac(obs_.z, Jdir);
}

void OmniImagePointInvDepthFeatureObserveModel::postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature_) {
  JFR_PRECOND(P.size1() == 7,
	      "OmniImagePointInvDepthFeatureObserveModel::postInitCovariance: invalid size");
  P(6,6) = m_sigmaId0 * m_sigmaId0;
}

void OmniImagePointInvDepthFeatureObserveModel::setup(double id0, double sigmaId0) {
  JFR_PRECOND(id0 > 0 && sigmaId0 > 0,
	      "OmniImagePointInvDepthFeatureObserveModel::setup: invalid values");
  m_id0 = id0;
  m_sigmaId0 = sigmaId0;

}

 /* class ImagePointFeatureObserveModel
 */

ImagePointInvDepthFeatureObserveModel::ImagePointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_, camera::CameraPinhole const& camera_) :
  FeatureObserveModel(featureModel_, 2),
  camera(camera_),
  pointInvDepthFeatureModel(featureModel_)
{
  setup();
}

ImagePointInvDepthFeatureObserveModel::~ImagePointInvDepthFeatureObserveModel() {}

vec const& ImagePointInvDepthFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);
  camera.project(pt,z);
  return z;
}

void ImagePointInvDepthFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{
  jblas::vec pt(3);
  pointInvDepthFeatureModel.compute3dPoint(f_, pt);
  pointInvDepthFeatureModel.compute3dPointJac(f_);
  
  jblas::mat Jproj(2,3);
  camera.projectJac(pt, Jproj);
  
  JobsSensor.assign(prod(Jproj, pointInvDepthFeatureModel.J3dPoint));
}

vec ImagePointInvDepthFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
   vec x(7);
   x.clear();
   vec_range u(x, range(3,6));
   camera.imageToCameraFrameDirection(obs_.z, u);
   x(6) = m_id0;
   return x;
}

void ImagePointInvDepthFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  JinvObs.clear();
  jblas::mat_range Jdir(JinvObs, range(3,6), range(0,2));
  camera.imageToCameraFrameDirectionJac(obs_.z, Jdir);
}

void ImagePointInvDepthFeatureObserveModel::postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature_) {
  JFR_PRECOND(P.size1() == 7,
	      "ImagePointInvDepthFeatureObserveModel::postInitCovariance: invalid size");
  P(6,6) = m_sigmaId0 * m_sigmaId0;
}


void ImagePointInvDepthFeatureObserveModel::setup(double id0, double sigmaId0) {
  JFR_PRECOND(id0 > 0 && sigmaId0 > 0,
	      "ImagePointInvDepthFeatureObserveModel::setup: invalid values");
  m_id0 = id0;
  m_sigmaId0 = sigmaId0;

}
