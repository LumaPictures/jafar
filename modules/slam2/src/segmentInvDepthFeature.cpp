/* $Id: segInvDepthFeature.cpp 3374 2008-12-01 18:17:38Z tvidal $ */

#include "slam/eulerTools.hpp"
#include "slam/lineTools.hpp"
#include "slam/segmentInvDepthFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::slam;

/*
 * class SegmentIDFeature
 */



//FIXME dynamic sizeObs
SegmentIDFeature::SegmentIDFeature(unsigned int id, FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs_) :
    BaseFeature(id, model, sizeObs, typeObs_),
    ENLARGE(true)
{
  segidmodel = dynamic_cast<SegInvDepthFeatureModel*>( &model );
  JFR_ASSERT( segidmodel, "Unsupported Feature model" );
  s1=0;
  s2=1;
}

SegmentIDFeature::~SegmentIDFeature() 
{
}

void SegmentIDFeature::setState(vec& x_, sym_mat& P_)
{
  BaseFeature::setState(x_, P_);
}

//TODO adapt to New Model
vec3 SegmentIDFeature::getExt1() const 
{
  vec3 ext;
  computeExt(s1, ext);  
  //JFR_DEBUG("ext1: " << ext);
  return ext;
}

sym_mat SegmentIDFeature::getExt1Cov() const
{
  sym_mat extCov(3,3);
  computeExtCov(s1,extCov); 
  //JFR_DEBUG("ext1Cov: " << extCov);
  return extCov;
}

mat SegmentIDFeature::getExt1Jac() const
{
  mat J1(3,11);
  computeExtJac(s1, J1);
  //JFR_DEBUG("ext1Cov: " << extCov);
  return J1;
}

vec3 SegmentIDFeature::getExt2() const 
{
  vec3 ext;
  computeExt(s2, ext);
  //JFR_DEBUG("ext2: " << ext);
  return ext;  
}

sym_mat SegmentIDFeature::getExt2Cov() const
{
  sym_mat extCov(3,3);
  computeExtCov(s2,extCov);  
  //JFR_DEBUG("ext2Cov: " << extCov);
  return extCov;
}

mat SegmentIDFeature::getExt2Jac() const
{
  mat J2(3,11);
  computeExtJac(s2, J2);
  //JFR_DEBUG("ext1Cov: " << extCov);
  return J2;
}


void SegmentIDFeature::computeExt(double s_, vec3& ext) const
{
   
  jblas::vec p(3);
  jblas::vec q(3);
  segidmodel->compute3dPoints( getX(), p, q);
  ext.assign((1-s_)*p + s_*q); // the 3D endpoint
  //JFR_DEBUG("abscissa: "<< s_);
}

void SegmentIDFeature::computeExtCov(double s_, sym_mat& extCov) const
{
  mat E_l(3,11); // the Jacobian wrt the line
  computeExtJac(s_, E_l);
  extCov.assign(prod(E_l, mat(prod(getP(), trans(E_l))) ));
  JFR_DEBUG("Covariace Extremities: " << extCov);
}


void SegmentIDFeature::computeExtJac(double s_, mat& E_l) const
{
  jblas::mat E_pts(3,6);
  E_pts.clear();
  subrange(E_pts,0,3,0,3).assign(identity_mat(3,3)*(1-s_));
  subrange(E_pts,0,3,3,6).assign(identity_mat(3,3)*(s_));
  segidmodel->compute3dPointsJac(jblas::vec(getX()));
  E_l.assign(ublas::prod(E_pts,segidmodel->J3dPoints)); // the Jacobian wrt the line
  JFR_DEBUG("Jacobian Extremities: " << E_l);
}


void SegmentIDFeature::writeLogHeader(jafar::kernel::DataLogger& log) const
{
  BaseFeature::writeLogHeader(log);

  log.writeComment("slam: SegmentFeature");
}
void SegmentIDFeature::writeLogData(jafar::kernel::DataLogger& log) const
{
  BaseFeature::writeLogData(log);
}

std::ostream& jafar::slam::operator <<(std::ostream& s, jafar::slam::SegmentIDFeature const& f)
{
  s << static_cast<BaseFeature const&>(f);
  s << "ext1: " << f.getExt1()
      << "ext2: " << f.getExt2();
  return s;
}




/*
 * class SegInvDepthFeatureModel
 */

SegInvDepthFeatureModel::SegInvDepthFeatureModel(): 
  FeatureModel(6, 11, 6),
  J3dPoints(6,11)
{
  // constant Jacobian values
  Jframe.clear();
  Jx.clear();
  Jx(6,6) = 1.0;
  Jx(10,10) = 1.0;

  J3dPoints.clear();
  J3dPoints(0,0) = 1.0;
  J3dPoints(1,1) = 1.0;
  J3dPoints(2,2) = 1.0;
}

SegInvDepthFeatureModel::~SegInvDepthFeatureModel() {}


void SegInvDepthFeatureModel::toFrame(const jblas::vec& frame_, jblas::vec const& x_, 
					jblas::vec& xRes) 
{
  vec_range xRes1(xRes, range(0,3));
  EulerTools::toFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, 
		      xRes1);
  vec_range xRes2(xRes, range(3,6));
  EulerTools::toFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, 
		      xRes2);
  xRes(6) = x_(6);
  vec_range xRes3(xRes, range(7,10));
  EulerTools::toFrame(frame_, project(x_, range(7,10)), EulerTools::h_0, 
                      xRes3);
  xRes(10) = x_(10);
}

void SegInvDepthFeatureModel::toFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
  mat_range Jframe1(Jframe, range(0,3), range(0,6));
  mat_range Jx1(Jx, range(0,3), range(0,3));
  EulerTools::toFrameJac(frame_, project(x_, range(0,3)), EulerTools::h_1, 
			 Jframe1, Jx1 );

  mat_range Jframe2(Jframe, range(3,6), range(0,6));
  mat_range Jx2(Jx, range(3,6), range(3,6));
  EulerTools::toFrameJac(frame_, project(x_, range(3,6)), EulerTools::h_0, 
			 Jframe2, Jx2);

  mat_range Jframe3(Jframe, range(7,10), range(0,6));
  mat_range Jx3(Jx, range(7,10), range(7,10));
  EulerTools::toFrameJac(frame_, project(x_, range(7,10)), EulerTools::h_0, 
                      Jframe3, Jx3);
}

void SegInvDepthFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes) {
  typedef ublas::vector_range<vec_range> vec_range_range;

  vec_range_range xRes1(xRes, range(0,3));
  EulerTools::fromFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, xRes1);
  vec_range_range xRes2(xRes, range(3,6));
  EulerTools::fromFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, xRes2);
  xRes(6) = x_(6);
  vec_range_range xRes3(xRes, range(7,10));
  EulerTools::fromFrame(frame_, project(x_, range(7,10)), EulerTools::h_0, xRes3);
  xRes(10) = x_(10);
}

void SegInvDepthFeatureModel::fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes) {
  vec_range xRes1(xRes, range(0,3));
  EulerTools::fromFrame(frame_, project(x_, range(0,3)), EulerTools::h_1, xRes1);

  vec_range xRes2(xRes, range(3,6));
  EulerTools::fromFrame(frame_, project(x_, range(3,6)), EulerTools::h_0, xRes2);

  xRes(6) = x_(6);

  vec_range xRes3(xRes, range(7,10));
  EulerTools::fromFrame(frame_, project(x_, range(7,10)), EulerTools::h_0, xRes3);

  xRes(10) = x_(10);
}
      
void SegInvDepthFeatureModel::fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
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
  
  mat_range Jframe3(Jframe, range(7,10), range(0,6));
  mat_range Jx3(Jx, range(7,10), range(7,10));
  EulerTools::fromFrameJac(frame_, project(x_, range(7,10)), EulerTools::h_0, 
                           Jframe3,
                           Jx3 );
}

void SegInvDepthFeatureModel::compute3dPoints(jblas::vec const& x, jblas::vec& p, jblas::vec& q)
{
  JFR_PRECOND(x.size() == 11,
	      "SegInvDepthFeatureModel::compute3dPoint: invalid size");
  JFR_PRECOND(p.size() == 3,
	      "SegInvDepthFeatureModel::compute3dPoint: invalid size");
  JFR_PRECOND(q.size() == 3,
              "SegInvDepthFeatureModel::compute3dPoint: invalid size");

  p.assign(project(x, range(0,3)) +  project(x, range(3,6))/x(6));
  q.assign(project(x, range(0,3)) +  project(x, range(7,10))/x(10));
}

void SegInvDepthFeatureModel::compute3dPointsJac(jblas::vec const& x, jblas::vec& p, jblas::vec& q, jblas::mat& Pt_id)
{
  JFR_PRECOND(x.size() == 11,
              "SegInvDepthFeatureModel::compute3dPoints: invalid size");
  JFR_PRECOND(p.size() == 3,
              "SegInvDepthFeatureModel::compute3dPoints: invalid size");
  JFR_PRECOND(q.size() == 3,
              "SegInvDepthFeatureModel::compute3dPoints: invalid size");

  p.assign(project(x, range(0,3)) +  project(x, range(3,6))/x(6));
  q.assign(project(x, range(0,3)) +  project(x, range(7,10))/x(10));
  
  double d1 = 1.0/x(6);
  Pt_id.clear();
  Pt_id(0,0) = 1.0;
  Pt_id(1,1) = 1.0;
  Pt_id(2,2) = 1.0;
  Pt_id(0,3) = d1;
  Pt_id(1,4) = d1;
  Pt_id(2,5) = d1;

  double dd1 = -1.0 / (x(6)*x(6));
  Pt_id(0,6) = dd1 * x(3);
  Pt_id(1,6) = dd1 * x(4);
  Pt_id(2,6) = dd1 * x(5);

  double d2 = 1.0/x(10);
  Pt_id(3,7) = d2;
  Pt_id(4,8) = d2;
  Pt_id(5,9) = d2;

  double dd2 = -1.0 / (x(10)*x(10));
  Pt_id(3,10) = dd2 * x(7);
  Pt_id(4,10) = dd2 * x(8);
  Pt_id(5,10) = dd2 * x(9);
}

void SegInvDepthFeatureModel::compute3dPointsJac(jblas::vec const& x)
{
   double d1 = 1.0/x(6);
  J3dPoints(0,3) = d1;
  J3dPoints(1,4) = d1;
  J3dPoints(2,5) = d1;

  double dd1 = -1.0 / (x(6)*x(6));
  J3dPoints(0,6) = dd1 * x(3);
  J3dPoints(1,6) = dd1 * x(4);
  J3dPoints(2,6) = dd1 * x(5);

  double d2 = 1.0/x(10);
  J3dPoints(3,7) = d2;
  J3dPoints(4,8) = d2;
  J3dPoints(5,9) = d2;

  double dd2 = -1.0 / (x(10)*x(10));
  J3dPoints(3,10) = dd2 * x(7);
  J3dPoints(4,10) = dd2 * x(8);
  J3dPoints(5,10) = dd2 * x(9);
}
 
//TODO this function does not work, needs to be done (maybe two inverse depth points)
void SegInvDepthFeatureModel::computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac )
{
  vec q(3);
  compute3dPoints( x_, mergeX_,q );
  compute3dPointsJac( x_ );
  jac.assign( J3dPoints );
}


 /* class ImageSegInvDepthFeatureObserveModel
 */

ImageSegInvDepthFeatureObserveModel::ImageSegInvDepthFeatureObserveModel(SegInvDepthFeatureModel& featureModel_, camera::CameraPinhole const& camera_) :
  FeatureObserveModel(featureModel_, 2),
  camera(camera_),
  segInvDepthFeatureModel(featureModel_),
  p_pixCov(2,2)
{
  p_pixCov.clear();
  setup();
}

ImageSegInvDepthFeatureObserveModel::~ImageSegInvDepthFeatureObserveModel() 
{
}

void ImageSegInvDepthFeatureObserveModel::setPixCov(vec const& pixCov, double stabilizingFactor)
{
  JFR_PRECOND(pixCov.size()==2,
              "ImageSegmentFeatureObserveModel::setPixCov:");
	
  p_pixCov.clear();
  p_pixCov(0,0) = stabilizingFactor*stabilizingFactor*pixCov(0);
  p_pixCov(1,1) = stabilizingFactor*stabilizingFactor*pixCov(1);
}

vec const& ImageSegInvDepthFeatureObserveModel::computeInnovation(vec const& z_,
    vec const& zPred)
{
  JFR_PRECOND(z_.size() == 2,
              "ImageSegInvDepthFeatureObserveModel::computeInnovation: size of z_ does not match");
  JFR_PRECOND(zPred.size() == 2,
              "ImageSegInvDepthFeatureObserveModel::computeInnovation: size of zPred_ does not match");
    

    // PI rotation when origin is traversed
	
	// new method
  vec2 inn = z_ - zPred;
  double innt = round(inn(1) / M_PI);
  vec2 zi;
  zi(1) = z_(1) - innt * M_PI;  // 
  if ((int)innt % 2 != 0) {
    zi(0) = -1.0*z_(0);  // change rho sign
  }else{
    zi(0) = z_(0);
  }
  z = zi - zPred;
	//JFR_DEBUG("Innovation" << z);
    
    /* 
	// old method
  if (z(1) > M_PI/2) {
  z(1) -= M_PI;
  z(0) = -1.0*z_(0) - zPred(0);
}
  else if (z(1) < -M_PI/2) {
  z(1) += M_PI;
  z(0) = -1.0*z_(0) - zPred(0);
}
    */
  return z;
}

// Build 2-endpoints observation covariances matrix
void ImageSegInvDepthFeatureObserveModel::computeSensorR(Observation const& obs) 
{
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs);
  mat J(2,4);
  extToImageModelJac(segObs.ext1, segObs.ext2, J);
    //JFR_DEBUG("ext to rho theta jacobian: must be 2x4" << J);
	//JFR_DEBUG("sizeObs: must be 2: " << sizeObs());
	
  sym_mat extsCov(4,4);
  extsCov.clear();
    
  range r1(0,2);
  project(extsCov, r1,r1).assign(p_pixCov);
    
  range r2(2,4);
  project(extsCov, r2,r2).assign(p_pixCov);
    
  p_isCorrelated = true;
  R.assign(prod(J, mat(prod(extsCov, trans(J)))));
    //JFR_DEBUG("SensorR : must be 2x2 " << R);
}

void ImageSegInvDepthFeatureObserveModel::initInternalState(BaseFeature& baseFeature, 
    geom::T3DEuler const& robotPose_, 
    Observation const& obs_) const
    {
      JFR_TRACE_BEGIN;
	
	//JFR_DEBUG("obs:" << obs_);
	
      SegmentIDFeature& segFeature = dynamic_cast<SegmentIDFeature&>(baseFeature);
      SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
	
      segFeature.s1 = 0;
      segFeature.s2 = 1;

      JFR_TRACE_END("ImageSegmentFeatureObserveModel::initInternalState()");
    }

    void ImageSegInvDepthFeatureObserveModel::updateInternalState(BaseFeature& baseFeature, geom::T3DEuler const& robotPose_, Observation const& obs_) const
    {
      JFR_TRACE_BEGIN;
      SegmentIDFeature& segFeature = dynamic_cast<SegmentIDFeature&>(baseFeature);
      SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
	
      JFR_TRACE_END("ImageSegmentFeatureObserveModel::updateInternalState()");
    }

vec const& ImageSegInvDepthFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{
  jblas::vec p(3);
  jblas::vec q(3);
  segInvDepthFeatureModel.compute3dPoints(f_, p,q);
  jblas::vec2 pi;
  jblas::vec2 qi;
  camera.project(p,pi);
  camera.project(q,qi);
  jblas::vec l(3);
  l = lineTools::extToHomogeneous(pi,qi);
  z = lineTools::homogeneousToRhoTheta(l);
  return z;
}

void ImageSegInvDepthFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{  
  JFR_PRECOND(f_.size() == 11,
              "ImageSegInvDepthFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");
  //Jacobian of computing 3d points
  jblas::vec p(3);
  jblas::vec q(3);
  jblas::mat Pts_id(6,11);
  segInvDepthFeatureModel.compute3dPointsJac(f_, p, q, Pts_id); 
  // Jacobian of projecting points into the image
  jblas::mat PtsI_pts(4,6);
  PtsI_pts.clear();
  jblas::vec2 pi;
  jblas::vec2 qi;
  camera.project(p,pi); 
  camera.project(q,qi);
  jblas::mat Pi_p(2,3);
  jblas::mat Qi_q(2,3);
  camera.projectJac(p, Pi_p);
  camera.projectJac(q, Qi_q);
  subrange(PtsI_pts,0,2,0,3).assign(Pi_p);
  subrange(PtsI_pts,2,4,3,6).assign(Qi_q);
    // Jacobian of extremities to Homogeneous
  jblas::vec3 hm;
  jblas::mat Hm_ptsI(3,4);
  Hm_ptsI.clear();
  jblas::mat Hm_pi(3,2);
  jblas::mat Hm_qi(3,2);
  lineTools::extToHomogeneous(pi, qi, hm, Hm_pi, Hm_qi); //
  subrange(Hm_ptsI,0,3,0,2).assign(Hm_pi);
  subrange(Hm_ptsI,0,3,2,4).assign(Hm_qi);
    // Jacobian of Homogeneous to RhoTheta
  jblas::vec2 rt;
  jblas::mat Rt_hm(2,3);
  lineTools::homogeneousToRhoTheta(hm, rt, Rt_hm);
    // Jacobian composition
  JobsSensor.assign(prod(Rt_hm,mat(prod(Hm_ptsI, mat(prod(PtsI_pts,Pts_id))))));
}

vec ImageSegInvDepthFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
   vec x(11);
   x.clear();
   vec_range u(x, range(3,6));
   camera.imageToCameraFrameDirection(segObs.ext1, u);
   x(6) = m_id0;
   vec_range v(x, range(7,10));
   camera.imageToCameraFrameDirection(segObs.ext2, v);
   x(10) = m_id0;
   return x;
}

void ImageSegInvDepthFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
  JinvObs.clear();
  jblas::mat_range Jdir1(JinvObs, range(3,6), range(0,2));
  camera.imageToCameraFrameDirectionJac(segObs.ext1, Jdir1);
  jblas::mat_range Jdir2(JinvObs, range(7,10), range(0,2));
  camera.imageToCameraFrameDirectionJac(segObs.ext2, Jdir2);
}

// Endpoints to Scaled Homogeneous transform
vec2 ImageSegInvDepthFeatureObserveModel::extToImageModel(vec const& ext1,vec const& ext2)
{
    
  double u1 = ext1(0);
  double v1 = ext1(1);
  double u2 = ext2(0);
  double v2 = ext2(1);
    
  vec2 line;
    
  /* Begin Maple */
    
  double t1 = u2 * u2;
  double t4 = u1 * u1;
  double t5 = v2 * v2;
  double t8 = v1 * v1;
  double t10 = sqrt(t1 - 0.2e1 * u2 * u1 + t4 + t5 - 0.2e1 * v2 * v1 + t8);
  double t11 = 0.1e1 / t10;
  double t13 = u2 - u1;
  double t16 = v2 - v1;
  double t19 = (-u1 * t11 * t13 - v1 * t11 * t16) * t11;
  double t21 = u1 + t19 * t13;
  double t22 = t21 * t21;
  double t24 = v1 + t19 * t16;
  double t25 = t24 * t24;
  double t27 = sqrt(t22 + t25);
  double t28 = atan2(t24, t21);
  line(0) = t27;
  line(1) = t28;
    
  /* End Maple */
    
  return line;
}



// endpoints to rho theta transform -- Jacobians
void ImageSegInvDepthFeatureObserveModel::extToImageModelJac(vec const& ext1,vec const& ext2, mat& J)
{
  JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
              "ImageSegInvDepthFeatureObserveModel::extToImageModelJac");
  JFR_PRECOND(J.size1() == 2 && J.size2() == 4,
              "ImageSegInvDepthFeatureObserveModel::extToImageModelJac");
    
  double u1 = ext1(0);
  double v1 = ext1(1);
  double u2 = ext2(0);
  double v2 = ext2(1);
    
  double t1 = v2 * v1;
  double t2 = u2 * u2;
  double t3 = t2 * u2;
  double t5 = v1 * v1;
  double t7 = u1 * t2;
  double t8 = v2 * v2;
  double t10 = t7 * t5;
  double t11 = t8 * v2;
  double t14 = t8 * t5;
  double t15 = t14 * u2;
  double t17 = t5 * v1;
  double t18 = v2 * t17;
  double t19 = t18 * u2;
  double t21 = u1 * u1;
  double t22 = t8 * u2 * t21;
  double t23 = t21 * u2;
  double t25 = t14 * u1;
  double t26 = t8 * t8;
  double t29 = t11 * u1 * v1;
  double t31 = -t1 * t3 + t3 * t5 + t7 * t8 - t10 - t11 * v1 * u2 + 0.2e1 * t15 - t19 - t22 + t23 * t1 + t25 + u1 * t26 - 0.2e1 * t29;
  double t32 = u2 * u1;
  double t35 = t2 - 0.2e1 * t32 + t21 + t8 - 0.2e1 * t1 + t5;
  double t36 = t35 * t35;
  double t37 = 0.1e1 / t36;
  double t39 = u1 * v2;
  double t42 = pow(-t39 + u2 * v1, 0.2e1);
  double t43 = 0.1e1 / t35;
  double t45 = sqrt(t42 * t43);
  double t46 = 0.1e1 / t45;
  double t48 = t2 * t2;
  double t50 = t3 * u1;
  double t52 = t50 * v1;
  double t54 = t2 * t21;
  double t55 = t54 * v1;
  double t56 = v2 * t5;
  double t57 = t56 * t2;
  double t58 = t8 * v1;
  double t60 = t54 * v2;
  double t65 = t21 * u1;
  double t66 = u2 * t65;
  double t67 = t66 * v2;
  double t70 = t21 * v1 * t8;
  double t71 = v1 * t48 - t50 * v2 - 0.2e1 * t52 + t55 - t57 + t58 * t2 + 0.2e1 * t60 - t11 * u2 * u1 + t32 * t56 - t67 + t11 * t21 - t70;
  double t78 = t5 * t5;
  double t85 = t10 - t39 * v1 * t2 + t22 - t23 * t5 - t15 + 0.2e1 * t19 - u2 * t78 - t8 * t65 + t18 * u1 + t65 * v2 * v1 + t29 - 0.2e1 * t25;
  double t96 = t21 * t21;
  double t98 = t52 + t57 - t60 - 0.2e1 * t55 - t2 * t17 + t32 * t17 + t66 * v1 + 0.2e1 * t67 - t32 * t58 - t21 * v2 * t5 - t96 * v2 + t70;
  double t102 = (-v2 + v1) * t43;
  double t104 = (u2 - u1) * t43;
  J(0,0) = t31 * t37 * t46;
  J(0,1) = t71 * t37 * t46;
  J(0,2) = -t85 * t37 * t46;
  J(0,3) = -t98 * t37 * t46;
  J(1,0) = -t102;
  J(1,1) = -t104;
  J(1,2) = t102;
  J(1,3) = t104;
    
  /* End Maple */
//    JFR_DEBUG("extToLineJac: " << J);
    
}

void ImageSegInvDepthFeatureObserveModel::predictExtObs(vec_range const& pose, sym_mat_range const& poseCov,vec3 const& ext, sym_mat const& extCov, vec& zPredExt, sym_mat& zPredExtCov)
{
  JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
              "ImageInvDepthFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND(ext.size() == 3 && extCov.size1() == 3,
              "ImageInvDepthFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND(zPredExt.size() == 2 && zPredExtCov.size1() == 2,
              "ImageInvDepthFeatureObserveModel::predictExtremitiesObservation");
	
  JFR_TRACE_BEGIN;
	
  vec extInSensor(3);
  sym_mat extInSensorCov(3,3);
  mat Jframe(3,6);
  mat Jpoint(3,3);
  mat Jproj(2,3);
	
  EulerTools::toFrame(pose, ext, EulerTools::h_1, extInSensor);
  EulerTools::toFrameJac(pose, ext, EulerTools::h_1, Jframe, Jpoint);
	
  extInSensorCov.assign( prod(Jframe, mat(prod(poseCov, trans(Jframe)))) +
      prod(Jpoint, mat(prod(extCov, trans(Jpoint)))) );
	
  if (!robotToSensor().isIdentity()) {
        //JFR_DEBUG("ROBOT2SENSOR: " << robotToSensor());
    vec extInRobot(extInSensor);
    EulerTools::toFrame(robotToSensor().getX(), extInRobot, EulerTools::h_1, extInSensor);
    EulerTools::toFrameJac(robotToSensor().getX(), extInRobot, EulerTools::h_1, Jframe, Jpoint);
    extInSensorCov = prod(Jpoint, mat(prod(extInSensorCov, trans(Jpoint))));
  }
	
  camera.project(extInSensor, zPredExt);
  camera.projectJac(extInSensor, Jproj);
  zPredExtCov.assign( prod(Jproj, mat(prod(extInSensorCov, trans(Jproj)))) +
      getR() );
	
	// FIXME - use of slam::CameraPinhole
	//   vec zPredExtTmp(zPredExt);
	//   sym_mat zPredExtCovTmp(zPredExtCov);
	
	//   zPredExt(0) = zPredExtTmp(1);
	//   zPredExt(1) = zPredExtTmp(0);
	
	//   zPredExtCov(0,0) = zPredExtCovTmp(1,1);
	//   zPredExtCov(1,1) = zPredExtCovTmp(0,0);
	//   zPredExtCov(0,1) = zPredExtCovTmp(0,1);
	
  JFR_TRACE_END("ImageInvDepthFeatureObserveModel::predictExtremitiesObservation");
}

void ImageSegInvDepthFeatureObserveModel::predictExtObsJac(vec_range const& pose, vec3 const Ext, vec& eInImage, mat& JePose, mat& JeExt)
{
//  JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
//              "ImageSegmentFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND( Ext.size() == 3, "ImageInvDepthFeatureObserveModel::predictExtremitiesObservationJacobians");
  
  JFR_TRACE_BEGIN;
	 
  vec extInRobot(3);  
  mat JRPose(3,6);
  mat JRExt(3,3);

  vec extInSensor(3);
  mat JECC(3,6);
  mat JCR(3,3);
  
  mat JeC(2,3);
	
//geom::T3D::pointToFrameJac(T3DEuler(pose),Ext,extInRobot,JRPose,JRExt);
  EulerTools::toFrame(pose, Ext, EulerTools::h_1, extInRobot);
  EulerTools::toFrameJac(pose, Ext, EulerTools::h_1, JRPose, JRExt);
 // JFR_DEBUG("Jacobians JRExt:" << JRExt);
//  if (!robotToSensor().isIdentity()) {
//    geom::T3D::pointToFrameJac(T3DEuler(robotToSensor().getX()),extInRobot,extInSensor,JECC,JCR);
  EulerTools::toFrame(robotToSensor().getX(), extInRobot, EulerTools::h_1, extInSensor);
  EulerTools::toFrameJac(robotToSensor().getX(), extInRobot, EulerTools::h_1, JECC, JCR);
//    JFR_DEBUG("Jacobians JCR:" << JCR);
//  }
	
  camera.project(extInSensor, eInImage);
  camera.projectJac(extInSensor, JeC);
//  JFR_DEBUG("Jacobians JeC:" << JeC);
  
  JePose.assign(prod(JeC,mat(prod(JCR,JRPose))) );
  JeExt.assign(prod(JeC,mat(prod(JCR,JRExt))));
//  JFR_DEBUG("Jacobians JeExt:" << JeExt);
  
  JFR_TRACE_END("ImageInvDepthFeatureObserveModel::predictExtremitiesObservationJacobians");
}

void ImageSegInvDepthFeatureObserveModel::postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature_) {
  JFR_PRECOND(P.size1() == 11,
	      "ImageSegInvDepthFeatureObserveModel::postInitCovariance: invalid size");
  P(6,6) = m_sigmaId0 * m_sigmaId0;
  P(10,10) = m_sigmaId0 * m_sigmaId0;
}


void ImageSegInvDepthFeatureObserveModel::setup(double id0, double sigmaId0) {
  JFR_PRECOND(id0 > 0 && sigmaId0 > 0,
	      "ImageSegInvDepthFeatureObserveModel::setup: invalid values");
  m_id0 = id0;
  m_sigmaId0 = sigmaId0;

}
