/* $Id$ */

#include "slam/eulerTools.hpp"
#include "slam/lineTools.hpp"
#include "slam/segmentInvDepthInovFeature.hpp"
#include "slam/segmentInvDepthFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::slam;


 /* class ImageSegInvDepthInovFeatureObserveModel
 */

ImageSegInvDepthInovFeatureObserveModel::ImageSegInvDepthInovFeatureObserveModel(SegInvDepthFeatureModel& featureModel_, camera::CameraPinhole const& camera_) :
  FeatureObserveModel(featureModel_, 4, 2, 2, 3),
  camera(camera_),
  segInvDepthFeatureModel(featureModel_),
  p_pixCov(2,2),
  Jobs1Tmp(3,6),
  Jobs2Tmp(3,11)
{
  p_pixCov.clear();
  setup();
}

ImageSegInvDepthInovFeatureObserveModel::~ImageSegInvDepthInovFeatureObserveModel() 
{
}

void ImageSegInvDepthInovFeatureObserveModel::setPixCov(vec const& pixCov, double stabilizingFactor)
{ // TODO pixCov should be a single double not a vector
  JFR_PRECOND(pixCov.size()==4,
              "ImageSegmentFeatureObserveModel::setPixCov:");
	
  p_pixCov.clear();
  p_pixCov(0,0) = stabilizingFactor*stabilizingFactor*pixCov(0);
  p_pixCov(1,1) = stabilizingFactor*stabilizingFactor*pixCov(1);
}

vec const& ImageSegInvDepthInovFeatureObserveModel::computeInnovation(vec const& y_,
    vec const& zPred)
{
  JFR_PRECOND(y_.size() == 4,
              "ImageSegInvDepthInovFeatureObserveModel::computeInnovation: size of z_ does not match");
  JFR_PRECOND(zPred.size() == 3,
              "ImageSegInvDepthInovFeatureObserveModel::computeInnovation: size of zPred_ does not match");
  
  jblas::vec2 ext1 = subrange( y_, 0,2);
  jblas::vec2 ext2 = subrange( y_, 2,4);
  
  jblas::mat DD_hm(2,3);
  
  // We don't need DD_ext1 and DD_ext2, since the uncertainty around
  // a point is a circle
  jblas::vec2 z_;
  lineTools::homogenousToExtDistances(zPred, ext1, ext2, z_, DD_hm );
  z.resize(sizeInnovation());
  z.assign(z_);
  
  Jobs1.assign( - prod( DD_hm, Jobs1Tmp ) );
  Jobs2.assign( - prod( DD_hm, Jobs2Tmp ) );

  return z;
  
}

// Build 2-endpoints observation covariances matrix
void ImageSegInvDepthInovFeatureObserveModel::computeSensorR(Observation const& obs) 
{
  R.clear();
  R(0,0) = p_pixCov(0,0);
  R(1,1) = p_pixCov(0,0);
}

void ImageSegInvDepthInovFeatureObserveModel::initInternalState(BaseFeature& baseFeature, 
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

    void ImageSegInvDepthInovFeatureObserveModel::updateInternalState(BaseFeature& baseFeature, geom::T3DEuler const& robotPose_, Observation const& obs_) const
    {
      JFR_TRACE_BEGIN;
      SegmentIDFeature& segFeature = dynamic_cast<SegmentIDFeature&>(baseFeature);
      SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
	
      JFR_TRACE_END("ImageSegmentFeatureObserveModel::updateInternalState()");
    }

void ImageSegInvDepthInovFeatureObserveModel::predictObservationJac(const vec_range& robotPose_,
                const vec_range& feature_) 
{
  computePredictObservationJac( robotPose_, feature_, Jobs1Tmp, Jobs2Tmp );
}


vec const& ImageSegInvDepthInovFeatureObserveModel::predictObservationInSensorFrame(const vec& f_)
{
  jblas::vec p(3);
  jblas::vec q(3);
  segInvDepthFeatureModel.compute3dPoints(f_, p,q);
  jblas::vec2 pi;
  jblas::vec2 qi;
  camera.project(p,pi);
  camera.project(q,qi);
  jblas::vec l(3);
  z = lineTools::extToHomogeneous(pi,qi);
  return z;
#if 0
  l = lineTools::extToHomogeneous(pi,qi);
  z = lineTools::homogeneousToRhoTheta(l);
  return z;
#endif
}

void ImageSegInvDepthInovFeatureObserveModel::predictObservationInSensorFrameJac(const vec& f_) 
{  
  JFR_PRECOND(f_.size() == 11,
              "ImageSegInvDepthInovFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");
  //Jacobian of computing 3d points
  jblas::vec p(3);
  jblas::vec q(3);
  jblas::mat Pts_id(6,11);
  segInvDepthFeatureModel.compute3dPointsJac(f_, p, q, Pts_id); 
  // Projects ID points on camera
  jblas::vec2 pi;
  jblas::vec2 qi;
  camera.project(p,pi); 
  camera.project(q,qi);
  jblas::mat Pi_p(2,3);
  jblas::mat Qi_q(2,3);
  camera.projectJac(p, Pi_p);
  camera.projectJac(q, Qi_q);
  // Jacobian of projecting points into the image
  jblas::mat PtsI_pts(4,6);
  PtsI_pts.clear();
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
  // Jacobian composition
  JobsSensor.resize( 3, 11);
  JobsSensor.assign(prod(Hm_ptsI, mat(prod(PtsI_pts,Pts_id))));
}

vec ImageSegInvDepthInovFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
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

void ImageSegInvDepthInovFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
  JinvObs.clear();
  jblas::mat_range Jdir1(JinvObs, range(3,6), range(0,2));
  camera.imageToCameraFrameDirectionJac(segObs.ext1, Jdir1);
  jblas::mat_range Jdir2(JinvObs, range(7,10), range(0,2));
  camera.imageToCameraFrameDirectionJac(segObs.ext2, Jdir2);
}

// Endpoints to Scaled Homogeneous transform
vec2 ImageSegInvDepthInovFeatureObserveModel::extToImageModel(vec const& ext1,vec const& ext2)
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
void ImageSegInvDepthInovFeatureObserveModel::extToImageModelJac(vec const& ext1,vec const& ext2, mat& J)
{
  JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
              "ImageSegInvDepthInovFeatureObserveModel::extToImageModelJac");
  JFR_PRECOND(J.size1() == 2 && J.size2() == 4,
              "ImageSegInvDepthInovFeatureObserveModel::extToImageModelJac");
    
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

void ImageSegInvDepthInovFeatureObserveModel::predictExtObs(vec_range const& pose, sym_mat_range const& poseCov,vec3 const& ext, sym_mat const& extCov, vec& zPredExt, sym_mat& zPredExtCov)
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

void ImageSegInvDepthInovFeatureObserveModel::predictExtObsJac(vec_range const& pose, vec3 const Ext, vec& eInImage, mat& JePose, mat& JeExt)
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

void ImageSegInvDepthInovFeatureObserveModel::postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature_) {
  JFR_PRECOND(P.size1() == 11,
	      "ImageSegInvDepthInovFeatureObserveModel::postInitCovariance: invalid size");
  P(6,6) = m_sigmaId0 * m_sigmaId0;
  P(10,10) = m_sigmaId0 * m_sigmaId0;
}


void ImageSegInvDepthInovFeatureObserveModel::setup(double id0, double sigmaId0) {
  JFR_PRECOND(id0 > 0 && sigmaId0 > 0,
	      "ImageSegInvDepthInovFeatureObserveModel::setup: invalid values");
  m_id0 = id0;
  m_sigmaId0 = sigmaId0;

}
