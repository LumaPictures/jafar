/* $Id$ */

#include <cmath>

#include "kernel/jafarException.hpp"

#ifdef THALES_TAROT
	#undef BOOST_NUMERIC_BINDINGS_LAPACK_2
#endif
#include "boost/numeric/bindings/traits/ublas_matrix.hpp"
#include "boost/numeric/bindings/traits/ublas_vector.hpp"
#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "geom/t3dEuler.hpp"

#include "slam/segmentFeature.hpp"
#include "slam/eulerTools.hpp"

#include "slam/lineTools.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar::jmath;
using namespace jafar::geom;
using namespace jafar::slam;
namespace lapack = boost::numeric::bindings::lapack;
//namespace lineTools = jafar::slam::lineTools

//#define ENABLE_PLUCKER_CONSTRAINT
//#define ENABLE_UNIT_CONSTRAINT
//#define ENABLE_NORMALISATION

/*
 * class SegmentObservation
 */

std::ostream& jafar::slam::operator <<(std::ostream& s, const jafar::slam::SegmentObservation& o_)
{
  s << (Observation) o_
    << ", ext1: " << o_.ext1 << ", ext2: " << o_.ext2;
  return s;
}

/*
 * class StereoSegmentObservation
 */

std::ostream& jafar::slam::operator <<(std::ostream& s, const jafar::slam::StereoSegmentObservation& o_)
{
  s << (Observation) o_
    << ", ext1: " << o_.ext1 << ", ext2: " << o_.ext2;
  return s;
}


/*
 * class SegmentFeature
 */

double SegmentFeature::p_constraintAlpha = 0.01;
int SegmentFeature::p_constraintMaxNbTimesApplied = -1;
bool SegmentFeature::ENABLE_PLUCKER_CONSTRAINT = false;

//FIXME dynamic sizeObs
SegmentFeature::SegmentFeature(unsigned int id, FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs_) :
  BaseFeature(id, model, sizeObs, typeObs_),
  ENLARGE(false),
  p_n(false),
  p_u(false)
{
//#ifdef ENABLE_PLUCKER_CONSTRAINT
if (ENABLE_PLUCKER_CONSTRAINT)
  constraints.push_back(new SegmentPluckerConstraint(p_constraintAlpha, p_constraintMaxNbTimesApplied));
//#endif

#ifdef ENABLE_UNIT_CONSTRAINT
  constraints.push_back(new SegmentUConstraint(p_constraintAlpha, p_constraintMaxNbTimesApplied));
#endif

featureConstraintModel= new SegmentFeatureContraintModel;
}

SegmentFeature::~SegmentFeature() {
  if (p_n) delete p_n;
  if (p_u) delete p_u;
}

void SegmentFeature::setState(vec& x_, sym_mat& P_)
{
  BaseFeature::setState(x_, P_);
  if (p_n) delete p_n;
  if (p_u) delete p_u;
  p_n = new vec_range(x_, range(filterIndex(), filterIndex()+3 ));
  p_u = new vec_range(x_, range(filterIndex()+3, filterIndex()+6 ));
}


vec3 SegmentFeature::getExt1() const 
{
  vec3 ext;
  computeExt(s1, ext);  
  //JFR_DEBUG("ext1: " << ext);
  return ext;
}

sym_mat SegmentFeature::getExt1Cov() const
{
  sym_mat extCov(3,3);
  computeExtCov(s1,extCov); 
  //JFR_DEBUG("ext1Cov: " << extCov);
  return extCov;
}

mat SegmentFeature::getExt1Jac() const
{
  mat J1(3,6);
  computeExtJac(s1, J1);
  //JFR_DEBUG("ext1Cov: " << extCov);
  return J1;
}

vec3 SegmentFeature::getExt2() const 
{
  vec3 ext;
  computeExt(s2, ext);
  //JFR_DEBUG("ext2: " << ext);
  return ext;  
}

sym_mat SegmentFeature::getExt2Cov() const
{
  sym_mat extCov(3,3);
  computeExtCov(s2,extCov);  
  //JFR_DEBUG("ext2Cov: " << extCov);
  return extCov;
}

mat SegmentFeature::getExt2Jac() const
{
  mat J2(3,6);
  computeExtJac(s2, J2);
  //JFR_DEBUG("ext1Cov: " << extCov);
  return J2;
}


void SegmentFeature::computeExt(double s_, vec3& ext) const
{
  double nU = norm_2(u()); // the norm of the director vector
  ublasExtra::crossProd(u()/nU, n()/nU, ext);
  ext.plus_assign(s_* u()/nU); // the 3D endpoint
  //JFR_DEBUG("abscissa: "<< s_);
}

void SegmentFeature::computeExtCov(double s_, sym_mat& extCov) const
{
  mat E_l(3,6); // the Jacobian wrt the plucker line
  computeExtJac(s_, E_l);
  extCov.assign(prod(E_l, mat(prod(getP(), trans(E_l))) ));
}


void SegmentFeature::computeExtJac(double s_, mat& E_l) const
{
  double n1  = n()(0); // normal vector
  double n2  = n()(1);
  double n3  = n()(2);
  double v1  = u()(0); // director vector
  double v2  = u()(1);
  double v3  = u()(2);
  double vn2 = v1*v1+v2*v2+v3*v3; // norm squared
  double vn  = sqrt(vn2); // norm
  double vn4 = vn2*vn2; // norm to the 4th power
  E_l(0,0) = 0.0; // the Jacobian wrt the plucker line
  E_l(0,1) = -v3/vn2;
  E_l(0,2) =  v2/vn2;
  E_l(0,3) =  -2.0*(v2*n3-v3*n2)/vn4*v1+s_/vn;
  E_l(0,4) =  (vn2*n3-2*(v2*n3-v3*n2)*v2)/vn4;
  E_l(0,5) =  (-vn2*n2-2*(v2*n3-v3*n2)*v3)/vn4;
  E_l(1,0) =  v3/vn2;
  E_l(1,1) =  0.0;
  E_l(1,2) =  -v1/vn2;
  E_l(1,3) =  (-vn2*n3-2*(v3*n1-v1*n3)*v1)/vn4;
  E_l(1,4) =  -2.0*(v3*n1-v1*n3)/vn4*v2+s_/vn;
  E_l(1,5) =  (vn2*n1-2*(v3*n1-v1*n3)*v3)/vn4;
  E_l(2,0) = -v2/vn2;
  E_l(2,1) =  v1/vn2;
  E_l(2,2) =  0.0;
  E_l(2,3) =  (vn2*n2-2*(v1*n2-v2*n1)*v1)/vn4;
  E_l(2,4) =  (-vn2*n1-2*(v1*n2-v2*n1)*v2)/vn4;
  E_l(2,5) =  -2.0*(v1*n2-v2*n1)/vn4*v3+s_/vn;
  
}

/*
void SegmentFeature::computeExtJac(double s_, mat& J) const
{
  double nU = norm_2(u());
  JFR_DEBUG("norm_2(u): " << nU);
  double nU2 = nU*nU;
// maple generated code 
  if (1.0-nU <= 0.0001) {
	J(0,0) = 0;
	J(0,1) = -u()(2);
	J(0,2) = u()(1);
	J(0,3) = s_;
	J(0,4) = n()(2);
	J(0,5) = -n()(1);
	J(1,0) = u()(2);
	J(1,1) = 0;
	J(1,2) = -u()(0);
	J(1,3) = -n()(2);
	J(1,4) = s_;
	J(1,5) = n()(0);
	J(2,0) = -u()(1);
	J(2,1) = u()(0);
	J(2,2) = 0;
	J(2,3) = n()(1);
	J(2,4) = -n()(0);
	J(2,5) = s_;
	// end of maple generated code 
  }
  else {
	  
	  J(0,0) = 0;
	  J(0,1) = -u()(2)/nU2;
	  J(0,2) = u()(1)/nU2;
	  J(0,3) = -2*u()(1)/pow(nU2,2)*n()(2)*u()(0)+2*u()(2)/pow(nU2,2)*n()(1)*u()(0)+s_/nU-s_*pow(u()(0),2)/pow(nU,3);
	  J(0,4) = 1/nU2*n()(2)-2*pow(u()(1),2)/pow(nU2,2)*n()(2)+2*u()(2)/pow(nU2,2)*n()(1)*u()(1)-s_*u()(0)/pow(nU,3)*u()(1);
	  J(0,5) = -2*u()(1)/pow(nU2,2)*n()(2)*u()(2)-1/nU2*n()(1)+2*pow(u()(2),2)/pow(nU2,2)*n()(1)-s_*u()(0)/pow(nU,3)*u()(2);
	  J(1,0) = u()(2)/nU2;
	  J(1,1) = 0;
	  J(1,2) = -u()(0)/nU2;
	  J(1,3) = -2*u()(2)/pow(nU2,2)*n()(0)*u()(0)-1/nU2*n()(2)+2*pow(u()(0),2)/pow(nU2,2)*n()(2)-s_*u()(0)/pow(nU,3)*u()(1);
	  J(1,4) = -2*u()(2)/pow(nU2,2)*n()(0)*u()(1)+2*u()(1)/pow(nU2,2)*n()(2)*u()(0)+s_/nU-s_*pow(u()(1),2)/pow(nU,3);
	  J(1,5) = 1/nU2*n()(0)-2*pow(u()(2),2)/pow(nU2,2)*n()(0)+2*u()(0)/pow(nU2,2)*n()(2)*u()(2)-s_*u()(1)/pow(nU,3)*u()(2);
	  J(2,0) = -u()(1)/nU2;
	  J(2,1) = u()(0)/nU2;
	  J(2,2) = 0;
	  J(2,3) = 1/nU2*n()(1)-2*pow(u()(0),2)/pow(nU2,2)*n()(1)+2*u()(1)/pow(nU2,2)*n()(0)*u()(0)-s_*u()(0)/pow(nU,3)*u()(2);
	  J(2,4) = -2*u()(0)/pow(nU2,2)*n()(1)*u()(1)-1/nU2*n()(0)+2*pow(u()(1),2)/pow(nU2,2)*n()(0)-s_*u()(1)/pow(nU,3)*u()(2);
	  J(2,5) = -2*u()(2)/pow(nU2,2)*n()(1)*u()(0)+2*u()(2)/pow(nU2,2)*n()(0)*u()(1)+s_/nU-s_*pow(u()(2),2)/pow(nU,3);
*/

void SegmentFeature::writeLogHeader(jafar::kernel::DataLogger& log) const
{
  BaseFeature::writeLogHeader(log);

  log.writeComment("slam: SegmentFeature");
  log.writeLegend("u");
  log.writeLegend("n");
  log.writeLegend("||u||");
  log.writeLegend("n.u");
  log.writeLegend("angle(n,u) (rad)");
}
void SegmentFeature::writeLogData(jafar::kernel::DataLogger& log) const
{
  BaseFeature::writeLogData(log);
  log.writeData(n());
  log.writeData(u());
  double normN = ublas::norm_2(n());
  double normU = ublas::norm_2(u());
  log.writeData(normU);
  double ip = ublas::inner_prod(n(),u());
  log.writeData(ip);
  log.writeData(acos(ip/(normN*normU)));

}

std::ostream& jafar::slam::operator <<(std::ostream& s, jafar::slam::SegmentFeature const& f)
{
  s << static_cast<BaseFeature const&>(f);
  s << "ext1: " << f.getExt1()
    << "ext2: " << f.getExt2();
  return s;
}

// Transformation from Homogeneous to Euclidean Plucker
// vec SegmentFeature::homogeneousToEuclideanPlucker(vec const& hPlucker)
// {
// 	vec ePlucker(6);
// 	ePlucker=hPlucker/norm_2(subrange(hPlucker,3,6));
// }

// void SegmentFeature::applyConstraints() 
// {
//   //JFR_DEBUG("(0) norm_2(u): " << norm_2(u()));
//   //JFR_DEBUG("(0) n.u = " << inner_prod(n(),u()));

//   // pseudo local observation to enforce n.u = 0
//   vec y(1);
//   y(0) = 0.0 - inner_prod(n(),u());

//   mat H(1,6);
//   H.clear();
//   sym_mat S(1,1);
//   sym_mat Sinv(1,1);
//   mat K(6,1);
    

//   ublasExtra::inner_prodJac<3>(n(), u(), H);
//   S.assign( prod(H, mat(prod(getP(), trans(H)))) );
//   ublasExtra::inv(S,Sinv);
    
//   K.assign( prod(getP(), mat(prod(trans(H), Sinv))) );
//   getX().plus_assign( prod(K, y)  );

//   ublasExtra::normalize(u());

//   //JFR_DEBUG("(1) norm_2(u): " << norm_2(u()));
//   //JFR_DEBUG("(1) n.u = " << inner_prod(n(),u()));

// }

//   const double EPSILON = 1e-4;

//   mat H(2,6);
//   H.clear();
//   mat_range Jnorm_2(H, range(0,1), range(3,6));
//   mat_range Jinner_prod(H, range(1,2), range(0,6));
//   sym_mat S(2,2);
//   sym_mat Sinv(2,2);
//   mat K(6,2);
//   while (fabs(y(0)) > EPSILON || fabs(y(1)) > EPSILON) {
//   //  while (i < 1) {
//     ++i;
    
//     ublasExtra::norm_2Jac<3>(*u, Jnorm_2);
//     ublasExtra::inner_prodJac<3>(*n,*u, Jinner_prod);
//     S.assign( prod(H, mat(prod(getP(), trans(H)))) );
//     ublasExtra::inv(S,Sinv);
    
//     K.assign( prod(getP(), mat(prod(trans(H), Sinv))) );
//     getX().plus_assign( prod(K, y)  );
//     //    getP().minus_assign(prod(K, mat(prod(S, trans(K)))));

//     y(0) = 1.0 - norm_2(*u);
//     y(1) = 0.0 - inner_prod(*n,*u);
//     JFR_VVDEBUG("(" << i << ") ||u|| = " << 1.0 - y(0)); 
//     JFR_VVDEBUG("(" << i << ") n.u   = " << -y(1)); 
//   }

//   //  (*u) /= norm_2(*u);

//   double s = inner_prod(*n,*u);
//   if (s > 0.01)
//     JFR_WARNING("SegmentFeature::applyConstraints: n.u = " << s);

//  //JFR_DEBUG("(1) n.u = " << inner_prod(*n,*u)); 


/*
 * class SegmentFeatureConstraintModel
 */
SegmentFeatureContraintModel::SegmentFeatureContraintModel() : 
  JacobianBlockPredictModel(6)
{
  Q.clear();
}

void SegmentFeatureContraintModel::predict(vec_range& L)
{
  JFR_PRECOND(L.size() == sizeState(),
          "SegmentFeatureContraintModel::predict: invalid state size");

  fixFeatureJac(L,F);
}


void SegmentFeatureContraintModel::fixFeature(vec_range& L)
{
  JFR_PRECOND(L.size()==6,
              "SegmentFeatureModel::fixFeature:");
 
  mat_column_major C(3,2);
  vec s(2);
  vec tmp2(2);
  mat_column_major U(3, 2);
  mat_column_major tmp22(2, 2);
  mat_column_major VT(2, 2);
  mat_column_major T(2,2);
  mat_column_major hvT(2, 2);
  mat22 Z;
  mat22 hV;

  //JFR_DEBUG("plucker line before: " << L); 
 
  ublas::column(C, 0).assign(project(L, range(0, 3)));
  ublas::column(C, 1).assign(project(L, range(3, 6)));

// JFR_DEBUG("C: " << C); 

  double normN = ublas::norm_2(subrange(L,0,3));
  double normU = ublas::norm_2(subrange(L,3,6));
  double ip = ublas::inner_prod(project(L,range(0,3)),project(L,range(3,6)));
  //JFR_DEBUG("plucker pre-angle: " << (acos(ip/(normN*normU))));
   
  int ierr1 = lapack::gesdd(C,s,U,VT);
  if (ierr1!=0) {
    throw(jmath::LapackException(ierr1,
          "SegmentFeatureModel::fixFeature: error in lapack::gesdd() routine.",
          __FILE__,
          __LINE__));
  }
 
  project(Z,range(0,1),range(0,2)).assign(s(0)*project(VT,range(0,1),range(0,2)));
  project(Z,range(1,2),range(0,2)).assign(s(1)*project(VT,range(1,2),range(0,2)));

  T(0,0)=Z(1,0);
  T(0,1)=Z(1,1);
  T(1,0)=Z(0,1);
  T(1,1)=-Z(0,0);

  int ierr2 = lapack::gesdd(T,tmp2,tmp22,hvT);
  if (ierr2!=0) {
    throw(jmath::LapackException(ierr2,
          "SegmentFeatureModel::fixFeature: error in lapack::gesdd() routine.",
          __FILE__,
          __LINE__));
  }

  hV(0,0)=hvT(1,0);
  hV(0,1)=-hvT(1,1);
  hV(1,0)=hvT(1,1);
  hV(1,1)=hvT(1,0);

mat mtmp(prod(trans(hV),Z));
  tmp22.assign(ublas::diagonal_adaptor<mat>(mtmp));

  mat B(3,2);
//  B.assign(prod(project(U,range(0,3),range(0,2)),mat(prod(hV,ublas::diagonal_adaptor<mat_column_major>(tmp22)))));
  B.assign(prod(U,mat(prod(hV,tmp22))));

//JFR_DEBUG("B: " << B);

  project(L,range(0,3)).assign(ublas::column(B, 0));
  project(L,range(3,6)).assign(ublas::column(B, 1));

  //JFR_DEBUG("plucker line after: " << L); 

  normN = ublas::norm_2(subrange(L,0,3));
  normU = ublas::norm_2(subrange(L,3,6));
  ip = ublas::inner_prod(project(L,range(0,3)),project(L,range(3,6)));
  //JFR_DEBUG("plucker post-angle: " << (acos(ip/(normN*normU))));
 
}

void SegmentFeatureContraintModel::fixFeatureJac(vec_range& L, mat& JfL)
{
  JFR_PRECOND(L.size()==6,
              "SegmentFeatureModel::fixFeature:");

    double dx=1e-6;
    vec L1(6);
    vec L0(6);
    vec_range L2=project(L1,range(0,6));
    L0=L;
    fixFeature(L);
//    JfL.assign(identity_mat(6));
   for (int i=0; i<=5; i++) {
        L1=L0;
        L1(i)+=dx;
        L2=project(L1,range(0,6));
        fixFeature(L2);
        ublas::column(JfL, i).assign((L2-L)/dx);
    }
//     vec s(6);
//     mat_column_major U(6,6);
//     mat_column_major VT(6,6);
//     mat_column_major J(6,6);
// 
//     J.assign(JfL);
// 
//     int ierr1 = lapack::gesdd(J,s,U,VT);
//     if (ierr1!=0) {
//     throw(jmath::LapackException(ierr1,
//           "SegmentFeatureModel::fixFeature: error in lapack::gesdd() routine.",
//           __FILE__,
//           __LINE__));
//     }
// 
//     mat S(6,6);
//     S.clear();
// 
//     for (int i=0; i<=5; i++)
//     {
//        S(i,i)=s(i);
//     }
// 
//     JfL.assign(prod(U,mat(prod(S,VT))));
 
//        JFR_DEBUG("L0: "<< L0);
//        JFR_DEBUG("L: "<< L);
//        JFR_DEBUG("norm: "<< ublas::norm_2(L0-L));
//     if (ublas::norm_2(L0-L) >= 0.01) {
//         JFR_DEBUG("JAJAJAJAJAJA");
//         sqrt(-1);
//     }
}


/*
 * class SegmentFeatureModel
 */

SegmentFeatureModel::SegmentFeatureModel() :
  FeatureModel(6, 6, 6) {} // Real size of the merge state

SegmentFeatureModel::~SegmentFeatureModel() {}

void SegmentFeatureModel::toFrame(const vec& frame_, const vec& x_, vec& xRes)
{
  JFR_TRACE_BEGIN;
  EulerTools::lineToFrame(frame_, x_, xRes);
  JFR_TRACE_END("SegmentFeatureModel::toFrame()");
}

void SegmentFeatureModel::toFrameJac(const vec& frame_, const vec& x_)
{
  JFR_TRACE_BEGIN;
  EulerTools::lineToFrameJac(frame_, x_, Jframe, Jx);
//  JFR_DEBUG("Jframe = " << Jframe);
//  JFR_DEBUG("LEAVING TOFRAME Jx     = " << Jx);
  JFR_TRACE_END("SegmentFeatureModel::toFrameJac()");
}

void SegmentFeatureModel::fromFrame(const vec& frame_, const vec& x_, vec& xRes)
{
  JFR_TRACE_BEGIN;
  //JFR_DEBUG("fromFrame (L) before " << x_);
  EulerTools::lineFromFrame(frame_, x_, xRes);
  JFR_TRACE_END("SegmentFeatureModel::fromFrame()");
  //JFR_DEBUG("fromFrame (L) after " << xRes);
}

void SegmentFeatureModel::fromFrame(const vec& frame_, const vec& x_, vec_range& xRes)
{
  JFR_TRACE_BEGIN;
  //JFR_DEBUG("fromFrame (L) before " << x_);
  EulerTools::lineFromFrame(frame_, x_, xRes);
  JFR_TRACE_END("SegmentFeatureModel::fromFrame()");
  //JFR_DEBUG("fromFrame (L) after " << xRes);
}

void SegmentFeatureModel::fromFrameJac(const vec& frame_, const vec& x_)
{
  JFR_TRACE_BEGIN;
  EulerTools::lineFromFrameJac(frame_, x_, Jframe, Jx);
//  JFR_DEBUG("Jframe = " << Jframe);
//  JFR_DEBUG("LEAVING FROMFRAME Jx     = " << Jx);
  JFR_TRACE_END("SegmentFeatureModel::fromFrameJac()");
  //JFR_DEBUG("fromFrameJac (Jframe) " << Jframe );
  //JFR_DEBUG("fromFrameJac (JL) " << Jx );
  //JFR_DEBUG("fromFrameJac (frame) " << frame_ );
  //JFR_DEBUG("fromFrameJac (x) " << x_ );
}


void SegmentFeatureModel::computeMergeState( const vec& x_, vec& mergeX_, mat& jac )
{
  //JFR_RUN_TIME("Unimplemented.");
  mergeX_ = x_;
  jac.assign(jblas::identity_mat(6));
}

/*
 * class ImageSegmentFeatureObserveModel
 */

ImageEuclideanPluckerFeatureObserveModel::ImageEuclideanPluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, 
								 jafar::camera::CameraPinhole const& camera_,
								 double depthAlpha_, double depthKSigma_,
								 double phiMin_, double phiMax_,
								 double phiSigma_,
								 double phiKSigma_) :
  BearingOnlyFeatureObserveModel(featureModel_, 2),
  camera(camera_),
  Pl(),
  Plinv(),
  p_pixCov(2,2),
  depthAlpha(depthAlpha_), depthBeta(kSigmaToBeta(depthAlpha_, depthKSigma_)),
  phiMin(phiMin_), phiMax(phiMax_),
  phiSigma(phiSigma_),
  phiKSigma(phiKSigma_)
{
  double alphaU_alphaV = camera.alphaU*camera.alphaV;

  Pl.clear();
  Pl(0,0) = camera.alphaV;
  Pl(1,1) = camera.alphaU;

  Pl(2,0) = -1.0*camera.alphaV*camera.u0;
  Pl(2,1) = -1.0*camera.alphaU*camera.v0;
  Pl(2,2) = alphaU_alphaV;

  Plinv.clear();
  Plinv(0,0) = 1.0 / camera.alphaV;
  Plinv(1,1) = 1.0 / camera.alphaU;
  Plinv(2,0) = camera.u0 / alphaU_alphaV;
  Plinv(2,1) = camera.v0 / alphaU_alphaV;
  Plinv(2,2) = 1.0 / alphaU_alphaV;

  p_pixCov.clear();

  // this part of the jacobian is always 0
  project(JobsSensor, range(0,2), range(3,6)).assign( zero_mat(2,3) );
}

ImageEuclideanPluckerFeatureObserveModel::~ImageEuclideanPluckerFeatureObserveModel() {}

void ImageEuclideanPluckerFeatureObserveModel::setPixCov(vec const& pixCov, double stabilizingFactor)
{
	JFR_PRECOND(pixCov.size()==2,
				"ImageEuclideanPluckerFeatureObserveModel::setPixCov:");
	
	p_pixCov.clear();
	p_pixCov(0,0) = stabilizingFactor*stabilizingFactor*pixCov(0);
	p_pixCov(1,1) = stabilizingFactor*stabilizingFactor*pixCov(1);
}

void ImageEuclideanPluckerFeatureObserveModel::initStateInSensorFrame(InitFeature& feature_,
							     Observation const& obs_,
							     double dMin_, double dMax_)
{
  JFR_TRACE_BEGIN;
  // get the SegmentObservation
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);

  feature_.initState.clear();
  
  // gaussian sum for depth of the segment
  std::list<jmath::WeightedGaussianVector> depthGaussianSum;
  geometricUniformGaussianSum(depthAlpha, depthBeta, dMin_, dMax_, depthGaussianSum);

  // gaussian sum for orientation of the segment
  std::list<jmath::GaussianVector> phiGaussianSum;
  basicUniformGaussianSum(phiMin, phiMax, phiSigma, phiKSigma, phiGaussianSum);
  
  // compute reference point abscissa on the image line
  double sg;
  {
    double rho = segObs.z(0);
    double theta = segObs.z(1);

    // reference pixel of the segment (the middle point)
    vec2 pixRef = (segObs.ext1+segObs.ext2)/2 ;

    // closest point to origin
    vec2 lo;
    lo(0) = rho*cos(theta);
    lo(1) = rho*sin(theta);

    // unit vector
    vec2 lu;
    lu(0) = -sin(theta);
    lu(1) = cos(theta);

    sg = inner_prod(pixRef - lo, lu);
  }

  // final gaussian sum

  // plucker line state
  vec xi(6);
  sym_mat xiCov(6,6);

  // jacobian of pluckerLineInit
  mat Jinit(6,4);
  // covariance of parameters defining the init state
  sym_mat paramCov(4,4);
  paramCov.clear();
  computeSensorR(obs_);
  project(paramCov, range(0,2), range(0,2)).assign(getR());

  for (std::list<jmath::WeightedGaussianVector>::iterator itDepth = depthGaussianSum.begin() ;
       itDepth != depthGaussianSum.end() ; ++itDepth)
    {
      for (std::list<jmath::GaussianVector>::iterator itPhi = phiGaussianSum.begin() ;
	   itPhi != phiGaussianSum.end() ; ++itPhi)
	{
	  pluckerLineInit(segObs.z, itDepth->x(0), itPhi->x(0), sg, xi);

	  pluckerLineInitJac(segObs.z, itDepth->x(0), itPhi->x(0), sg, Jinit);

	  paramCov(2,2) = itDepth->P(0,0);
	  paramCov(3,3) = itPhi->P(0,0);
	   
	  // 	   //JFR_DEBUG("param: z = " << segObs.z << " d=" << (*itDepth)->x(0) << " phi=" << (*itPhi)->x(0));
	  // 	   //JFR_DEBUG("paramCov = " << paramCov);
	  // 	   //JFR_DEBUG("det(paramCov)=" << jmath::ublasExtra::lu_det(paramCov));
	   
	  xiCov.assign( prec_prod(Jinit, mat(prec_prod(paramCov, trans(Jinit) )) ) );
	  feature_.initState.push_back(new InitStateMember(xi, xiCov, itDepth->w,*itDepth, *itPhi, sizeObs()));

	  // 	   //JFR_DEBUG("xi = " << xi);
	  // 	   //JFR_DEBUG("xiCov = " << xiCov);
	  //  	   //JFR_DEBUG("det(xiCov) = " << ublasExtra::lu_det(xiCov));
	  //  	   //JFR_DEBUG("det(niCov) = " << ublasExtra::lu_det(project(xiCov, range(0,3), range(0,3))));
	  //  	   //JFR_DEBUG("det(uiCov) = " << ublasExtra::lu_det(project(xiCov, range(3,6), range(3,6))));
	}
    }

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::initStateInSensorFrame()");
}

vec const& ImageEuclideanPluckerFeatureObserveModel::predictObservationInImageFrame(vec3 const& l_)
{
  vec3 l = l_;

  // orientation convention
  if (l(2)>0.0) l *= -1.0;

  z(0) = -1.0*l(2)/sqrt(l(0)*l(0) + l(1)*l(1));
  z(1) = atan2(l(1), l(0));
  
  
  
  return z;
}

void ImageEuclideanPluckerFeatureObserveModel::predictObservationInImageFrameJac(vec3 const& l_,
									mat& Jl) const
{
  JFR_PRECOND(Jl.size1() == 2 && Jl.size2() == 3,
	      "ImageEuclideanPluckerFeatureObserveModel::predictObservationInImageFrameJac: invalid size of Jl");

  vec3 l = l_;

  // orientation convention
  if (l_(2)>0.0) l *= -1.0;

  double a = l(0);
  double b = l(1);
  double c = l(2);
  /* begin copy/paste from maple */
  double t1 = a * a;
  double t2 = b * b;
  double t3 = t1 + t2;
  double t4 = sqrt(t3);
  double t7 = c / t4 / t3;
  double t14 = 0.1e1 / t1;
  double t18 = 0.1e1 / (0.1e1 + t2 * t14);
  Jl(0,0) = 0.10e1 * t7 * a;
  Jl(0,1) = 0.10e1 * t7 * b;
  Jl(0,2) = -0.10e1 / t4;
  Jl(1,0) = -b * t14 * t18;
  Jl(1,1) = 0.1e1 / a * t18;
  Jl(1,2) = 0.0e0;
  /* end copy/paste from maple */

  // orientation convention
  if (l_(2)>0.0) Jl *= -1.0;
}

vec const& ImageEuclideanPluckerFeatureObserveModel::computeInnovation(vec const& z_,
								     vec const& zPred)
{
  JFR_PRECOND(z_.size() == sizeObs(),
              "ImageEuclideanPluckerFeatureObserveModel::computeInnovation: size of z_ does not match");
  JFR_PRECOND(zPred.size() == sizeObs(),
              "ImageEuclideanPluckerFeatureObserveModel::computeInnovation: size of zPred_ does not match");

  z = z_ - zPred;

  // 2PI modulo
  // FIXME

  // PI rotation when origin is traversed
  if (z(1) > M_PI/2) {
     //JFR_DEBUG("computeInnovation: zPred=" << zPred);
     //JFR_DEBUG("computeInnovation: y=" << z);
    z(1) -= M_PI;
    z(0) = -1.0*z_(0) - zPred(0);
  }
  else if (z(1) < -M_PI/2) {
     //JFR_DEBUG("computeInnovation: zPred=" << zPred);
     //JFR_DEBUG("computeInnovation: y=" << z);
    z(1) += M_PI;
    z(0) = -1.0*z_(0) - zPred(0);
  }
  return z;
}

vec const& ImageEuclideanPluckerFeatureObserveModel::predictObservationInSensorFrame(const vec& feature_)
{
  JFR_TRACE_BEGIN;
  JFR_PRECOND(feature_.size() == 6,
	      "ImageEuclideanPluckerFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");

  return predictObservationInImageFrame( prod(Pl, project(feature_, range(0,3))) );

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::predictObservationInSensorFrame()");
}

void ImageEuclideanPluckerFeatureObserveModel::predictObservationInSensorFrameJac(const vec& feature_)
{

  JFR_PRECOND(feature_.size() == 6,
	      "ImageEuclideanPluckerFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");

  mat Jl(2,3);
  predictObservationInImageFrameJac( prod(Pl, project(feature_, range(0,3))), Jl );
  
  project(JobsSensor, range(0,2), range(0,3)).assign( prod(Jl,Pl) );

  // this is done at construction
  // project(JobsSensor, range(0,2), range(3,6)).assign( zero_mat(3,3) );
}


void ImageEuclideanPluckerFeatureObserveModel::pluckerLineInit(vec const& z_, double d, double phi, double sg, 
						      vec& pluckerLine) const
{
  JFR_PRECOND(z_.size() == 2, "ImageEuclideanPluckerFeatureObserveModel::pluckerLineInit:");
  JFR_PRECOND(pluckerLine.size() == 6, "ImageEuclideanPluckerFeatureObserveModel::pluckerLineInit:");

  double rho = z_(0);
  double theta = z_(1);

  double alphaU = camera.alphaU;
  double alphaV = camera.alphaV;
  double u0 = camera.u0;
  double v0 = camera.v0;

  /* begin maple */
  double t1 = sin(phi);
  double t2 = d * t1;
  double t3 = alphaV * alphaV;
  double t4 = 0.1e1 / t3;
  double t5 = cos(theta);
  double t6 = t5 * t5;
  double t7 = t4 * t6;
  double t8 = alphaU * alphaU;
  double t9 = 0.1e1 / t8;
  double t10 = sin(theta);
  double t11 = t10 * t10;
  double t12 = t9 * t11;
  double t13 = 0.1e1 / alphaU;
  double t14 = 0.1e1 / alphaV;
  double t15 = t13 * t14;
  double t21 = t15 * u0 * t5 + t15 * v0 * t10 - t15 * rho;
  double t22 = t21 * t21;
  double t23 = t7 + t12 + t22;
  double t24 = sqrt(t23);
  double t25 = 0.1e1 / t24;
  double t34 = phi * phi;
  double t35 = 0.1e1 / t23;
  double t36 = t34 * t35;
  double t40 = t36 * t7 + t36 * t12 + t36 * t22;
  double t41 = sqrt(t40);
  double t42 = cos(t41);
  double t44 = (0.1e1 - t42) * t34;
  double t45 = t44 * t35;
  double t46 = 0.1e1 / t40;
  double t52 = rho * t5 - sg * t10 - u0;
  double t53 = t52 * t52;
  double t57 = rho * t10 + sg * t5 - v0;
  double t58 = t57 * t57;
  double t61 = sqrt(0.100e1 + t53 * t9 + t58 * t4);
  double t62 = 0.1e1 / t61;
  double t64 = t52 * t13;
  double t69 = t13 * t10;
  double t71 = t44 * t35 * t14 * t5 * t46 * t69;
  double t72 = phi * t25;
  double t73 = 0.1e1 / t41;
  double t75 = sin(t41);
  double t77 = t72 * t21 * t73 * t75;
  double t80 = t57 * t14;
  double t83 = t46 * t21;
  double t85 = t45 * t14 * t5 * t83;
  double t89 = t72 * t13 * t10 * t73 * t75;
  double t103 = t45 * t69 * t83;
  double t107 = t72 * t14 * t5 * t73 * t75;
  pluckerLine(0) = t2 * t25 * t14 * t5;
  pluckerLine(1) = t2 * t25 * t13 * t10;
  pluckerLine(2) = t2 * t25 * t21;
  pluckerLine(3) = (t45 * t7 * t46 + t42) * t62 * t64 + (t71 - t77) * t62 * t80 + 0.10e1 * (t85 + t89) * t62;
  pluckerLine(4) = (t71 + t77) * t62 * t64 + (t45 * t12 * t46 + t42) * t62 * t80 + 0.10e1 * (t103 - t107) * t62;
  pluckerLine(5) = (t85 - t89) * t62 * t64 + (t103 + t107) * t62 * t80 + 0.10e1 * (t44 * t35 * t22 * t46 + t42) * t62;
  /* end maple */  
}

void ImageEuclideanPluckerFeatureObserveModel::pluckerLineInitJac(vec const& z_, double d, double phi, double sg, 
							 mat& J) const
{
  JFR_PRECOND(z_.size() == 2, "ImageEuclideanPluckerFeatureObserveModel::pluckerLineInitJac:");
  JFR_PRECOND(J.size1() == 6 && J.size2() == 4, "ImageEuclideanPluckerFeatureObserveModel::pluckerLineInitJac:");

  double rho = z_(0);
  double theta = z_(1);

  double alphaU = camera.alphaU;
  double alphaV = camera.alphaV;
  double u0 = camera.u0;
  double v0 = camera.v0;
  
  /* begin maple */
  double t1 = sin(phi);
  double t2 = d * t1;
  double t3 = alphaV * alphaV;
  double t4 = 0.1e1 / t3;
  double t5 = cos(theta);
  double t6 = t5 * t5;
  double t7 = t4 * t6;
  double t8 = alphaU * alphaU;
  double t9 = 0.1e1 / t8;
  double t10 = sin(theta);
  double t11 = t10 * t10;
  double t12 = t9 * t11;
  double t13 = 0.1e1 / alphaU;
  double t14 = 0.1e1 / alphaV;
  double t15 = t13 * t14;
  double t21 = t15 * u0 * t5 + t15 * v0 * t10 - t15 * rho;
  double t22 = t21 * t21;
  double t23 = t7 + t12 + t22;
  double t24 = sqrt(t23);
  double t26 = 0.1e1 / t24 / t23;
  double t27 = t2 * t26;
  double t28 = t4 * t5;
  double t29 = t21 * t13;
  double t32 = t14 * t5;
  double t33 = t28 * t10;
  double t34 = t9 * t10;
  double t35 = t34 * t5;
  double t40 = -t15 * u0 * t10 + t15 * v0 * t5;
  double t41 = t21 * t40;
  double t42 = -t33 + t35 + t41;
  double t46 = 0.1e1 / t24;
  double t47 = t46 * t14;
  double t51 = t1 * t46;
  double t53 = cos(phi);
  double t54 = d * t53;
  double t57 = t21 * t14;
  double t60 = t13 * t10;
  double t61 = 0.2e1 * t60 * t42;
  double t64 = t46 * t13;
  double t71 = t22 * t13;
  double t85 = t46 * t21;
  double t87 = phi * phi;
  double t88 = 0.1e1 / t23;
  double t89 = t87 * t88;
  double t90 = t89 * t7;
  double t91 = t89 * t12;
  double t92 = t89 * t22;
  double t93 = t90 + t91 + t92;
  double t94 = sqrt(t93);
  double t95 = sin(t94);
  double t97 = 0.1e1 / t94 / t93;
  double t98 = t95 * t97;
  double t99 = t23 * t23;
  double t100 = 0.1e1 / t99;
  double t101 = t87 * t100;
  double t103 = 0.1e1 / t3 / alphaV;
  double t109 = 0.1e1 / t8 / alphaU;
  double t114 = t22 * t21;
  double t120 = t101 * t103 * t6 * t21 * t13 + t101 * t109 * t11 * t21 * t14 + t101 * t114 * t13 * t14 - t89 * t29 * t14;
  double t121 = 0.2e1 * t98 * t120;
  double t124 = cos(t94);
  double t125 = 0.1e1 - t124;
  double t126 = t125 * t87;
  double t129 = 0.1e1 / t93;
  double t134 = t126 * t88;
  double t135 = t93 * t93;
  double t136 = 0.1e1 / t135;
  double t137 = 0.2e1 * t136 * t120;
  double t140 = 0.1e1 / t94;
  double t141 = t95 * t140;
  double t143 = t141 * t120;
  double t145 = rho * t5;
  double t146 = sg * t10;
  double t147 = t145 - t146 - u0;
  double t148 = t147 * t147;
  double t150 = rho * t10;
  double t151 = sg * t5;
  double t152 = t150 + t151 - v0;
  double t153 = t152 * t152;
  double t155 = 0.100e1 + t148 * t9 + t153 * t4;
  double t156 = sqrt(t155);
  double t157 = 0.1e1 / t156;
  double t159 = t147 * t13;
  double t161 = t7 * t129;
  double t163 = t134 * t161 + t124;
  double t165 = 0.1e1 / t156 / t155;
  double t166 = t163 * t165;
  double t167 = t147 * t9;
  double t169 = t152 * t4;
  double t171 = t167 * t5 + t169 * t10;
  double t172 = 0.2e1 * t159 * t171;
  double t175 = t163 * t157;
  double t176 = t5 * t13;
  double t179 = 0.2e1 * t98 * t120 * t87;
  double t180 = t88 * t14;
  double t182 = t180 * t176 * t10;
  double t184 = t179 * t182 / 0.2e1;
  double t186 = t126 * t100 * t4;
  double t187 = t5 * t129;
  double t191 = 0.2e1 * t186 * t187 * t34 * t21;
  double t192 = t126 * t180;
  double t193 = t5 * t136;
  double t196 = 0.2e1 * t192 * t193 * t60 * t120;
  double t197 = phi * t26;
  double t200 = t197 * t22 * t141 * t15;
  double t201 = phi * t46;
  double t202 = t201 * t13;
  double t205 = t202 * t14 * t140 * t95;
  double t206 = t201 * t21;
  double t208 = t206 * t121 / 0.2e1;
  double t209 = t129 * t124;
  double t212 = t206 * t209 * t120;
  double t215 = t152 * t14;
  double t217 = t187 * t60;
  double t218 = t192 * t217;
  double t221 = t201 * t21 * t140 * t95;
  double t222 = t218 - t221;
  double t223 = t222 * t165;
  double t224 = 0.2e1 * t215 * t171;
  double t227 = t222 * t157;
  double t228 = t10 * t14;
  double t231 = t180 * t5 * t21;
  double t233 = t179 * t231 / 0.2e1;
  double t236 = 0.2e1 * t186 * t187 * t71;
  double t237 = 0.2e1 * t21 * t120;
  double t239 = t192 * t193 * t237;
  double t240 = t129 * t13;
  double t242 = t134 * t28 * t240;
  double t245 = t197 * t34 * t141 * t57;
  double t246 = t10 * t97;
  double t247 = 0.2e1 * t95 * t120;
  double t250 = t202 * t246 * t247 / 0.2e1;
  double t251 = t10 * t129;
  double t252 = 0.2e1 * t124 * t120;
  double t255 = t202 * t251 * t252 / 0.2e1;
  double t259 = t129 * t21;
  double t260 = t32 * t259;
  double t261 = t134 * t260;
  double t262 = t10 * t140;
  double t263 = t262 * t95;
  double t264 = t202 * t263;
  double t266 = (t261 + t264) * t165;
  double t282 = -0.2e1 * t101 * t7 * t42 - 0.2e1 * t89 * t33 - 0.2e1 * t101 * t12 * t42 + 0.2e1 * t89 * t35 - 0.2e1 * t101 * t22 * t42 + 0.2e1 * t89 * t41;
  double t283 = t98 * t282;
  double t286 = t126 * t100;
  double t287 = 0.2e1 * t129 * t42;
  double t293 = t136 * t282;
  double t297 = t141 * t282 / 0.2e1;
  double t301 = -t150 - t151;
  double t303 = t145 - t146;
  double t305 = t167 * t301 + t169 * t303;
  double t306 = 0.2e1 * t159 * t305;
  double t309 = t301 * t13;
  double t312 = t98 * t282 * t87;
  double t314 = t312 * t182 / 0.2e1;
  double t316 = t126 * t100 * t14;
  double t318 = t316 * t187 * t61;
  double t321 = t134 * t14 * t11 * t240;
  double t324 = t192 * t193 * t60 * t282;
  double t327 = t134 * t14 * t6 * t240;
  double t331 = t197 * t21 * t141 * t42;
  double t334 = t201 * t40 * t140 * t95;
  double t336 = t206 * t283 / 0.2e1;
  double t339 = t206 * t209 * t282 / 0.2e1;
  double t343 = 0.2e1 * t215 * t305;
  double t346 = t303 * t14;
  double t349 = t312 * t231 / 0.2e1;
  double t350 = 0.2e1 * t21 * t42;
  double t352 = t316 * t187 * t350;
  double t354 = t134 * t228 * t259;
  double t355 = t21 * t282;
  double t357 = t192 * t193 * t355;
  double t358 = t129 * t40;
  double t360 = t134 * t32 * t358;
  double t362 = 0.2e1 * t95 * t42;
  double t365 = t197 * t13 * t262 * t362 / 0.2e1;
  double t366 = t5 * t140;
  double t367 = t366 * t95;
  double t368 = t202 * t367;
  double t369 = t95 * t282;
  double t372 = t202 * t246 * t369 / 0.2e1;
  double t373 = t124 * t282;
  double t376 = t202 * t251 * t373 / 0.2e1;
  double t383 = phi * t88;
  double t387 = t383 * t7 + t383 * t12 + t383 * t22;
  double t388 = 0.2e1 * t98 * t387;
  double t391 = t125 * phi;
  double t392 = t391 * t88;
  double t395 = 0.2e1 * t136 * t387;
  double t399 = t141 * t387;
  double t404 = 0.2e1 * t98 * t387 * t87;
  double t406 = t404 * t182 / 0.2e1;
  double t409 = 0.2e1 * t391 * t180 * t217;
  double t412 = 0.2e1 * t192 * t193 * t60 * t387;
  double t413 = t85 * t141;
  double t415 = t206 * t388 / 0.2e1;
  double t418 = t206 * t209 * t387;
  double t423 = t404 * t231 / 0.2e1;
  double t425 = 0.2e1 * t392 * t260;
  double t426 = 0.2e1 * t21 * t387;
  double t428 = t192 * t193 * t426;
  double t429 = t64 * t263;
  double t430 = 0.2e1 * t95 * t387;
  double t433 = t202 * t246 * t430 / 0.2e1;
  double t434 = 0.2e1 * t124 * t387;
  double t437 = t202 * t251 * t434 / 0.2e1;
  double t445 = t218 + t221;
  double t446 = t445 * t165;
  double t449 = t445 * t157;
  double t464 = t12 * t129;
  double t466 = t134 * t464 + t124;
  double t467 = t466 * t165;
  double t470 = t466 * t157;
  double t472 = t88 * t13;
  double t474 = t472 * t10 * t21;
  double t476 = t179 * t474 / 0.2e1;
  double t482 = 0.2e1 * t126 * t100 * t9 * t251 * t22 * t14;
  double t483 = t126 * t472;
  double t484 = t10 * t136;
  double t486 = t483 * t484 * t237;
  double t489 = t134 * t34 * t129 * t14;
  double t492 = t197 * t28 * t141 * t29;
  double t493 = t201 * t14;
  double t494 = t5 * t97;
  double t497 = t493 * t494 * t247 / 0.2e1;
  double t500 = t493 * t187 * t252 / 0.2e1;
  double t504 = t60 * t259;
  double t505 = t134 * t504;
  double t506 = t493 * t367;
  double t508 = (t505 - t506) * t165;
  double t534 = t312 * t474 / 0.2e1;
  double t538 = t126 * t100 * t13 * t251 * t350;
  double t540 = t134 * t176 * t259;
  double t542 = t483 * t484 * t355;
  double t544 = t134 * t60 * t358;
  double t548 = t197 * t14 * t366 * t362 / 0.2e1;
  double t549 = t493 * t263;
  double t552 = t493 * t494 * t369 / 0.2e1;
  double t555 = t493 * t187 * t373 / 0.2e1;
  double t575 = t404 * t474 / 0.2e1;
  double t577 = 0.2e1 * t392 * t504;
  double t579 = t483 * t484 * t426;
  double t580 = t47 * t367;
  double t583 = t493 * t494 * t430 / 0.2e1;
  double t586 = t493 * t187 * t434 / 0.2e1;
  double t594 = t261 - t264;
  double t595 = t594 * t165;
  double t598 = t594 * t157;
  double t603 = t505 + t506;
  double t604 = t603 * t165;
  double t607 = t603 * t157;
  double t618 = t22 * t136;
  double t625 = t88 * t22 * t129;
  double t628 = (t126 * t625 + t124) * t165;
  J(0,0) = t27 * t28 * t29;
  J(0,1) = -t27 * t32 * t42 - t2 * t47 * t10;
  J(0,2) = t51 * t32;
  J(0,3) = t54 * t47 * t5;
  J(1,0) = t27 * t34 * t57;
  J(1,1) = -t27 * t61 / 0.2e1 + t2 * t64 * t5;
  J(1,2) = t51 * t60;
  J(1,3) = t54 * t64 * t10;
  J(2,0) = t27 * t71 * t14 - t2 * t64 * t14;
  J(2,1) = -t2 * t26 * t21 * t42 + t2 * t46 * t40;
  J(2,2) = t51 * t21;
  J(2,3) = t54 * t85;
  J(3,0) = (t121 * t90 / 0.2e1 + 0.2e1 * t126 * t100 * t103 * t6 * t129 * t29 - t134 * t7 * t137 - t143) * t157 * t159 - t166 * t172 / 0.2e1 + t175 * t176 + (t184 + t191 - t196 - t200 + t205 + t208 - t212) * t157 * t215 - t223 * t224 / 0.2e1 + t227 * t228 + 0.10e1 * (t233 + t236 - t239 - t242 + t245 - t250 + t255) * t157 - 0.1000000000e1 * t266 * t171;
  J(3,1) = (t283 * t90 / 0.2e1 - t286 * t7 * t287 - 0.2e1 * t134 * t28 * t251 - t134 * t7 * t293 - t297) * t157 * t159 - t166 * t306 / 0.2e1 + t175 * t309 + (t314 - t318 - t321 - t324 + t327 + t331 - t334 + t336 - t339) * t157 * t215 - t223 * t343 / 0.2e1 + t227 * t346 + 0.10e1 * (t349 - t352 - t354 - t357 + t360 - t365 + t368 - t372 + t376) * t157 - 0.1000000000e1 * t266 * t305;
  J(3,2) = 0.0e0;
  J(3,3) = (t388 * t90 / 0.2e1 + 0.2e1 * t392 * t161 - t134 * t7 * t395 - t399) * t157 * t159 + (t406 + t409 - t412 - t413 + t415 - t418) * t157 * t215 + 0.10e1 * (t423 + t425 - t428 + t429 - t433 + t437) * t157;
  J(4,0) = (t184 + t191 - t196 + t200 - t205 - t208 + t212) * t157 * t159 - t446 * t172 / 0.2e1 + t449 * t176 + (t121 * t91 / 0.2e1 + 0.2e1 * t126 * t100 * t109 * t11 * t129 * t57 - t134 * t12 * t137 - t143) * t157 * t215 - t467 * t224 / 0.2e1 + t470 * t228 + 0.10e1 * (t476 + t482 - t486 - t489 - t492 + t497 - t500) * t157 - 0.1000000000e1 * t508 * t171;
  J(4,1) = (t314 - t318 - t321 - t324 + t327 - t331 + t334 - t336 + t339) * t157 * t159 - t446 * t306 / 0.2e1 + t449 * t309 + (t283 * t91 / 0.2e1 - t286 * t12 * t287 + 0.2e1 * t134 * t34 * t187 - t134 * t12 * t293 - t297) * t157 * t215 - t467 * t343 / 0.2e1 + t470 * t346 + 0.10e1 * (t534 - t538 + t540 - t542 + t544 + t548 + t549 + t552 - t555) * t157 - 0.1000000000e1 * t508 * t305;
  J(4,2) = 0.0e0;
  J(4,3) = (t406 + t409 - t412 + t413 - t415 + t418) * t157 * t159 + (t388 * t91 / 0.2e1 + 0.2e1 * t392 * t464 - t134 * t12 * t395 - t399) * t157 * t215 + 0.10e1 * (t575 + t577 - t579 - t580 + t583 - t586) * t157;
  J(5,0) = (t233 + t236 - t239 - t242 - t245 + t250 - t255) * t157 * t159 - t595 * t172 / 0.2e1 + t598 * t176 + (t476 + t482 - t486 - t489 + t492 - t497 + t500) * t157 * t215 - t604 * t224 / 0.2e1 + t607 * t228 + 0.10e1 * (t121 * t92 / 0.2e1 + 0.2e1 * t286 * t114 * t129 * t15 - 0.2e1 * t134 * t259 * t15 - 0.2e1 * t134 * t618 * t120 - t143) * t157 - 0.1000000000e1 * t628 * t171;
  J(5,1) = (t349 - t352 - t354 - t357 + t360 + t365 - t368 + t372 - t376) * t157 * t159 - t595 * t306 / 0.2e1 + t598 * t309 + (t534 - t538 + t540 - t542 + t544 - t548 - t549 - t552 + t555) * t157 * t215 - t604 * t343 / 0.2e1 + t607 * t346 + 0.10e1 * (t283 * t92 / 0.2e1 - 0.2e1 * t286 * t22 * t129 * t42 + 0.2e1 * t134 * t259 * t40 - t134 * t618 * t282 - t297) * t157 - 0.1000000000e1 * t628 * t305;
  J(5,2) = 0.0e0;
  J(5,3) = (t423 + t425 - t428 - t429 + t433 - t437) * t157 * t159 + (t575 + t577 - t579 + t580 - t583 + t586) * t157 * t215 + 0.10e1 * (t388 * t92 / 0.2e1 + 0.2e1 * t391 * t625 - 0.2e1 * t134 * t618 * t387 - t399) * t157;
  /* end maple */
}

void ImageEuclideanPluckerFeatureObserveModel::initInternalState(BaseFeature& baseFeature, 
							T3DEuler const& robotPose_, 
							Observation const& obs_) const
{
  JFR_TRACE_BEGIN;

//  JFR_DEBUG("obs:" << obs_);
  
  SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);

  double s1,s2;
  boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);

  //   JFR_DEBUG("s1: " << s1 << " s2: " << s2);

  if (s1 < s2) {
    segFeature.s1 = s1;
    segFeature.s2 = s2;
  }
  else {
    segFeature.s1 = s2;
    segFeature.s2 = s1;
  } 

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::initInternalState()");
}

void ImageEuclideanPluckerFeatureObserveModel::updateInternalState(BaseFeature& baseFeature, 
							  T3DEuler const& robotPose_, 
							  Observation const& obs_) const
{
  JFR_TRACE_BEGIN;
  SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);

     //JFR_DEBUG("norm(u) = " << norm_2(segFeature.u()));
#ifdef ENABLE_NORMALISATION
  ublasExtra::normalize(segFeature.u());
#endif

  double s1,s2;
  boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);

  //   //JFR_DEBUG("s1: " << s1 << " s2: " << s2);
#if 1
	// set extremitites to current ones
	if (s1 < s2) {
		segFeature.s1 = s1;
		segFeature.s2 = s2;
	}
	else {
		segFeature.s1 = s2;
		segFeature.s2 = s1;
	}
#else
	// only enlarge extremities
	if (s1 < s2) {
		if (s1 < segFeature.s1)
			segFeature.s1 = s1;
		if (s2 > segFeature.s2)
			segFeature.s2 = s2;
	}
	else {
		if (s2 < segFeature.s1)
			segFeature.s1 = s2;
		if (s1 > segFeature.s2)
			segFeature.s2 = s1;
	}
#endif

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::updateInternalState()");
}

boost::tuple<double,double> ImageEuclideanPluckerFeatureObserveModel::computeExtremitiesAbscissa(SegmentFeature const& segFeature_,
											T3DEuler const& robotPose_,
											SegmentObservation const& segObs) const
{
  JFR_TRACE_BEGIN;

  using namespace boost;

  T3DEuler sensorPose;
  T3D::compose(robotPose_, robotToSensor(), sensorPose);

  vec lineExt1Sensor(6);
  vec lineExt2Sensor(6);
  vec lineExt1(6);
  vec lineExt2(6);

  segmentExtremitiesPluckerLines(segObs, lineExt1Sensor, lineExt2Sensor);
 
  segFeature_.model.fromFrame(sensorPose.getX(), lineExt1Sensor, lineExt1);
  segFeature_.model.fromFrame(sensorPose.getX(), lineExt2Sensor, lineExt2);

  double s1, s2;
  double tmp1, tmp2;

  tie(s1, tmp1, tmp2) =
    pluckerLinesDistance(segFeature_.getX(), lineExt1);

  tie(s2, tmp1, tmp2) =
    pluckerLinesDistance(segFeature_.getX(), lineExt2);

  return make_tuple(s1,s2);

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::computeExtremitiesAbscissa()");
}

void ImageEuclideanPluckerFeatureObserveModel::segmentExtremitiesPluckerLines(SegmentObservation const& segObs_, 
								     vec& lineExt1Sensor, 
								     vec& lineExt2Sensor) const
{
  JFR_PRECOND(lineExt1Sensor.size()==6 && lineExt2Sensor.size() == 6,
	      "ImageEuclideanPluckerFeatureObserveModel::segmentExtremitiesPluckerLines: a plucker line is a 6-vector");
  lineExt1Sensor.clear();
  lineExt2Sensor.clear();

  vec ext1Sensor(3);
  camera.imageToCameraFrame(segObs_.ext1, ext1Sensor);
  vec ext2Sensor(3);
  camera.imageToCameraFrame(segObs_.ext2, ext2Sensor);

  project(lineExt1Sensor, range(3, 6)).assign(ext1Sensor/norm_2(ext1Sensor));
  project(lineExt2Sensor, range(3, 6)).assign(ext2Sensor/norm_2(ext2Sensor));
}

boost::tuple<double, double, double> ImageEuclideanPluckerFeatureObserveModel::pluckerLinesDistance(vec const& line1_, 
											   vec const& line2_)
{
  JFR_PRECOND(line1_.size() == 6 && line2_.size() == 6, 
	      "ImageEuclideanPluckerFeatureObserveModel::pluckerLinesDistance:");
  JFR_TRACE_BEGIN;

  vec3 u1 = project(line1_, range(3, 6));
  vec3 u2 = project(line2_, range(3, 6));
  vec3 o1;
  vec3 o2;
  ublasExtra::crossProd(u1, project(line1_, range(0, 3)), o1);
  ublasExtra::crossProd(u2, project(line2_, range(0, 3)), o2);
  //  JFR_DEBUG("o1 : " << o1 << " o2: " << o2);
  vec3 o1o2 = o2 - o1;

  double s1;
  double s2;
  double d;

  s1 = (inner_prod(o1o2, u1) - inner_prod(o1o2, u2)*inner_prod(u1,u2)) / (1 - pow(inner_prod(u1,u2),2));
  s2 = (inner_prod(o1o2, u1)*inner_prod(u1,u2) - inner_prod(o1o2, u2)) / (1 - pow(inner_prod(u1,u2),2));
  d = norm_2( (o1+s1*u1) - (o2+s2*u2) );
  //JFR_DEBUG("s1: " << s1 << " s2: " << s2 << " d: " << d);
  return boost::make_tuple(s1,s2,d);
  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::pluckerLinesDistance()");
}

void ImageEuclideanPluckerFeatureObserveModel::computeSensorR(Observation const& obs) 
{
  SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs);
  mat J(2,4);
  extToLineJac(segObs.ext1, segObs.ext2, J);

  sym_mat extsCov(4,4);
  extsCov.clear();

  range r1(0,2);
  project(extsCov, r1,r1).assign(p_pixCov);

  range r2(2,4);
  project(extsCov, r2,r2).assign(p_pixCov);

  p_isCorrelated = true;
  R.assign(prod(J, mat(prod(extsCov, trans(J)))));
}

vec2 ImageEuclideanPluckerFeatureObserveModel::extToLine(vec const& ext1,vec const& ext2)
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

void ImageEuclideanPluckerFeatureObserveModel::extToLineJac(vec const& ext1,vec const& ext2, mat& J)
{
  JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
	      "ImageEuclideanPluckerFeatureObserveModel::extToLineJac");
  JFR_PRECOND(J.size1() == 2 && J.size2() == 4,
	      "ImageEuclideanPluckerFeatureObserveModel::extToLineJac");

  double u1 = ext1(0);
  double v1 = ext1(1);
  double u2 = ext2(0);
  double v2 = ext2(1);

  /* Begin Maple */

  double t1 = u2 * u2;
  double t4 = u1 * u1;
  double t5 = v2 * v2;
  double t8 = v1 * v1;
  double t9 = t1 - 0.2e1 * u2 * u1 + t4 + t5 - 0.2e1 * v2 * v1 + t8;
  double t10 = sqrt(t9);
  double t11 = 0.1e1 / t10;
  double t12 = u1 * t11;
  double t13 = u2 - u1;
  double t15 = v1 * t11;
  double t16 = v2 - v1;
  double t18 = -t12 * t13 - t15 * t16;
  double t19 = t18 * t11;
  double t21 = u1 + t19 * t13;
  double t22 = t21 * t21;
  double t24 = v1 + t19 * t16;
  double t25 = t24 * t24;
  double t27 = sqrt(t22 + t25);
  double t28 = 0.1e1 / t27;
  double t31 = 0.1e1 / t10 / t9;
  double t32 = u1 * t31;
  double t33 = -0.2e1 * t13 * t13;
  double t36 = v1 * t31;
  double t37 = -0.2e1 * t16 * t13;
  double t41 = (-t11 * t13 + t32 * t33 / 0.2e1 + t12 + t36 * t37 / 0.2e1) * t11;
  double t43 = t18 * t31;
  double t46 = 0.1e1 + t41 * t13 - t43 * t33 / 0.2e1 - t19;
  double t50 = t43 * t37 / 0.2e1;
  double t51 = t41 * t16 - t50;
  double t59 = -0.2e1 * t16 * t16;
  double t63 = (t32 * t37 / 0.2e1 - t11 * t16 + t36 * t59 / 0.2e1 + t15) * t11;
  double t65 = t63 * t13 - t50;
  double t70 = 0.1e1 + t63 * t16 - t43 * t59 / 0.2e1 - t19;
  double t75 = 0.2e1 * t13 * t13;
  double t78 = 0.2e1 * t16 * t13;
  double t82 = (t32 * t75 / 0.2e1 - t12 + t36 * t78 / 0.2e1) * t11;
  double t86 = t82 * t13 - t43 * t75 / 0.2e1 + t19;
  double t90 = t43 * t78 / 0.2e1;
  double t91 = t82 * t16 - t90;
  double t98 = 0.2e1 * t16 * t16;
  double t102 = (t32 * t78 / 0.2e1 + t36 * t98 / 0.2e1 - t15) * t11;
  double t104 = t102 * t13 - t90;
  double t109 = t102 * t16 - t43 * t98 / 0.2e1 + t19;
  double t114 = 0.1e1 / t21;
  double t116 = 0.1e1 / t22;
  double t117 = t24 * t116;
  double t122 = 0.1e1 / (0.1e1 + t25 * t116);
  J(0,0) = t28 * (t21 * t46 + t24 * t51);
  J(0,1) = t28 * (t21 * t65 + t24 * t70);
  J(0,2) = t28 * (t21 * t86 + t24 * t91);
  J(0,3) = t28 * (t21 * t104 + t24 * t109);
  J(1,0) = (t51 * t114 - t117 * t46) * t122;
  J(1,1) = (t70 * t114 - t117 * t65) * t122;
  J(1,2) = (t91 * t114 - t117 * t86) * t122;
  J(1,3) = (t109 * t114 - t117 * t104) * t122;
  /* End Maple */

}



void ImageEuclideanPluckerFeatureObserveModel::predictExtObs(vec_range const& pose, sym_mat_range const& poseCov,
						    vec3 const& ext, sym_mat const& extCov,
						    vec& zPredExt, sym_mat& zPredExtCov)
{
  JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
	      "ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND(ext.size() == 3 && extCov.size1() == 3,
	      "ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND(zPredExt.size() == 2 && zPredExtCov.size1() == 2,
	      "ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservation");

  JFR_TRACE_BEGIN;

  vec     extInSensor(3);
  sym_mat extInSensorCov(3,3);
  mat     Jframe(3,6);
  mat     Jpoint(3,3);
  mat     Jproj(2,3);

  EulerTools::toFrame(pose, ext, EulerTools::h_1, extInSensor);
  EulerTools::toFrameJac(pose, ext, EulerTools::h_1, Jframe, Jpoint);

  extInSensorCov.assign( prod(Jframe, mat(prod(poseCov, trans(Jframe)))) +
 			 prod(Jpoint, mat(prod(extCov, trans(Jpoint)))) );

  if (!robotToSensor().isIdentity()) {
      vec extInRobot(extInSensor);
      EulerTools::toFrame(robotToSensor().getX(), extInRobot, EulerTools::h_1, extInSensor);
      EulerTools::toFrameJac(robotToSensor().getX(), extInRobot, EulerTools::h_1, Jframe, Jpoint);
      extInSensorCov = prod(Jpoint, mat(prod(extInSensorCov, trans(Jpoint))));
  }

  camera.project(extInSensor, zPredExt);
  camera.projectJac(extInSensor, Jproj);
  zPredExtCov.assign( prod(Jproj, mat(prod(extInSensorCov, trans(Jproj)))) + getR() );

  // FIXME - use of slam::CameraPinhole
//   vec zPredExtTmp(zPredExt);
//   sym_mat zPredExtCovTmp(zPredExtCov);

//   zPredExt(0) = zPredExtTmp(1);
//   zPredExt(1) = zPredExtTmp(0);

//   zPredExtCov(0,0) = zPredExtCovTmp(1,1);
//   zPredExtCov(1,1) = zPredExtCovTmp(0,0);
//   zPredExtCov(0,1) = zPredExtCovTmp(0,1);

  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservation");
}

void ImageEuclideanPluckerFeatureObserveModel::predictExtObsJac(vec_range const& pose, vec3 const Ext, vec& eInImage, mat& JePose, mat& JeExt)
{
//  JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
//              "ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND( Ext.size() == 3, "ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
  
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
  
  JFR_TRACE_END("ImageEuclideanPluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
}


/*
*class ImagePluckerFeatureObserveModel
*/

ImagePluckerFeatureObserveModel::ImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_) :
		FeatureObserveModel(featureModel_, 2),
segmentFeatureModel(featureModel_),
camera(camera_),
Pl(),
Plinv(),
JbBeta(3,2),
p_pixCov(2,2)
{
	double alphaU_alphaV = camera.alphaU*camera.alphaV;
	
	Pl.clear();
	Pl(0,0) = camera.alphaV;
	Pl(1,1) = camera.alphaU;
	
	Pl(2,0) = -1.0*camera.alphaV*camera.u0;
	Pl(2,1) = -1.0*camera.alphaU*camera.v0;
	Pl(2,2) = alphaU_alphaV;
	//JFR_DEBUG("Pl:" << Pl);
	
	Plinv.clear();
	Plinv(0,0) = 1.0 / camera.alphaV;
	Plinv(1,1) = 1.0 / camera.alphaU;
	Plinv(2,0) = camera.u0 / alphaU_alphaV;
	Plinv(2,1) = camera.v0 / alphaU_alphaV;
	Plinv(2,2) = 1.0 / alphaU_alphaV;
	
	JbBeta.clear();
	
	p_pixCov.clear();
	
	// this part of the jacobian is always 0
	project(JobsSensor, range(0,2), range(3,6)).assign( zero_mat(2,3) );
	setup();
	
}

ImagePluckerFeatureObserveModel::~ImagePluckerFeatureObserveModel() {}

void ImagePluckerFeatureObserveModel::setPixCov(vec const& pixCov, double stabilizingFactor)
{
	JFR_PRECOND(pixCov.size()==2,
		    "ImageSegmentFeatureObserveModel::setPixCov:");
	
	p_pixCov.clear();
	p_pixCov(0,0) = stabilizingFactor*stabilizingFactor*pixCov(0);
	p_pixCov(1,1) = stabilizingFactor*stabilizingFactor*pixCov(1);
}





// Plane sub-vector 'a' and plane base to direction vector -- only output
vec3 ImagePluckerFeatureObserveModel::planeBaseToDirVector(vec3 const& an_, vec2 const&  beta_)
{
	return lineTools::planeBaseToDirVector(an_, beta_);
}

 

// Plane sub-vector a to plane base matrix -- output and Jacobians
void ImagePluckerFeatureObserveModel::planeBaseToDirVectorJac(vec_range const& an_, vec2 const&  beta_, vec_range& b, mat33& B_a)
{
	lineTools::planeBaseToDirVector(an_, beta_, b, B_a, JbBeta);
		
}
  


void ImagePluckerFeatureObserveModel::initInternalState(BaseFeature& baseFeature, 
		T3DEuler const& robotPose_, 
		Observation const& obs_) const
{
	JFR_TRACE_BEGIN;
	
	//JFR_DEBUG("obs:" << obs_);
	
	SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
	SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
	
	double s1,s2;
	boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);
	
	//JFR_DEBUG("s1: " << s1 << " s2: " << s2);
	
	if (s1 < s2) {
		segFeature.s1 = s1;
		segFeature.s2 = s2;
	}
	else {
		segFeature.s1 = s2;
		segFeature.s2 = s1;
	} 
	
	JFR_TRACE_END("ImageSegmentFeatureObserveModel::initInternalState()");
}

void ImagePluckerFeatureObserveModel::updateInternalState(BaseFeature& baseFeature, T3DEuler const& robotPose_, Observation const& obs_) const
{
    JFR_TRACE_BEGIN;
    SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
    SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs_);
	
	   //JFR_DEBUG("norm(u) = " << norm_2(segFeature.u()));
	
    double s1,s2;
    boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);
	
	   //JFR_DEBUG("s1: " << s1 << " s2: " << s2);

    if (!segFeature.ENLARGE) {
      // set extremitites to current ones
    //JFR_DEBUG("flagENLARGE " << segFeature.ENLARGE);
      if (s1 < s2) {
        segFeature.s1 = s1;
        segFeature.s2 = s2;
      }
      else {
        segFeature.s1 = s2;
        segFeature.s2 = s1;
      }
    } else {
      // only enlarge extremities
    //JFR_DEBUG("flagENLARGE " << segFeature.ENLARGE);
      if (s1 < s2) {
        if (s1 < segFeature.s1)
          segFeature.s1 = s1;
        if (s2 > segFeature.s2)
          segFeature.s2 = s2;
      }
      else {
        if (s2 < segFeature.s1)
          segFeature.s1 = s2;
        if (s1 > segFeature.s2)
          segFeature.s2 = s1;
      }
}
	
	JFR_TRACE_END("ImageSegmentFeatureObserveModel::updateInternalState()");
}

boost::tuple<double,double> ImagePluckerFeatureObserveModel::computeExtremitiesAbscissa(SegmentFeature const& segFeature_, T3DEuler const& robotPose_, SegmentObservation const& segObs) const
{
	JFR_TRACE_BEGIN;
	
	using namespace boost;
	
	T3DEuler sensorPose;
	T3D::compose(robotPose_, robotToSensor(), sensorPose);
	//JFR_DEBUG("robotToSensor:"<< robotToSensor());
	
	vec lineExt1Sensor(6);
	vec lineExt2Sensor(6);
	vec lineExt1(6);
	vec lineExt2(6);
	
	segmentExtremitiesPluckerLines(segObs, lineExt1Sensor, lineExt2Sensor);
	
	segFeature_.model.fromFrame(sensorPose.getX(), lineExt1Sensor, lineExt1);
	segFeature_.model.fromFrame(sensorPose.getX(), lineExt2Sensor, lineExt2);
	
//         //TAVC To debug if p0 is behind of the camera
//         vec line=segFeature_.getX();
//         line /= norm_2(project(line, range(3, 6)));
//         vec3 p0;
//         vec3 p0InSensor;
//         ublasExtra::crossProd(project(line, range(3, 6)), project(line, range(0, 3)), p0);
//         EulerTools::toFrame(sensorPose.getX(), p0, EulerTools::h_1, p0InSensor);
//         JFR_DEBUG("Punto p0 :" << p0InSensor(2) );

	double s1, s2;
	double tmp1, tmp2;
	
	tie(s1, tmp1, tmp2) =
			pluckerLinesDistance(segFeature_.getX(), lineExt1);
	
	tie(s2, tmp1, tmp2) =
			pluckerLinesDistance(segFeature_.getX(), lineExt2);
	
	return make_tuple(s1,s2);

	JFR_TRACE_END("ImageSegmentFeatureObserveModel::computeExtremitiesAbscissa()");
}

// 2 Plucker lines from principal point through both image segment extremities
void ImagePluckerFeatureObserveModel::segmentExtremitiesPluckerLines(SegmentObservation const& segObs_, vec& lineExt1Sensor,  vec& lineExt2Sensor) const
{
	JFR_PRECOND(lineExt1Sensor.size()==6 && lineExt2Sensor.size() == 6,
		    "ImageSegmentFeatureObserveModel::segmentExtremitiesPluckerLines: a plucker line is a 6-vector");
	lineExt1Sensor.clear();
	lineExt2Sensor.clear();
	
	vec ext1Sensor(3);
	camera.imageToCameraFrame(segObs_.ext1, ext1Sensor);
	vec ext2Sensor(3);
	camera.imageToCameraFrame(segObs_.ext2, ext2Sensor);

	project(lineExt1Sensor, range(3, 6)).assign(ext1Sensor/norm_2(ext1Sensor));
	project(lineExt2Sensor, range(3, 6)).assign(ext2Sensor/norm_2(ext2Sensor));
}

// abscissas of closest points of 1 plucker line to 2 other plucker lines
boost::tuple<double, double, double> ImagePluckerFeatureObserveModel::pluckerLinesDistance(vec const& line1_, vec const& line2_)
{
	JFR_PRECOND(line1_.size() == 6 && line2_.size() == 6, 
		    "ImageSegmentFeatureObserveModel::pluckerLinesDistance:");
	JFR_TRACE_BEGIN;
	
	vec line1 = line1_;
	vec line2 = line2_;
  
	line1 /= norm_2(project(line1, range(3, 6)));
	line2 /= norm_2(project(line2, range(3, 6)));
	
	vec3 u1 = project(line1, range(3, 6));
	vec3 u2 = project(line2, range(3, 6));
	vec3 o1;
	vec3 o2;
	//tavc?
	ublasExtra::crossProd(u1, project(line1, range(0, 3)), o1);
	ublasExtra::crossProd(u2, project(line2, range(0, 3)), o2);
	//JFR_DEBUG("o1 : " << o1 << " o2: " << o2);
	vec3 o1o2 = o2 - o1;
	
	double s1;
	double s2;
	double d;
	
	s1 = (inner_prod(o1o2, u1) - inner_prod(o1o2, u2)*inner_prod(u1,u2)) / (1 - pow(inner_prod(u1,u2),2));
	s2 = (inner_prod(o1o2, u1)*inner_prod(u1,u2) - inner_prod(o1o2, u2)) / (1 - pow(inner_prod(u1,u2),2));
	d = norm_2( (o1+s1*u1) - (o2+s2*u2) );
	//JFR_DEBUG("s1: " << s1 << " s2: " << s2 << " d: " << d);
 
	return boost::make_tuple(s1,s2,d);
	JFR_TRACE_END("ImagePluckerFeatureObserveModel::pluckerLinesDistance()");
}





void ImagePluckerFeatureObserveModel::predictExtObs(vec_range const& pose, sym_mat_range const& poseCov,vec3 const& ext, sym_mat const& extCov, vec& zPredExt, sym_mat& zPredExtCov)
{
	JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
                "ImagePluckerFeatureObserveModel::predictExtremitiesObservation");
	JFR_PRECOND(ext.size() == 3 && extCov.size1() == 3,
                "ImagePluckerFeatureObserveModel::predictExtremitiesObservation");
	JFR_PRECOND(zPredExt.size() == 2 && zPredExtCov.size1() == 2,
                "ImagePluckerFeatureObserveModel::predictExtremitiesObservation");
	
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
	
    JFR_TRACE_END("ImagePluckerFeatureObserveModel::predictExtremitiesObservation");
}

void ImagePluckerFeatureObserveModel::predictExtObsJac(vec_range const& pose, vec3 const Ext, vec& eInImage, mat& JePose, mat& JeExt)
{
//  JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
//              "ImageSegmentFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND( Ext.size() == 3, "ImagePluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
  
//  JFR_TRACE_BEGIN;
	 
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
  
//   JFR_TRACE_END("ImagePluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
}




void ImagePluckerFeatureObserveModel::postInitCovariance(sym_mat_range & P_LL, mat const& Jx_) {
	JFR_PRECOND(P_LL.size1() == 6,
		    "ImagePluckerFeatureObserveModel::postInitCovariance: invalid size");
	
	// Jacobian of line in C frame wrt beta
	mat LC_beta(6,2);
	LC_beta.clear();
	subrange(LC_beta,3,6,0,2) = JbBeta;
	
	// Jacobian of line in W frame wrt beta
	//   we compute L_beta = L_Lc * LC_beta with
	//	   L_Lc = Jx_
	//	   L_Lc = [L_ac  L_bc], that is, L_bc = L_Lc(1:6,4:6) <-- this is the subrange
	//     LC_beta = [0 ; JbBeta]
	//   so that L_beta = L_bc*JbBeta
	mat L_beta(prod(subrange(Jx_, 0,6, 3,6),JbBeta));
	
	// Contribution of beta to the line covariance
	P_LL += prod(mat(prod(L_beta, covar_beta)), trans(L_beta));
	//JFR_DEBUG("P_LL(beta) = " << prod(mat(prod(L_beta, covar_beta)), trans(L_beta)));
		
}


// Assign initial inverse Homogeneous part and its sigma. -- 
void ImagePluckerFeatureObserveModel::setup(double beta0, double beta1, double sigma_beta) {
JFR_PRECOND(beta0 >= 0 && beta1 >= 0 && sigma_beta > 0,  "ImagePluckerFeatureObserveModel::setup: invalid values");
	beta(0) = beta0;
	beta(1) = beta1;
	covar_beta.clear();
	covar_beta(0,0) = pow(sigma_beta,2);
	covar_beta(1,1) = pow(sigma_beta,2);
	
}



/*
*class RhoThetaImagePluckerObserveModel
*/

// Constructor
RhoThetaImagePluckerFeatureObserveModel::RhoThetaImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_) :
ImagePluckerFeatureObserveModel(featureModel_,camera_) {}


// Destructor
RhoThetaImagePluckerFeatureObserveModel::~RhoThetaImagePluckerFeatureObserveModel() {}


// homogeneous to Scaled Homogeneous conversion - 
vec const& RhoThetaImagePluckerFeatureObserveModel::homogeneousToImageModel(vec3 const& l_)
{
	z = lineTools::homogeneousToRhoTheta(l_);
	return z;
}



// homogeneous to Scaled Homogeneous Jacobians
void RhoThetaImagePluckerFeatureObserveModel::homogeneousToImageModelJac(vec3 const& l_,
        mat& Jl) const
{
    JFR_PRECOND(Jl.size1() == 2 && Jl.size2() == 3,
            "RhoThetaImagePluckerObserveModel::homogeneousToRhoThetaJac: invalid size of Jl");
    
	vec2 _z;
	lineTools::homogeneousToRhoTheta(l_, _z, Jl);    
}   



// Rho theta to homogeneous line transform
vec3 RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneous(vec const& rt_)
{
	JFR_PRECOND(rt_.size() == 2, 
				"RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  

	return lineTools::rhoThetaToHomogeneous(rt_);
    
    
}


// RhoTheta Homogeneous to homogeneous transform -- Jacobian
void RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneousJac(vec const& rt_, mat& HMrt)
{
	JFR_PRECOND(rt_.size() == 2, 
				"RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  

	vec3 hm;
	lineTools::rhoThetaToHomogeneous(rt_, hm, HMrt);
}



// RhoTheta Homogeneous to homogeneous transform -- Jacobian
void RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneousJac(vec const& rt_, vec3& hm, mat& HMrt)
{
	
	JFR_PRECOND(rt_.size() == 2, 
				"RhoThetaImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  

	lineTools::rhoThetaToHomogeneous(rt_, hm, HMrt);
}	




vec const& RhoThetaImagePluckerFeatureObserveModel::computeInnovation(vec const& z_,
        vec const& zPred)
{
    JFR_PRECOND(z_.size() == 2,
            "RhoThetaImagePluckerObserveModel::computeInnovation: size of z_ does not match");
    JFR_PRECOND(zPred.size() == 2,
            "RhoThetaImagePluckerObserveModel::computeInnovation: size of zPred_ does not match");
    

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



// Project line into camera frame and convert to Scaled Homogeneous - 
vec const& RhoThetaImagePluckerFeatureObserveModel::predictObservationInSensorFrame( vec const& feature_)
{
    JFR_TRACE_BEGIN;
    JFR_PRECOND(feature_.size() == 6,
            "RhoThetaImagePluckerObserveModel::predictObservationInSensorFrame: invalid size for feature");
		
	return homogeneousToImageModel(lineTools::projectPlucker(Pl, feature_));
    JFR_TRACE_END("RhoThetaImagePluckerObserveModel::predictObservationInSensorFrame()");
}



// Project line into camera frame and convert to Scaled Homogeneous - Jacobians
void RhoThetaImagePluckerFeatureObserveModel::predictObservationInSensorFrameJac( vec const& feature_) 
{
    
    JFR_PRECOND(feature_.size() == 6,
            "RhoThetaImagePluckerObserveModel::predictObservationInSensorFrame: invalid size for feature");
    
    mat Jl(2,3);
    homogeneousToImageModelJac(lineTools::projectPlucker(Pl, feature_), Jl );
    
    subrange(JobsSensor, 0,2, 0,3).assign( prod(Jl,Pl) );

    // assigning this part of the Jacobian matrix is done at construction of JobsSensor:
    // project(JobsSensor, range(0,2), range(3,6)).assign( zero_mat(2,3) );
}





// Build 2-endpoints observation covariances matrix
void RhoThetaImagePluckerFeatureObserveModel::computeSensorR(Observation const& obs) 
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




// Retro project line, result in camera frame.
vec RhoThetaImagePluckerFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
	
	vec3 l = imageModelToHomogeneous(obs_.z);
	vec L(6);
	L = lineTools::retroProjectPluckerFromBase(Plinv, l, beta);
	
    return L; 

}



// Retro project plucker line  - result in camera frame.
void RhoThetaImagePluckerFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
    
	//JinvObs.clear();
	
	JFR_PRECOND(obs_.size() == 2, "RhoThetaImagePluckerFeatureObserveModel::inverseObservationInSensorFrameJac:");  
	
    // Conversion to homogeneous coordinates in the plane 
    vec3 l; // Homogeneous coordinates of the line segment in the image
    mat  L_rt(3,2); // rhotheta to homogeneous jacobian
    imageModelToHomogeneousJac(obs_.z, l, L_rt); // Jacobian L_z = L_rt

	// retro projection from base values beta
    vec6 L; 
	mat L_l(6,3);
	mat L_beta(6,2);
	lineTools::retroProjectPluckerFromBase(Plinv, l, beta, L, L_l, L_beta);
	JinvObs = prod(L_l,L_rt);
	JbBeta  = subrange(L_beta, 3,6, 0,2);
}


// Endpoints to Scaled Homogeneous transform
vec2 RhoThetaImagePluckerFeatureObserveModel::extToImageModel(vec const& ext1,vec const& ext2)
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



// endpoints to Scaled Homogeneous transform -- Jacobians
void RhoThetaImagePluckerFeatureObserveModel::extToImageModelJac(vec const& ext1,vec const& ext2, mat& J)
{
    JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
            "RhoThetaImagePluckerObserveModel::extToImageModelJac");
    JFR_PRECOND(J.size1() == 2 && J.size2() == 4,
            "RhoThetaImagePluckerObserveModel::extToImageModelJac");
    
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





/*
*class ScaledHomogeneousImagePluckerFeatureObserveModel
*/

// Constructor
ScaledHomogeneousImagePluckerFeatureObserveModel::ScaledHomogeneousImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_) : ImagePluckerFeatureObserveModel(featureModel_,camera_) {}


// Destructor
ScaledHomogeneousImagePluckerFeatureObserveModel::~ScaledHomogeneousImagePluckerFeatureObserveModel() {}


// homogeneous to Scaled Homogeneous conversion - 
vec const& ScaledHomogeneousImagePluckerFeatureObserveModel::homogeneousToImageModel(vec3 const& l_)
{
	z = l_;
	
	return z;
}



// homogeneous to Scaled Homogeneous Jacobians
void ScaledHomogeneousImagePluckerFeatureObserveModel::homogeneousToImageModelJac(vec3 const& l_,
        mat& SH_l) const
{
    JFR_PRECOND(SH_l.size1() == 3 && SH_l.size2() == 3,
            "ScaledHomogeneousImagePluckerFeatureObserveModel::homogeneousToRhoThetaJac: invalid size of SH_l");
    
	SH_l = identity_mat(3);
}   



// Scaled Homogeneous to homogeneous line transform
vec3 ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneous(vec const& sh_)
{
	JFR_PRECOND(sh_.size() == 3, 
				"ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  
	return sh_;
}


// Scaled Homogeneous to homogeneous transform -- Jacobian
void ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneousJac(vec const& sh_, mat& HM_sh)
{
	JFR_PRECOND(sh_.size() == 3, 
				"ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  

	HM_sh = identity_mat(3);
}



// Scaled Homogeneous to homogeneous transform -- Jacobian
void ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneousJac(vec const& sh_, vec3& hm, mat& HM_sh)
{
	
	JFR_PRECOND(sh_.size() == 3, 
				"ScaledHomogeneousImagePluckerFeatureObserveModel::imageModelToHomogeneousJac:");  
	hm = sh_;
	HM_sh = identity_mat(3);
}	



// Innovation
vec const& ScaledHomogeneousImagePluckerFeatureObserveModel::computeInnovation(vec const& z_,
        vec const& zPred)
{
    JFR_PRECOND(z_.size() == 3,
            "ScaledHomogeneousImagePluckerFeatureObserveModel::computeInnovation: size of z_ does not match");
    JFR_PRECOND(zPred.size() == 3,
            "ScaledHomogeneousImagePluckerFeatureObserveModel::computeInnovation: size of zPred_ does not match");
    
    z = z_ - zPred;
    
    return z;
}



// Project line into camera frame and convert to Scaled Homogeneous - 
vec const& ScaledHomogeneousImagePluckerFeatureObserveModel::predictObservationInSensorFrame( vec const& feature_)
{
    JFR_TRACE_BEGIN;
    JFR_PRECOND(feature_.size() == 6,
            "ScaledHomogeneousImagePluckerFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");
		
	return homogeneousToImageModel(lineTools::projectPlucker(Pl, feature_));
    JFR_TRACE_END("ScaledHomogeneousImagePluckerFeatureObserveModel::predictObservationInSensorFrame()");
}



// Project line into camera frame and convert to Scaled Homogeneous - Jacobians
void ScaledHomogeneousImagePluckerFeatureObserveModel::predictObservationInSensorFrameJac( vec const& feature_) 
{
    
    JFR_PRECOND(feature_.size() == 6,
            "ScaledHomogeneousImagePluckerFeatureObserveModel::predictObservationInSensorFrame: invalid size for feature");
    
    mat SH_l(3,3);
    homogeneousToImageModelJac(lineTools::projectPlucker(Pl, feature_), SH_l );
    
    subrange(JobsSensor, 0,3, 0,3).assign( prod(SH_l,Pl) );

    // assigning this part of the Jacobian matrix is done at construction of JobsSensor:
    // subrange(JobsSensor, 0,3, 3,6).assign( zero_mat(3,3) );
}





// Build observation covariances matrix
void ScaledHomogeneousImagePluckerFeatureObserveModel::computeSensorR(Observation const& obs) 
{
    SegmentObservation const& segObs = dynamic_cast<SegmentObservation const&>(obs);
    mat J(3,4);
    extToImageModelJac(segObs.ext1, segObs.ext2, J);
    
    sym_mat extsCov(4,4);
    extsCov.clear();
    
    subrange(extsCov, 0,2, 0,2) = p_pixCov;
    subrange(extsCov, 2,4, 2,4) = p_pixCov;
    
    p_isCorrelated = true;
    R.assign(prod(J, mat(prod(extsCov, trans(J)))));
}




// Retro project line, result in camera frame.
vec ScaledHomogeneousImagePluckerFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {
	
	vec3 l = imageModelToHomogeneous(obs_.z);
	vec L(6);
	L = lineTools::retroProjectPluckerFromBase(Plinv, l, beta);
	
    return L; 

}



// Retro project plucker line  - result in camera frame.
void ScaledHomogeneousImagePluckerFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {
    
	//JinvObs.clear();
	
	JFR_PRECOND(obs_.size() == 3, "ScaledHomogeneousImagePluckerFeatureObserveModel::inverseObservationInSensorFrameJac:");  
	
    // Conversion to homogeneous coordinates in the plane 
    vec3 l; // Homogeneous coordinates of the line segment in the image
    mat  L_sh(3,2); // rhotheta to homogeneous jacobian
    imageModelToHomogeneousJac(obs_.z, l, L_sh); // Jacobian L_z = L_sh
    
	// retro projection from base values beta
    vec6 L; 
	mat L_l(6,3);
	mat L_beta(6,2);
	lineTools::retroProjectPluckerFromBase(Plinv, l, beta, L, L_l, L_beta);
	JinvObs = prod(L_l,L_sh);
	JbBeta  = subrange(L_beta, 3,6, 0,2);
}


// Endpoints to Scaled Homogeneous transform
vec3 ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModel(vec const& ext1, vec const& ext2)
{
	return lineTools::extToHomogeneous(ext1, ext2);
}



// Endpoints to Scaled Homogeneous transform
vec3 ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModel(vec const& ext1,vec const& ext2, vec3 const& ref)
{
	return lineTools::adjustVec3(lineTools::extToHomogeneous(ext1, ext2), ref);
}



// endpoints to Scaled Homogeneous transform -- Jacobians
void ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac(vec const& ext1,vec const& ext2, mat& H_e)
{
	JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
				"ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac");
	JFR_PRECOND(H_e.size1() == 3 && H_e.size2() == 4,
				"ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac");
	vec3 h; // Homogeneous line
	mat H_e1, H_e2; // Jacobians wrt endpoints
	
	lineTools::extToHomogeneous(ext1, ext2, h, H_e1, H_e2);
		
	subrange(H_e, 0,3, 0,2) = H_e1;
	subrange(H_e, 0,3, 2,4) = H_e2;	
}



// endpoints to Scaled Homogeneous transform -- Jacobians
void ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac(vec const& ext1, vec const& ext2, vec3 const& ref, mat& HM_e)
{
    JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
				"ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac");
    JFR_PRECOND(HM_e.size1() == 3 && HM_e.size2() == 4,
				"ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModelJac");
	
	vec3 h; // Homogeneous line before adjustment
	mat H_e1, H_e2; // Jacobians wrt endpoints
	
	lineTools::extToHomogeneous(ext1, ext2, h, H_e1, H_e2);
	
	vec3 hm; // adjusted line
	mat33 HM_h; // jacobian of adj. line wrt non-adj. line
	lineTools::adjustVec3(h, ref, hm, HM_h);
	
	subrange(HM_e, 0,3, 0,2) = prod(HM_h, H_e1);
	subrange(HM_e, 0,3, 2,4) = prod(HM_h, H_e2);	
	
}


/*
*class StereoImagePluckerFeatureObserveModel
*/
// two rho-thetha?
StereoImagePluckerFeatureObserveModel::StereoImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::StereoBench const& stereoBench_) :
    FeatureObserveModel(featureModel_, 4, 6), 
    segmentFeatureModel(featureModel_),
    stereoBench(stereoBench_),
    Pl(),
    Plinv(),
    p_pixCov(2,2)
{
    double alphaU_alphaV = stereoBench.leftCamera.alphaU*stereoBench.leftCamera.alphaV;
    
    Pl.clear();
    Pl(0,0) = stereoBench.leftCamera.alphaV;
    Pl(1,1) = stereoBench.leftCamera.alphaU;
    
    Pl(2,0) = -1.0*stereoBench.leftCamera.alphaV*stereoBench.leftCamera.u0;
    Pl(2,1) = -1.0*stereoBench.leftCamera.alphaU*stereoBench.leftCamera.v0;
    Pl(2,2) = alphaU_alphaV;
    //JFR_DEBUG("Pl:" << Pl);
    
    Plinv.clear();
    Plinv(0,0) = 1.0 / stereoBench.leftCamera.alphaV;
    Plinv(1,1) = 1.0 / stereoBench.leftCamera.alphaU;
    Plinv(2,0) = stereoBench.leftCamera.u0 / alphaU_alphaV;
    Plinv(2,1) = stereoBench.leftCamera.v0 / alphaU_alphaV;
    Plinv(2,2) = 1.0 / alphaU_alphaV;
    
    p_pixCov.clear();
	p_dispVar = 0;
    
    //this part of the jacobian is always 0
    project(JobsSensor, range(0,4), range(3,6)).assign( zero_mat(4,3) );   //2 rho-theta
	
}

StereoImagePluckerFeatureObserveModel::~StereoImagePluckerFeatureObserveModel() {}

void StereoImagePluckerFeatureObserveModel::setPixCov(vec const& pixCov, double stabilizingFactor)
{
    JFR_PRECOND(pixCov.size()==2,
            "StereoImageSegmentFeatureObserveModel::setPixCov:");
    
    p_pixCov.clear();
    p_pixCov(0,0) = stabilizingFactor*stabilizingFactor*pixCov(0);
    p_pixCov(1,1) = stabilizingFactor*stabilizingFactor*pixCov(1);
}

void StereoImagePluckerFeatureObserveModel::setDispVar(double dispVar, double stabilizingFactor)
{
	p_dispVar = stabilizingFactor*stabilizingFactor*dispVar;
}


void StereoImagePluckerFeatureObserveModel::initInternalState(BaseFeature& baseFeature, 
        T3DEuler const& robotPose_, 
        Observation const& obs_) const
{
    JFR_TRACE_BEGIN;
    
    //JFR_DEBUG("obs:" << obs_);
    
    SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
    StereoSegmentObservation const& segObs = dynamic_cast<StereoSegmentObservation const&>(obs_);
    
    double s1,s2;
    boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);
    
    JFR_DEBUG("s1: " << s1 << " s2: " << s2);
    
    if (s1 < s2) {
        segFeature.s1 = s1;
        segFeature.s2 = s2;
    }
    else {
        segFeature.s1 = s2;
        segFeature.s2 = s1;
    } 
    JFR_DEBUG( JFR_PP_VAR(segFeature.getExt1()) << JFR_PP_VAR(segFeature.getExt2()) << JFR_PP_VAR(segObs.ext1) << JFR_PP_VAR(segObs.ext2) );
    JFR_TRACE_END("StereoImageSegmentFeatureObserveModel::initInternalState()");
}

void StereoImagePluckerFeatureObserveModel::updateInternalState(BaseFeature& baseFeature, T3DEuler const& robotPose_, Observation const& obs_) const
{
    JFR_TRACE_BEGIN;
    SegmentFeature& segFeature = dynamic_cast<SegmentFeature&>(baseFeature);
    StereoSegmentObservation const& segObs = dynamic_cast<StereoSegmentObservation const&>(obs_);
    
       //JFR_DEBUG("norm(u) = " << norm_2(segFeature.u()));
    
    double s1,s2;
    boost::tie(s1,s2) = computeExtremitiesAbscissa(segFeature, robotPose_, segObs);
    
       JFR_DEBUG("s1: " << s1 << " s2: " << s2);

    if (!segFeature.ENLARGE) {
      //set extremitites to current ones
      //JFR_DEBUG("flagENLARGE " << segFeature.ENLARGE);
      if (s1 < s2) {
        segFeature.s1 = s1;
        segFeature.s2 = s2;
      }
      else {
        segFeature.s1 = s2;
        segFeature.s2 = s1;
      }
    } else {
      //only enlarge extremities
    //JFR_DEBUG("flagENLARGE " << segFeature.ENLARGE);
      if (s1 < s2) {
        if (s1 < segFeature.s1)
          segFeature.s1 = s1;
        if (s2 > segFeature.s2)
          segFeature.s2 = s2;
      }
      else {
        if (s2 < segFeature.s1)
          segFeature.s1 = s2;
        if (s1 > segFeature.s2)
          segFeature.s2 = s1;
      }
}
    
    JFR_TRACE_END("StereoImageSegmentFeatureObserveModel::updateInternalState()");
}

boost::tuple<double,double> StereoImagePluckerFeatureObserveModel::computeExtremitiesAbscissa(SegmentFeature const& segFeature_, T3DEuler const& robotPose_, StereoSegmentObservation const& segObs) const
{
    JFR_TRACE_BEGIN;
    
    using namespace boost;
    
    T3DEuler sensorPose;
    T3D::compose(robotPose_, robotToSensor(), sensorPose);
    //JFR_DEBUG("robotToSensor:"<< robotToSensor());
    
    vec lineExt1Sensor(6);
    vec lineExt2Sensor(6);
    vec lineExt1(6);
    vec lineExt2(6);
    
    segmentExtremitiesPluckerLines(segObs, lineExt1Sensor, lineExt2Sensor);
    
    segFeature_.model.fromFrame(sensorPose.getX(), lineExt1Sensor, lineExt1);
    segFeature_.model.fromFrame(sensorPose.getX(), lineExt2Sensor, lineExt2);
    
//         //TAVC To debug if p0 is behind of the camera
//         vec line=segFeature_.getX();
//         line /= norm_2(project(line, range(3, 6)));
//         vec3 p0;
//         vec3 p0InSensor;
//         ublasExtra::crossProd(project(line, range(3, 6)), project(line, range(0, 3)), p0);
//         EulerTools::toFrame(sensorPose.getX(), p0, EulerTools::h_1, p0InSensor);
//         JFR_DEBUG("Punto p0 :" << p0InSensor(2) );

    double s1, s2;
    double tmp1, tmp2;
    
    tie(s1, tmp1, tmp2) =
            pluckerLinesDistance(segFeature_.getX(), lineExt1);
    
    tie(s2, tmp1, tmp2) =
            pluckerLinesDistance(segFeature_.getX(), lineExt2);
    
    return make_tuple(s1,s2);

    JFR_TRACE_END("StereoImageSegmentFeatureObserveModel::computeExtremitiesAbscissa()");
}

// 2 Plucker lines from principal point through both image segment extremities
void StereoImagePluckerFeatureObserveModel::segmentExtremitiesPluckerLines(StereoSegmentObservation const& segObs_, vec& lineExt1Sensor,  vec& lineExt2Sensor) const
{
    JFR_PRECOND(lineExt1Sensor.size()==6 && lineExt2Sensor.size() == 6,
            "StereoImageSegmentFeatureObserveModel::segmentExtremitiesPluckerLines: a plucker line is a 6-vector");
    lineExt1Sensor.clear();
    lineExt2Sensor.clear();
    
    vec ext1Sensor(3);
    vec ext1(2);
    vec ext2(2);

    ext1 = subrange(segObs_.ext1,0,2);
    ext2 = subrange(segObs_.ext2,0,2);

    stereoBench.leftCamera.imageToCameraFrame(ext1, ext1Sensor);
    vec ext2Sensor(3);
    stereoBench.leftCamera.imageToCameraFrame(ext2, ext2Sensor);

    project(lineExt1Sensor, range(3, 6)).assign(ext1Sensor/norm_2(ext1Sensor));
    project(lineExt2Sensor, range(3, 6)).assign(ext2Sensor/norm_2(ext2Sensor));
}

// abscissas of closest points of 1 plucker line to 2 other plucker lines
boost::tuple<double, double, double> StereoImagePluckerFeatureObserveModel::pluckerLinesDistance(vec const& line1_, vec const& line2_)
{
    JFR_PRECOND(line1_.size() == 6 && line2_.size() == 6, 
            "StereoImageSegmentFeatureObserveModel::pluckerLinesDistance:");
    JFR_TRACE_BEGIN;
    
    vec line1 = line1_;
    vec line2 = line2_;
  
    line1 /= norm_2(project(line1, range(3, 6)));
    line2 /= norm_2(project(line2, range(3, 6)));
    
    vec3 u1 = project(line1, range(3, 6));
    vec3 u2 = project(line2, range(3, 6));
    vec3 o1;
    vec3 o2;
    //tavc?
    ublasExtra::crossProd(u1, project(line1, range(0, 3)), o1);
    ublasExtra::crossProd(u2, project(line2, range(0, 3)), o2);
    //JFR_DEBUG("o1 : " << o1 << " o2: " << o2);
    vec3 o1o2 = o2 - o1;
    
    double s1;
    double s2;
    double d;
    
    s1 = (inner_prod(o1o2, u1) - inner_prod(o1o2, u2)*inner_prod(u1,u2)) / (1 - pow(inner_prod(u1,u2),2));
    s2 = (inner_prod(o1o2, u1)*inner_prod(u1,u2) - inner_prod(o1o2, u2)) / (1 - pow(inner_prod(u1,u2),2));
    d = norm_2( (o1+s1*u1) - (o2+s2*u2) );
    //JFR_DEBUG("s1: " << s1 << " s2: " << s2 << " d: " << d);
 
    return boost::make_tuple(s1,s2,d);
    JFR_TRACE_END("StereoImagePluckerFeatureObserveModel::pluckerLinesDistance()");
}





void StereoImagePluckerFeatureObserveModel::predictExtObs(vec_range const& pose, sym_mat_range const& poseCov,vec3 const& ext, sym_mat const& extCov, vec& zPredExt, sym_mat& zPredExtCov)
{
    //predictExtObs for the two cameras zPredExt is uvd
    JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
                "StereoImagePluckerFeatureObserveModel::predictExtremitiesObservation");
    JFR_PRECOND(ext.size() == 3 && extCov.size1() == 3,
                "StereoImagePluckerFeatureObserveModel::predictExtremitiesObservation");
    JFR_PRECOND(zPredExt.size() == 3 && zPredExtCov.size1() == 3,
                "StereoImagePluckerFeatureObserveModel::predictExtremitiesObservation"); //?
    
    JFR_TRACE_BEGIN;
    
    vec extInSensor(3);
    vec extInImage(2);
    double disp;

    sym_mat extInSensorCov(3,3);

    mat ExtInSensor_frame(3,6);
    mat ExtInSensor_ext(3,3); //Jacobian extremitites in SensorFrame wrt extremitite in 3D
    mat ExtInImage_extInSensor(3,3);
    
    EulerTools::toFrame(pose, ext, EulerTools::h_1, extInSensor);
    EulerTools::toFrameJac(pose, ext, EulerTools::h_1, ExtInSensor_frame, ExtInSensor_ext);
    
    extInSensorCov.assign( prod(ExtInSensor_frame, mat(prod(poseCov, trans(ExtInSensor_frame)))) +
            prod(ExtInSensor_ext, mat(prod(extCov, trans(ExtInSensor_ext)))) );
    
    if (!robotToSensor().isIdentity()) {
        vec extInRobot(extInSensor);
        EulerTools::toFrame(robotToSensor().getX(), extInRobot, EulerTools::h_1, extInSensor);
        EulerTools::toFrameJac(robotToSensor().getX(), extInRobot, EulerTools::h_1, ExtInSensor_frame, ExtInSensor_ext);
        extInSensorCov = prod(ExtInSensor_ext, mat(prod(extInSensorCov, trans(ExtInSensor_ext))));
    }

    stereoBench.computePixDisp(extInSensor, extInImage, disp);
    stereoBench.computePixDispJac(extInSensor, ExtInImage_extInSensor);
    subrange(zPredExt,0,2).assign(extInImage);
    zPredExt(2) = disp;
    zPredExtCov.assign( prod(ExtInImage_extInSensor, mat(prod(extInSensorCov, trans(ExtInImage_extInSensor)))) +
            getR() );
    
    
    JFR_TRACE_END("StereoImagePluckerFeatureObserveModel::predictExtremitiesObservation");
}

void StereoImagePluckerFeatureObserveModel::predictExtObsJac(vec_range const& pose, vec3 const Ext, vec& extInImage, mat& ExtInImage_frame, mat& ExtInImage_ext)
{
  //predictExtObs for the two cameras zPredExt is uvd
 //JFR_PRECOND(pose.size() == sizeRobotPose() && poseCov.size1() == sizeRobotPose(),
 //            "ImageSegmentFeatureObserveModel::predictExtremitiesObservation");
  JFR_PRECOND( Ext.size() == 3, "StereoImagePluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
  
  JFR_TRACE_BEGIN;

  vec extInRobot(3);  
  mat ExtInRobot_frame(3,6);
  mat ExtInRobot_ext(3,3);

  vec extInSensor(3);
  mat ExtInSensor_frame(3,6);
  mat ExtInSensor_ext(3,3);
 
  vec pix(2);
  double disp;
  mat ExtInImage_extInSensor(3,3);
    
//geom::T3D::pointToFrameJac(T3DEuler(pose),Ext,extInRobot,JRPose,JRExt);
  EulerTools::toFrame(pose, Ext, EulerTools::h_1, extInRobot);
  EulerTools::toFrameJac(pose, Ext, EulerTools::h_1, ExtInRobot_frame, ExtInRobot_ext);
// JFR_DEBUG("Jacobians JRExt:" << JRExt);
// if (!robotToSensor().isIdentity()) {
//   geom::T3D::pointToFrameJac(T3DEuler(robotToSensor().getX()),extInRobot,extInSensor,JECC,JCR);
    EulerTools::toFrame(robotToSensor().getX(), extInRobot, EulerTools::h_1, extInSensor);
    EulerTools::toFrameJac(robotToSensor().getX(), extInRobot, EulerTools::h_1, ExtInSensor_frame, ExtInSensor_ext);
//   JFR_DEBUG("Jacobians JCR:" << JCR);
// }
  
  
  stereoBench.computePixDisp(extInSensor, pix, disp);
  stereoBench.computePixDispJac(extInSensor, ExtInImage_extInSensor); 
  //JFR_DEBUG("Jacobians ExtInImage_extInSensor:" << ExtInImage_extInSensor);
  
  subrange(extInImage,0,2) = pix;
  extInImage(2) = disp;
  
  ExtInImage_frame.assign(prod(ExtInImage_extInSensor,mat(prod(ExtInSensor_ext,ExtInRobot_frame))) );
  ExtInImage_ext.assign(prod(ExtInImage_extInSensor,mat(prod(ExtInSensor_ext,ExtInRobot_ext))));
  //JFR_DEBUG("Jacobians ExtInImage_ext" << ExtInImage_ext);
  
  JFR_TRACE_END("StereoImagePluckerFeatureObserveModel::predictExtremitiesObservationJacobians");
}

vec const& StereoImagePluckerFeatureObserveModel::computeInnovation(vec const& z_,
        vec const& zPred)
{
// new method
 vec4 inn = z_ - zPred;
 double innt = round(inn(1) / M_PI);
 vec4 zi;
  zi(1) = z_(1)-innt*M_PI;
 if ((int)innt%2!=0){
zi(0)=-1.0*z_(0);// change rho sign
 }else{
  zi(0) = z_(0);
 }
 
innt=round(inn(3)/M_PI);
zi(3)=z_(3)-innt*M_PI;
 if ((int)innt % 2 != 0) {
zi(2)=-1.0*z_(2);// change rho sign
 }else{
zi(2)=z_(2);
 }
 
 z = zi - zPred;
 //JFR_DEBUG("Innovation" << z);
 return z;

/*    JFR_PRECOND(z_.size() == 4,
            "RhoThetaImagePluckerObserveModel::computeInnovation: size of z_ does not match");
    JFR_PRECOND(zPred.size() == 4,
            "RhoThetaImagePluckerObserveModel::computeInnovation: size of zPred_ does not match");
    
    z = z_ - zPred;
    

    // PI rotation when origin is traversed
    if (z(1) > M_PI/2) {
        z(1) -= M_PI;
        z(0) = -1.0*z_(0) - zPred(0);
    }
    else if (z(1) < -M_PI/2) {
        z(1) += M_PI;
        z(0) = -1.0*z_(0) - zPred(0);
    }

      // PI rotation when origin is traversed
    if (z(3) > M_PI/2) {
        z(3) -= M_PI;
        z(2) = -1.0*z_(2) - zPred(2);
    }
    else if (z(3) < -M_PI/2) {
        z(3) += M_PI;
        z(2) = -1.0*z_(2) - zPred(2);
    }

    return z;*/
}



// Project line into camera frame and convert to Rho-Theta - 
vec const& StereoImagePluckerFeatureObserveModel::predictObservationInSensorFrame( vec const& feature_)
{
    JFR_TRACE_BEGIN;
    JFR_PRECOND(feature_.size() == 6,
            "StereoImagePluckerObserveModel::predictObservationInSensorFrame: invalid size for feature");
    
    //JFR_DEBUG( "feature_ = " << feature_ );
	// intrinsic vector
	vec4 k;
    k = stereoBench.leftCamera.getIntrinsicParams();
	// feature
	vec6 feat;
	feat = feature_;
	
	// project two homogeneous lines
	vec6 hmhm;
	hmhm = lineTools::projectPluckerToStereo(k, stereoBench.alpha, feat);
	//JFR_DEBUG( "hmhm = " << hmhm );
	// convert to rhotheta
	vec2 rt;
	vec3 hm;
	// left image
	hm = subrange(hmhm,0,3);
	rt = lineTools::homogeneousToRhoTheta(hm);
	subrange(z,0,2) = rt;
    //JFR_DEBUG( "hm = " << hm << " rt = " << rt );
	// right image
	hm = subrange(hmhm,3,6);
	rt = lineTools::homogeneousToRhoTheta(hm);
	subrange(z,2,4) = rt;
    //JFR_DEBUG( "hm = " << hm << " rt = " << rt );
	
    //JFR_DEBUG( "z = " << z );
	JFR_TRACE_END("StereoImagePluckerObserveModel::predictObservationInSensorFrame()");
	
	return z;
}



// Project line into camera frame and convert to RhoTheta - Jacobians
void StereoImagePluckerFeatureObserveModel::predictObservationInSensorFrameJac( vec const& feature_) 
{
    JFR_PRECOND(feature_.size() == 6,
            "StereoImagePluckerObserveModel::predictObservationInSensorFrame: invalid size for feature");
    
	// intrinsic vector
	vec4 k;
	k = stereoBench.leftCamera.getIntrinsicParams();
	
	// project line and get homogeneous images
	vec6 hmhm;
	mat66 HMHM_l;
	lineTools::projectPluckerToStereo(k, stereoBench.alpha, feature_, hmhm, HMHM_l);
	//JFR_DEBUG( JFR_PP_VAR( hmhm ) << JFR_PP_VAR( HMHM_l ) );
	// conversions to rho theta
	mat RTL_hml(2,3);
	mat RTR_hmr(2,3);
	vec2 rt;
	vec3 hm;
	// left line
	hm = subrange(hmhm,0,3);
	lineTools::homogeneousToRhoTheta(hm, rt, RTL_hml);
	subrange(z,0,2) = rt;
	// right line
	hm = subrange(hmhm,3,6);
	lineTools::homogeneousToRhoTheta(hm, rt, RTR_hmr);
	subrange(z,2,4) = rt;
	
	// full Jacobian
	subrange(JobsSensor, 0,2, 0,6) = prod(RTL_hml,subrange(HMHM_l, 0,3, 0,6));
	subrange(JobsSensor, 2,4, 0,6) = prod(RTL_hml,subrange(HMHM_l, 3,6, 0,6));	
	
	//JFR_DEBUG( "RTL_hml = " << RTL_hml << " RTR_hmr = " << RTR_hmr << " HMHM_l = " << HMHM_l << " JobsSensor = " << JobsSensor );
}





// Build 2-endpoints observation covariances matrix
void StereoImagePluckerFeatureObserveModel::computeSensorR(Observation const& obs) 
{
    StereoSegmentObservation const& segObs = dynamic_cast<StereoSegmentObservation const&>(obs);
	
		mat RR_uu(4,6); // Jac of 2 rho-thetas wrt 2 uvds
		vec6 uvds;
		subrange(uvds, 0,3) = segObs.ext1;
		subrange(uvds, 3,6) = segObs.ext2;
		
		lineTools::uvdsToStereoRhoTheta(uvds, RR_uu);
		
		sym_mat extsCov(6,6);
		extsCov.clear();		
		subrange(extsCov, 0,2, 0,2) = p_pixCov;
		extsCov(2,2) = p_dispVar;
		subrange(extsCov, 3,5, 3,5) = p_pixCov;
		extsCov(5,5) = p_dispVar;
		
		p_isCorrelated = true;
		//JFR_DEBUG( JFR_PP_VAR( uvds ) << JFR_PP_VAR( RR_uu ) << JFR_PP_VAR( extsCov ) );
		R.assign(prod(RR_uu, mat(prod(extsCov, trans(RR_uu)))));
		//JFR_DEBUG( JFR_PP_VAR( R ) );
}



void StereoImagePluckerFeatureObserveModel::computeSensorRInit(Observation const& obs)
{
    RInit = jblas::zero_mat( 6, 6);
	subrange(RInit, 0,2, 0,2) = p_pixCov;
	RInit(2,2) = p_dispVar;
	subrange(RInit, 3,5, 3,5) = p_pixCov;
	RInit(5,5) = p_dispVar;
    //JFR_DEBUG("RInit :" << RInit);
}



// Retro project line, result in camera frame.
vec StereoImagePluckerFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_) {

    StereoSegmentObservation const& segObs = dynamic_cast<StereoSegmentObservation const&>(obs_);
 //TODO with zInit two extremitites with disparity that are stored in the observation
    vec6 zInit;
	subrange(zInit,0,3) = segObs.ext1;
	subrange(zInit,3,6) = segObs.ext2;
    //JFR_DEBUG("zInit: " << zInit);

    vec4 k;
    k(0) = stereoBench.leftCamera.u0;
    k(1) = stereoBench.leftCamera.v0;
    k(2) = stereoBench.leftCamera.alphaU;
    k(3) = stereoBench.leftCamera.alphaV;
    //JFR_DEBUG(k );
    vec4 p = lineTools::uvdToHmPoint(subrange(zInit,0,3), k , stereoBench.alpha);
    vec4 q = lineTools::uvdToHmPoint(subrange(zInit,3,6), k , stereoBench.alpha);
    //JFR_DEBUG("p: " << p << " q: " << q);
    vec6 L;
    L = lineTools::hmPointsToPlucker(p,q);
    //JFR_DEBUG("L: " << L);
    return L; 

}


// Retro project plucker line  - result in camera frame.
void StereoImagePluckerFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& obs_) {

    StereoSegmentObservation const& segObs = dynamic_cast<StereoSegmentObservation const&>(obs_);
    //TODO zInit two extremitites with disparity that are stored in the observation
    JinvObs.clear();
    vec6 zInit;
	subrange(zInit,0,3) = segObs.ext1;
	subrange(zInit,3,6) = segObs.ext2;

    vec4 k;
    k(0) = stereoBench.leftCamera.u0;
    k(1) = stereoBench.leftCamera.v0;
    k(2) = stereoBench.leftCamera.alphaU;
    k(3) = stereoBench.leftCamera.alphaV;
    
    vec4 p;
    mat P_u1(4,3);
    mat Q_u2(4,3);
    lineTools::uvdToHmPoint(subrange(zInit,0,3), k , stereoBench.alpha, p, P_u1);
    vec4 q;
    lineTools::uvdToHmPoint(subrange(zInit,3,6), k , stereoBench.alpha, q, Q_u2);
    //JFR_DEBUG( "P_u1: " << P_u1 << " Q_u2: " << Q_u2 );
    vec6 L;
    mat L_p(6,4);
    mat L_q(6,4);
    lineTools::hmPointsToPlucker(p,q, L, L_p, L_q);
    //JFR_DEBUG("L: " << L << " L_p: " << L_p << " L_q: " << L_q);

    subrange(JinvObs,0,6,0,3).assign(prod(L_p,P_u1));
    subrange(JinvObs,0,6,3,6).assign(prod(L_q,Q_u2));
    //JFR_DEBUG( " JinvObs: " << JinvObs );
}


// Endpoints to Rho-Theta transform
vec2 StereoImagePluckerFeatureObserveModel::extToRhoTheta(vec const& ext1, vec const& ext2)
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



// endpoints to Rho-Theta transform -- Jacobians
void StereoImagePluckerFeatureObserveModel::extToRhoThetaJac(vec const& ext1,vec const& ext2, mat& J)
{
    JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2,
            "RhoThetaImagePluckerObserveModel::extToImageModelJac");
    JFR_PRECOND(J.size1() == 2 && J.size2() == 4,
            "RhoThetaImagePluckerObserveModel::extToImageModelJac");
    
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
    
}
