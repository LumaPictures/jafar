/* $Id$*/

#include <cmath>

#include "kernel/jafarException.hpp"

#include "jmath/ublasExtra.hpp"

#include "slam/eulerTools.hpp"
#include "slam/bearingOnlyFeature.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar;
using namespace jafar::jmath;
using namespace jafar::slam;

/*
 * class InitStateMember
 */

std::ostream& jafar::slam::operator <<(std::ostream& s, const InitStateMember& ism) {
  s << ism.sprtL << " " << ism.x << " " << ism.P;
  return s;
}

/*
 * class InitFeature
 */

InitFeature::InitFeature(unsigned int id, unsigned int robotId, std::size_t sizeRobotPose) :
  m_id(id), m_robotId(robotId),
  previousInitPose(sizeRobotPose),
  deltaPose(sizeRobotPose),
  deltaPoseCov(sizeRobotPose),
  baselineMax(0.0)
{
}

InitFeature::~InitFeature() 
{
  clearInit();
}

unsigned int InitFeature::getRefFrameIndex() const {
  JFR_PRECOND(!initObs.empty(),
	      "InitFeature::getRefFrameIndex");
  return initObs.begin()->first;
}

Observation const& InitFeature::getRefObservation() const {
  JFR_PRECOND(!initObs.empty(),
	      "InitFeature::getRefObservation");
  return *(initObs.begin()->second);
}

void InitFeature::addInitObservation(unsigned int frameIndex,
				     jblas::vec_range const& refPose,
				     Observation* obs) {
  frameIndexes.push_back(frameIndex);
  initObs[frameIndex] = obs;

  previousInitPose.assign(refPose);
  deltaPose.clear();
  deltaPoseCov.clear();
}

void InitFeature::removeInitObservation(unsigned int frameIndex) {
  InitObsType::iterator it = initObs.find(frameIndex);
  JFR_PRECOND(it != initObs.end(),
	      "InitObsType::removeInitObservation: feature: " << id()
	      << " invalid frameIndex: " << frameIndex);

  delete it->second;
  initObs.erase(it);
}

bool InitFeature::hasInitObservation(unsigned int frameIndex) const {
  InitObsType::const_iterator it = initObs.find(frameIndex);
  return it != initObs.end();
}

InitStateMember const& InitFeature::getBestInitStateMember() const {
  InitStateType::const_iterator it = initState.begin();
  InitStateType::const_iterator itBest = it;
  double sprtLMax = (**itBest).sprtL;

  while (it != initState.end()) {
    if ((**it).sprtL > sprtLMax) {
      itBest = it;
      sprtLMax = (**it).sprtL;
    }
    ++it;
  }
  return **itBest;
}

void InitFeature::clearInit() {
  for (InitObsType::iterator it = initObs.begin() ; it != initObs.end() ; ++it) {
      delete it->second;
  }
  initObs.clear();
  initState.clear();
  deltaPose.clear();
  deltaPoseCov.clear();
  baselineMax=0.0;
}

void InitFeature::clearInitStateZPred() {
  for (InitStateType::iterator it = initState.begin() ; it != initState.end() ; ++it) {
    (**it).zPred.clear();
  }
}

void InitFeature::normalizeInitState() {
  double sum = 0;
  for (InitStateType::iterator it = initState.begin() ;
       it != initState.end() ; it++) {
    sum += (*it)->w;
  }

  for (InitStateType::iterator it = initState.begin() ;
       it != initState.end() ; it++) {
    (*it)->w /= sum;
  }
}

void InitFeature::writeLogHeader(jafar::kernel::DataLogger& log) const
{
  log.writeComment("slam: InitFeature");
  log.writeLegend("nb gaussian members (0=initialized)");
}

void InitFeature::writeLogData(jafar::kernel::DataLogger& log) const
{
  log.writeData(initState.size());
}

std::ostream& jafar::slam::operator <<(std::ostream& s, const InitFeature& f_) {
  s << f_.id() << std::endl;
  s << " observations :" << std::endl;
  for (InitFeature::InitObsType::const_iterator it = f_.initObs.begin() ; it != f_.initObs.end() ; ++it) {
    s << it->first << ": " << *(it->second) << std::endl;
  }
  s << " init state :" << std::endl;
  for (InitFeature::InitStateType::const_iterator it = f_.initState.begin() ; it != f_.initState.end() ; it++) {
    s << **it << std::endl;
  }
  s << " baselineMax:" << f_.baselineMax;
  return s;
}

/*
 * class BearingOnlyFeatureObserveModel
 */

BearingOnlyFeatureObserveModel::BearingOnlyFeatureObserveModel(FeatureModel& model, std::size_t sizeObs_) :
  BaseFeatureObserveModel(model, sizeObs_)
{
  setup();
}

BearingOnlyFeatureObserveModel::~BearingOnlyFeatureObserveModel() {}

void BearingOnlyFeatureObserveModel::initState(InitFeature& feature_,
                                               Observation const& obs_,
                                               double dMin_, double dMax_)
{
  JFR_DEBUG("BearingOnlyFeatureObserveModel::initState: " << feature_.id());
  initStateInSensorFrame(feature_, obs_, dMin_, dMax_);
  JFR_DEBUG("BearingOnlyFeatureObserveModel::initState: " << feature_.initState.size() << " gaussians");

  if (!robotToSensor().isIdentity()) {
    // now the sum of gaussians is turned to the robot frame
    vec x(sizeState());
    sym_mat xCov(sizeState());
    for (InitFeature::InitStateType::iterator it = feature_.initState.begin() ;
	 it != feature_.initState.end() ; it++) {

      jmath::WeightedGaussianVector& gv = **it;
      x.assign(gv.x);
      xCov.assign(gv.P);

      //     JFR_DEBUG("(1) xiCov = " << gv.P);
      //     JFR_DEBUG("(1) det(xiCov) = " << ublasExtra::lu_det(gv.P));

      featureModel.fromFrame(robotToSensor().getX(), x, gv.x);

      featureModel.fromFrameJac(robotToSensor().getX(), x);
      gv.P.assign(prod(featureModel.Jx, mat(prod(xCov, trans(featureModel.Jx)))));
    
      if (robotToSensor().hasCov()) {
        gv.P.plus_assign( prod(featureModel.Jframe, mat(prod(robotToSensor().getXCov(), trans(featureModel.Jframe)))) );
      }
    

      //     JFR_DEBUG("(2) xiCov = " << gv.P);
      //     JFR_DEBUG("(2) det(xiCov) = " << ublasExtra::lu_det(gv.P));

    }
  }
}

double BearingOnlyFeatureObserveModel::kSigmaToBeta(double alpha, double kSigma) {
  double d = alpha*kSigma;
  return (1+d)/(1-d);
}

void BearingOnlyFeatureObserveModel::basicUniformGaussianSum(double min, double max, 
							     double sigma, double kSigma,
							     std::list<jafar::jmath::GaussianVector>& gaussianSum)
{
  JFR_PRECOND(min < max,
	      "BearingOnlyFeatureObserveModel::basicUniformGaussianSum:");
  
  gaussianSum.clear();
  vec x(1);
  sym_mat cov(1,1);
  cov(0,0) = pow(sigma,2);
  std::size_t nbGaussian = static_cast<std::size_t>(ceil((max-min)/(2*kSigma*sigma)));

  for(unsigned int i=0 ; i < nbGaussian ; ++i) {
    x(0)=min + (kSigma+2*kSigma*i)*sigma;
    gaussianSum.push_back(jmath::GaussianVector(x, cov) );
  }
}

void BearingOnlyFeatureObserveModel::geometricUniformGaussianSum(double alpha_, double beta_,
								 double sMin_, double sMax_,
								 std::list<jafar::jmath::WeightedGaussianVector>& gaussianSum)
{

  JFR_PRECOND(alpha_ > 0 && beta_ > 1,
	      "BearingOnlyFeatureObserveModel::geometricUniformGaussianSum: invalid parameters for geometric series"
	      << "alpha=" << alpha_ << " beta=" << beta_);
  JFR_PRECOND(sMin_ < sMax_,
	      "BearingOnlyFeatureObserveModel::geometricUniformGaussianSum: invalid range");

  gaussianSum.clear();
  vec si(1);
  sym_mat siCov(1,1);    
  si(0) = sMin_/(1+alpha_);
  double siStdDev = alpha_*si(0);
  siCov(0,0) = siStdDev*siStdDev;

  while (si(0)+siStdDev < sMax_) {
    gaussianSum.push_back(jmath::WeightedGaussianVector(si, siCov, 1.0) );
    //gaussianSum.push_back(jmath::WeightedGaussianVector(si, siCov, siStdDev) );
    si(0) *= beta_;
    siStdDev = alpha_ * si(0);
    siCov(0,0) = siStdDev*siStdDev;
  }
  siCov(0,0) = siStdDev*siStdDev;
  gaussianSum.push_back(jmath::WeightedGaussianVector(si, siCov, 1.0) );
  //gaussianSum.push_back(jmath::WeightedGaussianVector(si, siCov, siStdDev) );
}

void BearingOnlyFeatureObserveModel::boPointsGeometricGaussianSum(jblas::vec const& d_, jblas::sym_mat const& dCov_,
								  double alpha_, double beta_,
								  double sMin_, double sMax_,
								  InitFeature::InitStateType& gaussianSum)
{
  JFR_PRECOND(d_.size() == 3,
	      "BearingOnlyFeatureObserveModel::boPointsGaussianSum: invalid size of d_");
  JFR_PRECOND(dCov_.size1() == 3 && dCov_.size2() == 3,
	      "BearingOnlyFeatureObserveModel::boPointsGaussianSum: invalid size of dCov_");

  std::list<jmath::WeightedGaussianVector> gaussianSum1D;
  geometricUniformGaussianSum(alpha_, beta_, sMin_, sMax_, gaussianSum1D);

  double si;
  vec3 xi;
  sym_mat Pi(3,3);

  // d as a matrix
  mat D(3,1);
  D(0,0) = d_(0);
  D(1,0) = d_(1);
  D(2,0) = d_(2);

  for (std::list<jmath::WeightedGaussianVector>::const_iterator it = gaussianSum1D.begin() ;
       it != gaussianSum1D.end() ; ++it)
    {
      si = it->x(0);
      xi.assign( si * d_ );
      Pi.assign( prod( D, mat(prod(it->P, trans(D))) ) + si*si*dCov_ );
      gaussianSum.push_back(new InitStateMember(xi, Pi, it->w, *it, sizeObs()) );
    }
}

bool BearingOnlyFeatureObserveModel::doUpdateInitState(jblas::vec const& closestMemberState,
						       jblas::vec const& deltaPose) {

  using namespace jblas;
  JFR_TRACE_BEGIN;

  JFR_DEBUG("closestMemberState: " << closestMemberState 
 	    << " deltaPose: " << deltaPose);

  vec zPredPrev(sizeObs());
  vec zPredCur(sizeObs());
  
  vec closestMemberStateInSensor(closestMemberState);
  vec sensorDeltaPose(deltaPose);

  if (!robotToSensor().isIdentity()) {
    featureModel.toFrame(robotToSensor().getX(), closestMemberState, closestMemberStateInSensor);
    vec sensorToRobot(deltaPose.size());

    EulerTools::invFrame(robotToSensor().getX(), sensorToRobot);
    
    vec xTmp(deltaPose.size());
    EulerTools::composeFrame(sensorToRobot, deltaPose, xTmp);
    EulerTools::composeFrame(xTmp, robotToSensor().getX(), sensorDeltaPose);
  }

  vec closestMemberStateInCur(sizeState());

  // translations only are of interest
  vec deltaPoseTrans(sizeRobotPose());
  deltaPoseTrans.clear();
  project(deltaPoseTrans, range(0,3)).assign(project(sensorDeltaPose, range(0,3)));
  
  featureModel.toFrame(deltaPoseTrans, closestMemberStateInSensor, closestMemberStateInCur);

  JFR_DEBUG("doUpdateInitState: closestMemberStateInSensor:" << closestMemberStateInSensor);
   zPredPrev.assign(predictObservationInSensorFrame(closestMemberStateInSensor));
   JFR_DEBUG("doUpdateInitState: closestMemberStateInCur:" << closestMemberStateInCur);    
   zPredCur.assign(predictObservationInSensorFrame(closestMemberStateInCur));

  // FIXME: should call computeR() for non-constant noise model
  // must buil a psedo observation...
  // compuyeR(obs);
  sym_mat invR(sizeObs());
  ublasExtra::inv(getR(), invR);

  vec deltaZ = computeInnovation(zPredCur, zPredPrev);

  double d = inner_prod(deltaZ, prod(invR,deltaZ));

  JFR_DEBUG("doUpdateInitState: deltaZ: " << deltaZ << " R:" << R << " d: " << d);

  return d > m_doUpdateTh;

  JFR_TRACE_END("BearingOnlyFeatureObserveModel::doUpdateInitState");
}

/*
 * class InfFeatureObserveModel
 */

InfFeatureObserveModel::InfFeatureObserveModel(FeatureModel& model, std::size_t sizeObs_) :
  FeatureObserveModel(model, sizeObs_)
{}

double InfFeatureObserveModel::computeBaseline(jblas::vec const& poseRef, Observation const& obsRef, 
					       jblas::vec_range const& poseCur) const 
{
  JFR_TRACE_BEGIN;

  vec poseRefS(poseRef);
  vec poseCurS(poseCur);

  if (!robotToSensor().isIdentity()) {
    EulerTools::composeFrame(poseRef, robotToSensor().getX(), poseRefS);
    EulerTools::composeFrame(poseCur, robotToSensor().getX(), poseCurS);
  }

  vec poseRefSInv(sizeRobotPose());
  EulerTools::invFrame(poseRefS, poseRefSInv);

  vec poseRefToCur(sizeRobotPose());
  EulerTools::composeFrame(poseRefSInv, poseCurS, poseRefToCur);
    
  return computeBaselineInSensorFrame(project(poseRefToCur, range(0,3)), obsRef);
  JFR_TRACE_END("InfFeatureObserveModel::computeBaseline");
}

