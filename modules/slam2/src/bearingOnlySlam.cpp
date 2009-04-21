/* $Id$ */

#include "kernel/jafarException.hpp"
#include "kernel/jafarDebug.hpp"

#include "jmath/ublasExtra.hpp"

#include "geom/t3dEuler.hpp"

#include "slam/slamException.hpp"
#include "slam/bearingOnlySlam.hpp"
#include "slam/pointFeature.hpp"
#include "slam/segmentFeature.hpp"
#include "slam/robot.hpp"

using namespace boost;

using namespace ublas;
using namespace jblas;

using namespace jafar::jmath;
using namespace jafar::filter;
using namespace jafar::slam;

/*
 * class TrajectoryPoseRecord
 */

std::ostream& jafar::slam::operator<<(std::ostream& s, TrajectoryPoseRecord const& tpr) {
  s << "frame: " << tpr.frameIndex << " - robotId: " << tpr.robotId << " - traj: " << tpr.trajIndex;
  s << " - initFeatures: ";
  for (TrajectoryPoseRecord::FeaturesIdType::const_iterator it = tpr.initFeatures.begin() ; it != tpr.initFeatures.end() ; ++it) {
    s << *it << " ";
  }
  s << "- initObs: ";
  for (TrajectoryPoseRecord::FeaturesIdType::const_iterator it = tpr.obsFeatures.begin() ; it != tpr.obsFeatures.end() ; ++it) {
    s << *it << " ";
  }
  return s;
}

std::ostream& jafar::slam::operator <<(std::ostream& s, TrajectoryPoseRecordKey const& tprk)
{
  s << tprk.frameIndex << " " << tprk.robotId;
  return s;
}

bool jafar::slam::operator< (TrajectoryPoseRecord const& p1, TrajectoryPoseRecord const& p2) {
  if (p1.initFeatures.empty() && p2.initFeatures.empty()) {
    if (p1.obsFeatures.size() == p2.obsFeatures.size())
      return p1.distanceToPrevious < p2.distanceToPrevious;
    else 
      return p1.obsFeatures.size() < p2.obsFeatures.size();
  }
  else if (!p1.initFeatures.empty() && !p2.initFeatures.empty()) {
    return p1.frameIndex > p1.frameIndex;
  }
  else if (p1.initFeatures.empty())
    return true;
  else 
    return false;
}

bool jafar::slam::operator< (TrajectoryPoseRecordKey const& k1, TrajectoryPoseRecordKey const& k2)
{
  return k1.robotId < k2.robotId || ( k1.robotId == k2.robotId && k1.frameIndex < k2.frameIndex);
}

bool jafar::slam::operator== (TrajectoryPoseRecordKey const& k1, TrajectoryPoseRecordKey const& k2)
{
  return k1.robotId == k2.robotId && k1.frameIndex == k2.frameIndex;
}


/*
 * class BearingOnlySlam
 */

BearingOnlySlam::BearingOnlySlam(std::size_t sizeMax_,
                                 std::size_t sizeRobotState_,
                                 std::size_t sizeRobotPose_,
                                 std::size_t sizeTrajectory) :
  SlamEkf(sizeMax_, sizeRobotState_, sizeRobotPose_),
  m_sizeTrajectory(sizeTrajectory),
  trajectoryStateIndex(sizeTrajectory),
  boFeatureObserveModels(),
  infFeatureObserveModels(),
  initFeatures()
{
  setBoParam();
  addEventListener(*this);


}

BearingOnlySlam::~BearingOnlySlam() {
  for (BoFeatureObserveModelsContType::iterator it = boFeatureObserveModels.begin() ; it != boFeatureObserveModels.end() ; ++it) {
    delete it->second;
  }

  for (InfFeatureObserveModelsContType::iterator it = infFeatureObserveModels.begin() ; it != infFeatureObserveModels.end() ; ++it) {
    delete it->second;
  }

  for (InitFeaturesMapType::iterator it = initFeatures.begin() ; it != initFeatures.end() ; ++it) {
    delete it->second;
  }
}

void BearingOnlySlam::addRobot( BaseRobot* baseRobot)
{
  JFR_PRECOND( robotsMap.size() == 0, "BearingOnlySlam works with only one robot." );
  SlamEkf::addRobot( baseRobot );

  JFR_PRECOND( baseRobot->filterIndex() == 0, "For BearingOnlySlam the robot must be at the begining of the filter." );
  
  range r0(baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizePose());
  robotIdToTrajectoryIndex[ baseRobot->id() ] = trajectory.size();
  trajectory.push_back( project(filter.getX(), r0) );
  trajectoryCov.push_back( project(filter.getP(), r0, r0) );
  trajectoryStateIndex.push_back( baseRobot->filterIndex());
  trajectoryIsRobot.push_back(true);

  filter.softResize( m_sizeTrajectory*sizeRobotPose() + baseRobot->sizePose());
  
  for (std::size_t i = 1 ; i < m_sizeTrajectory + 1 ; ++i) {
    trajectoryEmptyIndexes.push_back(i );
    trajectoryStateIndex[i] = baseRobot->sizePose() + (i - 1)* sizeRobotPose();
    range ri(trajectoryStateIndex[i], trajectoryStateIndex[i] + sizeRobotPose());
    trajectory.push_back( project(filter.getX(), ri) );
    trajectoryCov.push_back( project(filter.getP(), ri, ri) );
    trajectoryIsRobot.push_back(false);
  }

}

void BearingOnlySlam::init(boost::posix_time::time_duration const& curTime)
{
  SlamEkf::init(curTime);
  filter.softResize( m_sizeTrajectory * sizeRobotPose());
  for (InitFeaturesMapType::iterator it = initFeatures.begin() ; it != initFeatures.end() ; ++it) {
    delete it->second;
  }
  initFeatures.clear();
  trajectoryPoseRecords.clear();
  trajectoryEmptyIndexes.clear();
  for (std::size_t i = 0 ; i < m_sizeTrajectory ; ++i) {
    if( !trajectoryIsRobot[i] )
    {
      trajectoryEmptyIndexes.push_back(i);
    }
  }
}

bool BearingOnlySlam::hasTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_) const {
  TrajectoryPoseRecordsType::const_iterator it = trajectoryPoseRecords.find(TrajectoryPoseRecordKey( frameIndex, robotId_) );
  return it != trajectoryPoseRecords.end();
}

TrajectoryPoseRecord& BearingOnlySlam::getTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_) {
  TrajectoryPoseRecordsType::iterator it = trajectoryPoseRecords.find(TrajectoryPoseRecordKey( frameIndex, robotId_) );
  JFR_PRECOND(it != trajectoryPoseRecords.end(),
	      "BearingOnlySlam::getTrajectoryPoseRecord: invalid frameIndex " << frameIndex << "robotId:" << robotId_);
  return it->second;
}

TrajectoryPoseRecord const& BearingOnlySlam::getTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_) const {
  TrajectoryPoseRecordsType::const_iterator it = trajectoryPoseRecords.find(TrajectoryPoseRecordKey( frameIndex, robotId_) );
  JFR_PRECOND(it != trajectoryPoseRecords.end(),
	      "BearingOnlySlam::getTrajectoryPoseRecord: invalid frameIndex " << frameIndex);
  return it->second;
}

jblas::vec_range const& BearingOnlySlam::getTrajectoryPose(unsigned int frameIndex, unsigned int robotId_ ) const {
  TrajectoryPoseRecord const& tpr = getTrajectoryPoseRecord( frameIndex, robotId_ );
  return trajectory[tpr.trajIndex];
}

void BearingOnlySlam::setBoSensor(BearingOnlyFeatureObserveModel* model, int sensorId) {
  JFR_PRECOND(model->sizeState1() == sizeRobotPose(),
	      "BearingOnlySlam::setBoSensor: invalid model");
  JFR_PRECOND(boFeatureObserveModels.find(sensorId) == boFeatureObserveModels.end(),
	      "BearingOnlySlam::setBoSensor: sensor already exists");

  boFeatureObserveModels[sensorId] = model;
}

void BearingOnlySlam::setBoSensor(BearingOnlyFeatureObserveModel* model, InfFeatureObserveModel* modelInf, int sensorId) {
  JFR_PRECOND(model->sizeState1() == sizeRobotPose() && modelInf->sizeState1() == sizeRobotPose(),
	      "BearingOnlySlam::setBoSensor: invalid model");
  JFR_PRECOND(model->sizeObs() == modelInf->sizeObs(),
	      "BearingOnlySlam::setBoSensor: invalid models");
  JFR_PRECOND(boFeatureObserveModels.find(sensorId) == boFeatureObserveModels.end(),
	      "BearingOnlySlam::setBoSensor: sensor already exists");

  boFeatureObserveModels[sensorId] = model;
  infFeatureObserveModels[sensorId] = modelInf;
}

void BearingOnlySlam::setRobotToSensor(jblas::vec const& robotToSensor, int sensorId) {
  getBoFeatureObserveModel(sensorId).setRobotToSensor(robotToSensor);
  if (hasInfFeatureObserveModel(sensorId)) {
    getInfFeatureObserveModel(sensorId).setRobotToSensor(robotToSensor);
  }
}

void BearingOnlySlam::setRobotToSensor(jblas::vec const& robotToSensor, jblas::vec const& robotToSensorStdDev,
                                       int sensorId) {
  getBoFeatureObserveModel(sensorId).setRobotToSensor(geom::T3DEuler(robotToSensor, robotToSensorStdDev));
  if (hasInfFeatureObserveModel(sensorId))
    getInfFeatureObserveModel(sensorId).setRobotToSensor(geom::T3DEuler(robotToSensor, robotToSensorStdDev));
}

void BearingOnlySlam::removeFeature(unsigned int id) {
  InitFeaturesMapType::iterator featureIt = initFeatures.find(id);

  if (featureIt != initFeatures.end()) {
    removeInitFeature(featureIt);
  }
  else {
    JFR_VDEBUG("BearingOnlySlam::removeFeature: initialised feature " << id);
    JFR_TRACE_BEGIN;
    SlamEkf::removeFeature(id);
    JFR_TRACE_END("BearingOnlySlam::removeFeature id:" << id);
  }

}

void BearingOnlySlam::removeInitFeature(unsigned int id) {
  InitFeaturesMapType::iterator it = initFeatures.find(id);
  JFR_PRECOND(it != initFeatures.end(),
	      "BearingOnlySlam::removeInitFeature: invalid id " << id);
  removeInitFeature(it);
}

void BearingOnlySlam::removeInitFeature(InitFeaturesMapType::iterator featureIt) {
  JFR_TRACE_BEGIN;
  JFR_DEBUG("BearingOnlySlam::removeInitFeature: " << featureIt->first);

  InitFeature& f = *(featureIt->second);
  updateTrajectoryWithRemovedFeature(f);

  // send event
  for(BoSlamEventListenersList::iterator it = boSlamEventListeners.begin() ; it != boSlamEventListeners.end() ; ++it) {
    (**it).removeTentativeLandmark(f);
  }
  delete featureIt->second;
  initFeatures.erase(featureIt);
  JFR_TRACE_END("BearingOnlySlam::removeInitFeature: updateTraj: ");
}

void BearingOnlySlam::updateTrajectoryWithRemovedFeature(InitFeature const& f) {
  // update trajectory pose records
  for (InitFeature::InitObsType::const_iterator it = f.initObs.begin() ; it != f.initObs.end() ; ++it) {
    unsigned int frameIndex = it->first;
    TrajectoryPoseRecordsType::iterator itTpr = trajectoryPoseRecords.find(TrajectoryPoseRecordKey(frameIndex, f.robotId()));
    JFR_ASSERT(itTpr != trajectoryPoseRecords.end(),
	       "BearingOnlySlam::removeFeature:" << frameIndex << " " << f.robotId());
    TrajectoryPoseRecord& tpr = itTpr->second;
    tpr.initFeatures.erase(f.id());
    tpr.obsFeatures.erase(f.id());
//     if (frameIndex != currentFrameIndex && tpr.initFeatures.empty() && tpr.obsFeatures.empty()) {
//       // this pose is useless, erase the record
//       // the current pose cannot be removed now
//       eraseTrajectoryPoseRecord(itTpr);
//     }
  }
}

void BearingOnlySlam::beginBrowseInitFeatures() {
  browseInitFeaturesIt = initFeatures.begin();
}
bool BearingOnlySlam::hasNextInitFeature() {
  return browseInitFeaturesIt != initFeatures.end();
}
InitFeature* BearingOnlySlam::nextInitFeature() {
  JFR_PRECOND(hasNextInitFeature(),
	      "BearingOnlySlam::nextInitFeature: no more feature");
  InitFeature* f = browseInitFeaturesIt->second;
  browseInitFeaturesIt++;
  return f;
}


void BearingOnlySlam::setBoParam(double dMin_, double dMax_, 
				 double sprtAlphaFA_,  double sprtAlphaMD_)
{
  dMin = dMin_;
  dMax = dMax_;
  sprtAlphaFA = sprtAlphaFA_;
  sprtAlphaMD = sprtAlphaMD_;

  logA = log( (1-sprtAlphaMD)/sprtAlphaFA );
  logB = log( sprtAlphaMD / (1-sprtAlphaFA) );
}

// void BearingOnlySlam::addLandmarkManually(BaseFeature::IdType id_, Observation::ObservationType featureType_,
// 					  jblas::vec const& featureState_, jblas::sym_mat const& featureStateCov_)
// {
//   JFR_TRACE_BEGIN;
//   JFR_DEBUG("addFeatureManually " << id_);
//   JFR_PRECOND(featureStateCov_.size1() == featureState_.size(),
// 	      "BearingOnlySlam::addFeatureManually: size of featureStateCov does not match");

//   FeatureModel& model = *featureModels[featureType_];

//   JFR_PRECOND(featureState_.size() == model.sizeState(),
// 	      "BearingOnlySlam::addFeatureManually: size of featureState does not match featureType");

//   BaseBearingOnlyFeature* feature_ptr = new BaseBearingOnlyFeature(id_, model);
//   BaseBearingOnlyFeature& feature(*feature_ptr);
//   allBearingOnlyFeatures[feature.id] = feature_ptr;
//   featuresMap[feature.id] = feature_ptr;
//   feature.isFullStateInit = true;
//   mapManager->setLandmarkState(feature);

//   // initialization of state
//   feature.getX().assign(featureState_);

//   // initialization of state covariance
//   feature.getP().assign(featureStateCov_);

//   // initialization of state cross covariance (to 0)
//   range r_f(feature.filterIndex, feature.filterIndex + feature.featureModel.sizeState());
//   range r1(0, feature.filterIndex);
//   sym_mat_range PcrossFeature1(filter.getP(), r_f, r1);
//   PcrossFeature1.assign(zero_mat(r_f.size(), r1.size()));

//   if (feature.filterIndex + feature.featureModel.sizeState() < filter.sizeState() ) {
//     range r2(feature.filterIndex + feature.featureModel.sizeState(), filter.sizeState());
//     sym_mat_range PcrossFeature2(filter.getP(), r2, r_f);
//     //      PcrossFeature2.assign( prod<mat>(model->Jframe, PcrossRobot2) );        //FIXME bug in ublas ??
//     PcrossFeature2.assign(zero_mat(r2.size(), r_f.size()));
//   }
//   JFR_DEBUG("feature added :" << feature);
//   JFR_TRACE_END("BearingOnlySlam::addLandmarkManually()");
// }


void BearingOnlySlam::observeNewFeature(Observation* obs) {
  if (hasBoFeatureObserveModel(obs->sensorId)) {
    JFR_DEBUG("BearingOnlySlam::observeFeature: new bearing-only feature (" << obs->id << ")");
    JFR_DEBUG(*obs);
    InitFeature* feature = new InitFeature(obs->id, obs->robotId, sizeRobotPose());
    initFeatures[feature->id()] = feature;
    //    if (p_masterLogger) logFeature(*feature);
    feature->addInitObservation(currentFrameIndex, refPose(obs->robotId), obs);
    getTrajectoryPoseRecord(currentFrameIndex, obs->robotId).initFeatures.insert(feature->id());
    initState(*feature, *obs);
  } else {
    JFR_TRACE_BEGIN;
    SlamEkf::observeNewFeature(obs);
    JFR_TRACE_END("BearingOnlySlam::observeNewFeature()");
  }
}


void BearingOnlySlam::observeKnownFeature(Observation* obs) {
  InitFeaturesMapType::iterator it = initFeatures.find(obs->id);
  if (it != initFeatures.end()) {
    observeInitFeature(*(it->second), obs);
  }
  else {
    if (hasBoFeatureObserveModel(obs->sensorId)) {
      JFR_TRACE_BEGIN;

      BaseFeature& feature = getFeature(obs->id);
      SlamEkf::observeKnownFeature(*obs, feature, getBoInfFeatureObserveModel(obs->sensorId, feature.atInfinity));

      delete obs;
      JFR_TRACE_END("BearingOnlySlam::observeKnownFeature()");
    }
    else {
      JFR_TRACE_BEGIN;
      SlamEkf::observeKnownFeature(obs);
      JFR_TRACE_END("BearingOnlySlam::observeKnownFeature()");
      return;
    }    
  }
}

void BearingOnlySlam::observeInitFeature(InitFeature& feature, Observation* obs) {

  JFR_DEBUG("BearingOnlySlam::observeInitFeature: " << obs->id);
//   JFR_DEBUG(*obs);

  unsigned int sensorId = obs->sensorId;
  BearingOnlyFeatureObserveModel& obsModel = getBoFeatureObserveModel(sensorId);

  // check if we update the initial state

  // get the closest hypothesis in the map frame
  jblas::vec closestHypothesis(obsModel.sizeState());
  obsModel.featureModel.fromFrame(getTrajectoryPose(feature.getRefFrameIndex()),
 				  feature.initState.front()->x, closestHypothesis);

  // get the closest hypothesis in the previous robot pose frame
  jblas::vec closestHypothesisInPrevFrame(obsModel.sizeState());
  obsModel.featureModel.toFrame(feature.previousInitPose,
				closestHypothesis, closestHypothesisInPrevFrame);

  bool doFeatureExist;
  bool isObsAdded = false;

  if(!obsModel.doUpdateInitState(closestHypothesisInPrevFrame, feature.deltaPose)) {
    feature.frameIndexes.push_back(currentFrameIndex);
    feature.clearInitStateZPred();
    doFeatureExist = true;
  }
  else {
    feature.addInitObservation(currentFrameIndex, refPose(feature.robotId()), obs);
    isObsAdded = true;
    getTrajectoryPoseRecord(currentFrameIndex, obs->robotId).obsFeatures.insert(feature.id());
  
    JFR_TRACE_BEGIN;
    updateInitState(feature, *obs);
    JFR_TRACE_END("BearingOnlySlam::observeKnownFeature id:" << obs->id);

    // manage the multi gaussian initState
    doFeatureExist = manageInitState(feature);
  }
  
  // make test for feature at infinity
  if (doFeatureExist && hasInfFeatureObserveModel(sensorId)) {
    InfFeatureObserveModel& infObsModel =  getInfFeatureObserveModel(sensorId);

    // update baseline
    double b;
    for(InitFeature::InitObsType::const_iterator it = feature.initObs.begin() ; it != feature.initObs.end() ; ++it) {
      b = infObsModel.computeBaseline(getTrajectoryPose(it->first), *(it->second), refPose(feature.robotId()));
      if (feature.baselineMax < b)
	feature.baselineMax = b;
    }

    if (feature.baselineMax > infObsModel.getBaselineTh()) {
      if (!isObsAdded) {
	feature.addInitObservation(currentFrameIndex, refPose(feature.robotId()), obs);
	getTrajectoryPoseRecord(currentFrameIndex, obs->robotId).obsFeatures.insert(feature.id());
	isObsAdded = true;
      }

      vec x(infObsModel.sizeState());
      sym_mat P(infObsModel.sizeState());

//      x.assign( infObsModel.inverseObservation(feature.getRefObservation().z) );
//      infObsModel.inverseObservationJac(feature.getRefObservation().z);
      x.assign( infObsModel.inverseObservation(feature.getRefObservation()) );
      infObsModel.inverseObservationJac(feature.getRefObservation());
      obsModel.computeR(feature.getRefObservation());
      P.assign( prod(infObsModel.JinvObs, mat(prod(obsModel.getR(), trans(infObsModel.JinvObs))) ) );

      fullStateInit(feature, x, P, true);
    }
  }

  if (!isObsAdded) delete obs;

}

void BearingOnlySlam::initState(InitFeature& feature, Observation const& obs) {
  BearingOnlyFeatureObserveModel& model = getBoFeatureObserveModel(obs.sensorId);
  model.initState(feature, obs, dMin, dMax);

  feature.normalizeInitState();
}


void BearingOnlySlam::updateInitState(InitFeature& feature, Observation const& obs) {

  InitFeature::InitStateType::iterator it = feature.initState.begin();

  // optimal observations likelihood
  double llMax1 = -1e6;
  InitFeature::InitStateType::iterator itMax1;
  // 2nd optimal observations likelihood
  double llMax2 = -1e6;

  BaseRobot* baseRobot = robot( obs.robotId );
  std::size_t refObsTrajIndex = getTrajectoryPoseRecord(feature.getRefFrameIndex(), obs.robotId).trajIndex;

  vec_range const& xi = trajectory[refObsTrajIndex];
  sym_mat_range const& Pi = trajectoryCov[refObsTrajIndex];
  vec_range const& xt = *baseRobot->refPose();
  sym_mat_range const& Pt = *baseRobot->refPoseCov();
//   sym_mat_range const& Pti = trajectoryCrossCov[refObsTrajIndex];
  range ri(trajectoryStateIndex[refObsTrajIndex], trajectoryStateIndex[refObsTrajIndex] + sizeRobotPose());
  range r0(baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizePose());
  sym_mat_range Pti = project(filter.getP(), r0, ri);

  BearingOnlyFeatureObserveModel& obsModel = getBoFeatureObserveModel(obs.sensorId);
  double m2PiN = pow(2*M_PI, (int)obsModel.sizeObs());

  // inverse of innovation covariance
  sym_mat S_inv(obsModel.sizeObs(), obsModel.sizeObs());
  mat Jxi(obsModel.sizeObs(), obsModel.sizeState1());

  obsModel.computeR(obs);

  {
    // update regular hypothesises

    // feature in map frame
    vec xf_m(obsModel.sizeState2());
    // because predictObservation functions take a range as parameter
    // FIXME ublas any_vector ??
    vec_range xf_m_r(xf_m, range(0, obsModel.sizeState2()));

    // h(xt, xi, xf) predict observation
    // jacobians of h
    mat_range Jxt(obsModel.Jobs1, range(0, obsModel.sizeObs()), range(0, obsModel.sizeState1()));
    mat Jxf(obsModel.sizeObs(), obsModel.sizeState2());

    while (it != feature.initState.end()) {
      InitStateMember& ism = **it;
      try {
	ism.l = computeInitStateHypothesisLogLikelihood(ism, feature, 
							obsModel, obsModel.eraseHypothesisTh(), 
							obs, obsModel.getR(), m2PiN,
							xi, Pi, xt, Pt, Pti,
							xf_m_r, S_inv, Jxt, Jxi, Jxf);
	if (ism.l > llMax1) {
	  llMax2 = llMax1;
	  llMax1 = ism.l;
	  itMax1 = it;
	}
	else if (ism.l > llMax2) {
	  llMax2 = ism.l;
	}
	++it;
      }
      catch(SlamException const& e) {
	if (e.getExceptionId() == SlamException::INVALID_INITSTATE_UPDATE) {
	  //JFR_WARNING(e);
	  delete *it;
	  it = feature.initState.erase(it);
	}
	else
	  throw;
      }
    }
  }

//   JFR_DEBUG("lMax1: " << lMax1 << " - lMax2: " << lMax2);


  // sprt log-likelihood
  for (InitFeature::InitStateType::iterator it = feature.initState.begin() ;
       it != feature.initState.end() ; ++it) {

    InitStateMember& ism = **it;

    ism.sprtL += ism.l;

    if (it != itMax1)
      ism.sprtL -= llMax1;
    else
      ism.sprtL -= llMax2;

    // compute gaussian sum weights
    // this is only for the display
    ism.w = (ism.sprtL - logB) / (logA - logB);

  }

}

// TODO: this method should be in observe model class
double BearingOnlySlam::computeInitStateHypothesisLogLikelihood(InitStateMember& ism,
								InitFeature& feature,
								BaseFeatureObserveModel& obsModel,
								double eraseHypothesisTh,
								Observation const& obs,
								jblas::sym_mat const& R,
								double m2PiN,
								jblas::vec_range const& xi, jblas::sym_mat_range const& Pi,
								jblas::vec_range const& xt, jblas::sym_mat_range const& Pt,
								jblas::sym_mat_range const& Pti,
								jblas::vec_range& xf_m_r,
								jblas::sym_mat& S_inv,
								jblas::mat_range& Jxt,
								jblas::mat& Jxi,
								jblas::mat& Jxf)
{
  // compute innovation y
  obsModel.featureModel.fromFrame(xi, ism.x, xf_m_r);
  ism.zPred.x.assign(obsModel.predictObservation(xt, xf_m_r));
  vec y = obsModel.computeInnovation(obs.z, ism.zPred.x);

  //  JFR_DEBUG("BearingOnlyFeatureObserveModel::updateInitState: y=" << y);

  // compute Jobs1 (Jxt) and Jobs2
  obsModel.predictObservationJac(xt, xf_m_r);

  // compute jacobians
  // compute Jframe and Jx
  obsModel.featureModel.fromFrameJac(xi, ism.x);

  Jxi.assign( prod(obsModel.Jobs2, obsModel.featureModel.Jframe) );
  Jxf.assign( prod(obsModel.Jobs2, obsModel.featureModel.Jx) );

  // compute innovation covariance
  ism.zPred.P.assign(prod(Jxi, mat(prod(Pi, trans(Jxi)))) +
		     prod(Jxt, mat(prod(Pt, trans(Jxt)))) +
		     sym_mat(prod(Jxt, mat(prod(Pti, trans(Jxi)))) +          // the sum of these two terms
			     prod(Jxi, mat(prod(trans(Pti), trans(Jxt))))) +  // is symmetric
		     prod(Jxf, mat(prod(ism.P, trans(Jxf) )) ) +
		     R );

  double det_S = jmath::ublasExtra::det(ism.zPred.P);
  JFR_PRED_ERROR(det_S > 0,
		 SlamException,
		 SlamException::INVALID_INITSTATE_UPDATE,
		 "invalid determinant det_S=" << det_S << " P=" << ism.zPred.P);

  jmath::ublasExtra::inv(ism.zPred.P, S_inv);

  double d = inner_prod(y, prod(S_inv, y));

  JFR_PRED_ERROR(d < eraseHypothesisTh,
		 SlamException,
		 SlamException::INVALID_INITSTATE_UPDATE,
		 "incoonsistent hypothesis d=" << d );

  return (-0.5 * d ) - log( sqrt(m2PiN * det_S) );
}

void BearingOnlySlam::eraseTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_)
{
  TrajectoryPoseRecordsType::iterator it = trajectoryPoseRecords.find(TrajectoryPoseRecordKey(frameIndex, robotId_));
  JFR_PRECOND(it != trajectoryPoseRecords.end(),
	      "BearingOnlySlam::eraseTrajectoryPoseRecord: invalid frameIndex " << frameIndex);
  eraseTrajectoryPoseRecord(it);
}

void BearingOnlySlam::eraseTrajectoryPoseRecord(TrajectoryPoseRecordsType::iterator itPose)
{
  JFR_TRACE_BEGIN;
  TrajectoryPoseRecord& poseToErase = itPose->second;
  JFR_DEBUG("BearingOnlySlam::eraseTrajectoryPoseRecord: " << poseToErase);

  for (TrajectoryPoseRecord::FeaturesIdType::const_iterator it = poseToErase.obsFeatures.begin() ; it != poseToErase.obsFeatures.end() ; ++it) {
    JFR_DEBUG("obsFeature: " << *it);
    getInitFeature(*it).removeInitObservation(poseToErase.frameIndex);
  }

  JFR_DEBUG("size initFeatures: " << poseToErase.initFeatures.size());
  while(!poseToErase.initFeatures.empty()) {
    TrajectoryPoseRecord::FeaturesIdType::const_iterator it = poseToErase.initFeatures.begin();
    JFR_DEBUG("initFeatures: " << *it);
    removeInitFeature(*it);
  }
  
  if ( !trajectoryIsRobot[ itPose->second.trajIndex ] ) {
    trajectoryEmptyIndexes.push_front(itPose->second.trajIndex);
  }
  trajectoryPoseRecords.erase(itPose);
  JFR_TRACE_END("BearingOnlySlam::eraseTrajectoryPoseRecord");
}

void BearingOnlySlam::manageTrajectory(unsigned int robotId_)
{
  JFR_TRACE_BEGIN;

  // garbage collection
  std::list<TrajectoryPoseRecordsType::iterator> toBeErased;
  for (TrajectoryPoseRecordsType::iterator it =  trajectoryPoseRecords.begin() ; it != trajectoryPoseRecords.end() ; ++it) {
    if (it->second.initFeatures.empty() && it->second.obsFeatures.empty()) {
      // this pose is of no interest
      toBeErased.push_back(it);
    }
  }
  for (std::list<TrajectoryPoseRecordsType::iterator>::iterator it = toBeErased.begin() ; it != toBeErased.end() ; ++it) {
    eraseTrajectoryPoseRecord(*it);
  }

  if (hasTrajectoryPoseRecord(currentFrameIndex, robotId_)) {
    // the current trajectory record has not been garbage collected...
    TrajectoryPoseRecord& currentPoseRecord = getTrajectoryPoseRecord(currentFrameIndex, robotId_);    
    // we try to backup this pose
    if (trajectoryEmptyIndexes.empty()) {
      
      // the less important trajectory pose is forgotten
      TrajectoryPoseRecordsType::iterator it =  trajectoryPoseRecords.begin();
      it->second.distanceToPrevious = 0;

      TrajectoryPoseRecordsType::iterator itPrev = it;
      TrajectoryPoseRecordsType::iterator itWorst = it;

      it++;
      while(it != trajectoryPoseRecords.end()) {
	it->second.distanceToPrevious = it->second.frameIndex - itPrev->second.frameIndex;
	if (it->second < itWorst->second)
	  itWorst = it;
	it++; itPrev++;
      }
 
      if (itWorst->second.frameIndex != currentFrameIndex) {
	// backup current pose over the worst pose
	eraseTrajectoryPoseRecord(itWorst);
	currentPoseRecord.trajIndex = trajectoryEmptyIndexes.front();
	trajectoryEmptyIndexes.pop_front();
	backupCurrentPose(currentPoseRecord.trajIndex, robotId_);
      }
      else {
	eraseTrajectoryPoseRecord(itWorst);
      }

    }
    else {
      // backup the pose in the first free slot
      currentPoseRecord.trajIndex = trajectoryEmptyIndexes.front();
      trajectoryEmptyIndexes.pop_front();
      backupCurrentPose(currentPoseRecord.trajIndex);
    }    
  }
  JFR_TRACE_END("BearingOnlySlam::manageTrajectory");
}

void BearingOnlySlam::backupCurrentPose(std::size_t index, unsigned int _robotId) 
{
  JFR_PRECOND(index >= 0 && index < sizeTrajectory(),
	      "BearingOnlySlam::backupCurrentPose: invalid index " << index);

  copyCurrentPoseAt(trajectoryStateIndex[index], _robotId);
}

void BearingOnlySlam::cleanInitFeatures() {
  nbInitFeaturesLost = 0;

  InitFeaturesMapType::iterator it = initFeatures.begin() ;
  while(it != initFeatures.end()) {
    if (it->second->frameIndexes.back() != currentFrameIndex) {
      JFR_DEBUG("BearingOnlySlam::cleanInitFeatures: lost feature " << it->first << " (no obsevation during tentative step)");
      ++nbInitFeaturesLost;
      removeInitFeature(it++);
    }
    else {
      ++it;
    }
  }
}

bool BearingOnlySlam::manageInitState(InitFeature& feature) {
  

  //  JFR_DEBUG("log(A): " << logA << " - log(B): " << logB);
  JFR_DEBUG("id: " << feature.id());

  InitFeature::InitStateType::iterator it = feature.initState.begin();
  while(it != feature.initState.end()) {
//     JFR_DEBUG("sprtL: " << (*it)->sprtL);

    if ((*it)->sprtL < logB) {
      delete (*it);
      it = feature.initState.erase(it);
    }
    else if ((*it)->sprtL > logA) {
      fullStateInit(feature, (**it));
      return false;
    }
    else {
      ++it;
    }
  }

  if(feature.initState.size() == 0) {
    JFR_WARNING("feature " << feature.id() << " removed");
    removeInitFeature(feature.id());
    return false;
  }

  if (feature.initState.size() == 1) {
    // we make the assumption this is the good hypothesis
    fullStateInit(feature, *feature.initState.front());
    return false;
  }

//   if (feature.initState.size() == 2) {
//     // we choose the best hypothesis
//     if (feature.initState.front()->sprtL > feature.initState.back()->sprtL)
//       fullStateInit(feature, *feature.initState.front());
//     else
//       fullStateInit(feature, *feature.initState.back());
//     return false;
//   }

  return true;
}

void BearingOnlySlam::fullStateInit(InitFeature& iFeature,
				    InitStateMember const& state) {
  fullStateInit(iFeature, state.x, state.P);
}

void BearingOnlySlam::fullStateInit(InitFeature& iFeature,
				    jblas::vec const& x,
				    jblas::sym_mat const& P,
				    bool atInfinity) {

  InitFeature::InitObsType::const_iterator itObs = iFeature.initObs.begin();
  JFR_ASSERT(itObs != iFeature.initObs.end(),
	     "BearingOnlySlam::fullStateInit: initObs is empty - feature " << iFeature.id());

  unsigned int refFrameIndex = itObs->first;
  Observation const& refObs = *(itObs->second);
  std::size_t refObsTrajIndex = getTrajectoryPoseRecord(refFrameIndex, iFeature.robotId()).trajIndex;
  BaseFeatureObserveModel& obsModel = getBoInfFeatureObserveModel(refObs.sensorId, atInfinity);

  BaseFeature* feature_ptr = featureFactory(iFeature.id(), obsModel);
  BaseFeature& feature = *feature_ptr;
  feature.atInfinity = atInfinity;

  // log it
  if (p_masterLogger) SlamEkf::logFeature(feature);

  // copy frameIndexes 
  feature.frameIndexes.insert(feature.frameIndexes.end(), iFeature.frameIndexes.begin(), iFeature.frameIndexes.end());

  mapManager->setMapObjectState(feature);

  JFR_VDEBUG("nb init obs: " << iFeature.initObs.size());
  JFR_DEBUG("state: " << x << " " << P);
  
  featuresMap[feature.id()] = feature_ptr;

  FeatureModel& model = obsModel.featureModel;

  model.fromFrame(trajectory[refObsTrajIndex], x, feature.getX());
  model.fromFrameJac(trajectory[refObsTrajIndex], x);
  feature.getP().assign( prod(model.Jframe, mat(prod(trajectoryCov[refObsTrajIndex], trans(model.Jframe)))) +
			 prod(model.Jx, mat(prod(P, trans(model.Jx))))
			 );
  //   JFR_DEBUG("x0 = " << feature.getX());
  //   JFR_DEBUG("x0Cov = " << feature.getP());
  //   JFR_DEBUG("det(x0Cov) = " << ublasExtra::lu_det(feature.getP()));

  range r_r(trajectoryStateIndex[refObsTrajIndex], trajectoryStateIndex[refObsTrajIndex] + sizeRobotPose());
  range r_f(feature.filterIndex(), feature.filterIndex() + model.sizeState());
  range r1(0, feature.filterIndex());
  sym_mat_range PcrossFirstObsPose1(filter.getP(), r_r, r1);
  sym_mat_range PcrossFeature1(filter.getP(), r_f, r1);

  PcrossFeature1.assign( prod(model.Jframe, PcrossFirstObsPose1) );

  if (feature.filterIndex() + model.sizeState() < filter.sizeState() ) {
    range r2(feature.filterIndex() + model.sizeState(), filter.sizeState());
    sym_mat_range PcrossRobot2(filter.getP(), r_r, r2);
    //      sym_mat_range PcrossFeature2(filter.P, r_f, r2); //FIXME bug in ublas ??
    sym_mat_range PcrossFeature2(filter.getP(), r2, r_f);
    //      PcrossFeature2.assign( prod<mat>(model->Jframe, PcrossRobot2) );        //FIXME bug in ublas ??
    PcrossFeature2.assign( trans(prod(model.Jframe, PcrossRobot2)) );
  }

  if(  trajectoryStateIndex[refObsTrajIndex] > 0 )
  {
    range r3( 0, trajectoryStateIndex[refObsTrajIndex]);
    sym_mat_range PcrossRobot3(filter.getP(), r_r, r3);
    sym_mat_range PcrossFeature3(filter.getP(), r3, r_f);
    PcrossFeature3.assign( trans(prod(model.Jframe, PcrossRobot3)) );
  }

  // FIXME dynamic size of observation !
  //   feature.zPred.resize(obsModel.sizeObs());

  // init internal state using the first observation
  obsModel.initInternalState(feature, geom::T3DEuler(refPose(iFeature.robotId())), refObs);
  
  // goto next observation
  ++itObs;

  // update with past observations
  // using the second obervation to the last one
  while(itObs != iFeature.initObs.end()) {

    Observation const& obs = *(itObs->second);
    std::size_t obsTrajIndex = getTrajectoryPoseRecord(itObs->first, iFeature.robotId()).trajIndex;

    JFR_DEBUG("past obs at frame " << itObs->first << ": " << obs);

    try {
      // FIXME dynamic size of observation !
      //     feature.zPred.resize(obs_.z.size());
      obsModel.computeR(obs);
      filter.update(obsModel,
		    trajectoryStateIndex[obsTrajIndex], feature.filterIndex(),
		    trajectory[obsTrajIndex], feature.getX(),
		    trajectoryCov[obsTrajIndex], feature.getP(),
		    obs.z,
		    feature.zPred);

      //       feature.applyConstraints();
      //      applyConstraints(feature);

    } catch(filter::InconsistentUpdateException const& e) {
      JFR_WARNING("BearingOnlySlam::fullStateInit: inconsistent observation discarded, feature: " << feature.id()
		  << "\n" << e);
      nbInconsistentUpdates++;
      if (e.mahalanobisDistance > mahalanobisDistanceToRemoveLandmark) {
	JFR_WARNING("BearingOnlySlam::fullStateInit: remove landmark");
	removeFeature(feature.id());
	return;
      }
    }

    // non stochastic state
    obsModel.updateInternalState(feature, geom::T3DEuler(trajectory[obsTrajIndex]), obs);

    ++itObs;

    //    JFR_DEBUG("x" << time << ": " << feature_->getX());
    //    JFR_DEBUG("P" << time << ": " << feature_->getP());
  }

  //   applyConstraints(feature);

  initFeatures.erase(iFeature.id());
  updateTrajectoryWithRemovedFeature(iFeature);
  delete &iFeature;

  JFR_VDEBUG("x: " << feature.getX());
  JFR_VDEBUG("P: " << feature.getP());
}

void BearingOnlySlam::predict(unsigned int id, 
			      jblas::vec const& u)
{
  // Slam prediction
  SlamEkf::predict(id, u);
  
  BaseRobot* robotBase = robot( id );

  // compute de delta poses for the initFeatures
  for (InitFeaturesMapType::iterator it = initFeatures.begin() ; it != initFeatures.end() ; ++it) {
    InitFeature& f = *(it->second);
    range rr(0, sizeRobotPose());
    range rc(0, robotBase->model.sizeCommand());
    vec_range deltaPose_r(f.deltaPose, rr);
    robotBase->model.predict(deltaPose_r, u);
    // FIXME: project on robotPose
    f.deltaPoseCov = prod(project(robotBase->model.F, rr, rr), mat(prod(f.deltaPoseCov, trans(project(robotBase->model.F, rr, rr))))) + 
      prod(project(robotBase->model.G, rr, rc), mat(prod(robotBase->model.getUCov(), trans(project(robotBase->model.G, rr, rc))))) + 
      project(robotBase->model.Q, rr, rr);
  }
}

void BearingOnlySlam::writeLogHeader(jafar::kernel::DataLogger& log) const {
  SlamEkf::writeLogHeader(log);
  log.writeComment("slam: BearingOnlySlam");
  log.writeLegend("number of trajectory poses");
  log.writeLegend("number of init features");
  log.writeLegend("number of init features lost");
}

void BearingOnlySlam::writeLogData(jafar::kernel::DataLogger& log) const {
  SlamEkf::writeLogData(log);
  log.writeData(trajectoryPoseRecords.size());
  log.writeData(initFeatures.size());
  log.writeData(nbInitFeaturesLost);
}

// void BearingOnlySlam::logFeature(InitFeature& f) {
//   JFR_PRECOND(p_masterLogger, "BearingOnlySlam::logFeature: masterLogger not defined !");
//   std::ostringstream os;
//   os << "initFeature_" << f.id() << ".dat";
//   JFR_DEBUG("log feature: " << os.str());
//   jafar::kernel::DataLogger* featureLog = new jafar::kernel::DataLogger(os.str());
//   featureLog->addLoggable(f);
//   p_masterLogger->addSlaveLogger(*featureLog);
// }

std::ostream& jafar::slam::operator <<(std::ostream& s, const BearingOnlySlam& slam_) {
  s << " * ref: " << slam_.refPose() << " - " << slam_.refPoseCov() << std::endl;
  s << " * map: " << std::endl;
  for (SlamEkf::FeaturesMapType::const_iterator it = slam_.featuresMap.begin() ;
       it != slam_.featuresMap.end() ; it++) {
    s << *(it->second) << std::endl;
  }
  s << " * tentative features: " << std::endl;
  for (BearingOnlySlam::InitFeaturesMapType::const_iterator it = slam_.initFeatures.begin() ;
       it != slam_.initFeatures.end() ; it++) {
    s << *(it->second) << std::endl;
  }
  s << " * trajectory: " << std::endl;
  for (BearingOnlySlam::TrajectoryPoseRecordsType::const_iterator it = slam_.trajectoryPoseRecords.begin() ; it != slam_.trajectoryPoseRecords.end() ; ++it) {

    s << it->second << std::endl;

//     std::size_t trajIndex = it->second.trajIndex;
//     s << "frame " << it->first << ": "
//       << slam_.trajectory[trajIndex] << " - "
//       << slam_.trajectoryCov[trajIndex] << " - "
//       << slam_.trajectoryCrossCov[trajIndex] << std::endl;
  }
  return s;
}

