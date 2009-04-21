/* $Id$ */

#include <list>

#include "kernel/jafarException.hpp"
#include "kernel/jafarDebug.hpp"

#include "jmath/ublasExtra.hpp"
#include "jmath/jblas.hpp"

#include "geom/t3dEuler.hpp"
#include "geom/t3dIdentity.hpp"

#include "slam/slamException.hpp"
#include "slam/eulerTools.hpp"
#include "slam/pointFeature.hpp"
#include "slam/segmentFeature.hpp"
#include "slam/slamEkf.hpp"
#include "slam/robot.hpp"

using namespace boost;

using namespace ublas;
using namespace jblas;

using namespace jafar::jmath;
using namespace jafar::filter;
using namespace jafar::slam;

/*
 * class SlamEkf
 */

SlamEkf::SlamEkf(std::size_t sizeMax_, std::size_t sizeRobotState_, std::size_t sizeRobotPose_) :
  m_deleteSensors( true ),
  filter(sizeMax_, 0),
  mahalanobisDistanceToRemoveLandmark(200.0),
  m_sizeRobotState(sizeRobotState_),
  m_sizeRobotPose(sizeRobotPose_),
  enableInitCoordinateFrame(false),
  nbInconsistentUpdates(0),
  nbNewLandmarks(0),
  nbObservedLandmarks(0),
  updateElapsedTime(0),
  p_masterLogger(0)
{
  mapManager = new DefaultMapManager(*this);
  // for 1-dimension update only !!
  filter.setupConsistencyCheck(BaseKalmanFilter::CONSISTENCY_EXCEPTION, 9.0);

  m_refToRobot = new geom::T3DIdentity();
  m_robotToRef = new geom::T3DIdentity();
}

SlamEkf::~SlamEkf() 
{
  delete mapManager;
  delete m_refToRobot;
  delete m_robotToRef;

  if( m_deleteSensors )
  {
    for (FeatureObserveModelsContType::iterator it = featureObserveModels.begin() ; it != featureObserveModels.end() ; ++it) {
      delete it->second;
    }
  }
  for (FeaturesMapType::iterator it = featuresMap.begin() ; it != featuresMap.end() ; ++it) {
      delete it->second;
    }
}

void SlamEkf::init(boost::posix_time::time_duration const& curTime)
{
  for (FeaturesMapType::iterator it = featuresMap.begin() ; it != featuresMap.end() ; ++it) {
    delete it->second;
  }
  featuresMap.clear();
  for (PoseCopyContType::iterator it = poseCopyCont.begin() ; it != poseCopyCont.end() ; ++it) {
    delete it->second;
  }
  poseCopyCont.clear();

  filter.init(curTime);
  filter.softResize(sizeRobotState());
//   if (!robotToRef().isIdentity()) {
//     robot(0)->refPose()->assign(robotToRef().getX());
//   }
  
  if (!robotToRef().isIdentity()) {
    for( RobotsMapType::iterator it = robotsMap.begin(); it != robotsMap.end(); ++it)
    {
      it->second->refPose()->assign(robotToRef().getX());
    }
  }
  
  mapManager->clear();
}

void SlamEkf::setRefToRobot(jafar::geom::T3DEuler const& refToRobot_) {
  JFR_PRECOND(!refToRobot_.hasCov(),
	      "SlamEkf::setRefToRobot: not supported");
  delete m_refToRobot;
  delete m_robotToRef;
  m_refToRobot = new geom::T3DEuler(refToRobot_);
  m_robotToRef = new geom::T3DEuler();
  geom::T3D::inv(*m_refToRobot, *m_robotToRef);
  
  // init refPose accordingly
  jblas::vec robotPose(6);
  robotPose.clear();
  setRobotPose(robotPose);
}

void SlamEkf::setRobotPose(jblas::vec const& robotPose, int _robotId) {
  JFR_PRECOND(robotPose.size() == 6,
	      "SlamEkf::setRobotPose: robotPose must be a 6-vector");
  BaseRobot* baseRobot = robot( _robotId );
  if (robotToRef().isIdentity()) {
    baseRobot->refPose()->assign(robotPose);
  }
  else {
    EulerTools::composeFrame(robotPose, robotToRef().getX(), *baseRobot->refPose());
  }
  baseRobot->refPoseCov()->assign(jblas::zero_mat(sizeRobotPose()));
}

void SlamEkf::setRobotPoseCov(jblas::vec const& robotPose, jblas::sym_mat const& robotPoseCov, int _robotId )
{
  JFR_PRECOND(robotPose.size() == 6,
              "SlamEkf::setRobotPose: robotPose must be a 6-vector");
  JFR_PRECOND(robotPoseCov.size1() == 6 and robotPoseCov.size2() == 6,
              "SlamEkf::setRobotPose: robotPoseCov must be a 6x6-matrix");
  BaseRobot* baseRobot = robot( _robotId );
  if (robotToRef().isIdentity()) {
    baseRobot->refPose()->assign(robotPose);
    baseRobot->refPoseCov()->assign(robotPoseCov);
  }
  else {
    EulerTools::composeFrame(robotPose, robotToRef().getX(), *baseRobot->refPose());
    JFR_RUN_TIME("Unimplemented");
  }
}

void SlamEkf::logFeature(BaseFeature& f) {
  JFR_PRECOND(p_masterLogger, "SlamEkf::logFeature: masterLogger not defined !");
  std::ostringstream os;
  os << "feature_" << f.id() << ".dat";
  //JFR_DEBUG("log feature: " << os.str());
  jafar::kernel::DataLogger* featureLog = new jafar::kernel::DataLogger(os.str());
  featureLog->addLoggable(f);
  p_masterLogger->addSlaveLogger(*featureLog);
}

void SlamEkf::setDeleteSensors( bool v)
{
  m_deleteSensors = v;
}

void SlamEkf::setSensor(FeatureObserveModel* model, int sensorId)
{
  JFR_PRECOND(model->sizeState1() == sizeRobotPose(),
	      "SlamEkf::setSensor: invalid model");
  JFR_PRECOND(featureObserveModels.find(sensorId) == featureObserveModels.end(),
	      "SlamEkf::setSensor: sensor already exists");

  featureObserveModels[sensorId] = model;
}

void SlamEkf::setMapManager(AbstractMapManager* mapManager_) 
{
  delete mapManager;
  mapManager = mapManager_;
}

void SlamEkf::removeFeature(unsigned int id)
{
  FeaturesMapType::iterator featureIt = featuresMap.find(id);
  JFR_PRED_ERROR(featureIt != featuresMap.end(), 
		 SlamException,
		 SlamException::UNKNOWN_FEATURE,
		 "SlamEkf::removeFeature: unknown feature " << id);

  JFR_VDEBUG("SlamEkf::removeFeature: " << featureIt->second->id());

  mapManager->removeMapObject(*(featureIt->second));

  for(SlamEventListenersList::iterator it = slamEventListeners.begin() ; it != slamEventListeners.end() ; ++it) {
    (**it).removeLandmark(*(featureIt->second));
  }

  delete featureIt->second;
  featuresMap.erase(featureIt); 
}

double SlamEkf::getRobotPoseUncertaintyLevel(unsigned int robotId_) const 
{
  return ublasExtra::det(ublas::project(refPoseCov(robotId_), ublas::range(0,3), ublas::range(0,3) ));
}

double SlamEkf::getMapUncertaintyLevel() const 
{
  double d=0;
  for (FeaturesMapType::const_iterator it = featuresMap.begin() ; it != featuresMap.end() ; it++) {
    d += ublasExtra::trace(it->second->getP());
  }
  return d;
}

PoseCopy const& SlamEkf::copyCurrentRefPoseToMap(unsigned int id) {
  //JFR_DEBUG("SlamEkf::copyCurrentPoseToMap: id: " << id);
  PoseCopyContType::iterator it = poseCopyCont.find(id);
  if(it != poseCopyCont.end()) {
    JFR_WARNING("SlamEkf::copyCurrentPoseToMap: copying local frame over - " << id);
    delete it->second;
  }

  PoseCopy* pose = new PoseCopy(sizeRobotPose(), id, getCurrentFrameIndex());
  mapManager->setMapObjectState(*pose);
  poseCopyCont[id] = pose;
  
  copyCurrentPoseAt(pose->filterIndex());

  //JFR_DEBUG("SlamEkf::copyCurrentPoseAt: pose:" << refPose() << " copy:" << pose->x() << " index:" << pose->filterIndex());

  return *pose;
}

void SlamEkf::copyCurrentPoseAt(std::size_t index, int _robotId) {
//   JFR_DEBUG("DEBUT");
  BaseRobot* baseRobot = robot( _robotId );

  // range of the robot pose
  ublas::range rr(baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizePose());
  // range of the copy
  ublas::range rc(index, index+sizeRobotPose());
  // copy the pose itself
  project(filter.getX(), rc).assign(*baseRobot->refPose());
//   JFR_DEBUG(*baseRobot->refPose());
//   JFR_DEBUG(project(filter.getX(), rc));


//   JFR_DEBUG( project( filter.getP(), rc, range(0, filter.sizeState()) ) );
//   JFR_DEBUG( project( filter.getP(), rr, range(0, filter.sizeState()) ) );
//   project( filter.getP(), rc, range(0, filter.sizeState())).assign( project( filter.getP(), rr, range(0, filter.sizeState())) );

    sym_mat_range robotPoseCrossCov(filter.getP(), rr, range(0, filter.sizeState()));
    sym_mat_range poseCopyCrossCov (filter.getP(), rc, range(0, filter.sizeState()));
//     poseCopyCrossCov.assign(robotPoseCrossCov);
  for(std::size_t i = 0; i < robotPoseCrossCov.size1(); ++i)
  {
    for(std::size_t j = 0; j < robotPoseCrossCov.size2(); ++j)
    {
      poseCopyCrossCov(i,j) = robotPoseCrossCov(i,j);
    }
  }

//   JFR_DEBUG( project( filter.getP(), rc, range(0, filter.sizeState()) ) );

  // cpoy the covariance
//   JFR_DEBUG( project( filter.getP(), rc,rc ) );
//   JFR_DEBUG( *baseRobot->refPoseCov() );
  project(filter.getP(), rc,rc).assign(*baseRobot->refPoseCov());
//   JFR_DEBUG( project( filter.getP(), rc,rc ) );

#if 0
  // cpoy the covariance
  project(filter.getP(), rc,rc).assign(*baseRobot->refPoseCov());
  //JFR_DEBUG(project(filter.getP(), rc,rc));
  //JFR_DEBUG(*baseRobot->refPoseCov());

  {
    // the cross covariance between the pose and its copy
//     sym_mat_range poseCopyCrossCov(filter.getP(), rc, rr);
//     poseCopyCrossCov.assign( *baseRobot->refPoseCov());
    project(filter.getP(), rc, rr).assign( *baseRobot->refPoseCov() );
    //JFR_DEBUG( project(filter.getP(), rc, rr) );
  }

  // cross covariance
  if (index > sizeRobotPose()) {
    sym_mat_range robotPoseCrossCov(filter.getP(), rr, range(sizeRobotPose(), index));
    sym_mat_range poseCopyCrossCov (filter.getP(), rc, range(sizeRobotPose(), index));
    poseCopyCrossCov.assign(robotPoseCrossCov);
    //JFR_DEBUG( poseCopyCrossCov );
  }

  if (index+sizeRobotPose() < filter.sizeState()) {
    sym_mat_range robotPoseCrossCov(filter.getP(), range(index+sizeRobotPose(), filter.sizeState()), rr);
    sym_mat_range poseCopyCrossCov (filter.getP(), range(index+sizeRobotPose(), filter.sizeState()), rc);
    poseCopyCrossCov.assign(robotPoseCrossCov);
    //JFR_DEBUG( poseCopyCrossCov );
  }
#endif
}

void SlamEkf::removePoseCopy(unsigned int id) {
  PoseCopyContType::iterator poseIt = poseCopyCont.find(id);
  JFR_PRED_ERROR(poseIt != poseCopyCont.end(),
		 SlamException,
		 SlamException::UNKNOWN_POSE_COPY,
		 "SlamEkf::removePoseCopy: unknown pose copy " << id);

  mapManager->removeMapObject(*(poseIt->second));

  delete poseIt->second;
  poseCopyCont.erase(poseIt);
}

void SlamEkf::writePoseCopyToFile(PoseCopy const& pc, std::string const& dir)
{
  geom::T3DEuler t3d(pc.x(), pc.xCov());
  std::stringstream ss;
  ss << dir << "/index_" 
     << std::setw(4) << std::setfill('0') << getCurrentFrameIndex() 
     << "_frame_"
     << std::setw(2) << std::setfill('0') << pc.id() << ".t3d";
  t3d.save(ss.str());
  //JFR_DEBUG("Save t3d: " << t3d << "in: " << ss.str());
}

void SlamEkf::writePoseCopyToFile(unsigned int id, std::string const& dir)
{
  PoseCopyContType::const_iterator it = poseCopyCont.find(id);
  JFR_PRED_ERROR(it != poseCopyCont.end(), 
		 SlamException,
		 SlamException::UNKNOWN_POSE_COPY,
		 "SlamEkf::writePoseCopyToFile: unknown pose copy " << id);

  writePoseCopyToFile(*(it->second), dir);
}

void SlamEkf::writeAllPoseCopyToFile(std::string const& dir)
{
  for(PoseCopyContType::const_iterator it = poseCopyCont.begin() ; it != poseCopyCont.end() ; ++it) {
    writePoseCopyToFile(*(it->second), dir);
  }
}

void SlamEkf::beginBrowseFeatures() {
  browseFeaturesIt = featuresMap.begin();
}

bool SlamEkf::hasNextFeature() {
  return browseFeaturesIt != featuresMap.end();
}

BaseFeature* SlamEkf::nextFeature() {
  JFR_PRECOND(hasNextFeature(),
	      "SlamEkf::nextFeature: no more feature");
  BaseFeature* f = browseFeaturesIt->second;
  browseFeaturesIt++;
  return f;
}

BaseFeature* SlamEkf::feature( unsigned int id )
{
  FeaturesMapType::iterator it = featuresMap.find( id );
  return it->second;
}


// void SlamEkf::setRobotState(jblas::vec const& state_) {
//   JFR_PRECOND(state_.size()==sizeRobotState(), 
//               "SlamEkf::setRobotState: size of state_ is invalid");
//   robotState.assign(state_);
//   robotStateCov.assign(zero_mat(robotState.size()));
// }

// void SlamEkf::setRobotState(jblas::vec const& state_, jblas::vec const& stateCov_) {
//   JFR_PRECOND(state_.size()==sizeRobotState(), 
// 	      "SlamEkf::setRobotState: size of state_ is invalid");
//   JFR_PRECOND(stateCov_.size()==sizeRobotState(), 
// 	      "SlamEkf::setRobotState: size of stateCov_ is invalid");
//   robotState.assign(state_);
//   robotStateCov.assign(zero_mat(robotState.size()));
//   for (std::size_t i = 0 ; i < sizeRobotState() ; ++i) {
//     robotStateCov(i,i) = stateCov_(i);
//   }
// }

void SlamEkf::setRobotToSensor(jblas::vec const& robotToSensor, int sensorId) {
  getFeatureObserveModel(sensorId).setRobotToSensor(robotToSensor);
}

const jblas::vec& SlamEkf::robotToSensor( int sensorId ) const
{
  return getFeatureObserveModel(sensorId).robotToSensor().getX();
}


void SlamEkf::setRobotToSensor(jblas::vec const& robotToSensor, jblas::vec const& robotToSensorStdDev, 
			       int sensorId)
{
  getFeatureObserveModel(sensorId).setRobotToSensor(geom::T3DEuler(robotToSensor, robotToSensorStdDev));
}

// void SlamEkf::addLandmarkManually(BaseFeature::IdType id_, Observation::ObservationType featureType_, 
// 				  jblas::vec const& featureState_, jblas::sym_mat const& featureStateCov_) 
// {
//   JFR_PRECOND(featureStateCov_.size1() == featureState_.size(),
// 	      "SlamEkf::addFeatureManually: size of featureStateCov does not match");
  
//   FeatureModel& model = *featureModels[featureType_];

//   JFR_PRECOND(featureState_.size() == model.sizeState(),
// 	      "SlamEkf::addFeatureManually: size of fatureState does not match featureType");

//   BaseFeature* feature_ptr = new BaseFeature(id_, model);
//   BaseFeature& feature = *feature_ptr;

//   mapManager->setLandmarkState(feature);

//   featuresMap[feature.id] = feature_ptr;
  
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
// }

void SlamEkf::observeNewFeature(Observation* obs)
{
//   JFR_TRACE_BEGIN;
  //JFR_DEBUG("SlamEkf::observeFeature: new feature (" << obs->id << ")");
  JFR_DEBUG("OBS: " << *obs);
  //JFR_DEBUG("sensorId: " << obs->sensorId);
  FeatureObserveModel& model = getFeatureObserveModel(obs->sensorId);
  //JFR_DEBUG("model ");
  BaseFeature* feature = featureFactory(obs->id, model);
  mapManager->setMapObjectState(*feature);
  featuresMap[feature->id()] = feature;
  if (p_masterLogger) logFeature(*feature);
  feature->addObservation(currentFrameIndex);
  JFR_DEBUG( " feature.getX() =  " << feature->getX() << " feature.getP() =  " << feature->getP() );

  try {
    // stochastic state initialization
    //JFR_DEBUG("Init Feature State");
    initFeatureState(*feature, model, *obs);
    JFR_DEBUG( " feature.getX() =  " << feature->getX() << " feature.getP() =  " << feature->getP() );
          
    // non stochastic state update
    model.initInternalState(*feature, geom::T3DEuler(refPose(obs->robotId), refPoseCov(obs->robotId)), *obs);
    JFR_DEBUG( " feature.getX() =  " << feature->getX() << " feature.getP() =  " << feature->getP() );
    for(SlamEventListenersList::iterator it = slamEventListeners.begin() ; it != slamEventListeners.end() ; ++it) {
      (**it).addLandmark(*feature);
    }
      
    nbNewLandmarks++;
  }
 catch(kernel::JafarException const& e) {
    if (e.getExceptionId() == kernel::JafarException::NUMERIC) {
      removeFeature(obs->id);
      JFR_WARNING("SlamEkf::observeNewFeature: feature ignored:\n" << e);
    }
    else
      throw;
  }
//   JFR_TRACE_END("SlamEkf::observeNewFeature: id:" << obs->id);

  delete obs;
}

BaseFeature* SlamEkf::featureFactory(unsigned int id,
             FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs )
{
  switch(typeObs) {
  case Observation::POINT_CARTESIAN:
  case Observation::POINT_POLAR:
  case Observation::POINT_STEREOIMAGE:
  case Observation::POINT_BEARING:
  case Observation::POINT_IMAGE:
  case Observation::POINT_OMNIIMAGE:
  case Observation::BASIS:
    return new BaseFeature(id, model, sizeObs, typeObs);
  case Observation::SEGMENT_IMAGE:
    return new SegmentFeature(id, model, sizeObs, typeObs);
  case Observation::SEGMENT_STEREOIMAGE:
    return new SegmentFeature(id, model, sizeObs, typeObs);
  case Observation::SEGMENTID_EXT_IMAGE:
  case Observation::SEGMENTID_IMAGE:
    return new SegmentIDFeature(id, model, sizeObs, typeObs); 
  default:
    JFR_RUN_TIME("SlamEkf::featureFactory: unknown feature type");
  }
}

BaseFeature* SlamEkf::featureFactory(unsigned int id,
				     BaseFeatureObserveModel& model)
{
  return featureFactory( id, model.featureModel, model.sizeObs(), model.typeObs());
}



void SlamEkf::initFeatureState(BaseFeature& feature, 
			       FeatureObserveModel& observeModel,
			       Observation const& obs)
{
//    JFR_TRACE_BEGIN;
  JFR_DEBUG( " feature.getX() =  " << feature.getX() << " feature.getP() =  " << feature.getP() );

  FeatureModel& featureModel = observeModel.featureModel; // shortcut


  // the line in ROBOT frame LR
/*  vec x = observeModel.inverseObservation(obs.z); 
  observeModel.inverseObservationJac(obs.z); // The Jacobians are in observeModel.JinvObs and featureModel.Jframe*/
    vec x = observeModel.inverseObservation(obs); 
    //JFR_DEBUG( "x = " << x );
    observeModel.inverseObservationJac(obs); // The Jacobians are in observeModel.JinvObs and featureModel.Jframe
    //JFR_DEBUG( " feature.getX() =  " << feature.getX() << " feature.getP() =  " << feature.getP() );

	/* now we have:
  	x                    = LR
  	observeModel.JinvObs = dLR/drt = LR_rt
  	featureModel.Jframe  = dLR/dC  = LR_c
  	featureModel.Jx      = dLR/dLC = LR_lc
  	*/
  mat JxTmp(featureModel.Jx); // A copy of the Jacobian for the chain rule
    //for lines
    // JxTmp = dLR/dLC = LR_lc
    //JFR_DEBUG("LR_lc:" << JxTmp);
  // the line in WORLD frame L
  featureModel.fromFrame(refPose( obs.robotId ), x, feature.getX()); 
  featureModel.fromFrameJac(refPose( obs.robotId ), x); // the jacobians are in featureModel.Jx and featureModel.Jframe
  
  //JFR_DEBUG( "refPose( obs.robotId ) = " << refPose( obs.robotId ) << " refPoseCov( obs.robotId )" << refPoseCov( obs.robotId ) );
  //JFR_DEBUG( "robotToSensor( obs.sensorId ) = " << robotToSensor( obs.sensorId ) );
  
  /* now we have
  feature.getX()      = L
  featureModel.Jframe = dL/dR  = L_r
  featureModel.Jx     = dL/dLR = L_lr
  */
  //JFR_DEBUG( " feature.getX() =  " << feature.getX() << " feature.getP() =  " << feature.getP() );

  //JFR_DEBUG( " featureModel.Jx = " << featureModel.Jx << " observeModel.JinvObs = " << observeModel.JinvObs );
  
  mat Gz(prod(featureModel.Jx, observeModel.JinvObs)); // Jacobian dL/drt, world line wrt measurement
    //JFR_DEBUG("featureModel.Jframe:" << featureModel.Jframe);
    //JFR_DEBUG("featureModel.Jx:" << featureModel.Jx);
    //JFR_DEBUG("Gz" << Gz);
 // now we have
  // Gz = dL/drt = L_lr*LR_rt = L_rt

  JxTmp = prod(featureModel.Jx,JxTmp); // Jacobian dL/dLC, world line wrt camera line
  // JxTmp = dL/dLC = L_lr*LR_lc = L_lc
  //JFR_DEBUG("L_lc:" << JxTmp);
  
  // compute RT = the covariance of rt
  observeModel.computeRInit(obs);

  //JFR_DEBUG("initFeatureState");
  //JFR_DEBUG("observe model R: " << observeModel.getR());
  

  // Compute the contributions of robot pose and measurement to line covariances:
  //   P_ll = L_r * P_rr * L_r' + L_rt * RT * L_rt'

  //JFR_DEBUG( " featureModel.Jframe = " << featureModel.Jframe << " refPoseCov(obs.robotId) = " << refPoseCov(obs.robotId) << " Gz = " << Gz << " observeModel.getRInit() = " << observeModel.getRInit() );
  feature.getP().assign( prod( featureModel.Jframe, mat(prod(refPoseCov(obs.robotId), trans(featureModel.Jframe))) ) +
			 prod( Gz, mat(prod(observeModel.getRInit(), trans(Gz))) ) );
  //JFR_DEBUG("FeatureCov(before postInitCovariance): " << feature.getP());

    //JFR_DEBUG("frame: " << refPose(obs.robotId));
    //JFR_DEBUG("Feature: " << feature.getX());

  // Add the contribution of the unobservable part beta 
  //   P_ll = P_ll + L_beta * B * L_beta' <--- N(beta,B) is the non-observable part
  observeModel.postInitCovariance(feature.getP(),JxTmp);
//  JFR_DEBUG("FeatureCov(after postInitCovariance): " << feature.getP());


  BaseRobot* baseRobot = robot( obs.robotId );
  // initialization of feature state cross covariance
  range r_r( baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizeState());
  range r_f(feature.filterIndex(), feature.filterIndex() + featureModel.sizeState());
  range r1( baseRobot->filterIndex(), feature.filterIndex());
  sym_mat_range PcrossRobot1(filter.getP(), r_r, r1);
  sym_mat_range PcrossFeature1(filter.getP(), r_f, r1);
  PcrossFeature1.assign( prod(featureModel.Jframe, PcrossRobot1) );

  if (feature.filterIndex() + featureModel.sizeState() < filter.sizeState() ) {
//	  JFR_DEBUG("ENTERS SECTION PCROSSROBOT2 PCROSSFEATURE2");
//	  JFR_DEBUG("HALTED ON PURPOSE. REMOVE LINE exit(0) IN SLAMEKF.CPP !!");
//	  exit(0); // STOPS HERE
    range r2(feature.filterIndex() + featureModel.sizeState(), filter.sizeState());
    sym_mat_range PcrossRobot2(filter.getP(), r_r, r2);
    //      sym_mat_range PcrossFeature2(filter.P, r_f, r2); //FIXME bug in ublas ??
    sym_mat_range PcrossFeature2(filter.getP(), r2, r_f);
    //      PcrossFeature2.assign( prod(model->Jframe, PcrossRobot2) );        //FIXME bug in ublas ??
    PcrossFeature2.assign( trans(prod(featureModel.Jframe, PcrossRobot2)) );
  }

  if(  baseRobot->filterIndex() > 0 )
  {
    range r3( 0, baseRobot->filterIndex());
    sym_mat_range PcrossRobot3(filter.getP(), r_r, r3);
    sym_mat_range PcrossFeature3(filter.getP(), r3, r_f);
    PcrossFeature3.assign( trans(prod(featureModel.Jframe, PcrossRobot3)) );
  }

//  JFR_DEBUG("PcrossRobot1: " << PcrossRobot1);
  //JFR_DEBUG("PcrossFeature1: " << PcrossFeature1);

//   JFR_TRACE_END("SlamEkf::initFeatureState: id:" << feature.id());
    //  JFR_DEBUG("Robot covariance 4 = " << refPoseCov(obs.robotId));
}

void SlamEkf::observeKnownFeature(Observation* obs)
{
//   JFR_TRACE_BEGIN;

  observeKnownFeature(*obs,
		      getFeature(obs->id),
		      getFeatureObserveModel(obs->sensorId));

//   JFR_TRACE_END("SlamEkf::observeKnownFeature: id:" << obs->id);
  delete obs;
}

void SlamEkf::observeKnownFeature(Observation const& obs, BaseFeature& feature, BaseFeatureObserveModel& model)
{
  JFR_DEBUG("SlamEkf::observeFeature: Known feature (" << obs.id << ")");

  BaseRobot* baseRobot = robot( obs.robotId );
  
  JFR_DEBUG("Observation: " << obs);
  JFR_DEBUG("Ref pose: " << *baseRobot->refPose() << " feature.getX() =  " << feature.getX() << " feature.getP() =  " << feature.getP() );

  model.computeR(obs);

  if( feature.zPred.size() != obs.z.size() )
  {
    feature.zPred = jmath::GaussianVector( obs.z.size() );
  }

  feature.addObservation(currentFrameIndex);

  try {
    filter.update(model,
		  baseRobot->filterIndex(), feature.filterIndex(),
		  *baseRobot->refPose(), feature.getX(),
		  *baseRobot->refPoseCov(), feature.getP(),
		  obs.z,
		  feature.zPred);
  //JFR_DEBUG("Ref pose: " << *baseRobot->refPose() << "robotId: " << obs.robotId);
  //JFR_DEBUG("Pose: " << *baseRobot->refPose() );
  //JFR_DEBUG("PoseCov: " << *baseRobot->refPoseCov());
  //JFR_DEBUG("FeatureUp =  " << feature.getX() );
  //JFR_DEBUG("FeatureUpCov: " << feature.getP() );

//tl 
//     feature.applyConstraints();

//tavc applyContraint with Jacobian modifying directly the mean and covariance
//    FeatureModel& featureModel = model.featureModel; // shortcut
//    featureModel.fixFeature(feature.getX());

//tavc applyContraint without Jacobian implemented in SLAMEKF it needs to be uncommented
//    fixFeature(feature);

//tl applyConstraints with the filter using the model with noise to apply it in a soft way
    applyConstraints(feature);

    // non stochastic state update
    model.updateInternalState(feature, geom::T3DEuler(*baseRobot->refPose()), obs);

    nbObservedLandmarks++;
  }
  catch(filter::InconsistentUpdateException const& e) {
    JFR_WARNING("SlamEkf::observeKnownFeature: inconsistent observation discarded, landmark: " << feature.id() << "\n"
		<< e);
    nbInconsistentUpdates++;
    if (e.mahalanobisDistance > mahalanobisDistanceToRemoveLandmark) {
      JFR_WARNING("SlamEkf::observeKnownFeature: remove landmark " << e.mahalanobisDistance << " " << mahalanobisDistanceToRemoveLandmark);
      removeFeature(feature.id());
    }
 }
  catch(kernel::JafarException const& e) {
    if (e.getExceptionId() == kernel::JafarException::NUMERIC) {
      JFR_WARNING("SlamEkf::observeKnownFeature: observation discarded, feature: " << feature.id() << "\n"
		  << e);
    }
    else
      throw;
  } 
  //JFR_DEBUG("Ref pose: " << *baseRobot->refPose() << "m_refPoseCov: " << *baseRobot->refPoseCov() );
  //JFR_DEBUG("Prediction: " << model.zPred);
}

void SlamEkf::predict(unsigned int _robotId, 
		      jblas::vec const& u) 
{
  BaseRobot* robot_ = robot( _robotId );
  JFR_PRECOND(robot_->model.sizeState() == sizeRobotState(),
	      "SlamEkf::predict: invalid prediction model");
//    JFR_TRACE_BEGIN;
//   JFR_DEBUG("Ref pose before prediction: " << m_refPose );
  filter.predict( robot_->filterIndex(), robot_->filterIndex() + robot_->sizeState(), robot_->model, u);
  //   JFR_DEBUG("Ref pose after prediction: " << m_refPose );
  //   JFR_DEBUG("Robot Pose: " << m_refPose );
    //JFR_DEBUG("Pose: " << *robot_->refPose() );
   //JFR_DEBUG("PoseCov: " << *robot_->refPoseCov());

//   JFR_TRACE_END("SlamEkf::predict()");

}

BaseRobot* SlamEkf::robot( unsigned int _robotId )
{
  BaseRobot* baseRobot = robotsMap[_robotId];
  JFR_ASSERT( baseRobot, "No robot of Id = " << _robotId );
  return baseRobot;
}

const BaseRobot* SlamEkf::robot( unsigned int _robotId ) const
{
  RobotsMapType::const_iterator it = robotsMap.find( _robotId );
  JFR_ASSERT( it != robotsMap.end(), "No robot of Id = " << _robotId );
  return it->second;
}

void SlamEkf::addRobot( BaseRobot* _robot)
{
  robotsMap[_robot->id()] = _robot;
  mapManager->setMapObjectState( *_robot );
  
  
  size_t startIndex = _robot->filterIndex();
  size_t endIndex = _robot->filterIndex() + _robot->sizePose();
  
  _robot->m_refPose = new jblas::vec_range(filter.getX(), range( startIndex, endIndex ));
  _robot->m_refPoseCov = new jblas::sym_mat_range(filter.getP(), range(startIndex, endIndex), range(startIndex, endIndex));
  
}

void SlamEkf::addRobot( BaseRobot* _robot, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov )
{
  addRobot(_robot);
  setRobotPoseCov(_robotState, _robotStateCov, _robot->id());
}

void SlamEkf::applyConstraints(BaseFeature& feature)
{
  int i = 0;
  for (BaseFeature::ConstraintsListType::iterator it = feature.constraints.begin() ;
       it != feature.constraints.end() ; ++it) {
    //JFR_DEBUG("SlamEkf::applyConstraints: feature: " << feature.id() << " constraint: " << i);
    try {
      filter.applyConstraint((**it), feature.filterIndex(), feature.getX(), feature.getP());
    }
    catch(filter::InconsistentUpdateException const& e) {
      JFR_WARNING("SlamEkf::applyConstraints: inconsistent constraint discarded, feature: " 
		  << feature.id()
		  << ", constraint: " << i
		  << "\n" << e);
    }
    catch(kernel::JafarException const& e) {
      if (e.getExceptionId() == kernel::JafarException::NUMERIC) {
	JFR_WARNING("SlamEkf::observeKnownFeature: constraint discarded, feature: " 
		    << feature.id()
		    << ", constraint: " << i
		    << "\n" << e);
      }
      else
	throw;
    }
    i++;
  }
}

void SlamEkf::fixFeature(BaseFeature& feature) 
{
  JFR_TRACE_BEGIN;

  if (feature.featureConstraintModel) 
  {

//     JFR_DEBUG("index1: " << feature.filterIndex());
//     JFR_DEBUG("index2: " << feature.filterIndex() + feature.sizeState());
    filter.predict( feature.filterIndex(), feature.filterIndex() + feature.sizeState(), *feature.featureConstraintModel);
  }
  JFR_TRACE_END("SlamEkf::fixFeature()");
}

// double SlamEkf::computeMahalanobisDistance(const Observation& obs) {
//   BaseFeature& feature = getFeature(obs.id);
//   return computeMahalanobisDistance(feature, obs);
// }

// double SlamEkf::computeMahalanobisDistance(const BaseFeature& feature_, const Observation& obs) {
//   BaseFeatureObserveModel& model = getBaseFeatureObserveModel(obs.sensorId);
//   model.computeR(obs);
//   return filter.computeMahalanobisDistance(model,
//                                            0, feature_.filterIndex,
//                                            refPose(), feature_.getX(),
//                                            refPoseCov(), feature_.getP(),
//                                            obs.z);
// }

void SlamEkf::writeLogHeader(jafar::kernel::DataLogger& log) const 
{
  log.writeComment("slam: SlamEkf");
  log.writeLegend("time (ms)");
  log.writeLegend("frame index");
	log.writeLegendTokens("refPose1 refPose2 refPose3 refPose4 refPose5 refPose6 refPoseCov1 refPoseCov2 refPoseCov3 refPoseCov4 refPoseCov5 refPoseCov6");
  switch(sizeRobotState()) {
  case 6: // simple robot case
    for( std::size_t i = 0; i < robotsMap.size(); ++i)
    {
      log.writeLegendTokens("x y z yaw pitch roll");
      log.writeLegendTokens("cov_xx cov_xy cov_xz cov_xyaw cov_xpitch cov_xroll");
      log.writeLegendTokens("cov_yx cov_yy cov_yz cov_yyaw cov_ypitch cov_yroll");
      log.writeLegendTokens("cov_zx cov_zy cov_zz cov_zyaw cov_zpitch cov_zroll");
      log.writeLegendTokens("cov_yawx cov_yawy cov_yawz cov_yawyaw cov_yawpitch cov_yawroll");
      log.writeLegendTokens("cov_pitchx cov_pitchy cov_pitchz cov_pitchyaw cov_pitchpitch cov_pitchroll");
      log.writeLegendTokens("cov_rollx cov_rolly cov_rollz cov_rollyaw cov_rollpitch cov_rollroll");
      log.writeLegendTokens("sig_x sig_y sig_z sig_yaw sig_pitch sig_roll");
    }
    break;
  case 9:  // karma case constant speed 
    log.writeLegendTokens("x y z yaw pitch roll");
    log.writeLegendTokens("vx vy vz");
    log.writeLegendTokens("sig_x sig_y sig_z sig_yaw sig_pitch sig_roll");
    log.writeLegendTokens("sig_vx sig_vy sig_vz");
  case 10:  // karma case constant speed 
    log.writeLegendTokens("x y z yaw pitch roll");
    log.writeLegendTokens("vx vy vz");
    log.writeLegend("vyaw");
    log.writeLegendTokens("sig_x sig_y sig_z sig_yaw sig_pitch sig_roll");
    log.writeLegendTokens("sig_vx sig_vy sig_vz");
    log.writeLegend("sig_vyaw");
  default:
    JFR_RUN_TIME("SlamEkf::writeLogHeader: undefined case");
  }
  log.writeLegend("pose uncertainty volume"); 
  log.writeLegend("map uncertainty level");
  log.writeLegend("number of new landmarks");
  log.writeLegend("number of observed landmarks");
  log.writeLegend("number of lanmarks in map");
  log.writeLegend("number of inconsistent updates");
  log.writeLegend("update elapsed time (ms)");
}

void SlamEkf::writeLogData(jafar::kernel::DataLogger& log) const {
  log.writeData(filter.getCurrentTime().total_milliseconds());
  log.writeData(currentFrameIndex);

  for( RobotsMapType::const_iterator it = robotsMap.begin();
       it != robotsMap.end(); ++it )
  {
    log.writeDataVector( *it->second->refPose());
    for(std::size_t i = 0; i < it->second->refPoseCov()->size1(); ++i)
    {
      log.writeDataVector( ublas::matrix_row<jblas::sym_mat_range>( *it->second->refPoseCov(), i) );
    }
    for (std::size_t i=0 ; i < 6 ; ++i) {
      log.writeData(sqrt( (*it->second->refPoseCov())(i,i)));
    }
  }

  //  log.writeDataVector(getRobotState()); FIXME
  //   for (std::size_t i=0 ; i < sizeRobotState() ; ++i) {
  //     log.writeData(sqrt(filter.getP()(i,i)));
  //   }
    
  log.writeData(jmath::ublasExtra::lu_det(ublas::project(refPoseCov(), ublas::range(0,2), ublas::range(0,2) )));
  log.writeData(getMapUncertaintyLevel());
  log.writeData(nbNewLandmarks);
  log.writeData(nbObservedLandmarks);
  log.writeData(sizeMap());
  log.writeData(nbInconsistentUpdates);
  log.writeData(updateElapsedTime);
}


std::ostream& jafar::slam::operator <<(std::ostream& s, const SlamEkf& slam_) {
  s << " * robot: " << slam_.refPose() << " - " << slam_.refPoseCov() << std::endl;
  s << " * map: " << std::endl;
  for (SlamEkf::FeaturesMapType::const_iterator it = slam_.featuresMap.begin() ;
       it != slam_.featuresMap.end() ; it++) {
    s << *(it->second) << std::endl;
  }
  return s;
}

jblas::vec_range const& SlamEkf::refPose(int _robotId) const
{
  const BaseRobot* baseRobot = robot( _robotId );
  return *baseRobot->refPose();
}

jblas::sym_mat_range const& SlamEkf::refPoseCov(int _robotId) const
{
  const BaseRobot* baseRobot = robot( _robotId );
  return *baseRobot->refPoseCov();
}

void SlamEkf::setRobotZ(double z, double zStdDev, int _robotId )
{
  BaseRobot* baseRobot = robot( _robotId );
  (*baseRobot->refPose())(2) = z;
  (*baseRobot->refPoseCov())(2,2) = pow(zStdDev,2);
}

void SlamEkf::setRobotPitch(double pitch, double pitchStdDev, int _robotId )
{
  BaseRobot* baseRobot = robot( _robotId );
  (*baseRobot->refPose())(4) = pitch;
  (*baseRobot->refPoseCov())(4,4) = pow(pitchStdDev,2);
}

void SlamEkf::setRobotRoll(double roll, double rollStdDev, int _robotId )
{
  BaseRobot* baseRobot = robot( _robotId );
  (*baseRobot->refPose())(5) = roll;
  (*baseRobot->refPoseCov())(5,5) = pow(rollStdDev,2);
}


void SlamEkf::transform( const jafar::geom::T3DEuler& _transform )
{
  jblas::mat J1s = jblas::zero_mat( filter.sizeState(), 6 );
  // This vector contains a pointer to the AbstractMapObject and the jacobian (Jx) matrix
  std::vector< std::pair< AbstractMapObject*, jblas::mat > > infos;
  
  // Precompute the Jacobians
  for (SlamEkf::RobotsMapType::const_iterator it = robotsMap.begin() ;
       it != robotsMap.end() ; it++)
  {
    BaseRobot* robot = it->second;
    ublas::range rangeMat( robot->filterIndex(), robot->filterIndex() + robot->sizeState() );
    
    jblas::mat J1(6,6);
    jblas::mat J2(6,6);
    geom::T3DEuler robotPos( robot->getX() );
    geom::T3DEuler composedRobotPos;
    geom::T3D::compose( _transform, robotPos, composedRobotPos, J1, J2);
    
    robot->getX().assign( composedRobotPos.getX() );
    ublas::project( J1s, rangeMat, ublas::range(0, 6 ) ).assign( J1 );
    infos.push_back( std::pair< AbstractMapObject*, jblas::mat >( robot, J2 ) );
  }
  for (SlamEkf::FeaturesMapType::const_iterator it = featuresMap.begin() ;
       it != featuresMap.end() ; it++)
  {
    BaseFeature* feature = it->second;
    ublas::range rangeMat( feature->filterIndex(), feature->filterIndex() + feature->sizeState() );
    jblas::vec v( feature->sizeState() );
    feature->model.fromFrame( _transform.getX(), feature->getX(), v);
    feature->getX().assign( v );
    feature->model.fromFrameJac( _transform.getX(), feature->getX() );
    ublas::project( J1s, rangeMat, ublas::range(0, 6) ).assign( feature->model.Jframe );
    infos.push_back( std::pair< AbstractMapObject*, jblas::mat >( feature, feature->model.Jx ) );
  }
  // Add the feature to feature covariance
  for( std::vector< std::pair< AbstractMapObject*, jblas::mat > >::iterator it = infos.begin();
       it != infos.end(); ++it )
  {
    AbstractMapObject* object1 = it->first;
    ublas::range rangeMat1( object1->filterIndex(), object1->filterIndex() + object1->sizeState() ); 
    for( std::vector< std::pair< AbstractMapObject*, jblas::mat > >::iterator it2 = it;
         it2 != infos.end(); ++it2 )
    {
      AbstractMapObject* object2 = it2->first;
      ublas::range rangeMat2( object2->filterIndex(), object2->filterIndex() + object2->sizeState() );
      project( filter.getP(), rangeMat1, rangeMat2 ).assign(
          ublas::prod( it->second /* Jx of first object */,
                       mat( prod(
                           project( filter.getP(), rangeMat1, rangeMat2 ), it2->second  /* Jx of second object */ ) ) ) );
    }
  }
  // Add the transformation covariance
  ublas::range stateRange(0, filter.sizeState());
  ublas::project(filter.getP(), stateRange, stateRange).plus_assign(
      ublas::prod(J1s, jblas::mat( ublas::prod( _transform.getXCov(), ublas::trans(J1s) ) ) ) );
}
