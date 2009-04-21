/* $Id$ */

#include "geom/t3dIdentity.hpp"

#include "slam/featureModel.hpp"

using namespace ublas;
using namespace jblas;
using namespace jafar::slam;

/*
 * class BaseFeatureObserveModel
 */

BaseFeatureObserveModel::BaseFeatureObserveModel(FeatureModel& featureModel_, std::size_t sizeObs_) : 
BlockObserveModel(sizeObs_, featureModel_.sizeRobotPose(), featureModel_.sizeState()),
featureModel(featureModel_),
RInit(sizeObs_, sizeObs_),
zPred(sizeObs_),
JobsSensor(sizeObs_, featureModel_.sizeState()),
JobsRobotToSensor(sizeObs_, featureModel_.sizeRobotPose()),
featureR(sizeObs_, sizeObs_),
p_sizeObsInit(sizeObs_)
{
	m_robotToSensor = new geom::T3DIdentity();
}

BaseFeatureObserveModel::BaseFeatureObserveModel(FeatureModel& featureModel_, std::size_t sizeObs_, std::size_t sizeObsInit_) : 
BlockObserveModel(sizeObs_, featureModel_.sizeRobotPose(), featureModel_.sizeState()),
featureModel(featureModel_),
RInit(sizeObsInit_, sizeObsInit_),
zPred(sizeObs_),
JobsSensor(sizeObs_, featureModel_.sizeState()),
JobsRobotToSensor(sizeObs_, featureModel_.sizeRobotPose()),
featureR(sizeObs_, sizeObs_),
p_sizeObsInit(sizeObsInit_)
{
  m_robotToSensor = new geom::T3DIdentity();
}

BaseFeatureObserveModel::BaseFeatureObserveModel(FeatureModel& featureModel_, std::size_t sizeObs_, std::size_t sizeObsInit_, std::size_t sizeInnovation_, std::size_t sizePrediction) : 
BlockObserveModel(sizeObs_, featureModel_.sizeRobotPose(), featureModel_.sizeState(), sizeInnovation_, sizePrediction),
featureModel(featureModel_),
RInit(sizeObsInit_, sizeObsInit_),
zPred(sizeObs_),
JobsSensor(sizeObs_, featureModel_.sizeState()),
JobsRobotToSensor(sizeObs_, featureModel_.sizeRobotPose()),
featureR(sizeObs_, sizeObs_),
p_sizeObsInit(sizeObsInit_)
{
  m_robotToSensor = new geom::T3DIdentity();
}

BaseFeatureObserveModel::~BaseFeatureObserveModel() {
  delete m_robotToSensor;
}

void BaseFeatureObserveModel::setRobotToSensor(const jafar::geom::T3DEuler& robotToSensor_)
{
  delete m_robotToSensor;
  m_robotToSensor = new geom::T3DEuler(robotToSensor_);
  if (robotToSensor().hasCov())
    JFR_RUN_TIME("BaseFeatureObserveModel: robotToSensor uncertainty broken !!");
    //    JFR_DEBUG("BaseFeatureObserveModel: robotToSensor uncertainty enabled");
}


// Compute measurement noise covariances amtrix
void BaseFeatureObserveModel::computeR(Observation const& obs) {
	JFR_PRECOND(obs.size() == sizeObs(),
				"BaseFeatureObserveModel::computR: invalid size");
	computeSensorR(obs);
	//JFR_DEBUG("hasCov: " << robotToSensor().hasCov());
	
	if (robotToSensor().hasCov()) {
		//JFR_DEBUG("Rsensor: " << R);
		//JFR_DEBUG("Rtrans: " << prod(JobsRobotToSensor, mat(prod(robotToSensor().getXCov(), trans(JobsRobotToSensor)))) );
		featureR.assign( R + prod(JobsRobotToSensor, mat(prod(robotToSensor().getXCov(), trans(JobsRobotToSensor)))) );
	}
	
}


// Compute measurement noise covariances matrix for init purposes
void BaseFeatureObserveModel::computeRInit(Observation const& obs) {

	computeSensorRInit(obs);
    //JFR_DEBUG("RsensorInit: " << RInit);
	
	/* TODO implement uncertaint robotToSensor case.
	JFR_DEBUG("hasCov: " << robotToSensor().hasCov());
	if (robotToSensor().hasCov()) {
		//JFR_DEBUG("Rsensor: " << R);
		//JFR_DEBUG("Rtrans: " << prod(JobsRobotToSensor, mat(prod(robotToSensor().getXCov(), trans(JobsRobotToSensor)))) );
		featureR.assign( R + prod(JobsRobotToSensor, mat(prod(robotToSensor().getXCov(), trans(JobsRobotToSensor)))) );
	}*/	
}



jblas::vec const& BaseFeatureObserveModel::predictObservation(const vec_range& robotPose_, const vec_range& feature_)
{
  JFR_PRECOND(robotPose_.size() == _sizeState1, 
              "BaseFeatureObserveModel::predictObservation: size of robotPose_ does not match");
  JFR_PRECOND(feature_.size() == _sizeState2, 
              "BaseFeatureObserveModel::predictObservation: size of feature_ does not match");

  jblas::vec xTmp1(sizeState());
  if (robotToSensor().isIdentity()) {
    featureModel.toFrame(robotPose_, feature_, xTmp1);    
JFR_DEBUG(__FILE__ << ":" << __LINE__ << "|BaseFeatureObserveModel::predictObservation " <<
"robotPose " << robotPose_ << " ; feature " << feature_ << " ; toFrame " << xTmp1);
    return predictObservationInSensorFrame(xTmp1);
  }
  else {
    jblas::vec xTmp2(sizeState());
    featureModel.toFrame(robotPose_, feature_, xTmp1);
    featureModel.toFrame(robotToSensor().getX(), xTmp1, xTmp2);
JFR_DEBUG(__FILE__ << ":" << __LINE__ << "|BaseFeatureObserveModel::predictObservation " <<
"robotPose " << robotPose_ << " ; feature " << feature_ << " ; toFrame " << xTmp1 << " ; robotToSensor().getX() " 
<< robotToSensor().getX() << " ; toFrame " << xTmp2);;
    return predictObservationInSensorFrame(xTmp2);
  }
}



void BaseFeatureObserveModel::predictObservationJac(const vec_range& robotPose_,
						    const vec_range& feature_) 
{
  computePredictObservationJac( robotPose_, feature_, Jobs1, Jobs2 );
}
  
void BaseFeatureObserveModel::computePredictObservationJac(const jblas::vec_range& robotPose_,
           const jblas::vec_range& feature_, jblas::mat& J1_, jblas::mat& J2_)
{
  JFR_PRECOND(robotPose_.size() == _sizeState1, 
              "BaseFeatureObserveModel::predictObservationAndJac: size of robotPose_ does not match");
  JFR_PRECOND(feature_.size() == _sizeState2, 
              "BaseFeatureObserveModel::predictObservationAndJac: size of feature_ does not match");

  jblas::vec xTmp1(sizeState());
  mat JframeTmp(sizeState(), sizeRobotPose());
  mat JxTmp(sizeState(), sizeState());
  featureModel.toFrame(robotPose_, feature_, xTmp1);
  featureModel.toFrameJac(robotPose_, feature_);
  JframeTmp.assign(featureModel.Jframe);
  JxTmp.assign(featureModel.Jx);

//   JFR_DEBUG("JframeTmp = " << JframeTmp);
//   JFR_DEBUG("JxTmp = " << JxTmp);

  if (robotToSensor().isIdentity()) {
    predictObservationInSensorFrameJac(xTmp1);  
  }
  else {
    jblas::vec xTmp2(sizeState());
    featureModel.toFrame(robotToSensor().getX(), xTmp1, xTmp2);
    featureModel.toFrameJac(robotToSensor().getX(), xTmp1);
    JframeTmp = prod(featureModel.Jx, JframeTmp);
    JxTmp = prod(featureModel.Jx, JxTmp);

    predictObservationInSensorFrameJac(xTmp2);
  }

  //JFR_DEBUG("JobsSensor = " << JobsSensor);
  //JFR_DEBUG("JframeTmp = " << JframeTmp);
  //JFR_DEBUG("JxTmp = " << JxTmp);
  
  J1_.assign(prod(JobsSensor, JframeTmp));
  J2_.assign(prod(JobsSensor, JxTmp));

  if (robotToSensor().hasCov())
    JobsRobotToSensor.assign(prod(JobsSensor, featureModel.Jframe) );
}


/*
 * class FeatureObserveModel
 */

FeatureObserveModel::FeatureObserveModel(FeatureModel& model_, std::size_t sizeObs_) :
BaseFeatureObserveModel(model_, sizeObs_),
JinvObs(sizeState(), sizeObsInit())
{}

FeatureObserveModel::FeatureObserveModel(FeatureModel& model_, std::size_t sizeObs_, std::size_t sizeObsInit_) :
BaseFeatureObserveModel(model_, sizeObs_, sizeObsInit_),
JinvObs(sizeState(), sizeObsInit())
{}

FeatureObserveModel::FeatureObserveModel(FeatureModel& model_, std::size_t sizeObs_, std::size_t sizeObsInit_, std::size_t sizeInnovation_, std::size_t sizePrediction_) :
BaseFeatureObserveModel(model_, sizeObs_, sizeObsInit_, sizeInnovation_, sizePrediction_),
JinvObs(sizeState(), sizeObsInit())
{}


FeatureObserveModel::~FeatureObserveModel() {}

vec FeatureObserveModel::inverseObservation(Observation const& obs_) {
  JFR_PRECOND(obs_.size() == sizeObs(),
              "FeatureObserveModel::inverseObservation: size of obs_ does not match");

  vec x(sizeState());

  if (robotToSensor().isIdentity()) {
    x.assign(inverseObservationInSensorFrame(obs_));
  }
  else {
    featureModel.fromFrame(robotToSensor().getX(), inverseObservationInSensorFrame(obs_), x);
    //JFR_DEBUG("robotToSensor: " << robotToSensor().getX());
    //JFR_DEBUG("inverseObservation: " << inverseObservationInSensorFrame(obs_));
    //JFR_DEBUG("x: " << x);
  }

  return x;
}

      
void FeatureObserveModel::inverseObservationJac(Observation const& obs_) {
  inverseObservationInSensorFrameJac(obs_);

  //JFR_DEBUG(" Robot To Sensor: " << robotToSensor().getX());

  if (!robotToSensor().isIdentity()) {
    featureModel.fromFrameJac(robotToSensor().getX(), inverseObservationInSensorFrame(obs_));
	//JFR_DEBUG("INVERSEOBSERVATION Jx = " << featureModel.Jx);
	//JFR_DEBUG("Inverse Jacobian (before):" << JinvObs);
    JinvObs = prod(featureModel.Jx, JinvObs);
	//JFR_DEBUG("Inverse Jacobian (after):" << JinvObs);
  }
}

