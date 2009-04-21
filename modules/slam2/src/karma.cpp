/* $Id$ */

#ifdef HAVE_TTL

#include "jmath/angle.hpp"

#include "slam/karma.hpp"

using namespace jafar;
using namespace jafar::slam;

/*
 * class GPSPoseObserveModel
 */

GPSPoseObserveModel::GPSPoseObserveModel() :
  LinearObserveModel(3,3)
{
  H.assign(jblas::identity_mat(3));
  R.clear();
}

GPSPoseObserveModel::~GPSPoseObserveModel() {}

void GPSPoseObserveModel::setNoise(double xStdDev_, double yStdDev_, double zStdDev_)
{
  R(0,0) = xStdDev_*xStdDev_;
  R(1,1) = yStdDev_*yStdDev_;
  R(2,2) = zStdDev_*zStdDev_;
}

/*
 * class GPSSpeedObserveModel
 */

GPSSpeedObserveModel::GPSSpeedObserveModel() :
  LinearObserveModel(3,3)
{
  H.assign(jblas::identity_mat(3));
  R.clear();
}

GPSSpeedObserveModel::~GPSSpeedObserveModel() {}

void GPSSpeedObserveModel::setNoise(double vxStdDev_, double vyStdDev_, double vzStdDev_) 
{
  R(0,0) = vxStdDev_*vxStdDev_;
  R(1,1) = vyStdDev_*vyStdDev_;
  R(2,2) = vzStdDev_*vzStdDev_;
}

/*
 * class AttitudeObserveModel
 */

AttitudeObserveModel::AttitudeObserveModel() :
  LinearObserveModel(3,3)
{
  H.assign(jblas::identity_mat(3));
  R.clear();
}

AttitudeObserveModel::~AttitudeObserveModel() {}

void AttitudeObserveModel::setNoise(double yawStdDev_, double pitchStdDev_, double rollStdDev_)
{
  R(0,0) = yawStdDev_*yawStdDev_;
  R(1,1) = pitchStdDev_*pitchStdDev_;
  R(2,2) = rollStdDev_*rollStdDev_;
}
      
jblas::vec& AttitudeObserveModel::computeInnovation(const jblas::vec& z_, const jblas::vec& x_)
{
  LinearObserveModel::computeInnovation(z_, x_);
  z(0) = jmath::toMinusPiPi(z(0));
  return z;
}

// /*
//  * class KarmaPredictModel
//  */

// KarmaPredictModel::KarmaPredictModel() :
//   LinearPredictModel(9),
//   accVar(3)
// {
//   Q.clear();
//   F.assign(jblas::identity_mat(sizeState()));
// }

// KarmaPredictModel::~KarmaPredictModel() {}

// void KarmaPredictModel::setAttitudeStdDev(double yawSpeedStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_)
// {
//   yawSpeedVar = yawSpeedStdDev_*yawSpeedStdDev_;
//   pitchSpeedVar = pitchSpeedStdDev_*pitchSpeedStdDev_;
//   rollSpeedVar = rollSpeedStdDev_ * rollSpeedStdDev_;
// }

// void KarmaPredictModel::setAccStdDev(double xAccStdDev_, double yAccStdDev_, double zAccStdDev_)
// {
//   accVar(0) = xAccStdDev_*xAccStdDev_;
//   accVar(1) = yAccStdDev_*yAccStdDev_;
//   accVar(2) = zAccStdDev_*zAccStdDev_;
// }

// void KarmaPredictModel::setTimeStep(double dt)
// {
//   timeStep = dt;

//   double dt2 = pow(dt,2);

//   F(0,6) = dt;
//   F(1,7) = dt;
//   F(2,8) = dt;

//   //  JFR_DEBUG("F: \n" << jmath::prettyFormat(F));

//   // global position
//   // pose
//   double p0 = pow(dt,4)/4;
//   // pose / speed
//   double p01 = pow(dt,3)/2;
//   // speed
//   double p1 = dt2;

//   Q(0,0) = p0*accVar(0);
//   Q(0,6) = p01*accVar(0);
//   Q(6,6) = p1*accVar(0);

//   Q(1,1)   = p0*accVar(1);
//   Q(1,7)   = p01*accVar(1);
//   Q(7,7)   = p1*accVar(1);

//   Q(2,2)   = p0*accVar(2);
//   Q(2,8)   = p01*accVar(2);
//   Q(8,8)   = p1*accVar(2);

//   // attitude
//   double a0 = pow(dt,4)/4;
//   double a01 = pow(dt,3)/2;
//   double a1 = dt2;
//   Q(3,3) = dt2*yawSpeedVar;
//   Q(4,4) = dt2*pitchSpeedVar;
//   Q(5,5) = dt2*rollSpeedVar;

//   //  JFR_DEBUG("Q: \n" << jmath::prettyFormat(Q));
// }


/*
 * class KarmaPredictModel
 */

KarmaPredictModel::KarmaPredictModel() :
  LinearPredictModel(10),
  accVar(3)
{
  Q.clear();
  F.assign(jblas::identity_mat(10));
}

KarmaPredictModel::~KarmaPredictModel() {}

void KarmaPredictModel::setAttitudeStdDev(double yawAccStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_)
{
  yawAccVar = yawAccStdDev_*yawAccStdDev_;
  pitchSpeedVar = pitchSpeedStdDev_*pitchSpeedStdDev_;
  rollSpeedVar = rollSpeedStdDev_ * rollSpeedStdDev_;
}

void KarmaPredictModel::setAccStdDev(double xAccStdDev_, double yAccStdDev_, double zAccStdDev_)
{
  accVar(0) = xAccStdDev_*xAccStdDev_;
  accVar(1) = yAccStdDev_*yAccStdDev_;
  accVar(2) = zAccStdDev_*zAccStdDev_;
}

void KarmaPredictModel::setTimeStep(double dt)
{
  timeStep = dt;

  double dt2 = pow(dt,2);

  F(0,6) = dt;
  F(1,7) = dt;
  F(2,8) = dt;
  F(3,9) = dt;

  //  JFR_DEBUG("F: \n" << jmath::prettyFormat(F));

  // global position
  // pose
  double p0 = pow(dt,4)/4;
  // pose / speed
  double p01 = pow(dt,3)/2;
  // speed
  double p1 = dt2;

  Q(0,0) = p0*accVar(0);
  Q(0,6) = p01*accVar(0);
  Q(6,6) = p1*accVar(0);

  Q(1,1)   = p0*accVar(1);
  Q(1,7)   = p01*accVar(1);
  Q(7,7)   = p1*accVar(1);

  Q(2,2)   = p0*accVar(2);
  Q(2,8)   = p01*accVar(2);
  Q(8,8)   = p1*accVar(2);

  // attitude
  double a0 = pow(dt,4)/4;
  double a01 = pow(dt,3)/2;
  double a1 = dt2;
  Q(3,3) = a0*yawAccVar;
  Q(3,9) = a01*yawAccVar;
  Q(9,9) = a1*yawAccVar;

  Q(4,4) = dt2*pitchSpeedVar;
  Q(5,5) = dt2*rollSpeedVar;

  //  JFR_DEBUG("Q: \n" << jmath::prettyFormat(Q));
}

/*
 * class KarmaBoSlamHelper
 */

KarmaBoSlamHelper::KarmaBoSlamHelper(SlamEkf& slam_,
				     std::size_t sizeLocalMapMax_,
				     std::size_t sizeFeatureState_,
				     Map& map_) :
  LocalSlamHelper(slam_, sizeLocalMapMax_, sizeFeatureState_),
  map(map_)
{}

void KarmaBoSlamHelper::cleanSlamMap() 
{
  SlamEkf::map_const_iterator map_it;
  for (features_const_iterator it = notObservedFeatures->begin() ; 
       it != notObservedFeatures->end() ; it++) {
    JFR_DEBUG("LocalSlamHelper::cleanMap: landmark " << *it);
    map_it = slam.getMap().find(*it);
    freeStatesIndex.push_front(map_it->second->filterIndex);
    map.add(*(map_it->second));
    slam.removeFeature(*it);
  }
}

/*
 * class KarmaManager
 */

KarmaManager::KarmaManager(BearingOnlySlam& slam_,
			   unsigned int sizeLocalMap_,
			   double xmin_, double ymin_, 
			   double xmax_, double ymax_) :
//   gpsPoseObserveModel(),
//   gpsSpeedObserveModel(),
//   attitudeObserveModel(),
//   gpsPoseData(),
//   gpsSpeeddata(),
//   attitudeData(),
//   allSensorsData(),
//   allObserveModel(gpsPoseObserveModel, gpsSpeedObserveModel,attitudeObserveModel) 
  slam(slam_),
  map(xmin_, ymin_, xmax_, ymax_),
  slamHelper(slam, sizeLocalMap_, 3, map)
{}

void KarmaManager::predictTo(double time_)
{
  JFR_DEBUG("KarmaManager::predictTo: " <<  time_ );
  if (time_ > slam.getFilter().getTime()) {
    predictModel.setTimeStep(time_ - slam.getFilter().getTime());
    JFR_TRACE_BEGIN;
    slam.predict(predictModel);
    JFR_TRACE_END;
  }
}

void KarmaManager::setInitState(double timeStamp,
				double x_, double y_, double z_, 
				double yaw_, double pitch_, double roll_,
				double vx_, double vy_, double vz_)
				// 				double vyaw_, double vyawStdDev_)
{
  jblas::vec state(10);
  state(0) = x_;
  state(1) = y_;
  state(2) = z_;
  state(3) = jmath::toMinusPiPi(yaw_);
  state(4) = pitch_;
  state(5) = roll_;
  state(6) = vx_;
  state(7) = vy_;
  state(8) = vz_;
  //  state(9) = vyaw_;
  state(9) = 0;
  jblas::vec stateCov(10);
  stateCov(0) = gpsPoseObserveModel.getR()(0,0);
  stateCov(1) = gpsPoseObserveModel.getR()(1,1);
  stateCov(2) = gpsPoseObserveModel.getR()(2,2);
  stateCov(3) = attitudeObserveModel.getR()(0,0);
  stateCov(4) = attitudeObserveModel.getR()(1,1);
  stateCov(5) = attitudeObserveModel.getR()(2,2);
  stateCov(6) = gpsSpeedObserveModel.getR()(0,0);
  stateCov(7) = gpsSpeedObserveModel.getR()(1,1);
  stateCov(8) = gpsSpeedObserveModel.getR()(2,2);
  //  stateCov(9) = vyawStdDev_*vyawStdDev_;
  stateCov(9) = 0;

  JFR_TRACE_BEGIN;
  slam.setRobotState(state,stateCov);
  JFR_TRACE_END;
  slam.getFilter().setTime(timeStamp);
}

void KarmaManager::addGPSData(double timeStamp_, 
			      double x_, double y_, double z_,
			      double vx_, double vy_, double vz_)
{
  jblas::vec data(3);
  JFR_TRACE_BEGIN;
  predictTo(timeStamp_);
  JFR_TRACE_END;
  data(0) = x_;
  data(1) = y_;
  data(2) = z_;
//   JFR_DEBUG("KarmaManager::addGPSData: " << slam.robotState);
//   JFR_DEBUG("KarmaManager::addGPSData: " << slam.robotStateCov);
  JFR_TRACE_BEGIN;
  double md = slam.getFilter().computeMahalanobisDistance(gpsPoseObserveModel, 0, data);

  slam.getFilter().update(gpsPoseObserveModel, 0, data);
  if (md < 6.25) {
    //  slam.getFilter().update(gpsPoseObserveModel, 0, data);
  } else {
    JFR_DEBUG("\nWARNING: gps pose inconsistant (md=" << md << ") ************************");
  }
  JFR_TRACE_END;
//   JFR_DEBUG("KarmaManager::addGPSData: " << slam.robotState);
//   JFR_DEBUG("KarmaManager::addGPSData: " << slam.robotStateCov);
  data(0) = vx_;
  data(1) = vy_;
  data(2) = vz_;
  JFR_TRACE_BEGIN;
  double md = slam.getFilter().computeMahalanobisDistance(gpsSpeedObserveModel, 6, data);
  
  slam.getFilter().update(gpsSpeedObserveModel, 6, data);

  if (md < 6.25) {
    //  slam.getFilter().update(gpsSpeedObserveModel, 6, data);
  } else {
    JFR_DEBUG("\nWARNING: gps speed inconsistant (md=" << md << ") ************************");
  }

  JFR_TRACE_END;
//   gpsPoseData.push_back(timeStampData(timeStamp_, data_));
//   update();
}

void KarmaManager::addAttitudeData(double timeStamp_, double yaw_, double pitch_, double roll_)
{
  jblas::vec data(3);
  data(0) =  jmath::toMinusPiPi(yaw_);
  data(1) = pitch_;
  data(2) = roll_;
  JFR_TRACE_BEGIN;
  predictTo(timeStamp_);
  JFR_TRACE_POINT;
  slam.getFilter().update(attitudeObserveModel, 3, data);
  JFR_TRACE_END;
  slam.getFilter().getX()(3) = jmath::toMinusPiPi(slam.getFilter().getX()(3));
  
//   attitudeData.push_back(timeStampData(timeStamp_, data_));
//   update();
}

void KarmaManager::addLandmarkManually(jafar::hpm::vecHarrisPoints& points_,
				       unsigned int index_,
				       jblas::vec const& state_, 
				       jblas::sym_mat const& stateCov_) 
{
  unsigned int id = observationsManager.getNewFeatureId();
  points_[index_].id = id;
  slam.addLandmarkManually(id, Observation::POINT_BEARING, state_, stateCov_);
}


void KarmaManager::addImageData(double timeStamp_, int imageFrameNumber_, jafar::hpm::vecHarrisPoints& points_)
{
  JFR_TRACE_BEGIN;
  predictTo(timeStamp_);
  slamHelper.nextObservationFrame();
  JFR_TRACE_POINT;
  observationsManager.manageObservations(imageFrameNumber_, points_);
  JFR_TRACE_POINT;
  try {
    observationsManager.slamObserve(slam);
  }
  catch (SlamException const& e) {
    if (e.getExceptionId() == SlamException::UNKNOWN_FEATURE) {
      JFR_DEBUG("\nKarmaManager::addImageData: feature ignored");
    }
    else throw;
  }
  JFR_TRACE_POINT;
  slam.stepTrajectory();
  JFR_TRACE_POINT;
  slam.manageTentativeFeatures();
  JFR_TRACE_END;
}

void KarmaManager::saveCurrentMap(std::string const& filename_) const
{
  map.writeCalife(filename_);
}

void KarmaManager::test()
{
  ofstream ofile("qweEdges.dat");
  map.triangulation.printEdges(ofile);
  ofile.close();
}

// void KarmaManager::update() 
// {
//   if (gpsPoseData.empty() ||
//       gpsSpeeddata.empty() ||
//       attitudeData.empty()) return;

//   // find oldest data

// }

// /*
//  * class KarmaPredictModel
//  */

// KarmaPredictModel::KarmaPredictModel() :
//   LinearPredictModel(13),
//   jerkVar(3)
// {
//   Q.clear();
//   F.assign(jblas::identity_mat(13));
// }

// KarmaPredictModel::~KarmaPredictModel() {}

// void KarmaPredictModel::setAttitudeStdDev(double yawAccStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_)
// {
//   yawAccVar = yawAccStdDev_*yawAccStdDev_;
//   pitchSpeedVar = pitchSpeedStdDev_*pitchSpeedStdDev_;
//   rollSpeedVar = rollSpeedStdDev_ * rollSpeedStdDev_;
// }

// void KarmaPredictModel::setJerkStdDev(double xJerkStdDev_, double yJerkStdDev_, double zJerkStdDev_)
// {
//   jerkVar(0) = xJerkStdDev_*xJerkStdDev_;
//   jerkVar(1) = yJerkStdDev_*yJerkStdDev_;
//   jerkVar(2) = zJerkStdDev_*zJerkStdDev_;
// }

// void KarmaPredictModel::setTimeStep(double dt)
// {
//   timeStep = dt;

//   double dt2 = dt*dt;
//   double dt2_2 = dt2/2;

//   F(0,6)  = dt;
//   F(0,9)  = dt2_2;
//   F(1,7)  = dt;
//   F(1,10) = dt2_2;
//   F(2,8)  = dt;
//   F(2,11) = dt2_2;
//   F(3,12) = dt;

//   //  JFR_DEBUG("F: \n" << jmath::prettyFormat(F));

//   // global position
//   // pose
//   double p0 = pow(dt,6)/36;
// // double p0 = pow(dt,6)/4;
//   // pose / speed
//   double p01 = pow(dt,5)/12;
// //   double p01 = pow(dt,5)/2;

//   // pose / acceleration
//   double p02 = pow(dt,4)/6;
// //   double p02 = pow(dt,4)/2;
//   // speed
//   double p1 = pow(dt,4)/4; 
// //   double p1 = pow(dt,4); 
//   // speed/acceleration
//   double p12 = pow(dt,3)/2;
// //   double p12 = pow(dt,3);
//   // acceleration
//   double p2 = pow(dt,2);

//   Q(0,0) = p0*jerkVar(0);
//   Q(0,6) = p01*jerkVar(0);
//   Q(0,9) = p02*jerkVar(0);
//   Q(6,6) = p1*jerkVar(0);
//   Q(6,9) = p12*jerkVar(0);
//   Q(9,9) = p2*jerkVar(0);

//   Q(1,1)   = p0*jerkVar(1);
//   Q(1,7)   = p01*jerkVar(1);
//   Q(1,10)  = p02*jerkVar(1);
//   Q(7,7)   = p1*jerkVar(1);
//   Q(7,10)  = p12*jerkVar(1);
//   Q(10,10) = p2*jerkVar(1);

//   Q(2,2)   = p0*jerkVar(2);
//   Q(2,8)   = p01*jerkVar(2);
//   Q(2,11)  = p02*jerkVar(2);
//   Q(8,8)   = p1*jerkVar(2);
//   Q(8,11)  = p12*jerkVar(2);
//   Q(11,11) = p2*jerkVar(2);

//   // attitude
//   double a0 = pow(dt,4)/4;
//   double a01 = pow(dt,3)/2;
//   double a1 = dt2;
//   Q(3,3) = a0*yawAccVar;
//   Q(3,12) = a01*yawAccVar;
//   Q(12,12) = a1*yawAccVar;

//   Q(4,4) = dt2*pitchSpeedVar;
//   Q(5,5) = dt2*rollSpeedVar;

//   //  JFR_DEBUG("Q: \n" << jmath::prettyFormat(Q));
// }

#endif // HAVE_TTL
