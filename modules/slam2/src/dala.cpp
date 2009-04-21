/* $Id$ */

#include <cmath>

#include "jmath/ublasExtra.hpp"

#include "slam/dala.hpp"
#include "slam/robot.hpp"

using namespace jafar::slam;
using namespace jafar;

/*
 * class DalaManager
 */

DalaManager::DalaManager(BaseSlam& slam_, unsigned int robotId_) :
  slam(slam_),
  deltaPosMin(0.1),
  currentRobotPosePredict(true),
  worldToRobotCur(),
  worldToRobotPrev(),
  robotPrevToRobotCur(),
  worldToRobot(true),
  robotId(robotId_)
{
  currentRobotPosePredict.clear();
  worldToRobot.clear();
  refToWorld.clear();
  worldToRef.clear();
  slam.addRobot( new BaseRobot( robotId, predictModel, 6 ) );
}

DalaManager::DalaManager(BaseSlam& slam_, unsigned int robotId_, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov) :
  slam(slam_),
  deltaPosMin(0.1),
  currentRobotPosePredict(true),
  worldToRobotCur(_robotState),
  worldToRobotPrev(),
  robotPrevToRobotCur(),
  worldToRobot(true),
  robotId(robotId_ )
{
  JFR_DEBUG( "Hello");
  currentRobotPosePredict.clear();
  worldToRobot.clear();
  refToWorld.clear();
  worldToRef.clear();
  slam.addRobot( new BaseRobot( robotId, predictModel, 6 ), _robotState, _robotStateCov);
}

bool DalaManager::nextFrameAbsolute(jblas::vec const& worldToRobotCur_)
{
  worldToRobotPrev = worldToRobotCur;
  worldToRobotCur.set(worldToRobotCur_);

  geom::T3DEuler robotPrevToWorld;
  geom::T3DEuler::inv(worldToRobotPrev, robotPrevToWorld);
  geom::T3DEuler::compose(robotPrevToWorld, worldToRobotCur, robotPrevToRobotCur);

  return nextFrameRelativeNoUncertainty();
}


bool DalaManager::nextFrameRelativeNoUncertainty()
{
  JFR_PRECOND(!robotPrevToRobotCur.hasCov(),
	      "DalaManager::nextFrameRelativeNoUncertainty:");

  double deltaS = robotPrevToRobotCur.translationNorm();
//  double deltaS2 = pow(deltaS, 2);

  JFR_DEBUG("deltaS: " << deltaS);

  jblas::vec deltaX = robotPrevToRobotCur.getX();
  jblas::sym_mat xCov(6,6);

  xCov.clear();
//   xCov(0,0) = kTrans*pow(robotPrevToRobotCur.getX()(0), 2);
//   xCov(1,1) = kTrans*pow(robotPrevToRobotCur.getX()(1), 2);
//   xCov(2,2) = kTrans*pow(robotPrevToRobotCur.getX()(2), 2);
  xCov(0,0) = kTrans*kTrans  * deltaS;
  xCov(1,1) = kTrans*kTrans  * deltaS;
  xCov(2,2) = kzTrans*kzTrans * deltaS;

  xCov(3,3) = ksYaw*ksYaw*deltaS +  krYaw*krYaw * fabs(deltaX(3));
  xCov(4,4) = ksAtt*ksAtt*deltaS;
  xCov(5,5) = ksAtt*ksAtt*deltaS;

  robotPrevToRobotCur.set(deltaX, xCov);  

  return nextFrameRelative();
}

bool DalaManager::nextFrameRelative()
{
  JFR_PRECOND(robotPrevToRobotCur.hasCov(),
	      "DalaManager::nextFrameRelative:");

  geom::T3DEuler::composeIncr(currentRobotPosePredict, robotPrevToRobotCur);

  geom::T3DEuler::composeIncr(worldToRobot, robotPrevToRobotCur);

  if (currentRobotPosePredict.translationNorm() >= deltaPosMin) {
    slamPredict();
    return true;
  }
  else
    return false;
}

void DalaManager::slamPredict() {
  // predict robot pose
  JFR_PRECOND(currentRobotPosePredict.hasCov(),
	      "DalaManager::nextFrameRelative:");

  predictModel.setUCov(currentRobotPosePredict.getXCov());

  slam.predict(robotId, currentRobotPosePredict.getX());

//   vec2 odo;
//   odo(0) = currentRobotPosePredict.getEuclidienDistance();
//   odo(1) = currentRobotPosePredict.getX()(3);
//   sym_mat odoCov(2,2);
//   odoCov.clear();
//   odoCov(0,0) = currentRobotPosePredict.getXCov()(0,0)+currentRobotPosePredict.getXCov()(1,1)+currentRobotPosePredict.getXCov()(2,2);
//   odoCov(1,1) = currentRobotPosePredict.getXCov()(3,3);


  currentRobotPosePredict.clear();
}

bool DalaManager::isVmeOk(jblas::vec const& me, 
			  jblas::sym_mat const& meCov,
			  double thTrans,
			  double thRot)
{
  double dTrans = sqrt(meCov(0,0) + meCov(1,1) + meCov(2,2));
  double dRot = sqrt(meCov(3,3) + meCov(4,4) + meCov(5,5));
  return (dTrans < thTrans && dRot < thRot);
}

void DalaManager::writeLogHeader(jafar::kernel::DataLogger& log) const
{
  log.writeComment("slam: DalaManager");
  log.writeLegendTokens("x y z yaw pitch roll");
  log.writeLegendTokens("sig_x sig_y sig_z sig_yaw sig_pitch sig_roll");
  log.writeLegend("pose uncertainty volume");
}

void DalaManager::writeLogData(jafar::kernel::DataLogger& log) const
{
  geom::T3DEuler refToSensor;
  geom::T3D::compose(refToWorld, worldToRobot, refToSensor);
  geom::T3DEuler refToRobot;
  geom::T3D::compose(refToSensor, worldToRef, refToRobot);

  log.writeDataVector(refToRobot.getX());
  log.writeData(sqrt(refToRobot.getXCov()(0,0)));
  log.writeData(sqrt(refToRobot.getXCov()(1,1)));
  log.writeData(sqrt(refToRobot.getXCov()(2,2)));
  log.writeData(sqrt(refToRobot.getXCov()(3,3)));
  log.writeData(sqrt(refToRobot.getXCov()(4,4)));
  log.writeData(sqrt(refToRobot.getXCov()(5,5)));

  log.writeData( jmath::ublasExtra::lu_det(ublas::project(refToRobot.getXCov(), ublas::range(0,3), ublas::range(0,3) )) );
}

