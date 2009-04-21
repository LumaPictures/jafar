/* $Id$ */

#ifndef SLAM_DALA_HPP
#define SLAM_DALA_HPP

#include "kernel/dataLog.hpp"

#include "jmath/jblas.hpp"

#include "geom/t3dEuler.hpp"

#include "slam/full3dPredictModel.hpp"
#include "slam/baseSlam.hpp"
// #include "slam/MultiMapManager.hpp"

namespace jafar {
  namespace slam {

    /**
			Class for managing Dala
     * \ingroup slam
     */
    class DalaManager : public jafar::kernel::DataLoggable {

    private:

      Full3dPredictModel predictModel;

      BaseSlam& slam;

      // ego motion model;
      double kTrans;
      double kzTrans;
      double ksYaw;
      double ksAtt;
      double krYaw;

      double deltaPosMin;
      geom::T3DEuler currentRobotPosePredict;

      geom::T3DEuler worldToRobotCur;
      geom::T3DEuler worldToRobotPrev;
      geom::T3DEuler robotPrevToRobotCur;

      geom::T3DEuler worldToRobot;

      // for data logging...
      geom::T3DEuler refToWorld;
      geom::T3DEuler worldToRef;

      bool nextFrameRelativeNoUncertainty();

      bool nextFrameRelative();

      void slamPredict();
      int robotId;

    public:

      DalaManager(BaseSlam& slam_, unsigned int robotId_ = 0);
      DalaManager(BaseSlam& slam_, unsigned int robotId_, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov);

      void setDeltaPoseMin(double deltaPosMin_) {
	deltaPosMin = deltaPosMin_;
      }
	/**
		@param kTrans_ uncertainty on forward translation when moving forward (m/sqrt(m))
		@param kzTrans_ uncertainty on vertical translation when moving forward (m/sqrt(m))
		@param ksYaw_ uncertainty on Yaw-rotation when moving forward (rad/sqrt(m))
		@param ksAtt_ uncertainty on Pitch-rotation and Roll-rotation when moving forward (rad/sqrt(m))
		@param krYaw_ uncertainty on Yaw-rotation when Yaw-rotating (rad/sqrt(rad))
	*/
      void setEgoMotionModel(double kTrans_,
			     double kzTrans_,
			     double ksYaw_,
			     double ksAtt_,
			     double krYaw_) 
      {
	kTrans = kTrans_;
	kzTrans = kzTrans_;
	ksYaw = ksYaw_;
	ksAtt = ksAtt_;
	krYaw = krYaw_;
      }

      void setInitPose(jblas::vec const& worldToRobotCur_) {
	worldToRobotCur = worldToRobotCur_;
      }

      void setRefToWorld(jblas::vec const& refToWorld_) {
	refToWorld.set(refToWorld_);
	geom::T3D::inv(refToWorld, worldToRef);
      }

      bool nextFrameAbsolute(jblas::vec const& worldToRobotCur_);

      bool nextFrameRelative(jblas::vec const& robotPrevToRobotCur_)
      {
	robotPrevToRobotCur.set(robotPrevToRobotCur_);
	return nextFrameRelativeNoUncertainty();
      }

      bool nextFrameRelative(jblas::vec const& robotPrevToRobotCur_, 
			     jblas::sym_mat const& robotPrevToRobotCurCov_,
			     double noiseFactor = 1.0)
      {
	robotPrevToRobotCur.set(robotPrevToRobotCur_, noiseFactor*robotPrevToRobotCurCov_);
	return nextFrameRelative();
      }

      static bool isVmeOk(jblas::vec const& me, 
			  jblas::sym_mat const& meCov,
			  double thTrans,
			  double thRot);

	void setPredToRef(geom::T3DEuler const& predToRef)
	{
		predictModel.setPredToRef(predToRef);
	}

    protected:

      void writeLogHeader(jafar::kernel::DataLogger& dataLogger) const;
      void writeLogData(jafar::kernel::DataLogger& dataLogger) const;

    };

  }
}

#endif // SLAM_DALA_HPP
