/* $Id$ */

#ifndef SLAM_KARMA_HPP
#define SLAM_KARMA_HPP

#ifdef HAVE_TTL

#include "jmath/jblas.hpp"
#include "filter/observeModel.hpp"
#include "slam/bearingOnlySlam.hpp"
#include "slam/robot.hpp"
#include "slam/observationsManager.hpp"
#include "slam/map3d.hpp"

namespace jafar {
  namespace slam {
    
    /**
     * \ingroup slam
     */
    class GPSPoseObserveModel : public jafar::filter::LinearObserveModel {
    
    public:

      GPSPoseObserveModel();
      virtual ~GPSPoseObserveModel();

      void setNoise(double xStdDev_, double yStdDev_, double zStdDev_);
    };

    /**
     * \ingroup slam
     */
    class GPSSpeedObserveModel : public jafar::filter::LinearObserveModel {
    
    public:

      GPSSpeedObserveModel();
      virtual ~GPSSpeedObserveModel();

      void setNoise(double vxStdDev_, double vyStdDev_, double vzStdDev_);
    };

    /**
     * \ingroup slam
     */
    class AttitudeObserveModel : public jafar::filter::LinearObserveModel {

    public:

      AttitudeObserveModel();
      virtual ~AttitudeObserveModel();

      void setNoise(double yawStdDev_, double pitchStdDev_, double rollStdDev_);

      virtual jblas::vec& computeInnovation(const jblas::vec& z_, const jblas::vec& x_);
    };

//     /** A constant speed model. Orientation is supposed
//      *	constant except ("simple" model). State is
//      *	[x,y,z,yaw,pitch,roll,vx,vy,vz].
//      *
//      * \ingroup slam
//      */
//     class KarmaPredictModel : public jafar::filter::LinearPredictModel {

//     protected:

//       /// variance of linear accelerations
//       jblas::vec accVar;

//       /// variance of  yaw acceleration
//       double yawSpeedVar;

//       /// variance of pitch speed
//       double pitchSpeedVar;

//       /// variance of roll speed
//       double rollSpeedVar;

//     public:

//       KarmaPredictModel();

//       virtual ~KarmaPredictModel();

//       /// set standard deviation on attitude model
//       void setAttitudeStdDev(double yawSpeedStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_);

//       /// set standard deviation on acceleration (m/s^2)
//       void setAccStdDev(double xAccStdDev_, double yAccStdDev_, double zAccStdDev_);

//       virtual void setTimeStep(double timeStep_);

//     };

    /** A constant speed model. Orientation is supposed
     *	constant except for yaw (constant speed) ("simple" model). State is
     *	[x,y,z,yaw,pitch,roll,vx,vy,vz,vyaw].
     *
     * \ingroup slam
     */
    class KarmaPredictModel : public jafar::filter::LinearPredictModel {

    protected:

      /// variance of linear accelerations
      jblas::vec accVar;

      /// variance of  yaw acceleration
      double yawAccVar;

      /// variance of pitch speed
      double pitchSpeedVar;

      /// variance of roll speed
      double rollSpeedVar;

    public:

      KarmaPredictModel();

      virtual ~KarmaPredictModel();

      /// set standard deviation on attitude model
      void setAttitudeStdDev(double yawAccStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_);

      /// set standard deviation on acceleration (m/s^2)
      void setAccStdDev(double xAccStdDev_, double yAccStdDev_, double zAccStdDev_);

      virtual void setTimeStep(double timeStep_);

    };

    /**
     * \ingroup slam
     */
    class KarmaBoSlamHelper : public LocalSlamHelper {

    protected:

      Map& map;

    public:

      KarmaBoSlamHelper(SlamEkf& slam_,
			std::size_t sizeLocalMapMax_,
			std::size_t sizeFeatureState_,
			Map& map_);

      
      virtual void cleanSlamMap();			

    };

    /**
     * \ingroup slam
     */
    class KarmaManager {

    protected:

      BearingOnlySlam& slam;
      Map map;
      KarmaBoSlamHelper slamHelper;


      BoObservationsManager observationsManager;
      KarmaPredictModel predictModel;

      GPSPoseObserveModel gpsPoseObserveModel;
      GPSSpeedObserveModel gpsSpeedObserveModel;
      AttitudeObserveModel attitudeObserveModel;

//       typedef std::pair<double, jblas::vec> timeStampData;
//       typedef std::list<timeStampData> sensorData;

//       sensorData gpsPoseData;
//       sensorData gpsSpeeddata;
//       sensorData attitudeData;

//       std::list<sensorData&> allSensorsData;
//       boost::tuple<GPSPoseObserveModel&, 
// 		   GPSSpeedObserveModel&, 
// 		   AttitudeObserveModel&> allObserveModel;


//       void update();

    public:

      KarmaManager(BearingOnlySlam& slam_,
		   unsigned int sizeLocalMap_,
		   double xmin_, double ymin_,
		   double xmax_, double ymax_);

      BearingOnlySlam& getSlam() {return slam;};
      jafar::slam::Map& getMap() {return map;};
      KarmaPredictModel& getPredictModel() {return predictModel;};
      BoObservationsManager& getObservationsManager() {return observationsManager;};
      GPSPoseObserveModel& getGPSPoseObserveModel() {return gpsPoseObserveModel;};
      GPSSpeedObserveModel& getGPSSpeedObserveModel() {return gpsSpeedObserveModel;};
      AttitudeObserveModel& getAttitudeObserveModel() {return  attitudeObserveModel;};

      void predictTo(double time_);

      void setInitState(double timeStamp_,
			double x_, double y_, double z_, 
			double yaw_, double pitch_, double roll_,
			double vx_, double vy_, double vz_);
			//			double vyaw_, double vyawStdDev_);

      void addGPSData(double timeStamp_,
		      double x_, double y_, double z_, 
		      double vx_, double vy_, double vz_);

      void addAttitudeData(double timeStamp_, 
			   double yaw_, double pitch_, double roll_);

      void addLandmarkManually(jafar::hpm::vecHarrisPoints& points_,
			       unsigned int index_,
			       jblas::vec const& state_,
			       jblas::sym_mat const& stateCov_);

      void addImageData(double timeStamp_, int imageFrameNumber_, jafar::hpm::vecHarrisPoints& points_);

      void saveCurrentMap(std::string const& filename_) const;

      void test();

    };

//     /** A constant acceleration model. Orientation is supposed
//      *	constant except for yaw ("simple" model). State is
//      *	[x,y,z,yaw,pitch,roll,vx,vy,vz,ax,ay,az,vyaw].
//      *
//      * \ingroup slam
//      */
//     class KarmaPredictModel : public jafar::filter::LinearPredictModel {

//     protected:

//       /// variance of linear accelerations
//       jblas::vec jerkVar;

//       /// variance of  yaw acceleration
//       double yawAccVar;

//       /// variance of pitch speed
//       double pitchSpeedVar;

//       /// variance of roll speed
//       double rollSpeedVar;


//     public:

//       KarmaPredictModel();

//       virtual ~KarmaPredictModel();

//       /// set standard deviation on attitude model
//       void setAttitudeStdDev(double yawAccStdDev_, double pitchSpeedStdDev_, double rollSpeedStdDev_);

//       /// set standard deviation on jerk (m/s^3)
//       void setJerkStdDev(double xJerkStdDev_, double yJerkStdDev_, double zJerkStdDev_);

//       virtual void setTimeStep(double timeStep_);

//     };

  }
}


#endif // HAVE_TTL
#endif // SLAM_KARMA_HPP
