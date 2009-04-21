/* $Id$ */

#ifndef SLAM_BEARING_ONLY_SLAM_HPP
#define SLAM_BEARING_ONLY_SLAM_HPP

#include <string>
#include <map>
#include <vector>
#include <list>
#include <set>

#include "kernel/jafarException.hpp"

#include "jmath/jblas.hpp"

#include "slam/slamEkf.hpp"
#include "slam/bearingOnlyFeature.hpp"
#include "slam/slamEvents.hpp"

namespace jafar {
  namespace slam {

    class KarmaManager;

    /** A pose from the trajectory.
     *
     * \ingroup slam
     */
    class TrajectoryPoseRecord {
    public:

      TrajectoryPoseRecord(unsigned int frameIndex_, unsigned int robotId_, std::size_t trajIndex_) :
        frameIndex(frameIndex_),
        robotId(robotId_),
        trajIndex(trajIndex_),
        initFeatures(),
        obsFeatures()
      {}

      unsigned int frameIndex;
  
      unsigned int robotId;

      std::size_t trajIndex;

      typedef std::set<unsigned int> FeaturesIdType;

      FeaturesIdType initFeatures;
      FeaturesIdType obsFeatures;

      unsigned int distanceToPrevious;

    };
    
    std::ostream& operator <<(std::ostream& s, TrajectoryPoseRecord const& tpr);

    /** This operator compares TrajectoryPoseRecord. It is used to
     * find the pose to throw away.
     *
     * \ingroup slam
     */
    bool operator< (TrajectoryPoseRecord const& p1, TrajectoryPoseRecord const& p2);

    struct TrajectoryPoseRecordKey {
      TrajectoryPoseRecordKey(unsigned int frameIndex_, unsigned int robotId_) :
          frameIndex(frameIndex_), robotId(robotId_)
      {
      }
      unsigned int frameIndex;
      unsigned int robotId;
    };
    bool operator< (TrajectoryPoseRecordKey const& k1, TrajectoryPoseRecordKey const& k2);
    bool operator== (TrajectoryPoseRecordKey const& k1, TrajectoryPoseRecordKey const& k2);
    std::ostream& operator <<(std::ostream& s, TrajectoryPoseRecordKey const& tpr);
    
    /** Bearing only slam.
     * 
     * @warning only works with one robot
     *
     * \ingroup slam
     */
    class BearingOnlySlam : public SlamEkf, public SlamEventAdapter {
    private:
      std::size_t m_sizeTrajectory;
    public:
      typedef std::map<TrajectoryPoseRecordKey, TrajectoryPoseRecord> TrajectoryPoseRecordsType;
    
    protected:

      std::vector<jblas::vec_range> trajectory;
      std::vector<jblas::sym_mat_range> trajectoryCov;
//       std::vector<jblas::sym_mat_range> trajectoryCrossCov;
      std::vector<std::size_t> trajectoryStateIndex;
      std::vector<bool> trajectoryIsRobot;

      std::list<std::size_t> trajectoryEmptyIndexes;

      std::map< unsigned int, std::size_t> robotIdToTrajectoryIndex;

      TrajectoryPoseRecordsType trajectoryPoseRecords;

      bool hasTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_ = 0 ) const;
      TrajectoryPoseRecord const& getTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_ = 0) const;
      TrajectoryPoseRecord& getTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_ = 0);
 
      void eraseTrajectoryPoseRecord(unsigned int frameIndex, unsigned int robotId_ = 0);
      void eraseTrajectoryPoseRecord(TrajectoryPoseRecordsType::iterator it);

      void updateTrajectoryWithRemovedFeature(InitFeature const& f );

      typedef std::map<int, BearingOnlyFeatureObserveModel*> BoFeatureObserveModelsContType;

      /// Observe models for bearing only features
      BoFeatureObserveModelsContType boFeatureObserveModels;

      typedef std::map<int, InfFeatureObserveModel*> InfFeatureObserveModelsContType;

      /// Observe models for features at infinity
      InfFeatureObserveModelsContType infFeatureObserveModels;

      /// parameter for the depth initialization (min)
      double dMin;
      /// parameter for the depth initialization (max)
      double dMax;

      /// False alarm probability
      double sprtAlphaFA;
      /// Missed detection probability
      double sprtAlphaMD;

      /// sprt upper bound
      double logA;
      /// sprt lower bound
      double logB;

      typedef std::list<BoSlamEventListener*> BoSlamEventListenersList;
      BoSlamEventListenersList boSlamEventListeners;

      void manageTrajectory(unsigned int robotId_ = 0);
      void backupCurrentPose(std::size_t index, unsigned int _robotPose = 0);

      void observeNewFeature(Observation* obs_);

      void observeKnownFeature(Observation* obs_);

      void observeInitFeature(InitFeature& feature, Observation* obs);

      virtual void beginProcessObservations(unsigned int robotId_ ) {
	// add trajectory pose record for current pose
	TrajectoryPoseRecord tpr(currentFrameIndex, robotId_, robotIdToTrajectoryIndex[ robotId_ ] );
	trajectoryPoseRecords.insert(std::make_pair(TrajectoryPoseRecordKey(currentFrameIndex, robotId_),tpr));
	JFR_DEBUG("BearingOnlySlam::beginProcessObservations: add: " << tpr);
	JFR_DEBUG(getTrajectoryPoseRecord(currentFrameIndex, robotId_));
      }

      virtual void endProcessObservations(unsigned int robotId_ ) {
	cleanInitFeatures();
	manageTrajectory(robotId_);
      }

      /// init the state of bearing-only feature
      void initState(InitFeature& feature, Observation const& obs);

      /// update the init state of bearing-only feature
      void updateInitState(InitFeature& feature, Observation const& obs);


      /// compute hypothesis log-likelihood
      static double computeInitStateHypothesisLogLikelihood(InitStateMember& ism,
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
							    jblas::mat& Jxf);

      /// during initialisation steps, unobserved features are deleted
      void cleanInitFeatures();

      /** Manage the Sum Of Gaussians.
       * @return true if \a iFeature still exists.
       */
      bool manageInitState(InitFeature& iFeature);

      /// proceed with all operations to add a feature to the map
      void fullStateInit(InitFeature& iFeature,
                         InitStateMember const& state);

      void fullStateInit(InitFeature& iFeature,
                         jblas::vec const& x,
			 jblas::sym_mat const& P,
			 bool atInfinity = false);

      /// for logging this info
      unsigned int nbInitFeaturesLost;

      virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
      virtual void writeLogData(jafar::kernel::DataLogger& log) const;

//       void logFeature(InitFeature& f);

    public:

      typedef std::map<unsigned int, InitFeature*> InitFeaturesMapType;

      InitFeaturesMapType initFeatures;

      BearingOnlySlam(std::size_t sizeMax_,
		      std::size_t sizeRobotState_,
                      std::size_t sizeRobotPose_,
                      std::size_t sizeTrajectory_);

      ~BearingOnlySlam();

      /// clear the map and the robot pose
      virtual void init(boost::posix_time::time_duration const& curTime);
      virtual void addRobot( BaseRobot* );

      /// @return the number of poses estimated by the slam (current pose + past poses)
      std::size_t sizeTrajectory() const {return trajectory.size();}

      jblas::vec_range const& getTrajectoryPose(unsigned int frameIndex, unsigned int robotId_ = 0) const;

      /// add a bearing-only sensor
      void setBoSensor(BearingOnlyFeatureObserveModel* model, int sensorId = 0);

      /// add a bearing-only sensor wich takes into account landmarks at infinity 
      void setBoSensor(BearingOnlyFeatureObserveModel* model, InfFeatureObserveModel* modelInf, int sensorId = 0);

      /** Set the \a robotToSensor transformation of sensor \a
       * sensorId.
       */
      virtual void setRobotToSensor(jblas::vec const& robotToSensor, int sensorId = 0);

      /** Set the \a robotToSensor transformation along with its
       * uncertainty of sensor \a sensorId.
       */
      virtual void setRobotToSensor(jblas::vec const& robotToSensor, jblas::vec const& robotToSensorStdDev, 
				    int sensorId = 0);      


      bool hasBoFeatureObserveModel(int sensorId) {
	return boFeatureObserveModels.find(sensorId) != boFeatureObserveModels.end();
      }

      BearingOnlyFeatureObserveModel& getBoFeatureObserveModel(int sensorId = 0) {
	BoFeatureObserveModelsContType::iterator it = boFeatureObserveModels.find(sensorId);
	JFR_PRED_ERROR(it != boFeatureObserveModels.end(),
		       SlamException,
		       SlamException::UNKNOWN_SENSOR,
		       "BearingOnlySlam::getBoFeatureObserveModel: Unknown sensor: " << sensorId);
	return *(it->second);
      }

      bool hasInfFeatureObserveModel(int sensorId) {
	return infFeatureObserveModels.find(sensorId) != infFeatureObserveModels.end();
      }

      InfFeatureObserveModel& getInfFeatureObserveModel(int sensorId) {
	InfFeatureObserveModelsContType::iterator it = infFeatureObserveModels.find(sensorId);
	JFR_PRED_ERROR(it != infFeatureObserveModels.end(),
		       SlamException,
		       SlamException::UNKNOWN_SENSOR,
		       "BearingOnlySlam::getInfFeatureObserveModel: Unknown sensor: " << sensorId);
	return *(it->second);
      }

      BaseFeatureObserveModel& getBoInfFeatureObserveModel(int sensorId, bool infinity) {
	if (infinity)
	  return getInfFeatureObserveModel(sensorId);
	else
	  return getBoFeatureObserveModel(sensorId);
      }

      TrajectoryPoseRecordsType const& getTrajectoryPoseRecords() const {return trajectoryPoseRecords;}

      void addBoEventListener(BoSlamEventListener& listener) {
	addEventListener(listener);
	boSlamEventListeners.push_back(&listener);
	JFR_DEBUG("BearingOnlySlam::addBoSlamEventListener: " << boSlamEventListeners.size() << " listeners");
      }

      bool hasInitFeature(unsigned int id_) const {
        return (initFeatures.find(id_) != initFeatures.end());
      };

      virtual bool isFeatureKnown(unsigned int id) const {
	return hasInitFeature(id) || hasFeature(id);
      };

//       virtual void addLandmarkManually(unsigned int id_, Observation::ObservationType featureType_, 
// 				       jblas::vec const& featureState_, jblas::sym_mat const& stateCov_);


      InitFeature& getInitFeature(unsigned int id) const {
	InitFeaturesMapType::const_iterator feature_it = initFeatures.find(id);
	if (feature_it != initFeatures.end())
	  return *(feature_it->second);
	else 
	  JFR_ERROR(SlamException, SlamException::UNKNOWN_FEATURE,
		    "BearingOnlySlam::getInitFeature: unknown feature " << id);
      }

      virtual void removeFeature(unsigned int id);
      void removeInitFeature(unsigned int id);
      void removeInitFeature(InitFeaturesMapType::iterator featureIt);

      // easy to use with swig/tcl
      InitFeaturesMapType::iterator browseInitFeaturesIt;
      void beginBrowseInitFeatures();
      bool hasNextInitFeature();
      InitFeature* nextInitFeature();

      void setBoParam(double dMin_=0.5, double dMax_=100,  double sprtAlphaFA_=0.01,  double sprtAlphaMD_=0.05);

      void predict(unsigned int id, 
		   jblas::vec const& u);

      friend std::ostream& operator <<(std::ostream& s, const BearingOnlySlam& slam_);
      friend class KarmaManager;
//       friend std::string boSegFeatureInitState(InitFeature const& f_, BearingOnlySlam& slam_);
      friend std::string boSlamTrajectory(BearingOnlySlam const& slam);
    }; // class BearingOnlySlam

    std::ostream& operator <<(std::ostream& s, const BearingOnlySlam& slam_);

  } // namespace slam
} // namespace jafar
#endif // SLAM_BEARING_ONLY_SLAM_HPP
