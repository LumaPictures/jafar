/* $Id$ */

#ifndef SLAM_SLAM_EKF_HPP
#define SLAM_SLAM_EKF_HPP

#include <string>
#include <vector>
#include <map>

#include "kernel/jafarException.hpp"
#include "kernel/timingTools.hpp"
#include "kernel/dataLog.hpp"

#include "jmath/jblas.hpp"

#include "geom/t3dEuler.hpp"

#include "filter/kalmanFilter.hpp"

#include "slam/baseSlam.hpp"
#include "slam/slamException.hpp"
#include "slam/mapManager.hpp"
#include "slam/feature.hpp"
#include "slam/pointFeature.hpp"
#include "slam/segmentFeature.hpp"
#include "slam/segmentInvDepthFeature.hpp"
#include "slam/slamEvents.hpp"
#include "slam/poseCopy.hpp"
#include "slam/robot.hpp"

namespace jafar {
	namespace slam {
		class BaseRobot;
		/** This class solves SLAM problem using the EKF-based academic
		* solution. The pose of the robot is represented using Euler
		* angle in a vector \f$ [x,y,z,yaw,pitch,roll] \f$.
		*
		* \todo add robot model, also with constraints (for attitude quaternion for instance) 
		* \todo free space management in the state vector ! (when the removeLandmark function is used)
		*
		* \ingroup slam
		*/
		class SlamEkf : public BaseSlam, public jafar::kernel::DataLoggable {

		private:

			/** init value of feature state, covariance and cross covariance.
			*/
			void initFeatureState(BaseFeature& feature, 
						FeatureObserveModel& observeModel,       
						Observation const& obs);

		protected:
			bool m_deleteSensors;

			unsigned int currentFrameIndex;
			unsigned int prevFrameIndex;

			jafar::filter::BlockExtendedKalmanFilter filter;      

			double mahalanobisDistanceToRemoveLandmark;

			std::size_t m_sizeRobotState;
			std::size_t m_sizeRobotPose;

//       jblas::vec_range m_refPose;
//       jblas::sym_mat_range m_refPoseCov;

			AbstractMapManager* mapManager;

			typedef std::list<SlamEventListener*> SlamEventListenersList;
			SlamEventListenersList slamEventListeners;

			/// FIXME not implemented yet
			bool enableInitCoordinateFrame; 
			
			typedef std::map<unsigned int, FeatureObserveModel*> FeatureObserveModelsContType;
			/// Observe models for simple features
			FeatureObserveModelsContType featureObserveModels;

			/** This method adds a new feature to the map.
			* A new feature can be initialized only with an observation with the same 
			* dimension as the state of the feature.
			*/
			virtual void observeNewFeature(Observation* obs_);
			
		/** This method performs observation update to a feature in the map.
			*/
			virtual void observeKnownFeature(Observation* obs_);

		/** This method performs observation update to a feature in the map.
			*/
			void observeKnownFeature(Observation const& obs, BaseFeature& feature, BaseFeatureObserveModel& model);

			/// apply constraints of \a feature
			void applyConstraints(BaseFeature& feature);

			/// Make feature satisfy a constraint.
			void fixFeature(BaseFeature& feature);

			//       void applyAllConstraints();

		/// Copy current robot pose at given map location and size
			void copyCurrentPoseAt(std::size_t index, int _robotId = 0);

			/// relation between the robot frame and the slam (ref) frame
			geom::T3D* m_refToRobot;
			geom::T3D* m_robotToRef;

			static BaseFeature* featureFactory(unsigned int id,
					BaseFeatureObserveModel& model);
			static BaseFeature* featureFactory(unsigned int id,
						FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs );

			void writePoseCopyToFile(PoseCopy const& pc, std::string const& dir);

			// for data logging
			unsigned short int nbInconsistentUpdates;
			unsigned short int nbNewLandmarks;
			unsigned short int nbObservedLandmarks;
			long updateElapsedTime;      

			virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
			virtual void writeLogData(jafar::kernel::DataLogger& log) const;

			jafar::kernel::DataLogger* p_masterLogger;

			void logFeature(BaseFeature& f);

		public:

			typedef std::map<unsigned int, BaseFeature*> FeaturesMapType;
			/// Current map of features
			FeaturesMapType featuresMap;

			typedef std::map<unsigned int, PoseCopy*> PoseCopyContType;
			/// estimated local frames
			PoseCopyContType poseCopyCont;
			
			typedef std::map<unsigned int, BaseRobot*> RobotsMapType;
			/// List of robots inside the map
			RobotsMapType robotsMap;

		/// Constructor
			SlamEkf(std::size_t sizeMax_, std::size_t sizeRobotState_, std::size_t sizeRobotPose_);

		/// Destructor
			virtual ~SlamEkf();

			/// clear the map and the robot pose
			virtual void init(boost::posix_time::time_duration const& curTime);

		/// Return robot's pose size
			std::size_t sizeRobotPose() const {return m_sizeRobotPose;}
		
		/// Returns robot's state size
			std::size_t sizeRobotState() const {return m_sizeRobotState;}

		/// Return robot pose
			jblas::vec_range const& refPose(int _robotId = 0) const;
		
		/// Return robot pose covariances matrix
			jblas::sym_mat_range const& refPoseCov(int _robotId = 0) const;

			void enableAutoLogFeatures(jafar::kernel::DataLogger& masterLogger) {p_masterLogger = &masterLogger;}

		/// get the SLAM filter
			jafar::filter::BlockExtendedKalmanFilter& getFilter() {return filter;};
		
		/// get the SLAM filter
			jafar::filter::BlockExtendedKalmanFilter const& getFilter() const {return filter;};

			/// Return the map of features
			FeaturesMapType const& getMap() const {return featuresMap;}

			/// Return the map of features
			FeaturesMapType& getMap() {return featuresMap;}

			jafar::geom::T3D const& refToRobot() const {return *m_refToRobot;}
			jafar::geom::T3D const& robotToRef() const {return *m_robotToRef ;}
			void setRefToRobot(jafar::geom::T3DEuler const& refToRobot_);

			void setRobotPose(jblas::vec const& robotPose, int _robotId = 0);
			void setRobotPoseCov(jblas::vec const& robotPose, jblas::sym_mat const& robotPoseCov, int _robotId = 0);

//       BaseFeatureObserveModel& getBaseFeatureObserveModel(int sensorId) {
// 	BaseFeatureObserveModelsContType::iterator it = baseFeatureObserveModels.find(sensorId);
// 	JFR_PRED_ERROR(it != baseFeatureObserveModels.end(),
// 		       SlamException,
// 		       SlamException::UNKNOWN_SENSOR,
// 		       "Unknown sensor: " << sensorId);
// 	return *(it->second);
//       }

			FeatureObserveModel& getFeatureObserveModel(unsigned int sensorId = 0) const 
			{
				FeatureObserveModelsContType::const_iterator it = featureObserveModels.find(sensorId);
				JFR_PRED_ERROR(it != featureObserveModels.end(), SlamException,
											SlamException::UNKNOWN_SENSOR, "Unknown sensor: " << sensorId);
				return *(it->second);
			}

			unsigned int getCurrentFrameIndex() const {return currentFrameIndex;}
			unsigned int getPrevFrameIndex() const {return prevFrameIndex;}

			void setMapManager(AbstractMapManager* mapManager_);

			/**
			* Set wether the SlamEkf can delete \ref FeatureObserveModel pass
			* as argument of \ref setSensor .
			*/
			void setDeleteSensors( bool );
			/** Add a sensor with its feature model. Enable to deal with
			*  multiple sensors.
			*/
			void setSensor(FeatureObserveModel* model, int sensorId = 0);

//       void setRobotState(jblas::vec const& state_);

//       void setRobotState(jblas::vec const& state_, jblas::vec const& stateCov_);

			void setRobotZ(double z, double zStdDev, int _robotId = 0);

			void setRobotPitch(double pitch, double pitchStdDev, int _robotId = 0);

			void setRobotRoll(double roll, double rollStdDev, int _robotId = 0);

			/** Set the \a robotToSensor transformation of sensor \a
			* sensorId.
			*/
			virtual void setRobotToSensor(jblas::vec const& robotToSensor, int sensorId = 0);
			const jblas::vec& robotToSensor( int sensorId = 0) const;

			/** Set the \a robotToSensor transformation along with its
			* uncertainty of sensor \a sensorId.
			*/
			virtual void setRobotToSensor(jblas::vec const& robotToSensor, jblas::vec const& robotToSensorStdDev, 
						int sensorId = 0);

			virtual void addEventListener(SlamEventListener& listener) {
				JFR_ASSERT( &listener != 0, "Can't add null listener");
				slamEventListeners.push_back(&listener);
				//JFR_DEBUG("SlamEkf::addEventListener: " << slamEventListeners.size() << " listeners");
			}

			virtual void removeEventListener(SlamEventListener& listener) {
				slamEventListeners.remove(&listener);
			}

			bool hasFeature(unsigned int id, unsigned int _robotId = 0) const {
				return featuresMap.find(id) != featuresMap.end();
			};

			virtual bool isFeatureKnown(unsigned int id) const {
				//JFR_DEBUG("id :" << id);
				return hasFeature(id);
			};

			//       virtual void addLandmarkManually(AbstractMapObject::IdType id_, Observation::ObservationType featureType_, 
			// 				       jblas::vec const& featureState_, jblas::sym_mat const& stateCov_);

			BaseFeature& getFeature(unsigned int id) {
				FeaturesMapType::const_iterator feature_it = featuresMap.find(id);
				if (feature_it != featuresMap.end())
					return *(feature_it->second);
				else
				{
					JFR_DEBUG( "SlamEkf::getFeature: unknown feature " << id );
					JFR_ERROR(SlamException, SlamException::UNKNOWN_FEATURE,
							"SlamEkf::getFeature: unknown feature " << id);
				}
			}

			BaseFeature const& getFeature(unsigned int id) const {
				FeaturesMapType::const_iterator feature_it = featuresMap.find(id);
				if (feature_it != featuresMap.end())
					return *(feature_it->second);
				else
					JFR_ERROR(SlamException, SlamException::UNKNOWN_FEATURE,
							"SlamEkf::getFeature: unknown feature " << id);
			}

			/// \todo Implement with a BaseFeature& in parameter
			virtual void removeFeature(unsigned int id);

			/** This method copy the current reference pose to the map, along
			* with the appropriate correlations.
			*/
			PoseCopy const& copyCurrentRefPoseToMap(unsigned int id);

			/** Return the current estimate of the poseCopy \a id
			*/
			PoseCopy const& getPoseCopy(unsigned int id) const {
				PoseCopyContType::const_iterator poseIt = poseCopyCont.find(id);
				if (poseIt != poseCopyCont.end())
					return *(poseIt->second);
				else
					JFR_ERROR(SlamException, SlamException::UNKNOWN_POSE_COPY,
							"SlamEkf::getPoseCopy: unknown pose copy " << id);
			}

			/// Remove poseCopy \a id
			void removePoseCopy(unsigned int id);

			/** Write poseCopy \a id in a t3d file.
			* write in dir/prefix_id.t3d
			*/
			void writePoseCopyToFile(unsigned int id, std::string const& dir);
			void writeAllPoseCopyToFile(std::string const& dir);

			double getRobotPoseUncertaintyLevel(unsigned int robotId_ = 0) const;

			void getRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId = 0) const {
				if (refToRobot().isIdentity()) {
					pose.set(refPose(_robotId), refPoseCov(_robotId));
				}
				else {
					geom::T3DEuler worldToRef(refPose(_robotId), refPoseCov(_robotId));
					geom::T3D::compose(worldToRef, refToRobot(), pose);
				}
			}

			/**
			* This function will transform the position of the landmarks, and of
			* the robot with the given transformation.
			*/
			void transform( const geom::T3DEuler& _transform );

//       jblas::vec_range const& getRobotState() const {return robotState;}

			/**
			* @return number of features in the map.
			*/
			std::size_t sizeMap() const {return featuresMap.size();}
			double getMapUncertaintyLevel() const;

			// easy to use with swig/tcl
			FeaturesMapType::iterator browseFeaturesIt;
			void beginBrowseFeatures();
			bool hasNextFeature();
			BaseFeature* nextFeature();
			/**
			* @return the feature of the given \a id , or 0, if there is no feature \a id
			*/
			BaseFeature* feature( unsigned int id );

			/**
			* Compute the prediction for the movement of a robot.
			* @param _robotId id of the robot
			* @param u command
			*/
			virtual void predict(unsigned int _robotId, jblas::vec const& u);
			
			/**
			* @return the robot corresponding to the id.
			*/
			BaseRobot* robot( unsigned int _id );
			/**
			* @return the robot corresponding to the id.
			*/
			const BaseRobot* robot( unsigned int ) const;
			
			/**
			* Add a new \ref BaseRobot in the map. The function will
			* initialize the state of the robot in the filter.
			*/
			virtual void addRobot( BaseRobot* );
			virtual void addRobot( BaseRobot*, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov );

			template<class SequenceType1, class SequenceType2>
			void processObservationsImpl(unsigned int frameIndex_,
						SequenceType1 const& knownObs,
						SequenceType2 const& newObs, unsigned int robotId_ = 0);

			virtual void processObservations(unsigned int frameIndex_,
					std::list<Observation*> const& knownObs,
					std::list<Observation*> const& newObs, unsigned int robotId_ = 0)
			{
				processObservationsImpl( frameIndex_, knownObs, newObs, robotId_ );
			}
			virtual void processObservations(unsigned int frameIndex_,
					std::vector<Observation*> const& knownObs,
					std::vector<Observation*> const& newObs, unsigned int robotId_ = 0)
			{
				processObservationsImpl( frameIndex_, knownObs, newObs, robotId_ );
			}

			template<class SequenceType>
			void processObservations(unsigned int frameIndex_,
						SequenceType const& allObs, unsigned int robotId_ = 0);
			
			/** Compute Extremites Observation 
			/// @param _robotId Robot Identifier
			/// @param segmentFeature_ Segment Landmark. CAUTION this function it is only needed in the case of segments.
			/// @param zPredExt1 and @param zPredExt2 Extremities Observation
			/// @param zPredExt1Cov and @param zPredExt2Cov Extremities Observation Covariances**/
// //      void computeExtObs(Observation* obs, jblas::vec2& zPredExt1, jblas::sym_mat& zPredExt1Cov, jblas::vec2& zPredExt2, jblas::sym_mat& zPredExt2Cov);
//       void computeExtObs(int robotId_, SegmentFeature& segmentFeature_, ImagePluckerFeatureObserveModel& obsModel, jblas::vec& zPredExt1, jblas::sym_mat& zPredExt1Cov, jblas::vec& zPredExt2, jblas::sym_mat& zPredExt2Cov);
//       void computeExtObs(int robotId_, SegmentFeature& segmentFeature_, ImageEuclideanPluckerFeatureObserveModel& obsModel, jblas::vec& zPredExt1, jblas::sym_mat& zPredExt1Cov, jblas::vec& zPredExt2, jblas::sym_mat& zPredExt2Cov);
//       void computeExtObs(int robotId_, SegmentFeature& segmentFeature_, StereoImagePluckerFeatureObserveModel& obsModel, jblas::vec& zPredExt1, jblas::sym_mat& zPredExt1Cov, jblas::vec& zPredExt2, jblas::sym_mat& zPredExt2Cov);

				template<class segmentFeatureObserveModel>
				void computeExtObs(int robotId_, 
					SegmentFeature& segmentFeature_, segmentFeatureObserveModel& obsModel, jblas::vec& zPredExt1, 
					jblas::sym_mat& zPredExt1Cov, 
					jblas::vec& zPredExt2, 
					jblas::sym_mat& zPredExt2Cov);

				template<class segmentFeatureObserveModel>
				void computeExtObs(int robotId_, 
					SegmentIDFeature& segmentFeature_, segmentFeatureObserveModel& obsModel, jblas::vec& zPredExt1, 
					jblas::sym_mat& zPredExt1Cov, 
					jblas::vec& zPredExt2, 
					jblas::sym_mat& zPredExt2Cov);


			/// @return the Mahalanobis for this observation (already matched)
//       double computeMahalanobisDistance(Observation const& obs_);

			/// @return the Mahalanobis distance between \a feature_ and \a obs_
//       double computeMahalanobisDistance(const BaseFeature& feature_, const Observation& obs_);

			/**
			* Merge two maps, and creates a new one.
			*/
			enum MergeMapNumber {
				MAP_1,
				MAP_2
			};
			
			/**
			* @param _chooseRobotPose a map that indicates the merge which map should be used
			*        to select the robot pose
			*/
			static SlamEkf* mergeMap( const SlamEkf* map1, const SlamEkf* map2, const jblas::vec& map1ToMap2, const jblas::mat& map1ToMap2Cov, const std::map< unsigned int, MergeMapNumber >& _chooseRobotPose );

			friend std::ostream& operator <<(std::ostream& s, const SlamEkf& slam_);
			friend class DefaultMapManager;
			friend class LocalMapManager;
			friend class GlobalMapManager;

		}; // class SlamEkf

		std::ostream& operator <<(std::ostream& s, const SlamEkf& slam_);


//###############################################################################################
// Templates Implementation
//###############################################################################################


		template<class SequenceType1, class SequenceType2>
		void slamEkf::processObservationsImpl(unsigned int frameIndex_,
					SequenceType1 const& knownObs,
					SequenceType2 const& newObs, unsigned int robotId_) 
		{
			nbInconsistentUpdates = 0;
			nbNewLandmarks = 0;
			nbObservedLandmarks = 0;

			prevFrameIndex = currentFrameIndex;
			currentFrameIndex = frameIndex_;
				//JFR_DEBUG("RobotId:" << robotId_);
			for(SlamEventListenersList::iterator it = slamEventListeners.begin() ; it != slamEventListeners.end() ; ++it) {
				(*it)->beginProcessObservations(robotId_);
			}

			kernel::Chrono chrono;

			for (typename SequenceType1::const_iterator it = knownObs.begin() ; it != knownObs.end() ; ++it) {
				observeKnownFeature(*it);
				delete *it;
			}

			//FIXME temp, change to real constraint for id
			/*const double eps = 1e-6;
			for (typename SequenceType1::const_iterator it = knownObs.begin() ; it != knownObs.end() ; ++it) {
			BaseFeature &feat = getFeature((*it)->id);
			if (feat.getX()(6) < eps)
			{
			JFR_WARNING("Negative estimated inverse depth " << feat.getX()(6) << " corrected to " << eps << " updating X and P");
			double n = eps - feat.getX()(6);
			feat.getX()(6) = eps;
			feat.getP()(6,6) += n*n;
			}

			delete *it;
			}*/

			for (typename SequenceType2::const_iterator it = newObs.begin() ; it != newObs.end() ; ++it) {
				observeNewFeature(*it);
			}

			// 	applyAllConstraints();

			updateElapsedTime = chrono.elapsed();

			for(SlamEventListenersList::iterator it = slamEventListeners.begin() ; it != slamEventListeners.end() ; ++it) {
				(**it).endProcessObservations(robotId_);
			}
		}


		template<class SequenceType>
		void slamEkf::processObservations(unsigned int frameIndex_,
					SequenceType const& allObs, unsigned int robotId_ = 0) 
		{
			SequenceType knownObs;
			SequenceType newObs;
			
			for (typename SequenceType::const_iterator it = allObs.begin() ; it != allObs.end() ; ++it) {
				if ( isFeatureKnown((**it).id) )
					knownObs.push_back(*it);
				else
					newObs.push_back(*it);
			}
			processObservations(frameIndex_, knownObs, newObs, robotId_);
		}
		
		
		template<class segmentFeatureObserveModel>
		void slamEkf::computeExtObs(int robotId_, 
			SegmentIDFeature& segmentFeature_, segmentFeatureObserveModel& obsModel, jblas::vec& zPredExt1, 
			jblas::sym_mat& zPredExt1Cov, 
			jblas::vec& zPredExt2, 
			jblas::sym_mat& zPredExt2Cov)
		{
			//JFR_PRECOND(,
			//        "SlamEkf::predictExtremitiesObservation");
			BaseRobot* baseRobot = robot(robotId_ );
			//   SegmentFeature& segmentFeature_ = (SegmentFeature&)getFeature(obs->id);
			//   ImagePluckerFeatureObserveModel& obsModel =  (ImagePluckerFeatureObserveModel&) getFeatureObserveModel(obs->sensorId);
			
			ublas::range rr(baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizePose());
			ublas::range ff(segmentFeature_.filterIndex(), segmentFeature_.filterIndex()+ segmentFeature_.sizeState());
			
			jblas::mat Je1R(2,6);
			jblas::mat Je2R(2,6);
			jblas::mat Je1L(2,11);
			jblas::mat Je2L(2,11);
			jblas::mat Je1Ext1(2,3);
			jblas::mat Je2Ext2(2,3);
			
			jblas::sym_mat_range robotSegmentCrossCov(filter.getP(), rr, ff);
			
			obsModel.predictExtObsJac(*baseRobot->refPose(),segmentFeature_.getExt1(), zPredExt1, Je1R, Je1Ext1);
			obsModel.predictExtObsJac(*baseRobot->refPose(),segmentFeature_.getExt2(), zPredExt2, Je2R, Je2Ext2);
			//  JFR_DEBUG("Extremities zPredExt1:" << zPredExt1);
			//  JFR_DEBUG("Extremities zPredExt1:" << zPredExt2);
			
			Je1L.assign(ublas::prod(Je1Ext1,segmentFeature_.getExt1Jac()));
			Je2L.assign(ublas::prod(Je2Ext2,segmentFeature_.getExt2Jac()));
			
			//  JFR_DEBUG("Jacobians:" << Je1L);
			//  JFR_DEBUG("Jacobians:" << Je2L);
			
			zPredExt1Cov.assign(ublas::prod(Je1R,jblas::mat(ublas::prod(*baseRobot->refPoseCov(),trans(Je1R))))+ublas::prod(Je1L,jblas::mat(prod(segmentFeature_.getP(),ublas::trans(Je1L))))+ ublas::prod(Je1R,jblas::mat(ublas::prod(robotSegmentCrossCov,trans(Je1L))))+ ublas::prod(Je1L,jblas::mat(prod(trans(robotSegmentCrossCov),ublas::trans(Je1R)))));
			
			zPredExt2Cov.assign(ublas::prod(Je2R,jblas::mat(ublas::prod(*baseRobot->refPoseCov(),trans(Je2R))))+ublas::prod(Je2L,jblas::mat(prod(segmentFeature_.getP(),ublas::trans(Je2L))))+ ublas::prod(Je2R,jblas::mat(ublas::prod(robotSegmentCrossCov,trans(Je2L))))+ ublas::prod(Je2L,jblas::mat(prod(trans(robotSegmentCrossCov),ublas::trans(Je2R)))));
			
			//  JFR_DEBUG("Jacobians:" << Je1L);
			//  JFR_DEBUG("Jacobians:" << Je2L);
		}
		
		template<class segmentFeatureObserveModel>
			void slamEkf::computeExtObs(int robotId_, 
			SegmentFeature& segmentFeature_, segmentFeatureObserveModel& obsModel, jblas::vec& zPredExt1, 
			jblas::sym_mat& zPredExt1Cov, 
			jblas::vec& zPredExt2, 
			jblas::sym_mat& zPredExt2Cov)
		{
			//JFR_PRECOND(,
			//        "SlamEkf::predictExtremitiesObservation");
			BaseRobot* baseRobot = robot(robotId_ );
			//   SegmentFeature& segmentFeature_ = (SegmentFeature&)getFeature(obs->id);
			//   ImagePluckerFeatureObserveModel& obsModel =  (ImagePluckerFeatureObserveModel&) getFeatureObserveModel(obs->sensorId);
			
			ublas::range rr(baseRobot->filterIndex(), baseRobot->filterIndex() + baseRobot->sizePose());
			ublas::range ff(segmentFeature_.filterIndex(), segmentFeature_.filterIndex()+ segmentFeature_.sizeState());
			
			jblas::mat Je1R(2,6);
			jblas::mat Je2R(2,6);
			jblas::mat Je1L(2,6);
			jblas::mat Je2L(2,6);
			jblas::mat Je1Ext1(2,3);
			jblas::mat Je2Ext2(2,3);
			
			jblas::sym_mat_range robotSegmentCrossCov(filter.getP(), rr, ff);
			
			obsModel.predictExtObsJac(*baseRobot->refPose(),segmentFeature_.getExt1(), zPredExt1, Je1R, Je1Ext1);
			obsModel.predictExtObsJac(*baseRobot->refPose(),segmentFeature_.getExt2(), zPredExt2, Je2R, Je2Ext2);
			//  JFR_DEBUG("Extremities zPredExt1:" << zPredExt1);
			//  JFR_DEBUG("Extremities zPredExt1:" << zPredExt2);
			
			Je1L.assign(ublas::prod(Je1Ext1,segmentFeature_.getExt1Jac()));
			Je2L.assign(ublas::prod(Je2Ext2,segmentFeature_.getExt2Jac()));
			
			//  JFR_DEBUG("Jacobians:" << Je1L);
			//  JFR_DEBUG("Jacobians:" << Je2L);
			
			zPredExt1Cov.assign(ublas::prod(Je1R,jblas::mat(ublas::prod(*baseRobot->refPoseCov(),trans(Je1R))))+ublas::prod(Je1L,jblas::mat(prod(segmentFeature_.getP(),ublas::trans(Je1L))))+ ublas::prod(Je1R,jblas::mat(ublas::prod(robotSegmentCrossCov,trans(Je1L))))+ ublas::prod(Je1L,jblas::mat(prod(trans(robotSegmentCrossCov),ublas::trans(Je1R)))));
			
			zPredExt2Cov.assign(ublas::prod(Je2R,jblas::mat(ublas::prod(*baseRobot->refPoseCov(),trans(Je2R))))+ublas::prod(Je2L,jblas::mat(prod(segmentFeature_.getP(),ublas::trans(Je2L))))+ ublas::prod(Je2R,jblas::mat(ublas::prod(robotSegmentCrossCov,trans(Je2L))))+ ublas::prod(Je2L,jblas::mat(prod(trans(robotSegmentCrossCov),ublas::trans(Je2R)))));
			
			//  JFR_DEBUG("Jacobians:" << Je1L);
			//  JFR_DEBUG("Jacobians:" << Je2L);
			
			
			double sumEigExtStdDev = sqrt(zPredExt1Cov(0,0)+zPredExt1Cov(1,1)+sqrt(fabs(((zPredExt1Cov(0,0)-zPredExt1Cov(1,1))*(zPredExt1Cov(0,0)-zPredExt1Cov(1,1)))-4*zPredExt1Cov(0,1)*zPredExt1Cov(0,1))));
			sumEigExtStdDev = sumEigExtStdDev + sqrt(zPredExt2Cov(0,0)+zPredExt2Cov(1,1)+sqrt(fabs(((zPredExt2Cov(0,0)-zPredExt2Cov(1,1))*(zPredExt2Cov(0,0)-zPredExt2Cov(1,1)))-4*zPredExt2Cov(0,1)*zPredExt2Cov(0,1))));
			//double sumExtCovDet=sqrt(jmath::ublasExtra::lu_det(zPredExt1Cov))+sqrt(jmath::ublasExtra::lu_det(zPredExt2Cov));
			//test to evalute if enlarge or not
			//   JFR_DEBUG("SegmentId: " << segmentFeature_.id());
			//   JFR_DEBUG("Sum Max Eigenvalues Ext: " << sumEigExtStdDev);
			
			if ( sumEigExtStdDev <= 30.0 && sumEigExtStdDev > 0.0) {
					segmentFeature_.setFlagEnlarge();
			//  JFR_DEBUG("FLAG " << segmentFeature_.ENLARGE);
			}
			
			//  JFR_DEBUG("Extremeties Covariance:" << zPredExt1Cov);
			//  JFR_DEBUG("Extremeties Covariance:" << zPredExt2Cov);
		}

	} // namespace slam
} // namespace jafar


#endif // SLAM_SLAM_EKF_HPP
