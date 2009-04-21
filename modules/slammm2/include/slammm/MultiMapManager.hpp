/* $Id$ */

#ifndef _MULTI_MAPS_MANAGER_HPP_
#define _MULTI_MAPS_MANAGER_HPP_

#include <kernel/dataLog.hpp>
#include <geom/t3dEuler.hpp>
#include <slam/slamEkf.hpp>
#include <slam/robot.hpp>
#include <slam/baseSlam.hpp>
#include <slam/MapManagerFactory.hpp>

#include "slammm/StartNewMapStrategy.hpp"
#include "slammm/MergeMapsStrategy.hpp"

namespace jafar {
  namespace slammm {
    class StartNewMapStrategy;
    class MultiMapsSlam;
    class MapsPoseManager;
    
    /**
     * @ingroup slammm
     */
    class MultiMapsSlam : public slam::BaseSlam, public kernel::DataLoggable  {
      typedef std::map<unsigned int, slam::FeatureObserveModel*> FeatureObserveModelsContType;
      public:
        /**
         * @param sizeMax_ the maximum size of the state of a local map
         */
        MultiMapsSlam(std::size_t sizeMax_, MapsPoseManager* _mapsPoseManager, StartNewMapStrategy* _startNewMapStrategy, MergeMapsStrategy* _mergeMapsStrategy, slam::MapManagerFactory* mapManagerFactory = 0 );
        ~MultiMapsSlam();
        /**
         * Process observations, it will create a new map if needed
         */
        template<class SequenceType1, class SequenceType2>
        void processObservationsImpl(unsigned int frameIndex_,
              SequenceType1 const& knownObs,
              SequenceType2 const& newObs, unsigned int _robotId = 0);
        
        /**
         * Merge the map of a robot into the map of an other map
         */
        void mergeMap( unsigned int _map1Id, unsigned int _map2Id, const geom::T3DEuler& transfo );
        /**
         * Function that return the current map
         */
        slam::SlamEkf* getCurrentMap( unsigned int _robotId = 0 );
        const slam::SlamEkf* getCurrentMap( unsigned int _robotId = 0 ) const;
        /**
         * @return a map
         */
        slam::SlamEkf* getMap( unsigned int _mapId, unsigned int _robotId = 0 );

        /**
         * @return the identifier of a map
         */
        unsigned int mapId( slam::SlamEkf* ) const;

        /**
         * Function that return the global position of the robot.
         */
        void getWorldRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId = 0 ) const;
        jafar::geom::T3DEuler getWorldRobotPose( unsigned int _robotId = 0 ) const;
        
        /**
         * @return the state of the feature in the world reference.
         */
        void getWorldFeatureState(jblas::vec& _result , unsigned int _featureId, unsigned int _currentMapId ) const;
        /**
         * @return the covariance of the feature in the world reference
         */
        void getWorldFeatureState(jblas::vec& _result, jblas::sym_mat& _resultCov, unsigned int _featureId, unsigned int _currentMapId ) const;
        /**
         * Get the transformation from the World frame to current map
         */
        jafar::geom::T3DEuler getWorldToCurrentMapTransformation( unsigned int _robotId = 0 ) const;
        /**
         * Get the transformation from the World frame to a given map
         * @param _mapiId 
         */
        jafar::geom::T3DEuler getWorldToMapTransformation( unsigned int _mapId ) const;

        void beginBrowseMaps();
        bool hasNextMap();
        slam::SlamEkf* nextMap();
        
        const std::map< unsigned int, slam::SlamEkf*> getMaps() const;
        
        /**
         * Merge all maps in one global map, using a similar method as Divide & Conquer. The maps of
         * \ref MultiMapsSlam are not touched and remain valid after a call to this function.
         */
        slam::SlamEkf* getGlobalMap( ) const;
        void addRobot( slam::BaseRobot* );
        void addRobot( slam::BaseRobot*, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov );
        void setSensor(slam::FeatureObserveModel* model, unsigned int sensorId, unsigned int _robotId );
        void setRobotToSensor(jblas::vec const& robotToSensor, int sensorId, unsigned int _robotId );
        // Implementation of BaseSlam functions
        virtual void predict(unsigned int _robotId, jblas::vec const& u);
        virtual bool hasFeature(unsigned int id, unsigned int _robotId = 0) const;
        virtual void getRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId = 0) const;
        virtual void removeFeature(unsigned int id);
        virtual void addEventListener(slam::SlamEventListener& listener);
        virtual void removeEventListener(slam::SlamEventListener& listener);
        virtual void processObservations(unsigned int frameIndex_,
             std::list<slam::Observation*> const& knownObs,
             std::list<slam::Observation*> const& newObs, unsigned int robotId_ = 0);
        virtual void processObservations(unsigned int frameIndex_,
             std::vector<slam::Observation*> const& knownObs,
             std::vector<slam::Observation*> const& newObs, unsigned int robotId_ = 0);
        template<typename SequenceType>
        void processObservations(unsigned int frameIndex_,
                                 SequenceType const& allObs, unsigned int robotId_ = 0);
      public:
        // Functions related to events
        /**
         * Call this function when a rendez-vous event happen between two robots and
         * that the tranformation between the two robots is known.
         *
         * This function will start a new map for each robot.
         */
        void rendezVous( unsigned int _firstRobotId, unsigned int _secondRobotId, const geom::T3DEuler& firstToSecond );
        /**
         * Call this function when the position of multiple robots is known.
         *
         * This function will start a new map for each robot.
         */
        void gpsFix( const std::map<unsigned int, geom::T3DEuler>& worldToRobots );
        /**
         * Convenient function that will execute a GPS fix event for a single robot.
         *
         * This function will start a new map for each robot.
         */
        void gpsFix( unsigned int _robotId, const geom::T3DEuler& worldToRobot );
        /**
         * Close the loop between two maps
         */
        void closeLoop( unsigned int _firstMapId, unsigned int _secondMapId, const geom::T3DEuler& firstToSecond );
      public:
        virtual void writeLogHeader(kernel::DataLogger& log) const;
        virtual void writeLogData(kernel::DataLogger& log) const;
        virtual void addMembersToLog(kernel::DataLogger& log) const;
        /**
         * Log events that happen in the \ref MultiMapsSlam
         */
        void setEventsDataLogger( kernel::DataLogger* log );
      private:
        struct RobotMaps;
        void initNewMap( unsigned int _robotId);
        void startNewMap( RobotMaps& );
        /**
         * Update all world transformation that are cached in RobotMaps
         */
        void updateCacheWorldToCurrentMap();
      private:
        struct RobotMaps {
          RobotMaps( ) : currentMapId(-1), currentMap(0), baseRobot(0)
          {
          }
          unsigned int currentMapId;
          slam::SlamEkf* currentMap;
          std::map< unsigned int, slam::SlamEkf*> maps;
          jafar::geom::T3DEuler worldToCurrentMap;
          slam::BaseRobot* baseRobot;
          /// Observe models for simple features
          FeatureObserveModelsContType featureObserveModels;
          std::map<int, jblas::vec> robotToSensors;
        };
        int nextMapId;
        /// This structure contains a filter and various information.
        struct SlamEkfInfo {
          slam::SlamEkf* slamEkf;
          /**
           * This variable contains the "age" of the robot position used
           * in this filter. A higher value indicates a newer position.
           */
          std::map< unsigned int, unsigned int > robotPositionAge;
        };
        std::map< unsigned int, SlamEkfInfo> m_maps;
        std::map<unsigned int, RobotMaps> m_robotMaps;
        MapsPoseManager* m_mapsPoseManager;
        StartNewMapStrategy* m_startNewMapStrategy;
        MergeMapsStrategy* m_mergeMapsStrategy;
        slam::MapManagerFactory* m_mapManagerFactory;
        std::size_t m_sizeMax;
        std::list<unsigned int > m_sensorsIds; ///< list of sensors ids, this list is used by setSensor to ensure that the id of a sensor is unique
        std::list<slam::SlamEventListener*> m_listeners;
        std::map< unsigned int, SlamEkfInfo>::iterator browseFeaturesIt;
        kernel::DataLogger* m_eventsLogger;
        unsigned int m_frameIndex;
    };
    template<class SequenceType1, class SequenceType2>
    void MultiMapsSlam::processObservationsImpl(unsigned int frameIndex_,
          SequenceType1 const& knownObs,
          SequenceType2 const& newObs, unsigned int _robotId )
    {
      // Get the robot
      std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( _robotId );
      JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
      // Process observation
      it->second.currentMap->processObservations( frameIndex_, knownObs, newObs, _robotId );
      // Decide wether to start a new map
      JFR_DEBUG( "================================== Nb of features = " << it->second.currentMap->featuresMap.size() << " new obs = " << newObs.size());
      if(m_startNewMapStrategy->startNewMap( *it->second.currentMap, _robotId, newObs.size() ) )
      {
        startNewMap( it->second );
      } else {
        m_mergeMapsStrategy->decideMergeMap( it->second.currentMapId, it->second.currentMap );
      }
    }
    
    template<class SequenceType>
    void MultiMapsSlam::processObservations(unsigned int frameIndex_, SequenceType const& allObs, unsigned int _robotId ) 
    {
      m_frameIndex = frameIndex_;
      // Get the robot
      std::map<unsigned int, RobotMaps>::iterator itRobot = m_robotMaps.find( _robotId );
      JFR_ASSERT( itRobot != m_robotMaps.end(), "No robot " << _robotId );

      SequenceType knownObs;
      SequenceType newObs;

      for (typename SequenceType::const_iterator it = allObs.begin() ; it != allObs.end() ; ++it) {
        if ( itRobot->second.currentMap->isFeatureKnown((**it).id) )
          knownObs.push_back(*it);
        else
          newObs.push_back(*it);
      }

      processObservations(frameIndex_, knownObs, newObs, _robotId);

    }
  }
}

#endif
