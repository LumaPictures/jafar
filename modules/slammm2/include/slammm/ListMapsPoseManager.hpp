/* $Id$ */

#ifndef _LIST_MAPS_POSE_MANAGER_HPP_
#define _LIST_MAPS_POSE_MANAGER_HPP_

#include "slammm/MapsPoseManager.hpp"

namespace jafar {
  namespace slammm {
    /**
     * Store the pose of each maps in a list.
     */
    class ListMapsPoseManager : public MapsPoseManager {
      public:
        ListMapsPoseManager( std::size_t maxStateSize );
        virtual ~ListMapsPoseManager();
        virtual void addFirstMapPose(unsigned int _mapId, const jblas::vec& _robotState, const jblas::mat& _robotStateCov );
        virtual void addMapPose(unsigned int _mapId, unsigned int _previousMapId, const jblas::vec& _robotState, const jblas::mat& _robotStateCov );
        virtual jafar::geom::T3DEuler getWorldToMapTransformation( unsigned int _mapId ) const;
        virtual jafar::geom::T3DEuler getMapToMapTransformation( unsigned int _firstMapId, unsigned int _secondMapId ) const;
        virtual void enforceLoopConstraint( unsigned int _firstMapId, unsigned int _secondMapId, const jblas::vec& _transformation, const jblas::sym_mat& _transformationCov );
        virtual void writeLogHeader(kernel::DataLogger& log) const;
        virtual void writeLogData(kernel::DataLogger& log) const;
        void displayConnections() const;
      private:
        /**
         * Get the transformation to a map
         * @param _secondMapId id of the second map (if possible)
         * @return true if the transformation is global
         */
        void getToMapTransformation( jafar::geom::T3DEuler& transfo, unsigned int _mapId, unsigned int _secondMapId ) const;
      private:
        struct SCBInfo;
        struct ELCInfo;
        struct MapPoseInfo;
        struct MapPoseInfoEdge {
          MapPoseInfoEdge( const ublas::range& _range, jblas::vec& _state, jblas::sym_mat& _cov) : range(_range), state(_state, _range), cov(_cov, _range, _range) {}
          ~MapPoseInfoEdge() {
          }
          MapPoseInfo* previousMap;
          MapPoseInfo* nextMap;
          ublas::range range;
          jblas::vec_range state; ///< point to the transformation between one map and the previous map
          jblas::sym_mat_range cov;
          void getTransfo( geom::T3DEuler& transfo, MapPoseInfo* info1, MapPoseInfo* info2);
          MapPoseInfo* theOtherOne( MapPoseInfo* );
          bool connectWith( MapPoseInfo* _map1, MapPoseInfo* _map2 );
          bool isCertainelyNull() const;
        };
        struct MapPoseInfo {
          unsigned int mapId;
          std::list< MapPoseInfoEdge* > edges;
        };
        typedef std::list< std::pair<MapPoseInfo*, MapPoseInfoEdge*> > CycleType;
        MapPoseInfoEdge* createEdge(int robotStateSize);
        /**
         * @return the smallest loop between two maps (or an empty list if there isn't such a loop)
         */
        CycleType smallestCycleBetween( MapPoseInfo* _firstMap, MapPoseInfo* _secondMap ) const;
        /**
         * @return true if the two maps in argument are connected
         */
        bool mapAreConnected( MapPoseInfo* _map1, MapPoseInfo* _map2 );
        std::list<MapPoseInfoEdge*> m_mapPoseEdgeInfos; ///< list of edges for memory management purposes
        std::map<unsigned int, MapPoseInfo* > m_mapPoseInfos;
        jblas::vec m_state;
        jblas::sym_mat m_cov;
        int m_nextFreeState;
        MapPoseInfo* globalFrameInfo;
    };
  }
}

#endif
