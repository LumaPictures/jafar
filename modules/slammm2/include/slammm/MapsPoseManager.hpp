/* $Id$ */

#ifndef _MAPS_POSE_MANAGER_HPP_
#define _MAPS_POSE_MANAGER_HPP_

#include <kernel/dataLog.hpp>
#include <geom/t3dEuler.hpp>
#include <slam/slamEkf.hpp>
#include <slam/robot.hpp>
#include <slam/baseSlam.hpp>
#include <slam/MapManagerFactory.hpp>

#include "slammm/StartNewMapStrategy.hpp"

namespace jafar {
  namespace slammm {
    class StartNewMapStrategy;
    /**
     * Global Level
     */
    class MapsPoseManager : public kernel::DataLoggable {
      public:
        virtual ~MapsPoseManager() {}
        /**
         * Add the pose of the first map in the world.
         */
        virtual void addFirstMapPose(unsigned int _mapId,const jblas::vec& _robotState, const jblas::mat& _robotStateCov ) = 0;
        /**
         * Add the pose of a map in reference to an other map.
         */
        virtual void addMapPose(unsigned int _mapId, unsigned int _previousMapId, const jblas::vec& _robotState, const jblas::mat& _robotStateCov ) = 0;
        /**
         * @return the transformation from the given map to the world.
         */
        virtual jafar::geom::T3DEuler getWorldToMapTransformation( unsigned int _mapId ) const = 0;
        /**
         * @return the transformation between two maps. It the transformation of robot2/map2 expressed
         * in robot1/map1 frame.
         */
        virtual jafar::geom::T3DEuler getMapToMapTransformation( unsigned int _firstMapId, unsigned int _secondMapId ) const = 0;
        /**
         * Enforce loop constraints when a transformation between two maps is known.
         * 
         * @param _transformation the transformation from the first map to the second map
         * @param _transformationCov 
         */
        virtual void enforceLoopConstraint( unsigned int _firstMapId, unsigned int _secondMapId, const jblas::vec& _transformation, const jblas::sym_mat& _transformationCov ) = 0;
        virtual void enforceLoopConstraint( unsigned int _firstMapId, unsigned int _secondMapId, const geom::T3DEuler& _transformation );
    };
  }
}

#endif
