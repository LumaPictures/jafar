/* $Id$ */

#ifndef _MERGE_MAP_STRATEGY_HPP_
#define _MERGE_MAP_STRATEGY_HPP_

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
    class MultiMapsSlam;
    class MapsPoseManager;
    
    /**
     * @ingroup slammm
     * This class is used to define strategy for merging maps.
     */
    class MergeMapsStrategy {
      public:
        MergeMapsStrategy();
        virtual ~MergeMapsStrategy();
        /**
         * Set the MultiMapsSlam instance, this can only be done once, and it is usually done
         * in the MultiMapsSlam constructor.
         */
        void setMultiMapsSlam( MultiMapsSlam* );
        /**
         * @return the MultiMapsSlam managed by this strategy
         */
        MultiMapsSlam* multiMapsSlam();
        /**
         * This function is called after processing observations to decide wether or not
         * the maps should be merged.
         */
        virtual void decideMergeMap( unsigned int currentMapId, slam::SlamEkf* currentMap ) = 0;
      private:
        MultiMapsSlam* m_multiMapsSlam;
    };
    
    /**
     * @ingroup slammm
     * This \ref MergeMapsStragegy decide to merge maps whenever two non consecutive maps
     * have overlapping features.
     */
    class AlwaysMergeMapsStrategy : public MergeMapsStrategy{
      public:
        /**
         * @param minimalOverlapingFeaturesNumber the minimal number of features needed
         *          to consider that two maps overlap and should be merged.
         */
        AlwaysMergeMapsStrategy( int minimalOverlapingFeaturesNumber );
        virtual void decideMergeMap( unsigned int currentMapId, slam::SlamEkf* currentMap );
      private:
        void getFeatureCov( slam::BaseFeature& baseFeature, jblas::vec& point, jblas::sym_mat& cov );
      private:
        unsigned int m_minimalOverlapingFeaturesNumber;
        struct DMMInfo;
    };
  }
}

#endif
