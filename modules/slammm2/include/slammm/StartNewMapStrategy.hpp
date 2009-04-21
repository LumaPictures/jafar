/* $Id$ */

#ifndef _MULTI_START_NEW_MAP_STRATEGY_HPP_
#define _MULTI_START_NEW_MAP_STRATEGY_HPP_

#include <kernel/dataLog.hpp>
#include <geom/t3dEuler.hpp>
#include <slam/slamEkf.hpp>
#include <slam/robot.hpp>
#include <slam/baseSlam.hpp>
#include <slam/MapManagerFactory.hpp>

namespace jafar {
  namespace slammm {
    /**
     * @ingroup slammmm
     * This class control when a new map is created by the \ref MultiMapManager .
     */
    class StartNewMapStrategy {
      public:
        virtual ~StartNewMapStrategy();
        /**
         * @param _currentMap the current map
         * @param _robotId id of the current robot
         * @return true if the \ref MultiMapManager should start a new map.
         */
        virtual bool startNewMap( const slam::SlamEkf& _currentMap, unsigned int _robotId, unsigned int _newFeaturesCount) const =0;
    };
    /**
     * @ingroup slammmm
     * This \ref StartNewMapStrategy starts a new map whenever the number of features
     * in the current map is bigger than a given number of features.
     */
    class FeaturesCountStartNewMapStrategy : public StartNewMapStrategy {
      public:
        /**
         * @param _maxFeatures this is the maximal number of features that a local map should contains,
         *                     in reality the number of features can be bigger, since the test is called
         *                     after new features have been added to the local map
         */
        FeaturesCountStartNewMapStrategy( unsigned int _maxFeatures );
        virtual ~FeaturesCountStartNewMapStrategy();
        virtual bool startNewMap( const slam::SlamEkf& _currentMap, unsigned int _robotId, unsigned int _newFeaturesCount) const;
      private:
        unsigned int m_maxFeatures;
    };
  }
}

#endif
