
#ifndef _BASE_SLAM_HPP_
#define _BASE_SLAM_HPP_

#include <vector>
#include <list>
#include <jmath/jblas.hpp>

namespace jafar {
  namespace geom {
    class T3DEuler;
  }
  namespace slam {
    class BaseRobot;
    class Observation;
    class SlamEventListener;
    /**
     * @ingroup slam
     * Base class for Slam algorithms that can be used by managers.
     */
    class BaseSlam {
      public:
        virtual ~BaseSlam();
        virtual void addRobot( BaseRobot* ) = 0;
        virtual void addRobot( BaseRobot*, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov ) = 0;
        virtual void predict(unsigned int _robotId, jblas::vec const& u) = 0;
        virtual bool hasFeature(unsigned int id, unsigned int robotId_ = 0) const = 0;
        virtual void getRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId = 0) const = 0;
        virtual void removeFeature(unsigned int id) = 0;
        virtual void addEventListener(SlamEventListener& listener) = 0;
        virtual void removeEventListener(SlamEventListener& listener) = 0;
        virtual void processObservations(unsigned int frameIndex_,
             std::list<Observation*> const& knownObs,
             std::list<Observation*> const& newObs, unsigned int robotId_ = 0) = 0;
        virtual void processObservations(unsigned int frameIndex_,
             std::vector<Observation*> const& knownObs,
             std::vector<Observation*> const& newObs, unsigned int robotId_ = 0) = 0;

    };
  }
}

#endif // _BASE_SLAM_HPP_
