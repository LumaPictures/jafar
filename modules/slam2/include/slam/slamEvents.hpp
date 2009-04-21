/* $Id$ */

#ifndef SLAM_SLAM_EVENTS_HPP
#define SLAM_SLAM_EVENTS_HPP

namespace jafar {
  namespace slam {

    // fwd declarations
    class BaseFeature;
    class SlamEkf;

    /** Interface for Slam listeners.  
     *
     * \ingroup slam
     */
    class SlamEventListener {

    protected:

      virtual ~SlamEventListener() {}

      virtual void addLandmark(BaseFeature const& landmark) = 0;
      virtual void removeLandmark(BaseFeature const& landmark) = 0;

      virtual void beginProcessObservations(unsigned int robotId_) = 0;
      virtual void endProcessObservations(unsigned int robotId_) = 0;

      friend class SlamEkf;

    };

    /** Adapter for Slam listeners.  
     *
     * \ingroup slam
     */
    class SlamEventAdapter : public SlamEventListener {

    protected:

      virtual ~SlamEventAdapter() {}

      virtual void addLandmark(BaseFeature const& landmark) {};
      virtual void removeLandmark(BaseFeature const& landmark) {};

      virtual void beginProcessObservations(unsigned int robotId_) {};
      virtual void endProcessObservations(unsigned int robotId_) {};

    };

    // fwd declarations
    class InitFeature;
    class BearingOnlySlam;

    /** Interface for BoSlam listeners.  
     *
     * \ingroup slam
     */
    class BoSlamEventListener : public SlamEventListener {

    protected:

      virtual ~BoSlamEventListener() {}

      virtual void addTentativeLandmark(InitFeature const& landmark) = 0;
      virtual void removeTentativeLandmark(InitFeature const& landmark) = 0;

      friend class BearingOnlySlam;

    };

    /** Adapter for BoSlam listeners.  
     *
     * \ingroup slam
     */
    class BoSlamEventAdapter : public BoSlamEventListener {

    protected:

      virtual ~BoSlamEventAdapter() {}

      virtual void addLandmark(BaseFeature const& landmark) {};
      virtual void removeLandmark(BaseFeature const& landmark) {};
      virtual void beginProcessObservations(unsigned int robotId_) {};
      virtual void endProcessObservations(unsigned int robotId_) {};
      virtual void addTentativeLandmark(InitFeature const& landmark) {};
      virtual void removeTentativeLandmark(InitFeature const& landmark) {};

    };

  } // namespace slam
} // namespace jafar

#endif // SLAM_SLAM_EVENTS_HPP
