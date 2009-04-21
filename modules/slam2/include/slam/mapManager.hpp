/* $Id$ */

#ifndef SLAM_MAP_MANAGER_HPP
#define SLAM_MAP_MANAGER_HPP

#include <list>

#include "jmath/jblas.hpp"

#include "slam/abstractMapManager.hpp"
#include "slam/feature.hpp"
#include "slam/slamEvents.hpp"

namespace jafar {
  namespace slam {

    class SlamEkf;
    
    /**
     * This class acts as a collector of free state in the SLAM. It should
     * be used by \ref AbstractMapManager that want to reuse free state
     * for next landmarks.
     * \ingroup slam
     */
    class FreeStateCollector {
        struct StateBlock {
          StateBlock(std::size_t index_, std::size_t size_) : index(index_), size(size_) {}
          std::size_t index;
          std::size_t size;
          inline bool operator <(StateBlock const& block2) const {
            return index < block2.index;
          }
        };

        /// container type for free state blocks
        typedef std::list<StateBlock> StateBlockCont;
        /// free substates of the filter state, to store new features
        StateBlockCont m_freeStateBlocks;
        SlamEkf& m_slam;
      public:
        FreeStateCollector(SlamEkf& slam_);
        ~FreeStateCollector();
        /**
         * @return  a free index of a given size (@p sizeState)
         */
        std::size_t getFreeIndex(std::size_t sizeState);
        /**
         * Collect some index, mark it as free.
         * @param index the position in the state
         * @param sizeState the size of the state that is to be freed.
         */
        void collectIndex( std::size_t index, std::size_t sizeState);
        /**
         * Reset the collector.
         */
        void clear();
        /**
         * This function will reorder block, and merge them.
         */
        void defrag();
    };

    /** Default map manager. It allocates feature state using filter
     * softresize() method.
     * 
     * \ingroup slam
     */
    class DefaultMapManager : public AbstractMapManager {
  
    public:

      DefaultMapManager(SlamEkf& slam_) : AbstractMapManager(slam_) {}

      void clear();
      void setMapObjectState(AbstractMapObject& mapObj);
      void removeMapObject(AbstractMapObject const& mapObj);

    };

    /** Local slam manager. The size of the map is fixed (and so the
     * computation time). An unobserved feature is deleted.
     *
     * \ingroup slam
     */
    class LocalMapManager : public AbstractMapManager, public SlamEventAdapter {

    protected:

      /// state size of a feature
      const std::size_t m_sizeFeatureState;

      const std::size_t m_sizeLocalMapMax;

      /// free substates of the filter state, to store new features
      FreeStateCollector m_freeStateCollector;

     public:

      LocalMapManager(SlamEkf& slam_,
		      std::size_t sizeLocalMapMax,
		      std::size_t sizeFeatureState);
      ~LocalMapManager();

      void clear();
      void setMapObjectState(AbstractMapObject& mapObj);
      void removeMapObject(AbstractMapObject const& mapObj);

      void endProcessObservations(unsigned int robotId_);
    };

#ifndef SWIG
    namespace details {

      struct VoxelCoord {
	int x,y,z;
      };

      std::ostream& operator <<(std::ostream& s, VoxelCoord  const& v);

      inline bool operator <(VoxelCoord const& v1, VoxelCoord const& v2)
      {
	if (v1.x != v2.x)
	  return v1.x < v2.x;
	else if (v1.y != v2.y)
	  return v1.y < v2.y;
	else
	  return v1.z < v2.z;
      }
    } // namespace details
#endif // SWIG

    /** Global map manager. It has two main functions: 
     * - first, the density of the landmarks in the map is kept low
     * thanks to a voxel mechanism
     * - second, it garbage-collects subvectors of the filter state
     * and use them to add new landmarks
     *
     * \ingroup slam
     */
    class GlobalMapManager : public AbstractMapManager, public SlamEventAdapter {

    protected:

      /// minimum number of observations for a landmark to be kept in the map
      unsigned int m_nbObsMin;

      double m_sizeVoxel;
//       unsigned int nbLandmarkPerVoxel;

      /// world decomposition in voxels
      typedef std::map<details::VoxelCoord, BaseFeature*> VoxelsType;
      VoxelsType m_voxels;
#if 1
      details::VoxelCoord getVoxelCoord(BaseFeature const& f);
#endif
      FreeStateCollector m_freeStateCollector;
    public:

      GlobalMapManager(SlamEkf& slam_, 
		       double sizeVoxel_,
		       unsigned int nbObsMin);
      GlobalMapManager(SlamEkf& slam_);
      ~GlobalMapManager();

      void setParams(double sizeVoxel_ = 1.0, unsigned int nbObsMin = 5);
      void clear();
      void setMapObjectState(AbstractMapObject& mapObj);
      void removeMapObject(AbstractMapObject const& mapObj);

      void endProcessObservations(unsigned int robotId_);
    };

    /**
     * Strong map manager. It only keep "strong" feature,
     * features that get observed and that have a small uncertainty.
     *
     * @ingroup slam
     */
    class StrongMapManager : public AbstractMapManager, public SlamEventAdapter {
      public:
        StrongMapManager(SlamEkf& slam_);
        ~StrongMapManager();
        /**
         * @param minUncertainty minimal uncertainty on the maximal eigen value, a feature
         *                       below that threshold will be rejected if there isn't enough
         *                       observations
         * @param nbObsMin defines the minimal number of observations
         */
        void setParams(double minUncertainty = 0.01, unsigned int nbObsMin = 5);
        void clear();
        void setMapObjectState(AbstractMapObject& mapObj);
        void removeMapObject(AbstractMapObject const& mapObj);

        void endProcessObservations(unsigned int robotId_);
      private:
        FreeStateCollector m_freeStateCollector;
        double m_minUncertainty;
        unsigned int m_nbObsMin;
    };
 }
}

#endif // SLAM_MAP_MANAGER_HPP
