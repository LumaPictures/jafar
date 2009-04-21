/* $Id$ */

#ifndef SLAM_ABSTRACT_MAP_MANAGER_HPP
#define SLAM_ABSTRACT_MAP_MANAGER_HPP

#include "jmath/jblas.hpp"

namespace jafar {
  namespace slam {

    class SlamEkf;
    class DefaultMapManager;
    class LocalMapManager;
    class GlobalMapManager;
    class AbstractMapObject;
    
    /** Abstract map manager. The map manager is basically responsible
     * for allocating map objects state.
     *
     * \ingroup slam
     */
    class AbstractMapManager {

    protected:

      SlamEkf& m_slam;

    public:

      AbstractMapManager(SlamEkf& slam) : m_slam(slam) {};

      virtual ~AbstractMapManager() {};

      virtual void clear() = 0;
      virtual void setMapObjectState(AbstractMapObject& mapObj) = 0;
      virtual void removeMapObject(AbstractMapObject const& mapObj) = 0;

    };

  }
}

#endif // SLAM_ABSTRACT_MAP_MANAGER_HPP
