/* $Id: */

#ifndef _MAP_MANAGER_FACTORY_HPP_
#define _MAP_MANAGER_FACTORY_HPP_

#include <cstddef>

namespace jafar {
  namespace slam {
    class AbstractMapManager;
    class SlamEkf;
    class MapManagerFactory {
      public:
        virtual ~MapManagerFactory();
        virtual AbstractMapManager* createMapManager( SlamEkf& slam_ ) const = 0;
    };
    class DefaultMapManagerFactory : public MapManagerFactory {
      public:
        virtual ~DefaultMapManagerFactory();
        virtual AbstractMapManager* createMapManager( SlamEkf& slam_ ) const;
    };
    class LocalMapManagerFactory : public MapManagerFactory {
      public:
        LocalMapManagerFactory( std::size_t _sizeLocalMapMax, std::size_t _sizeFeatureState);
        virtual ~LocalMapManagerFactory();
        virtual AbstractMapManager* createMapManager( SlamEkf& slam_ ) const;
      private:
        std::size_t m_sizeLocalMapMax;
        std::size_t m_sizeFeatureState;
    };
    class GlobalMapManagerFactory : public MapManagerFactory {
      public:
        GlobalMapManagerFactory( double _sizeVoxel, unsigned int _nbObsMin);
        virtual ~GlobalMapManagerFactory();
        virtual AbstractMapManager* createMapManager( SlamEkf& slam_ ) const;
      private:
        double m_sizeVoxel;
        unsigned int m_nbObsMin;
    };
    class StrongMapManagerFactory : public MapManagerFactory {
      public:
        StrongMapManagerFactory( double minUncertainty = 0.01, unsigned int nbObsMin = 5);
        virtual ~StrongMapManagerFactory();
        virtual AbstractMapManager* createMapManager( SlamEkf& slam_ ) const;
      private:
        double m_minUncertainty;
        unsigned int m_nbObsMin;
    };
  }
}

#endif
