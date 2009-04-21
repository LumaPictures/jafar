
#include "slam/MapManagerFactory.hpp"
#include "slam/mapManager.hpp"

using namespace jafar::slam;

MapManagerFactory::~MapManagerFactory()
{
}

DefaultMapManagerFactory::~DefaultMapManagerFactory()
{
}

AbstractMapManager* DefaultMapManagerFactory::createMapManager( SlamEkf& slam_ ) const
{
  return new DefaultMapManager( slam_ );
}

LocalMapManagerFactory::LocalMapManagerFactory( std::size_t _sizeLocalMapMax, std::size_t _sizeFeatureState) : m_sizeLocalMapMax(_sizeLocalMapMax), m_sizeFeatureState(_sizeFeatureState)
{
}

LocalMapManagerFactory::~LocalMapManagerFactory()
{
}

AbstractMapManager* LocalMapManagerFactory::createMapManager( SlamEkf& slam_ ) const
{
  return new LocalMapManager( slam_, m_sizeLocalMapMax, m_sizeFeatureState );
}

GlobalMapManagerFactory::GlobalMapManagerFactory( double _sizeVoxel, unsigned int _nbObsMin) : m_sizeVoxel(_sizeVoxel), m_nbObsMin(_nbObsMin)
{
}

GlobalMapManagerFactory::~GlobalMapManagerFactory()
{
}

AbstractMapManager* GlobalMapManagerFactory::createMapManager( SlamEkf& slam_ ) const
{
  return new GlobalMapManager( slam_, m_sizeVoxel, m_nbObsMin);
}


StrongMapManagerFactory::StrongMapManagerFactory( double minUncertainty, unsigned int _nbObsMin) : m_minUncertainty(minUncertainty), m_nbObsMin(_nbObsMin)
{
}

StrongMapManagerFactory::~StrongMapManagerFactory()
{
}

AbstractMapManager* StrongMapManagerFactory::createMapManager( SlamEkf& slam_ ) const
{
  StrongMapManager* manager = new StrongMapManager( slam_ );
  manager->setParams(m_minUncertainty, m_nbObsMin);
  return manager;
}

