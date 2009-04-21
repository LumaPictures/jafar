/* $Id$ */

#include <cmath>

#include "slam/mapManager.hpp"
#include "slam/slamEkf.hpp"
#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

using namespace jafar::slam;
namespace lapack = boost::numeric::bindings::lapack;


FreeStateCollector::FreeStateCollector(SlamEkf& slam_) : m_slam(slam_)
{
}

FreeStateCollector::~FreeStateCollector()
{
}

std::size_t FreeStateCollector::getFreeIndex(std::size_t sizeState)
{
  // try to find a continuous block in the free blocks
  StateBlockCont::iterator it = m_freeStateBlocks.begin();
  while (it != m_freeStateBlocks.end()) {
    if (it->size >= sizeState) {
      std::size_t index = it->index;
      it->index += sizeState;
      it->size -= sizeState;
      if (it->size == 0) m_freeStateBlocks.erase(it);
      return index;
    }
    ++it;
  }

  // if no block is found, enlarge the state vector
  std::size_t index = m_slam.getFilter().sizeState();
  m_slam.getFilter().softResize(m_slam.getFilter().sizeState() + sizeState);
  JFR_VDEBUG("FreeStateCollector::getFreeIndex: resize filter state: " << m_slam.getFilter().sizeState());
  return index;
}

void FreeStateCollector::collectIndex( std::size_t index, std::size_t sizeState)
{
  m_freeStateBlocks.push_back(StateBlock(index, sizeState));
}

void FreeStateCollector::clear()
{
  m_freeStateBlocks.clear();
}

void FreeStateCollector::defrag()
{
  // clean and unfragment free blocks
  // first sort them
  m_freeStateBlocks.sort();
  // second merge contiguous blocks
  StateBlockCont::iterator it1 = m_freeStateBlocks.begin();
  for(; it1 != m_freeStateBlocks.end() ; ++it1) {
    StateBlockCont::iterator it2 = it1; ++it2;
    while(it2 != m_freeStateBlocks.end() && it2->index == it1->index + it1->size) {
      it1->size += it2->size;
      it2 = m_freeStateBlocks.erase(it2);
    }
  }
}

/*
 * class DefaultMapManager
 */

void DefaultMapManager::clear() {}

void DefaultMapManager::setMapObjectState(AbstractMapObject& mapObj)
{
  mapObj.m_filterIndex = m_slam.filter.sizeState();
  m_slam.filter.softResize(m_slam.filter.sizeState() + mapObj.sizeState());
  
  mapObj.setState(m_slam.filter.getX(), m_slam.filter.getP());
}

void DefaultMapManager::removeMapObject(AbstractMapObject const& mapObj) 
{
  JFR_WARNING("DefaultMapManager::removeMapObject: definitly lost range: [" 
	      << mapObj.filterIndex() << "," << mapObj.filterIndex()+mapObj.sizeState()-1 << "]");
}

/*
 * class LocalMapManager
 */

LocalMapManager::LocalMapManager(SlamEkf& slam,
				 std::size_t sizeLocalMapMax,
				 std::size_t sizeFeatureState) :
  AbstractMapManager(slam),
  m_sizeFeatureState(sizeFeatureState),
  m_sizeLocalMapMax(sizeLocalMapMax),
  m_freeStateCollector(slam)
{
  JFR_PRECOND(sizeLocalMapMax*sizeFeatureState <= m_slam.filter.sizeStateMax() - m_slam.filter.sizeState(),
	      "LocalMapManager::LocalMapManager: size of kalman state too small");

  m_slam.addEventListener(*this);

}

LocalMapManager::~LocalMapManager()
{
  m_slam.removeEventListener(*this);
}

void LocalMapManager::clear() {
  m_freeStateCollector.clear();
}

void LocalMapManager::setMapObjectState(AbstractMapObject& mapObj) 
{
  JFR_PRECOND(mapObj.sizeState() == m_sizeFeatureState,
	      "LocalMapManager::setMapObjectState: invalid object state size " << mapObj.sizeState() 
	      << " valid size: " << m_sizeFeatureState);

//   JFR_PRED_ERROR(!freeStatesIndex.empty(), 
// 		 SlamException, 
// 		 SlamException::LOCAL_MAP_FULL, 
// 		 "LocalMapManager::initFeatureState: map is full");

  
  mapObj.m_filterIndex = m_freeStateCollector.getFreeIndex(m_sizeFeatureState);
  mapObj.setState(m_slam.filter.getX(), m_slam.filter.getP());
}

void LocalMapManager::removeMapObject(AbstractMapObject const& mapObj)
{
  JFR_DEBUG("LocalMapManager: collect index " << mapObj.filterIndex() << " from object " << mapObj.id());
  m_freeStateCollector.collectIndex( mapObj.filterIndex(), m_sizeFeatureState);
}

void LocalMapManager::endProcessObservations(unsigned int robotId_)
{
  SlamEkf::FeaturesMapType& map = m_slam.getMap();

  typedef std::list<unsigned int> CleanListType;
  CleanListType cleanList;

  // remove unobserved features
  for(SlamEkf::FeaturesMapType::iterator it = map.begin() ; it != map.end() ; ++it) {
    if (it->second->frameIndexes.back() != m_slam.getCurrentFrameIndex()) {
      cleanList.push_back(it->first);
    }
  }
  
  for(CleanListType::iterator it = cleanList.begin() ; it != cleanList.end() ; ++it) {
    JFR_DEBUG("LocalMapManager: remove feature " << (*it));
    m_slam.removeFeature(*it);
  }
}

/*
 * class GlobalMapManager
 */

GlobalMapManager::GlobalMapManager(SlamEkf& slam,
				   double sizeVoxel,
				   unsigned int nbObsMin) :
  AbstractMapManager(slam), m_freeStateCollector(slam)
{
  m_slam.addEventListener(*this);
  setParams(sizeVoxel,nbObsMin);
}

GlobalMapManager::~GlobalMapManager()
{
  m_slam.removeEventListener(*this);
}

GlobalMapManager::GlobalMapManager(SlamEkf& slam) :
  AbstractMapManager(slam), m_freeStateCollector(slam)
{
  m_slam.addEventListener(*this);
  setParams();
}

void GlobalMapManager::clear() 
{
  m_freeStateCollector.clear();
  m_voxels.clear();
}

void GlobalMapManager::setParams(double sizeVoxel, unsigned int nbObsMin)
{
  m_sizeVoxel = sizeVoxel;
  m_nbObsMin = nbObsMin;
}

#if 1
details::VoxelCoord GlobalMapManager::getVoxelCoord(BaseFeature const& f)
{
  details::VoxelCoord vc;
  vc.x = int(rint(f.getX()(0)/m_sizeVoxel));
  vc.y = int(rint(f.getX()(1)/m_sizeVoxel));
  vc.z = int(rint(f.getX()(2)/m_sizeVoxel));
  return vc;
}
#endif

void GlobalMapManager::setMapObjectState(AbstractMapObject& mapObj)
{
  mapObj.m_filterIndex = m_freeStateCollector.getFreeIndex(mapObj.sizeState());
  mapObj.setState(m_slam.filter.getX(), m_slam.filter.getP());
  JFR_DEBUG("GlobalMapManager::setMapObjectState: size:" << mapObj.sizeState() << " index:" << mapObj.filterIndex());
}

void GlobalMapManager::removeMapObject(AbstractMapObject const& mapObj)
{
  JFR_DEBUG("GlobalMapManager: collect state " << mapObj.filterIndex() << "-" << mapObj.filterIndex()+mapObj.sizeState() 
	    << " from object " << mapObj.id());
  
  m_freeStateCollector.collectIndex( mapObj.filterIndex(), mapObj.sizeState() );

  // FIXME: performance... pb of landmark added to two different voxels...
#if 1
  VoxelsType::iterator it = m_voxels.begin();
  while (it != m_voxels.end())
    {
      if (it->second->id() == mapObj.id())
	m_voxels.erase(it++);
      else
	++it;
    }
#endif
}

void GlobalMapManager::endProcessObservations(unsigned int robotId_)
{
#if 1
  SlamEkf::FeaturesMapType& map = m_slam.getMap();

  typedef std::list<unsigned int> CleanListType;
  CleanListType cleanList;

  JFR_VDEBUG("GlobalMapManager: slam current frame: " << m_slam.getCurrentFrameIndex());

  // remove some unobserved features to limit the density of landmarks in the map
  for(SlamEkf::FeaturesMapType::iterator it = map.begin() ; it != map.end() ; ++it) {
    // if the perception has just lost the feature
    if (it->second->frameIndexes.back() == m_slam.getPrevFrameIndex()) {
      JFR_VDEBUG("GlobalMapManager: landmark " << it->first << " has just been lost");
      if (it->second->frameIndexes.size() < m_nbObsMin) {
	JFR_VDEBUG("GlobalMapManager: remove: not enough observations" << it->first)
	cleanList.push_back(it->first);
      }
      else {
	details::VoxelCoord vc;
	vc = getVoxelCoord(*(it->second));
	JFR_VDEBUG("landmark voxel: " << vc);
	VoxelsType::iterator itVox = m_voxels.find(vc);

	if (itVox != m_voxels.end()) { 
	  if (itVox->second->id() != it->first) {
	    if (it->second->frameIndexes.size() > itVox->second->frameIndexes.size()) {
	      JFR_VDEBUG("GlobalMapManager: remove: " << itVox->second->id()
			 << " keep: " << it->first);
	      cleanList.push_back(itVox->second->id());
	      itVox->second = it->second;
	      JFR_VDEBUG("voxel: " << itVox->first << " landmark: " << itVox->second->id());
	    }
	    else {
	      JFR_VDEBUG("GlobalMapManager: remove: " << it->first 
			 << " keep: " << itVox->second->id());
	      cleanList.push_back(it->first);
	    }
	  }
	}
	else {
	  JFR_VDEBUG("GlobalMapManager: keep: " << it->first);
	  m_voxels[vc] = it->second;
	  JFR_VDEBUG("voxel: " << vc << " landmark: " << it->first);
	}
      }
    }
  }
  // do the actual removing
  for(CleanListType::iterator it = cleanList.begin() ; it != cleanList.end() ; ++it) {
    JFR_VDEBUG("GlobalMapManager: remove: " << (*it));
    m_slam.removeFeature(*it);
  }
  m_freeStateCollector.defrag();
#endif
}

std::ostream& jafar::slam::details::operator <<(std::ostream& s, VoxelCoord  const& v)
{
  s << "[" << v.x << "," << v.y << "," << v.z << "]";
  return s;
}


StrongMapManager::StrongMapManager(SlamEkf& slam_) : AbstractMapManager(slam_), m_freeStateCollector(slam_)
{
  m_slam.addEventListener(*this);
  setParams();
}

StrongMapManager::~StrongMapManager()
{
  m_slam.removeEventListener(*this);
}

void StrongMapManager::setParams(double minUncertainty, unsigned int nbObsMin)
{
  m_minUncertainty = minUncertainty;
  m_nbObsMin = nbObsMin;
}

void StrongMapManager::clear()
{
  m_freeStateCollector.clear();
}

void StrongMapManager::setMapObjectState(AbstractMapObject& mapObj)
{
  mapObj.m_filterIndex = m_freeStateCollector.getFreeIndex(mapObj.sizeState());
  mapObj.setState(m_slam.getFilter().getX(), m_slam.getFilter().getP());
  JFR_DEBUG("StrongMapManager::setMapObjectState: size:" << mapObj.sizeState() << " index:" << mapObj.filterIndex());
}

void StrongMapManager::removeMapObject(AbstractMapObject const& mapObj)
{
  m_freeStateCollector.collectIndex( mapObj.filterIndex(), mapObj.sizeState() );
}

void StrongMapManager::endProcessObservations(unsigned int robotId_)
{
  SlamEkf::FeaturesMapType& map = m_slam.getMap();
  typedef std::list<unsigned int> CleanListType;
  CleanListType cleanList;
    // remove some unobserved features to limit the density of landmarks in the map
  for(SlamEkf::FeaturesMapType::iterator it = map.begin() ; it != map.end() ; ++it) {
    // if the perception has just lost the feature
    if (it->second->frameIndexes.back() == m_slam.getPrevFrameIndex()) {
     JFR_DEBUG("StrongMapManager: landmark " << it->first << " has just been lost");
      if (it->second->frameIndexes.size() < m_nbObsMin) {
        JFR_DEBUG("StrongMapManager: remove: not enough observations " << it->first << " count obs: " << it->second->frameIndexes.size() );
        cleanList.push_back(it->first);
      } else if (it->second->frameIndexes.size() < 2 * m_nbObsMin) {

        
        jblas::vec lambda1( it->second->sizeState() );
        
        ublas::matrix<double, ublas::column_major> A1(it->second->getP());
        int ierr1 = lapack::syev( 'N', 'U', A1, lambda1, lapack::optimal_workspace() );
        JFR_DEBUG(lambda1 << " " << jmath::ublasExtra::maxV( lambda1 ) );
        if( jmath::ublasExtra::maxV( lambda1 ) > m_minUncertainty )
        {
          JFR_DEBUG("StrongMapManager: remove: not enough precision" << it->first << " count obs: " << it->second->frameIndexes.size() );
          cleanList.push_back(it->first);
        }
      }
    }
  }
  // do the actual removing
  for(CleanListType::iterator it = cleanList.begin() ; it != cleanList.end() ; ++it) {
    JFR_DEBUG("GlobalMapManager: remove: " << (*it));
    m_slam.removeFeature(*it);
  }
  if( not cleanList.empty() )
  {
    m_freeStateCollector.defrag();
  }
}
