/* $Id$ */

#include "slammm/MultiMapManager.hpp"

#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

#include "slam/featureModel.hpp"
#include "slam/pointInvDepthFeature.hpp"

#include "jacobianEuler.hpp"

#include "slammm/ListMapsPoseManager.hpp"

using namespace jafar;
using namespace jafar::slammm;
namespace lapack = boost::numeric::bindings::lapack;

MultiMapsSlam::MultiMapsSlam(std::size_t sizeMax_, MapsPoseManager* _mapsPoseManager, StartNewMapStrategy* _startNewMapStrategy, MergeMapsStrategy* _mergeMapsStrategy, slam::MapManagerFactory* mapManagerFactory) : nextMapId(0), m_mapsPoseManager( _mapsPoseManager), m_startNewMapStrategy(_startNewMapStrategy), m_mergeMapsStrategy(_mergeMapsStrategy),  m_mapManagerFactory( mapManagerFactory ), m_sizeMax( sizeMax_ ), m_eventsLogger(0)
{
  m_mergeMapsStrategy->setMultiMapsSlam( this );
}

MultiMapsSlam::~MultiMapsSlam()
{
  // FIXME fix memory leak
  delete m_mapsPoseManager;
  delete m_startNewMapStrategy;
  // delete ObserveModels
  // delete baseRobot
  // delete maps
  delete m_mapManagerFactory;
}

slam::SlamEkf* MultiMapsSlam::getCurrentMap( unsigned int _robotId )
{
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  return it->second.currentMap;
}

const slam::SlamEkf* MultiMapsSlam::getCurrentMap( unsigned int _robotId ) const
{
  std::map<unsigned int, RobotMaps>::const_iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  return it->second.currentMap;
}

slam::SlamEkf* MultiMapsSlam::getMap( unsigned int _mapId, unsigned int _robotId )
{
  return m_robotMaps[ _robotId ].maps[_mapId ]; 
}

unsigned int MultiMapsSlam::mapId( slam::SlamEkf* _slamEkf) const
{
  for( std::map< unsigned int, SlamEkfInfo>::const_iterator it = m_maps.begin();
       it != m_maps.end(); ++it )
  {
    if(it->second.slamEkf == _slamEkf )
    {
      return it->first;
    }
  }
  JFR_ASSERT(false, "No Id found for " << _slamEkf );
}

jafar::geom::T3DEuler MultiMapsSlam::getWorldToCurrentMapTransformation( unsigned int _robotId ) const
{
  std::map<unsigned int, RobotMaps>::const_iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  return it->second.worldToCurrentMap;
}

jafar::geom::T3DEuler MultiMapsSlam::getWorldToMapTransformation( unsigned int _mapId ) const
{
  return m_mapsPoseManager->getWorldToMapTransformation( _mapId );
}

void MultiMapsSlam::getWorldRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId ) const
{
  // Get the robot
  std::map<unsigned int, RobotMaps>::const_iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  // Get the robot pose
  slam::SlamEkf* slam = it->second.currentMap;
  jafar::geom::T3DEuler localRobotPose;
  slam->getRobotPose( localRobotPose, _robotId );
  // Compose with the worldtomap transformation
  jafar::geom::T3DEuler worldToMap = getWorldToCurrentMapTransformation( _robotId );
  
  geom::T3D::compose( worldToMap, localRobotPose, pose );
   JFR_DEBUG("robotId: " << _robotId);
   JFR_DEBUG( worldToMap << localRobotPose << pose );
}

jafar::geom::T3DEuler MultiMapsSlam::getWorldRobotPose( unsigned int _robotId ) const
{
  jafar::geom::T3DEuler t3d;
  getWorldRobotPose( t3d, _robotId );
  return t3d;
}

void MultiMapsSlam::getWorldFeatureState(jblas::vec& _result, unsigned int _featureId, unsigned int _currentMapId ) const
{
  // Get the map
  std::map< unsigned int, SlamEkfInfo>::const_iterator mapIt = m_maps.find( _currentMapId );
  JFR_ASSERT( mapIt != m_maps.end(), "No map " << _currentMapId );
  // Get world transformation
  jafar::geom::T3DEuler worldToMap = getWorldToMapTransformation( _currentMapId );
  slam::BaseFeature& feature = mapIt->second.slamEkf->getFeature(_featureId);
  feature.model.fromFrame( worldToMap.getX(), feature.getX(), _result );  
}


void MultiMapsSlam::getWorldFeatureState(jblas::vec& _result , jblas::sym_mat& _resultCov, unsigned int _featureId, unsigned int _currentMapId ) const
{
  // Get the map
  std::map< unsigned int, SlamEkfInfo>::const_iterator mapIt = m_maps.find( _currentMapId );
  JFR_ASSERT( mapIt != m_maps.end(), "No map " << _currentMapId );
  
  // Get world transformation
  jafar::geom::T3DEuler worldToMap = getWorldToMapTransformation( _currentMapId );
  
  // Get the feature
  slam::BaseFeature& feature = mapIt->second.slamEkf->getFeature(_featureId);
  // Convert the state of the feature
  feature.model.fromFrame( worldToMap.getX(), feature.getX(), _result );
  // Compute the jacobians
  feature.model.fromFrameJac( worldToMap.getX(), feature.getX() );
  // Convert the covariance
  _resultCov.assign( ublas::prod(
                        jblas::mat( ublas::prod( feature.model.Jframe, worldToMap.getXCov() ) ),
                        ublas::trans( feature.model.Jframe ) )
                  + ublas::prod( jblas::mat( ublas::prod( feature.model.Jx, feature.getP() ) ),
                        ublas::trans( feature.model.Jx ) ) );
}

void MultiMapsSlam::addRobot( slam::BaseRobot* baseRobot )
{
  JFR_ASSERT( m_robotMaps.find( baseRobot->id() ) == m_robotMaps.end(), "Robot id " << baseRobot->id() << " already exists" );
  RobotMaps robotMaps;
  robotMaps.baseRobot = baseRobot;
  m_robotMaps[ baseRobot->id() ] = robotMaps;
  initNewMap( baseRobot->id());
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( baseRobot->id() );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << baseRobot->id() );
  m_mapsPoseManager->addFirstMapPose( it->second.currentMapId, jblas::zero_vec( baseRobot->sizePose() ), jblas::zero_mat( baseRobot->sizePose(), baseRobot->sizePose() ) );
  it->second.worldToCurrentMap = m_mapsPoseManager->getWorldToMapTransformation( it->second.currentMapId);
}

void MultiMapsSlam::addRobot( slam::BaseRobot* baseRobot, const jblas::vec& _robotState, const jblas::sym_mat& _robotStateCov )
{
  JFR_ASSERT( m_robotMaps.find( baseRobot->id() ) == m_robotMaps.end(), "Robot id " << baseRobot->id() << " already exists" );
  RobotMaps robotMaps;
  robotMaps.baseRobot = baseRobot;
  m_robotMaps[ baseRobot->id() ] = robotMaps;
  initNewMap( baseRobot->id());
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( baseRobot->id() );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << baseRobot->id() );
  m_mapsPoseManager->addFirstMapPose( it->second.currentMapId, _robotState, _robotStateCov );  
  it->second.worldToCurrentMap = m_mapsPoseManager->getWorldToMapTransformation( it->second.currentMapId);
}

void MultiMapsSlam::setSensor(slam::FeatureObserveModel* model, unsigned int sensorId, unsigned int _robotId )
{
  // Get the robot
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  
  JFR_PRECOND(model->sizeRobotPose() == it->second.baseRobot->sizePose(),
              "SlamEkf::setSensor: invalid model " << model->sizeState1() << " " << it->second.baseRobot->sizePose());
  JFR_PRECOND(it->second.featureObserveModels.find(sensorId) == it->second.featureObserveModels.end(),
        "SlamEkf::setSensor: sensor already exists");

  JFR_PRECOND( std::find( m_sensorsIds.begin(), m_sensorsIds.end(), sensorId) == m_sensorsIds.end(), "SlamEkf::setSensor: sensors ID must be unique even for different robots" );
  m_sensorsIds.push_back( sensorId );
  
  it->second.featureObserveModels[sensorId] = model;
  it->second.currentMap->setSensor( model, sensorId);

}

void MultiMapsSlam::setRobotToSensor(jblas::vec const& robotToSensor, int sensorId, unsigned int _robotId )
{
    JFR_DEBUG( _robotId );
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  if( it->second.currentMap )
  {
    JFR_DEBUG( it->second.currentMap );
    JFR_DEBUG( it->second.currentMap->robotToSensor( sensorId ) );
    it->second.currentMap->setRobotToSensor( robotToSensor, sensorId );
    JFR_DEBUG( it->second.currentMap->robotToSensor( sensorId ) );
  }
  it->second.robotToSensors[ sensorId ] = robotToSensor;
}

void MultiMapsSlam::initNewMap( unsigned int _robotId)
{
  // Get the robot
  std::map<unsigned int, RobotMaps>::iterator it = m_robotMaps.find( _robotId );
  JFR_ASSERT( it != m_robotMaps.end(), "No robot " << _robotId );
  
  // Init new Slam Map
  slam::SlamEkf* newMap = new slam::SlamEkf( m_sizeMax, it->second.baseRobot->sizeState(), it->second.baseRobot->sizePose() );
  
  // Create a MapManager
  if( m_mapManagerFactory )
  {
    newMap->setMapManager( m_mapManagerFactory->createMapManager( *newMap ) );
  }
  
  // Init feature model
  newMap->setDeleteSensors( false );
  for (FeatureObserveModelsContType::const_iterator itFOM = it->second.featureObserveModels.begin() ; itFOM != it->second.featureObserveModels.end() ; ++itFOM) {
    newMap->setSensor( itFOM->second, itFOM->first);
  }
  
  // Add robot
  newMap->addRobot( new slam::BaseRobot( it->second.baseRobot->id(), it->second.baseRobot->model ) );
  
  // Set RobotToSensor
  for(std::map<int, jblas::vec>::const_iterator itRTS = it->second.robotToSensors.begin(); itRTS != it->second.robotToSensors.end(); ++itRTS )
  {
    newMap->setRobotToSensor( itRTS->second, itRTS->first  );
  }
  
  // Add event listeners
  for( std::list<slam::SlamEventListener*>::iterator it = m_listeners.begin();
       it != m_listeners.end(); ++it)
  {
    newMap->addEventListener( **it );
  }

  if( it->second.currentMap )
  { // Copy the consistency check
    newMap->getFilter().setCovUpdateType( it->second.currentMap->getFilter().getCovUpdateType() );
    newMap->getFilter().setupConsistencyCheck( it->second.currentMap->getFilter().getConsistencyCheckLevel(), it->second.currentMap->getFilter().getChi2Threshold() );
  }
  
  // Init robot pose to 0
  newMap->setRobotPose( jblas::zero_vec(6), _robotId );
  
  it->second.currentMap = newMap;
  it->second.currentMapId = nextMapId;
  ++nextMapId;
  it->second.maps[ it->second.currentMapId ] = newMap;
  SlamEkfInfo slamEkfInfo;
  slamEkfInfo.slamEkf = newMap;
  slamEkfInfo.robotPositionAge[ _robotId ] = it->second.currentMapId;
  m_maps[ it->second.currentMapId ] = slamEkfInfo;

}

void MultiMapsSlam::predict(unsigned int _robotId, jblas::vec const& u)
{
  getCurrentMap( _robotId )->predict( _robotId, u );
}

bool MultiMapsSlam::hasFeature(unsigned int id, unsigned int _robotId) const
{ // FIXME should it search in all maps
   return getCurrentMap( _robotId )->hasFeature( id );
}

void MultiMapsSlam::getRobotPose(jafar::geom::T3DEuler& pose, unsigned int _robotId ) const
{
  getWorldRobotPose( pose, _robotId );
}

void MultiMapsSlam::removeFeature(unsigned int id)
{
  for( std::map< unsigned int, SlamEkfInfo>::iterator it = m_maps.begin(); it != m_maps.end(); ++it )
  {
    if( it->second.slamEkf->hasFeature( id ) )
    {
      it->second.slamEkf->removeFeature( id );
    }
  }
}

void MultiMapsSlam::addEventListener(slam::SlamEventListener& listener)
{
  m_listeners.push_back( &listener );
  for( std::map< unsigned int, SlamEkfInfo>::iterator it = m_maps.begin(); it != m_maps.end(); ++it )
  {
    it->second.slamEkf->addEventListener( listener );
  }
}

void MultiMapsSlam::removeEventListener(slam::SlamEventListener& listener)
{
  m_listeners.remove( &listener );
  for( std::map< unsigned int, SlamEkfInfo>::iterator it = m_maps.begin(); it != m_maps.end(); ++it )
  {
    it->second.slamEkf->removeEventListener( listener );
  }
}

void MultiMapsSlam::processObservations(unsigned int frameIndex_,
             std::list<slam::Observation*> const& knownObs,
             std::list<slam::Observation*> const& newObs, unsigned int robotId_)
{
  processObservationsImpl( frameIndex_, knownObs, newObs, robotId_ );
}
void MultiMapsSlam::processObservations(unsigned int frameIndex_,
             std::vector<slam::Observation*> const& knownObs,
             std::vector<slam::Observation*> const& newObs, unsigned int robotId_)
{
  processObservationsImpl( frameIndex_, knownObs, newObs, robotId_ );
}

void MultiMapsSlam::mergeMap( unsigned int _map1Id, unsigned int _map2Id, const geom::T3DEuler& transfo )
{
  // FIXME
  
  JFR_ASSERT( false, "bouh" );
}

struct GlobalMapMergeInfo {
  explicit GlobalMapMergeInfo(const slam::SlamEkf* _map ) : map(_map), canBeDeleted(false)
  {
  }
  const slam::SlamEkf* map;
  bool canBeDeleted;
  jafar::geom::T3DEuler transfo;
  std::map< unsigned int, unsigned int > robotPositionAge;
  bool operator==( const GlobalMapMergeInfo& gmmi ) const
  {
    return map == gmmi.map;
  }
};

slam::SlamEkf* MultiMapsSlam::getGlobalMap( ) const
{
  std::list< GlobalMapMergeInfo > maps;
  std::list< GlobalMapMergeInfo > nextMaps;
  
  for( std::map< unsigned int, SlamEkfInfo>::const_iterator it = m_maps.begin();
       it != m_maps.end(); ++it)
  {
    if( it->second.slamEkf->getMap().size() != 0 )
    {
      GlobalMapMergeInfo gmmi( it->second.slamEkf );
      gmmi.robotPositionAge = it->second.robotPositionAge;
      gmmi.canBeDeleted = false;
      gmmi.transfo = getWorldToMapTransformation( it->first );
      maps.push_back( gmmi );
    }
  }
  while(true)
  {
    JFR_DEBUG( maps.size() );
    int startSize = maps.size();
    for( std::list< GlobalMapMergeInfo >::iterator it1 = maps.begin();
         it1 != maps.end(); ++it1)
    {
      JFR_DEBUG("====================================================================");
      JFR_DEBUG( it1->map );
      const slam::SlamEkf::FeaturesMapType& fm1 = it1->map->getMap();
      for( slam::SlamEkf::FeaturesMapType::const_iterator featureIt1 = fm1.begin();
           featureIt1 != fm1.end(); ++featureIt1 )
      {
        JFR_DEBUG( featureIt1->first );
      }
      JFR_DEBUG("====================================================================");
    }
    
    for( std::list< GlobalMapMergeInfo >::iterator it1 = maps.begin();
        it1 != maps.end(); )
    {
      const slam::SlamEkf::FeaturesMapType& fm1 = it1->map->getMap();
      int bestCount = 0;
      GlobalMapMergeInfo* bestFriend = 0;
      for( std::list< GlobalMapMergeInfo >::iterator it2 = maps.begin();
          it2 != maps.end(); ++it2 )
      {
        int count = 0;
        if( it1->map != it2->map )
        {
          const slam::SlamEkf::FeaturesMapType& fm2 = it2->map->getMap();
  //         std::list< int > feature2featureCurrent;
          for( slam::SlamEkf::FeaturesMapType::const_iterator featureIt1 = fm1.begin();
              featureIt1 != fm1.end(); ++featureIt1 )
          {
            for( slam::SlamEkf::FeaturesMapType::const_iterator featureIt2 = fm2.begin();
                featureIt2 != fm2.end(); ++featureIt2 )
            {
              if( featureIt1->first == featureIt2->first )
              {
                ++count;
              }
            }
          }
          if( count > bestCount )
          {
            bestCount = count;
            bestFriend = &*it2;
          }
        }
      }
      if( bestFriend )
      {
        JFR_ASSERT( bestCount > 0, "Bug, no common landmark can't merge" );
        
        
//         geom::T3DEuler inv;
//         geom::T3D::inv( bestFriend->transfo, inv );
//         geom::T3DEuler m2Tom1;
//         geom::T3D::compose( inv, it1->transfo, m2Tom1 );
//         JFR_DEBUG( m2Tom1 );
        
//         GlobalMapMergeInfo gmmi( SlamEkf::mergeMap( it1->map, bestFriend->map, m2Tom1.getX(), m2Tom1.getXCov() ) );
        geom::T3DEuler inv;
        geom::T3D::inv( it1->transfo, inv );
        geom::T3DEuler m1Tom2;
        geom::T3D::compose( inv, bestFriend->transfo, m1Tom2 );
        
        std::map< unsigned int, unsigned int > robotPositionAgeNext;
        std::map< unsigned int, slam::SlamEkf::MergeMapNumber > chooseRobotPose;
        
        for( std::map<unsigned int, RobotMaps>::const_iterator it = m_robotMaps.begin();
             it != m_robotMaps.end(); ++it )
        {
          std::map< unsigned int, unsigned int >::iterator robotAge1 = it1->robotPositionAge.find( it->first );
          std::map< unsigned int, unsigned int >::iterator robotAge2 = bestFriend->robotPositionAge.find( it->first );
          if( robotAge1 != it1->robotPositionAge.end()
              or robotAge2 != bestFriend->robotPositionAge.end() )
          {
            if( ( robotAge2 == bestFriend->robotPositionAge.end() )
                  or ( robotAge1 != it1->robotPositionAge.end() and robotAge1->second > robotAge2->second ) )
            { // robotAge1 is the more recent
              robotPositionAgeNext[ robotAge1->first ] = robotAge1->second;
              chooseRobotPose[ robotAge1->first ] = slam::SlamEkf::MAP_1;
            } else {
              robotPositionAgeNext[ robotAge2->first ] = robotAge2->second;
              chooseRobotPose[ robotAge2->first ] = slam::SlamEkf::MAP_2;
            }
          }
        }
        JFR_DEBUG( chooseRobotPose[0] );
        JFR_DEBUG( chooseRobotPose[1] );
        JFR_DEBUG( robotPositionAgeNext[0] );
        JFR_DEBUG( robotPositionAgeNext[1] );
        JFR_DEBUG( "Merge " << it1->map << " with " << bestFriend->map );
        GlobalMapMergeInfo gmmi( slam::SlamEkf::mergeMap( it1->map, bestFriend->map, m1Tom2.getX(), m1Tom2.getXCov(), chooseRobotPose ) );
        gmmi.canBeDeleted = true;
        gmmi.robotPositionAge = robotPositionAgeNext;
        gmmi.transfo = it1->transfo;
        nextMaps.push_back( gmmi );
        if(bestFriend->canBeDeleted)
        {
          JFR_DEBUG( "Delete " << bestFriend->map );
          delete bestFriend->map;
        }
        maps.erase( std::find( maps.begin(), maps.end(), *bestFriend ) );
        if(it1->canBeDeleted)
        {
          JFR_DEBUG( "Delete " << it1->map );
          delete it1->map;
        }
      } else {
        nextMaps.push_front( *it1 ); // put this map at the beginning since this will give it a higher priority for the next iteration for merging
      }
      it1 = maps.erase( it1 );
    }
    if(nextMaps.size() == 1)
    {
      JFR_DEBUG( nextMaps.begin()->transfo );
//       worldToMap.assign( nextMaps.begin()->transfo );
      slam::SlamEkf* slam = const_cast<slam::SlamEkf*>(nextMaps.begin()->map);
      slam->transform( nextMaps.begin()->transfo );
      return slam;
    }
    JFR_ASSERT( startSize != nextMaps.size(), "No common landmark");
    maps = nextMaps;
    nextMaps.clear();
  }
}

void MultiMapsSlam::beginBrowseMaps()
{
  browseFeaturesIt = m_maps.begin();
}

bool MultiMapsSlam::hasNextMap()
{
  return browseFeaturesIt != m_maps.end();
}

slam::SlamEkf* MultiMapsSlam::nextMap()
{
  JFR_ASSERT( hasNextMap(), "No more maps" );
  SlamEkfInfo& m = browseFeaturesIt->second;
  ++browseFeaturesIt;
  return m.slamEkf;
}

const std::map< unsigned int, slam::SlamEkf*> MultiMapsSlam::getMaps() const
{
  std::map< unsigned int, slam::SlamEkf*> maps;
  for( std::map< unsigned int, SlamEkfInfo>::const_iterator it = m_maps.begin();
       it != m_maps.end(); ++it)
  {
    maps[ it->first ] = it->second.slamEkf;
  }
  return maps;
}

void MultiMapsSlam::startNewMap( RobotMaps& robotsMap )
{
   JFR_DEBUG("==============START NEW MAP ================");
  slam::SlamEkf* previousCurrentMap = robotsMap.currentMap;
  int previousCurrentMapId = robotsMap.currentMapId;
  int robotId = robotsMap.baseRobot->id();
  initNewMap( robotId );
        // Add map to map transformation to the top level map
  geom::T3DEuler pose;
  previousCurrentMap->getRobotPose( pose, robotId );
  m_mapsPoseManager->addMapPose( robotsMap.currentMapId, previousCurrentMapId, pose.getX(), pose.getXCov() );
  robotsMap.worldToCurrentMap = m_mapsPoseManager->getWorldToMapTransformation( robotsMap.currentMapId);
        // Init robot pose
  robotsMap.currentMap->setRobotPose( jblas::zero_vec( robotsMap.baseRobot->sizePose() ), robotId );
}

void MultiMapsSlam::rendezVous( unsigned int _firstRobotId, unsigned int _secondRobotId, const geom::T3DEuler& firstToSecond )
{
  std::map<unsigned int, RobotMaps>::iterator itFirstRobot = m_robotMaps.find( _firstRobotId );
  JFR_ASSERT( itFirstRobot != m_robotMaps.end(), "No robot " << _firstRobotId );
  std::map<unsigned int, RobotMaps>::iterator itSecondRobot = m_robotMaps.find( _secondRobotId );
  JFR_ASSERT( itSecondRobot != m_robotMaps.end(), "No robot " << _secondRobotId );
  
  JFR_DEBUG( firstToSecond );
  
  startNewMap( itFirstRobot->second );
  startNewMap( itSecondRobot->second );
  m_mapsPoseManager->enforceLoopConstraint( itFirstRobot->second.currentMapId, itSecondRobot->second.currentMapId, firstToSecond );
  
  updateCacheWorldToCurrentMap();
  
  if( m_eventsLogger )
  {
    m_eventsLogger->writeData( m_frameIndex );
    m_eventsLogger->writeData( "Rendez-vous" );
    m_eventsLogger->writeData( -1 );
    m_eventsLogger->writeData( -1 );
    m_eventsLogger->writeData( _firstRobotId );
    m_eventsLogger->writeData( _secondRobotId );
  }
}

void MultiMapsSlam::closeLoop( unsigned int _firstMapId, unsigned int _secondMapId, const geom::T3DEuler& firstToSecond )
{
  m_mapsPoseManager->enforceLoopConstraint( _firstMapId, _secondMapId, firstToSecond );
  updateCacheWorldToCurrentMap();
  if( m_eventsLogger )
  {
    m_eventsLogger->writeData( m_frameIndex );
    m_eventsLogger->writeData( "Close Loop" );
    m_eventsLogger->writeData( _firstMapId );
    m_eventsLogger->writeData( _secondMapId );
    m_eventsLogger->writeData( -1 );
    m_eventsLogger->writeData( -1 );
  }
}

void MultiMapsSlam::gpsFix( const std::map<unsigned int, geom::T3DEuler>& worldToRobots )
{
  // First execute a gpsFix for each robot
  for( std::map<unsigned int, geom::T3DEuler>::const_iterator it = worldToRobots.begin(); it != worldToRobots.end(); ++it)
  {
    gpsFix( it->first, it->second );
  }
/*  // Then execute a rendez-vous event for each robot with each other robot
  for( std::map<unsigned int, geom::T3DEuler>::const_iterator it1 = worldToRobots.begin(); it1 != worldToRobots.end(); ++it1)
  {
    std::map<unsigned int, RobotMaps>::iterator itFirstRobot = m_robotMaps.find( it1->first );
    JFR_ASSERT( itFirstRobot != m_robotMaps.end(), "No robot " << it1->first );
    geom::T3DEuler inv;
    geom::T3DEuler R1toR2;
    geom::T3D::inv( it1->second, inv );
    for( std::map<unsigned int, geom::T3DEuler>::const_iterator it2 = worldToRobots.begin(); it2 != it1; ++it2)
    {
      std::map<unsigned int, RobotMaps>::iterator itSecondRobot = m_robotMaps.find( it2->first );
      JFR_ASSERT( itSecondRobot != m_robotMaps.end(), "No robot " << it2->first );
      geom::T3D::compose( inv, it2->second, R1toR2 );
      JFR_DEBUG("Rendez-vous: " << it1->first << " " << it2->first );
      m_mapsPoseManager->enforceLoopConstraint( it1->first, it2->first, R1toR2 );
    }
  }*/
  updateCacheWorldToCurrentMap();
}

void MultiMapsSlam::writeLogHeader(kernel::DataLogger& log) const
{
  log.writeComment("slam: MultiMapsSlam");
  for( std::size_t i = 0; i < m_robotMaps.size(); ++i )
  {
    log.writeLegendTokens("x y z yaw pitch roll");
    log.writeLegendTokens("cov_xx cov_xy cov_xz cov_xyaw cov_xpitch cov_xroll");
    log.writeLegendTokens("cov_yx cov_yy cov_yz cov_yyaw cov_ypitch cov_yroll");
    log.writeLegendTokens("cov_zx cov_zy cov_zz cov_zyaw cov_zpitch cov_zroll");
    log.writeLegendTokens("cov_yawx cov_yawy cov_yawz cov_yawyaw cov_yawpitch cov_yawroll");
    log.writeLegendTokens("cov_pitchx cov_pitchy cov_pitchz cov_pitchyaw cov_pitchpitch cov_pitchroll");
    log.writeLegendTokens("cov_rollx cov_rolly cov_rollz cov_rollyaw cov_rollpitch cov_rollroll");
  }
}

void MultiMapsSlam::writeLogData(kernel::DataLogger& log) const
{
  
  jafar::geom::T3DEuler pose;
  
  for( std::map<unsigned int, RobotMaps>::const_iterator it = m_robotMaps.begin();
       it != m_robotMaps.end(); ++it )
  {
    getRobotPose( pose, it->second.baseRobot->id() );
    log.writeDataVector( pose.getX() );
    for(std::size_t i = 0; i < pose.getXCov().size1(); ++i)
    {
      log.writeDataVector( ublas::matrix_row<jblas::sym_mat>( pose.getXCov(), i) );
    }
  }
}

void MultiMapsSlam::addMembersToLog(kernel::DataLogger& log) const
{
 log.addLoggable( *m_mapsPoseManager ); 
}

void MultiMapsSlam::setEventsDataLogger( kernel::DataLogger* log )
{
  m_eventsLogger->writeLegend("Frame #");
  m_eventsLogger->writeLegend("Event name");
  m_eventsLogger->writeLegend("MapId1");
  m_eventsLogger->writeLegend("MapId2");
  m_eventsLogger->writeLegend("RobotId1");
  m_eventsLogger->writeLegend("RobotId2");
}

void MultiMapsSlam::gpsFix( unsigned int _robotId, const geom::T3DEuler& worldToRobot )
{
  JFR_DEBUG("GPS Fix for " << _robotId << " " << worldToRobot );
  std::map<unsigned int, RobotMaps>::iterator itRobot = m_robotMaps.find( _robotId );
  JFR_ASSERT( itRobot != m_robotMaps.end(), "No robot " << _robotId );
  
  startNewMap( itRobot->second );
  m_mapsPoseManager->enforceLoopConstraint( -1, itRobot->second.currentMapId, worldToRobot );
  updateCacheWorldToCurrentMap();
  if( m_eventsLogger )
  {
    m_eventsLogger->writeData( m_frameIndex );
    m_eventsLogger->writeData( "GPS Fix" );
    m_eventsLogger->writeData( -1 );
    m_eventsLogger->writeData( itRobot->second.currentMapId );
    m_eventsLogger->writeData( _robotId );
    m_eventsLogger->writeData( _robotId );
  }
}

void MultiMapsSlam::updateCacheWorldToCurrentMap()
{ // FIXME give the possibility to block update of the cache
  for( std::map<unsigned int, RobotMaps>::iterator itRobot = m_robotMaps.begin();
       itRobot != m_robotMaps.end(); ++itRobot )
  {
    itRobot->second.worldToCurrentMap = m_mapsPoseManager->getWorldToMapTransformation( itRobot->second.currentMapId );
  }
}
