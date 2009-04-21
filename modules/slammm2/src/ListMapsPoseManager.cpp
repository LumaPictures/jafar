/* $Id$ */

#include "slammm/ListMapsPoseManager.hpp"

#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

#include "slam/featureModel.hpp"
#include "slam/pointInvDepthFeature.hpp"

#include "jacobianEuler.hpp"

using namespace jafar;
using namespace jafar::slammm;
namespace lapack = boost::numeric::bindings::lapack;

void ListMapsPoseManager::MapPoseInfoEdge::getTransfo( geom::T3DEuler& transfo, MapPoseInfo* info1, MapPoseInfo* info2)
{
//   JFR_DEBUG( previousMap << " " << nextMap);
//   JFR_DEBUG( info1 << " " << info2);
  if(info1 == previousMap and info2 == nextMap )
  {
    transfo.set( state, cov );
  } else {
    JFR_ASSERT(info2 == previousMap and info1 == nextMap, "Invalid maps" );
    geom::T3DEuler tmp(state, cov);
    geom::T3D::inv( tmp, transfo );
  }
}

ListMapsPoseManager::MapPoseInfo* ListMapsPoseManager::MapPoseInfoEdge::theOtherOne( ListMapsPoseManager::MapPoseInfo* info)
{
  return (info == previousMap) ? nextMap : previousMap;
}

bool ListMapsPoseManager::MapPoseInfoEdge::isCertainelyNull() const
{
  bool covIsNull = true;
  for( std::size_t i = 0; i < cov.size1(); ++i )
  {
    for( std::size_t j = 0; j < cov.size2(); ++j )
    {
      if( fabs( cov(i, j) ) )
      {
        covIsNull = false;
        break;
      }
    }
  }
  return covIsNull and (ublas::norm_2( state ) < 1e-10);
}

bool ListMapsPoseManager::MapPoseInfoEdge::connectWith( MapPoseInfo* _map1, MapPoseInfo* _map2 )
{
  return ( previousMap == _map1 and nextMap == _map2 ) or ( previousMap == _map2 and nextMap == _map1 );
}

ListMapsPoseManager::ListMapsPoseManager( std::size_t maxStateSize ) : m_state(maxStateSize), m_cov( maxStateSize, maxStateSize ), m_nextFreeState(0), globalFrameInfo(new MapPoseInfo)
{
  m_state.clear();
  m_cov.clear();
  globalFrameInfo->mapId = -1;
  m_mapPoseInfos[ -1 ] = globalFrameInfo;
}

ListMapsPoseManager::~ListMapsPoseManager()
{
  // FIXME: delete the content of m_mapPoseInfos
}

void ListMapsPoseManager::displayConnections() const
{
  for( std::list<MapPoseInfoEdge*>::const_iterator it = m_mapPoseEdgeInfos.begin();
       it != m_mapPoseEdgeInfos.end(); ++it )
  {
    JFR_DEBUG( (*it)->range.start() << " " << (*it)->range.size() << " " << (*it)->previousMap->mapId << " " << (*it)->nextMap->mapId );
  }
  for( std::map<unsigned int, MapPoseInfo* >::const_iterator it = m_mapPoseInfos.begin();
       it != m_mapPoseInfos.end(); ++it )
  {
    if( it->second->mapId != std::size_t(-1) )
    {
      JFR_DEBUG( it->second->mapId << " " << getWorldToMapTransformation( it->second->mapId ) );
    }
  }
}

void ListMapsPoseManager::addFirstMapPose(unsigned int _mapId, const jblas::vec& _robotState, const jblas::mat& _robotStateCov )
{

  JFR_ASSERT( m_mapPoseInfos.find( _mapId) == m_mapPoseInfos.end(), "Already known map " << _mapId );
  MapPoseInfo* mpi = new MapPoseInfo();
  mpi->mapId = _mapId;

  // Construct the edge
  MapPoseInfoEdge* mpie = createEdge(_robotState.size());
  mpie->state.assign( _robotState );
  mpie->cov.assign( _robotStateCov );
  mpie->previousMap = globalFrameInfo;
  mpie->nextMap = mpi;
  
  globalFrameInfo->edges.push_back( mpie );
  mpi->edges.push_back( mpie );
  
  m_mapPoseInfos[_mapId] = mpi;
  JFR_ASSERT( m_mapPoseInfos.find( _mapId) != m_mapPoseInfos.end(), "Unknown map " << _mapId );

}

void ListMapsPoseManager::addMapPose(unsigned int _mapId, unsigned int _previousMapId, const jblas::vec& _robotState, const jblas::mat& _robotStateCov )
{
  JFR_ASSERT( m_mapPoseInfos.find( _mapId) == m_mapPoseInfos.end(), "Already known map " << _mapId );
  MapPoseInfo* mpi = new MapPoseInfo( );
  mpi->mapId = _mapId;
  
  // Construct the edge
  MapPoseInfoEdge* mpie = createEdge(_robotState.size());
  mpie->state.assign( _robotState );
  mpie->cov.assign( _robotStateCov );

  MapPoseInfo* previousMap = m_mapPoseInfos[ _previousMapId ];
  JFR_ASSERT( previousMap, "No previous map: " << _previousMapId );
  
  mpie->previousMap = previousMap;
  mpie->nextMap = mpi;
  
  mpi->edges.push_back( mpie );
  previousMap->edges.push_back( mpie );
  
  m_mapPoseInfos[_mapId] = mpi;
  JFR_ASSERT( m_mapPoseInfos.find( _mapId) != m_mapPoseInfos.end(), "Unknown map " << _mapId );
}

ListMapsPoseManager::MapPoseInfoEdge* ListMapsPoseManager::createEdge(int robotStateSize)
{
  JFR_DEBUG( "We have " << m_mapPoseInfos.size() << " maps, robotStateSize = " << robotStateSize << " m_nextFreeState = " << m_nextFreeState << " m_state.size() = " << m_state.size());
  JFR_ASSERT( std::size_t(m_nextFreeState + robotStateSize) < m_state.size(), "No more free space" );
  ublas::range range( m_nextFreeState, m_nextFreeState + robotStateSize );
  m_nextFreeState += robotStateSize;
  MapPoseInfoEdge* mpie = new MapPoseInfoEdge( range, m_state, m_cov );
  m_mapPoseEdgeInfos.push_back( mpie );
  return mpie;
}

jafar::geom::T3DEuler ListMapsPoseManager::getWorldToMapTransformation( unsigned int _mapId ) const
{
  geom::T3DEuler transfo;
  getToMapTransformation( transfo, -1, _mapId );
  return transfo;
}

void ListMapsPoseManager::getToMapTransformation( geom::T3DEuler& transfo, unsigned int _mapId, unsigned int _secondMapId ) const
{
  std::map<unsigned int, MapPoseInfo* >::const_iterator mapIt = m_mapPoseInfos.find( _mapId );
  JFR_ASSERT( mapIt != m_mapPoseInfos.end(), "Unknown map " << _mapId );
  
  std::map<unsigned int, MapPoseInfo*>::const_iterator itSecondMap = m_mapPoseInfos.find( _secondMapId );
  JFR_ASSERT( itSecondMap != m_mapPoseInfos.end(), "No map " << _secondMapId );  
  MapPoseInfo* secondMap = itSecondMap->second;
  
  std::list< std::pair<MapPoseInfo*, MapPoseInfoEdge*> > cycle = smallestCycleBetween( mapIt->second, secondMap );
  
  JFR_ASSERT( cycle.size() >= 2, "No cycle between " << _mapId << " and " << _secondMapId );
  
  std::list< std::pair< MapPoseInfo*, MapPoseInfoEdge* > >::iterator it = cycle.begin();
  MapPoseInfo* nextMapPoseInfo = it->first;
  ++it;
  it->second->getTransfo( transfo, nextMapPoseInfo, it->first );
  nextMapPoseInfo = it->first;
  ++it;
  geom::T3DEuler tmp;
  geom::T3DEuler tmp2;
  for( ; it != cycle.end(); ++it )
  {
    it->second->getTransfo( tmp2, nextMapPoseInfo, it->first);
    geom::T3D::compose( transfo, tmp2, tmp );
    transfo = tmp;
    nextMapPoseInfo = it->first;

  }
}

jafar::geom::T3DEuler ListMapsPoseManager::getMapToMapTransformation( unsigned int _firstMapId, unsigned int _secondMapId ) const
{
  geom::T3DEuler result;
  getToMapTransformation( result, _firstMapId, _secondMapId);
  return result;
}

struct ListMapsPoseManager::SCBInfo {
  typedef std::pair< MapPoseInfo*, MapPoseInfoEdge* > CycleElt;
  std::list< CycleElt > cycle;
};

ListMapsPoseManager::CycleType ListMapsPoseManager::smallestCycleBetween( MapPoseInfo* _firstMap, MapPoseInfo* _secondMap ) const
{
//   JFR_DEBUG( "smallestCycleBetween " << _firstMap << " " << _secondMap);
//   std::map<unsigned int, MapPoseInfo*>::const_iterator itFirstMap = m_mapPoseInfos.find( _firstMapId );
//   JFR_ASSERT( itFirstMap != m_mapPoseInfos.end(), "No map " << _firstMapId );
  
  // Initialise the search of the cycle with the first node
  SCBInfo initialInfo;
  initialInfo.cycle.push_back( SCBInfo::CycleElt( const_cast<MapPoseInfo*>(_firstMap), 0 ) );
  std::list< SCBInfo > infos;
  infos.push_back( initialInfo );
  
  while( not infos.empty() )
  {
    // Loop over current possible cycles
    std::list< SCBInfo > nextInfos;
    for( std::list< SCBInfo >::iterator itInfo = infos.begin(); itInfo != infos.end(); ++itInfo)
    {
      // Loop over the previous maps
      MapPoseInfo* currentMap = (--itInfo->cycle.end())->first;
//       JFR_DEBUG(currentMap);
//      JFR_DEBUG(currentMap->edges.size());
      
      for( std::list< MapPoseInfoEdge*>::iterator itAnte = currentMap->edges.begin();
           itAnte != currentMap->edges.end(); ++itAnte )
      {
        MapPoseInfo* nextCurrentMap = (*itAnte)->theOtherOne( currentMap );
//         JFR_DEBUG( nextCurrentMap << " " << currentMap << " " << itInfo->cycle.size())
        if( nextCurrentMap == _secondMap )
        { // We have found the shortest path to the second map, lets return it
          itInfo->cycle.push_back( SCBInfo::CycleElt( nextCurrentMap, const_cast<MapPoseInfoEdge*>(*itAnte ) ) );
//           JFR_DEBUG(itInfo->cycle.size());
          return itInfo->cycle;
        } else {
          bool alreadyInCycle = false;
          for( std::list< SCBInfo::CycleElt >::iterator it = 
               itInfo->cycle.begin(); it != itInfo->cycle.end(); ++it)
          {
            if( it->first == nextCurrentMap )
            {
              alreadyInCycle = true;
            }
          }
//           JFR_DEBUG(alreadyInCycle);
          if(not alreadyInCycle)
          {
            SCBInfo newInfo = *itInfo;
            newInfo.cycle.push_back( SCBInfo::CycleElt( nextCurrentMap, const_cast<MapPoseInfoEdge*>(*itAnte ) ) );
            nextInfos.push_back( newInfo );
          }
        }
      }
    }
    infos = nextInfos;
  }
//   JFR_DEBUG("No cycle");
  return std::list< std::pair<MapPoseInfo*, MapPoseInfoEdge*> >();
}

bool ListMapsPoseManager::mapAreConnected( MapPoseInfo* _map1, MapPoseInfo* _map2 )
{
  
  for( std::list< MapPoseInfoEdge* >::iterator it = _map1->edges.begin();
       it != _map1->edges.end(); ++it )
  {
    if( (*it)->connectWith( _map1, _map2 ) )
    {
      return true;
    }
  }
  return false;
}

#define ELCINFOS_INDEX( index ) ( countTransformations - (index) - 1 )

struct ListMapsPoseManager::ELCInfo {
  ELCInfo() : J1(6,6), J2(6,6), Jinv(0) {}
  ~ELCInfo() { delete Jinv; }
  MapPoseInfo* mapPoseInfo;
  MapPoseInfoEdge* mapPoseInfoEdge;
  
  jblas::mat J1;
  jblas::mat J2;
  ublas::range range;
  jblas::mat* Jinv;
};

void ListMapsPoseManager::enforceLoopConstraint( unsigned int _firstMapId, unsigned int _secondMapId, const jblas::vec& _transformation, const jblas::sym_mat& _transformationCov )
{
  
  // List of map that are touched by this loop constraint enforcement
  std::vector< ELCInfo > infos; // in reverse order, allways use ELCINFOS_INDEX to access its elements
  
  std::map<unsigned int, MapPoseInfo*>::const_iterator itFirstMap = m_mapPoseInfos.find( _firstMapId );
  JFR_ASSERT( itFirstMap != m_mapPoseInfos.end(), "No map " << _firstMapId );
  MapPoseInfo* firstMap = itFirstMap->second;
  
  std::map<unsigned int, MapPoseInfo*>::const_iterator itSecondMap = m_mapPoseInfos.find( _secondMapId );
  JFR_ASSERT( itSecondMap != m_mapPoseInfos.end(), "No map " << _secondMapId );
  MapPoseInfo* secondMap = itSecondMap->second;
  
  if( mapAreConnected( firstMap, secondMap ) )
  {
    JFR_DEBUG("Maps are already connected");
    return;
  }

  
  JFR_ASSERT( firstMap != secondMap, "Maps must be different" );
  
  CycleType cycle = smallestCycleBetween( firstMap, secondMap );
  JFR_ASSERT( cycle.size() >= 2, "No cycle between " << _firstMapId << " and " << _secondMapId );
  
  for(CycleType::iterator it = cycle.begin(); it != cycle.end(); ++it )
  {
    JFR_DEBUG( it->second );
    if( (not it->second) or (not it->second->isCertainelyNull()) )
    {
//    if( it->second ) { JFR_DEBUG( it->second->isCertainelyNull() << " " << it->second->state << " " << it->second->cov ); }
      ELCInfo info;
      
      info.mapPoseInfo = it->first;
      info.mapPoseInfoEdge = it->second;
      infos.push_back( info );
    }
  }
  
  int countTransformations = infos.size();
  if( countTransformations < 2 )
  {
    JFR_DEBUG("Not enough non-null transformations, no enforcement has been done");
    return;
  }
  
  // Init the newly created edge
  MapPoseInfoEdge* mpie = createEdge(_transformation.size());
  mpie->previousMap = firstMap;
  mpie->nextMap = secondMap;
  mpie->state = _transformation;
  mpie->cov = _transformationCov;
  
  JFR_DEBUG( infos.begin()->mapPoseInfoEdge );
  JFR_ASSERT( not infos.begin()->mapPoseInfoEdge, "Bug: shouldn't already have a edge in the first info." );
  
  infos.begin()->mapPoseInfoEdge = mpie;
  
  
  // Init the ranges  
  for( int i = 0; i < countTransformations; ++i)
  {
//     JFR_DEBUG( i << " " << ELCINFOS_INDEX( i ) );
    infos[ ELCINFOS_INDEX( i ) ].range = ublas::range( i * 6, i * 6 + 6 );
  }
  
  // Initialize the state vector
//   JFR_DEBUG( "countTransformations = " << countTransformations);
  jblas::vec x_u(countTransformations * 6);
  jblas::sym_mat P_u(countTransformations * 6, countTransformations * 6 );
  for( int i = 0; i < countTransformations; ++i)
  {
//     JFR_DEBUG( "=== " << i );
    ELCInfo* info = &infos[ ELCINFOS_INDEX( i ) ];
    if( info->mapPoseInfo == info->mapPoseInfoEdge->previousMap )
    {
      ublas::project( x_u, info->range ).assign( info->mapPoseInfoEdge->state );
    } else {
      // Then we need to inverse the transformation
      geom::T3DEuler trans( info->mapPoseInfoEdge->state );
      geom::T3DEuler transInv;
      info->Jinv = new jblas::mat(6,6);
      geom::T3D::inv( trans, transInv, *info->Jinv );
      ublas::project( x_u, info->range ).assign( transInv.getX() );
    }
  }
  
  // Copy the covariance and invert it as needed
  for( int i = 0; i < countTransformations; ++i)
  {
    ELCInfo& info = infos[ ELCINFOS_INDEX( i ) ];
    for( int j = 0; j <= i; ++j )
    {
      ELCInfo& info2 = infos[ ELCINFOS_INDEX(j) ];
      if( info.Jinv or info2.Jinv )
      {
        jblas::mat m = ublas::project( m_cov, info.mapPoseInfoEdge->range, info2.mapPoseInfoEdge->range  );
        JFR_DEBUG( m );
        // Multiply by the transformation jacobian as needed
        if(info.Jinv )
        {
          JFR_DEBUG( *info.Jinv );
          m = ublas::prod( *info.Jinv, m );
        }
        if(info2.Jinv )
        {
          JFR_DEBUG( *info2.Jinv );
          m = ublas::prod( m, ublas::trans( *info2.Jinv ) );
        }
        JFR_DEBUG( m );
        ublas::project( P_u, info.range, info2.range ).assign( m );
      } else {
        ublas::project( P_u, info.range, info2.range ).assign( 
            ublas::project( m_cov, info.mapPoseInfoEdge->range, info2.mapPoseInfoEdge->range  ) );
      }
    }
  }

  
  // Initialize the recuring vectors
//   JFR_DEBUG("Initialize the recuring vectors");
  jblas::vec x_i = x_u;
  bool didOneIteration = false;
  jblas::mat HPHinv( 6, 6 );
  jblas::mat H(6, 6 * countTransformations );
//   JFR_DEBUG( P_u);
  while( true )
  {
    
    // Compute Jacobians and composition
//     JFR_DEBUG("Compute Jacobians and composition");
    geom::T3DEuler totalComposition;
    geom::T3DEuler tmpComposition1, tmpComposition2;
    
    // Start one iteration
//     JFR_DEBUG("Start one iteration");
    ELCInfo& info1 = infos[ ELCINFOS_INDEX( 0 ) ];
    ELCInfo& info2 = infos[ ELCINFOS_INDEX( 1 ) ];
//     JFR_DEBUG( ELCINFOS_INDEX( 0 ) << " " << infos.size() );
//     JFR_DEBUG( info1.mapPoseInfo );
//     JFR_DEBUG( info1.mapPoseInfoEdge );
//     JFR_DEBUG( info1.range.start() );
//     JFR_DEBUG( info1.range.size() );
    tmpComposition1.set( ublas::project(x_i, info1.range ) );
    tmpComposition2.set( ublas::project(x_i, info2.range )  );
    geom::T3D::compose( tmpComposition1, tmpComposition2, totalComposition, info2.J1, info2.J2 );
    
    // Compute Jacobians and the full composition
//     JFR_DEBUG("Compute Jacobians and the full composition");
    for( int i = 2; i < countTransformations; ++i )
    {
      ELCInfo& info = infos[ ELCINFOS_INDEX( i ) ];
      tmpComposition1.set( ublas::project(x_i, info.range )  );
      geom::T3D::compose( totalComposition, tmpComposition1, tmpComposition2, info.J1, info.J2 );
      totalComposition.assign( tmpComposition2 );
    }
    
    if( ublas::norm_2( totalComposition.getX() ) < 1e-5 ) break;
    JFR_DEBUG( totalComposition << " " << ublas::norm_2( totalComposition.getX() ) );
    
    didOneIteration = true;
    
    // Construct "H"
//     JFR_DEBUG("Construct 'H'");
    ublas::project( H,
                    ublas::range(0,6),
                    ublas::range( 6 * ( countTransformations -1),
                                  6* (countTransformations - 1) + 6 )
                  ).assign( infos[ ELCINFOS_INDEX( countTransformations - 1 ) ].J2 ) ;
    jblas::mat tmpMat = infos[ ELCINFOS_INDEX( countTransformations - 1 ) ].J1;
    
    for( int i = countTransformations - 2; i >= 1; --i )
    {
      ublas::project( H, ublas::range(0,6), ublas::range( 6 * i, 6* i + 6 )).assign( ublas::prod( tmpMat, infos[ ELCINFOS_INDEX( i ) ].J2 ));
      tmpMat = ublas::prod( tmpMat, infos[ ELCINFOS_INDEX( i ) ].J1 );
    }    ublas::project( H, ublas::range(0,6), ublas::range(0,6) ).assign( tmpMat );
    
    // Compute (H*P*Ht)^-1
//     JFR_DEBUG("Compute (H*P*Ht)^-1");
    jmath::ublasExtra::inv( jblas::mat(ublas::prod( H, jblas::mat(ublas::prod( P_u, ublas::trans(H) ) ) ) ), HPHinv );
    
    // Update
//     JFR_DEBUG("Update");
    x_i = x_u + jblas::vec(ublas::prod( P_u, jblas::vec(ublas::prod( ublas::trans(H),
        jblas::vec(ublas::prod( HPHinv, jblas::vec(ublas::prod( H, x_i - x_u) - totalComposition.getX() ) ) ) ) ) ) );
    JFR_DEBUG( x_i << " " << x_u );
  }
  if( didOneIteration )
  {
    // Compute update of P_u
//     JFR_DEBUG(P_u);
//     JFR_DEBUG(jmath::ublasExtra::prettyFormat(P_u));
//     JFR_DEBUG("Compute update of P_u");
    JFR_DEBUG( P_u );
    JFR_DEBUG( H );
    JFR_DEBUG( jmath::ublasExtra::prettyFormat( HPHinv ) );
    JFR_DEBUG( jmath::ublasExtra::prettyFormat( P_u - ublas::prod( P_u, jblas::mat(ublas::prod( ublas::trans( H ), jblas::mat(ublas::prod( HPHinv, jblas::mat(ublas::prod(H, P_u) ) ) ) ) ) ) ) );
    P_u = P_u - ublas::prod( P_u, jblas::mat(ublas::prod( ublas::trans( H ), jblas::mat(ublas::prod( HPHinv, jblas::mat(ublas::prod(H, P_u) ) ) ) ) ) );
    JFR_DEBUG( P_u );
//     JFR_DEBUG(P_u);
//     JFR_DEBUG(jmath::ublasExtra::prettyFormat(P_u));
//     JFR_DEBUG(P_u);
    // Copy the state vector back to the "global" state vector
//     JFR_DEBUG("Copy the state vector back to the 'global' state vector");
    for( int i = 0; i < countTransformations; ++i)
    {
      ELCInfo* info = &infos[ ELCINFOS_INDEX( i )  ];
      if( info->Jinv )
      { // Need inversion
        geom::T3DEuler trans( ublas::project( x_i, info->range ) );
        geom::T3DEuler transInv;
        geom::T3D::inv( trans, transInv, *info->Jinv ); // replace Jinv with the jacobian for the new transformation
        info->mapPoseInfoEdge->state.assign( transInv.getX() );
      } else {
        info->mapPoseInfoEdge->state.assign( ublas::project( x_i, info->range ) );
      }
  //     }
    }
    
    
    for( int i = 0; i < countTransformations; ++i)
    {
      ELCInfo* info = &infos[ ELCINFOS_INDEX( i )  ];
      for( int j = 0; j <= 0; ++j )
      {
        ELCInfo* info2 = &infos[ ELCINFOS_INDEX( i )  ];
        
        if( info->Jinv or info2->Jinv )
        {
          jblas::mat m = ublas::project( P_u, info->range, info2->range );
        // Multiply by the transformation jacobian as needed
          if(info->Jinv )
          {
            m = ublas::prod( *info->Jinv, m );
          }
          if(info2->Jinv )
          {
            m = ublas::prod( m, ublas::trans( *info2->Jinv ) );
          }
        
          ublas::project( m_cov, info->mapPoseInfoEdge->range, info2->mapPoseInfoEdge->range  ).assign( m );
        } else {
          ublas::project( m_cov, info->mapPoseInfoEdge->range, info2->mapPoseInfoEdge->range  ).assign( ublas::project( P_u, info->range, info2->range ) );
        }
        
      }
    }

  }
  if( mpie->previousMap )
  {
    mpie->previousMap->edges.push_back( mpie );
  }
  if( mpie->nextMap )
  {
    mpie->nextMap->edges.push_back( mpie );
  }
  
//   JFR_DEBUG( "m_state = " << m_state );
//   JFR_DEBUG( "m_cov = " << m_cov );
  // Clean up
}

void ListMapsPoseManager::writeLogHeader(kernel::DataLogger& log) const
{
  log.writeComment("slam: ListMapsPoseManager: map transformations");
  for(std::size_t i = 0; i < m_state.size() / 6; i++ )
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

void ListMapsPoseManager::writeLogData(kernel::DataLogger& log) const
{
  std::size_t count = 0;
  for( std::map<unsigned int, MapPoseInfo* >::const_iterator it = m_mapPoseInfos.begin();
       it != m_mapPoseInfos.end(); ++it )
  {
    if( it->second->mapId != std::size_t(-1) )
    {
      geom::T3DEuler t3d = getWorldToMapTransformation( it->second->mapId );
      log.writeDataVector( t3d.getX() );
      for(std::size_t j = 0; j < 6; ++j)
      {
        log.writeDataVector( ublas::matrix_row<const jblas::sym_mat>(  t3d.getXCov(), j) );
      }
      count += 42;
    }
  }
  for( ; count < 42 * (m_state.size() / 6); ++count)
  {
    log.writeData(0);
  }

#if 0
  for(std::size_t i = 0; i < m_state.size() / 6; i++ )
  {
    ublas::range r( i * 6, (i+1) * 6 );
    log.writeDataVector( ublas::project( m_state, r ) );
    for(std::size_t j = 0; j < 6; ++j)
    {
      log.writeDataVector( ublas::project( ublas::matrix_row<const jblas::sym_mat>(  m_cov, 6 * i + j), r ) );
    }
  }
#endif
}
