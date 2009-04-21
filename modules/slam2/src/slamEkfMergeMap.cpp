/* $Id$ */

#include "slam/slamEkf.hpp"
#include "slam/feature.hpp"
#include "slam/robot.hpp"

#include <jmath/ublasExtra.hpp>

using namespace jafar;
using namespace jafar::slam;
using jmath::ublasExtra::prettyFormat;

//FIXME dynamic sizeObs
struct MergeMapStateInformation {
  MergeMapStateInformation(unsigned int _id, const ublas::range& _originalRange, const ublas::range& _range, jblas::vec& _fullState, SlamEkf::MergeMapNumber _mapNumber ) : featureId(_id), range(_range), originalRange(_originalRange), state(_fullState, _range ), robotModel(0), featureModel(0), abstractMapObjectInResult(0), isPair( false), mapNumber(_mapNumber) {}
  ~MergeMapStateInformation() { }
  unsigned int featureId;
  ublas::range range;
  ublas::range originalRange;
  jblas::vec_range state;
  std::size_t sizeObs;
  jafar::filter::JacobianBlockCommandPredictModel* robotModel;
  FeatureModel* featureModel;
  Observation::ObservationType typeObs;
  AbstractMapObject* abstractMapObjectInResult; ///< Feature or robot in the final map
  bool isPair; ///< tell if this feature is a pair or not
  SlamEkf::MergeMapNumber mapNumber;
};

struct MergeMapPairInformation {
  MergeMapPairInformation() :featureMap1(0), featureMap2(0)
  {}
  unsigned int featureId;
  ublas::range hRange; ///< Range of the pair in the h and H vector / matrix
  MergeMapStateInformation* featureMap1; ///< Information on the feature in the first map
  MergeMapStateInformation* featureMap2; ///< Information on the feature in the second map  
};

SlamEkf* SlamEkf::mergeMap( const SlamEkf* map1, const SlamEkf* map2, const jblas::vec& map1ToMap2, const jblas::mat& map1ToMap2Cov, const std::map< unsigned int, SlamEkf::MergeMapNumber >& _chooseRobotPose )
{
  SlamEkf* resultMap = new SlamEkf( map1->filter.sizeState() + map2->filter.sizeState(), map1->sizeRobotState(), map1->sizeRobotPose() );
  resultMap->setDeleteSensors( false );
  // Add sensors and set robot to sensor from first map
  std::list<unsigned int> sensorIds;
  for( FeatureObserveModelsContType::const_iterator it = map1->featureObserveModels.begin();
       it != map1->featureObserveModels.end(); ++it )
  {
    resultMap->setSensor( it->second, it->first );
    sensorIds.push_back( it->first );
    resultMap->setRobotToSensor( map1->robotToSensor( it->first ) , it->first );
  }
  // Add sensors and set robot to sensor from second map, if needed
  for( FeatureObserveModelsContType::const_iterator it = map2->featureObserveModels.begin();
       it != map2->featureObserveModels.end(); ++it )
  {
    if( std::find( sensorIds.begin(), sensorIds.end(), it->first ) == sensorIds.end())
    { // Only add a sensor to the new map if it wasn't allready in the first map
      resultMap->setSensor( it->second, it->first );
      resultMap->setRobotToSensor( map2->robotToSensor( it->first ) , it->first );
    }
  }
  std::size_t sizeState = map1->filter.sizeState() + map2->filter.sizeState();
  jblas::vec x_ik(sizeState);
  jblas::sym_mat P_ik( sizeState, sizeState );
  std::vector< MergeMapStateInformation* > infos;
  
  int nextFreeState = 0;
  // Init the MergeMapStateInformation infos for first map's robots
//   JFR_DEBUG("Init the MergeMapStateInformation infos for first map's robots");
  for( RobotsMapType::const_iterator it = map1->robotsMap.begin();
       it != map1->robotsMap.end(); ++it )
  {
    BaseRobot* robot = it->second;
    MergeMapStateInformation* mpsi = new MergeMapStateInformation( it->first,
        ublas::range( robot->filterIndex(), robot->filterIndex() + robot->sizeState()),
        ublas::range( nextFreeState, nextFreeState + robot->sizeState() ), x_ik, SlamEkf::MAP_1 );
    mpsi->robotModel = &robot->model;
    mpsi->state = robot->getX();
    infos.push_back( mpsi );
    nextFreeState += robot->sizeState();
  }
  
  // Init the MergeMapStateInformation infos for first map's features
//   JFR_DEBUG("Init the MergeMapStateInformation infos for first map's features");
  for( FeaturesMapType::const_iterator it = map1->featuresMap.begin();
       it != map1->featuresMap.end(); ++it)
  {
    BaseFeature* feature = it->second;
    MergeMapStateInformation* mpsi = new MergeMapStateInformation( it->first,
        ublas::range( feature->filterIndex(), feature->filterIndex() + feature->sizeState()),
        ublas::range( nextFreeState, nextFreeState + feature->sizeState() ), x_ik, SlamEkf::MAP_1 );
    //FIXME zPred should be obtained from observeModel
    mpsi->sizeObs = feature->zPred.size();
    mpsi->featureModel = &feature->model;  
    mpsi->typeObs = feature->typeObs;
    mpsi->state = feature->getX();
    infos.push_back( mpsi );
    nextFreeState += feature->sizeState();
  }
  
  // Copy the covariance
//   JFR_DEBUG("Copy the covariance");
  for( int i = 0; i < infos.size(); ++i )
  {
    MergeMapStateInformation* info1 = infos[i];
    for( int j = 0; j <= i; ++j )
    {
      MergeMapStateInformation* info2 = infos[j];
      project( P_ik, info1->range, info2->range ).assign( project( map1->filter.getP(), info1->originalRange, info2->originalRange ) );
    }
  }
  
  geom::T3DEuler map1ToMap2T3DEuler( map1ToMap2, map1ToMap2Cov); 
  jblas::mat J1s = jblas::zero_mat( map2->filter.sizeState(), 6 );
  jblas::mat J2s = jblas::zero_mat( map2->filter.sizeState(), map2->filter.sizeState() );
  std::vector< MergeMapStateInformation* > infos2;
  int firstStateOfSecondMap = nextFreeState;
  int J1J2Pos = 0;

  // Init the MergeMapStateInformation infos for second map's robots
//   JFR_DEBUG("Init the MergeMapStateInformation infos for second map's robots");
  for( RobotsMapType::const_iterator it = map2->robotsMap.begin();
       it != map2->robotsMap.end(); ++it )
  {
    BaseRobot* robot = it->second;
    MergeMapStateInformation* mpsi = new MergeMapStateInformation( it->first,
        ublas::range( robot->filterIndex(), robot->filterIndex() + robot->sizeState()),
        ublas::range( nextFreeState, nextFreeState + robot->sizeState() ), x_ik, SlamEkf::MAP_2 );
    
    mpsi->robotModel = &robot->model;
    
    // Compose the position of the robot
    jblas::mat J1(6,6);
    jblas::mat J2(6,6);
    geom::T3DEuler robotPos( robot->getX() );
    geom::T3DEuler composedRobotPos;
    geom::T3D::compose( map1ToMap2T3DEuler, robotPos, composedRobotPos, J1, J2);
    mpsi->state = composedRobotPos.getX();
    
    // Fill J1s and J2s
    ublas::range rangeMat(6 * J1J2Pos, 6 * J1J2Pos + 6);
    ublas::project( J1s, rangeMat, ublas::range(0, 6) ).assign( J1 );
    ublas::project( J2s, rangeMat, rangeMat ).assign( J2 );
    
    infos.push_back( mpsi );
    infos2.push_back( mpsi );
    nextFreeState += robot->sizeState();
    J1J2Pos += robot->sizeState();
  }
  
  // Init the MergeMapStateInformation infos for second map's features
//   JFR_DEBUG("Init the MergeMapStateInformation infos for second map's features");
  for( FeaturesMapType::const_iterator it = map2->featuresMap.begin();
       it != map2->featuresMap.end(); ++it)
  {
    BaseFeature* feature = it->second;
    MergeMapStateInformation* mpsi = new MergeMapStateInformation( it->first,
        ublas::range( feature->filterIndex(), feature->filterIndex() + feature->sizeState()),
        ublas::range( nextFreeState, nextFreeState + feature->sizeState()), x_ik, SlamEkf::MAP_2 );
    //FIXME zPred should be obtained form observeModel
    mpsi->featureModel = &feature->model;
    mpsi->sizeObs = feature->zPred.size();
    mpsi->typeObs = feature->typeObs;
    
    // Compose the state of the feature
    jblas::vec v( feature->sizeState() );
    feature->model.fromFrame( map1ToMap2, feature->getX(), v);
    mpsi->state = v;
//     JFR_DEBUG( feature->getX() << v );
    // Compute J1s and J2s
    feature->model.fromFrameJac( map1ToMap2, feature->getX() );
    ublas::range rangeMat(J1J2Pos, J1J2Pos + feature->sizeState());
    ublas::project( J1s, rangeMat, ublas::range(0, 6) ).assign( feature->model.Jframe );
    ublas::project( J2s, rangeMat, rangeMat ).assign( feature->model.Jx );
    
    infos.push_back( mpsi );
    infos2.push_back( mpsi );
    nextFreeState += feature->sizeState();
    J1J2Pos += feature->sizeState();
  }
  
//   JFR_DEBUG( "P_ik = " << P_ik );
  // Copy the covariance
//   JFR_DEBUG("Copy the covariance");
  for( int i = 0; i < infos2.size(); ++i )
  {
    MergeMapStateInformation* info1 = infos2[i];
    for( int j = 0; j <= i; ++j )
    {
      MergeMapStateInformation* info2 = infos2[j];
      project( P_ik, info1->range, info2->range ).assign( project( map2->filter.getP(), info1->originalRange, info2->originalRange ) );
    }
  }
  
//   JFR_DEBUG( "P_ik = " << P_ik );
  
  // Compute the real size of the state
  int realStateSize = nextFreeState;
  JFR_ASSERT( realStateSize <= P_ik.size1(), "The real state size must be smaller or equal to the maximum state size");
  // Change the frame of the covariant
  jblas::sym_mat_range subP_ik( P_ik, ublas::range(firstStateOfSecondMap, nextFreeState),
                        ublas::range(firstStateOfSecondMap, nextFreeState) );
//   JFR_DEBUG( J2s << subP_ik);
  jblas::mat_range subJ1s = project( J1s, ublas::range( 0, subP_ik.size2() ), ublas::range( 0, 6 ) );
  jblas::mat_range subJ2s = project( J2s, ublas::range( 0, subP_ik.size2() ), ublas::range( 0, subP_ik.size2() ) );
  
//   JFR_DEBUG(J1s);
//   JFR_DEBUG(J2s);
//   JFR_DEBUG(P_ik);
  
//   JFR_DEBUG(subP_ik);
//   JFR_DEBUG(subJ1s);
//   JFR_DEBUG(subJ2s);
//   JFR_DEBUG( ublas::prod( subJ1s, jblas::mat(ublas::prod( map1ToMap2Cov, ublas::trans( subJ1s ) ) ) ) );
//   JFR_DEBUG( ublas::prod( subJ2s, jblas::mat(ublas::prod( subP_ik, ublas::trans( subJ2s ) ) ) ) );
  
  subP_ik.assign(
      ublas::prod( subJ1s, jblas::mat(ublas::prod( map1ToMap2Cov, ublas::trans( subJ1s ) ) ) )
      + ublas::prod( subJ2s, jblas::mat(ublas::prod( subP_ik, ublas::trans( subJ2s ) ) ) ) );
  
  // Look for pairs
  std::size_t nextHFreeState = 0;
  std::vector<MergeMapPairInformation> pairInfos;
  for( FeaturesMapType::const_iterator it = map1->featuresMap.begin();
       it != map1->featuresMap.end(); ++it)
  {
    unsigned int featureId = it->first;
    if( map2->hasFeature( featureId ) )
    {
      MergeMapPairInformation mmpi;
      mmpi.featureId = featureId;
      for( std::vector<MergeMapStateInformation* >::iterator it = infos.begin(); it != infos.end(); ++it )
      {
        if( (*it)->featureId == featureId and (*it)->featureModel )
        {
          mmpi.featureMap1 = *it;
          break;
        }
      }
      JFR_ASSERT(mmpi.featureMap1, "Feature " << featureId << " not found in map 1.");
      for( std::vector<MergeMapStateInformation* >::iterator it = infos2.begin(); it != infos2.end(); ++it )
      {
        if( (*it)->featureId == featureId and (*it)->featureModel )
        {
          mmpi.featureMap2 = *it;
          break;
        }
      }
      JFR_ASSERT(mmpi.featureMap2, "Feature " << featureId << " not found in map 2.");
      JFR_ASSERT(mmpi.featureMap1->featureModel == mmpi.featureMap2->featureModel, "The features don't have the same feature model");
      JFR_ASSERT(mmpi.featureMap1 != mmpi.featureMap2, "mergeMap is bogus, both features state infos must be different");
      mmpi.featureMap1->isPair = true;
      mmpi.featureMap2->isPair = true;
      // Initialize the range of the pair of feature in h and H
      int stateSize = mmpi.featureMap1->featureModel->sizeMergeState();
      mmpi.hRange = ublas::range( nextHFreeState, nextHFreeState + stateSize );
      nextHFreeState += stateSize;
      pairInfos.push_back( mmpi );
    }
  }
  
  JFR_ASSERT( not pairInfos.empty(), "No pair found, map merging is impossible");
  // Compute h and H
  jblas::vec h( nextHFreeState );
  jblas::mat H = jblas::zero_mat( nextHFreeState, realStateSize );
  
  for( int i = 0; i < pairInfos.size(); ++i)
  {
    MergeMapPairInformation& mmpi = pairInfos[i];
    JFR_ASSERT( mmpi.featureMap1 != mmpi.featureMap2, "Buggy MergeMapPairInformation");
    
    int mergeStateSize = mmpi.featureMap1->featureModel->sizeMergeState();
//     JFR_DEBUG( mmpi.featureMap1->state << " " << mmpi.featureMap2->state );
    
    jblas::vec v1( mergeStateSize );
    jblas::mat m1( mergeStateSize, mmpi.featureMap1->state.size() );
    mmpi.featureMap1->featureModel->computeMergeState( mmpi.featureMap1->state, v1, m1 );
    jblas::vec v2( mergeStateSize );
    jblas::mat m2( mergeStateSize, mmpi.featureMap2->state.size() );
    //FIXME should it be mmpi.featureMap2?
    mmpi.featureMap1->featureModel->computeMergeState( mmpi.featureMap1->state, v2, m2 );
    
//     JFR_DEBUG(v1 << " " << v2 );
    ublas::project( h, mmpi.hRange ).assign( v1 - v2 );

//     JFR_DEBUG(m1 << " " << m2 );
    ublas::project( H, mmpi.hRange, ublas::range( mmpi.featureMap1->range.start(), mmpi.featureMap1->range.start()+ mmpi.featureMap1->featureModel->sizeState() ) ).assign( m1 );
    ublas::project( H, mmpi.hRange, ublas::range( mmpi.featureMap2->range.start(), mmpi.featureMap2->range.start() + mmpi.featureMap2->featureModel->sizeState() ) ).assign( -m2 );

  }
  
//   JFR_DEBUG(h);
//   JFR_DEBUG(jmath::ublasExtra::prettyFormat(H));
  
//   JFR_DEBUG( " x_ik = " << x_ik );
//   JFR_DEBUG( " P_ik = " << P_ik);
  // 
// TODO check P_ik is diagonal by block  JFR_ASSERT( ublas::equals( ublas::project(P_ik, ublas::range( firstStateOfSecondMap, nextFreeState), ublas::range( 0, firstStateOfSecondMap) ), jblas::zero_mat( nextFreeState - firstStateOfSecondMap, firstStateOfSecondMap ), BOOST_UBLAS_TYPE_CHECK_EPSILON, BOOST_UBLAS_TYPE_CHECK_MIN ), "P_ik isn't diagonal by block" );
  
  
  // Compute update
//   JFR_DEBUG( h );
//   JFR_DEBUG( H );
//   JFR_DEBUG( P_ik );
  ublas::range realStateRange(0,  realStateSize );
  jblas::sym_mat_range subP_ik_2(P_ik, realStateRange, realStateRange);
  jblas::mat HPH = ublas::prod( H, jblas::mat(ublas::prod( subP_ik_2, ublas::trans( H ) ) ) );
  jblas::mat HPHinv( HPH.size1(), HPH.size2());
//   JFR_DEBUG( HPH );
  jmath::ublasExtra::inv( HPH, HPHinv );
  jblas::mat K = jblas::mat( ublas::prod( subP_ik_2, jblas::mat( ublas::prod( ublas::trans( H ), HPHinv ) ) ) );
  ublas::project( x_ik, realStateRange ).assign( ublas::project( x_ik, realStateRange ) - ublas::prod( K, h ) );
  jblas::mat IKH = jblas::identity_mat( realStateSize, realStateSize ) - ublas::prod( K, H );
  subP_ik_2 = ublas::prod( IKH, subP_ik_2 );
  
//   JFR_DEBUG( " x_ik = " << x_ik );
  
  // Construct the filter
  // Add robots from first map
  std::list<unsigned int> robotIds;
  // Update the merge map states information and initialize features and robots in the map
  for( std::vector< MergeMapStateInformation* >::iterator it = infos.begin();
       it != infos.end(); ++it)
  {
    MergeMapStateInformation* info = *it;
    if( info->featureModel and 
        (   info->mapNumber == SlamEkf::MAP_1 or
          ( info->mapNumber == SlamEkf::MAP_2 and not info->isPair ) ) )
    { // It's a feature
      // Insert the new feature in the map
      BaseFeature* feature = featureFactory( info->featureId, *info->featureModel, info->sizeObs, info->typeObs );
      resultMap->mapManager->setMapObjectState( *feature );
      resultMap->featuresMap[ feature->id() ] = feature;
      // Update the Map State Information
      info->abstractMapObjectInResult = feature;
    } else if( info->robotModel and std::find( robotIds.begin(), robotIds.end(), info->featureId ) == robotIds.end() )
    { // It's a new robot
      std::map< unsigned int, SlamEkf::MergeMapNumber >::const_iterator it = _chooseRobotPose.find( info->featureId );
      
      if( it == _chooseRobotPose.end() or it->second == info->mapNumber )
      {
        BaseRobot* robot = new BaseRobot( info->featureId, *info->robotModel );
        robotIds.push_back( robot->id() );
        resultMap->addRobot( robot );
        info->abstractMapObjectInResult = robot;
      }
    }
  }
  
  // Copy the state vector
//   JFR_DEBUG("Copy the state vector");
  for( int i = 0; i < infos.size(); ++i )
  {
    MergeMapStateInformation* info = infos[i];
    if( info->abstractMapObjectInResult )
    {
      info->abstractMapObjectInResult->getX().assign( info->state );
      ublas::range range1 ( info->abstractMapObjectInResult->filterIndex(),
                            info->abstractMapObjectInResult->filterIndex() +
                                info->abstractMapObjectInResult->sizeState() );
      for( int j = 0; j <= i; ++j )
      {
        MergeMapStateInformation* info2 = infos[j];
        if( info2->abstractMapObjectInResult )
        {
          ublas::project( resultMap->filter.getP(), range1,
                          ublas::range( info2->abstractMapObjectInResult->filterIndex(),
                            info2->abstractMapObjectInResult->filterIndex() +
                                info2->abstractMapObjectInResult->sizeState() ) ).assign(
                              ublas::project( P_ik, info->range, info2->range ) );
        }
      }
    }
  }
  
  ublas::range map2Range( 0, resultMap->filter.sizeState() );

  // Delete infos
  for( std::vector< MergeMapStateInformation* >::iterator it = infos.begin();
       it != infos.end(); ++it)
  {
    delete *it;
  }
  
  return resultMap;
}
