/* $Id$ */

#include "slammm/MergeMapsStrategy.hpp"

#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

#include "slam/featureModel.hpp"
#include "slam/pointInvDepthFeature.hpp"

#include "jacobianEuler.hpp"

#include "slammm/ListMapsPoseManager.hpp"
#include "slammm/MultiMapManager.hpp"

using namespace jafar;
using namespace jafar::slammm;
namespace lapack = boost::numeric::bindings::lapack;

MergeMapsStrategy::MergeMapsStrategy() : m_multiMapsSlam(0)
{
}

MergeMapsStrategy::~MergeMapsStrategy()
{
}

void MergeMapsStrategy::setMultiMapsSlam( MultiMapsSlam* _multiMapsSlam)
{
  JFR_ASSERT( m_multiMapsSlam == 0, "A MultiMapsSlam instance has already being assigned to this MergeMapsStragegy" );
  m_multiMapsSlam = _multiMapsSlam;
}

MultiMapsSlam* MergeMapsStrategy::multiMapsSlam()
{
  return m_multiMapsSlam;
}

AlwaysMergeMapsStrategy::AlwaysMergeMapsStrategy( int minimalOverlapingFeaturesNumber ) : m_minimalOverlapingFeaturesNumber(minimalOverlapingFeaturesNumber)
{
}

void AlwaysMergeMapsStrategy::getFeatureCov( slam::BaseFeature& baseFeature, jblas::vec& point, jblas::sym_mat& cov )
{
  slam::FeatureModel* model = &baseFeature.model;
  slam::PointFeatureModel* pmodel = dynamic_cast<slam::PointFeatureModel*>( model );
  if( pmodel )
  {
    point.assign( baseFeature.getX() );
    cov.assign( baseFeature.getP() );
  } else {
    slam::PointInvDepthFeatureModel* pidmodel = dynamic_cast<slam::PointInvDepthFeatureModel*>( model );
    JFR_ASSERT( pidmodel, "Unsupported Feature model" );
    pidmodel->compute3dPoint( jblas::vec( baseFeature.getX() ), point );
    pidmodel->compute3dPointJac( jblas::vec( baseFeature.getX() ) );
    cov.assign( ublas::prod(pidmodel->J3dPoint, jblas::mat( ublas::prod( baseFeature.getP(), ublas::trans( pidmodel->J3dPoint ) ) ) ) );
  }
}

struct AlwaysMergeMapsStrategy::DMMInfo {
  DMMInfo() : point1(3), point2(3), cov1(3,3), cov2(3,3), weight(0.0) { }
  jblas::vec point1, point2;
  jblas::sym_mat cov1, cov2;
  double weight;
};

void AlwaysMergeMapsStrategy::decideMergeMap( unsigned int currentMapId, slam::SlamEkf* currentMap )
{
  const slam::SlamEkf::FeaturesMapType& fm1 = currentMap->getMap();
  const std::map< unsigned int, slam::SlamEkf*> maps = multiMapsSlam()->getMaps();
  std::list< int > feature2feature;
  unsigned int bestMapForMergeId = -1;
  slam::SlamEkf* bestMap = 0;
  // Choose a map with enough common features to initiate a merge
  for( std::map< unsigned int, slam::SlamEkf*>::const_iterator it = maps.begin();
       it != maps.end(); ++it )
  {
    if( it->first != currentMapId )
    {
      const slam::SlamEkf::FeaturesMapType& fm2 = it->second->getMap();
      std::list< int > feature2featureCurrent;
      for( slam::SlamEkf::FeaturesMapType::const_iterator featureIt1 = fm1.begin();
           featureIt1 != fm1.end(); ++featureIt1 )
      {
        for( slam::SlamEkf::FeaturesMapType::const_iterator featureIt2 = fm2.begin();
             featureIt2 != fm2.end(); ++featureIt2 )
        {
          if( featureIt1->first == featureIt2->first )
          {
            feature2featureCurrent.push_back( featureIt1->first );
          }
        }
      }
      if( feature2featureCurrent.size() > feature2feature.size() and feature2featureCurrent.size() >= m_minimalOverlapingFeaturesNumber )
      {
        feature2feature = feature2featureCurrent;
        bestMapForMergeId = it->first;
        bestMap = it->second;
      }
    }
  }
  JFR_DEBUG( "Best features count = " << feature2feature.size() );
  JFR_ASSERT( bestMap != currentMap, "Bug, best map can't be current map.");
  if( feature2feature.size() >= m_minimalOverlapingFeaturesNumber )
  { // We found a map with enough common features
    // FIXME Work with a mix of points, segments, facets, whatever
    
    // Initialise DMMInfo
    std::list< DMMInfo > infos;
    
    // barycenters
    // \bar{p}
    jblas::vec p_1_bar(3);
    p_1_bar.clear();
    // \bar{p'}
    jblas::vec p_2_bar(3);
    p_2_bar.clear();
    
    double sumWeight = 0.0;
    
    // Initialise the DMMInfos and compute the barycenter of all points
    for( std::list< int >::iterator it = feature2feature.begin();
         it != feature2feature.end(); ++it )
    {
      DMMInfo info;
      slam::BaseFeature& feature1 = currentMap->getFeature( *it );
      slam::BaseFeature& feature2 = bestMap->getFeature( *it );
      getFeatureCov( feature1, info.point1, info.cov1 );
      getFeatureCov( feature2, info.point2, info.cov2 );
      
      // Compute the weight
      
      jblas::vec lambda1(3);
      ublas::matrix<double, ublas::column_major> A1(ublas::project(info.cov1, ublas::range(0,3), ublas::range(0,3))); 
      int ierr1 = lapack::syev( 'N', 'U', A1, lambda1, lapack::optimal_workspace() );

      jblas::vec lambda2(3);
      ublas::matrix<double, ublas::column_major> A2(ublas::project(info.cov2, ublas::range(0,3), ublas::range(0,3))); 
      int ierr2 = lapack::syev( 'N', 'U', A2, lambda2, lapack::optimal_workspace() );
      
      if( jmath::ublasExtra::maxV( lambda1 ) < 0.1 and jmath::ublasExtra::maxV( lambda2 ) < 0.1 and ierr1 ==0 and ierr2 ==0 )
      {
        info.weight = 1.0 / ( ublas::sum( lambda1 ) + ublas::sum( lambda2 ) );
      
        sumWeight += info.weight;
      
        p_1_bar.plus_assign( info.point1 * info.weight );
        p_2_bar.plus_assign( info.point2 * info.weight );
      
        JFR_DEBUG( *it << " " << info.point1 << " " << info.point2 << " " << info.weight << " " << info.cov1 << " " << info.cov2 << " " << lambda1 << " " << ublas::sum( lambda1 ) << " " << jmath::ublasExtra::maxV( lambda1 ) << " " << lambda2 << " " << ublas::sum( lambda2 ) << " " << jmath::ublasExtra::maxV( lambda2 ) );
        JFR_DEBUG("Filter cov: " << feature1.getP() << " " << feature2.getP() );
      
        infos.push_back( info );
      }
    }
    if(infos.size() <= m_minimalOverlapingFeaturesNumber )
    {
      return;
    }
    p_1_bar /= sumWeight;
    p_2_bar /= sumWeight;
    
    JFR_DEBUG( "p_1_bar = " << p_1_bar << " p_2_bar = " << p_2_bar << " sumWeight = " << sumWeight );
    
    jblas::mat_column_major CovPPp(3,3);
    CovPPp.clear();
    
    // Compute the covariance
    jblas::vec3 dp;
    jblas::vec3 dpp;
    for( std::list< DMMInfo >::iterator it = infos.begin() ; it != infos.end() ; ++it)
    {
      dp.assign( it->point1 - p_1_bar );
      dpp.assign( it->point2 - p_2_bar );
      CovPPp.plus_assign( it->weight * outer_prod( dp, dpp) );
    }
    CovPPp /= sumWeight;
    JFR_DEBUG(CovPPp);
    
    // svd
    jblas::vec s(3);
    jblas::mat_column_major U(3, 3);
    jblas::mat_column_major VT(3, 3);

    int ierr = lapack::gesdd(CovPPp,s,U,VT);
    if (ierr!=0) {
      throw(jmath::LapackException(ierr, 
            "VmeEngine::estimateMotion: error in lapack::gesdd() routine.",
            __FILE__,
            __LINE__));
    }
    
    jblas::mat33 R;
    // compute rotation matrix
    if (jmath::ublasExtra::det(U)*jmath::ublasExtra::det(trans(VT)) > 0)
      R.assign( prod(U, VT) );
    else {
      jblas::diag_mat S(3);
      S(0,0) = 1.0;
      S(1,1) = 1.0;
      S(2,2) = -1.0;
      R.assign(ublas::prod(U, jblas::mat(prod(S, VT))));
    }
    
    JFR_DEBUG( "R = " << R );
    JFR_DEBUG( "R * p_1_bar = " << ublas::prod(R, p_1_bar) << " R * p_2_bar = " <<  ublas::prod(R, p_2_bar) );
    JFR_DEBUG( "p_1_bar - R * p_2_bar = " << jblas::vec3( p_1_bar - ublas::prod(R, p_2_bar) ) );
    JFR_DEBUG( "p_2_bar - R * p_1_bar = " << jblas::vec3( p_2_bar - ublas::prod(R, p_1_bar) ) );

    // compute translation and create transfo
    geom::T3DEuler transfo;
    transfo.set( R, jblas::vec3( p_1_bar - ublas::prod(R, p_2_bar) ) );
    
    JFR_DEBUG( transfo.getX() << " " << transfo.getM() );
    
    jblas::vec transfoX = transfo.getX();
    
    // covariance matrix
    jblas::sym_mat TCov(6);
    TCov.clear();
  
    // jacobian matrix
  
    // dG/dT
    // this jacobian is symmetric
    jblas::sym_mat JGT(6,6);
    JGT.clear();
    jblas::sym_mat JGTinv(6,6);
    // dg/dT
    jblas::sym_mat JgT(6,6);
    // dg/dp
    jblas::mat Jgp(6,3);
    // dg/dpp
    jblas::mat Jgpp(6,3);
    
    // compute transformation uncertainty
    for (std::list< DMMInfo >::iterator it = infos.begin() ; it != infos.end() ; ++it) {

    // compute jacobians
      gJacTEuler( it->point1, it->point1, transfoX, JgT);
      gJacpEuler( it->point1, it->point2, transfoX, Jgp);
      gJacppEuler( it->point1, it->point2, transfoX, Jgpp);

      JGT.plus_assign(JgT);

      TCov.plus_assign(ublas::prod(Jgp, jblas::mat( ublas::prod(it->cov1, ublas::trans(Jgp)) )) +
          ublas::prod(Jgpp, jblas::mat( prod( it->cov2, ublas::trans(Jgpp)) )) );
    }
    
    jmath::ublasExtra::lu_inv(JGT, JGTinv);

    TCov = ublas::prod(JGTinv, jblas::mat( ublas::prod(TCov, JGTinv)) );

    transfo.set(transfoX, TCov);
    
    // Trigger the map merging
//     multiMapsSlam()->mergeMap( currentMapId, bestMapForMergeId, transfo );
    JFR_DEBUG( currentMapId << " " << bestMapForMergeId << " " << transfo );
//     abort();
    multiMapsSlam()->closeLoop( currentMapId, bestMapForMergeId, transfo );
  }
}
