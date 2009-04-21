/* $Id$ */

#include "slammm/StartNewMapStrategy.hpp"

#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

#include "slam/featureModel.hpp"
#include "slam/pointInvDepthFeature.hpp"

#include "jacobianEuler.hpp"

using namespace jafar;
using namespace jafar::slammm;
namespace lapack = boost::numeric::bindings::lapack;

StartNewMapStrategy::~StartNewMapStrategy()
{
}

FeaturesCountStartNewMapStrategy::FeaturesCountStartNewMapStrategy( unsigned int _maxFeatures ) : m_maxFeatures(_maxFeatures)
{
}

FeaturesCountStartNewMapStrategy::~FeaturesCountStartNewMapStrategy()
{
}

bool FeaturesCountStartNewMapStrategy::startNewMap( const slam::SlamEkf& _currentMap, unsigned int, unsigned int _newFeaturesCount ) const
{
  return ( _currentMap.featuresMap.size() > m_maxFeatures ) and _newFeaturesCount != 0 and _newFeaturesCount != _currentMap.featuresMap.size();
}
