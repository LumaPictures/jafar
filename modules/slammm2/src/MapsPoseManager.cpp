/* $Id$ */

#include "slammm/MapsPoseManager.hpp"

#include "jmath/ublasExtra.hpp"
#include "boost/numeric/bindings/lapack/gesdd.hpp"
#include "boost/numeric/bindings/lapack/syev.hpp"

#include "slam/featureModel.hpp"
#include "slam/pointInvDepthFeature.hpp"

#include "jacobianEuler.hpp"

using namespace jafar;
using namespace jafar::slammm;
namespace lapack = boost::numeric::bindings::lapack;

void MapsPoseManager::enforceLoopConstraint( unsigned int _firstMapId, unsigned int _secondMapId, const geom::T3DEuler& _transformation )
{
  enforceLoopConstraint( _firstMapId, _secondMapId, _transformation.getX(), _transformation.getXCov() );
}
