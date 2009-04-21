/* $Id$ */

#include <boost/test/auto_unit_test.hpp>

#include <slam/MultiMapManager.hpp>

#include <kernel/jafarTestMacro.hpp>

using namespace jafar;
using namespace jafar::slam;

jblas::vec str2vec(const std::string& value) {
  std::stringstream s;
  s << value;
  jblas::vec v(6);
  s >> v;
  return v;
};

JAFAR_AUTO_TEST_CASE_BEGIN( test_list_maps_pose_manager )
{
  ListMapsPoseManager lmpm(100);
  lmpm.addFirstMapPose(0,jblas::zero_vec(6), 0.01 * jblas::identity_mat(6,6));
  {
    jafar::geom::T3DEuler wtm = lmpm.getWorldToMapTransformation(0);
    JFR_CHECK_VEC_NULL( wtm.getX() );
  }

  lmpm.addMapPose( 1, 0, str2vec("[6](2,2,0,-1.5707963267,0,0)"), 0.01 * jblas::identity_mat(6,6) );
  {
    jafar::geom::T3DEuler wtm = lmpm.getWorldToMapTransformation(1);
    JFR_CHECK_VEC_EQUAL( wtm.getX(), str2vec("[6](2,2,0,-1.5707963267,0,0)") );
  }
  lmpm.addMapPose( 2, 1, str2vec("[6](4,0,0,-1.5707963267,0,0)"), 0.01 * jblas::identity_mat(6,6) );
  {
    jafar::geom::T3DEuler wtm = lmpm.getWorldToMapTransformation(2);
    JFR_CHECK_VEC_EQUAL( wtm.getX(), str2vec("[6](2,-2,0,-3.1415926535,0,0)") );
  }
  lmpm.addMapPose( 3, 2, str2vec("[6](2,-1.8,0,-3.1215926535,0,0)"), 0.01 * jblas::identity_mat(6,6) );
  {
    jafar::geom::T3DEuler wtm = lmpm.getWorldToMapTransformation(3);
    JFR_CHECK_VEC_EQUAL( wtm.getX(), str2vec("[6](0,-0.2,0,0.02,0,0)") );
  }
//   lmpm.enforceLoopConstraint( 1, 0, jblas::zero_vec(6), 0.00 * jblas::identity_mat(6,6) );
  
  lmpm.enforceLoopConstraint( 2, 1, jblas::zero_vec(6), 0.00 * jblas::identity_mat(6,6) );
  JFR_DEBUG( lmpm.getWorldToMapTransformation( 0 ) );
  JFR_DEBUG( lmpm.getMapToMapTransformation( 0, 1 ) );
  JFR_DEBUG( lmpm.getWorldToMapTransformation( 1 ) );
  JFR_DEBUG( lmpm.getMapToMapTransformation( 1, 2 ) );
  JFR_DEBUG( lmpm.getWorldToMapTransformation( 2 ) );
  JFR_DEBUG( lmpm.getMapToMapTransformation( 2, 3 ) );
  JFR_DEBUG( lmpm.getWorldToMapTransformation( 3 ) );
}
JAFAR_AUTO_TEST_CASE_END()


