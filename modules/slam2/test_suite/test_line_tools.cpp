/* $Id: */

#include <boost/test/auto_unit_test.hpp>

#include <kernel/jafarTestMacro.hpp>

#include "slam/lineTools.hpp"

using namespace jafar;
using namespace jafar::slam;

            jblas::vec2 homogeneousToRhoTheta(jblas::vec3 const& l);
            jblas::vec3 rhoThetaToHomogeneous(jblas::vec2 const& rt);

inline double random_float() {
  return rand() / (double)rand();
}

template<typename V>
inline V random_vec()
{
    V v;
    for(uint i = 0; i < v.size(); i++)
    {
        v(i) = random_float();
    }
    return v;
}

JAFAR_AUTO_TEST_CASE_BEGIN( test_line_tools )
{
  {
    jblas::vec3 l1 = random_vec<jblas::vec3>();
    l1 /= ublas::norm_2( l1 );
    jblas::vec2 rt1 = lineTools::homogeneousToRhoTheta( l1 );
    jblas::vec3 l2 = lineTools::rhoThetaToHomogeneous( rt1 );
    l2 /= ublas::norm_2( l2 );
    if( l1(0) * l2(0) < 0.0 )
    {
      l2 *= -1;
    }
    JFR_CHECK_VEC_EQUAL( l1, l2 );
  }
  {
    jblas::vec2 rt1 = random_vec<jblas::vec2>();
    jblas::vec3 l1 = lineTools::rhoThetaToHomogeneous( rt1 );
    jblas::vec3 rt2 = lineTools::homogeneousToRhoTheta( l1 );
    JFR_CHECK_VEC_EQUAL( rt1, rt2 );
  }
  
}
JAFAR_AUTO_TEST_CASE_END()
