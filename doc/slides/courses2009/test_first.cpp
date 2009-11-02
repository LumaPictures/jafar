#include <boost/test/auto_unit_test.hpp>
#include <kernel/jafarTestMacro.hpp>
#include "playmodule/MyFirstClass.hpp"

BOOST_AUTO_TEST_CASE( test_MyFirstClass )
{
  MyFirstClass mfc;
  JFR_CHECK_EQUAL( mfc.add(1, 2), 3 );
}
