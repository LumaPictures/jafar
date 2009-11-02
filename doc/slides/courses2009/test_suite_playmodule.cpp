/* $Id$ */

// boost unit test includes
#define BOOST_TEST_MAIN 
#define BOOST_TEST_DYN_LINK 
#include <boost/test/auto_unit_test.hpp>
using boost::unit_test_framework::test_suite;
using boost::unit_test_framework::test_case;

#include "kernel/jafarDebug.hpp"

BOOST_AUTO_TEST_CASE( dummy ){}

test_suite*
init_unit_test_suite( int, char* [] ) {

  // we set the debug level to Warning
  jafar::debug::DebugStream::setDefaultLevel(jafar::debug::DebugStream::Warning);
  return 0;
}

