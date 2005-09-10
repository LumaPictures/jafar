/* $Id$ */

// boost unit test includes
#include <boost/test/unit_test.hpp>
using boost::unit_test_framework::test_suite;
using boost::unit_test_framework::test_case;

// jafar debug include
#include "kernel/jafarDebug.hpp"

// include here your defined test suite
//#include "test_suite_MyClass.hpp"


//using namespace jafar::_jfr_module_;


/*
 * standard init_unit_test_suite function
 */

test_suite*
init_unit_test_suite( int, char* [] ) {

  // we send debug message to test_suite__jfr_module_.dbg so that they
  // do not pollute unit test output
  jafar::kernel::JafarDebug::setOutputFile("test_suite__jfr_module_.dbg");

  // module helloworld test suite
  test_suite* test= BOOST_TEST_SUITE( "_jfr_module_ module test suite" );

  // add here your test suite to the module test suite
  //test->add( new test_suite_MyClass() );

  return test;
}
