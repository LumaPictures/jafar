/* $Id$ */

/** \file jafarTestMacro.hpp Defines usefull macros when writting tests.
 *
 * \ingroup kernel
 */

#ifndef KERNEL_JAFAR_TEST_MACRO_HPP_
#define KERNEL_JAFAR_TEST_MACRO_HPP_

#define TEST_MACRO_EPSILON 1e-6

/**
 * This macro check if a value is true
 */
#define JFR_CHECK(a) BOOST_CHECK_MESSAGE( (a) , (a) << " is not true");

/**
 * This macro check if a value is nearly null ( fabs(a) \< TEST_MACRO_EPSILON )
 */
#define JFR_CHECK_NULL(a) BOOST_CHECK_MESSAGE(fabs(a) < TEST_MACRO_EPSILON, (a) << " is not null");
/**
 * This macro check that two values are nearly equal ( fabs(a - b) \< TEST_MACRO_EPSILON )
 */
#define JFR_CHECK_EQUAL(a,b) BOOST_CHECK_MESSAGE( fabs((a) - (b)) < TEST_MACRO_EPSILON, (a) << " != " << (b ));
/**
 * This macro check that two values aren't nearly equal ( fabs(a - b) \> TEST_MACRO_EPSILON )
 */
#define JFR_CHECK_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( fabs((a) - (b)) > TEST_MACRO_EPSILON, (a) << " == " << (b ));
/**
 * This macro check that two values are strictly equal ( a == b )
 */
#define JFR_CHECK_STRICT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (a) == (b), (a) << " != " << (b ));
/**
 * This macro check that two values are strictly different ( a != b )
 */
#define JFR_CHECK_STRICT_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (a) != (b), (a) << " == " << (b ));
/**
 * This macro check that two vectors are nearly equal ( |a - b | / (|a|*|b|) \< TEST_MACRO_EPSILON )
 */
#define JFR_CHECK_VEC_EQUAL(a,b) BOOST_CHECK_MESSAGE( (ublas::norm_2((a) - (b)) / (ublas::norm_2(a) * ublas::norm_2(b))) < TEST_MACRO_EPSILON, (a) << " != " << (b) );
/**
 * This macro check that a vector is nearly null ( |a| \< TEST_MACRO_EPSILON)
 */
#define JFR_CHECK_VEC_NULL(a) BOOST_CHECK_MESSAGE( ublas::norm_2(a) < TEST_MACRO_EPSILON, (a) << " != 0 ");
/**
 * This macro check that two vectors aren't nearly equal ( |a - b | / (|a|*|b|) \> TEST_MACRO_EPSILON )
 */
#define JFR_CHECK_VEC_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (ublas::norm_2((a) - (b)) / (ublas::norm_2(a) * ublas::norm_2(b)) ) > TEST_MACRO_EPSILON, (a) << " == " << (b) );
/**
 * This macro check that two matrix are nearly equal ( tr( (a - b) * transpose(a-b) ) ) \< TEST_MACRO_EPSILON)
 */
#define JFR_CHECK_MAT_EQUAL(a,b) BOOST_CHECK_MESSAGE( jmath::ublasExtra::trace( ublas::prod( ((a) - (b)), ublas::trans((a)-(b)) ) ) < TEST_MACRO_EPSILON, (a) << " != " << (b) );
/**
 * This macro check that two matrix aren't nearly equal ( tr( (a - b) * transpose(a-b) ) ) \> TEST_MACRO_EPSILON)
 */
#define JFR_CHECK_MAT_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( jmath::ublasExtra::trace( ublas::prod( ((a) - (b)), ublas::trans((a)-(b)) ) ) > TEST_MACRO_EPSILON, (a) << " == " << (b) );

/**
 * Use this macro instead of BOOST_AUTO_TEST_CASE to define a test in case you want
 * jafar exception to be catched and display.
 * 
 * You need to call \ref JAFAR_AUTO_TEST_CASE_END after to finish the creation of
 * the test case. Exceptions will be reported as Boost.Test classical error.
 */
#define JAFAR_AUTO_TEST_CASE_BEGIN( _TEST_NAME_ ) \
  BOOST_AUTO_TEST_CASE( _TEST_NAME_ ) \
  { \
    try

/**
 * Finalize a test that was created with \ref JAFAR_AUTO_TEST_CASE_BEGIN .
 */
#define JAFAR_AUTO_TEST_CASE_END() \
  catch( jafar::kernel::JafarException ex ) \
  { \
    std::cout << "** jafar::kernel::JafarException: **" << std::endl; \
    std::cout << ex << std::endl; \
    BOOST_CHECK_MESSAGE(false, ex ); \
  } \
  catch (std::exception& ex) { \
    std::cout << "** std::exception: **" << std::endl; \
    std::cout << ex.what() << std::endl; \
    BOOST_CHECK_MESSAGE(false, ex.what() ); \
  } \
}

#endif
