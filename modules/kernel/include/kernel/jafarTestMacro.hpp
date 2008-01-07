/* $Id$ */

/** \file jafarTestMacro.hpp Defines usefull macros when writting tests.
 *
 * \ingroup kernel
 */

#ifndef KERNEL_JAFAR_TEST_MACRO_HPP_
#define KERNEL_JAFAR_TEST_MACRO_HPP_

#define EPSILON 1e-6

/**
 * This macro check if a value is nearly null ( fabs(a) \< EPSILON )
 */
#define JFR_CHECK_NULL(a) BOOST_CHECK_MESSAGE(fabs(a) < EPSILON, (a) << " is not null");
/**
 * This macro check that two values are nearly equal ( fabs(a - b) \< EPSILON )
 */
#define JFR_CHECK_EQUAL(a,b) BOOST_CHECK_MESSAGE( fabs((a) - (b)) < EPSILON, (a) << " != " << (b ));
/**
 * This macro check that two values are strictly equal ( a == b )
 */
#define JFR_CHECK_STRICT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (a) == (b), (a) << " != " << (b ));
/**
 * This macro check that two values are strictly different ( a != b )
 */
#define JFR_CHECK_STRICT_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (a) != (b), (a) << " == " << (b ));
/**
 * This macro check that two vectors are nearly equal ( |a - b | / (|a|*|b|) \< EPSILON )
 */
#define JFR_CHECK_VEC_EQUAL(a,b) BOOST_CHECK_MESSAGE( (ublas::norm_2((a) - (b)) / (ublas::norm_2(a) * ublas::norm_2(b))) < EPSILON, (a) << " != " << (b) );
/**
 * This macro check that a vector is nearly null ( |a| \< EPSILON)
 */
#define JFR_CHECK_VEC_NULL(a) BOOST_CHECK_MESSAGE( ublas::norm_2(a) < EPSILON, (a) << " != 0 ");
/**
 * This macro check that two vectors aren't nearly equal ( |a - b | / (|a|*|b|) \> EPSILON )
 */
#define JFR_CHECK_VEC_NOT_EQUAL(a,b) BOOST_CHECK_MESSAGE( (ublas::norm_2((a) - (b)) / (ublas::norm_2(a) * ublas::norm_2(b)) ) > EPSILON, (a) << " == " << (b) );
/**
 * This macro check that two matrix are nearly equal ( tr( (a - b) * transpose(a-b) ) ) \< EPSILON)
 */
#define JFR_CHECK_MAT_EQUAL(a,b) BOOST_CHECK_MESSAGE( jmath::ublasExtra::trace( ublas::prod( ((a) - (b)), ublas::trans((a)-(b)) ) ) < EPSILON, (a) << " != " << (b) );

#endif
