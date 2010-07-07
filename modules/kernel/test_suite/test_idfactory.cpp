/* $Id: test_suite_kernel.cpp 2643 2007-10-22 15:06:49Z cberger $ */

// boost unit test includes
#include <boost/test/auto_unit_test.hpp>

// jafar debug include
#include "kernel/jafarDebug.hpp"
#include "kernel/jafarTestMacro.hpp"
#include "kernel/IdFactory.hpp"

using namespace jafar;

BOOST_AUTO_TEST_CASE( test_idfactory )
{
	kernel::IdFactory<unsigned,kernel::IdCollectorNone> if_none;
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 0);
	BOOST_CHECK_EQUAL(if_none.getId(), 1);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 1);
	BOOST_CHECK_EQUAL(if_none.getId(), 2);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_none.getId(), 3);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_none.getId(), 4);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_none.releaseId(4), true);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.getId(), 4);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_none.releaseId(3), false);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.releaseId(2), false);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.getId(), 5);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 5);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.releaseId(4), false);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 5);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.getId(), 6);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 6);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_none.releaseId(7), false);
	BOOST_CHECK_EQUAL(if_none.countUsed(), 6);
	BOOST_CHECK_EQUAL(if_none.countCollected(), 0);
	
	
	kernel::IdFactory<unsigned,kernel::IdCollectorList> if_list;
	BOOST_CHECK_EQUAL(if_list.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 0);
	BOOST_CHECK_EQUAL(if_list.getId(), 1);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 1);
	BOOST_CHECK_EQUAL(if_list.getId(), 2);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_list.getId(), 3);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.getId(), 4);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_list.releaseId(4), true);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_list.getId(), 4);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_list.releaseId(3), true);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_list.releaseId(2), true);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 2);
	BOOST_CHECK_EQUAL(if_list.getId(), 3);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_list.releaseId(4), true);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_list.getId(), 2);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_list.releaseId(7), false);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_list.releaseId(2), true); // 4 is already released, crap !
	BOOST_CHECK_EQUAL(if_list.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_list.releaseId(2), true);
	BOOST_CHECK_EQUAL(if_list.countUsed(), 1);
	BOOST_CHECK_EQUAL(if_list.countCollected(), 2);
	
	
	kernel::IdFactory<unsigned,kernel::IdCollectorSet> if_set;
	BOOST_CHECK_EQUAL(if_set.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 0);
	BOOST_CHECK_EQUAL(if_set.getId(), 1);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 1);
	BOOST_CHECK_EQUAL(if_set.getId(), 2);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_set.getId(), 3);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_set.getId(), 4);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_set.releaseId(4), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_set.getId(), 4);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 4);
	BOOST_CHECK_EQUAL(if_set.releaseId(3), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_set.releaseId(2), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 2);
	BOOST_CHECK_EQUAL(if_set.getId(), 2);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_set.releaseId(4), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_set.getId(), 3);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 3);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 0);
	BOOST_CHECK_EQUAL(if_set.releaseId(2), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_set.releaseId(2), false);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 2);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 1);
	BOOST_CHECK_EQUAL(if_set.releaseId(1), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 1);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 2);
	BOOST_CHECK_EQUAL(if_set.releaseId(3), true);
	BOOST_CHECK_EQUAL(if_set.countUsed(), 0);
	BOOST_CHECK_EQUAL(if_set.countCollected(), 0);
}

