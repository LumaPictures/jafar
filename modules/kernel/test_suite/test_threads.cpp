/* $Id: test_suite_kernel.cpp 2643 2007-10-22 15:06:49Z cberger $ */

// boost unit test includes
#include <boost/test/auto_unit_test.hpp>

// jafar debug include
#include "kernel/jafarDebug.hpp"
#include "kernel/jafarTestMacro.hpp"
#include "kernel/threads.hpp"

#include <sys/time.h>
#include <stdio.h>

using namespace jafar;


/******************************************************************************/

kernel::FifoMutex m;

void writer()
{
	struct timeval tv; struct timezone tz;
	while(true)
	{
		//gettimeofday(&tv,&tz); printf("%d.%06d writer wait\n",(int)tv.tv_sec,(int)tv.tv_usec);
		boost::unique_lock<kernel::FifoMutex> l(m);
		gettimeofday(&tv,&tz); printf("%d.%06d writer run\n",(int)tv.tv_sec,(int)tv.tv_usec);
		sleep(1);
		gettimeofday(&tv,&tz); printf("%d.%06d writer release\n",(int)tv.tv_sec,(int)tv.tv_usec);
		l.unlock();
		//sleep(5);
	}
}
void reader1()
{
	struct timeval tv; struct timezone tz;
	while(true)
	{
		//gettimeofday(&tv,&tz); printf("%d.%06d reader1 wait\n",(int)tv.tv_sec,(int)tv.tv_usec);
		boost::unique_lock<kernel::FifoMutex> l(m);
		gettimeofday(&tv,&tz); printf("%d.%06d reader1 run\n",(int)tv.tv_sec,(int)tv.tv_usec);
		m.lock();
		sleep(1);
		gettimeofday(&tv,&tz); printf("%d.%06d reader1 release\n",(int)tv.tv_sec,(int)tv.tv_usec);
		l.unlock();
		sleep(5);
	}
}
void reader2()
{
	struct timeval tv; struct timezone tz;
	while(true)
	{
		//gettimeofday(&tv,&tz); printf("%d.%06d reader2 wait\n",(int)tv.tv_sec,(int)tv.tv_usec);
		while (!m.try_lock());
		//m.lock();
		gettimeofday(&tv,&tz); printf("%d.%06d reader2 run\n",(int)tv.tv_sec,(int)tv.tv_usec);
		sleep(1);
		gettimeofday(&tv,&tz); printf("%d.%06d reader2 release\n",(int)tv.tv_sec,(int)tv.tv_usec);
		m.unlock();
		sleep(5);
	}
}



BOOST_AUTO_TEST_CASE( test_threads )
{

	
	boost::thread *thread_writer, *thread_reader1, *thread_reader2;
	thread_writer = new boost::thread(writer);
	sleep(1);
	thread_reader1 = new boost::thread(reader1);
	sleep(1);
	thread_reader2 = new boost::thread(reader2);
	sleep(1000);

}

