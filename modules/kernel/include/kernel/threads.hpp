#ifndef _KERNEL_THREADS_H_
#define _KERNEL_THREADS_H_


#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace boost {
	typedef interprocess::interprocess_semaphore semaphore;
}

namespace jafar {
namespace kernel {

	/**
	This class implements a mutex with FIFO scheduling policy,
	based on boost mutexes and semaphores. When the mutex is unlocked, the thread
	that takes the mutex is the first one that tried to lock it.
	*/
	class FifoMutex
	{
		private:
			boost::mutex m;
			typedef std::map<boost::thread::id,boost::semaphore*> SemsMap;
			SemsMap allSems;
			typedef std::list<boost::semaphore*> SemsList;
			SemsList waitingSems;
			boost::semaphore *computingSem;

		public:
			FifoMutex(): computingSem(NULL) {}
			
			void lock()
			{
				m.lock();
				// get this thread's sem
				boost::thread::id id = boost::this_thread::get_id();
				SemsMap::iterator itsem = allSems.find(id);
				boost::semaphore *sem = (itsem == allSems.end()) ? new boost::semaphore(0) : (*itsem).second;
				// process
				if (computingSem == sem)
				{
					m.unlock();
				} else
				if (computingSem == NULL) 
				{
					computingSem = sem; 
					m.unlock();
				} else
				{
					waitingSems.push_back(sem);
					/* hum, unlock and wait not atomic...
					The only thing that can happen, is that this is the only sem in waiting list,
					and between unlock and wait the computing thread releases and post on this
					sem before/at the same times it waits.
					It's ok to post before to wait, and sems are protected by a mutex, 
					so everything should be ok.
					*/
					m.unlock(); 
					sem->wait();
				}
			}
			
			void unlock()
			{
				m.lock();
				// optionally here we could check that it is the active thread that is unlocking...
				if (false)
				{
					boost::thread::id id = boost::this_thread::get_id();
					SemsMap::iterator itsem = allSems.find(id);
					boost::semaphore *sem = (itsem == allSems.end()) ? NULL : (*itsem).second;
					if (sem != computingSem) { m.unlock(); return; }
				}
				// process
				if (waitingSems.empty())
				{
					computingSem = NULL;
				} else
				{
					computingSem = waitingSems.front();
					waitingSems.pop_front();
					computingSem->post();
				}
				m.unlock();
			}


	};

}}

#endif

