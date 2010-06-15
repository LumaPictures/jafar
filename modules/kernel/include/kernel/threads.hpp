#ifndef _KERNEL_THREADS_H_
#define _KERNEL_THREADS_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

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
	class FifoMutex: public boost::noncopyable
	{
		private:
			boost::mutex m;
			typedef std::pair<boost::semaphore*,bool> SemWait;
			typedef std::pair<boost::thread::id, SemWait> SemThread;
			typedef std::map<boost::thread::id, SemWait> SemsMap;
			SemsMap allSems;
			typedef std::list<SemsMap::iterator> SemsList;
			SemsList waitingSems;
			boost::optional<SemsMap::iterator> computingSem;
			
			bool lock_(bool try_);
			
		public:
			
			void lock()
			{
				lock_(false);
			}
			bool try_lock()
			{
				return lock_(true);
			}
			
			void unlock();
			
			bool own_lock()
			{
				boost::thread::id id = boost::this_thread::get_id();
				return (computingSem && (*computingSem)->first == id);
			}

			~FifoMutex()
			{
				for(SemsMap::iterator it = allSems.begin(); it != allSems.end(); ++it)
					delete it->second.first;
			}

	};
	
	/**
	@warning not working
	instead, use priority inheritance : set current prio, start thread, reset current prio,
	and the created thead will inherit the prio of its parent thread
	*/
	int setThreadPriority(boost::thread &t, int prio);
	
	/**
	The program needs to be privileged to be able to increase its priority beyond normal.
	@param prio between -20 (highest priority), to 20 (lowest priority), including 0 (normal priority). 
	@return 0 if successful
	*/
	int setCurrentThreadPriority(int prio);
	
}}

#endif

