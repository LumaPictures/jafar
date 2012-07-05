#ifndef _KERNEL_THREADS_H_
#define _KERNEL_THREADS_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/lambda/lambda.hpp>

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
	*/
	void setCurrentThreadPriority(int prio);

	/**
	 * @brief setCurrentThreadScheduler
	 * @param policy SCHED_OTHER (default), SCHED_FIFO, SCHED_RR... (see man sched_setscheduler)
	 * @param priority
	 */
	void setCurrentThreadScheduler(int policy, int priority);

	/**
	 * @brief setProcessScheduler
	 * @param policy SCHED_OTHER (default), SCHED_FIFO, SCHED_RR... (see man sched_setscheduler)
	 * @param priority
	 */
	void setProcessScheduler(int policy, int priority);

	
	/**
	This class wraps a simple variable with a mutex
	very limited but can be useful for lazy people
	respects the Lockable concept
	*/
	template<typename T>
	class VariableMutex
	{
		public:
			T var; ///< @warning direct access to the variable, needs manual locking! Provided for efficiency purpose when doing several operations.
			
		protected:
			boost::mutex m;
			
		public:
			VariableMutex(const T &val_init): var(val_init) {}
			
			T get() { boost::unique_lock<boost::mutex> l(m); return var; }
			template<typename Assign> void apply(Assign assign) { boost::unique_lock<boost::mutex> l(m); assign(var); }
			void set(const T &val) { apply(boost::lambda::_1 = val); }
			T operator()() { return get(); }
			void operator()(const T &val) { set(val); }
			
			void lock() { m.lock(); }
			bool try_lock() { return m.try_lock(); }
			void unlock() { m.unlock(); }
	};

	/**
	This class represents a standalone variable condition
	quite limited but can be useful.
	It is a VariableMutex that you can wait and notify for change
	*/
	template<typename T>
	class VariableCondition: public VariableMutex<T>
	{
		protected:
			boost::condition_variable c;

		public:
			VariableCondition(const T &val_init): VariableMutex<T>(val_init) {}
			
			/**
			You can use boost::lambda to easily create the predicate:
				(#include <boost/lambda/lambda.hpp>)
				vc.wait(boost::lambda::_1 != 3);
			or STL bind1st and bind2nd and comparison operators:
				#include <functional>
			  vc.wait(std::bind1st(std::not_equal_to<int>(),3));
			*/
			template<typename Pred>
			void wait(Pred pred, bool unlock = true)
			{
				boost::unique_lock<boost::mutex> l(VariableMutex<T>::m);
				while(!pred(VariableMutex<T>::var)) c.wait(l);
				if (!unlock) l.release();
			}
			void notify() { c.notify_all(); }
			template<typename Assign> void applyAndNotify(Assign assign) { 
				VariableMutex<T>::apply(assign); notify(); 
			}
			void setAndNotify(const T &val) { applyAndNotify(boost::lambda::_1 = val); }
	};

}}

#endif

