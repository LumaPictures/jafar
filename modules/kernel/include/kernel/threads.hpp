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
			void set(const T &val) { boost::unique_lock<boost::mutex> l(m); var = val; }
			T operator()() { return get(); }
			void operator()(const T &val) { set(val); }
			
			void lock() { m.lock(); }
			bool try_lock() { return m.try_lock(); }
			void unlock() { m.unlock(); }
	};

	/**
	This class represents a standalone variable condition
	quite limited but can be useful
	*/
	template<typename T>
	class VariableCondition: public VariableMutex<T>
	{
		protected:
			boost::condition_variable c;

		public:
			VariableCondition(const T &val_init): VariableMutex<T>(val_init) {}
			
			template<typename Comp>
			void wait(const T &val, Comp comp)
			{
				boost::unique_lock<boost::mutex> l(VariableMutex<T>::m);
				while(!(comp(VariableMutex<T>::var, val))) c.wait(l);
			}
			void notify() { c.notify_all(); }
			void setNotify(const T &val) { set(val); notify(); }
			
			static bool eq(const T &val1, const T &val2) { return (val1 == val2); }
			static bool ne(const T &val1, const T &val2) { return (val1 != val2); }
			static bool le(const T &val1, const T &val2) { return (val1 <= val2); }
			static bool lt(const T &val1, const T &val2) { return (val1 < val2); }
			static bool ge(const T &val1, const T &val2) { return (val1 >= val2); }
			static bool gt(const T &val1, const T &val2) { return (val1 > val2); }
			
	};

}}

#endif

