
#include "kernel/threads.hpp"
#include "kernel/kernelException.hpp"
#include <sys/resource.h>
//#define DEBUG

namespace jafar {
namespace kernel {

#ifdef DEBUG
	int get_count(boost::semaphore *sem)
	{
		int res = 0;
		while (sem->try_wait()) res++;
		for(int i = 0; i < res; i++) sem->post();
		return res;
	}
#endif
	
	bool FifoMutex::lock_(bool try_)
	{
		boost::unique_lock<boost::mutex> l(m);
		bool own = false;
		// get this thread's sem
		boost::thread::id id = boost::this_thread::get_id();
		SemsMap::iterator itsem = allSems.find(id);
		if (itsem == allSems.end())
		{
#ifdef DEBUG
std::cout << "FifoMutex::lock(" << id << ") : creating semaphore" << std::endl;
#endif
			itsem = allSems.insert(SemThread(id, SemWait(new boost::semaphore(0), false))).first;
		}
		boost::semaphore &sem = *(itsem->second.first);
		// process
		if (!computingSem) // no one have the control, take it
		{
#ifdef DEBUG
std::cout << "FifoMutex::lock(" << id << " " << &sem << " " << get_count(&sem) << ") : no one has the control, take it" << std::endl;
#endif
			itsem->second.second = false;
			computingSem = itsem;
			own = true;
		} else
		if (itsem->second.second == true) // already waiting, means try_lock
		{
			if (try_) own = sem.try_wait();
				else JFR_ERROR(KernelException, KernelException::THREAD_ERROR, "already waiting, should be blocked!");
		} else 
		if (*computingSem == itsem) // already have the control, do nothing
		{
#ifdef DEBUG
std::cout << "FifoMutex::lock(" << id << " " << &sem << " " << get_count(&sem) << ") : already have the control, do nothing" << std::endl;
#endif
			own = true;
		} else // else, get in the queue
		{
#ifdef DEBUG
std::cout << "FifoMutex::lock(" << id << " " << &sem << " " << get_count(&sem) << ") : else, get in the queue" << std::endl;
#endif
			itsem->second.second = true;
			waitingSems.push_back(itsem);
			if (try_) own = sem.try_wait(); 
			else 
			{
				/* hum, unlock and wait not atomic...
				The only thing that can happen, is that this is the only sem in waiting list,
				and between unlock and wait the computing thread releases and post on this
				sem before/at the same times it waits.
				It's ok to post before to wait, and sems are protected by a mutex,
				so everything should be ok. */
				l.unlock();
				sem.wait();
				own = true;
			}
		}
#ifdef DEBUG
if (own) std::cout << "FifoMutex::lock(" << id << " " << &sem << " " << get_count(&sem) << ") : own" << std::endl;
#endif
		if (own) (*computingSem)->second.second = false;
		return own;
	}
	


	void FifoMutex::unlock()
	{
		boost::unique_lock<boost::mutex> l(m);
#ifdef DEBUG
if (!own_lock())
std::cout << "FifoMutex::unlock(" << boost::this_thread::get_id() << ") : do not own lock" << std::endl;
#endif
		if (!own_lock()) return; // nothing to do
		// process
		if (waitingSems.empty())
		{
#ifdef DEBUG
std::cout << "FifoMutex::unlock(" << boost::this_thread::get_id() << ") : no one waiting" << std::endl;
#endif
			computingSem = boost::optional<SemsMap::iterator>();
		} else
		{
			computingSem = waitingSems.front();
			waitingSems.pop_front();
#ifdef DEBUG
std::cout << "FifoMutex::unlock(" << boost::this_thread::get_id() << ") : posting " << (*computingSem)->second.first << std::endl;
#endif
			(*computingSem)->second.first->post();
		}
	}


	void printThreadsPrioInfo(int r, int policy, struct sched_param &param, bool data)
	{
		std::cout << "error ";
		switch (r)
		{
			case 0: std::cout << "OK"; break;
			case EINVAL: std::cout << "EINVAL"; break;
			case ENOTSUP: std::cout << "ENOTSUP"; break;
			case EPERM: std::cout << "EPERM"; break;
			case ESRCH: std::cout << "ESRCH"; break;
			default: std::cout << "UNKNOWN/" << r; break;
		}
		if (data)
		{
			std::cout << ", policy ";
			switch(policy)
			{
				case SCHED_FIFO: std::cout << "SCHED_FIFO"; break;
				case SCHED_RR: std::cout << "SCHED_RR"; break;
				case SCHED_OTHER: std::cout << "SCHED_OTHER"; break;
				default: std::cout << "UNKNOWN"; break;
			}
			std::cout << ", priority " << param.sched_priority;
		}
		std::cout << std::endl;
	}
	
	int setThreadPriority(boost::thread &t, int prio)
	{
		int r;
		
		#if 0
		// all of this isn't working
		
		int policy;
		struct sched_param param;
		r = pthread_getschedparam(t.native_handle(), &policy, &param);
		printThreadsPrioInfo(r, policy, param, true);
		
		//policy = SCHED_FIFO;
		param.sched_priority = prio;
		r = pthread_setschedparam(t.native_handle(), policy, &param);
		printThreadsPrioInfo(r, policy, param, false);
		r = pthread_getschedparam(t.native_handle(), &policy, &param);
		printThreadsPrioInfo(r, policy, param, true);
		
		r = pthread_setschedprio(t.native_handle(), prio);
		printThreadsPrioInfo(r, policy, param, false);
		r = pthread_getschedparam(t.native_handle(), &policy, &param);
		printThreadsPrioInfo(r, policy, param, true);
		
		#endif
		
		/*
		TODO get pid of thread
		seems impossible to do in a portable way (need to look inside pthread_t struct)
		*/
		int pid = 0;
		r = setpriority(PRIO_PROCESS, pid, prio);
		return r;
	}
	
	void setCurrentThreadPriority(int priority)
	{
		int r = setpriority(PRIO_PROCESS, 0, priority);
		if (r) perror("Error setCurrentThreadPriority");
	}

	void setCurrentThreadScheduler(int policy, int priority)
	{
		struct sched_param param;
		param.sched_priority = priority;
		int r = pthread_setschedparam(pthread_self(), policy, &param);
		if (r) perror("Error setCurrentThreadScheduler");
	}

	void setProcessScheduler(int policy, int priority)
	{
		struct sched_param param;
		param.sched_priority = priority;
		int r = sched_setscheduler(0/*getpid()*/, policy, &param);
		if (r) perror("Error setProcessScheduler");
	}


}}


