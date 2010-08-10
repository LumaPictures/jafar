/* $Id$ */

#ifndef KERNEL_TIMINGTOOLS_HPP
#define KERNEL_TIMINGTOOLS_HPP

#include "boost/date_time/posix_time/posix_time.hpp"
// we could use a better singleton here...
#include "boost/pool/detail/singleton.hpp"
#include <unistd.h>
#include <sys/time.h>

#include "kernel/jafarDebug.hpp"


namespace jafar {
  namespace kernel {

		class Timestamp
		{
			private:
				/*long*/ double data;
			public:
				Timestamp(): data(-1.0) {}
				Timestamp(double param): data(param) {}
				//Timestamp(long double param): data(param) {}
				operator double() const { return data; }
				//operator long double() const { return data; }
				friend std::ostream& operator<<(std::ostream& os, const jafar::kernel::Timestamp &t);
		};
		
		std::ostream& operator<<(std::ostream& os, const Timestamp &t);
		
		
    /** This class functions to measure time consumed by your
     * algorithm.
     * 
     * \ingroup kernel
     */
    class Chrono {

    private :

      boost::posix_time::ptime refTime;

    public :

      /// Constructor calls reset().
      Chrono() {
	reset();
      }

      /// reset the reference time.
      void reset() {
	refTime = boost::posix_time::microsec_clock::local_time();
      }

      /// return the time elapsed since the last reset (in milliseconds)
      long elapsed() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_milliseconds();
      }

      /// return the time elapsed since the last reset (in seconds)
      long elapsedSecond() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_seconds();
      }

      /// return the time elapsed since the last reset (in microseconds)
      long elapsedMicrosecond() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_microseconds();
      }
      

    }; // class Chrono


		/// This function returns an absolute time
		class Clock
		{
			public:
			static double getTime()
			{
				// FIXME should use boost::posix_time
				//boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
				//return curTime.total_seconds() + curTime.fractional_seconds() / time_duration::ticks_per_second();
				struct timeval tv; struct timezone tz;
				gettimeofday(&tv, &tz);
				return tv.tv_sec + tv.tv_usec*1e-6;
			}
			
		};

		class Timer
		{
			private:
				double period;
				double last_tic_time;
			public:
				Timer(int period_us): period(period_us*1e-6) { restart(); }
				void restart()  { last_tic_time = Clock::getTime(); }
				void wait()
				{
					// FIXME should use nanosleep
					double current_time = Clock::getTime();
					last_tic_time += period;
					double wait_time = last_tic_time - current_time;
					if (wait_time < 0)
						last_tic_time = current_time;
					else
						usleep(wait_time*1e6);
				}
				
		};

#ifndef SWIG

    void tic();
    long toc();

    namespace detail {

      /** Global chronos for tic() and toc() interactive functions.
       *
       * \ingroup kernel
       */
      class TicTocChrono {

      private:
	
	TicTocChrono() {};

	Chrono chrono;

	static TicTocChrono& instance() {
	  return boost::details::pool::singleton_default<TicTocChrono>::instance(); 
	}

	// necessary because the constructor is private
	friend class boost::details::pool::singleton_default<TicTocChrono>;

	friend void jafar::kernel::tic();
	friend long jafar::kernel::toc();

      };
    } // namespace detail

#endif // SWIG
    
    /** Class to generate framerate data.
     *
     * @ingroup kernel
     */
    class FrameRate {
      
    public:

      enum TypeUpdate{
          ontime,
          onframe
      };

      /** Constructor
       * @param upd_ update type
       * @param updTimeInt_ time interval for time based update in milliseconds
       * @param updFrameInt_ frames interval for time based update 
       */
      FrameRate(TypeUpdate upd_ = ontime, int updTimeInt_ = 2000, int updFrameInt_ = 10);
      
      /** Update fps. to be called after each frame computation
       */
      void updateFps();

      /** get last fps
       */
      float getFps();

    private:
      TypeUpdate upd;
      boost::posix_time::ptime lastUpdate;
      boost::posix_time::time_duration fpsUpdateInterval;
      int numFramesInterval;
      int  numFrames;
      float fps;
    }; // class FrameRate

  } // namespace kernel
} // namespace jafar


#endif // KERNEL_TIMINGTOOLS_HPP
