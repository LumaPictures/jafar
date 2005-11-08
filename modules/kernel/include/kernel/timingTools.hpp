/* $Id$ */

#ifndef KERNEL_TIMINGTOOLS_HPP
#define KERNEL_TIMINGTOOLS_HPP

#include <stack>

#include "boost/date_time/posix_time/posix_time.hpp"
// we could use a better singleton here...
#include "boost/pool/detail/singleton.hpp"

#include "kernel/jafarDebug.hpp"

namespace jafar {
  namespace kernel {

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

      /// return the time elapsed since the last reset (in millisecond)
      long elapsed() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_milliseconds();
      }

      /// return the time elapsed since the last reset (in microsecond)
      long elapsedSecond() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_seconds();
      }

      /// return the time elapsed since the last reset (in microsecond)
      long elapsedMicrosecond() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	return (curTime - refTime).total_microseconds();
      }

    }; // class Chrono


#ifndef SWIG

    void tic();
    long toc();

    namespace detail {

      /** Global chronos for tic() and toc() interective functions.
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

	// necessary because because the constructor is private
	friend class boost::details::pool::singleton_default<TicTocChrono>;

	friend void jafar::kernel::tic();
	friend long jafar::kernel::toc();

      };
    } // namespace detail

#endif // SWIG
    
    /** Class for generate framerate data
     *
     * @ingroup kernel
     */
    class FrameRate {
      
    public:

      typedef enum TypeUpdate{
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
