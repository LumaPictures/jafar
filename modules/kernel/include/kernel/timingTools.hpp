/* $Id$ */

#ifndef KERNEL_TIMINGTOOLS_HPP
#define KERNEL_TIMINGTOOLS_HPP

#include "boost/date_time/posix_time/posix_time.hpp"

namespace jafar {
  namespace kernel {

    /** This class provides tic() and toc() functions (like in
     * matlab). You use them to measure time consumed by your
     * algorithm. Also ticToc.tcl defines direct macros.
     *
     * \ingroup kernel
     */
    class TicToc {

    private :

      static boost::posix_time::ptime refTime;

    public :

      /// reset the reference time
      static void tic();

      /// return the time elapsed since the last tic (in millisecond)
      static long toc();

      /// return the time elapsed since the last tic (in microsecond)
      static long toc_micro();

    };
    
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
