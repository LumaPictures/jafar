/* $Id$ */

#ifndef SLAM_SLAM_EXCEPTION_HPP
#define SLAM_SLAM_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace slam {

    /** Base class for all exceptions defined in the module
     * slam.
     *
     * @ingroup slam
     */
    class SlamException : public jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * Slam.
       */
      enum ExceptionId {
        UNKNOWN_FEATURE,          /**< Unknown feature          */
	UNKNOWN_POSE_COPY,        /**< Unknown pose copy        */
	LOCAL_MAP_FULL,           /**< Local map is full        */
	UNKNOWN_SENSOR,           /**< Unknown sensor           */
	INVALID_INITSTATE_UPDATE  /**< Invalid boFeature initstate update */
      };

      /** Constructor. You should not use this constructor directly,
       * prefer macros jfrThrowEx or jfrCreateEx which fill for you
       * parameters \c file_ and \c line_.
       *
       * @param id_ exception id
       * @param message_ message used for debug
       * @param file_ where the exception was thrown
       * @param line_ where the exception was thrown
       */
      SlamException(ExceptionId id_, 
		    const std::string& message_, 
		    const std::string& file_, int line_) throw();

      virtual ~SlamException() throw();

      ExceptionId getExceptionId() const throw(); 

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();
      
    }; // class SlamException

  } // namespace slam
} // namespace jafar

#endif // SLAM_SLAM_EXCEPTION_HPP

