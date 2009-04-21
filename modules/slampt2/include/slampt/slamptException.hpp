/* $Id$ */

#ifndef SLAMPT_SLAMPT_EXCEPTION_HPP
#define SLAMPT_SLAMPT_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace slampt {

    /** Base class for all exceptions defined in the module
     * slampt.
     *
     * @ingroup slampt
     */
    class SlamptException : public ::jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * slampt.
       */
      enum ExceptionId {
        //        MY_ERROR /**< my error */
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
      SlamptException(ExceptionId id_, 
                            const std::string& message_, 
                            const std::string& file_, int line_) throw();

      virtual ~SlamptException() throw();

      ExceptionId getExceptionId() const throw(); 

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();
      
    }; // class SlamptException

  } // namespace slampt
} // namespace jafar

#endif // SLAMPT_SLAMPT_EXCEPTION_HPP

