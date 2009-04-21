/* $Id$ */

#ifndef SLAMLINES_SLAMLINES_EXCEPTION_HPP
#define SLAMLINES_SLAMLINES_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace slamlines {

    /** Base class for all exceptions defined in the module
     * slamlines.
     *
     * @ingroup slamlines
     */
    class SlamlinesException : public ::jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * slamlines.
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
      SlamlinesException(ExceptionId id_, 
                            const std::string& message_, 
                            const std::string& file_, int line_) throw();

      virtual ~SlamlinesException() throw();

      ExceptionId getExceptionId() const throw(); 

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();
      
    }; // class SlamlinesException

  } // namespace slamlines
} // namespace jafar

#endif // SLAMLINES_SLAMLINES_EXCEPTION_HPP

