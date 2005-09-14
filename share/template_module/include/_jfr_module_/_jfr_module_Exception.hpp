/* $Id$ */

#ifndef _jfr_MODULE___jfr_MODULE__EXCEPTION_HPP
#define _jfr_MODULE___jfr_MODULE__EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace _jfr_module_ {

    /** Base class for all exceptions defined in the module
     * _jfr_module_.
     *
     * @ingroup _jfr_module_
     */
    class _jfr_Module_Exception : public ::jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * _jfr_module_.
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
      _jfr_Module_Exception(ExceptionId id_, 
                            const std::string& message_, 
                            const std::string& file_, int line_) throw();

      virtual ~_jfr_Module_Exception() throw();

      ExceptionId getExceptionId() const throw(); 

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();
      
    }; // class _jfr_Module_Exception

  } // namespace _jfr_module_
} // namespace jafar

#endif // _jfr_MODULE___jfr_MODULE__EXCEPTION_HPP
