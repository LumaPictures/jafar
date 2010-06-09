/* $Id$ */

#ifndef KERNEL_EXCEPTION_HPP
#define KERNEL_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace kernel {

    /** Base class for all exceptions defined in the
     * module kernel.
     *
     * @ingroup kernel
     */
    class KernelException : public jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * kernel.
       */
      enum ExceptionId {
	KEYVALUEFILE_UNKNOWN_KEY, /**< KeyValueFile: unknown key           */
	KEYVALUEFILE_INVALID_LINE, /**< KeyValueFile: invalide line in file */
        CSVFILE_INVALID_LINE, /**< CSVFile: invalide line in file */
	CSVFILE_UNKNOWN_COLUMN_NAME, /**< CSVFile: unknown column name */
	CSVFILE_UNKNOWN_LINE, /**< CSVFile: unknown line in file */
        CSVFILE_UNKNOWN_COLUMN, /**< CSVFile: unknown column in file */
        THREAD_ERROR /**< Thread error */
      };

      /** Constructor. You should not use this constructor directly,
       * prefer macros jfrThrowEx or jfrCreateEx which fill for you
       * parameters \c file_ and \c line_.
       *
       * @param id_ exception id
       * @param message_ message used for debug
       * @param file_ where the exception was thown
       * @param line_ where the exception was thown
       */
      KernelException(ExceptionId id_, 
                      const std::string& message_, 
                      const std::string& file_, int line_) throw();

      virtual ~KernelException() throw();

      ExceptionId getExceptionId() const throw();

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();      

    };

  }
}

#endif // KERNEL_EXCEPTION_HPP

