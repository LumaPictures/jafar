/* $Id$ */

#ifndef KERNEL_JAFAR_EXCEPTION_HPP
#define KERNEL_JAFAR_EXCEPTION_HPP

#include <string>

#include <list>

#include <iostream>

#include "kernel/jafarMacro.hpp"

namespace jafar {

  namespace kernel {

    /** Base class for all exceptions defined in jafar. It cannot be
     * instanciated, instead use exceptions classes defined in each
     * module. Have a look at the module \ref helloworld to see an example.
     *
     * \todo print ExceptionId with an operator<< rather than a toString() method.
     *
     * \ingroup kernel
     */
    class Exception {

    public:

      /** Return the description of the exception. Example:
          \verbatim
          ** Exception from jafar module foo
          Id: fooException::MY_FOO_ERROR
          foo.cpp:25: the given message
          \endverbatim
       */
      const std::string& what() const throw();

      /** Append \a message_ to exception message.
       * @see JFR_TRACE, JFR_TRACE_POINT, JFR_TRACE_END, JFR_TRACE_MESSAGE
       */
      void addTrace(std::string const& module_, std::string const& file_, int line_, std::string const& message_ = "");

    protected:

      /// @deprecated
      Exception(const std::string& message_, 
                const std::string& module_, const std::string& id_, 
                const std::string& file_, int line_) throw() JAFAR_DEPRECATED;

      virtual ~Exception() throw();

      /// message to be displayed
      std::string _what;

      /** calling stack crossed by the exception (use JFR_TRACE,
       * JFR_TRACE_POINT and JFR_TRACE_END macros)
       */
       std::list<std::string> trace;

      friend std::ostream& operator <<(std::ostream& s, const Exception& e);

    }; // end class Exception

    std::ostream& operator <<(std::ostream& s, const Exception& e);

    /** This class defines some standard exceptions to be used from
     * all modules. For ease of use, the macros defined in
     * jafarMacro.hpp. See also \ref pageJafarError.
     *
     * \ingroup kernel
     */
    class JafarException : public jafar::kernel::Exception {

    public:

      /// This enumeration defines exceptions id of JafarException
      enum ExceptionId {
	ASSERT,            /**< assertion not met                                   */
        PRECONDITION,      /**< precondition not met                                */
        POSTCONDITION,     /**< postcondition not met                               */
        INVARIANT,         /**< invariant not respected                             */
        RUN_TIME,          /**< run time error, the programm should not end up here */
        IO_STREAM,         /**< input/output error when using a c++ stream          */ 
	NUMERIC,           /**< numeric error                                       */
	INVALID_PARAM      /**< invalid parameter value                             */
      };

      /** Constructor. You should not use this constructor directly,
       * prefer the macros.
       *
       * @param id_ exception id
       * @param message_ message used for debug
       * @param module_ module from which exception was thrown
       * @param file_ where the exception was thown
       * @param line_ where the exception was thown
       */
      JafarException(ExceptionId id_, const std::string& message_, 
                     const std::string& module_, const std::string& file_, int line_) throw();

      virtual ~JafarException() throw();

      ExceptionId getExceptionId() const throw();

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();      

    };

    /** Special exception class for invalid parameter error. This class
     * contains the value of the parameter.
     *
     * \ingroup kernel
     */
    template<typename T>
    class InvalidParamException : public JafarException {

    public:

      InvalidParamException(const T& value, const std::string& message_,
			    const std::string& module_, const std::string& file_, int line_) : 
	JafarException(INVALID_PARAM, message_, module_, file_, line_),
	m_value(value) {}

      /// Returns the invalid value of the parameter.
      T value() const {return m_value;}

    private:
      
      T m_value;

    };

    /** Actually throws an InvalidParamException. This trick is needed because
     * of the template parameter.
     *
     * \ingroup kernel
     */
    template <typename T>
    void throwInvalidParamException(const T& value, const std::string& message_,
				    const std::string& module_, const std::string& file_, int line_) 
    {
      throw InvalidParamException<T>(value, message_,
				     module_, file_, line_);
    }

  } // namespace kernel
} // namespace jafar

// automatically include macros
#include "kernel/jafarMacro.hpp"

#endif // KERNEL_JAFAR_EXCEPTION_HPP
