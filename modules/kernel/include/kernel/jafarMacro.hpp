/* $Id$ */

/** \file jafarMacro.hpp Defines usefull macros. All these macro are
 * disabled when -DJFR_NDEBUG is passed to the compiler.
 *
 * \ingroup kernel
 */

#ifndef KERNEL_JAFAR_MACRO
#define KERNEL_JAFAR_MACRO

#include <string>
#include <sstream>

/**
 * The JAFAR_DEPRECATED macro can be used to trigger compile-time warnings when a deprecated
 * functions are used.
 * 
 * For non-inline functions, the macro has to be inserted at the end of the declaration like in :
 * @code
 * DeprecatedConstructor() JAFAR_DEPRECATED;
 * void deprecatedFunction() const JAFAR_DEPRECATED;
 * @endcode
 * 
 * For inline functions, the macro has to be inserted before the declartion but after virtual or
 * static keywoard, like in :
 * @code
 * JAFAR_DEPRECATED void deprecatedInline() { ... }
 * virtual JAFAR_DEPRECATED int depreactedInline() { ... }
 * static JAFAR_DEPRECATED char* deprecatedInline() { ... }
 * @endcode
 * 
 * You can declare a class or struct to be deprecated :
 * @code
 * class JAFAR_DEPRECATED deprecatedClass { };
 * struct JAFAR_DEPRECATED deprecatedStruct { };
 * @endcode
 */
#ifndef JAFAR_DEPRECATED
#if __GNUC__ - 0 > 3 || (__GNUC__ - 0 == 3 && __GNUC_MINOR__ - 0 >= 2)
  /* gcc >= 3.2 */
# define JAFAR_DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER) && (_MSC_VER >= 1300)
  /* msvc >= 7 */
# define JAFAR_DEPRECATED __declspec(deprecated)
#else
# define JAFAR_DEPRECATED
#endif
#endif

// Must be include after the declaration of the JAFAR_DEPRECATED
#include "kernel/jafarException.hpp"

/** Throw \a ExceptionName with ID \a id along with \a message. The
 * constructor of the class \a ExceptionName must have signature
 * compatible with ExceptionName(id, message, __FILE__,
 * __LINE__). This is the case for the exception class which is
 * generated with a module, but may not be the case for user defined
 * exceptions.
 */
#define JFR_ERROR(ExceptionName, id, message)			\
  {								\
    std::ostringstream s;					\
    s << message;						\
    throw ExceptionName(id, s.str(), __FILE__, __LINE__);	\
  }

/** If the \a predicate is \c FALSE, throw \a ExceptionName with ID \a
 * id along with \a message. The constructor of the class \a
 * ExceptionName must have signature compatible with ExceptionName(id,
 * message, __FILE__, __LINE__). This is the case for the exception
 * class which is generated with a module, but may not be the case for
 * user defined exceptions.
 */
#define JFR_PRED_ERROR(predicate, ExceptionName, id, message)		\
  if (!(predicate)) {							\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw ExceptionName(id, s.str(), __FILE__, __LINE__);		\
  }

/** Throw a \c JafarException with id RUN_TIME.
 */
#define JFR_RUN_TIME(message)						\
  {									\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    throw JafarException(JafarException::RUN_TIME,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }

/** If \a predicate is \c FALSE, throw a \c JafarException with id
 *  RUN_TIME.
 */
#define JFR_PRED_RUN_TIME(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::RUN_TIME,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }

/** If \a predicate is \c FALSE, throw a IO_STREAM
 * jafar::kernel::JafarException.  
 * example: 
 * \code 
 *    void T3D::read(const std::string& fileName_) { 
 *      std::ifstream file(fileName_.c_str()); 
 *      JFR_IO_STREAM(file, "error while opening file " << fileName_); 
 *
 *      JFR_TRACE_BEGIN; 
 *      file >> (*this); 
 *      JFR_TRACE_END(" (reading file:" << fileName_ << ")"); 
 *
 *      file.close(); 
 *    } 
 * \endcode
 */
#define JFR_IO_STREAM(predicate, message)				\
  if (!(predicate))							\
    {									\
      using jafar::kernel::JafarException;				\
      std::ostringstream s;						\
      s << message;							\
      throw JafarException(JafarException::IO_STREAM,			\
			   s.str(), _JFR_MODULE_, __FILE__, __LINE__);	\
    }

/** If \a predicate is \c FALSE, throw a NUMERIC
 * jafar::kernel::JafarException.  
 * example: 
 * \code 
 *  JFR_NUMERIC(fabs(v(0)) > jmath::constant::EPSILON,
 *	  "CameraPinhole::project: 3D points x is too small: " << v);
 * \endcode
 */
#define JFR_NUMERIC(predicate, message)					\
  if (!(predicate))							\
    {									\
      using jafar::kernel::JafarException;				\
      std::ostringstream s;						\
      s << message;							\
      s << " (" << #predicate << ")";					\
      throw JafarException(JafarException::NUMERIC,			\
			   s.str(),					\
			   _JFR_MODULE_, __FILE__, __LINE__);		\
    }

/** If \a predicate is \c FALSE throw a jafar::kernel::InvalidParamException
 */
#  define JFR_CHECK_PARAM(predicate, param)				\
  if (!(predicate)) {							\
    std::ostringstream s;						\
    s << "invalid parameter " << #param << "=" << param;		\
    s << " (" << #predicate << ")";					\
    jafar::kernel::throwInvalidParamException(param, s.str(),		\
					      _JFR_MODULE_,		\
					      __FILE__, __LINE__);	\
  }


/** When JFR_NDEBUG is defined, contract programming checks, debug
 * messages and trace information are disabled.
 */
#ifndef JFR_NDEBUG

#include "kernel/jafarDebug.hpp"
using jafar::debug::DebugStream;

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c ASSERT, along with \a message.
 */
#  define JFR_ASSERT(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::ASSERT,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }


/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c PRECONDITION, along with \a message.
 */
#  define JFR_PRECOND(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::PRECONDITION,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }
  

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c POSTCONDITION, along with \a message.
 */
#  define JFR_POSTCOND(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::POSTCONDITION,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c INVARIANT, along with \a message.
 */
#  define JFR_INVARIANT(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::INVARIANT,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  }

/** Send \a message to the debug stream with level
 * DebugStream::Warning. \c operator<< can be used to format the
 * message.
 *
 * \code
 *  if (d < EPSILON)
 *    JFR_WARING("Small value, check numerical stability - d=" << d); 
 *
 * output:
 *  W:pipo/test.cpp:55: Small value, check numerical stability - d= 1e-12 
 * \endcode
 */
#define JFR_WARNING(message)						\
    {									\
      DebugStream::setup(_JFR_MODULE_, DebugStream::Warning);		\
      DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
      DebugStream::instance() << message				\
			      << jafar::debug::endl;			\
    }

/** Send \a message to the debug stream with level
 * DebugStream::Debug. \c operator<< can be used to format the
 * message.
 *
 * \code
 *  JFR_DEBUG("The value of i is " << i); 
 *
 * output:
 *  D:pipo/test.cpp:55: The value of i is 2
 * \endcode
 */
#define JFR_DEBUG(message)						\
  {									\
    DebugStream::setup(_JFR_MODULE_, DebugStream::Debug);		\
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
    DebugStream::instance() << message					\
			    << jafar::debug::endl;			\
  }


/** Send \a message to the debug stream with level
 * DebugStream::VerboseDebug. \c operator<< can be used to format the
 * message.
 */
#define JFR_VDEBUG(message)						\
  {									\
    DebugStream::setup(_JFR_MODULE_, DebugStream::VerboseDebug);	\
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
    DebugStream::instance() << message					\
			    << jafar::debug::endl;			\
  }

/** Send \a message to the debug stream with level
 * DebugStream::VeryVerboseDebug. \c operator<< can be used to format the
 * message.
 */
#define JFR_VVDEBUG(message)						\
  {									\
    DebugStream::setup(_JFR_MODULE_, DebugStream::VeryVerboseDebug);	\
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
    DebugStream::instance() << message					\
			    << jafar::debug::endl;			\
  }

/** This macro add a trace with \a message to \a exception. \a
 * exception must be a jafar::kernel::Exception (all jafar exceptions
 * normally are !).
 * example: 
 * \code 
 *    void T3D::read(const std::string& filename_) { 
 *      std::ifstream file(filename_.c_str()); 
 *      JFR_IO_STREAM(file, "error while opening file " + filename_); 
 *      try { 
 *        file >> (*this); 
 *      }
 *      catch(jafar::kernel::Exception& e) { 
 *        JFR_TRACE(e," (reading file:" + filename_ + ")"); 
 *        throw; 
 *      } 
 *      file.close(); 
 *    } 
 * \endcode
 */
#  define JFR_TRACE(exception, message)					\
  {									\
    std::ostringstream s;						\
    s << message;							\
    exception.addTrace(_JFR_MODULE_, __FILE__, __LINE__, s.str());	\
  }

/** Begin a trace block. A trace block helps the developer to known
 * the call stack followed up by any exception thrown in the
 * block. This block must end with a JFR_TRACE_END.
 */
#  define JFR_TRACE_BEGIN                       \
  try { ((void)0)

/** End a trace block. Append \a message to the trace.
 */
#  define JFR_TRACE_END(message)					\
  }									\
  catch(jafar::kernel::Exception& e) {                                  \
    std::ostringstream m;						\
    m << message;							\
    e.addTrace(_JFR_MODULE_, __FILE__, __LINE__, m.str());		\
    throw;                                                              \
  }                                                                     \
  catch(std::exception& e) {                                            \
    DebugStream::setup(_JFR_MODULE_, DebugStream::Trace);		\
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
    DebugStream::instance() << "std::exception" << jafar::debug::endl	\
			    << message << jafar::debug::endl;		\
    throw;                                                              \
  }                                                                     \
  catch(...) {                                                          \
    DebugStream::setup(_JFR_MODULE_, DebugStream::Trace);		\
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__);	\
    DebugStream::instance() << "unknown exception"			\
			    << jafar::debug::endl			\
			    << message << jafar::debug::endl;		\
    throw;                                                              \
  } ((void)0)

/** shortcut for: 
 * \code
 *   JFR_TRACE_END(message); 
 *   JFR_TRACE.
 * \endcode
 */
#    define JFR_TRACE_POINT(message)					\
  JFR_TRACE_END(message);						\
  JFR_TRACE_BEGIN

#else // JFR_NDEBUG
#  define JFR_PRECOND(predicate, message) ((void)0)
#  define JFR_POSTCOND(predicate, message) ((void)0)
#  define JFR_INVARIANT(predicate, message) ((void)0)

#  ifndef JFR_QUIET
#    define JFR_WARNING(message)					\
  {									\
    std::cerr << "W: " << _JFR_MODULE_ << "/"				\
	      << __FILE__ << ":" << __LINE__ << ": "			\
	      << message << std::endl;					\
  }
#  else // JFR_QUIET
#    define JFR_WARNING(message) ((void)0)
#  endif // JFR_QUIET

#  define JFR_DEBUG(message) ((void)0)
#  define JFR_VDEBUG(message) ((void)0)
#  define JFR_VVDEBUG(message) ((void)0)
#  define JFR_TRACE ((void)0)
#  define JFR_TRACE_BEGIN ((void)0)
#  define JFR_TRACE_POINT ((void)0)
#  define JFR_TRACE_END(message) ((void)0)
#endif // JFR_NDEBUG

#endif // KERNEL_JAFAR_MACRO
