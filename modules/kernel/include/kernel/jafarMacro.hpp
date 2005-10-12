/* $Id$ */

/** \file jafarMacro.hpp Defines usefull macros. All these macro are
 * disabled when -DJFR_NDEBUG is passed to the compiler.
 *
 * \ingroup kernel
 */

#ifndef KERNEL_JAFAR_MACRO
#define KERNEL_JAFAR_MACRO

#ifndef JFR_NDEBUG

#include <string>
#include <iostream>
#include <sstream>

#include "kernel/jafarDebug.hpp"
#include "kernel/jafarException.hpp"

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
			 s.str(),					\
			 _JFR_MODULE_, __FILE__, __LINE__);		\
  }									\
  

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
			 s.str(),					\
			 _JFR_MODULE_, __FILE__, __LINE__);		\
  }									\

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
			 s.str(),					\
			 _JFR_MODULE_, __FILE__, __LINE__);		\
  }

/** Throw \a ExceptionName with ID \a id along with \a message. The
 * constructor of the class \a ExceptionName must have signature
 * compatible with ExceptionName(id, message, __FILE__,
 * __LINE__). This is the case for the exception class which is
 * generated with a module, but may not be the case for user defined
 * exceptions.
 */
#  define JFR_ERROR(ExceptionName, id, message)			\
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
#  define JFR_PRED_ERROR(predicate, ExceptionName, id, message)         \
  if (!(predicate)) {							\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw ExceptionName(id, s.str(), __FILE__, __LINE__);		\
  }

/** Throw a \c JafarException with id RUN_TIME.
 */
#  define JFR_RUN_TIME(message)                                         \
  {									\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    throw JafarException(JafarException::RUN_TIME,			\
			 s.str(),					\
			 _JFR_MODULE_, __FILE__, __LINE__);		\
  }

/** If \a predicate is \c FALSE, throw a \c JafarException with id
 *  RUN_TIME.
 */
#  define JFR_PRED_RUN_TIME(predicate, message)				\
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::RUN_TIME,			\
			 s.str(),					\
			 _JFR_MODULE_, __FILE__, __LINE__);		\
  }

/** If \a predicate is \c FALSE, throw a IO_STREAM
 * jafar::kernel::JafarException.  
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
#define JFR_IO_STREAM(predicate, message)				\
  if (!(predicate))							\
    {									\
      using jafar::kernel::JafarException;				\
      std::ostringstream s;						\
      s << message;							\
      throw JafarException(JafarException::IO_STREAM,			\
			   s.str(),					\
			   _JFR_MODULE_, __FILE__, __LINE__);		\
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

/** Send \a message to the debug stream. \c operator<< can be used to
 * format the message.
 *
 * \code
 *  JFR_DEBUG("The value of i is " << i); 
 *
 * output:
 *  D: jafarMacro.hpp:55: The value of i is 2
 * \endcode
 */
#define JFR_DEBUG(message)						\
  {									\
    (*jafar::kernel::JafarDebug::dbg)					\
      << "D:" << _JFR_MODULE_ << "/"					\
      << basename(__FILE__) << ":" << __LINE__ << ": "			\
      << message							\
      << std::endl							\
      << std::flush;							\
  }

/** If \a predicate is \c FALSE, send warning \a message to the
 * debug. \c operator<< can be used to format the message.
 *
 * \code 
 *   JFR_WARNING(x*x+y*y > 1e-3, "(x,y) vector is too small"); 
 * \endcode
 *
 */
#define JFR_WARNING(predicate, message)					\
  if (!(predicate))							\
    {									\
      (*jafar::kernel::JafarDebug::dbg)					\
	<< "W: " << _JFR_MODULE_ << "/"					\
	<< basename(__FILE__) << ":" << __LINE__ << ": "		\
	<< message << " (" << #predicate << ")"				\
	<< std::endl							\
	<< std::flush;							\
    }

/** This macro add a trace with \a message to \a exception. \a
 * exception must be a jafar::kernel::Exception (all jafar exceptions
 * normally are !).
 * example: 
 * \code 
 *    void T3D::read(const std::string& fileName_) { 
 *      std::ifstream file(fileName_.c_str()); 
 *      JFR_IO_STREAM(file, "error while opening file " << fileName_); 
 *      try { 
 *        file >> (*this); 
 *      }
 *      catch(jafar::kernel::Exception& e) { 
 *        JFR_TRACE(e," (reading file:" << fileName_ << ")"); 
 *        throw; 
 *      } 
 *      file.close(); 
 *    } 
 * \endcode
 */
#  define JFR_TRACE(exception, message)			\
  {							\
    std::ostringstream s;				\
    s << message;					\
    exception.addTrace(__FILE__, __LINE__, s.str());	\
  }

/** Begin a trace block. A trace block helps the developer to known
 * the call stack followed up by any exception thrown in the
 * block. This block must end with a JFR_TRACE_END.
 */
#  define JFR_TRACE_BEGIN                       \
  try { ((void)0)

/** shortcut for: 
 * \code
 *   JFR_TRACE_END 
 *   JFR_TRACE.
 * \endcode
 */
#  define JFR_TRACE_POINT                                               \
  }						                        \
  catch(jafar::kernel::Exception& e) {                                  \
    e.addTrace(__FILE__,__LINE__);                                      \
    throw;                                                              \
  }                                                                     \
  catch(std::exception& e) {                                            \
    (*jafar::kernel::JafarDebug::dbg)                                   \
      << "E: " << __FILE__ << ":" << __LINE__                           \
      << ": std::exception" << std::endl;                               \
    throw;                                                              \
  }                                                                     \
  catch(...) {                                                          \
    (*jafar::kernel::JafarDebug::dbg)                                   \
      << "E: " << __FILE__ << ":" << __LINE__                           \
      << ": ..." << std::endl;                                          \
    throw;                                                              \
  }                                                                     \
  try { ((void)0)

/** End a trace block.
 */
#  define JFR_TRACE_END							\
  }									\
  catch(jafar::kernel::Exception& e) {                                  \
    e.addTrace(__FILE__, __LINE__);                                     \
    throw;                                                              \
  }                                                                     \
  catch(std::exception& e) {                                            \
    (*jafar::kernel::JafarDebug::dbg)                                   \
      << "E: " << __FILE__ << ":" << __LINE__                           \
      << ": std::exception" << std::endl;                               \
    throw;                                                              \
  }                                                                     \
  catch(...) {                                                          \
    (*jafar::kernel::JafarDebug::dbg)                                   \
      << "E: " << __FILE__ << ":" << __LINE__                           \
      << ": ..." << std::endl;                                          \
    throw;                                                              \
  } ((void)0)

#else // JFR_NDEBUG
#  define JFR_PRECOND(predicate, message) ((void)0)
#  define JFR_POSTCOND(predicate, message) ((void)0)
#  define JFR_INVARIANT(predicate, message) ((void)0)
#  define JFR_ERROR(ExceptionName, id, message) ((void)0)
#  define JFR_PRED_ERROR(ExceptionName, id, message) ((void)0)
#  define JFR_RUN_TIME(message) ((void)0)
#  define JFR_PRED_RUN_TIME(message) ((void)0)
#  define JFR_IO_STREAM(predicate, message) ((void)0)
#  define JFR_NUMERIC(predicate, message) ((void)0)
#  define JFR_DEBUG(message) ((void)0)
#  define JFR_WARNING(message) ((void)0)
#  define JFR_TRACE ((void)0)
#  define JFR_TRACE_BEGIN ((void)0)
#  define JFR_TRACE_POINT ((void)0)
#  define JFR_TRACE_END ((void)0)
#endif // JFR_NDEBUG

#endif // KERNEL_JAFAR_MACRO
