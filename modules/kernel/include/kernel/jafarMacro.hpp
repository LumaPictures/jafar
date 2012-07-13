/* $Id$ */

/** \file jafarMacro.hpp Defines usefull macros. All these macro are
 * disabled when -DJFR_NDEBUG is passed to the compiler.
 *
 * \ingroup kernel
 */

#ifndef KERNEL_JAFAR_MACRO
#define KERNEL_JAFAR_MACRO

// Define _JFR_MODULE_ when using a jafar library outside the jafar build system
#ifndef _JFR_MODULE_
#define _JFR_MODULE_ "jafar"
#endif

#include <string>
#include <sstream>

namespace jafar {
  namespace kernel {
    namespace details {

      struct ForeachInfoBase {};
  
      template<typename T>
      struct ForeachInfo : public ForeachInfoBase {
        inline ForeachInfo( const T& t ) : it( t.begin() ), end( t.end() ), count(0)
        {
        }
        inline bool finished() const
        {
          return it == end;
        }
        inline void next() const
        {
          count = 0;
          ++it;
        }
        mutable typename T::const_iterator it, end;
        mutable int count;
      };
      template <typename T>
      inline ForeachInfo<T> CreateForeachInfo(const T& t)
      {
        return ForeachInfo<T>( t );
      }
  
      template <typename T>
      inline const ForeachInfo<T> *getForeachInfo(const ForeachInfoBase *base, const T *)
      {
        return static_cast<const ForeachInfo<T> *>(base); 
      }
      template <typename T>
      inline T* getForeachType(const T &) { return 0; }
    }
  }
}

/**
 * This macro allow to easily loop over all the elements of a vector, without writing the annoying iterators.
 * @code
 * std::vector\< int \> vec;
 * JFR_FOREACH( int var, vec )
 * {
 *  JFR_DEBUG( var );
 * }
 * std::vector\< CoolObject \> coolObjects;
 * JFR_FOREACH( CoolObject& coolObject, coolObjects )
 * {
 *  coolObject.soSomethingCool();
 * }
 * @endcode
 */
#define JFR_FOREACH(var, cont ) \
  for( const jafar::kernel::details::ForeachInfoBase&  contInfo = jafar::kernel::details::CreateForeachInfo( cont ); \
       not jafar::kernel::details::getForeachInfo( &contInfo, true ? 0 : jafar::kernel::details::getForeachType(cont) )->finished(); \
       jafar::kernel::details::getForeachInfo( &contInfo, true ? 0 : jafar::kernel::details::getForeachType(cont) )->next() ) \
    for( var = *jafar::kernel::details::getForeachInfo( &contInfo, true ? 0 : jafar::kernel::details::getForeachType(cont) )->it; \
         jafar::kernel::details::getForeachInfo( &contInfo, true ? 0 : jafar::kernel::details::getForeachType(cont) )->count < 1; \
         ++jafar::kernel::details::getForeachInfo( &contInfo, true ? 0 : jafar::kernel::details::getForeachType(cont) )->count )


/**
 * This macro defines the member used to store the value of a parameter.
 * 
 * There is no reason to use that macro directly, you will want to use
 * JFR_DEFINE_PARAM_RW, JFR_DEFINE_PARAM_RO or JFR_DEFINE_PARAM_WO
 * 
 * @param type the type of the parameter
 * @param name the name of the parameter (the name of the variable is
 *             prefixed with m_)
 */
#define JFR_DEFINE_PARAM_VAR(type, name) \
  private: \
      type m_##name;

/**
 * This macro defines the function used to return the value of a parameter.
 * There is no reason to use that macro directly, you will want to use
 * JFR_DEFINE_PARAM_RW, JFR_DEFINE_PARAM_RO or JFR_DEFINE_PARAM_WO
 * @param type the type of the parameter
 * @param name the name of the parameter
 */
#define JFR_DEFINE_PARAM_GETTER(type, name) \
  public: \
     inline type name() const { return m_##name; }

/**
 * This macro defines the function used to set the value of a parameter.
 * There is no reason to use that macro directly, you will want to use
 * JFR_DEFINE_PARAM_RW, JFR_DEFINE_PARAM_RO or JFR_DEFINE_PARAM_WO
 * @param type the type of the parameter
 * @param name the name of the parameter
 * @param settername the name of the setter (if the parameter is called
 * myParameter, the setter is ususally called setMyParamter)
 */
#define JFR_DEFINE_PARAM_SETTER(type, name, settername ) \
  public: \
     inline void settername(type v) { m_##name = v; }

/**
 * This macro defines a parameter that can be read and write.
 * 
 * Exemple of use:
 * @code
 *  class MyClass {
 *    JFR_DEFINE_PARAM_RW( double, myParam, setMyParam);
 *  };
 *  MyClass instance;
 *  instance.setMyParam(10.0);
 *  JFR_DEBUG(instance.myParam())
 * @endcode
 * 
 * @param type the type of the parameter
 * @param name the name of the parameter
 * @param settername the name of the setter (if the parameter is called
 * myParameter, the setter is ususally called setMyParamter)
 */
#define JFR_DEFINE_PARAM_RW( type, name, settername ) \
  JFR_DEFINE_PARAM_GETTER(type, name) \
  JFR_DEFINE_PARAM_SETTER(type, name, settername ) \
  JFR_DEFINE_PARAM_VAR(type,name)

/**
 * This macro defines a parameter that can be read, but not set.
 * 
 * Exemple of use:
 * @code
 *  class MyClass {
 *    public:
 *      MyClass( ) : m_myParam(10.0) { }
 *    JFR_DEFINE_PARAM_RO( double, myParam)
 *  };
 *  MyClass instance;
 *  JFR_DEBUG(instance.myParam())
 * @endcode
 * 
 * Or if you want a personal setter
 * @code
 *  class MyClass {
 *    JFR_DEFINE_PARAM_RO( double, myParam)
 *    public:
 *      MyClass( ) : m_myParam(0.0) { }
 *    public:
 *      void setMyParam(double p)
 *      {
 *        if(p < 5.0) m_myParam = p;
 *      }
 *  };
 *  MyClass instance;
 *  instance.setMyParam(10.0);
 *  JFR_DEBUG(instance.myParam()); // will return 0.0
 * @endcode
 * 
 * @param type the type of the parameter
 * @param name the name of the parameter
 */
#define JFR_DEFINE_PARAM_RO( type, name) \
  JFR_DEFINE_PARAM_GETTER(type, name) \
  JFR_DEFINE_PARAM_VAR(type,name)

/**
 * This macro defines a parameter that can only be written.
 * 
 * Exemple of use:
 * @code
 *  class MyClass {
 *    JFR_DEFINE_PARAM_WO( double, myParam, setMyParam)
 *  };
 *  MyClass instance;
 *  instance.setMyParam(10.0);
 * @endcode
 * 
 * @param type the type of the parameter
 * @param name the name of the parameter
 * @param settername the name of the setter (if the parameter is called
 * myParameter, the setter is ususally called setMyParamter)
 */
#define JFR_PARAM_WO( type, name, settername ) \
  JFR_DEFINE_PARAM_SETTER(type, name, settername ) \
  JFR_DEFINE_PARAM_VAR(type,name)

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

/**
 * Display a variable name in a flow and its value.
 * 
 * For instance:
 * @code
 * int a = 1;
 * JFR_DEBUG( JFR_PP_VAR( a ) );
 * @endcode
 * will display "a = 1 ".
 */
#define JFR_PP_VAR( var ) #var << " = " << var << " "


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

/**
 * general try/catch block to display the exception message and forward the exception
 * (usually to make the program catch), but disable this in debug mode because it
 * prevents from getting a backtrace in debuggers.
 */
#ifdef JFR_NDEBUG
	#define JFR_GLOBAL_TRY try{
	#define JFR_GLOBAL_CATCH } catch (kernel::Exception &e) { std::cout << e.what(); throw e; }
#else
	#define JFR_GLOBAL_TRY
	#define JFR_GLOBAL_CATCH
#endif

/** If the \a predicate is \c FALSE, throw \a ExceptionName with ID \a
 * id along with \a message. The constructor of the class \a
 * ExceptionName must have signature compatible with ExceptionName(id,
 * message, __FILE__, __LINE__). This is the case for the exception
 * class which is generated with a module, but may not be the case for
 * user defined exceptions.
 */
#define JFR_PRED_ERROR(predicate, ExceptionName, id, message)		\
{ \
  if (!(predicate)) {							\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw ExceptionName(id, s.str(), __FILE__, __LINE__);		\
  } \
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
{ \
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::RUN_TIME,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  } \
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
{ \
  if (!(predicate))							\
    {									\
      using jafar::kernel::JafarException;				\
      std::ostringstream s;						\
      s << message;							\
      throw JafarException(JafarException::IO_STREAM,			\
			   s.str(), _JFR_MODULE_, __FILE__, __LINE__);	\
  } \
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
{ \
  if (!(predicate))							\
    {									\
      using jafar::kernel::JafarException;				\
      std::ostringstream s;						\
      s << message;							\
      s << " (" << #predicate << ")";					\
      throw JafarException(JafarException::NUMERIC,			\
			   s.str(),					\
			   _JFR_MODULE_, __FILE__, __LINE__);		\
  } \
} 

/** If \a predicate is \c FALSE throw a jafar::kernel::InvalidParamException
 */
#define JFR_CHECK_PARAM(predicate, param)				\
{ \
  if (!(predicate)) {							\
    std::ostringstream s;						\
    s << "invalid parameter " << #param << "=" << param;		\
    s << " (" << #predicate << ")";					\
    jafar::kernel::throwInvalidParamException(param, s.str(),		\
					      _JFR_MODULE_,		\
					      __FILE__, __LINE__);	\
  } \
} 

#include "kernel/jafarDebug.hpp"
using jafar::debug::DebugStream;


/** When JFR_NDEBUG is defined, contract programming checks, debug
 * messages and trace information are disabled.
 */
#ifndef JFR_NDEBUG

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c ASSERT, along with \a message.
 */
#  define JFR_ASSERT(predicate, message)				\
{ \
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::ASSERT,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  } \
} 


/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c PRECONDITION, along with \a message.
 */
#  define JFR_PRECOND(predicate, message)				\
{ \
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::PRECONDITION,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  } \
} 
  

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c POSTCONDITION, along with \a message.
 */
#  define JFR_POSTCOND(predicate, message)				\
{ \
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::POSTCONDITION,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  } \
} 

/** If \a predicate is \c FALSE throw a jafar::kernel::JafarException
 * with ID \c INVARIANT, along with \a message.
 */
#  define JFR_INVARIANT(predicate, message)				\
{ \
  if (!(predicate)) {							\
    using jafar::kernel::JafarException;				\
    std::ostringstream s;						\
    s << message;							\
    s << " (" << #predicate << ")";					\
    throw JafarException(JafarException::INVARIANT,			\
			 s.str(),_JFR_MODULE_, __FILE__, __LINE__);	\
  } \
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


/** Tell the debug stream to start a new line of debug
 * \code
 *  JFR_DEBUG_BEGIN()
 *  JFR_DEBUG_SEND("The values of i are "
 *  for(int i = 0; i < 3; i ++)
 *    JFR_DEBUG_SEND( i << " "); 
 *  JFR_DEBUG_END()
 * 
 * output:
 *  D:pipo/test.cpp:55: The value of i are 0 1 2
 * \endcode
 */
#define JFR_DEBUG_BEGIN() \
{                          \
    DebugStream::setup(_JFR_MODULE_, DebugStream::Debug); \
    DebugStream::sendLocation(_JFR_MODULE_, __FILE__, __LINE__); \
}

/**
 * Use after a call to JFR_DEBUG_BEGIN() to send \a message to
 * the debug stream
 */
#define JFR_DEBUG_SEND(message) \
{                                \
    DebugStream::instance() << message; \
}

/**
 * Use when you want to close a line of debug
 */
#define JFR_DEBUG_END() \
{                        \
    DebugStream::instance() << jafar::debug::endl; \
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
    JFR_DEBUG_BEGIN()       \
    JFR_DEBUG_SEND(message)     \
    JFR_DEBUG_END()     \
  }

/**
 * Send \a message to the debug stream with level
 * DebugStream::Debug, if the condition \a test
 * is true. \c operator<< can be used to format the
 * message.
 */
#define JFR_DEBUG_COND(test, message) \
{ \
  if( (test) ) JFR_DEBUG( message); \
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
#if 0
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
#else
#  define JFR_TRACE_BEGIN

/** End a trace block. Append \a message to the trace.
 */
#  define JFR_TRACE_END(message)

#endif
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
#  define JFR_ASSERT(predicate, message) ((void)0)
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

#  define JFR_DEBUG_BEGIN()  ((void)0)
#  define JFR_DEBUG_SEND(message)  ((void)0)
#  define JFR_DEBUG_END()  ((void)0)

#  define JFR_DEBUG(message) ((void)0)
#  define JFR_DEBUG_COND(test, message) ((void)0)
#  define JFR_VDEBUG(message) ((void)0)
#  define JFR_VVDEBUG(message) ((void)0)
#  define JFR_TRACE(exception, message) ((void)0)
#  define JFR_TRACE_BEGIN ((void)0)
#  define JFR_TRACE_POINT ((void)0)
#  define JFR_TRACE_END(message) ((void)0)
#endif // JFR_NDEBUG

#endif // KERNEL_JAFAR_MACRO
