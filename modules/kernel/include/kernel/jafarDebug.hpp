/* $Id$ */

#ifndef KERNEL_JAFAR_DEBUG_HPP
#define KERNEL_JAFAR_DEBUG_HPP

#include <iostream>
#include <string>
#include <map>

// we could use better singleton here...
#include "boost/pool/detail/singleton.hpp"

namespace jafar { 
  namespace debug {


    /** Debug message stream. The default output stream is std::cerr.
     *
     * When \c JFR_NDEBUG is defined at compilation time, debugs messages
     * are suppressed.
     *
     *
     * \ingroup kernel
     */
    class DebugStream {

    public:

      /// Levels of the debug output.
      enum Level {Null             = 0, /**< nothing */
		  Trace            = 1, /**< trace message for exceptions */
		  Warning          = 2, /**< warning message */
		  Debug            = 3, /**< debug message, this is the default level */
		  VerboseDebug     = 4, /**< verbose debug */
		  VeryVerboseDebug = 5  /**< very verbose debug */
      };

      std::ostream& stream() { return *debugStream; }

      /// set the global debug level
      void setLevel(Level level_) {level = level_;}

      /// set the level for a given module
      void setLevel(std::string const& module_, Level level_) {modulesLevel[module_]=level_;}

      /// alias for setLevel(module_, DebugStream::Debug)
      void setModuleOn(std::string const& module_) { setLevel(module_, Debug); }

      /// alias for setLevel(module_, DebugStream::Warning)
      void setModuleOff(std::string const& module_) { setLevel(module_, Warning); }

      // void setOutputFile(std::string const& of_);

      void setStream(std::ostream& os_) { debugStream = &os_; }

      void on() { p_on = true; }
      void off() { p_on = false; }

      bool isDebugging() const { return debugging; }

#ifndef SWIG
      void setup(std::string const& module_, Level level_);
#endif // SWIG

    private: 

      /** Debug output stream pointer.
       */
      std::ostream* debugStream;

      bool p_on;

      std::map<std::string, Level> modulesLevel;

      Level level;

      bool debugging;

      DebugStream();

      friend class boost::details::pool::singleton_default<DebugStream>;

    }; // class DebugStream

#ifndef SWIG

    namespace details
    {
        typedef DebugStream& (*stream_function)(DebugStream& stream);
        typedef std::ios_base&(*iosbase_function)(std::ios_base&);
    }

    inline DebugStream& operator << (DebugStream& debugStream, details::stream_function function)
    { 
        if (debugStream.isDebugging())
	  return function(debugStream);
        
	return debugStream;
    }

    inline DebugStream& operator << (DebugStream& debugStream, details::iosbase_function function)
    { 
        if (debugStream.isDebugging())
	  function(debugStream.stream()); 
        
        return debugStream;
    }

    template<typename T>
    inline DebugStream& operator << (DebugStream& debugStream, T const& value)
    {
      if (debugStream.isDebugging())
        debugStream.stream() << value;

      return debugStream;
    }

    inline DebugStream& endl(DebugStream& debugStream)
    {
      if (debugStream.isDebugging())
	debugStream.stream() << std::endl;

      return debugStream;
    }

#endif // SWIG
    
  } // namespace debug
} // namespace jafar


#include "kernel/jafarMacro.hpp"

#endif // KERNEL_JAFAR_DEBUG_HPP
