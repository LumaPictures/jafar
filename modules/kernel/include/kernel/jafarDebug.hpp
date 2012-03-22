/* $Id$ */

#ifndef KERNEL_JAFAR_DEBUG_HPP
#define KERNEL_JAFAR_DEBUG_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <list>
#include <vector>

// we could use a better singleton here...
#include <boost/serialization/singleton.hpp>

namespace jafar { 
  namespace debug {

#ifndef SWIG
    
    class DebugStream;
    // special functions needed for the stream definition
    namespace details
    {
        typedef DebugStream& (*stream_function)(DebugStream& stream);
        typedef std::ios_base&(*iosbase_function)(std::ios_base&);
    }

#endif // SWIG


    /** Debug message stream. The default output stream is std::cerr.
     * In order to send messages to the debugStream, you should use
     * macros JFR_WARNING, JFR_DEBUG, JFR_VDEBUG, JFR_VVDEBUG defined
     * in jafarMacro.hpp.
     *
     * \ingroup kernel
     */
    class DebugStream {

    public:

      /// Levels of the debug output.
      enum Level {Off              = 0, /**< nothing */
		  Trace            = 1, /**< trace message for non-jafar exceptions */
		  Warning          = 2, /**< warning message */
		  Debug            = 3, /**< debug message, this is the default level */
		  VerboseDebug     = 4, /**< verbose debug */
		  VeryVerboseDebug = 5  /**< very verbose debug */
      };

      /** Set the default debug level. This level is used when no
       * level is specified for a given module.
       */
      static void setDefaultLevel(Level level_) {
	instance().defaultLevel = level_;
      }

      /// set the level for a given module
      static void setLevel(std::string const& module_, Level level_) {
	instance().modulesLevel[module_]=level_;
      }

      /// shortcut for setLevel(module_, DebugStream::Off)
      static void moduleOff(std::string const& module_) { 
	setLevel(module_, DebugStream::Off); 
      }

      /// shortcut for setLevel(module_, DebugStream::Warning)
      static void moduleWarning(std::string const& module_) { 
	setLevel(module_, DebugStream::Warning); 
      }

      /// shortcut for setLevel(module_, DebugStream::Debug)
      static void moduleDebug(std::string const& module_) { 
	setLevel(module_, DebugStream::Debug); 
      }

      /// shortcut for setLevel(module_, DebugStream::VerboseDebug)
      static void moduleVerboseDebug(std::string const& module_) { 
	setLevel(module_, DebugStream::VerboseDebug); 
      }

      /// shortcut for setLevel(module_, DebugStream::VeryVerboseDebug)
      static void moduleVeryVerboseDebug(std::string const& module_) {
	setLevel(module_, DebugStream::VeryVerboseDebug); 
      }

      /// unset debug level configuration for \a module_
      static void unsetModule(std::string const& module_) {
	instance().modulesLevel.erase(module_);
      }

      /// send the debug stream to \a filename_.
      static void setOutputFile(std::string const& filename_);

      /// send the debug stream to the default stream (std::cerr).
      static void setDefaultStream();

#ifndef SWIG // hide these methods in the interactive interface

      /// send the debug stream to the given stream \a os_.
      static void setStream(std::ostream& os_) { 
	instance().debugStream = &os_; 
      }

      /** Call this method before sending any data to the stream.
       * \warning usual developpers do not use it directly.
       */
      static void setup(std::string const& module_, Level level_);

      /** Send the given location to the debug stream. Format of this
       * location is controlled by the flag JFR_DEBUG_FULL_PATH.
       * \warning usual developpers do not use it directly.
       */
      static void sendLocation(std::string const& module_, char const* file_, int line_);

      /** Returns the unique instance of the debug stream.
       * \warning usual developpers do not use it directly.
       */
      static DebugStream& instance() {
	// we could use better singleton here...
	return boost::serialization::singleton<DebugStream>::get_mutable_instance(); 

      }

#endif // SWIG

    private: 

      ///Debug output stream pointer.
      std::ostream* debugStream;

      /// file stream used when sending the debug to a file.
      std::ofstream fileStream;
      bool fsOpen;

      std::map<std::string, Level> modulesLevel;

      /// level used when module level is not set.
      Level defaultLevel;

      bool debugging;

      DebugStream();
      ~DebugStream();

      /// @return the state of the stream.
      bool isDebugging() const { return debugging; }

      /// @return the underlying std::ostream.
      static std::ostream& stream() { return *(instance().debugStream); }

      /*
	   * The documented method
	   * http://www.boost.org/doc/libs/1_49_0/libs/serialization/doc/singleton.html
	   * does not seem to work. Use the following friend hack to make it work,
	   * but it is not really better than previous, as it depends on something
	   * which can break at any instant
	   */
	  friend struct boost::serialization::detail::singleton_wrapper<DebugStream>;

      friend DebugStream& operator << (DebugStream& debugStream, 
				       details::stream_function function);
      friend DebugStream& operator << (DebugStream& debugStream, 
				       details::iosbase_function function);
      friend DebugStream& endl(DebugStream& debugStream);

      template<typename T>
      friend DebugStream& operator << (DebugStream& debugStream, 
				       T const& value);
    }; // class DebugStream


#ifndef SWIG

    // special functions needed for the stream definition

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

    template< typename _Type_ >
    std::ostream& operator<<(std::ostream& s, const std::list<_Type_*>& gr)
    {
      s << " ( ";
      for( typename std::list<_Type_*>::const_iterator it = gr.begin(); it != gr.end(); ++it )
      {
        if( it != gr.begin() ) s << " , ";
        s << **it;
      }
      s << " ) ";
      return s;
    }

    template< typename _Type_ >
    std::ostream& operator<<(std::ostream& s, const std::list<_Type_>& gr)
    {
      s << " ( ";
      for( typename std::list<_Type_>::const_iterator it = gr.begin(); it != gr.end(); ++it )
      {
        if( it != gr.begin() ) s << " , ";
        s << *it;
      }
      s << " ) ";
      return s;
    }
    
    template< typename _Type_ >
    std::ostream& operator<<(std::ostream& s, const std::vector<_Type_*>& gr)
    {
      s << " ( ";
      for( typename std::vector<_Type_*>::const_iterator it = gr.begin(); it != gr.end(); ++it )
      {
        if( it != gr.begin() ) s << " , ";
        s << **it;
      }
      s << " ) ";
      return s;
    }
    
    template< typename _Type_ >
    std::ostream& operator<<(std::ostream& s, const std::vector<_Type_>& gr)
    {
      s << " ( ";
      for( typename std::vector<_Type_>::const_iterator it = gr.begin(); it != gr.end(); ++it )
      {
        if( it != gr.begin() ) s << " , ";
        s << *it;
      }
      s << " ) ";
      return s;
    }

#endif // SWIG
    
  } // namespace debug
} // namespace jafar


#include "kernel/jafarMacro.hpp"

#endif // KERNEL_JAFAR_DEBUG_HPP
