/* $Id$ */

#ifndef KERNEL_JAFAR_DEBUG_HPP
#define KERNEL_JAFAR_DEBUG_HPP

#include <iostream>
#include <sstream>
#include <string>

namespace jafar { 
  namespace kernel {


    /** Debug message. This class defines a set of static methods
     * which controls the debug messages. For ease of use in C++ a
     * macro JFR_DEBUG(message) is defined:
     *
     * \code
     *  JFR_DEBUG("The value of i is " << i); 
     *
     * output:
     *  D: jafarDebug.hpp:22: The value of i is 2
     * \endcode
     *
     * In tcl, you should use the wrapped version of JafarDebug::puts() :
     *
     * \code
     *  kernel::JafarDebug_puts "My message"
     *
     * output:
     *  W: My message
     * \endcode
     *
     * When \c JFR_NDEBUG is defined at compilation time, debugs messages
     * are suppressed.
     *
     * \warning
     * Because of (or thanks to !) the strange behavior of static
     * attributes and dynamic libraries, all the actions on the output
     * debug stream executed inside a tcl shell are done on a per
     * module basis (In fact there is a different "instance" of the
     * static output stream attribute for each loaded module).
     *
     * \ingroup kernel
     */
    class JafarDebug {

    public:

      /** Debug output stream pointer. It can be used directly or
       * through the macro jfrDbg() (recommended).
       */
      static std::ostream* dbg;

      /** sends \a message_ to the debug stream (use this method from tcl). 
       */
      static void puts(const std::string& message_);

      /// turns on debug to the default \c std::cerr.
      static void setOn();

      /// turns off debug (send it to /dev/null).
      static void setOff();

      /// set the debug output to the default \c std::cerr. (alias for setOn())
      static void setOutputDefault();

      /// set the debug output to the given \a file_.
      static void setOutputFile(const std::string& file_);

      /// set the warning output to the given \a stream_
      static void setOutputStream(std::ostream* stream_);

    private:

      /// flag to know wether or not to free the stream.
      static bool isDbgStreamAllocated;

      static const std::string _dev_null;

      /// The class can't be instanciated
      JafarDebug();
      ~JafarDebug();

    };


  } // namespace kernel
} // namespace jafar

#include "kernel/jafarMacro.hpp"

#endif // KERNEL_JAFAR_DEBUG_HPP
