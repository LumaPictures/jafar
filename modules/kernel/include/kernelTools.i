/* $Id$ */

/** tools for module kernel.
 *
 * \file kernelTools.hpp
 * \ingroup kernel
 */   

%{

#include <iostream>

// we could use better singleton here...
#include "boost/pool/detail/singleton.hpp"

#include "kernel/jafarDebug.hpp"

%}

%inline %{

namespace jafar {
  namespace kernel {

    jafar::debug::DebugStream& debug() {
      return boost::details::pool::singleton_default<jafar::debug::DebugStream>::instance();
    };

    void putsDbg(std::string const& message, 
		 jafar::debug::DebugStream::Level level_ = jafar::debug::DebugStream::Debug)
    {
      jafar::debug::DebugStream& dbg = 
	boost::details::pool::singleton_default<jafar::debug::DebugStream>::instance();
  
      dbg.setup("tcl", level_);
      dbg << "D:tcl macro: " << message << jafar::debug::endl;
    };

    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

  } // namespace kernel
} // namespace jafar

%}
