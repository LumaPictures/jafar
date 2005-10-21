/* $Id$ */

/** Tools for module kernel.
 *
 * \file kernelTools.i
 * \ingroup kernel
 */   

%{

#include <iostream>

#include "kernel/jafarDebug.hpp"

%}

%inline %{

namespace jafar {
  namespace kernel {

    /// Send \a message to the debug stream.
    void sendDebug(std::string const& message, 
		   jafar::debug::DebugStream::Level level_ = jafar::debug::DebugStream::Debug)
    {
      jafar::debug::DebugStream::setup("shell", level_);
      jafar::debug::DebugStream::instance() << "shell: " << message << jafar::debug::endl;
    };

    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

  } // namespace kernel
} // namespace jafar

%}
