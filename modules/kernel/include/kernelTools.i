/* $Id$ */

/** Tools for module kernel.
 *
 * \file kernelTools.i
 * \ingroup kernel
 */   

%{

#include <iostream>

#include "kernel/jafarDebug.hpp"
#include "kernel/timingTools.hpp"

%}

%inline %{

namespace jafar {
  namespace kernel {

    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

    /// Send \a message to the debug stream.
    void sendDebug(std::string const& message, 
		   jafar::debug::DebugStream::Level level_ = jafar::debug::DebugStream::Debug)
    {
      jafar::debug::DebugStream::setup("shell", level_);
      jafar::debug::DebugStream::instance() << "shell: " << message << jafar::debug::endl;
    };

    /// tic ala matlab, resets the global chrono
    void tic() {
      jafar::kernel::detail::TicTocChrono::instance().chrono.reset();
    };

    /// toc ala matlab, returns elapsed time in ms since the last tic() call.
    long toc() {
      return jafar::kernel::detail::TicTocChrono::instance().chrono.elapsed();
    };

  } // namespace kernel
} // namespace jafar

%}
