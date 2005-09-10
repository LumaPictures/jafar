/* $Id$ */

/** tools for module kernel.
 *
 * \file kernelTools.hpp
 * \ingroup kernel
 */   

%{

#include <iostream>

%}

%inline %{

namespace jafar {
  namespace kernel {

    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

  } // namespace kernel
} // namespace jafar

%}
