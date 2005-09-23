/* $Id$ */

/** tcl tools for module helloworld.
 *
 * \file helloworldTools.i
 * \ingroup helloworld
 */   

%{

#include <iostream>

%}

%inline %{

namespace jafar {
  namespace helloworld {
    
    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

  } // namespace helloworld
} // namespace jafar

%}


