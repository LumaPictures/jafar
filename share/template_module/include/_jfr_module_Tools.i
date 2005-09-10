/* $Id$ */

/** tcl tools for module _jfr_module_.
 *
 * \file _jfr_module_Tools.i
 * \ingroup _jfr_module_
 */   

%{

#include <iostream>

%}

%inline %{

namespace jafar {
  namespace _jfr_module_ {

    /// template print function which calls the output operator<<.
    template<class A>
    void print(const A& a_) {
      std::cout << a_ << std::endl;
    };

  } // namespace _jfr_module_
} // namespace jafar

%}

