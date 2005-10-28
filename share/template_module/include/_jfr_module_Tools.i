/* $Id$ */

/** Tools for module _jfr_module_.
 *
 * \file _jfr_module_Tools.i
 * \ingroup _jfr_module_
 */   

%{

#include <sstream>

%}

%inline %{

namespace jafar {
  namespace _jfr_module_ {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

  } // namespace _jfr_module_
} // namespace jafar

%}

