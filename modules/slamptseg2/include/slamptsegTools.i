/* $Id$ */

/** Tools for module slamptseg.
 *
 * \file slamptsegTools.i
 * \ingroup slamptseg
 */   

%{

#include <sstream>

%}

%inline %{

namespace jafar {
  namespace slamptseg {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

  } // namespace slamptseg
} // namespace jafar

%}


