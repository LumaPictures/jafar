/* $Id$ */

/** Tools for module slammm.
 *
 * \file slammmTools.i
 * \ingroup slammm
 */   

%{

#include <sstream>

%}

%inline %{

namespace jafar {
  namespace slammm {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

  } // namespace slammm
} // namespace jafar

%}


