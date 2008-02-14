/* $Id$ */

/** Tools for module lines.
 *
 * \file linesTools.i
 * \ingroup lines
 */   

%{

#include <sstream>

%}

%inline %{

namespace jafar {
  namespace lines {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

  } // namespace lines
} // namespace jafar

%}


