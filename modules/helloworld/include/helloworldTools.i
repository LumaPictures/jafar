/* $Id$ */

/** tcl tools for module helloworld.
 *
 * \file helloworldTools.i
 * \ingroup helloworld
 */   

%{

#include <sstream>

%}

%inline %{

namespace jafar {
  namespace helloworld {
    
    /// template print function which calls the output operator<<.
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

    class A {
      
    public:
      
      int pipoA() const {return 1;}

    };

    class B : public A {

    public:

      int pipoB() {return 2;}

    };

  } // namespace helloworld
} // namespace jafar

%}


