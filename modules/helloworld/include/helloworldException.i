/* $Id$ */

/** swig/tcl interface file for module helloworld exceptions.
 *
 * \file helloworldException.i
 */  

%{
  #include <iostream>
  #include <exception>
  #include <kernel/jafarException.hpp>
%}


#if defined(SWIGTCL)
/* Tcl exceptions handler.
 *
 * You can customize this handler and add catch blocks to handle your
 * own exceptions. Be aware that order in catch clauses is meaningfull
 * because of JafarException heritage tree.
 */
%exception {
  try {
    $action // Gets substituted by actual function call
  }
  catch(jafar::helloworld::HelloworldFormatException& e) {
    std::cout << e;
    std::cout << "For example:" << std::endl;
    std::cout << "helloworld::HelloWorld_setString $hello \"Hello world\"" << std::endl;
    SWIG_fail;
  }
  catch (jafar::kernel::Exception& e) {
    std::cout << e;
    SWIG_fail;
  }
  catch (std::exception& e) {
    std::cout << "** std::exception: **" << std::endl;
    std::cout << e.what() << std::endl;
    SWIG_fail;
  }
  catch(...) {
    std::cout << "** unknown exception **" << std::endl;
    SWIG_fail;
  }
}
#elif defined(SWIGRUBY)

%import "kernel/jafarException.hpp"
%include "helloworld/helloworldException.hpp"

JAFAR_EXCEPTION_SUPPORT
JAFAR_RUBY_EXCEPTION(Helloworld, HelloworldException);
%exception {
    try {
        $action
    }
    JAFAR_CATCH_MODULE_EXCEPTION(helloworld, HelloworldException)
    JAFAR_CATCH_GENERIC
}

#endif

