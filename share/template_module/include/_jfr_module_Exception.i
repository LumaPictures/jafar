/* $Id:$ */

/** swig/tcl interface file for module _jfr_module_ exceptions.
 *
 * \file _jfr_module_Exception.i
 * \ingroup _jfr_module_
 */      

%{
  #include <iostream>
  #include <exception>

  #include "kernel/jafarException.hpp"
  #include "_jfr_module_/_jfr_module_Exception.hpp"
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
%include "_jfr_module_/_jfr_module_Exception.hpp"

JAFAR_EXCEPTION_SUPPORT
JAFAR_RUBY_EXCEPTION(_jfr_Module_, _jfr_Module_Exception);
%exception {
    try {
        $action
    }
    JAFAR_CATCH_MODULE_EXCEPTION(_jfr_module_, _jfr_Module_Exception)
    JAFAR_CATCH_GENERIC
}

#endif

