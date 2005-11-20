/* $Id$ */

/** swig/tcl interface file for module _jfr_module_.
 *
 * \file _jfr_module_.i
 * \ingroup _jfr_module_
 */      

%module _jfr_module_

%{
  /* 
   * headers necessary to compile the wrapper
   *
   * example:
   *
   * #include <cmath>
   * #include "toto.hpp"
   *
   */

// to wrap std::string
// #include <string> 

// using namespace jafar::_jfr_module_;

%}

%include "jafarConfig.h"
%include "jafar.i"
%include "_jfr_module_Exception.i"

/*
 * headers to be wrapped goes here
 *
 * example:
 *
 * %include "toto.hpp"
 */

// %include "_jfr_module_Tools.i"
// instantiate some print functions
// replace "Type" with appropriate class name
// %template(print) jafar::_jfr_module_::print<jafar::_jfr_module_::Type>;

