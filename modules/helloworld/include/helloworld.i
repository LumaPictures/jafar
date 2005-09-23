/* $Id$ */

/** swig/tcl interface file for module helloworld.
 *
 * \file helloworld.i
 * \ingroup helloworld
 */  

%module helloworld

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

#include <string>

// uncomment to control debug output from tcl
#include "kernel/jafarDebug.hpp"

#include "helloworld/helloWorld.hpp"

%}

%include "std_string.i"

%include "helloworldException.i"

/*
 * headers to be wrapped goes here
 *
 * example:
 *
 * %include "toto.hpp"
 *
 */

// uncomment to control debug output from tcl
%include "kernel/jafarDebug.hpp"

%include "helloworldTools.i"
%template(print) jafar::helloworld::print<jafar::helloworld::HelloWorld>;

%include "helloworld/helloWorld.hpp"

